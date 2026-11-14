"""출력 준비: 부품을 뱀부랩 스튜디오로 잘라, 판마다 바로 뽑을 3MF를 만든다 (docs/decisions/0035-slicer-automation.md).

출력 설정은 params/print.toml 하나에서 읽는다. 사람이 고르는 출력 설정은 없다.

    uv run python -m horilab_design.slicing plan --name first_joint wrist.wrist_body:2 wrist.out_disc gripper
    uv run python -m horilab_design.slicing reference <화면에서 잘라 저장한 3MF>

부품은 `모듈`(그 모듈의 출력 부품 모두), `모듈.부품`, 뒤에 `:n`(n벌)으로 준다. 공차 시험편은 색을 붙인다
(`coupons.fit3_6806_bores@white`).

plan:
  1. 스튜디오 앱 판, 프리셋 묶음 판, 펼친 프리셋 내용의 지문을 값 파일과 견준다. 다르면 자르지 않는다.
  2. 부품을 프리셋과 색으로 묶고, 묶음마다 한 번 자동 배치해 판 나눔을 알아낸 뒤 판마다 다시 자른다.
  3. 판마다 3MF를 검사한다: 핵심 설정, 판 종류와 G-code의 바닥 온도, 설정 전체의 지문, 서포트, 판 밖,
     들어간 부품과 개수, 부품이 눕지 않았는지(Z축으로만 돌렸는지), 스튜디오 로그(무해하다고 적은 줄 말고는 막는다).
  4. 모든 판이 통과해야 build/print/<이름>/에 판마다 3MF와 판 그림, plan.json을 낸다. 하나라도 걸리면 그 폴더를
     지우고(옛 파일을 뽑지 않게) 아무것도 내지 않는다.

reference (처음 한 번, 가정 B): 화면에서 늘 쓰던 설정으로 잘라 저장한 3MF를, 같은 모델 자리 그대로 명령 모드로 다시
자른다. 설정 칸, 예상 시간과 무게, G-code의 움직임 줄을 견주고, 같으면 설정 전체의 지문을 알려 준다 (값 파일에 적는다).

스튜디오의 시스템 프리셋은 상위 파일의 값을 물려받는다. 명령 모드에 그대로 주면 멈춰서(2.4.0.70), 상속을 펼친
파일을 만들어 준다. 공식 사용법도 "값을 다 채운 설정"을 요구한다.
"""

from __future__ import annotations

import argparse
import functools
import hashlib
import importlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tomllib
import xml.etree.ElementTree as ET
import zipfile
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Any

from build123d import Axis, Compound, GeomType, Pos, export_stl

from horilab_design.printcheck import overhangs
from horilab_design.values import REPO_ROOT

PRINT_FILE = REPO_ROOT / "params" / "print.toml"
OUT_DIR = REPO_ROOT / "build" / "print"
STUDIO = Path(os.environ.get("BAMBU_STUDIO", r"C:\Program Files\Bambu Studio\bambu-studio.exe"))
TAG = re.compile(r"\[(error|warning)\]\s*(.*?)\s*$")
SPEC = re.compile(
    r"^(?P<module>[a-z0-9_]+)(?:\.(?P<part>[a-z0-9_]+))?(?::(?P<count>[1-9][0-9]*))?(?:@(?P<color>[a-z]+))?$"
)
# 판 종류 → 첫 층 바닥 온도 칸
BED_TEMP_KEY = {
    "Textured PEI Plate": "textured_plate_temp_initial_layer",
    "Cool Plate": "cool_plate_temp_initial_layer",
    "Engineering Plate": "eng_plate_temp_initial_layer",
    "High Temp Plate": "hot_plate_temp_initial_layer",
}
SLENDER = 3.0  # 높이가 √(바닥 넓이)의 이만큼을 넘으면 넘어질 수 있다고 표시한다 (참고값, 어림)
COLOR_KO = {"white": "흰색", "black": "검정"}

Vec = tuple[float, float, float]


# ---- 값 파일 ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PrintConfig:
    studio_version: str
    profiles_version: str
    machine: str
    filament: str
    bed_type: str
    max_height_mm: float
    overnight_hours: float
    process: dict[str, str]
    colors: dict[str, str]
    presets_sha: dict[str, str]
    settings_sha: dict[str, str]
    sha_ignore: tuple[str, ...]
    key_settings: dict[str, dict[str, str]]
    harmless: frozenset[tuple[str, str]]

    def keys_for(self, preset: str) -> dict[str, str]:
        return {**self.key_settings["common"], **self.key_settings[preset]}


def load_config(path: Path = PRINT_FILE) -> PrintConfig:
    with path.open("rb") as f:
        d = tomllib.load(f)
    sha = dict(d["settings_sha"])
    ignore = tuple(str(k) for k in sha.pop("ignore", []))
    return PrintConfig(
        studio_version=str(d["studio"]["version"]),
        profiles_version=str(d["studio"]["profiles_version"]),
        machine=str(d["printer"]["machine"]),
        filament=str(d["printer"]["filament"]),
        bed_type=str(d["printer"]["bed_type"]),
        max_height_mm=float(d["printer"]["max_height_mm"]),
        overnight_hours=float(d["printer"]["overnight_hours"]),
        process={str(k): str(v) for k, v in d["process"].items()},
        colors={str(k): str(v) for k, v in d["colors"].items()},
        presets_sha={str(k): str(v) for k, v in d["presets_sha"].items()},
        settings_sha={str(k): str(v) for k, v in sha.items()},
        sha_ignore=ignore,
        key_settings={
            str(group): {str(k): str(v) for k, v in values.items()}
            for group, values in d["key_settings"].items()
        },
        harmless=frozenset((str(h["level"]), str(h["text"])) for h in d.get("harmless", [])),
    )


# ---- 스튜디오와 프리셋 ---------------------------------------------------------------------------------


def studio_version(exe: Path = STUDIO) -> str:
    """스튜디오 앱 판 (--help의 첫 줄 "BambuStudio-02.04.00.70:")."""
    run = subprocess.run(
        [str(exe), "--help"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=120,
    )
    found = re.search(r"BambuStudio-(\d+(?:\.\d+){3})", run.stdout + run.stderr)
    if not found:
        raise RuntimeError("스튜디오 판을 읽지 못했다 (--help 출력에 판이 없다)")
    return found.group(1)


def profiles_root(exe: Path = STUDIO) -> Path:
    """화면이 쓰는 시스템 프리셋 폴더: 사용자 폴더에 받은 것이 있으면 그것, 없으면 설치 폴더."""
    appdata = os.environ.get("APPDATA")
    if appdata:
        user = Path(appdata) / "BambuStudio" / "system" / "BBL"
        if user.is_dir():
            return user
    return exe.parent / "resources" / "profiles" / "BBL"


def profiles_version(root: Path) -> str:
    return str(json.loads((root.parent / "BBL.json").read_text(encoding="utf-8"))["version"])


def digest(data: object, length: int = 12) -> str:
    """열쇠를 정렬한 JSON의 sha256 앞자리."""
    text = json.dumps(data, sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:length]


@functools.cache
def _index(root: Path, kind: str) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for path in sorted((root / kind).rglob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if isinstance(data, dict) and "name" in data:
            out.setdefault(str(data["name"]), data)  # pyright: ignore[reportUnknownArgumentType]
    return out


def flatten(root: Path, kind: str, name: str) -> dict[str, Any]:
    """시스템 프리셋 하나의 상속을 펼친다. root는 .../profiles/BBL, kind는 machine, process, filament."""
    index = _index(root, kind)
    chain: list[dict[str, Any]] = []
    current = name
    while current:
        if current not in index:
            raise KeyError(f"{kind}/{current} 프리셋이 없다")
        chain.append(index[current])
        current = str(index[current].get("inherits", ""))
        if len(chain) > 20:
            raise RuntimeError(f"{kind}/{name}: 상속이 끝나지 않는다")
    merged: dict[str, Any] = {}
    for data in reversed(chain):
        merged.update(data)
    merged.update({"name": name, "inherits": "", "from": "system"})
    merged.pop("instantiation", None)
    return merged


def flat_presets(cfg: PrintConfig, root: Path) -> dict[str, dict[str, Any]]:
    return {
        "machine": flatten(root, "machine", cfg.machine),
        **{key: flatten(root, "process", name) for key, name in cfg.process.items()},
        "filament": flatten(root, "filament", cfg.filament),
    }


def check_studio(
    cfg: PrintConfig, exe: Path = STUDIO
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    """앱 판, 프리셋 묶음 판, 펼친 프리셋 내용이 값 파일과 같은지. (펼친 프리셋, 문제)."""
    if not exe.exists():
        return {}, [
            f"뱀부랩 스튜디오가 없다: {exe} (다른 곳에 깔았으면 BAMBU_STUDIO로 경로를 준다)"
        ]
    problems: list[str] = []
    version = studio_version(exe)
    if version != cfg.studio_version:
        problems.append(
            f"스튜디오 앱 판이 {version}이다 (값 파일 {cfg.studio_version}). 공차를 잰 판이 아니라 자르지 않는다"
        )
    root = profiles_root(exe)
    pv = profiles_version(root)
    if pv != cfg.profiles_version:
        problems.append(f"프리셋 묶음 판이 {pv}이다 (값 파일 {cfg.profiles_version}, {root})")
    flat = flat_presets(cfg, root)
    for key, data in flat.items():
        if digest(data) != cfg.presets_sha.get(key):
            problems.append(
                f"프리셋 {key}의 내용이 바뀌었다 (지문 {digest(data)}, 값 파일 {cfg.presets_sha.get(key)})"
            )
    return flat, problems


def write_presets(cfg: PrintConfig, flat: dict[str, dict[str, Any]], out: Path) -> dict[str, Path]:
    """펼친 프리셋을 쓴다. 열쇠는 machine, 공정 이름들, filament_<색>."""
    out.mkdir(parents=True, exist_ok=True)
    data = {key: flat[key] for key in ("machine", *cfg.process)}
    for color, hex_code in cfg.colors.items():
        data[f"filament_{color}"] = {**flat["filament"], "filament_colour": [hex_code]}
    paths: dict[str, Path] = {}
    for key, value in data.items():
        paths[key] = out / f"{key}.json"
        paths[key].write_text(json.dumps(value, ensure_ascii=False, indent=1), encoding="utf-8")
    return paths


def run_studio(
    cfg: PrintConfig,
    presets: dict[str, Path],
    preset: str,
    color: str,
    inputs: Sequence[Path],
    work: Path,
    name: str,
    arrange: bool = True,
    exe: Path = STUDIO,
) -> tuple[Path, int, str]:
    """명령 모드로 자른다. 출력 방향은 바꾸지 않는다(자동 방향 끔). (3MF, 끝난 값, 로그)."""
    work.mkdir(parents=True, exist_ok=True)
    args = [
        str(exe),
        "--load-settings",
        f"{presets['machine']};{presets[preset]}",
        "--load-filaments",
        str(presets[f"filament_{color}"]),
        "--curr-bed-type",
        cfg.bed_type,
        "--orient",
        "0",
        "--slice",
        "0",
        "--outputdir",
        str(work),
        "--export-3mf",
        f"{name}.3mf",
    ]
    if arrange:
        args += ["--arrange", "1", "--allow-rotations"]
    args += [str(p) for p in inputs]
    run = subprocess.run(
        args, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=1800
    )
    return work / f"{name}.3mf", run.returncode, run.stdout + run.stderr


# ---- 3MF 읽기 -----------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PlateInfo:
    index: int
    seconds: float
    grams: float
    support: bool
    outside: bool
    objects: tuple[str, ...]


@dataclass(frozen=True)
class Sliced:
    plates: tuple[PlateInfo, ...]
    settings: dict[str, Any]
    gcode: str
    png: bytes | None
    transforms: tuple[tuple[float, ...], ...]


def read_3mf(path: Path) -> Sliced:
    """잘린 3MF: 판마다의 결과, 들어간 설정, 1번 판의 G-code와 그림, 놓인 모양의 변환 행렬."""
    with zipfile.ZipFile(path) as z:
        names = set(z.namelist())
        plates: list[PlateInfo] = []
        if "Metadata/slice_info.config" in names:
            for p in ET.fromstring(z.read("Metadata/slice_info.config")).iter("plate"):
                meta = {m.get("key", ""): m.get("value", "") for m in p.findall("metadata")}
                plates.append(
                    PlateInfo(
                        index=int(meta.get("index") or 0),
                        seconds=float(meta.get("prediction") or 0),
                        grams=float(meta.get("weight") or 0),
                        support=meta.get("support_used") == "true",
                        outside=meta.get("outside") == "true",
                        objects=tuple(o.get("name", "") for o in p.findall("object")),
                    )
                )
        settings: dict[str, Any] = json.loads(z.read("Metadata/project_settings.config"))
        gcode = (
            z.read("Metadata/plate_1.gcode").decode("utf-8", "replace")
            if "Metadata/plate_1.gcode" in names
            else ""
        )
        png = z.read("Metadata/plate_1.png") if "Metadata/plate_1.png" in names else None
        model = (
            z.read("3D/3dmodel.model").decode("utf-8", "replace")
            if "3D/3dmodel.model" in names
            else ""
        )
    transforms = tuple(
        tuple(float(v) for v in t.split()) for t in re.findall(r'transform="([^"]+)"', model)
    )
    return Sliced(tuple(plates), settings, gcode, png, transforms)


# ---- 검사 ---------------------------------------------------------------------------------------------


def _norm(value: object) -> str:
    text = str(value).strip()
    percent = text.endswith("%")
    try:
        return f"{float(text.rstrip('%')):g}" + ("%" if percent else "")
    except ValueError:
        return text


def _first(value: object) -> object:
    if isinstance(value, list) and value:
        return value[0]  # pyright: ignore[reportUnknownVariableType]
    return value


def setting_problems(settings: dict[str, Any], expected: dict[str, str]) -> list[str]:
    """3MF의 설정이 핵심 설정과 다른 칸. 값이 여럿인 칸은 첫 값(일반 노즐)을 본다."""
    out: list[str] = []
    for key, want in expected.items():
        got = settings.get(key)
        if got is None or _norm(_first(got)) != _norm(want):
            out.append(f"설정 {key}: 값 파일 {want}, 3MF {got}")
    return out


def settings_sha(settings: dict[str, Any], ignore: Sequence[str]) -> str:
    """3MF에 들어간 설정 전체의 지문 (빼는 칸 말고)."""
    return digest({k: v for k, v in settings.items() if k not in ignore}, 16)


def fingerprint_problems(cfg: PrintConfig, preset: str, settings: dict[str, Any]) -> list[str]:
    want = cfg.settings_sha.get(preset, "")
    got = settings_sha(settings, cfg.sha_ignore)
    if not want:
        return [
            f"{preset}: 설정 전체의 기준 지문이 없다. 화면에서 자른 기준과 먼저 견준다 (reference, 가정 B)"
        ]
    if got != want:
        return [f"{preset}: 설정 전체의 지문 {got}이 기준 {want}과 다르다"]
    return []


def bed_problems(cfg: PrintConfig, sliced: Sliced) -> list[str]:
    """판 종류가 값 파일과 같고, G-code가 그 판의 첫 층 바닥 온도로 데우는지."""
    out: list[str] = []
    bed = sliced.settings.get("curr_bed_type")
    if bed != cfg.bed_type:
        out.append(f"판 종류가 {bed}이다 (값 파일 {cfg.bed_type})")
    key = BED_TEMP_KEY.get(cfg.bed_type)
    want = _norm(_first(sliced.settings.get(key))) if key else None
    for code in ("M140", "M190"):
        found = re.search(rf"^{code} S(\d+)", sliced.gcode, re.M)
        if want is None or found is None or _norm(found.group(1)) != want:
            out.append(
                f"G-code의 첫 {code}(바닥 온도)가 {found.group(1) if found else '없음'}이다 (판에 맞는 값 {want})"
            )
    return out


def orientation_problems(transforms: Sequence[Sequence[float]]) -> list[str]:
    """부품을 Z축으로만 돌렸는지 (출력 방향이 그대로인지). 3MF 변환은 3 × 4 행렬을 행으로 늘어놓은 것이다."""
    for t in transforms:
        if len(t) != 12:
            return [f"변환 행렬을 읽지 못했다 ({len(t)}칸)"]
        if (
            abs(t[2]) > 1e-6
            or abs(t[5]) > 1e-6
            or abs(t[6]) > 1e-6
            or abs(t[7]) > 1e-6
            or abs(t[8] - 1) > 1e-6
        ):
            return ["부품이 Z축 밖으로 돌아갔다 (출력 방향이 바뀌었다)"]
    return []


def log_problems(cfg: PrintConfig, returncode: int, log: str) -> list[str]:
    """스튜디오 로그: 무해하다고 적은 줄(문구 전체가 같을 때)만 빼고, 오류와 처음 보는 경고는 막는다."""
    out = [f"스튜디오가 {returncode}로 끝났다"] if returncode != 0 else []
    for line in log.splitlines():
        found = TAG.search(line)
        if not found:
            continue
        level, text = found.group(1), found.group(2)
        if (level, text) in cfg.harmless:
            continue
        out.append(
            ("스튜디오 오류: " if level == "error" else "검토 필요 (처음 보는 경고): ") + text
        )
    return out


# ---- 부품 ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Item:
    """뽑을 부품 하나와 개수."""

    module: str
    name: str  # STL 파일 이름 (확장자 없이)
    label: str  # 한글 이름
    shape: Compound
    up: Vec  # 뽑을 때 위를 보는 방향
    color: str
    preset: str
    count: int


@dataclass(frozen=True)
class Geometry:
    height: float
    base_area: float  # 바닥에 닿는 넓이 [mm²]
    overhang: float  # 바닥에 닿지 않고 아래를 보는 면 [mm²] (printcheck)
    overhang_largest: float
    slender: bool  # 높이에 비해 바닥이 좁다 (참고값)


def _module_items(module: str, part: str | None, color: str | None, cfg: PrintConfig) -> list[Item]:
    if module == "coupons":
        from horilab_design import coupons

        if part is None or part not in coupons.COUPONS:
            raise ValueError(f"시험편 이름을 모른다: {part} ({', '.join(coupons.COUPONS)})")
        if color not in cfg.colors:
            raise ValueError(
                f"시험편은 색을 붙인다: coupons.{part}@white ({', '.join(cfg.colors)})"
            )
        return [
            Item(
                "coupons",
                part,
                f"시험편 {part}",
                coupons.COUPONS[part](),
                (0.0, 0.0, 1.0),
                color,
                "strength",
                1,
            )
        ]
    if color is not None:
        raise ValueError(f"색은 시험편에만 붙인다 ({module}의 부품 색은 코드에 있다)")
    mod = importlib.import_module(f"horilab_design.parts.{module}")
    printed: dict[str, tuple[Any, ...]] = mod.printed()
    labels: dict[str, str] = getattr(mod, "LABELS", {})
    out: list[Item] = []
    for name in [part] if part else list(printed):
        if name not in printed:
            raise ValueError(f"{module}에 {name}이 없다 ({', '.join(printed)})")
        entry = printed[name]
        shape, up, col, preset = entry[:4]
        if name not in labels:
            raise ValueError(f"{module}.LABELS에 {name}의 한글 이름이 없다")
        if col not in cfg.colors or preset not in cfg.process:
            raise ValueError(f"{module}.{name}: 값 파일에 없는 색이나 프리셋 ({col}, {preset})")
        count = int(entry[4]) if len(entry) > 4 else 1
        vec = (float(up[0]), float(up[1]), float(up[2]))
        out.append(Item(module, name, labels[name], shape, vec, col, preset, count))
    return out


def parse_items(specs: Sequence[str], cfg: PrintConfig) -> list[Item]:
    """부품 지정(`wrist`, `wrist.wrist_body:2`, `coupons.fit3_6806_bores@white`)을 부품 목록으로. 같은 부품은 더한다."""
    found: dict[str, Item] = {}
    for spec in specs:
        m = SPEC.match(spec)
        if not m:
            raise ValueError(
                f"부품을 알아보지 못했다: {spec} (예: wrist, wrist.wrist_body:2, coupons.fit3_6806_bores@white)"
            )
        sets = int(m["count"]) if m["count"] else 1
        for item in _module_items(m["module"], m["part"], m["color"], cfg):
            prev = found.get(item.name)
            if prev is not None and prev.module != item.module:
                raise ValueError(
                    f"파일 이름 {item.name}이 {prev.module}와 {item.module}에 같이 있다"
                )
            count = item.count * sets + (prev.count if prev else 0)
            found[item.name] = replace(item, count=count)
    return list(found.values())


def oriented(shape: Compound, up: Vec) -> Compound:
    """up이 +Z를 보게 돌리고, 바닥을 z = 0에, 가운데를 원점 위에 둔다."""
    norm = math.sqrt(sum(a * a for a in up))
    u = (up[0] / norm, up[1] / norm, up[2] / norm)
    axis = (u[1], -u[0], 0.0)  # u × z
    s, c = math.hypot(axis[0], axis[1]), u[2]
    if s < 1e-9:
        turned = shape if c > 0 else shape.rotate(Axis.X, 180)
    else:
        turned = shape.rotate(Axis((0, 0, 0), axis), math.degrees(math.atan2(s, c)))
    bb = turned.bounding_box()
    centre = bb.center()
    return turned.moved(Pos(-centre.X, -centre.Y, -bb.min.Z))


def geometry(item: Item) -> tuple[Compound, Geometry]:
    """뽑는 방향으로 놓은 형상과, 안내에 보일 높이, 바닥 넓이, 걸침."""
    placed = oriented(item.shape, item.up)
    height = placed.bounding_box().size.Z
    base = sum(
        f.area
        for f in placed.faces()
        if f.geom_type == GeomType.PLANE and f.center().Z < 0.01 and f.normal_at().Z < -0.999
    )
    over = overhangs(item.shape, item.up)
    slender = height > SLENDER * math.sqrt(max(base, 1e-9))
    return placed, Geometry(height, base, over.total - over.flat, over.largest, slender)


# ---- 판 나누기와 판마다 자르기 ------------------------------------------------------------------------------


@dataclass
class Plate:
    preset: str
    color: str
    items: dict[str, int]  # 파일 이름 → 개수
    path: Path | None = None
    seconds: float = 0.0
    grams: float = 0.0
    png: bytes | None = None
    problems: list[str] = field(default_factory=list[str])


def groups(items: Sequence[Item]) -> list[tuple[tuple[str, str], list[Item]]]:
    """프리셋과 색으로 묶는다. 같은 색 묶음을 이어 두고(필라멘트를 덜 바꾼다), 그 안에서는 먼저 적은 부품이 먼저다."""
    colors: list[str] = []
    out: dict[tuple[str, str], list[Item]] = {}
    first: dict[tuple[str, str], int] = {}
    for k, item in enumerate(items):
        if item.color not in colors:
            colors.append(item.color)
        key = (item.preset, item.color)
        out.setdefault(key, []).append(item)
        first.setdefault(key, k)
    return sorted(out.items(), key=lambda kv: (colors.index(kv[0][1]), first[kv[0]]))


def check_plate(cfg: PrintConfig, plate: Plate, sliced: Sliced) -> list[str]:
    out: list[str] = []
    if len(sliced.plates) != 1:
        out.append(f"판이 {len(sliced.plates)}개로 나왔다 (하나여야 한다)")
    if sliced.plates:
        info = sliced.plates[0]
        plate.seconds, plate.grams = info.seconds, info.grams
        if info.support:
            out.append("서포트가 생겼다")
        if info.outside:
            out.append("판 밖으로 나간 부품이 있다")
        got = Counter(Path(o).stem for o in info.objects)
        if got != Counter(plate.items):
            out.append(f"판에 든 부품 {dict(got)}이 계획 {plate.items}과 다르다")
    out += setting_problems(sliced.settings, cfg.keys_for(plate.preset))
    out += bed_problems(cfg, sliced)
    out += fingerprint_problems(cfg, plate.preset, sliced.settings)
    out += orientation_problems(sliced.transforms)
    if not sliced.gcode:
        out.append("G-code가 들어 있지 않다")
    plate.png = sliced.png
    return out


@dataclass
class Plan:
    name: str
    items: list[Item]
    geometry: dict[str, Geometry]
    plates: list[Plate]
    problems: list[str]  # 판 하나에 매이지 않는 문제

    @property
    def ok(self) -> bool:
        return not self.problems and all(not p.problems for p in self.plates)


def make_plan(specs: Sequence[str], name: str, out: Path = OUT_DIR, exe: Path = STUDIO) -> Plan:
    """자르고 검사한다. 파일은 내지 않는다 (publish)."""
    cfg = load_config()
    flat, problems = check_studio(cfg, exe)
    plan = Plan(name, [], {}, [], problems)
    if problems:
        return plan
    plan.items = parse_items(specs, cfg)
    work = out / ".work" / name
    if work.exists():
        shutil.rmtree(work)
    presets = write_presets(cfg, flat, work / "presets")
    stls: dict[str, Path] = {}
    for item in plan.items:
        placed, geo = geometry(item)
        plan.geometry[item.name] = geo
        if geo.height > cfg.max_height_mm:
            problems.append(
                f"{item.name}: 높이 {geo.height:.1f} mm가 출력 범위({cfg.max_height_mm:.0f} mm)를 넘는다"
            )
        stls[item.name] = work / "stl" / f"{item.name}.stl"
        stls[item.name].parent.mkdir(parents=True, exist_ok=True)
        export_stl(placed, str(stls[item.name]))  # 공차 시험편과 같은 허용 오차 (build123d 기본값)
    for (preset, color), members in groups(plan.items):
        inputs = [stls[i.name] for i in members for _ in range(i.count)]
        made, code, log = run_studio(
            cfg, presets, preset, color, inputs, work / f"{preset}-{color}", "layout", exe=exe
        )
        if code != 0 or not made.exists():
            problems.append(
                f"{preset} · {color}: 배치하려고 자르지 못했다 ({code}) "
                + "; ".join(log_problems(cfg, code, log))
            )
            continue
        for info in read_3mf(made).plates:
            plan.plates.append(
                Plate(preset, color, dict(Counter(Path(o).stem for o in info.objects)))
            )
    for k, plate in enumerate(plan.plates, start=1):
        inputs = [stls[n] for n, c in plate.items.items() for _ in range(c)]
        made, code, log = run_studio(
            cfg,
            presets,
            plate.preset,
            plate.color,
            inputs,
            work / f"plate{k:02d}",
            "plate",
            exe=exe,
        )
        plate.problems += log_problems(cfg, code, log)
        if not made.exists():
            plate.problems.append("3MF가 나오지 않았다")
            continue
        plate.path = made
        plate.problems += check_plate(cfg, plate, read_3mf(made))
    placed_all: Counter[str] = Counter()
    for plate in plan.plates:
        placed_all.update(plate.items)
    wanted = Counter({i.name: i.count for i in plan.items})
    if placed_all != wanted:
        problems.append(f"판에 들어간 부품 {dict(placed_all)}이 목록 {dict(wanted)}과 다르다")
    return plan


def publish(plan: Plan, out: Path = OUT_DIR) -> Path:
    """통과한 계획을 build/print/<이름>/에 낸다. 통과하지 못했으면 그 폴더를 지우고 아무것도 내지 않는다."""
    cfg = load_config()
    dest = out / plan.name
    if dest.exists():
        shutil.rmtree(dest)  # 옛 파일을 뽑지 않게
    report = out / ".work" / plan.name / "report.txt"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(describe(plan, cfg), encoding="utf-8")
    if not plan.ok:
        return report
    dest.mkdir(parents=True)
    labels = {i.name: i.label for i in plan.items}
    plates: list[dict[str, Any]] = []
    for k, plate in enumerate(plan.plates, start=1):
        assert plate.path is not None
        data = plate.path.read_bytes()
        stem = f"{k:02d}-{plate.color}-{hashlib.sha256(data).hexdigest()[:6]}"
        (dest / f"{stem}.3mf").write_bytes(data)
        if plate.png:
            (dest / f"{stem}.png").write_bytes(plate.png)
        plates.append(
            {
                "file": f"{stem}.3mf",
                "image": f"{stem}.png" if plate.png else None,
                "color": plate.color,
                "preset": cfg.process[plate.preset],
                "minutes": round(plate.seconds / 60),
                "grams": round(plate.grams, 1),
                "overnight": plate.seconds > cfg.overnight_hours * 3600,
                "parts": [
                    {
                        "file": f"{n}.stl",
                        "label": labels[n],
                        "count": c,
                        **_geo_json(plan.geometry[n]),
                    }
                    for n, c in plate.items.items()
                ],
            }
        )
    summary = {
        "name": plan.name,
        "made": datetime.now().isoformat(timespec="seconds"),
        "studio": cfg.studio_version,
        "profiles": cfg.profiles_version,
        "bed_type": cfg.bed_type,
        "plates": plates,
    }
    (dest / "plan.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    return dest


def _geo_json(g: Geometry) -> dict[str, Any]:
    return {
        "height_mm": round(g.height, 1),
        "base_mm2": round(g.base_area),
        "overhang_mm2": round(g.overhang),
        "slender": g.slender,
    }


def _hm(seconds: float) -> str:
    minutes = round(seconds / 60)
    return f"{minutes // 60}시간 {minutes % 60}분" if minutes >= 60 else f"{minutes}분"


def describe(plan: Plan, cfg: PrintConfig) -> str:
    lines = [
        f"출력 준비 {plan.name}: 스튜디오 {cfg.studio_version}, 프리셋 묶음 {cfg.profiles_version}, 판 {cfg.bed_type}"
    ]
    labels = {i.name: i.label for i in plan.items}
    for k, plate in enumerate(plan.plates, start=1):
        night = "  (밤에)" if plate.seconds > cfg.overnight_hours * 3600 else ""
        state = "통과" if not plate.problems else "막힘"
        lines.append(
            f"\n판 {k} [{state}] {COLOR_KO.get(plate.color, plate.color)} · {cfg.process[plate.preset]} · "
            f"{_hm(plate.seconds)} · {plate.grams:.1f} g · 부품 {sum(plate.items.values())}개{night}"
        )
        for n, c in plate.items.items():
            g = plan.geometry[n]
            note = "  넘어질 수 있음(참고)" if g.slender else ""
            lines.append(
                f"  {labels[n]} ×{c} ({n}.stl)  높이 {g.height:.1f} mm, 바닥 {g.base_area:.0f} mm², "
                f"걸침 {g.overhang:.0f} mm²{note}"
            )
        lines += [f"  - {p}" for p in plate.problems]
    lines += [f"- {p}" for p in plan.problems]
    total = sum(p.seconds for p in plan.plates)
    lines.append(
        f"\n판 {len(plan.plates)}개, {_hm(total)}, {sum(p.grams for p in plan.plates):.1f} g"
    )
    lines.append("결과: " + ("통과, 파일을 냈다" if plan.ok else "막힘, 파일을 내지 않았다"))
    return "\n".join(lines)


# ---- 기준 견주기 (가정 B) ---------------------------------------------------------------------------------


def _moves(gcode: str) -> list[str]:
    """G-code에서 주석과 빈 줄을 뺀 명령 줄."""
    out: list[str] = []
    for line in gcode.splitlines():
        code = line.split(";", 1)[0].strip()
        if code:
            out.append(code)
    return out


def compare_reference(gui: Sliced, cli: Sliced, ignore: Sequence[str]) -> list[str]:
    """화면에서 자른 것과 명령 모드로 자른 것의 차이 (설정 칸, 시간과 무게, G-code 명령 줄)."""
    out: list[str] = []
    keys = sorted(set(gui.settings) | set(cli.settings))
    for key in keys:
        if key in ignore:
            continue
        a, b = gui.settings.get(key), cli.settings.get(key)
        if a != b:
            out.append(f"설정 {key}: 화면 {a}, 명령 모드 {b}")
    if not gui.plates or not cli.plates:
        out.append("잘린 결과(slice_info)가 없다")
    else:
        g, c = gui.plates[0], cli.plates[0]
        if (g.seconds, g.grams) != (c.seconds, c.grams):
            out.append(
                f"예상 시간·무게: 화면 {g.seconds:.0f} s, {g.grams} g / 명령 모드 {c.seconds:.0f} s, {c.grams} g"
            )
    if not gui.gcode:
        out.append("화면 3MF에 G-code가 없다 (자른 뒤 저장했는지 본다)")
    else:
        a, b = _moves(gui.gcode), _moves(cli.gcode)
        if a != b:
            diff = next(
                (k for k, (x, y) in enumerate(zip(a, b, strict=False)) if x != y),
                min(len(a), len(b)),
            )
            out.append(
                f"G-code 명령 줄이 다르다: 화면 {len(a)}줄, 명령 모드 {len(b)}줄, 처음 다른 줄 {diff + 1}: "
                f"{a[diff] if diff < len(a) else '-'} / {b[diff] if diff < len(b) else '-'}"
            )
    return out


def reference(gui_path: Path, out: Path = OUT_DIR, exe: Path = STUDIO) -> int:
    cfg = load_config()
    flat, problems = check_studio(cfg, exe)
    if problems:
        print("\n".join(problems))
        return 1
    gui = read_3mf(gui_path)
    preset = next(
        (k for k, v in cfg.process.items() if v == gui.settings.get("print_settings_id")), None
    )
    if preset is None:
        print(f"화면 3MF의 공정 프리셋 {gui.settings.get('print_settings_id')}이 값 파일에 없다")
        return 1
    colour = str(_first(gui.settings.get("filament_colour"))).upper()
    color = next((k for k, v in cfg.colors.items() if v.upper() == colour), next(iter(cfg.colors)))
    work = out / ".work" / "reference"
    if work.exists():
        shutil.rmtree(work)
    presets = write_presets(cfg, flat, work / "presets")
    made, code, log = run_studio(
        cfg, presets, preset, color, [gui_path], work, "cli", arrange=False, exe=exe
    )
    problems = log_problems(cfg, code, log)
    if not made.exists():
        print("\n".join([*problems, "명령 모드 3MF가 나오지 않았다"]))
        return 1
    cli = read_3mf(made)
    problems += compare_reference(gui, cli, cfg.sha_ignore)
    if problems:
        print(f"가정 B 실패 ({preset}): 차이 {len(problems)}개")
        print("\n".join(f"- {p}" for p in problems[:80]))
        return 1
    print(
        f'가정 B 통과 ({preset}). 값 파일에 적는다: settings_sha.{preset} = "{settings_sha(cli.settings, cfg.sha_ignore)}"'
    )
    return 0


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="출력 부품을 뱀부랩 스튜디오로 잘라, 판마다 바로 뽑을 3MF를 만든다."
    )
    sub = parser.add_subparsers(dest="command", required=True)
    p_plan = sub.add_parser(
        "plan", help="부품을 판에 모아 자르고 검사한다. 모두 통과해야 파일을 낸다"
    )
    p_plan.add_argument(
        "items",
        nargs="+",
        help="wrist, wrist.wrist_body:2, coupons.fit3_6806_bores@white 같은 부품 지정",
    )
    p_plan.add_argument("--name", required=True, help="묶음 이름 (build/print/<이름>/)")
    p_ref = sub.add_parser(
        "reference", help="화면에서 잘라 저장한 3MF와 명령 모드 결과를 견준다 (가정 B)"
    )
    p_ref.add_argument("gui_3mf", type=Path)
    args = parser.parse_args(argv)
    if args.command == "reference":
        sys.exit(reference(args.gui_3mf))
    plan = make_plan(args.items, args.name)
    where = publish(plan)
    print(describe(plan, load_config()))
    print(f"\n{where.relative_to(REPO_ROOT).as_posix()}")
    sys.exit(0 if plan.ok else 1)


if __name__ == "__main__":
    main()
