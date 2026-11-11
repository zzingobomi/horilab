"""레포 구조 검사: Python 패키지와 Rust 크레이트의 경계, 의존 방향 (docs/decisions/0007-repo-structure.md).

uv 워크스페이스에서는 모든 패키지가 한 가상환경에 깔려서, 경계를 어기는 import도
PC에서는 그냥 돈다. 그래서 소스를 읽어 import를 확인한다. 경계 규칙의 원본은 이 파일이다.
"""

import ast
import functools
import importlib
import importlib.metadata
import json
import re
import shutil
import subprocess
import sys
import tomllib
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[1]

# 패키지마다 불러와도 되는 다른 horilab 패키지. 아직 없는 패키지(client, sim)도 미리 적어 둔다.
# 새 패키지는 여기에 규칙을 먼저 넣어야 테스트가 통과한다.
ALLOWED_INTERNAL: dict[str, set[str]] = {
    "horilab_design": set(),
    "horilab_servo": set(),
    "horilab_camera": set(),
    "horilab_client": set(),
    "horilab_sim": {"horilab_camera"},
}

# 응용(apps/)은 모두 같은 규칙이다. 로봇 안쪽을 부르지 않아야 다른 레포로 옮길 수 있다.
APP_ALLOWED_INTERNAL = {"horilab_client", "horilab_camera"}

# 서보에 직접 닿는 모듈. servo 패키지 밖에서 부르면 안전 검사를 건너뛰는 길이 생긴다 (0009).
SERVO_ACCESS = {"horilab_servo", "serial", "scservo_sdk"}

# 따로 설치되는 패키지(파이, 다른 레포, 별도 환경)와, 그 패키지가 불러오면 안 되는 무거운 라이브러리
LIGHT_PACKAGES = ("horilab_servo", "horilab_camera", "horilab_client")
HEAVY_LIBRARIES = {"build123d", "OCP", "bd_warehouse", "ocp_vscode", "mujoco", "mink", "eaik"}

# Rust 크레이트마다 부를 수 있는 다른 horilab 크레이트. 아직 없는 크레이트(controller)도 미리 적어 둔다.
CRATE_ALLOWED_INTERNAL: dict[str, set[str]] = {
    "horilab-servo": set(),
    "horilab-controller": {"horilab-servo"},
    "horilab-bench": {"horilab-servo"},
}

# 시리얼 포트를 여는 크레이트. 서보에 닿는 길이라 horilab-servo만 쓴다.
SERIAL_CRATES = {"serialport", "serial2", "tokio-serial", "mio-serial", "rustypot"}

# 검사하지 않는 폴더 (생성물, 받은 도구, 가상환경)
SKIP_DIRS = {".venv", ".git", "build", "target", "node_modules", "__pycache__"}


def _projects(folder: str) -> dict[str, Path]:
    """folder 아래 pyproject.toml이 있는 폴더를 import 이름으로 찾는다."""
    base = ROOT / folder
    if not base.is_dir():
        return {}
    return {
        "horilab_" + path.name.replace("-", "_"): path
        for path in sorted(base.iterdir())
        if (path / "pyproject.toml").exists()
    }


PACKAGES = _projects("packages")
APPS = _projects("apps")
PROJECTS = PACKAGES | APPS
INDEPENDENT = sorted([p for p in LIGHT_PACKAGES if p in PACKAGES] + list(APPS))


def _top_level_imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module.split(".")[0])
    return names


def _source_files(name: str) -> list[Path]:
    return sorted((PROJECTS[name] / "src" / name).rglob("*.py"))


def _repo_python_files() -> list[Path]:
    files: list[Path] = []
    for path in ROOT.rglob("*.py"):
        parts = path.relative_to(ROOT).parts
        if SKIP_DIRS.intersection(parts) or parts[:2] == ("tools", "local"):
            continue
        files.append(path)
    return sorted(files)


def _normalize(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def _declared(project: Path) -> set[str]:
    data = tomllib.loads((project / "pyproject.toml").read_text(encoding="utf-8"))
    names: set[str] = set()
    for requirement in data["project"].get("dependencies", []):
        match = re.match(r"[A-Za-z0-9._-]+", requirement)
        assert match, f"{project.name}/pyproject.toml: {requirement}"
        names.add(_normalize(match.group(0)))
    return names


_DISTRIBUTIONS = importlib.metadata.packages_distributions()


def _distributions_of(module: str) -> set[str]:
    if module.startswith("horilab_"):
        return {_normalize(module)}  # 워크스페이스 패키지: horilab_servo → horilab-servo
    return {_normalize(name) for name in _DISTRIBUTIONS.get(module, [])}


@functools.cache
def _crates() -> list[dict[str, Any]]:
    """워크스페이스의 크레이트와 그 크레이트가 Cargo.toml에 적은 의존성."""
    if not (ROOT / "Cargo.toml").exists():
        return []
    cargo = shutil.which("cargo")
    assert cargo, "cargo가 없다. docs/setup.md#8-rust대로 Rust를 깐다"
    result = subprocess.run(
        [cargo, "metadata", "--format-version", "1", "--no-deps", "--offline"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=True,
    )
    return json.loads(result.stdout)["packages"]


def test_every_crate_has_rules() -> None:
    missing = {crate["name"] for crate in _crates()} - set(CRATE_ALLOWED_INTERNAL)
    assert not missing, f"규칙이 없는 크레이트: {sorted(missing)}. 0007과 이 파일에 먼저 넣는다"


def test_crate_dependencies_follow_direction() -> None:
    for crate in _crates():
        name = crate["name"]
        dependencies = {dependency["name"] for dependency in crate["dependencies"]}
        internal = {d for d in dependencies if d.startswith("horilab-")}
        allowed = CRATE_ALLOWED_INTERNAL.get(name, set())
        assert internal <= allowed, f"{name}: {sorted(internal - allowed)}"
        serial = dependencies & SERIAL_CRATES
        assert not serial or name == "horilab-servo", (
            f"{name}: {sorted(serial)}. 시리얼 포트는 horilab-servo만 연다"
        )


def test_every_package_has_rules() -> None:
    missing = set(PACKAGES) - set(ALLOWED_INTERNAL)
    assert not missing, f"규칙이 없는 패키지: {sorted(missing)}. 0007과 이 파일에 먼저 넣는다"


@pytest.mark.parametrize("name", sorted(PROJECTS))
def test_project_imports(name: str) -> None:
    importlib.import_module(name)


@pytest.mark.parametrize("name", sorted(PROJECTS))
def test_internal_imports_follow_dependency_direction(name: str) -> None:
    allowed = (APP_ALLOWED_INTERNAL if name in APPS else ALLOWED_INTERNAL[name]) | {name}
    for path in _source_files(name):
        internal = {n for n in _top_level_imports(path) if n.startswith("horilab_")}
        assert internal <= allowed, f"{path.relative_to(ROOT)}: {sorted(internal - allowed)}"


def test_only_servo_package_reaches_servos() -> None:
    servo = ROOT / "packages" / "servo"
    for path in _repo_python_files():
        if path.is_relative_to(servo):
            continue
        found = _top_level_imports(path) & SERVO_ACCESS
        assert not found, f"{path.relative_to(ROOT)}: {sorted(found)}. 컨트롤러를 거친다"


@pytest.mark.parametrize("name", [p for p in LIGHT_PACKAGES if p in PACKAGES])
def test_light_packages_avoid_heavy_libraries(name: str) -> None:
    for path in _source_files(name):
        heavy = _top_level_imports(path) & HEAVY_LIBRARIES
        assert not heavy, f"{path.relative_to(ROOT)}: {sorted(heavy)}"


@pytest.mark.parametrize("name", INDEPENDENT)
def test_independent_installs_declare_their_imports(name: str) -> None:
    """따로 설치되는 곳에서는 한 환경에 다 깔린 PC와 달리 적지 않은 의존성이 빠진다."""
    project = PROJECTS[name]
    declared = _declared(project)
    for path in _source_files(name):
        for module in _top_level_imports(path) - set(sys.stdlib_module_names) - {name}:
            distributions = _distributions_of(module)
            where = f"{path.relative_to(ROOT)}: {module}"
            assert distributions, f"{where}을 내는 배포판을 찾지 못했다"
            assert distributions & declared, (
                f"{where}({', '.join(sorted(distributions))})이 "
                f"{project.name}/pyproject.toml의 dependencies에 없다"
            )
