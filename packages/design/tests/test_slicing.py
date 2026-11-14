"""출력 준비 도구 (docs/decisions/0035-slicer-automation.md): 값 파일, 검사, 묶기, 실패하면 파일을 내지 않기.
스튜디오가 깔린 PC에서는 시험편 하나를 실제로 잘라 본다."""

from __future__ import annotations

import json
import math
import zipfile
from pathlib import Path
from typing import Any

import pytest
from build123d import Box, Compound, Pos

from horilab_design import slicing as sl

CFG = sl.load_config()
NOZZLE_LINE = (
    "[2026-11-14 17:01:21.691221] [0x0000a730] [error]   3198: nozzle_volume_type not found, when different_extruder "
    "or multiple extruder, new_printer_name Bambu Lab P1S 0.4 nozzle, extruder_count 1"
)


def _write(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data), encoding="utf-8")


def _settings(preset: str) -> dict[str, Any]:
    """값 파일의 핵심 설정을 스튜디오가 3MF에 쓰는 모양으로 (어떤 칸은 목록, 속도는 노즐 종류마다)."""
    values: dict[str, Any] = dict(CFG.keys_for(preset))
    for key in (
        "filament_settings_id",
        "filament_density",
        "nozzle_diameter",
        "textured_plate_temp_initial_layer",
    ):
        values[key] = [values[key]]
    values["outer_wall_speed"] = [values["outer_wall_speed"], "350"]
    values["filament_colour"] = ["#FFFFFF"]
    return values


def _sliced(settings: dict[str, Any], gcode: str = "M140 S55\nM190 S55\n", **kw: Any) -> sl.Sliced:
    plates = kw.get("plates", (sl.PlateInfo(1, 600.0, 5.0, False, False, ("a.stl",)),))
    return sl.Sliced(plates, settings, gcode, None, kw.get("transforms", ()))


def test_config_holds_documented_values() -> None:
    strength = CFG.keys_for("strength")
    assert strength["wall_loops"] == "6" and strength["sparse_infill_density"] == "25%"
    assert strength["curr_bed_type"] == CFG.bed_type == "Textured PEI Plate"
    assert CFG.keys_for("standard")["wall_loops"] == "2"
    assert "filament_colour" in CFG.sha_ignore
    assert ("error", sl.TAG.search(NOZZLE_LINE).group(2)) in CFG.harmless  # pyright: ignore[reportOptionalMemberAccess]


def test_flatten_follows_inherits(tmp_path: Path) -> None:
    _write(
        tmp_path / "process" / "child.json",
        {"name": "Child", "inherits": "Base", "wall_loops": "6"},
    )
    _write(
        tmp_path / "process" / "sub" / "base.json",
        {
            "name": "Base",
            "inherits": "",
            "wall_loops": "2",
            "layer_height": "0.2",
            "instantiation": "false",
        },
    )
    flat = sl.flatten(tmp_path, "process", "Child")
    assert flat["wall_loops"] == "6" and flat["layer_height"] == "0.2"
    assert flat["inherits"] == "" and flat["from"] == "system" and "instantiation" not in flat
    with pytest.raises(KeyError):
        sl.flatten(tmp_path, "process", "Missing")


def test_key_settings_compare_first_value() -> None:
    for preset in CFG.process:
        assert sl.setting_problems(_settings(preset), CFG.keys_for(preset)) == []
    wrong = sl.setting_problems(
        _settings("strength") | {"wall_loops": "4"}, CFG.keys_for("strength")
    )
    assert len(wrong) == 1 and "wall_loops" in wrong[0]
    missing = _settings("standard")
    del missing["enable_support"]
    assert any(
        "enable_support" in p for p in sl.setting_problems(missing, CFG.keys_for("standard"))
    )
    assert (
        sl.setting_problems(
            _settings("strength") | {"layer_height": "0.20"}, CFG.keys_for("strength")
        )
        == []
    )


def test_settings_fingerprint_ignores_colour_only() -> None:
    a = _settings("strength")
    b = a | {"filament_colour": ["#202020"]}
    c = a | {"wall_loops": "5"}
    assert sl.settings_sha(a, CFG.sha_ignore) == sl.settings_sha(b, CFG.sha_ignore)
    assert sl.settings_sha(a, CFG.sha_ignore) != sl.settings_sha(c, CFG.sha_ignore)
    empty = sl.PrintConfig(**{**CFG.__dict__, "settings_sha": {"strength": ""}})
    assert "기준 지문이 없다" in sl.fingerprint_problems(empty, "strength", a)[0]
    right = sl.PrintConfig(
        **{**CFG.__dict__, "settings_sha": {"strength": sl.settings_sha(a, CFG.sha_ignore)}}
    )
    assert sl.fingerprint_problems(right, "strength", b) == []
    assert "다르다" in sl.fingerprint_problems(right, "strength", c)[0]


def test_log_lets_only_exact_harmless_lines_through() -> None:
    harmless = (
        NOZZLE_LINE
        + "\n[trace] Initializing\n[2026] [0x1] [warning] tree support default to organic support\n"
    )
    assert sl.log_problems(CFG, 0, harmless) == []
    near = NOZZLE_LINE.replace("extruder_count 1", "extruder_count 2")
    assert sl.log_problems(CFG, 0, near)[0].startswith("스튜디오 오류")
    assert sl.log_problems(CFG, 0, "[warning] floating regions found")[0].startswith("검토 필요")
    assert sl.log_problems(CFG, 0, "[error] ZFiller: encounter idx from clip: 85") == [
        "스튜디오 오류: ZFiller: encounter idx from clip: 85"
    ]
    assert sl.log_problems(CFG, -1073741819, "") == ["스튜디오가 -1073741819로 끝났다"]


def test_bed_type_and_gcode_temperature() -> None:
    good = _settings("strength")
    assert sl.bed_problems(CFG, _sliced(good)) == []
    cool = good | {"curr_bed_type": "Cool Plate"}
    assert any("판 종류" in p for p in sl.bed_problems(CFG, _sliced(cool)))
    assert len(sl.bed_problems(CFG, _sliced(good, "M140 S35\nM190 S35\n"))) == 2
    assert len(sl.bed_problems(CFG, _sliced(good, ""))) == 2


def test_orientation_allows_only_turns_about_z() -> None:
    c, s = math.cos(0.7), math.sin(0.7)
    identity = (1, 0, 0, 0, 1, 0, 0, 0, 1, 10, 20, 0)
    about_z = (c, -s, 0, s, c, 0, 0, 0, 1, 128, 128, 25.4)
    about_x = (1, 0, 0, 0, c, -s, 0, s, c, 0, 0, 0)
    assert sl.orientation_problems([identity, about_z]) == []
    assert sl.orientation_problems([about_x]) != []
    assert sl.orientation_problems([(1, 0, 0)]) != []


def test_read_3mf(tmp_path: Path) -> None:
    info = """<?xml version="1.0" encoding="UTF-8"?>
<config><plate>
<metadata key="index" value="1"/><metadata key="prediction" value="1525"/><metadata key="weight" value="9.40"/>
<metadata key="outside" value="false"/><metadata key="support_used" value="false"/>
<object identify_id="15" name="a.stl" skipped="false" /><object identify_id="16" name="b.stl" skipped="false" />
</plate></config>"""
    model = '<model><build><item objectid="2" transform="1 0 0 0 1 0 0 0 1 97 101 0" printable="1"/></build></model>'
    path = tmp_path / "x.3mf"
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("Metadata/slice_info.config", info)
        z.writestr("Metadata/project_settings.config", json.dumps({"wall_loops": "6"}))
        z.writestr("Metadata/plate_1.gcode", "; head\nM140 S55\n")
        z.writestr("Metadata/plate_1.png", b"png")
        z.writestr("3D/3dmodel.model", model)
    got = sl.read_3mf(path)
    assert got.plates[0].seconds == 1525 and got.plates[0].grams == pytest.approx(9.4)
    assert got.plates[0].objects == ("a.stl", "b.stl") and not got.plates[0].support
    assert got.settings["wall_loops"] == "6" and "M140 S55" in got.gcode and got.png == b"png"
    assert got.transforms == ((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 97.0, 101.0, 0.0),)


def test_item_specs() -> None:
    items = sl.parse_items(
        ["coupons.fit3_6806_bores@white", "coupons.fit3_6806_bores:2@white"], CFG
    )
    assert [(i.name, i.count, i.color, i.preset) for i in items] == [
        ("fit3_6806_bores", 3, "white", "strength")
    ]
    with pytest.raises(ValueError, match="색을 붙인다"):
        sl.parse_items(["coupons.fit3_6806_bores"], CFG)
    with pytest.raises(ValueError, match="시험편에만"):
        sl.parse_items(["wrist@white"], CFG)
    with pytest.raises(ValueError, match="알아보지 못했다"):
        sl.parse_items(["wrist.wrist_body:0"], CFG)


@pytest.mark.parametrize(
    ("up", "height"),
    [
        ((0.0, -1.0, 0.0), 24.0),
        ((0.0, 1.0, 0.0), 24.0),
        ((0.0, 0.0, -1.0), 30.0),
        ((1.0, 0.0, 0.0), 10.0),
    ],
)
def test_oriented_puts_up_along_z(up: sl.Vec, height: float) -> None:
    # -Y 쪽에 혹이 난 상자: up이 -Y이면 혹이 위로 간다
    shape = Compound([Box(10, 20, 30), Pos(0, -12, 0) * Box(2, 4, 2)])
    placed = sl.oriented(shape, up)
    bb = placed.bounding_box()
    assert bb.size.Z == pytest.approx(height) and bb.min.Z == pytest.approx(0.0, abs=1e-6)
    assert bb.center().X == pytest.approx(0.0, abs=1e-6) and bb.center().Y == pytest.approx(
        0.0, abs=1e-6
    )
    if up == (0.0, -1.0, 0.0):
        assert placed.center().Z < 12.0  # 큰 상자가 바닥(0~20)에, 혹이 위(20~24)에
    if up == (0.0, 1.0, 0.0):
        assert placed.center().Z > 12.0  # 혹이 바닥에


def _item(name: str, shape: Compound, color: str = "white", preset: str = "strength") -> sl.Item:
    return sl.Item("t", name, name, shape, (0.0, 0.0, 1.0), color, preset, 1)


def test_geometry_reports_base_area_and_slender_parts() -> None:
    _, flat = sl.geometry(_item("flat", Compound([Box(10, 20, 30)])))
    assert (
        flat.height == pytest.approx(30)
        and flat.base_area == pytest.approx(200)
        and not flat.slender
    )
    _, tall = sl.geometry(_item("tall", Compound([Box(5, 5, 40)])))
    assert tall.slender


def test_groups_keep_colours_together_in_listed_order() -> None:
    box = Compound([Box(1, 1, 1)])
    items = [
        _item("a", box, "white", "strength"),
        _item("b", box, "black", "strength"),
        _item("c", box, "white", "standard"),
        _item("d", box, "black", "strength"),
    ]
    order = [(key, [i.name for i in members]) for key, members in sl.groups(items)]
    assert order == [
        (("strength", "white"), ["a"]),
        (("standard", "white"), ["c"]),
        (("strength", "black"), ["b", "d"]),
    ]


def test_reference_compare_ignores_comments_only() -> None:
    s = _settings("strength")
    a = _sliced(s, "; made 10:00\nG1 X1 Y2\n")
    assert (
        sl.compare_reference(a, _sliced(s, "; made 11:00\nG1 X1 Y2 ; move\n"), CFG.sha_ignore) == []
    )
    diff = sl.compare_reference(a, _sliced(s | {"wall_loops": "5"}, "G1 X1 Y3\n"), CFG.sha_ignore)
    assert any("wall_loops" in d for d in diff) and any("G-code" in d for d in diff)


def test_failed_plan_removes_old_files(tmp_path: Path) -> None:
    old = tmp_path / "batch"
    old.mkdir()
    (old / "01-black-abcdef.3mf").write_bytes(b"old")
    plan = sl.Plan("batch", [], {}, [], ["프리셋 내용이 바뀌었다"])
    where = sl.publish(plan, tmp_path)
    assert (
        not old.exists()
        and where.name == "report.txt"
        and "막힘" in where.read_text(encoding="utf-8")
    )


@pytest.mark.skipif(not sl.STUDIO.exists(), reason="뱀부랩 스튜디오가 없는 PC")
def test_studio_slices_a_coupon_with_the_pinned_settings(tmp_path: Path) -> None:
    _, problems = sl.check_studio(CFG)
    if problems:
        pytest.skip("이 PC의 스튜디오나 프리셋이 값 파일과 다르다: " + "; ".join(problems))
    plan = sl.make_plan(["coupons.fit3_6806_bores@white"], "t", tmp_path)
    assert plan.problems == [] and len(plan.plates) == 1
    plate = plan.plates[0]
    assert plate.seconds > 0 and plate.grams > 0 and plate.png
    left = [p for p in plate.problems if "기준 지문이 없다" not in p]
    assert left == []  # 판 종류, 핵심 설정, 방향, 로그가 모두 통과
    assert plan.ok == bool(CFG.settings_sha.get("strength"))
