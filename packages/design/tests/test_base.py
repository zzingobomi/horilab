"""J1 관절(베이스 몸통, 돌림판, J1 몸통): 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 멈춤과 영점 핀이 맞는지."""

from __future__ import annotations

import pytest

from horilab_design.parts import base as b
from horilab_design.parts import joint as j
from horilab_design.parts import shoulder as sh
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.studies import ur_arm
from horilab_design.studies.packaging import MM
from horilab_design.values import ServoModel

BED = 256.0  # 출력 범위 [mm]
HORN_MASS_G = 2.0  # STS 서보의 출력 혼과 가운데 나사 (실측)


@pytest.fixture(scope="module")
def parts() -> list[j.Part]:
    return b.assembly("STS3215")


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_without_steep_overhang() -> None:
    for name, (shape, up, _color, _preset) in b.printed().items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        bb = shape.bounding_box()
        assert max(bb.size.X, bb.size.Y, bb.size.Z) < BED, name
        # 바닥에 닿지 않고 아래를 보는 면은 양쪽이 붙은 다리와 작은 가로 구멍 윗변뿐이어야 한다
        over = overhangs(shape, up)
        assert over.total - over.flat < 80.0, (name, over.total, over.flat)


@pytest.mark.slow
def test_base_parts_do_not_overlap(parts: list[j.Part]) -> None:
    assert b.overlaps([p for p in parts if "핀" not in p.name]) == {}


@pytest.mark.slow
def test_sts3250_fits_the_same_seat() -> None:
    # J1 서보 자리는 STS3250도 들어간다 (0028: 정밀도가 모자라면 바꿀 수 있게)
    parts = b.assembly("STS3250")
    assert b.overlaps([p for p in parts if "핀" not in p.name]) == {}


@pytest.mark.slow
def test_turntable_turns_freely_and_stops_on_the_web(parts: list[j.Part]) -> None:
    # 프로그램 한계까지는 도는 부품이 어디에도 닿지 않고, 멈춤 각에서 기둥이 살에 닿는다
    for q in (0.0, 90.0, j.SOFT_DEG, -90.0, -j.SOFT_DEG):
        gaps = b.clearances(parts, q)
        assert min(gaps.values()) > 0.4, (q, gaps)
    for q in (j.STOP_DEG, -j.STOP_DEG):
        gaps = b.clearances(parts, q)
        assert gaps[("돌림판", "베이스 몸통")] < 0.05, (q, gaps)


@pytest.mark.slow
def test_zero_pin_sits_in_both_holes(parts: list[j.Part]) -> None:
    core = next(p for p in parts if p.name == "베이스 몸통")
    body = next(p for p in parts if p.name == "J1 몸통")
    table = next(p for p in parts if p.name == "돌림판")
    pin = next(p for p in parts if "핀" in p.name)
    # 핀(지름 3.0)이 구멍(세로 3.35, 가로 3.30) 속에 있으면 벽과의 틈이 0.15 ~ 0.18이다
    assert 0.1 < pin.shape.distance_to(core.shape) < 0.25
    assert 0.1 < pin.shape.distance_to(body.shape) < 0.25
    # 돌림판은 핀을 잡지 않는다 (영점은 J1 몸통과 베이스 몸통의 구멍이 정한다)
    assert pin.shape.distance_to(table.shape) > 0.4


@pytest.mark.parametrize("model", ["STS3215", "STS3250"])
def test_screws_engage_fully_and_do_not_bottom(model: ServoModel) -> None:
    checks = b.screw_checks(model)
    # 혼 나사 M3 × 8: 혼의 테(2.5)에 거의 다 물리고, 끝이 서보 앞면에 닿지 않는다
    assert checks["혼 나사가 혼 테(2.5)에 물린 길이"] >= 2.4
    assert checks["혼 나사 끝과 서보 앞면 사이"] > 0.25
    # 조임 볼트 M3 × 45: 너트에 다 물리고, 끝이 허브 속에서 끝난다 (혼에 닿지 않는다)
    assert checks["조임 볼트가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert checks["조임 볼트 끝과 허브 밑면 사이"] > 0.5
    # 머리가 돌림판 윗면 아래에 있고, 머리 자리 밑에 위 안쪽 링까지 살이 2 mm 넘게 남는다
    assert checks["조임 볼트 머리 윗면과 돌림판 윗면 사이"] > 1.0
    assert checks["조임 볼트 머리 자리와 위 안쪽 링 윗면 사이의 살"] >= 2.0
    # J1 몸통 나사 M3 × 12: 바닥판 속 너트에 다 물리고, 끝이 바닥판 위로 나오지 않는다
    assert checks["J1 몸통 나사가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert checks["J1 몸통 나사 끝과 바닥판 윗면 사이"] >= 0.0
    assert checks["J1 몸통 바닥판과 어깨 서보 먼 끝 사이"] > 2.0


def test_stack_matches_the_skeleton() -> None:
    # 돌림판 윗면이 어깨 서보의 먼 끝과 J1 몸통의 바닥판 아래에 있다
    assert b.NECK_X == pytest.approx(-(b.SHOULDER_H - b.TOP_H))
    assert (b.SHOULDER_H - sh.FAR) - (b.TOP_H + b.FLOOR_T) > 2.0
    # 두 베어링의 가운데 사이가 30 mm다 (0028)
    assert (b.B1 - b.B0) == pytest.approx(30.0)
    # 저널 끝이 허브에 닿지 않는다: 베어링 폭 공차(둘 합쳐 0.24)와 출력 오차를 넘는 틈
    assert b.JOURNAL_GAP >= 0.5
    # 멈춤 나사 끝이 홈 바닥에 닿지 않고, 영점 핀이 살 속에 3.5 넘게 들어간다
    assert (b.ATOP - 1.8 - 10.0) - b.CH_Y > 0.8
    assert b.FRONT - (b.ATOP + j.EAR_T + 3.0 - 20.0) > 3.5


def test_j1_body_screws_sit_close_to_the_journal() -> None:
    # J1 몸통 나사가 돌림판의 단(반지름 16.5)에서 5 mm 안에 있다: 얇은 테가 휘는 길이가 짧다
    assert b.R_FOOT - j.R_STEP <= 5.0
    # 나사 머리 자리가 단과 겹치지 않고, 조임 볼트의 와셔(반지름 4)와 바닥판 가운데 구멍을 비킨다
    assert b.R_FOOT - 3.1 > j.R_STEP
    assert b.R_FOOT - 2.85 > b.FOOT_HOLE
    assert b.R_CLAMP + 4.0 < b.FOOT_HOLE


def test_clamp_bolt_heads_sit_in_pockets() -> None:
    # 가진 M3 × 45로 문다: 머리 자리 깊이가 볼트 끝이 너트를 지나고 허브 속에서 끝나는 범위(4.5 ~ 6.5)의 가운데다
    tip = b.ATOP - b.SINK + b.WASHER_T - b.CLAMP_LEN
    assert (b.B0 - b.NUT_ROOF - j.NUT_T) - tip >= 0.9  # 너트를 지나간 길이
    assert tip - (b.B0 - b.HUB_BODY) >= 0.9  # 허브 밑면까지 남은 길이
    # 천장이 0.2 mm 층의 경계에 온다 (막 한 층이 고르게 걸친다)
    assert b.SINK / 0.2 == pytest.approx(round(b.SINK / 0.2))
    assert pytest.approx(0.2) == b.SKIN
    # 머리 자리가 J1 몸통 바닥판의 가운데 구멍 안에 있고, 넓힌 단에 살 1.5 넘게 싸인다
    assert b.R_CLAMP + b.SINK_R < b.FOOT_HOLE
    assert b.STEP_WIDE_R - (b.R_CLAMP + b.SINK_R) >= 1.5
    # 넓힌 단은 몸통의 턱 구멍과 1.5 넘게 뜨고, 볼트는 혼의 바깥을 지난다
    assert j.R_LIP - b.STEP_WIDE_R >= 1.5
    assert b.R_CLAMP - 1.5 > j.HORN_R
    # 머리 자리끼리 떨어져 있다
    assert b.R_CLAMP * 2**0.5 - 2 * b.SINK_R > 5.0


@pytest.mark.slow
def test_clamp_bolt_holes_are_closed_by_one_layer(parts: list[j.Part]) -> None:
    # 머리 자리의 천장은 막 한 층이 볼트 구멍까지 덮는다 (뽑을 때 줄이 끊기지 않고 걸친다)
    table = next(p for p in parts if p.name == "돌림판").shape.solids()[0]
    for x, z in b.clamp_xz():
        assert table.is_inside((x, b.ATOP - b.SINK - b.SKIN / 2, z))
        assert not table.is_inside((x, b.ATOP - b.SINK + 0.5, z))
        assert not table.is_inside((x, b.ATOP - b.SINK - b.SKIN - 0.5, z))


def test_plate_bolts_and_clamp_zones() -> None:
    checks = b.screw_checks("STS3215")
    # 판 나사 M3 × 10: 몸통 속 너트에 다 물리고, 구멍 바닥에 닿지 않는다
    assert checks["판 나사가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert checks["판 나사 끝과 나사 구멍 바닥 사이"] >= 0.5
    assert checks["판 나사 머리 자리 위에 남은 판의 살"] >= 1.5
    # 머리와 와셔가 판 밑면 밖으로 나오지 않는다 (책상에 닿지 않는다)
    assert b.PLATE_RECESS >= 1.65 + b.WASHER_T
    # 가진 클램프(가장 넓게 60 mm)가 책상(40 mm)과 날개를 같이 문다: 조일 틈이 5 mm 넘게 남는다
    assert checks["클램프가 벌어지는 틈에서 책상과 날개를 뺀 나머지"] >= 5.0
    # 판 나사는 서보 먼 끝 자리(|z| 12.7 안)와 베어링 구멍(반지름 21) 밖의 두꺼운 벽에 있다
    for x, z in b.plate_xz():
        assert abs(z) - 1.6 > j.NECK_IN + 3.0
        assert (x * x + z * z) ** 0.5 - 1.6 > j.R_BORE + 3.0
    # 클램프가 무는 자리는 몸통을 비키고 날개 안에 있다 (가장자리에서 25 mm 들어온 곳까지)
    x_in = -b.EDGE + b.CLAMP_IN + b.CLAMP_HW
    near = (x_in**2 + (b.CLAMP_Z - b.CLAMP_HW) ** 2) ** 0.5
    assert near > b.R_CORE + b.SEAT_GAP + 2.0
    assert b.WING_IN <= b.CLAMP_Z - b.CLAMP_HW
    assert b.CLAMP_Z + b.CLAMP_HW <= b.ACC_Z - 3.2 - 2.0  # 부속 자리의 너트와 2 mm 넘게 떨어진다
    assert x_in <= b.WING_X
    # 판이 책상 가장자리 밖으로 나가지 않는다
    assert b.EDGE >= b.R_CORE + b.SEAT_GAP + b.RING_W


@pytest.mark.slow
def test_plate_seats_the_core(parts: list[j.Part]) -> None:
    core = next(p for p in parts if p.name == "베이스 몸통")
    plate = next(p for p in parts if p.name == "책상에 무는 판")
    # 몸통이 판의 바닥에 앉는다 (닿는다). 옆으로는 턱과 0.2 mm 틈이다
    assert core.shape.distance_to(plate.shape) < 0.01
    solid = plate.shape.solids()[0]
    for x, z in b.plate_xz():  # 머리 자리와 나사 구멍 사이에 막 한 층이 있다
        assert solid.is_inside((x, b.Y_DESK + b.PLATE_RECESS + b.SKIN / 2, z))
        assert not solid.is_inside((x, b.Y_DESK + b.PLATE_RECESS - 0.5, z))
    # 선이 나가는 길이 턱에 뚫려 있다
    assert not solid.is_inside((-(b.R_CORE + 3.0), b.Y_FLOOR + 3.0, 0.0))


def test_wires_wrap_at_the_d405_bend_radius() -> None:
    # 선이 감기는 면의 반지름: 지름 3.2 mm D405 선의 가운데가 40 mm 넘는 곳을 돈다 (design.md의 D405 케이블)
    assert b.R_TABLE + 1.6 >= 40.0
    # 감기는 면의 높이 (몸통과의 틈 + 돌림판 테 + J1 몸통 바닥판)에 D405 선과 서보 선 둘이 나란히 든다
    assert j.GAP + b.BRIM_T + j.EAR_T >= 4.5 + 2 * 3.0


@pytest.mark.slow
def test_arm_model_uses_the_j1_stack_and_masses(parts: list[j.Part]) -> None:
    # 팔 전체 모델의 돌림판 높이와 J1 쪽 무게가 이 CAD의 값이다
    assert ur_arm.TABLE_TOP == pytest.approx(b.TOP_H * MM)
    assert ur_arm.shoulder_room(ur_arm.Concept()) > 2.0 * MM
    printed = {
        name: printed_mass_g(shape, up, preset)
        for name, (shape, up, _color, preset) in b.printed().items()
    }
    moving = sum(p.mass_g for p in parts if p.density > 0 and p.moving)
    table = sum(printed[f"base_{n}"] for n in ("turntable", "hub", "spacer")) + moving + HORN_MASS_G
    assert table == pytest.approx(ur_arm.J1_TABLE_MASS * 1000, abs=2.0)
    house = printed_mass_g(sh.house(), (0.0, -1.0, 0.0))
    assert printed["j1_body"] - house == pytest.approx(ur_arm.J1_FOOT_MASS * 1000, abs=2.0)
