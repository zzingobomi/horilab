"""STS 관절, 손목 몸통, 링크 부품: 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 멈춤과 영점 핀이 맞는지."""

from __future__ import annotations

import pytest

from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
from horilab_design.printcheck import overhangs
from horilab_design.values import ServoModel, load_measured

BED = 256.0  # 출력 범위 [mm]


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_that_fit_the_bed() -> None:
    for name, (shape, _up, _color, _preset) in wrist.printed().items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        bb = shape.bounding_box()
        assert max(bb.size.X, bb.size.Y, bb.size.Z) < BED, name
    for name, (length, tube_r, foot) in links.LINKS.items():
        part = links.link(length, tube_r, foot)
        assert part.is_valid, name
        assert len(part.solids()) == 1, name
        assert part.bounding_box().size.Y < BED, name
        assert links.fits_bed(part)[0], name


@pytest.mark.slow
def test_printed_parts_have_no_steep_overhang() -> None:
    # 바닥에 닿지 않고 아래를 보는 면은 양쪽이 붙은 다리(거의 수평)와 작은 가로 구멍 윗변뿐이어야 한다.
    # 비스듬히(45° ~ 70°) 걸친 면은 서포트 없이 처진다
    parts = {name: (shape, up) for name, (shape, up, _c, _p) in wrist.printed().items()}
    for name, (length, tube_r, foot) in links.LINKS.items():
        parts[name] = (links.link(length, tube_r, foot), (0.0, -1.0, 0.0))
    for name, (shape, up) in parts.items():
        over = overhangs(shape, up)
        # 45°로 깎은 면은 삼각형으로 나눌 때 45°를 조금 넘는 조각이 생긴다 (누름 틀의 원뿔 면 70 mm²)
        assert over.total - over.flat < 80.0, (name, over.total, over.flat)


@pytest.mark.slow
@pytest.mark.parametrize("model", j.STS_MODELS)
def test_first_joint_parts_do_not_overlap(model: ServoModel) -> None:
    parts = [p for p in wrist.first_joint(model) if "핀" not in p.name]
    assert j.overlaps(parts) == {}


@pytest.mark.slow
def test_child_turns_freely_and_stops_on_the_web() -> None:
    # 프로그램 한계까지는 도는 부품이 어디에도 닿지 않고, 멈춤 각에서 기둥이 살에 닿는다
    for q in (0.0, 90.0, j.SOFT_DEG, -90.0, -j.SOFT_DEG):
        gaps = wrist.clearances("STS3215", q)
        assert min(gaps.values()) > 0.5, (q, gaps)
    for q in (j.STOP_DEG, -j.STOP_DEG):
        gaps = wrist.clearances("STS3215", q)
        assert gaps[("출력 원판", "손목 몸통 (부모)")] < 0.05, (q, gaps)


@pytest.mark.slow
def test_zero_pin_sits_in_both_holes_at_the_pin_pose() -> None:
    body = wrist.wrist_body()
    pin = next(p for p in wrist.child_hardware() if "핀" in p.name)
    child = wrist.as_child(body, wrist.PIN_POSE_DEG)
    # 핀(지름 3.0)이 구멍(3.30, 3.35) 속에 있으면 벽과의 틈이 0.15 ~ 0.18이다
    assert 0.1 < pin.shape.distance_to(body) < 0.25
    assert 0.1 < pin.shape.distance_to(child) < 0.25


def test_servo_sits_where_the_measured_horn_height_puts_it() -> None:
    measured = load_measured()
    depth = measured.servo_depth
    assert depth is not None
    # 혼 윗면을 낮은 뒷면(두께 29)에서 잰 것으로 읽으면, 혼이 앞면 라벨 판보다 높은 양이 깊이 막대로 잰 값과 맞는다
    low, high = depth.horn_above_label_mm
    for model in j.STS_MODELS:
        assert low - 0.15 <= j.HORN_TOP[model] - j.LABEL <= high + 0.15, model
    # 도면: 혼 윗면은 낮은 앞면에서 약 4.3 (축 3.4 + 혼의 남는 살 0.9), 보스 끝은 낮은 뒷면에서 4.1
    for model in j.STS_MODELS:
        assert j.HORN_TOP[model] - j.F29 == pytest.approx(4.3, abs=0.15), model
    boss = measured.servo_body["STS3215"].horn_to_boss_mm
    assert boss is not None
    assert j.HORN_TOP["STS3215"] - boss == pytest.approx(j.BOSS_TIP, abs=0.05)
    # 몸통은 두께의 가운데가 원점이고, 두께 32인 면이 낮은 면보다 앞뒤로 1.5씩 높다
    assert (j.L0, j.F29, j.L1, j.F32) == (-14.5, 14.5, -16.0, 16.0)
    # 관절의 길이는 혼 윗면 ~ 꽂아 눕힌 선(39.00)에서 나와서 그대로다
    assert j.ATOP - j.BACK_OUT == pytest.approx(58.9, abs=0.01)


def test_horn_screw_head_and_back_side_fit() -> None:
    # 혼 가운데 나사 머리(지름 5.15, 혼 윗면 위로 2.0까지)가 허브 판 속에 들고, 조인 뒤의 저널 끝과 0.5 넘게 뜬다
    assert (j.HEAD_D, j.HEAD_H) == (5.15, 2.0)
    assert j.B0 - (j.SEAT + j.HEAD_H) >= 0.5
    assert 3.6 - j.HEAD_D / 2 >= 0.8  # 허브 판 가운데 구멍과 머리 사이
    # 뒷면: 꽂은 플러그는 커넥터가 있는 뒷면에서 2.6 나오고, 눕힌 선의 윗면은 잰 값이다
    assert j.L1 - j.PLUG_TOP == pytest.approx(2.62, abs=0.01)
    assert j.PLUG_TOP - j.WIRE_TOP == pytest.approx(j.WIRE_T)
    # 서보 받침: 먼 가로대의 선 자리는 눕힌 선 밑보다 낮고, 그 밑에 판이 2 mm 넘게 남는다
    groove = j.L1 - j.PLATE_T + j.WIRE_GROOVE
    assert groove - j.PLUG_TOP >= 0.3 and j.PLATE_T - j.WIRE_GROOVE >= 2.0
    # 서보 뒷면 나사 M2 × 4: 머리(높이 2.0까지)는 판 속 자리에 들어 뒷덮개와 1.5 넘게 뜨고, 서보에 2.5 ~ 3.5 물린다
    head_top = j.L1 - j.PLATE_T + j.M2_SINK - 2.0
    assert head_top - j.BACK_IN >= 1.5
    assert j.PLATE_T - j.M2_SINK >= 1.2 - 1e-9
    assert 2.5 <= j.M2_SCREW - (j.PLATE_T - j.M2_SINK) <= 3.5
    # 판 위를 지나는 선(두께 1.5)과 뒷덮개 사이
    assert (j.L1 - j.PLATE_T - j.WIRE_T) - j.BACK_IN >= 0.5
    # 뒷덮개 나사 M3 × 12의 끝이 받침의 기둥 속에서 끝난다
    tip = j.BACK_OUT + 1.2 + j.COVER_SCREW
    assert j.L1 + 6.0 < tip < j.L1 + 7.0


def test_screw_lengths_match_the_stack() -> None:
    # 혼 볼트 M3 × 20: 머리는 출력 원판 윗면, 끝은 혼(윗면 SEAT, 테 두께 2.5)에 2.4 ~ 2.9 들어간다.
    # 혼의 테 뒤로 서보 앞면까지는 1.7 mm가 비어 있다
    engaged = 20.0 - (j.ATOP - j.SEAT)
    assert 2.4 <= engaged <= 2.9
    for model in j.STS_MODELS:
        assert (j.SEAT - engaged) - (j.F29 + j.servo_dy(model)) > 1.0, model
    # 자식 몸통 나사 M3 × 10: 너트를 끝까지 지나가고, 원통 앞 끝 쪽 틈으로 0.2 넘게 나오지 않는다
    tip = j.ATOP + j.EAR_T - j.BODY_SCREW
    assert j.SEAM - 0.2 <= tip <= j.SEAM + 0.3
    # 영점 핀 20 mm: 자식 귀 위로 3 나오고, 살 속에 4 넘게 들어간다
    assert j.FRONT - (j.ATOP + j.EAR_T + 3.0 - 20.0) > 4.0
    # 멈춤 나사 끝이 홈 바닥에 닿지 않는다
    assert (j.ATOP - 1.8 - 10.0) - j.CH_Y > 0.8
