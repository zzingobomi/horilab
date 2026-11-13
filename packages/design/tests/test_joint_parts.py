"""STS 관절, 손목 몸통, 링크 부품: 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 멈춤과 영점 핀이 맞는지."""

from __future__ import annotations

import pytest

from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
from horilab_design.printcheck import overhangs
from horilab_design.values import ServoModel

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


def test_screw_lengths_match_the_stack() -> None:
    # 혼 볼트 M3 × 20: 머리는 출력 원판 윗면, 끝은 혼(윗면 SEAT, 두께 약 2.6)에 2.4 ~ 3.0 들어간다.
    # 더 들어가면 혼 밑의 서보 앞면(틈 0.3)에 닿는다
    engaged = 20.0 - (j.ATOP - j.SEAT)
    assert 2.4 <= engaged <= 2.9
    # 자식 몸통 나사 M3 × 10: 너트를 끝까지 지나가고, 원통 앞 끝 쪽 틈으로 0.2 넘게 나오지 않는다
    tip = j.ATOP + j.EAR_T - j.BODY_SCREW
    assert j.SEAM - 0.2 <= tip <= j.SEAM + 0.3
    # 영점 핀 20 mm: 자식 귀 위로 3 나오고, 살 속에 4 넘게 들어간다
    assert j.FRONT - (j.ATOP + j.EAR_T + 3.0 - 20.0) > 4.0
    # 멈춤 나사 끝이 홈 바닥에 닿지 않는다
    assert (j.ATOP - 1.8 - 10.0) - j.CH_Y > 0.8
