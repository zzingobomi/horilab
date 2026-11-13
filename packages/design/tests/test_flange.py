"""J6 플랜지(플랜지 원판, 도구 어댑터 판): 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 핀이 맞는지."""

from __future__ import annotations

import math

import pytest

from horilab_design.parts import flange as f
from horilab_design.parts import joint as j
from horilab_design.parts import wrist
from horilab_design.parts.shapes import far_apart
from horilab_design.printcheck import overhangs


@pytest.fixture(scope="module")
def parts() -> list[j.Part]:
    return f.assembly("STS3215")


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_without_steep_overhang() -> None:
    for name, (shape, up, _color, _preset) in f.printed().items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        over = overhangs(shape, up)
        assert over.total - over.flat < 40.0, (name, over.total, over.flat)


@pytest.mark.slow
def test_flange_parts_do_not_overlap(parts: list[j.Part]) -> None:
    solids = [p for p in parts if "핀" not in p.name and "손잡이" not in p.name]
    assert j.overlaps(solids) == {}


@pytest.mark.slow
def test_flange_turns_freely_and_stops_on_the_web(parts: list[j.Part]) -> None:
    for q in (0.0, 90.0, j.SOFT_DEG, -90.0, -j.SOFT_DEG):
        gaps = f.clearances(parts, q)
        assert min(gaps.values()) > 0.5, (q, gaps)
    for q in (j.STOP_DEG, -j.STOP_DEG):
        gaps = f.clearances(parts, q)
        assert gaps[("플랜지 원판", "손목 몸통")] < 0.05, (q, gaps)


@pytest.mark.slow
def test_flange_clears_the_stop_ear_of_the_joint_before(parts: list[j.Part]) -> None:
    # 손목 2 몸통이 앉은 J5 출력 원판의 멈춤 귀는 플랜지 쪽을 보고, 끝이 플랜지 면과 같은 높이까지 온다.
    # 플랜지 원판의 귀 둘과 어댑터 판, 손나사는 그 귀와 2 mm 넘게 떨어져 돈다
    before = wrist.parent_disc()
    ear = next(p for p in before if p.name == "앞 관절의 출력 원판")
    tip = ear.shape.bounding_box().max.Y
    assert tip == pytest.approx(j.ATOP, abs=0.05)
    # 바깥 상자끼리 3 mm 넘게 떨어진 쌍은 틈이 2 mm를 넘으니 재지 않는다 (지금은 모든 쌍이 그렇다)
    gaps = [
        shape.distance_to(b.shape)
        for q in range(-160, 161, 10)
        for a in parts
        if a.moving
        for shape in [j.turned(a.shape, float(q))]
        for b in before
        if not far_apart(shape, b.shape, 2.0)
    ]
    assert all(d >= 2.0 for d in gaps), gaps


@pytest.mark.slow
def test_pins_sit_in_their_holes(parts: list[j.Part]) -> None:
    body = next(p for p in parts if p.name == "손목 몸통")
    disc = next(p for p in parts if p.name == "플랜지 원판")
    plate = next(p for p in parts if p.name == "도구 어댑터 판")
    zero = next(p for p in parts if p.name.startswith("영점 핀"))
    # 영점 핀(지름 3.0)이 원판 귀의 구멍과 손목 몸통 살의 구멍(3.35) 속에 있다
    assert 0.1 < zero.shape.distance_to(body.shape) < 0.25
    assert 0.1 < zero.shape.distance_to(disc.shape) < 0.25
    for pin in (p for p in parts if p.name.startswith("도구 핀")):
        # 도구 핀은 어댑터 판의 둥근 구멍과 긴 구멍 속에 있다 (틈 0.15 ~ 0.18)
        assert 0.1 < pin.shape.distance_to(plate.shape) < 0.25


def test_tool_pins_and_screws_fit_the_disc() -> None:
    # 도구 핀이 저널 끝을 뚫지 않고, 어댑터 판 윗면 위로 나오지 않는다
    assert j.ATOP - f.TOOL_PIN_DEPTH > j.B0 + j.CLAMP_GAP
    assert f.TOOL_PIN_UP <= f.ADAPTER_T
    # 도구 핀 구멍이 혼 볼트 머리(반지름 2.85)와 저널 바깥 면에서 떨어져 있다
    for px, pz in f.TOOL_PINS:
        for x, z in j.bolt_xz():
            assert math.hypot(x - px, z - pz) - 2.85 - f.PIN_PRESS / 2 > 0.8
        # 원판 밑면의 너트 자리(맞변 5.6)와도 떨어져 있다
        for x, z in j.RIM:
            assert math.hypot(x - px, z - pz) - 3.3 - f.PIN_PRESS / 2 > 2.0
    assert j.R_JOURNAL - f.TOOL_PIN_R - f.PIN_PRESS / 2 > 1.5
    # 손나사 M3 × 10이 원판 밑면의 너트에 다 물리고, 원통 앞 끝 쪽 틈으로 0.2 넘게 나오지 않는다
    tip = j.ATOP + f.ADAPTER_T - j.BODY_SCREW
    assert j.SEAM - 0.2 <= tip <= j.SEAM + 0.3
    # 어댑터 판이 혼 볼트 머리(높이 1.65)를 비킨다
    assert f.HEAD_POCKET[1] > 1.65 and f.HEAD_POCKET[0] > 2.85
    # 영점 핀이 귀 위로 3 mm 넘게 나와 손으로 잡힌다
    assert (j.TUB_Y - 1.0 + 20.0) - j.ATOP > 3.0
