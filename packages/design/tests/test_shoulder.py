"""어깨 관절(ST-3120) 부품: 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 멈춤과 영점 핀이 맞는지."""

from __future__ import annotations

import pytest

from horilab_design.parts import joint as j
from horilab_design.parts import links
from horilab_design.parts import shoulder as s
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.studies import clamp_plate, ur_arm
from horilab_design.studies.packaging import MM

BED = 256.0  # 출력 범위 [mm]
HORN_MASS_G = 6.9  # 혼 FT110M과 가운데 나사 (도면 부피로 어림. 서보가 오면 잰다)


@pytest.fixture(scope="module")
def parts() -> list[j.Part]:
    length, tube_r, foot = links.LINKS["upper_arm"]
    return s.assembly(links.link(length, tube_r, foot), length)


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_without_steep_overhang() -> None:
    for name, (shape, up, _color, _preset) in s.printed().items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        bb = shape.bounding_box()
        assert max(bb.size.X, bb.size.Y, bb.size.Z) < BED, name
        # 바닥에 닿지 않고 아래를 보는 면은 양쪽이 붙은 다리와 작은 가로 구멍 윗변뿐이어야 한다
        over = overhangs(shape, up)
        assert over.total - over.flat < 80.0, (name, over.total, over.flat)


@pytest.mark.slow
def test_shoulder_parts_do_not_overlap(parts: list[j.Part]) -> None:
    assert j.overlaps([p for p in parts if "핀" not in p.name]) == {}


@pytest.mark.slow
def test_upper_arm_turns_freely_and_stops_on_the_web(parts: list[j.Part]) -> None:
    # 프로그램 한계까지는 도는 부품이 어디에도 닿지 않고, 멈춤 각에서 기둥이 살에 닿는다
    for q in (0.0, 90.0, j.SOFT_DEG, -90.0, -j.SOFT_DEG):
        gaps = s.clearances(parts, q)
        assert min(gaps.values()) > 0.5, (q, gaps)
    for q in (j.STOP_DEG, -j.STOP_DEG):
        gaps = s.clearances(parts, q)
        assert gaps[("출력 원판", "어깨 집")] < 0.05, (q, gaps)


@pytest.mark.slow
def test_zero_pin_sits_in_both_holes(parts: list[j.Part]) -> None:
    body = next(p for p in parts if p.name == "어깨 집")
    arm = next(p for p in parts if p.name == "위팔")
    pin = next(p for p in parts if "핀" in p.name)
    # 핀(지름 3.0)이 구멍(3.35) 속에 있으면 벽과의 틈이 0.15 ~ 0.18이다
    assert 0.1 < pin.shape.distance_to(body.shape) < 0.25
    assert 0.1 < pin.shape.distance_to(arm.shape) < 0.25


def test_screw_lengths_match_the_stack() -> None:
    # 혼 볼트 M3 × 20: 혼의 테(두께 2.5)를 다 지나고 0.3 넘게 나오지 않는다. 테 뒤는 허브 둘레의 빈자리다
    engaged = 20.0 - (s.ATOP - s.SEAT)
    assert s.HORN_T <= engaged <= s.HORN_T + 0.3
    # 위팔 나사 M3 × 10: 너트를 끝까지 지나가고, 원통 앞 끝 쪽 틈으로 0.2 넘게 나오지 않는다
    tip = s.ATOP + j.EAR_T - j.BODY_SCREW
    assert s.SEAM - 0.2 <= tip <= s.SEAM + 0.3
    # 영점 핀 20 mm: 위팔 귀 위로 3 나오고, 살 속에 4 넘게 들어간다
    assert s.FRONT - (s.ATOP + j.EAR_T + 3.0 - 20.0) > 4.0
    # 멈춤 나사 끝이 홈 바닥에 닿지 않는다
    assert (s.ATOP - 1.8 - 10.0) - s.CH_Y > 0.8
    # 덮개 나사 M3 × 12 끝이 서보 받침의 기둥(판 위로 5.5) 안에서 멈춘다
    tip = s.BACK_OUT + 1.2 + 12.0
    assert -s.HT < tip < -s.HT + 5.5
    # 혼 가운데 나사 머리가 출력 원판의 저널 끝에 닿지 않는다
    assert (s.B0 + j.CLAMP_GAP) - (s.HORN_TOP + s.HORN_SCREW[1]) > 0.8


def test_side_plug_fits_inside_the_neck_but_cannot_be_plugged_there() -> None:
    # 옆면 커넥터에 꽂은 플러그와 꺾인 선이 목 안쪽 벽 안에 든다
    assert s.HW + s.WIRE_OUT < s.NECK_IN
    # 목 속 옆 틈이 플러그 길이(9.58)보다 좁다: 서보를 넣기 전에 꽂는다 (조립 순서)
    assert s.NECK_IN - s.HW < 9.58


def test_hub_plate_spring_stays_well_below_the_bending_strength() -> None:
    for plate in clamp_plate.plates():
        force_lo, _ = clamp_plate.press(plate, clamp_plate.E_PLA[0])
        _, stress_hi = clamp_plate.press(plate, clamp_plate.E_PLA[1])
        assert stress_hi < 0.55 * clamp_plate.BEND_STRENGTH, plate.name
        assert force_lo > 60.0, plate.name


def test_shoulder_keeps_the_skeleton_plane() -> None:
    c = ur_arm.Concept()
    st = c.st("ST-3120")
    assert c.ya2 == pytest.approx(ur_arm.SHOULDER_PLANE)
    # 어깨 원통(뒷덮개 ~ 출력 원판 윗면)이 J1 축을 가운데로 1 mm 안에서 대칭이다
    assert abs((c.c2 + st.back) + (c.c2 + st.atop)) < 1.0 * MM


@pytest.mark.slow
def test_shoulder_mass_constants_match_the_cad(parts: list[j.Part]) -> None:
    printed = {
        name: printed_mass_g(shape, up, preset)
        for name, (shape, up, _color, preset) in s.printed().items()
    }
    screws = [p for p in parts if p.density > 0 and "핀" not in p.name]
    static = sum(p.mass_g for p in screws if not p.moving)
    moving = sum(p.mass_g for p in screws if p.moving)
    # 집과 뒷덮개는 어깨만 따로 본 모양이다 (뽑는 J1 몸통에서 더 든 몫은 ur_arm.J1_FOOT_MASS)
    house = printed_mass_g(s.house(), (0.0, -1.0, 0.0))
    cover = printed_mass_g(s.back_cover(), (0.0, 1.0, 0.0), "standard")
    house += printed["shoulder_retainer"] + printed["shoulder_carrier"] + cover + static
    assert house == pytest.approx(ur_arm.SHOULDER_HOUSE_MASS * 1000, abs=2.0)
    out = printed["shoulder_out_disc"] + printed["shoulder_hub_plate"] + moving + HORN_MASS_G
    assert out == pytest.approx(ur_arm.SHOULDER_OUT_MASS * 1000, abs=2.0)
