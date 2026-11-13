"""평행 그리퍼: 뽑을 수 있는 형상인지, 겹치지 않는지, 이가 물리는지, 박스 구멍에 맞는지, 무게와 나사."""

from __future__ import annotations

import math

import pytest

from horilab_design.parts import flange
from horilab_design.parts import gripper as g
from horilab_design.parts import joint as j
from horilab_design.printcheck import overhangs


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_without_steep_overhang() -> None:
    for name, (shape, up, _color, _preset, _n) in g.printed().items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        over = overhangs(shape, up)
        # 걸치는 면은 양쪽이 붙은 짧은 다리뿐이다 (나사 머리 자리, 너트 홈과 창의 윗변)
        assert over.total - over.flat < 15.0, (name, over.total, over.flat)
        assert over.largest < 40.0, (name, over.largest)


@pytest.mark.slow
@pytest.mark.parametrize("opening", [0.0, 2 * g.grasp_x(), 30.0, g.OPEN_MAX])
def test_nothing_overlaps_at_any_opening(opening: float) -> None:
    # 플랜지 원판과 손목 몸통까지 넣고 본다. 이가 서로 파고들면 여기서 걸린다
    assert g.overlaps(g.assembly(opening)) == {}


@pytest.mark.slow
@pytest.mark.parametrize("opening", [0.0, 17.0, g.OPEN_MAX])
def test_sliders_run_free_and_teeth_mesh(opening: float) -> None:
    gaps = g.slide_gaps(opening)
    # 받침은 통로의 위아래와 옆벽에서 0.1 mm씩 뜬다
    assert gaps[("손가락 받침 1", "랙 집")] == pytest.approx(g.SLIDE_GAP, abs=0.02)
    assert gaps[("손가락 받침 1", "바닥판")] == pytest.approx(g.SLIDE_GAP, abs=0.02)
    # 이와 이 사이에 틈이 남되(파고들지 않고) 떨어져 있지 않다
    for k in (1, 2):
        assert 0.03 < gaps[(f"손가락 받침 {k}", "피니언")] < 0.15, gaps
    # 통로 속 나사 머리, 서보 앞면, 바닥판 구멍과 닿지 않는다
    assert gaps[("손가락 받침 1", "바닥판 나사 M3x8 1")] > 0.8
    assert gaps[("손가락 받침 2", "바닥판 나사 M3x8 2")] > 0.8
    assert gaps[("피니언", "바닥판")] > 0.25
    assert gaps[("피니언", "그리퍼 서보")] > 0.5
    assert gaps[("피니언", "바닥판 나사 M3x8 1")] > 1.5
    assert gaps[("손가락 1", "랙 집")] == pytest.approx(g.SHOULDER_GAP, abs=0.05)
    assert gaps[("그리퍼 플러그", "그리퍼 몸통")] > 0.8


@pytest.mark.slow
def test_servo_is_clamped_between_frame_and_floor() -> None:
    by = {p.name: p.shape for p in g.gripper_parts(0.0)}
    assert by["그리퍼 서보"].distance_to(by["그리퍼 몸통"]) < 0.01
    assert by["그리퍼 서보"].distance_to(by["바닥판"]) < 0.01
    # 닫으면 손가락 안쪽 면끼리, 받침끼리 닿는다 (닫힘 쪽 멈춤)
    assert by["손가락 1"].distance_to(by["손가락 2"]) < 0.01
    assert by["손가락 받침 1"].distance_to(by["손가락 받침 2"]) < 0.01


@pytest.mark.slow
def test_frame_sits_on_the_flange_pins_and_leaves_room_for_the_knobs() -> None:
    by = {p.name: p.shape for p in g.assembly(0.0)}
    for k in (1, 2):
        # 도구 핀은 몸통 윗판의 구멍 속에 있고(틈 0.15 ~ 0.18), 손나사 머리는 윗판에 앉는다
        assert 0.1 < by[f"도구 핀 MSH3-20 {k}"].distance_to(by["그리퍼 몸통"]) < 0.25
        assert by[f"손나사 M3x10 {k}"].distance_to(by["그리퍼 몸통"]) < 0.01
        # 손잡이는 옆벽의 창에 들어 몸통과 서보에 닿지 않는다
        assert by[f"손잡이 {k}"].distance_to(by["그리퍼 몸통"]) >= 0.25
        assert by[f"손잡이 {k}"].distance_to(by["그리퍼 서보"]) >= 0.8


@pytest.mark.slow
def test_gripper_clears_the_wrist_and_camera_when_j6_turns() -> None:
    gaps = g.sweep_gaps()
    assert min(gaps.values()) >= 5.0, gaps


@pytest.mark.slow
def test_fingertips_enter_the_box_hole_and_seat_on_the_cone() -> None:
    top = g.box_top()
    gx = g.grasp_x()
    # 닫은 손가락 끝은 구멍에 1.5 mm 넘는 틈을 두고 들어간다
    assert g.tip(0.0).distance_to(top) > 1.5
    # 벌리면 턱의 끝 모서리가 원뿔에 닿는다. 그 직전에는 떠 있고, 그때 목은 구멍 입구에 닿지 않는다
    assert 0.03 < g.tip(gx - 0.1).distance_to(top) < 0.1
    assert g.tip(gx).distance_to(top) < 0.01
    assert g.BOX_LIP_R - math.hypot(gx + g.NECK_U, g.NECK_HW) > 0.15
    # 손가락 끝은 구멍 바닥에 닿지 않는다
    assert g.BOX_HOLE_DEPTH - g.TIP_LEN >= 2.0


def test_pinion_fits_the_horn_and_the_travel_fits_the_servo() -> None:
    # 혼 볼트 머리 자리(반지름 3.1)가 이뿌리 원 안에 있다
    assert (g.RP - g.DED) - (7.0 + 3.1) > 0.8
    # 이끝이 뾰족해지지 않는다 (이끝 두께가 압출 두 줄 넘는다)
    rb = g.RP * math.cos(g.PA)
    tip_half = math.pi / (2 * g.TEETH) + g._inv(g.PA) - g._inv(math.acos(rb / (g.RP + g.ADD)))
    assert 2 * tip_half * (g.RP + g.ADD) > 0.8
    # 다 벌릴 때 서보가 도는 각이 한 바퀴의 절반보다 작다 (0/4095 경계에서 멀다)
    assert g.travel()["피니언이 도는 각 [°]"] < 180.0
    # 랙 팔은 어느 열림에서도 피니언 양옆으로 이 하나 넘게 남는다
    assert g.ARM_U[0] + g.OPEN_MAX / 2 < -g.PITCH and g.ARM_U[1] > g.PITCH


def test_screws_engage_and_do_not_bottom() -> None:
    c = g.screw_checks()
    length, bite = g.pinion_screw()
    assert length in g.PINION_SCREWS and 2.0 <= bite <= 3.4
    assert c["피니언 나사 끝과 서보 앞면 사이"] >= 0.5
    assert c["피니언 나사 머리 밑의 살"] >= 3.0
    assert c["바닥판 나사가 너트(2.4)를 지나간 길이"] >= 0.3
    assert c["귀 나사가 끝벽에 물린 길이"] >= 5.0
    assert c["끝벽 나사가 끝벽에 물린 길이"] >= 5.0
    assert c["끝벽 나사 끝과 구멍 바닥 사이"] >= 1.0
    assert c["손가락 나사가 너트(2.4)를 지나간 길이"] >= 0.5
    assert c["손가락 나사 머리 밑의 판 살"] >= 2.0


def test_geometry_keeps_the_camera_view_and_the_weight_cap() -> None:
    # D405가 랙 집의 밑 모서리 너머로 본다: 밑면의 반폭이 17 mm 안이고, 가장 넓은 곳도 18.5 mm 안이다
    assert g.HOUSE_HW - g.TUB_CHAMFER <= 17.0
    assert g.HOUSE_HW <= 18.5
    # 서보 뒷면 보스(4.1 mm)가 판 속 자리에 들고, 판에 1 mm 넘는 살이 남는다
    assert g.BACK_GAP + g.BOSS_REC[1] > (j.L1 - j.BOSS_TIP) + 0.3
    assert flange.ADAPTER_T - g.BOSS_REC[1] - flange.HEAD_POCKET[1] > 1.0
    # 선을 꽂는 틈이 도구 핀과 손나사 구멍에서 떨어져 있다
    for x, z in (*flange.TOOL_PINS, *flange.TOOL_SCREWS):
        dx = max(0.0, x - g.NOTCH_X)
        dz = max(0.0, abs(z) - g.NOTCH_HW)
        assert math.hypot(dx, dz) > 4.0, (x, z)


@pytest.mark.slow
def test_camera_sees_the_work_ahead_of_the_fingers() -> None:
    # 잡기 전 높이(도구 끝점보다 6 cm 앞)에서 D405가 도구 끝점 둘레의 80% 넘게 본다 (0017, 0029).
    # 손가락을 40 mm 넘게 벌린 채 본다. 닫으면 손가락이 가운데를 가린다
    assert g.view_share(40.0) >= 0.80
    assert g.view_share(g.OPEN_MAX) >= 0.80
    assert g.view_share(0.0) < g.view_share(g.OPEN_MAX)


@pytest.mark.slow
def test_weight_stays_under_the_cap() -> None:
    m = g.mass_g()
    # 그리퍼 전체(플랜지에 앉는 판, 서보, 나사, 손나사, 선까지)가 140 g 안이다 (0017의 가정)
    assert sum(m.values()) < 140.0, m
    assert m["서보와 혼"] == pytest.approx(58.65, abs=0.01)
