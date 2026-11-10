"""관절 하나 시험(J3) 부품: 출력할 수 있는 형상인지, 관절 범위에서 부딪히지 않는지, 나사와 핀이 맞는지."""

from __future__ import annotations

import math

import numpy as np
import pytest
from build123d import GeomType

from horilab_design.parts import elbow as e
from horilab_design.parts import joint_test as jt
from horilab_design.parts.shapes import box
from horilab_design.values import load_arm

BED = 256.0  # 출력 범위 [mm]


@pytest.mark.slow
def test_parts_are_single_valid_solids_that_fit_the_bed() -> None:
    for spec in jt.PARTS:
        part = jt.placed_for_print(spec)
        assert part.is_valid, spec.name
        assert len(part.solids()) == 1, spec.name
        bb = part.bounding_box()
        assert bb.min.Z == pytest.approx(0.0, abs=1e-6), spec.name
        assert max(bb.size.X, bb.size.Y, bb.size.Z) < BED, spec.name


@pytest.mark.slow
def test_no_clash_over_the_j3_range() -> None:
    # 관절 범위 양 끝에서 4° 더 돌려도(반지름 30 mm에서 약 2 mm) 자식 부품이 부모 부품, 서보, 커넥터에 닿지 않는다.
    lower, upper = load_arm().joints["j3"].range_deg
    angles = [float(q) for q in np.arange(lower - 4.0, upper + 4.0 + 1e-9, 4.0)]
    hits = {q: h for q, h in jt.clashes(angles).items() if h}
    assert not hits, hits


@pytest.mark.slow
def test_zero_pin_goes_in_at_zero_and_can_be_pulled_out() -> None:
    assert all(v == pytest.approx(0.0, abs=1e-6) for v in jt.pin_interference().values())
    assert jt.pin_grip() >= 4.0


def test_horn_screws_end_inside_the_horns() -> None:
    # 나사 끝이 혼을 뚫고 나가 서보 몸에 닿으면 안 되고, 혼에 1.5 mm 이상은 물려야 한다.
    depths = jt.screw_depths()
    out, back = depths["출력 혼 (M3 × 10)"], depths["뒷면 혼 (M3 × 10)"]
    assert 1.5 <= out <= e.HORN_T - 0.5
    assert 1.5 <= back <= e.BACK_HORN_T - 0.5


def test_gauge_set_screws_grip_before_the_head_lands() -> None:
    # 누름 나사(M3 × 10)가 PLA에 5 mm 넘게 물리고, 머리가 면에 닿기 전(1 mm 넘게 남아서) 줄기를 누른다.
    fit = jt.gauge_set_screws()
    for name in ("세로 집게", "옆 집게"):
        assert fit[f"{name}: 물리는 길이"] >= 5.0
        assert fit[f"{name}: 머리와 면 사이"] >= 1.0


def test_gauge_needle_can_follow_the_link() -> None:
    # 바늘은 10 mm 움직인다. 세로 집게는 바늘을 5 mm 넘게 눌러 둘 수 있어서, 서보가 시험 짐에 2~3 mm 처져도
    # (다른 사람이 잰 STS3250 값으로 어림, docs/joint-test.md) 바늘이 링크를 따라간다. 옆 집게는 목이 닿을 때
    # 바늘이 2 mm쯤 눌린다.
    fit = jt.gauge_preload()
    for name, need in (("세로 집게", 5.0), ("옆 집게", 2.0)):
        lo, hi = fit[name]
        assert lo <= 1.0, name
        assert need <= hi <= 9.0, name


@pytest.mark.slow
def test_gauge_bracket_has_no_layer_starting_in_the_air() -> None:
    # 출력 방향(관절 쪽 옆면이 바닥)에서 바닥이 아닌 곳에 아래를 향한 평평한 면이 없다. 모든 부분이 바닥에서 곧게
    # 올라오고, 두 줄기 구멍은 위로 45° 지붕을 낸다.
    spec = next(s for s in jt.PARTS if s.name == "gauge_bracket")
    part = jt.placed_for_print(spec)
    downs = [
        f
        for f in part.faces()
        if f.geom_type == GeomType.PLANE and f.normal_at().Z < -0.99 and f.center().Z > 0.05
    ]
    assert not downs, [d.center() for d in downs]


@pytest.mark.slow
def test_rig_parts_do_not_overlap_and_the_load_hangs_free() -> None:
    # 나사로 고정하는 부품끼리(고리와 서보 집 등)도 겹치지 않고, 매단 물병은 게이지 받침과 받침 기둥에 닿지 않는다.
    gaps = jt.rig_gaps()
    for k, v in gaps.items():
        if k.startswith("짐 아래"):
            assert v >= 25.0, k
        else:
            assert v == pytest.approx(0.0, abs=0.01), (k, v)


@pytest.mark.slow
def test_gauge_foot_leaves_room_for_the_clamp() -> None:
    # 클램프 패드(지름 20 mm, 가장자리에서 2 cm)와 클램프 위 팔(발 위 30 mm, 폭 16 mm)이 지나는 자리에 받침의 다른
    # 부분이 없다.
    y, z = jt.GAUGE_CLAMP_FOOT_PAD
    x_foot = jt.X_DESK + jt.GAUGE_FOOT_T
    reach = box(x_foot + 0.01, x_foot + 30.0, jt.EDGE_Y - 15.0, y + 12.0, z - 10.0, z + 10.0)
    assert (reach & jt.gauge_bracket()).volume == pytest.approx(0.0, abs=0.01)


def test_spigot_slides_into_the_measured_collar() -> None:
    # 서보 집 꽂이는 실제로 뽑힌 받침 기둥 고리(실측 안지름)에 손으로 들어가고, 고리를 죄면 틈이 닫힌다.
    # 고리를 가르는 틈만큼 둘레가 줄어들 수 있으므로 지름은 그 1/π까지 줄어든다.
    gap = e.FITS.tube_bore - 2 * e.SPIGOT_R
    assert 0.15 <= gap <= 0.4
    assert gap < jt.COLLAR_SLIT / math.pi


def test_ring_screw_passes_the_nut() -> None:
    assert jt.ring_screw_past_nut() >= 0.5


def test_back_hub_clears_the_connectors() -> None:
    # 뒷면 원판의 허브는 어느 각에서나 축 둘레 원이라, 반지름이 커넥터 자리(축에서 10 mm부터)보다 작아야 한다.
    assert e.HUB_R < abs(e.CONNECTOR_X[1])


def test_discs_leave_moving_gaps_to_the_house_and_ring() -> None:
    assert e.Y_DISC_OUT - e.Y_OUT >= e.MOVING_GAP - 0.05
    ring_back = e.Y_DISC_BACK + e.MOVING_GAP
    assert ring_back - e.Y_DISC_BACK >= e.MOVING_GAP - 1e-9
    assert e.Y_DISC_BACK <= e.CONNECTOR_Y - 1.0  # 꽂은 커넥터와 원판 사이 1 mm 이상


def test_printed_thicknesses_follow_the_layer_height() -> None:
    # 높이 방향으로 맞춰야 하는 두께가 층(0.2 mm)의 배수가 아니면 슬라이서가 반올림해서 어긋난다.
    for value in (
        e.DISC_T,
        e.OUT_HUB,
        e.BACK_HUB,
        e.OUT_SCREW_HEAD,
        e.BACK_SCREW_HEAD,
        e.T_OUT,
        e.M2_HEAD_DEPTH,
        e.OUT_CENTER_DEPTH,
        e.SPIGOT_CHAMFER,
    ):
        layers = value / 0.2
        assert layers == pytest.approx(round(layers), abs=1e-9), value
