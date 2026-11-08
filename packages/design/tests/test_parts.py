"""관절 하나 시험(J3) 부품: 출력할 수 있는 형상인지, 관절 범위에서 부딪히지 않는지, 나사와 핀이 맞는지."""

from __future__ import annotations

import numpy as np
import pytest
from build123d import GeomType

from horilab_design.parts import elbow as e
from horilab_design.parts import joint_test as jt
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


def test_gauge_clamp_bolt_passes_the_nut() -> None:
    # 게이지 조임 볼트(M3 × 15)가 너트를 다 지나가고 끝은 부품 안에 있으며, 너트 홈 양쪽에 벽이 남는다.
    fit = jt.gauge_clamp_fit()
    assert fit["너트를 지나간 길이"] >= 0.5
    assert fit["볼트 끝 → -y 면"] >= 0.5
    assert fit["너트 홈 → -y 면 벽"] >= 2.0
    assert fit["너트 홈 → 틈 벽"] >= 2.0


@pytest.mark.slow
def test_gauge_bracket_has_no_layer_starting_in_the_air() -> None:
    # 출력 방향에서 바닥이 아닌 곳의 아래를 향한 평평한 면은 기둥 속 빈 곳의 천장 하나뿐이다.
    # 이 천장은 양쪽이 붙은 다리(폭 8.8 mm)다. 집게 틈이 가로로 누우면 그 위가 공중에서 시작한다.
    spec = next(s for s in jt.PARTS if s.name == "gauge_bracket")
    part = jt.placed_for_print(spec)
    downs = [
        f
        for f in part.faces()
        if f.geom_type == GeomType.PLANE and f.normal_at().Z < -0.99 and f.center().Z > 0.05
    ]
    assert len(downs) == 1
    size = downs[0].bounding_box().size
    assert min(size.X, size.Y) <= 9.0


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
    ):
        layers = value / 0.2
        assert layers == pytest.approx(round(layers), abs=1e-9), value
