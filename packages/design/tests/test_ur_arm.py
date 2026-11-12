"""UR식 관절 팔의 검토 모델: 쌓임에서 나온 옆 자리, 무게가 CAD와 맞는지, 세운 자세."""

from __future__ import annotations

import numpy as np
import pytest

from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
from horilab_design.printcheck import printed_mass_g
from horilab_design.studies import ur_arm
from horilab_design.studies.packaging import MM

UP = (0.0, -1.0, 0.0)  # 몸통은 원통 앞 끝을 바닥에 두고 뽑는다


def test_links_step_sideways_by_the_joint_stack() -> None:
    c = ur_arm.Concept()
    # 아래팔의 납작한 면은 위팔의 납작한 면에서 틈 2 + 출력 원판 테만큼 안쪽이고, 손목 1은 다시 위팔의 평면이다
    assert c.ya2 - c.yb3 == pytest.approx((j.GAP + j.BRIM_T) * MM)
    assert c.yb4 == pytest.approx(c.ya2)
    # 손목 길이: 출력 원판 윗면 + 틈 + 서보 먼 끝
    assert c.d4 - c.yb4 == pytest.approx((wrist.HEAD + j.FAR) * MM)
    assert c.d5 == pytest.approx((j.ATOP - wrist.AX_Y + wrist.HEAD + j.FAR) * MM)


def test_upright_arm_is_clear_and_within_the_mass_budget() -> None:
    info = ur_arm.check(ur_arm.Concept(), verbose=False)
    assert info["upright_hits"] == []
    # J2가 드는 무게 예산 1,050 g (그리퍼 140, 어댑터 15, D405 60 포함. 선과 형상에 없는 것은 빼고)
    assert info["j2_load"] < 1.05


def test_fit_ranges_moves_angles_into_the_joint_ranges() -> None:
    lower, upper = (
        np.radians([-165, -160, -160, 20, -160, -160]),
        np.radians([165, 160, 160, 340, 160, 160]),
    )
    q = np.radians([0.0, 10.0, 20.0, -170.0, 30.0, 170.0])
    fit = ur_arm.fit_ranges(q, lower, upper)
    assert fit is not None
    assert np.degrees(fit[3]) == pytest.approx(190.0)  # J4는 한 바퀴 더해 범위 안으로
    assert np.degrees(fit[5]) == pytest.approx(-10.0)  # 대칭 도구는 J6를 180° 돌려도 같다
    assert ur_arm.fit_ranges(np.radians([0.0, 170.0, 0.0, 180.0, 0.0, 0.0]), lower, upper) is None


@pytest.mark.slow
def test_mass_constants_match_the_cad() -> None:
    assert printed_mass_g(wrist.wrist_body(), UP) == pytest.approx(
        ur_arm.WRIST_BODY_MASS * 1000, abs=1.0
    )
    inner = sum(
        printed_mass_g(s, up, preset)
        for n, (s, up, _c, preset) in wrist.printed().items()
        if n in ("retainer", "carrier", "wrist_cover")
    )
    assert inner + 8.0 == pytest.approx(ur_arm.INNER_MASS * 1000, abs=1.5)  # 나사와 너트 8.0
    out = sum(
        printed_mass_g(s, up)
        for n, (s, up, _c, _p) in wrist.printed().items()
        if n in ("out_disc", "hub_plate")
    )
    assert out + 13.3 == pytest.approx(ur_arm.OUT_MASS * 1000, abs=1.5)  # 혼 2.0, 볼트와 너트 11.3
    for key, name in (("upper", "upper_arm"), ("fore", "forearm")):
        length, tube_r, pin_r = links.LINKS[name]
        k, m_foot, _y = ur_arm.LINK_TUBE[key]
        model = ur_arm.LINK_HOUSE_MASS + k * (length * MM - 0.0668) + m_foot
        assert printed_mass_g(links.link(length, tube_r, pin_r), UP) == pytest.approx(
            model * 1000, abs=2.0
        )
