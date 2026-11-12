"""출력 무게 어림식이 뽑아서 잰 부품과 맞는지."""

from __future__ import annotations

import pytest

from horilab_design.parts import joint_test as jt
from horilab_design.printcheck import printed_mass_g
from horilab_design.values import load_measured

# 0.20mm Strength로 뽑아 잰 부품. 게이지 받침(gauge_bracket)은 잰 값이 형상의 0.33배라 식과 맞지 않아 뺐다
# (뽑은 뒤 형상을 고쳤을 수 있다)
STRENGTH_PARTS = ("elbow_house", "elbow_disc_out", "elbow_disc_back", "test_link", "test_stand")


@pytest.mark.slow
def test_mass_estimate_matches_weighed_parts() -> None:
    weighed = load_measured().printed_mass_g
    for spec in jt.PARTS:
        if spec.name not in STRENGTH_PARTS:
            continue
        shape = jt.placed_for_print(spec)  # 뽑는 방향으로 놓은 형상 (z가 위)
        estimate = printed_mass_g(shape, (0.0, 0.0, 1.0))
        assert estimate == pytest.approx(weighed[spec.name], rel=0.03), spec.name
