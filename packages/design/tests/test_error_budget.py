"""오차 예산: 몫을 모으는 식과, 팔 모델에서 나온 값이 서로 맞는지."""

from __future__ import annotations

import numpy as np
import pytest

from horilab_design.studies import error_budget as eb


def test_relative_error_comes_from_the_radial_component() -> None:
    # 반지름 쪽 오차가 20 cm에서 0, 30 cm에서 1 mm면 그 두 점 사이 거리가 1 mm 틀린다.
    # 반지름 26 cm에서 2.6 mm면 원이 1% 커져서, 둘레 쪽 100 mm도 1 mm 틀린다
    moves = {
        0.20: np.array([0.0, 0.0005, 0.0]),
        0.30: np.array([0.001, 0.0, 0.002]),
        0.26: np.array([0.0026, 0.0, 0.0]),
    }
    radial, tangential = eb.relative(moves)
    assert radial == pytest.approx(0.001)
    assert tangential == pytest.approx(0.001)
    # 부호를 모르는 몫 둘(3 × 2)은 제곱합의 제곱근으로 모은다
    two = {r: np.column_stack([v, v]) for r, v in moves.items()}
    radial, tangential = eb.relative(two)
    assert radial == pytest.approx(0.001 * np.sqrt(2))
    assert tangential == pytest.approx(0.001 * np.sqrt(2))


def test_repeatability_statistic_is_the_iso_form_for_a_normal_spread() -> None:
    # 평균점에서의 거리의 평균 + 3 × 그 표준편차: 정규 분포 한 축에서 2.61σ
    assert eb.REPEAT_SIGMAS == pytest.approx(2.606, abs=0.001)
    rng = np.random.default_rng(0)
    distance = np.abs(rng.normal(0.0, 1.0, 200_000))
    assert distance.mean() + 3 * distance.std() == pytest.approx(eb.REPEAT_SIGMAS, abs=0.02)


@pytest.mark.slow
def test_budget_is_consistent_with_the_arm_model() -> None:
    b = eb.budget()
    far = b["far"]
    # J1이 한 칸 돌면 TCP는 둘레 쪽으로 반지름 × 한 칸만큼 움직인다
    assert b["step_far"][1, 0] == pytest.approx(far * eb.STEP, rel=0.01)
    assert b["step_far"][0, 0] == pytest.approx(0.0, abs=1e-6)
    # 흩어지는 폭에 비례한다
    assert b["repeat"][1.0] == pytest.approx(2 * b["repeat"][0.5])
    # 도구가 아래면 J1과 J6에는 중력이 걸리지 않는다
    assert b["free_joints"] == [1, 6]
    sources = b["sources"]
    empty, loaded = sources["서보 처짐 (그리퍼)"], sources["서보 처짐 (그리퍼 + 짐 150 g)"]
    assert loaded["far"][2] > empty["far"][2] > 0.001  # 짐을 들면 더 처지고, 처짐은 mm 단위다
    # 처짐은 다른 몫보다 크다 (보정이 필요한 까닭)
    others = [s["radial"] for name, s in sources.items() if "처짐" not in name and "짐" not in name]
    assert sources["서보 처짐 (포인터)"]["radial"] > max(others)
    for tilt in eb.TILTS:
        assert b["tilted"][tilt]["reached"] == 8
