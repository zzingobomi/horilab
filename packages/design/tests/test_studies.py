"""검토 코드가 깨지지 않았는지와, 결정 기록에 쓴 결론이 그대로인지 확인한다."""

from __future__ import annotations

import numpy as np

from horilab_design.studies.drive_torque import (
    MASS_CASES,
    STS3250_RATED,
    point_masses,
    stick_arm,
    task_poses,
    worst_torques,
)


def test_tool_down_targets_are_reachable_up_to_30_cm() -> None:
    robot = stick_arm(0.20, 0.18)
    worst = worst_torques(robot, point_masses(0.20, 0.18, MASS_CASES[0], 0.0), 0.30, steps=3)
    assert worst.unreachable == 0


def test_heavy_case_needs_more_than_direct_shoulder() -> None:
    # 0011의 근거: 무거운 질량 가정에서는 30 cm 반지름에서 어깨 직결이 정격 토크를 넘는다.
    robot = stick_arm(0.20, 0.18)
    worst = worst_torques(robot, point_masses(0.20, 0.18, MASS_CASES[1], 0.1), 0.30, steps=3)
    assert worst.shoulder > STS3250_RATED


def test_task_poses_point_the_tool_down() -> None:
    for T in task_poses(0.30, steps=3):
        np.testing.assert_allclose(T[:3, 2], [0.0, 0.0, -1.0], atol=1e-12)
