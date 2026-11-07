"""검토 코드가 깨지지 않았는지 확인한다. 결론(어느 대안이 낫나)은 질량 가정에 달려서 여기서 보지 않는다."""

from __future__ import annotations

import numpy as np
import pytest

from horilab_design.ik import AnalyticIK
from horilab_design.studies import packaging, skeleton
from horilab_design.studies.common import (
    MASS_CASES,
    TASK_YAWS_DEG,
    pick_solution,
    point_masses,
    spherical_arm,
    task_poses,
    task_solutions,
    tool_down,
    ur_arm,
)


def test_task_poses_point_the_tool_down() -> None:
    poses = list(task_poses(0.30, steps=3))
    assert len(poses) == 3 * 2 * len(TASK_YAWS_DEG)
    for T in poses:
        np.testing.assert_allclose(T[:3, 2], [0.0, 0.0, -1.0], atol=1e-12)


def test_both_wrist_templates_reach_every_task_pose_and_keep_one_wrist_side() -> None:
    for robot in (spherical_arm(), ur_arm()):
        solutions = task_solutions(robot, point_masses(robot, MASS_CASES[0], 0.05), 0.30)
        assert all(q is not None for q in solutions)
        assert all(q[4] > 0 for q in solutions if q is not None)


def test_skeleton_targets_follow_the_layout() -> None:
    # 0014: 집는 곳 1자리(박스 1층) + 팔레트 4자리 × 4층, 자리마다 박스 윗면과 그 위 3 cm
    poses = skeleton.targets()
    assert len(poses) == (1 + 4 * 4) * 2
    radii = [radius for radius, _, _ in poses]
    heights = [height for _, height, _ in poses]
    assert 0.179 < min(radii) and max(radii) < 0.241
    assert min(heights) == pytest.approx(0.04) and max(heights) == pytest.approx(0.19)


def test_far_target_is_not_reached_by_flipping_the_wrist() -> None:
    robot = ur_arm()
    ik = AnalyticIK(robot)
    T = tool_down(0.40, 0.16)
    assert len(ik.solve_in_limits(T)) > 0  # 손목을 뒤집은 해로는 닿는다
    assert pick_solution(robot, ik, T, point_masses(robot, MASS_CASES[0], 0.05)) is None


@pytest.mark.parametrize("sts", ["drawing", "step"])
def test_packaging_layout_stacks_boxes_without_collisions(sts: str) -> None:
    # 손목 55 · 55 · 50 mm는 STS 외형 자료 두 가지에서 다 박스 쌓기 자세가 부딪히지 않는다
    layout = packaging.Layout(sts=sts)  # type: ignore[arg-type]
    tool = packaging.TOOLS["suction"]
    robot, prims, _ = packaging.build_arm(layout, tool)
    scene = packaging.build_scene(robot, prims)
    poses = packaging.stacking_poses(tool)
    result = packaging.check_set(scene, AnalyticIK(robot), poses, tool.name, tool.symmetric)
    assert result.reached == result.total == len(poses)
    assert result.clear == result.total, result.failures


def test_packaging_reports_designed_gaps_but_not_as_collisions() -> None:
    tool = packaging.TOOLS["gripper"]
    robot, prims, _ = packaging.build_arm(packaging.Layout(), tool)
    scene = packaging.build_scene(robot, prims)
    q = packaging.solve(
        AnalyticIK(robot), packaging.tool_pose((0.2, 0.0, 0.1), 0.0, 0.0, 0.0, tool.length)
    )[0]
    found = packaging.hits(scene, q)
    assert not packaging.collisions(found)
    assert any(h.designed for h in found)  # 서보와 그 혼·브래킷은 일부러 가깝다


def test_outward_wrist_keeps_j5_negative() -> None:
    robot = ur_arm(wrist=-1)
    masses = point_masses(robot, MASS_CASES[0], 0.05)
    solutions = task_solutions(robot, masses, 0.40, wrist=-1, r_min=0.16)
    assert all(q is not None for q in solutions)
    assert all(q[4] < 0 and q[2] > 0 for q in solutions if q is not None)
