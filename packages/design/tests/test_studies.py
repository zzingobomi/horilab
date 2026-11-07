"""검토 코드가 깨지지 않았는지 확인한다. 결론(어느 대안이 낫나)은 질량 가정에 달려서 여기서 보지 않는다."""

from __future__ import annotations

import numpy as np
import pytest

from horilab_design.ik import AnalyticIK
from horilab_design.studies import cameras, packaging, skeleton
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
from horilab_design.values import load_arm


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


def test_packaging_layout_matches_the_value_file() -> None:
    # 배치 검토의 기본 골격은 params/arm.toml(0017)과 같아야 한다
    arm = load_arm().arm
    layout = packaging.Layout()
    assert layout.upper * 1000 == pytest.approx(arm.upper_arm_mm)
    assert layout.fore * 1000 == pytest.approx(arm.forearm_mm)
    assert layout.d45 * 1000 == pytest.approx(arm.j4_to_j5_mm)
    assert layout.d56 * 1000 == pytest.approx(arm.j5_to_j6_mm)
    assert layout.d6f * 1000 == pytest.approx(arm.j6_to_flange_mm)


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


def test_placed_stacking_poses_keep_the_skeleton_targets() -> None:
    # 기둥 검토는 박스 쌓기 자세를 실제 방위에 둔다. 반지름과 높이는 골격 검토의 목표와 같아야 한다.
    tool = packaging.TOOLS["suction"]
    placed = cameras.placed_stacking_poses(tool)
    expected = skeleton.targets()
    assert len(placed) == len(expected)
    for pose, (radius, height, _) in zip(placed, expected, strict=True):
        tcp = pose.T[:3, 3] + pose.T[:3, 2] * tool.length
        assert np.hypot(tcp[0], tcp[1]) == pytest.approx(radius)
        assert tcp[2] == pytest.approx(height)


def test_overhead_post_camera_sees_the_work_spots_and_stays_out_of_the_way() -> None:
    cam = cameras.RECOMMENDED
    R, eye = cam.frame(), np.array(cam.eye)
    for pts in cameras.work_targets().values():
        assert cameras.in_view(R, eye, pts).all()
    fixed = cameras.post_prims(cam)
    tool = packaging.TOOLS["suction"]
    robot, prims, _ = packaging.build_arm(packaging.Layout(), tool)
    scene = packaging.build_scene(robot, prims, fixed)
    poses = cameras.placed_stacking_poses(tool) + cameras.transit_poses(tool)
    result = packaging.check_set(scene, AnalyticIK(robot), poses, tool.name, tool.symmetric)
    assert result.clear == result.total, result.failures
    robot, prims, _ = packaging.build_arm(packaging.Layout(), packaging.TOOLS["gripper"])
    assert cameras.upright_hits(packaging.build_scene(robot, prims, fixed)) == 0


def test_wrist_camera_aim_puts_the_lens_on_the_view_point() -> None:
    layout = packaging.Layout()
    robot, _, mods = packaging.build_arm(layout, packaging.TOOLS["suction"])
    cam = packaging.camera_of(layout, mods)
    board = np.array([*cameras.CAL_BOARD, 0.0])
    eye = board + np.array([0.0, 0.0, 0.25])
    seeds = [np.radians([0.0, 20.0, 80.0, 165.0, 90.0, 10.0])]
    solutions = cameras.aim(robot, cam, eye, board, seeds)
    assert solutions
    lens, axis = cameras.camera_pose(robot, cam, solutions[0])
    np.testing.assert_allclose(lens, eye, atol=1e-4)
    assert axis @ np.array([0.0, 0.0, -1.0]) > np.cos(np.radians(0.2))


def test_outward_wrist_keeps_j5_negative() -> None:
    robot = ur_arm(wrist=-1)
    masses = point_masses(robot, MASS_CASES[0], 0.05)
    solutions = task_solutions(robot, masses, 0.40, wrist=-1, r_min=0.16)
    assert all(q is not None for q in solutions)
    assert all(q[4] < 0 and q[2] > 0 for q in solutions if q is not None)
