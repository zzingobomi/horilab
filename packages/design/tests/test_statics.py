"""정적 계산(중력 토크, 백래시 끝점 오차)을 손 계산과 MuJoCo로 확인한다."""

from __future__ import annotations

import mujoco
import numpy as np

from horilab_design.mjcf import build_spec
from horilab_design.robot import Joint, Robot, Vec3
from horilab_design.statics import (
    FLANGE,
    GRAVITY,
    PointMass,
    backlash_errors,
    frame_poses,
    holding_torques,
)

Y: Vec3 = (0.0, 1.0, 0.0)


def planar_arm() -> Robot:
    """y축으로 도는 관절 두 개. 영점에서 팔이 x축으로 수평이다 (링크 0.3 m, 플랜지까지 0.2 m)."""
    joints = (
        Joint("j1", (0.0, 0.0, 0.0), Y, -3.0, 3.0, effort=1.0, velocity=1.0),
        Joint("j2", (0.3, 0.0, 0.0), Y, -3.0, 3.0, effort=1.0, velocity=1.0),
    )
    T = np.eye(4)
    T[:3, 3] = (0.2, 0.0, 0.0)
    return Robot("planar_test", joints, T)


MASSES = (
    PointMass("link1", 1, (0.15, 0.0, 0.0), 0.10),
    PointMass("link2", 2, (0.10, 0.0, 0.0), 0.05),
    PointMass("payload", FLANGE, (0.0, 0.0, 0.0), 0.02),
    PointMass("base", 0, (0.0, 0.0, 0.0), 5.0),  # 베이스 질량은 어느 관절에도 걸리지 않는다
)


def test_horizontal_arm_matches_hand_calculation() -> None:
    torques = holding_torques(planar_arm(), [0.0, 0.0], MASSES)
    # y축 기준으로, x 방향 팔 끝의 무게는 음(-)의 토크로 버텨야 한다.
    j1 = -GRAVITY * (0.10 * 0.15 + 0.05 * 0.40 + 0.02 * 0.50)
    j2 = -GRAVITY * (0.05 * 0.10 + 0.02 * 0.20)
    np.testing.assert_allclose(torques, [j1, j2], atol=1e-12)


def test_link_hanging_below_its_joint_needs_no_torque() -> None:
    # j2를 90° 돌리면 링크 2가 j2 바로 아래로 늘어진다.
    torques = holding_torques(planar_arm(), [0.0, np.pi / 2], MASSES)
    assert abs(torques[1]) < 1e-12


def test_matches_mujoco_jacobian() -> None:
    # 같은 질량에 대해 MuJoCo의 자코비안으로 J^T F를 계산해 비교한다.
    robot = planar_arm()
    model = build_spec(robot).compile()
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    for _ in range(10):
        q = rng.uniform(-2.0, 2.0, 2)
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        frames = frame_poses(robot, q)
        expected = np.zeros(2)
        for point in MASSES:
            index = 3 if point.link == FLANGE else point.link
            body = model.body("base" if index == 0 else f"link{min(index, 2)}").id
            T = frames[index]
            position = T[:3, :3] @ np.asarray(point.position) + T[:3, 3]
            jacp = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jacp, None, position, body)
            expected -= jacp.T @ np.array([0.0, 0.0, -GRAVITY * point.mass])
        np.testing.assert_allclose(holding_torques(robot, q, MASSES), expected, atol=1e-12)


def test_backlash_error_is_lever_arm_times_play() -> None:
    errors = backlash_errors(planar_arm(), [0.0, 0.0], [0.01, 0.02], tcp=(0.05, 0.0, 0.0))
    np.testing.assert_allclose(errors, [0.55 * 0.01, 0.25 * 0.02], atol=1e-12)
