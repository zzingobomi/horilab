"""정적 계산(중력 토크, 백래시 끝점 오차)을 손 계산과 MuJoCo로 확인한다."""

from __future__ import annotations

import mujoco
import numpy as np

from horilab_design.mjcf import build_spec
from horilab_design.robot import Joint, Mat, Robot, Vec3, flange_pose
from horilab_design.statics import (
    FLANGE,
    GRAVITY,
    PointMass,
    backlash_displacements,
    backlash_errors,
    frame_poses,
    holding_torques,
    worst_backlash_error,
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


PLANAR_MASSES = (
    PointMass("link1", 1, (0.15, 0.0, 0.0), 0.10),
    PointMass("link2", 2, (0.10, 0.0, 0.0), 0.05),
    PointMass("payload", FLANGE, (0.0, 0.0, 0.0), 0.02),
    PointMass("base", 0, (0.0, 0.0, 0.0), 5.0),  # 베이스 질량은 어느 관절에도 걸리지 않는다
)


def six_axis_masses() -> tuple[PointMass, ...]:
    """링크마다 축에서 비켜 있는 질량을 둔다. 좌표계를 잘못 돌리면 토크가 달라지게."""
    return (
        *(PointMass(f"link{k}", k, (0.03 * k, -0.02, 0.04), 0.05 + 0.01 * k) for k in range(1, 7)),
        PointMass("tool", FLANGE, (0.02, 0.01, 0.05), 0.08),
    )


def _mujoco_holding_torques(robot: Robot, q: Mat, masses: tuple[PointMass, ...]) -> Mat:
    """같은 질량에 대해 MuJoCo의 자코비안으로 J^T F를 계산한다 (버티는 토크라 부호를 바꾼다)."""
    model = build_spec(robot).compile()
    data = mujoco.MjData(model)
    for joint, angle in zip(robot.joints, q, strict=True):
        data.qpos[model.joint(joint.name).qposadr[0]] = angle
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    frames = frame_poses(robot, q)
    n = len(robot.joints)
    torques = np.zeros(model.nv)
    for point in masses:
        index = n + 1 if point.link == FLANGE else point.link
        body = model.body("base" if index == 0 else f"link{min(index, n)}").id
        T = frames[index]
        position = T[:3, :3] @ np.asarray(point.position) + T[:3, 3]
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jacp, None, position, body)
        torques -= jacp.T @ np.array([0.0, 0.0, -GRAVITY * point.mass])
    return np.array([torques[model.joint(j.name).dofadr[0]] for j in robot.joints])


def test_horizontal_arm_matches_hand_calculation() -> None:
    torques = holding_torques(planar_arm(), [0.0, 0.0], PLANAR_MASSES)
    # y축 기준으로, x 방향 팔 끝의 무게는 음(-)의 토크로 버텨야 한다.
    j1 = -GRAVITY * (0.10 * 0.15 + 0.05 * 0.40 + 0.02 * 0.50)
    j2 = -GRAVITY * (0.05 * 0.10 + 0.02 * 0.20)
    np.testing.assert_allclose(torques, [j1, j2], atol=1e-12)


def test_link_hanging_below_its_joint_needs_no_torque() -> None:
    # j2를 90° 돌리면 링크 2가 j2 바로 아래로 늘어진다.
    torques = holding_torques(planar_arm(), [0.0, np.pi / 2], PLANAR_MASSES)
    assert abs(torques[1]) < 1e-12


def test_matches_mujoco_jacobian_on_six_axis_robots(robot: Robot, configs: list[Mat]) -> None:
    masses = six_axis_masses()
    for q in configs:
        np.testing.assert_allclose(
            holding_torques(robot, q, masses), _mujoco_holding_torques(robot, q, masses), atol=1e-12
        )


def test_backlash_error_is_lever_arm_times_play() -> None:
    errors = backlash_errors(planar_arm(), [0.0, 0.0], [0.01, 0.02], tcp=(0.05, 0.0, 0.0))
    np.testing.assert_allclose(errors, [0.55 * 0.01, 0.25 * 0.02], atol=1e-12)


def test_backlash_displacement_matches_finite_rotation(robot: Robot, configs: list[Mat]) -> None:
    # 관절 하나를 아주 조금 돌렸을 때 TCP가 실제로 움직이는 양과 비교한다.
    step = 1e-6
    for q in configs[1:4]:
        moves = backlash_displacements(robot, q, np.full(len(q), step))
        for i in range(len(q)):
            nudged = q.copy()
            nudged[i] += step
            actual = flange_pose(robot, nudged)[:3, 3] - flange_pose(robot, q)[:3, 3]
            np.testing.assert_allclose(moves[i], actual, atol=1e-11)


def test_worst_backlash_error_adds_vectors_not_lengths() -> None:
    # 서로 수직인 두 오차는 길이를 더한 값이 아니라 빗변이 된다.
    moves = np.array([[0.003, 0.0, 0.0], [0.0, 0.004, 0.0]])
    assert abs(worst_backlash_error(moves) - 0.005) < 1e-12
