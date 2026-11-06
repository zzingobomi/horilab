"""로봇 기술에서 나온 FK, MJCF, URDF, 해석적 IK가 서로 맞는지 확인한다.

docs/decisions/0006-data-and-pipeline.md의 FK 대조 테스트다. 축 방향이 뒤집혀도 영점에서는
같아 보이므로 무작위 자세에서도 비교한다. 테스트용 로봇 두 대(구형 손목, UR형)를 쓴다.
실제 치수는 골격 설계에서 정한다.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yourdfpy

from horilab_design.ik import AnalyticIK, wrap_angle
from horilab_design.mjcf import build_spec
from horilab_design.robot import Joint, Mat, Robot, Vec3, flange_pose, rotation
from horilab_design.urdf import build_urdf, matrix_from_rpy, rpy_from_matrix

X: Vec3 = (1.0, 0.0, 0.0)
Y: Vec3 = (0.0, 1.0, 0.0)
Z: Vec3 = (0.0, 0.0, 1.0)
MINUS_Z: Vec3 = (0.0, 0.0, -1.0)


def _joint(name: str, origin: Vec3, axis: Vec3, limit: float) -> Joint:
    return Joint(name, origin, axis, -limit, limit, effort=1.0, velocity=4.0)


def _transform(R: Mat, p: Vec3) -> Mat:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def spherical_wrist_robot() -> Robot:
    joints = (
        _joint("j1", (0.0, 0.0, 0.06), Z, 3.0),
        _joint("j2", (0.0, 0.0, 0.06), Y, 1.9),
        _joint("j3", (0.0, 0.0, 0.20), Y, 2.6),
        _joint("j4", (0.0, 0.0, 0.18), Z, 3.0),
        _joint("j5", (0.0, 0.0, 0.0), Y, 2.0),
        _joint("j6", (0.0, 0.0, 0.04), Z, 3.0),
    )
    return Robot("spherical_wrist_test", joints, _transform(np.eye(3), (0.0, 0.0, 0.01)))


def ur_type_robot() -> Robot:
    joints = (
        _joint("j1", (0.0, 0.0, 0.16), Z, np.pi),
        _joint("j2", (0.0, 0.14, 0.0), Y, np.pi),
        _joint("j3", (0.42, -0.12, 0.0), Y, np.pi),
        _joint("j4", (0.39, 0.0, 0.0), Y, np.pi),
        _joint("j5", (0.0, 0.10, 0.0), MINUS_Z, np.pi),
        _joint("j6", (0.0, 0.0, -0.10), Y, np.pi),
    )
    # 플랜지 z가 마지막 축(y) 방향을 보도록 x축으로 -90° 돌린다. URDF rpy 변환도 함께 시험된다.
    return Robot("ur_type_test", joints, _transform(rotation(X, -np.pi / 2), (0.0, 0.10, 0.0)))


ROBOTS = {"spherical_wrist": spherical_wrist_robot(), "ur_type": ur_type_robot()}


@pytest.fixture(params=sorted(ROBOTS))
def robot(request: pytest.FixtureRequest) -> Robot:
    return ROBOTS[request.param]


def _configs(robot: Robot, n: int = 20, seed: int = 0) -> list[Mat]:
    """영점과, 관절 한계 안의 무작위 자세들."""
    rng = np.random.default_rng(seed)
    span = robot.upper - robot.lower
    randoms = [robot.lower + span * (0.05 + 0.9 * rng.random(len(robot.joints))) for _ in range(n)]
    return [np.zeros(len(robot.joints)), *randoms]


def _mujoco_flange_pose(model: mujoco.MjModel, robot: Robot, q: Mat) -> Mat:
    data = mujoco.MjData(model)
    for joint, angle in zip(robot.joints, q, strict=True):
        data.qpos[model.joint(joint.name).qposadr[0]] = angle
    mujoco.mj_kinematics(model, data)
    site = data.site("flange")
    return _transform(site.xmat.reshape(3, 3), site.xpos)


def test_fk_matches_mujoco(robot: Robot) -> None:
    model = build_spec(robot).compile()
    for q in _configs(robot):
        np.testing.assert_allclose(
            _mujoco_flange_pose(model, robot, q), flange_pose(robot, q), atol=1e-9
        )


def test_mjcf_xml_keeps_kinematics(robot: Robot) -> None:
    # XML로 저장했다가 다시 읽어도 같아야 한다. 각도 단위(도, 라디안) 실수를 잡는다.
    model = mujoco.MjModel.from_xml_string(build_spec(robot).to_xml())
    for q in _configs(robot):
        np.testing.assert_allclose(
            _mujoco_flange_pose(model, robot, q), flange_pose(robot, q), atol=1e-9
        )


def test_fk_matches_urdf(robot: Robot, tmp_path: Path) -> None:
    path = tmp_path / "robot.urdf"
    path.write_text(build_urdf(robot), encoding="utf-8")
    urdf = yourdfpy.URDF.load(str(path), load_meshes=False)
    for q in _configs(robot):
        urdf.update_cfg({joint.name: float(a) for joint, a in zip(robot.joints, q, strict=True)})
        np.testing.assert_allclose(
            urdf.get_transform("flange", "base"), flange_pose(robot, q), atol=1e-9
        )


def test_analytic_ik_finds_the_original_pose(robot: Robot) -> None:
    ik = AnalyticIK(robot)
    for q in _configs(robot)[1:]:  # 영점은 특이 자세라 뺀다
        target = flange_pose(robot, q)
        solutions = ik.solve(target)
        assert len(solutions) > 0
        for solution in solutions:
            np.testing.assert_allclose(flange_pose(robot, solution), target, atol=1e-8)
        assert any(np.allclose(wrap_angle(s - q), 0.0, atol=1e-6) for s in solutions)


@pytest.mark.parametrize(
    "rpy",
    [(0.1, 0.2, 0.3), (-2.0, 1.0, 3.0), (0.3, np.pi / 2, -0.4), (-0.2, -np.pi / 2, 0.5)],
)
def test_rpy_round_trip(rpy: Vec3) -> None:
    R = matrix_from_rpy(rpy)
    np.testing.assert_allclose(matrix_from_rpy(rpy_from_matrix(R)), R, atol=1e-12)


def test_joint_rejects_non_unit_axis() -> None:
    with pytest.raises(ValueError):
        Joint("j1", (0.0, 0.0, 0.1), (0.0, 0.0, 2.0), -1.0, 1.0, effort=1.0, velocity=1.0)


def test_robot_rejects_non_rotation_flange() -> None:
    bad = np.eye(4)
    bad[0, 0] = 2.0
    with pytest.raises(ValueError):
        Robot("bad", spherical_wrist_robot().joints, bad)
