"""design 테스트가 같이 쓰는 로봇: 테스트용 두 대(구형 손목, UR형)와 레포의 값 파일로 만든 지금 설계."""

from __future__ import annotations

import numpy as np
import pytest

from horilab_design.robot import Joint, Mat, Robot, Vec3, rotation
from horilab_design.urdf import matrix_from_rpy
from horilab_design.values import load_robot

X: Vec3 = (1.0, 0.0, 0.0)
Y: Vec3 = (0.0, 1.0, 0.0)
Z: Vec3 = (0.0, 0.0, 1.0)
MINUS_Z: Vec3 = (0.0, 0.0, -1.0)


def _joint(name: str, origin: Vec3, axis: Vec3, limit: float) -> Joint:
    return Joint(name, origin, axis, -limit, limit, effort=1.0, velocity=4.0)


def transform(R: Mat, p: Vec3) -> Mat:
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
    # 세 rpy 각이 모두 0이 아닌 플랜지: URDF rpy의 순서가 틀리면 URDF 대조 테스트가 잡는다.
    flange = transform(matrix_from_rpy((0.3, -0.5, 0.7)), (0.01, -0.02, 0.03))
    return Robot("spherical_wrist_test", joints, flange)


def ur_type_robot() -> Robot:
    joints = (
        _joint("j1", (0.0, 0.0, 0.16), Z, np.pi),
        _joint("j2", (0.0, 0.14, 0.0), Y, np.pi),
        _joint("j3", (0.42, -0.12, 0.0), Y, np.pi),
        _joint("j4", (0.39, 0.0, 0.0), Y, np.pi),
        _joint("j5", (0.0, 0.10, 0.0), MINUS_Z, np.pi),
        _joint("j6", (0.0, 0.0, -0.10), Y, np.pi),
    )
    # 플랜지 z가 마지막 축(y) 방향을 보도록 x축으로 -90° 돌린다.
    return Robot("ur_type_test", joints, transform(rotation(X, -np.pi / 2), (0.0, 0.10, 0.0)))


ROBOTS = {
    "spherical_wrist": spherical_wrist_robot(),
    "ur_type": ur_type_robot(),
    "horilab": load_robot(),
}


@pytest.fixture(params=sorted(ROBOTS))
def robot(request: pytest.FixtureRequest) -> Robot:
    return ROBOTS[request.param]


def random_configs(robot: Robot, n: int = 20, seed: int = 0) -> list[Mat]:
    """영점과, 관절 한계 안의 무작위 자세들."""
    rng = np.random.default_rng(seed)
    span = robot.upper - robot.lower
    randoms = [robot.lower + span * (0.05 + 0.9 * rng.random(len(robot.joints))) for _ in range(n)]
    return [np.zeros(len(robot.joints)), *randoms]


@pytest.fixture
def configs(robot: Robot) -> list[Mat]:
    return random_configs(robot)
