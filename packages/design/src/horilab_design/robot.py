"""로봇 기술: 관절의 원점, 축, 한계 (SI 단위).

링크 좌표계는 영점 자세에서 모두 베이스 좌표계와 같은 방향이다
(docs/decisions/0010-code-test-git-rules.md의 "링크 좌표계"). 그래서 관절 원점은
부모 링크 좌표계에서의 위치만 주고, 관절 축은 단위 벡터로 준다. 회전이 있는 좌표계는
마지막 링크에 붙는 플랜지뿐이다.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vec3 = tuple[float, float, float]
Mat = NDArray[np.float64]


@dataclass(frozen=True)
class Joint:
    """회전 관절 하나. 관절의 원점이 곧 자식 링크 좌표계의 원점이다."""

    name: str
    origin: Vec3  # 부모 링크 좌표계에서 본 관절 원점 [m]
    axis: Vec3  # 관절 축 단위 벡터. 영점에서는 베이스 좌표계 기준과 같다
    lower: float  # [rad]
    upper: float  # [rad]
    effort: float  # 낼 수 있는 토크 [N·m]
    velocity: float  # 최대 각속도 [rad/s]

    def __post_init__(self) -> None:
        if not np.isclose(np.linalg.norm(self.axis), 1.0, rtol=0.0, atol=1e-9):
            raise ValueError(f"{self.name}: 관절 축은 단위 벡터여야 한다: {self.axis}")
        if not self.lower < self.upper:
            raise ValueError(f"{self.name}: 관절 한계는 lower < upper여야 한다")


@dataclass(frozen=True, eq=False)
class Robot:
    """직렬 팔 하나. 관절은 베이스에서 플랜지 쪽 순서다."""

    name: str
    joints: tuple[Joint, ...]
    T_last_flange: Mat  # 마지막 링크 좌표계에서 본 플랜지 좌표계 (z가 도구 쪽)

    def __post_init__(self) -> None:
        names = [joint.name for joint in self.joints]
        if len(set(names)) != len(names):
            raise ValueError(f"관절 이름이 겹친다: {names}")
        check_transform(self.T_last_flange)

    @property
    def lower(self) -> Mat:
        return np.array([joint.lower for joint in self.joints])

    @property
    def upper(self) -> Mat:
        return np.array([joint.upper for joint in self.joints])


def check_transform(T: ArrayLike) -> None:
    """4×4 동차 변환 행렬인지 확인한다 (회전 부분이 직교이고 행렬식이 1)."""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError("4×4 동차 변환 행렬이어야 한다")
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-9) or not np.isclose(np.linalg.det(R), 1.0):
        raise ValueError("회전 부분이 회전 행렬이 아니다")


def rotation(axis: ArrayLike, angle: float) -> Mat:
    """단위 축과 각도 [rad]로 3×3 회전 행렬을 만든다 (로드리게스 공식)."""
    x, y, z = np.asarray(axis, dtype=float)
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def link_poses(robot: Robot, q: ArrayLike) -> list[Mat]:
    """관절 각 q [rad]에서 링크 좌표계들의 자세 T_base_link1 … T_base_linkN."""
    angles = np.asarray(q, dtype=float)
    if angles.shape != (len(robot.joints),):
        raise ValueError(f"관절 각은 {len(robot.joints)}개여야 한다: {angles.shape}")
    T = np.eye(4)
    poses: list[Mat] = []
    for joint, angle in zip(robot.joints, angles, strict=True):
        step = np.eye(4)
        step[:3, :3] = rotation(joint.axis, float(angle))
        step[:3, 3] = joint.origin
        T = T @ step
        poses.append(T)
    return poses


def flange_pose(robot: Robot, q: ArrayLike) -> Mat:
    """관절 각 q [rad]에서 플랜지 자세 T_base_flange."""
    return link_poses(robot, q)[-1] @ robot.T_last_flange
