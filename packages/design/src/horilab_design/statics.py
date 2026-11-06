"""정적 계산: 중력을 버티는 관절 토크와, 백래시로 생기는 끝점 오차.

골격 설계의 자동 검사에 쓴다 (docs/design.md의 "설계 방식"). 골격 설계 단계에는 부품
모양이 없어서 질량을 점 몇 개(서보, 링크, 손목, 도구, 짐)로 어림한다. 부품이 생기면
CAD에서 나온 질량 특성으로 바꾼다 (docs/decisions/0006-data-and-pipeline.md).
"""

from __future__ import annotations

import itertools
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from horilab_design.robot import Mat, Robot, Vec3, link_poses

GRAVITY = 9.81  # [m/s²]
FLANGE = -1  # PointMass.link에서 플랜지 좌표계를 뜻하는 값


@dataclass(frozen=True)
class PointMass:
    """좌표계 하나에 붙은 점 질량.

    link는 0이 베이스, i(1…N)가 관절 i가 움직이는 링크, FLANGE(-1)가 플랜지 좌표계다.
    """

    label: str
    link: int
    position: Vec3  # 그 좌표계에서 본 위치 [m]
    mass: float  # [kg]


def frame_poses(robot: Robot, q: ArrayLike) -> list[Mat]:
    """[베이스, 링크 1 … 링크 N, 플랜지]의 자세를 베이스 좌표계로 준다."""
    links = link_poses(robot, q)
    return [np.eye(4), *links, links[-1] @ robot.T_last_flange]


def holding_torques(robot: Robot, q: ArrayLike, masses: Sequence[PointMass]) -> Mat:
    """자세 q에서 중력을 버티려고 관절마다 내야 하는 토크 [N·m]. 부호는 관절 축 방향 기준이다."""
    frames = frame_poses(robot, q)
    n = len(robot.joints)
    origins = [frames[i + 1][:3, 3] for i in range(n)]
    axes = [frames[i + 1][:3, :3] @ np.asarray(robot.joints[i].axis) for i in range(n)]
    torques = np.zeros(n)
    for point in masses:
        index = n + 1 if point.link == FLANGE else point.link
        if not 0 <= index <= n + 1:
            raise ValueError(f"{point.label}: 없는 좌표계 번호다: {point.link}")
        T = frames[index]
        position = T[:3, :3] @ np.asarray(point.position) + T[:3, 3]
        force = np.array([0.0, 0.0, -GRAVITY * point.mass])
        # 관절 i는 링크 i부터 끝까지를 움직이므로, 그 뒤에 달린 질량만 관절 i에 걸린다.
        for i in range(min(index, n)):
            torques[i] -= axes[i] @ np.cross(position - origins[i], force)
    return torques


def backlash_displacements(
    robot: Robot, q: ArrayLike, play: ArrayLike, tcp: Vec3 = (0.0, 0.0, 0.0)
) -> Mat:
    """관절마다 각도 틈 play [rad]만큼 돌았을 때 TCP가 움직이는 벡터 [m] (관절 수 × 3, 베이스 좌표계).

    tcp는 플랜지 좌표계에서 본 TCP 위치다. 자세(방향) 오차는 보지 않는다.
    """
    frames = frame_poses(robot, q)
    n = len(robot.joints)
    gaps = np.asarray(play, dtype=float)
    if gaps.shape != (n,):
        raise ValueError(f"틈은 관절 수({n})만큼 줘야 한다")
    tcp_position = frames[n + 1][:3, :3] @ np.asarray(tcp) + frames[n + 1][:3, 3]
    moves = np.zeros((n, 3))
    for i in range(n):
        axis = frames[i + 1][:3, :3] @ np.asarray(robot.joints[i].axis)
        moves[i] = np.cross(axis, tcp_position - frames[i + 1][:3, 3]) * gaps[i]
    return moves


def backlash_errors(
    robot: Robot, q: ArrayLike, play: ArrayLike, tcp: Vec3 = (0.0, 0.0, 0.0)
) -> Mat:
    """관절마다 각도 틈 play [rad]가 TCP 위치를 얼마나 흔드는지 [m]. 관절별 크기다."""
    return np.linalg.norm(backlash_displacements(robot, q, play, tcp), axis=1)


def worst_backlash_error(moves: Mat) -> float:
    """여러 관절의 틈이 가장 나쁜 방향으로 겹칠 때 TCP가 흔들리는 양 [m].

    관절마다 틈은 + 또는 - 쪽으로 붙을 수 있으므로, 부호 조합 중 합 벡터가 가장 긴 것을 고른다.
    """
    signs = itertools.product((-1.0, 1.0), repeat=len(moves))
    return max(float(np.linalg.norm(np.asarray(s) @ moves)) for s in signs)
