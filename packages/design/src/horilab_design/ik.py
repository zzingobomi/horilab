"""해석적 IK (EAIK). 로봇 기술의 관절 축과 원점을 그대로 넘긴다 (0008, 0010).

골격 설계에서 작업 범위를 셀 때 쓴다. 가능한 해를 모두 주므로, 수치 IK처럼 못 푼 자세를
닿지 않는 자세로 잘못 세지 않는다.
"""

from __future__ import annotations

import numpy as np
from eaik.IK_HP import HPRobot
from numpy.typing import ArrayLike

from horilab_design.robot import Mat, Robot


class AnalyticIK:
    """한 로봇의 해석적 IK. EAIK가 축 배치로 풀 수 있는 구조인지 판별하지 못하면 만들 수 없다."""

    def __init__(self, robot: Robot) -> None:
        # 링크 좌표계가 영점에서 베이스와 같은 방향이라, 축과 원점이 곧 EAIK가 받는
        # 베이스 기준의 축(H)과 관절 사이 위치(P)다.
        H = np.array([joint.axis for joint in robot.joints], dtype=float)
        P = np.array(
            [*(joint.origin for joint in robot.joints), robot.T_last_flange[:3, 3]], dtype=float
        )
        self._solver = HPRobot(H, P)
        if not self._solver.hasKnownDecomposition():
            raise ValueError(f"{robot.name}: EAIK가 풀 수 있는 축 배치가 아니다")
        self._joint_count = len(robot.joints)
        # EAIK의 끝 좌표계는 영점에서 베이스와 같은 방향이다. 플랜지의 회전은 따로 빼고 넣는다.
        self._R_last_flange = robot.T_last_flange[:3, :3]

    @property
    def family(self) -> str:
        """EAIK가 판별한 구조 (예: 구형 손목, 평행한 세 축)."""
        return str(self._solver.getKinematicFamily())

    def solve(self, T_base_flange: ArrayLike) -> Mat:
        """플랜지 자세에 맞는 정확한 해를 모두 준다 (k × 관절 수, 각은 -π~π). 관절 한계는 보지 않는다."""
        target = np.array(T_base_flange, dtype=float)
        target[:3, :3] = target[:3, :3] @ self._R_last_flange.T
        solutions = self._solver.IK(target)
        Q = np.atleast_2d(np.asarray(solutions.Q, dtype=float))
        exact = ~np.asarray(solutions.is_LS, dtype=bool)
        return wrap_angle(Q[exact]) if Q.size else np.empty((0, self._joint_count))


def wrap_angle(q: ArrayLike) -> Mat:
    """각을 -π~π로 맞춘다."""
    return (np.asarray(q, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi
