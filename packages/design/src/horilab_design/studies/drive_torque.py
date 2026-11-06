"""골격 설계 전 구동 검토: 관절 토크, 관절 범위, 백래시 끝점 오차를 어림한다.

docs/decisions/0011-joint-drives.md의 숫자를 다시 만든다.

    uv run python -m horilab_design.studies.drive_torque

- 팔은 0013에서 제안한 UR형 막대 모델이다 (common.py). 실제 축 배치와 길이는 골격 설계에서 정한다.
- 목표 자세는 박스 쌓기처럼 도구가 아래를 향한 자세다. 박스를 돌려 놓는 각도까지 바꿔 가며 본다.
- 질량은 점 질량으로 어림한 가정이다 (common.MASS_CASES). 실물을 재면 바꾼다.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Mat, Robot
from horilab_design.statics import (
    PointMass,
    backlash_displacements,
    holding_torques,
    worst_backlash_error,
)
from horilab_design.studies.common import (
    BACKLASH,
    BELT_EFFICIENCY,
    DUAL_SHARE,
    L1,
    L2,
    LT,
    MASS_CASES,
    RATED_MARGIN,
    STS3215_MASS,
    STS3215_RATED,
    STS3250_RATED,
    TASK_HEIGHTS,
    pick_solution,
    point_masses,
    task_solutions,
    tool_down,
    ur_arm,
)


def max_reach(robot: Robot, height: float) -> float:
    """도구가 아래를 향한 채 TCP가 닿는 가장 먼 반지름 [m] (5 mm 간격, 박스 돌림 없음)."""
    ik = AnalyticIK(robot)
    reachable = [
        r
        for r in np.arange(0.10, 0.60, 0.005)
        if len(ik.solve_in_limits(tool_down(float(r), height)))
    ]
    return float(max(reachable, default=0.0))


@dataclass(frozen=True)
class Worst:
    shoulder: float  # 어깨(J2)가 버티는 최대 토크 [N·m]
    elbow: float  # 팔꿈치(J3)가 버티는 최대 토크 [N·m]
    unreachable: int  # 닿지 않은 목표 수


def worst_torques(robot: Robot, masses: list[PointMass], r_max: float) -> Worst:
    solutions = task_solutions(robot, masses, r_max)
    reached = [q for q in solutions if q is not None]
    taus = np.abs(np.array([holding_torques(robot, q, masses) for q in reached]))
    return Worst(float(taus[:, 1].max()), float(taus[:, 2].max()), len(solutions) - len(reached))


def task_torques(robot: Robot, masses: list[PointMass], r_max: float = 0.30) -> Mat:
    """작업 자세마다 관절이 버티는 토크 [N·m]. 행이 자세다."""
    reached = [q for q in task_solutions(robot, masses, r_max) if q is not None]
    return np.array([holding_torques(robot, q, masses) for q in reached])


def used_ranges_deg(robot: Robot, masses: list[PointMass], r_max: float) -> tuple[Mat, Mat]:
    """목표 자세들에서 실제로 쓰는 관절 각의 최솟값과 최댓값 [deg]."""
    reached = [q for q in task_solutions(robot, masses, r_max) if q is not None]
    Q = np.degrees(np.array(reached))
    return Q.min(axis=0), Q.max(axis=0)


def direct_drive_radius(robot: Robot, masses: list[PointMass]) -> float:
    """어깨 직결(STS3250)이 정격의 RATED_MARGIN 안에 드는 가장 먼 작업 반지름 [m] (1 cm 간격)."""
    ok = [
        r
        for r in np.arange(0.15, 0.31, 0.01)
        if worst_torques(robot, masses, float(r)).shoulder <= RATED_MARGIN * STS3250_RATED
    ]
    return float(max(ok, default=0.0))


def main() -> None:
    robot = ur_arm()
    shoulder_options = {
        "직결": STS3250_RATED,
        "2:1 벨트": 2 * STS3250_RATED * BELT_EFFICIENCY,
        "STS3250 두 개": 2 * STS3250_RATED * DUAL_SHARE,
    }

    print("## 도구가 아래를 향할 때 닿는 거리 (UR형)")
    for upper, fore in [(L1, L2), (0.17, 0.15)]:
        arm = ur_arm(upper, fore)
        reach = ", ".join(
            f"TCP 높이 {z * 100:.0f} cm에서 {max_reach(arm, z) * 100:.1f} cm" for z in TASK_HEIGHTS
        )
        print(f"- 위팔 {upper * 100:.0f} cm, 아래팔 {fore * 100:.0f} cm: {reach}")

    print(
        f"\n## 최대 버팀 토크와 정격 대비 비율 (위팔 {L1 * 100:.0f} cm, 아래팔 {L2 * 100:.0f} cm,"
        f" 목표는 {RATED_MARGIN} 이하)"
    )
    print(
        "| 반지름 | 질량 | 짐 | 어깨 [N·m] | 직결 | 2:1 벨트 | 두 개 | 팔꿈치 STS3215 | 팔꿈치 STS3250 | 못 닿음 |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r_max, case, payload in itertools.product((0.25, 0.30), MASS_CASES, (0.0, 0.1)):
        proposed = worst_torques(robot, point_masses(robot, case, payload), r_max)
        with_3215 = worst_torques(
            robot, point_masses(robot, case, payload, elbow_mass=STS3215_MASS), r_max
        )
        ratios = " | ".join(f"{proposed.shoulder / cap:.2f}" for cap in shoulder_options.values())
        print(
            f"| {r_max * 100:.0f} cm | {case.name} | {payload * 1000:.0f} g | {proposed.shoulder:.2f} |"
            f" {ratios} | {with_3215.elbow / STS3215_RATED:.2f} | {proposed.elbow / STS3250_RATED:.2f}"
            f" | {proposed.unreachable} |"
        )

    print(f"\n## 어깨 직결이 정격의 {RATED_MARGIN} 안에 드는 가장 먼 작업 반지름 (짐 50 g)")
    for case in MASS_CASES:
        radius = direct_drive_radius(robot, point_masses(robot, case, 0.05))
        print(
            f"- {case.name}: {radius * 100:.0f} cm"
            if radius
            else f"- {case.name}: 15 cm에서도 넘는다"
        )

    load = point_masses(robot, MASS_CASES[0], 0.05)
    print("\n## 박스 쌓기 자세에서 쓰는 관절 범위 (반지름 12~30 cm, 박스 돌림 ±90°)")
    print("- J1: 작업 방위만큼 돈다 (방위는 J1만 바꾼다)")
    lo, hi = used_ranges_deg(robot, load, 0.30)
    for i in range(1, 6):
        print(f"- J{i + 1}: {lo[i]:.0f}° ~ {hi[i]:.0f}°")

    print("\n## 중력 토크의 방향: 한쪽 방향이면 백래시가 한쪽으로 눌린다 (반지름 12~30 cm) [N·m]")
    for case, payload in itertools.product(MASS_CASES, (0.0, 0.1)):
        taus = task_torques(robot, point_masses(robot, case, payload))
        print(
            f"- {case.name}, 짐 {payload * 1000:.0f} g: "
            + ", ".join(
                f"J{j + 1} {taus[:, j].min():+.2f} ~ {taus[:, j].max():+.2f}" for j in (1, 2, 3, 4)
            )
        )

    print("\n## 백래시 ±0.87°가 TCP를 흔드는 양 (반지름 30 cm, TCP 높이 4 cm) [mm]")
    q = pick_solution(robot, AnalyticIK(robot), tool_down(0.30, 0.04), load)
    assert q is not None
    moves = backlash_displacements(robot, q, np.full(6, BACKLASH), tcp=(0.0, 0.0, LT))
    sizes = np.linalg.norm(moves, axis=1) * 1000
    print("- 관절마다: " + ", ".join(f"J{i + 1} {e:.1f}" for i, e in enumerate(sizes)))
    floating = [0, 4]  # 도구가 아래일 때 중력이 누르지 않는 관절 (UR형: J1, J5)
    print(
        "- 중력이 누르지 않는 J1, J5가 가장 나쁘게 겹칠 때: "
        f"{worst_backlash_error(moves[floating]) * 1000:.1f}"
    )


if __name__ == "__main__":
    main()
