"""골격 설계 전 구동 검토: 관절 토크, 관절 범위, 백래시 끝점 오차를 어림한다.

docs/decisions/0011-joint-drives.md의 숫자를 다시 만든다.

    uv run python -m horilab_design.studies.drive_torque

- 팔은 구형 손목 막대 모델이다. 실제 축 배치와 길이는 골격 설계에서 정한다.
- 목표 자세는 박스 쌓기처럼 도구가 아래를 향한 자세다. 해석적 IK로 모든 해를 구하고,
  관절 한계 안의 해 중에서 어깨 토크가 가장 작은 해를 고른다.
- 질량은 점 질량으로 어림한 가정이다 (MASS_CASES). 실물을 재면 바꾼다.
"""

from __future__ import annotations

import itertools
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Joint, Mat, Robot, rotation
from horilab_design.statics import FLANGE, PointMass, backlash_errors, holding_torques

DEG = np.pi / 180
Y = (0.0, 1.0, 0.0)
Z = (0.0, 0.0, 1.0)

# 서보 사양 (docs/hardware.md, 12 V). 무게는 실물을 재면 바꾼다.
STS3215_MASS, STS3215_RATED = 0.055, 0.98  # [kg], [N·m] (정격 10 kg·cm)
STS3250_MASS, STS3250_RATED = 0.0745, 1.57  # [kg], [N·m] (정격 16 kg·cm)
BELT_EFFICIENCY = 0.95  # 2:1 벨트 감속의 효율 (가정)
DUAL_SHARE = 0.9  # 서보 두 개가 한 관절을 같이 돌릴 때 나눠 갖는 비율 (가정, 서로 밀어내는 손실)
BACKLASH = 0.87 * DEG  # 이전 프로젝트에서 잰 STS3215 백래시 (±, docs/background.md)

# 막대 모델의 고정 치수 [m]: 바닥→어깨 높이, 손목 중심→플랜지, 플랜지→TCP(흡착 그리퍼 가정)
H0, LW, LT = 0.12, 0.06, 0.06
TASK_HEIGHTS = (0.04, 0.16)  # TCP 높이: 탁자 위 박스 윗면, 세 층 쌓은 박스 윗면 (가정)


@dataclass(frozen=True)
class MassCase:
    """질량 가정. 링크는 길이당 질량, 관절마다 하우징·베어링·나사, 플랜지 끝은 한 덩어리로 본다."""

    name: str
    link_per_m: float  # 상자 단면 링크 + 지나가는 케이블 [kg/m]
    housing: float  # 관절 하나의 하우징, 베어링, 나사 [kg]
    end_effector: float  # 로봇 쪽 플랜지 + D405 + 흡착 그리퍼 [kg]


MASS_CASES = (
    MassCase("가벼움", link_per_m=0.35, housing=0.04, end_effector=0.15),
    MassCase("무거움", link_per_m=0.50, housing=0.07, end_effector=0.22),
)


def stick_arm(L1: float, L2: float) -> Robot:
    """구형 손목 막대 모델. 영점에서 팔이 위로 곧게 선다. L1: 위팔, L2: 아래팔(→손목 중심)."""

    def joint(
        name: str,
        origin: tuple[float, float, float],
        axis: tuple[float, float, float],
        limit_deg: float,
    ) -> Joint:
        return Joint(
            name, origin, axis, -limit_deg * DEG, limit_deg * DEG, effort=1.0, velocity=4.0
        )

    joints = (
        joint("j1", (0.0, 0.0, 0.05), Z, 175),
        joint("j2", (0.0, 0.0, H0 - 0.05), Y, 120),
        joint("j3", (0.0, 0.0, L1), Y, 150),
        joint("j4", (0.0, 0.0, L2), Z, 175),
        joint("j5", (0.0, 0.0, 0.0), Y, 125),
        joint("j6", (0.0, 0.0, LW), Z, 175),
    )
    return Robot("drive_study", joints, np.eye(4))


def point_masses(
    L1: float, L2: float, case: MassCase, payload: float, *, elbow_mass: float = STS3215_MASS
) -> list[PointMass]:
    """서보는 관절 위에, 링크는 가운데에, 플랜지 끝 덩어리는 플랜지 3 cm 앞에, 짐은 TCP에 둔다."""
    return [
        PointMass("어깨 서보", 1, (0.0, 0.0, H0 - 0.05), STS3250_MASS + case.housing),
        PointMass("위팔", 2, (0.0, 0.0, L1 / 2), case.link_per_m * L1),
        PointMass("팔꿈치 서보", 2, (0.0, 0.0, L1), elbow_mass + case.housing),
        PointMass("아래팔", 3, (0.0, 0.0, L2 / 2), case.link_per_m * L2),
        PointMass("J4 서보", 3, (0.0, 0.0, L2 - 0.04), STS3215_MASS + case.housing),
        PointMass("J5 서보", 4, (0.0, 0.0, 0.0), STS3215_MASS + case.housing),
        PointMass("J6 서보", 5, (0.0, 0.0, LW / 2), STS3215_MASS + 0.7 * case.housing),
        PointMass("플랜지 끝", FLANGE, (0.0, 0.0, 0.03), case.end_effector),
        PointMass("짐", FLANGE, (0.0, 0.0, LT), payload),
    ]


def tool_down(radius: float, azimuth: float, height: float) -> Mat:
    """TCP를 (반지름, 방위각, 높이)에 두고 도구가 아래를 향한 플랜지 자세. 플랜지 x는 베이스 쪽이다."""
    T = np.eye(4)
    T[:3, :3] = rotation(Z, azimuth) @ rotation(Y, np.pi)
    tcp = np.array([radius * np.cos(azimuth), radius * np.sin(azimuth), height])
    T[:3, 3] = tcp - T[:3, :3] @ np.array([0.0, 0.0, LT])
    return T


def task_poses(r_max: float, *, r_min: float = 0.12, steps: int = 9) -> Iterator[Mat]:
    for r, azimuth, z in itertools.product(
        np.linspace(r_min, r_max, steps), np.linspace(-80, 80, steps) * DEG, TASK_HEIGHTS
    ):
        yield tool_down(float(r), float(azimuth), z)


def within_limits(robot: Robot, q: Mat) -> bool:
    return bool(np.all(q >= robot.lower) and np.all(q <= robot.upper))


def lowest_shoulder_solution(
    robot: Robot, ik: AnalyticIK, T: Mat, masses: list[PointMass]
) -> Mat | None:
    solutions = [q for q in ik.solve(T) if within_limits(robot, q)]
    if not solutions:
        return None
    return min(solutions, key=lambda q: abs(holding_torques(robot, q, masses)[1]))


def max_reach(robot: Robot, height: float) -> float:
    """도구가 아래를 향한 채 TCP가 닿는 가장 먼 반지름 [m] (5 mm 간격)."""
    ik = AnalyticIK(robot)
    reachable = [
        r
        for r in np.arange(0.10, 0.60, 0.005)
        if any(within_limits(robot, q) for q in ik.solve(tool_down(float(r), 0.0, height)))
    ]
    return float(max(reachable, default=0.0))


@dataclass(frozen=True)
class Worst:
    shoulder: float  # 어깨(J2)가 버티는 최대 토크 [N·m]
    elbow: float  # 팔꿈치(J3)가 버티는 최대 토크 [N·m]
    unreachable: int  # 닿지 않은 목표 수


def worst_torques(robot: Robot, masses: list[PointMass], r_max: float, steps: int = 9) -> Worst:
    ik = AnalyticIK(robot)
    shoulder = elbow = 0.0
    unreachable = 0
    for T in task_poses(r_max, steps=steps):
        q = lowest_shoulder_solution(robot, ik, T, masses)
        if q is None:
            unreachable += 1
            continue
        tau = np.abs(holding_torques(robot, q, masses))
        shoulder, elbow = max(shoulder, tau[1]), max(elbow, tau[2])
    return Worst(shoulder, elbow, unreachable)


def used_ranges(robot: Robot, masses: list[PointMass], r_max: float) -> tuple[Mat, Mat]:
    """목표 자세들에서 실제로 쓰는 관절 각의 최솟값과 최댓값 [deg]."""
    ik = AnalyticIK(robot)
    chosen = [
        q
        for T in task_poses(r_max)
        if (q := lowest_shoulder_solution(robot, ik, T, masses)) is not None
    ]
    Q = np.degrees(np.array(chosen))
    return Q.min(axis=0), Q.max(axis=0)


def main() -> None:
    shoulder_options = {
        "직결": STS3250_RATED,
        "2:1 벨트": 2 * STS3250_RATED * BELT_EFFICIENCY,
        "STS3250 두 개": 2 * STS3250_RATED * DUAL_SHARE,
    }
    print("## 도구가 아래를 향할 때 닿는 거리")
    for L1, L2 in [(0.20, 0.18), (0.17, 0.15)]:
        robot = stick_arm(L1, L2)
        reach = ", ".join(
            f"TCP 높이 {z * 100:.0f} cm에서 {max_reach(robot, z) * 100:.1f} cm"
            for z in TASK_HEIGHTS
        )
        print(f"- 위팔 {L1 * 100:.0f} cm, 아래팔 {L2 * 100:.0f} cm: {reach}")

    print("\n## 최대 버팀 토크와 정격 대비 비율 (위팔 20 cm, 아래팔 18 cm)")
    print(
        "| 반지름 | 질량 | 짐 | 어깨 [N·m] | 직결 | 2:1 벨트 | 두 개 | 팔꿈치 [N·m] | STS3215 | STS3250 |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|")
    L1, L2 = 0.20, 0.18
    robot = stick_arm(L1, L2)
    for r_max, case, payload in itertools.product((0.25, 0.30), MASS_CASES, (0.0, 0.1)):
        worst = worst_torques(robot, point_masses(L1, L2, case, payload), r_max)
        ratios = " | ".join(f"{worst.shoulder / cap:.2f}" for cap in shoulder_options.values())
        print(
            f"| {r_max * 100:.0f} cm | {case.name} | {payload * 1000:.0f} g | {worst.shoulder:.2f} | {ratios} | "
            f"{worst.elbow:.2f} | {worst.elbow / STS3215_RATED:.2f} | {worst.elbow / STS3250_RATED:.2f} |"
        )

    print("\n## 박스 쌓기 자세에서 쓰는 관절 범위 (반지름 12~30 cm, 방위 ±80°)")
    lo, hi = used_ranges(robot, point_masses(L1, L2, MASS_CASES[0], 0.05), 0.30)
    for i in range(6):
        print(f"- J{i + 1}: {lo[i]:.0f}° ~ {hi[i]:.0f}°")

    print("\n## 백래시 ±0.87°가 TCP를 흔드는 양 (반지름 30 cm, TCP 높이 4 cm) [mm]")
    q = lowest_shoulder_solution(
        robot,
        AnalyticIK(robot),
        tool_down(0.30, 0.0, 0.04),
        point_masses(L1, L2, MASS_CASES[0], 0.05),
    )
    assert q is not None
    errors = backlash_errors(robot, q, np.full(6, BACKLASH), tcp=(0.0, 0.0, LT)) * 1000
    print("- " + ", ".join(f"J{i + 1} {e:.1f}" for i, e in enumerate(errors)))

    print("\n## 중력 토크의 방향: 한쪽 방향이면 백래시가 한쪽으로 눌린다 (반지름 12~30 cm) [N·m]")
    for case, payload in itertools.product(MASS_CASES, (0.0, 0.1)):
        taus = _task_torques(robot, point_masses(L1, L2, case, payload))
        print(
            f"- {case.name}, 짐 {payload * 1000:.0f} g: "
            + ", ".join(
                f"J{j + 1} {taus[:, j].min():+.2f} ~ {taus[:, j].max():+.2f}" for j in (1, 2, 3, 4)
            )
        )

    print("\n## D405(60 g)를 플랜지 중심에서 비켜 달 때 손목의 중력 토크 (짐 50 g) [N·m]")
    without_camera = MassCase("가벼움, D405 따로", link_per_m=0.35, housing=0.04, end_effector=0.09)
    for label, offset in (
        ("가운데", (0.0, 0.0, 0.01)),
        ("플랜지 x로 4.5 cm", (0.045, 0.0, 0.01)),
        ("대각선으로 4.5 cm", (0.032, 0.032, 0.01)),
    ):
        masses = [
            *point_masses(L1, L2, without_camera, 0.05),
            PointMass("D405", FLANGE, offset, 0.06),
        ]
        taus = _task_torques(robot, masses)
        print(
            f"- {label}: "
            + ", ".join(
                f"J{j + 1} {taus[:, j].min():+.3f} ~ {taus[:, j].max():+.3f}" for j in (3, 4)
            )
        )


def _task_torques(robot: Robot, masses: list[PointMass]) -> Mat:
    """작업 자세마다(어깨 토크가 가장 작은 해) 관절이 버티는 토크 [N·m]. 행이 자세다."""
    ik = AnalyticIK(robot)
    solutions = [lowest_shoulder_solution(robot, ik, T, masses) for T in task_poses(0.30)]
    return np.array([holding_torques(robot, q, masses) for q in solutions if q is not None])


if __name__ == "__main__":
    main()
