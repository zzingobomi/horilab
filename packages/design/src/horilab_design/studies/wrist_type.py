"""손목 형식 비교: 구형 손목(롤-피치-롤)과 UR형, 그리고 D405 자리.

docs/decisions/0013-wrist-type.md의 숫자를 다시 만든다.

    uv run python -m horilab_design.studies.wrist_type

어깨, 위팔, 아래팔, 서보와 플랜지 끝의 질량은 같게 두고 손목만 바꾼다 (common.py).
치수는 STS3215 크기를 생각한 어림값이고, 실제 배치는 골격 설계에서 정한다. 비교하는 것:

1. 박스 쌓기 자세(도구가 아래, 반지름 12~30 cm, 박스 돌림 ±90°)에 닿는 비율
2. 손목 관절의 백래시(±0.87°)가 TCP를 흔드는 양
3. 손목 관절의 중력 토크: 0이면 백래시가 떠 있다
4. 특이점과의 거리: 손목은 J4와 J6가 나란해질 때, 팔꿈치는 팔이 곧게 펴질 때
5. 한 점에 TCP를 둔 채 도구를 기울일 수 있는 범위 (포인터 시험)
6. D405를 비켜 달 때 손목 백래시를 누르는 방향이 박스 돌림과 상관없이 같은지
"""

from __future__ import annotations

import itertools

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Mat, Robot, rotation
from horilab_design.statics import FLANGE, PointMass, backlash_errors, holding_torques
from horilab_design.studies.common import (
    BACKLASH,
    DEG,
    LT,
    MASS_CASES,
    UR_OFFSET,
    Y,
    Z,
    pick_solution,
    point_masses,
    task_poses,
    task_solutions,
    tool_down,
    ur_arm,
)
from horilab_design.studies.common import spherical_arm as spherical


def masses(robot: Robot, payload: float = 0.05) -> list[PointMass]:
    return point_masses(robot, MASS_CASES[0], payload)


def tilted(radius: float, height: float, tilt: float, azimuth: float) -> Mat:
    """TCP를 한 점에 두고 도구를 아래 방향에서 tilt만큼, azimuth 쪽으로 기울인 플랜지 자세."""
    T = tool_down(radius, height)
    tcp = T[:3, :3] @ np.array([0.0, 0.0, LT]) + T[:3, 3]
    R = rotation(Z, azimuth) @ rotation(Y, tilt) @ rotation(Z, -azimuth) @ T[:3, :3]
    T[:3, :3] = R
    T[:3, 3] = tcp - R @ np.array([0.0, 0.0, LT])
    return T


def torque_ranges(robot: Robot, load: list[PointMass], joints: tuple[int, ...]) -> Mat:
    """박스 쌓기 자세들에서 관절별 중력 토크의 [최소, 최대] [N·m]."""
    reached = [q for q in task_solutions(robot, load, 0.30) if q is not None]
    taus = np.array([holding_torques(robot, q, load) for q in reached])
    return np.array([[taus[:, j].min(), taus[:, j].max()] for j in joints])


def pointer_test(robot: Robot) -> tuple[int, float]:
    """TCP를 (반지름 22 cm, 높이 10 cm)에 둔 채 모든 방향으로 기울일 수 있는 최대 각 [deg]과 닿는 비율."""
    ik = AnalyticIK(robot)
    results = [
        (tilt, len(ik.solve_in_limits(tilted(0.22, 0.10, tilt * DEG, azimuth * DEG))) > 0)
        for tilt, azimuth in itertools.product(range(0, 91, 10), range(0, 360, 30))
    ]
    full_cone = max(
        (t for t in range(0, 91, 10) if all(ok for tt, ok in results if tt <= t)), default=-10
    )
    return full_cone, float(np.mean([ok for _, ok in results]))


def main() -> None:
    for robot in (spherical(), ur_arm()):
        ik = AnalyticIK(robot)
        load = masses(robot)
        poses = list(task_poses(0.30))
        solutions = task_solutions(robot, load, 0.30)
        reached = [q for q in solutions if q is not None]
        print(f"\n## {robot.name} (EAIK: {ik.family})")
        print(f"- 박스 쌓기 자세에 닿음: {len(reached)}/{len(poses)}")

        q30 = pick_solution(robot, ik, tool_down(0.30, 0.04), load)
        if q30 is not None:
            errors = backlash_errors(robot, q30, np.full(6, BACKLASH), tcp=(0.0, 0.0, LT)) * 1000
            print(
                f"- 손목 백래시가 흔드는 양 (반지름 30 cm) [mm]: "
                f"J4 {errors[3]:.1f}, J5 {errors[4]:.1f}, J6 {errors[5]:.1f}"
            )

        ranges = torque_ranges(robot, load, (3, 4, 5))
        print(
            "- 손목 중력 토크 (박스 돌림 포함) [N·m]: "
            + ", ".join(
                f"J{j} {lo:+.3f} ~ {hi:+.3f}" for j, (lo, hi) in zip((4, 5, 6), ranges, strict=True)
            )
        )

        Q = np.array(reached)
        print(
            f"- 특이점과의 거리: 손목 |sin J5| 최소 {np.abs(np.sin(Q[:, 4])).min():.2f}, "
            f"팔꿈치 |sin J3| 최소 {np.abs(np.sin(Q[:, 2])).min():.2f}"
        )

        cone, share = pointer_test(robot)
        print(
            f"- 포인터 시험 (TCP 반지름 22 cm, 높이 10 cm): 모든 방향으로 {cone}°까지 기울어짐, "
            f"0~90° 전체의 {share * 100:.0f}%에 닿음"
        )

    print("\n## D405(60 g)를 비켜 달 때 손목의 중력 토크 (박스 돌림 ±90° 포함) [N·m]")
    without_camera = masses  # 플랜지 끝 덩어리(150 g)에 D405가 들어 있어 60 g을 빼고 따로 단다
    cases = (
        (
            "구형 손목, 플랜지에 대각선 4.5 cm",
            spherical(),
            PointMass("D405", FLANGE, (0.032, 0.032, 0.01), 0.06),
        ),
        (
            "UR형, 플랜지에 x로 4.5 cm",
            ur_arm(),
            PointMass("D405", FLANGE, (0.045, 0.0, 0.01), 0.06),
        ),
        (
            "UR형, J6 앞 손목 하우징에 4.5 cm",
            ur_arm(),
            PointMass("D405", 5, (-0.045, 0.0, UR_OFFSET), 0.06),
        ),
    )
    for label, robot, camera in cases:
        base = [
            PointMass(p.label, p.link, p.position, p.mass - 0.06) if p.label == "플랜지 끝" else p
            for p in without_camera(robot)
        ]
        (j4_lo, j4_hi), (j5_lo, j5_hi) = torque_ranges(robot, [*base, camera], (3, 4))
        print(f"- {label}: J4 {j4_lo:+.3f} ~ {j4_hi:+.3f}, J5 {j5_lo:+.3f} ~ {j5_hi:+.3f}")


if __name__ == "__main__":
    main()
