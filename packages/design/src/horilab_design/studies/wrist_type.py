"""손목 형식 비교: 구형 손목(롤-피치-롤)과 UR형 (docs/design.md의 "기구 원칙").

    uv run python -m horilab_design.studies.wrist_type

어깨, 위팔, 아래팔, 서보와 플랜지 끝의 질량은 같게 두고 손목만 바꾼다. 치수는 STS3215
크기를 생각한 어림값이고, 실제 배치는 골격 설계에서 정한다. 비교하는 것:

1. 박스 쌓기 자세(도구가 아래, 반지름 12~30 cm)에 닿는 비율
2. 손목 관절의 백래시(±0.87°)가 TCP를 흔드는 양
3. 손목 관절의 중력 토크: 0이면 백래시가 떠 있다
4. 특이점과의 거리: 손목은 J4와 J6가 나란해질 때, 팔꿈치는 팔이 곧게 펴질 때
5. 한 점에 TCP를 둔 채 도구를 기울일 수 있는 범위 (포인터 시험)
"""

from __future__ import annotations

import itertools

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Joint, Mat, Robot, rotation
from horilab_design.statics import FLANGE, PointMass, backlash_errors, holding_torques
from horilab_design.studies.drive_torque import (
    BACKLASH,
    DEG,
    H0,
    LT,
    MASS_CASES,
    STS3215_MASS,
    STS3250_MASS,
    task_poses,
    tool_down,
    within_limits,
)

X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)
Z = (0.0, 0.0, 1.0)
L1, L2 = 0.20, 0.18  # 위팔, 아래팔(어깨 쪽 관절 → J4) [m]
SPHERICAL_LW = 0.06  # 구형 손목: 손목 중심 → 플랜지 [m]
UR_OFFSET = 0.05  # UR형: J4→J5, J5→J6, J6→플랜지 오프셋 [m] (STS3215 하나 크기 어림)


def _joint(
    name: str,
    origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    limit_deg: float,
) -> Joint:
    return Joint(name, origin, axis, -limit_deg * DEG, limit_deg * DEG, effort=1.0, velocity=4.0)


def _arm_joints() -> tuple[Joint, ...]:
    return (
        _joint("j1", (0.0, 0.0, 0.05), Z, 175),
        _joint("j2", (0.0, 0.0, H0 - 0.05), Y, 120),
        _joint("j3", (0.0, 0.0, L1), Y, 150),
    )


def spherical() -> Robot:
    joints = (
        *_arm_joints(),
        _joint("j4", (0.0, 0.0, L2), Z, 175),
        _joint("j5", (0.0, 0.0, 0.0), Y, 125),
        _joint("j6", (0.0, 0.0, SPHERICAL_LW), Z, 175),
    )
    return Robot("spherical", joints, np.eye(4))


def ur_type() -> Robot:
    # J2, J3, J4가 나란하다(y). J5는 영점에서 아래팔 방향(z), J6는 다시 y. 도구는 J6 축 방향이다.
    joints = (
        *_arm_joints(),
        _joint("j4", (0.0, 0.0, L2), Y, 175),
        _joint("j5", (0.0, UR_OFFSET, 0.0), Z, 175),
        _joint("j6", (0.0, 0.0, UR_OFFSET), Y, 175),
    )
    flange = np.eye(4)
    flange[:3, :3] = rotation(X, -np.pi / 2)  # 플랜지 z가 J6 축(y)을 본다
    flange[:3, 3] = (0.0, UR_OFFSET, 0.0)
    return Robot("ur_type", joints, flange)


def masses(robot: Robot, payload: float = 0.05) -> list[PointMass]:
    """두 형식에 같은 질량을 둔다. 손목 서보는 각 관절 원점에, 플랜지 끝은 플랜지 3 cm 앞에."""
    case = MASS_CASES[0]
    j4, j5, j6 = robot.joints[3:]
    return [
        PointMass("어깨 서보", 1, (0.0, 0.0, H0 - 0.05), STS3250_MASS + case.housing),
        PointMass("위팔", 2, (0.0, 0.0, L1 / 2), case.link_per_m * L1),
        PointMass("팔꿈치 서보", 2, (0.0, 0.0, L1), STS3250_MASS + case.housing),
        PointMass("아래팔", 3, (0.0, 0.0, L2 / 2), case.link_per_m * L2),
        PointMass("J4 서보", 3, j4.origin, STS3215_MASS + case.housing),
        PointMass("J5 서보", 4, j5.origin, STS3215_MASS + case.housing),
        PointMass("J6 서보", 5, j6.origin, STS3215_MASS + 0.7 * case.housing),
        PointMass("플랜지 끝", FLANGE, (0.0, 0.0, 0.03), case.end_effector),
        PointMass("짐", FLANGE, (0.0, 0.0, LT), payload),
    ]


def best_solution(robot: Robot, ik: AnalyticIK, T: Mat, load: list[PointMass]) -> Mat | None:
    """관절 한계 안의 해 중에서 고른다. 손목은 늘 같은 쪽(J5 > 0)을 쓰고, 그중 어깨 토크가 가장 작은 해.

    손목을 뒤집은 해를 섞어 쓰면 손목 관절의 중력 토크 방향이 자세마다 바뀌어 백래시가 뒤집힌다.
    """
    solutions = [q for q in ik.solve(T) if within_limits(robot, q)]
    same_side = [q for q in solutions if q[4] > 0]
    candidates = same_side or solutions
    if not candidates:
        return None
    return min(candidates, key=lambda q: abs(holding_torques(robot, q, load)[1]))


def tilted(radius: float, height: float, tilt: float, azimuth: float) -> Mat:
    """TCP를 한 점에 두고 도구를 아래 방향에서 tilt만큼, azimuth 쪽으로 기울인 플랜지 자세."""
    T = tool_down(radius, 0.0, height)
    tcp = T[:3, :3] @ np.array([0.0, 0.0, LT]) + T[:3, 3]
    R = rotation(Z, azimuth) @ rotation(Y, tilt) @ rotation(Z, -azimuth) @ T[:3, :3]
    T[:3, :3] = R
    T[:3, 3] = tcp - R @ np.array([0.0, 0.0, LT])
    return T


def yawed(T: Mat, yaw: float) -> Mat:
    """같은 TCP에서 도구를 축 둘레로 yaw만큼 돌린 자세 (박스를 돌려 놓을 때 J6가 도는 것)."""
    out = T.copy()
    tcp = T[:3, :3] @ np.array([0.0, 0.0, LT]) + T[:3, 3]
    out[:3, :3] = T[:3, :3] @ rotation(Z, yaw)
    out[:3, 3] = tcp - out[:3, :3] @ np.array([0.0, 0.0, LT])
    return out


def camera_preload(robot: Robot, camera: PointMass, yaw_deg: float) -> Mat:
    """D405를 더한 질량으로, 도구를 yaw만큼 돌린 작업 자세들에서 J4, J5의 중력 토크 범위 [N·m]."""
    ik = AnalyticIK(robot)
    load = [*masses(robot), camera]
    solutions = [best_solution(robot, ik, yawed(T, yaw_deg * DEG), load) for T in task_poses(0.30)]
    taus = np.array([holding_torques(robot, q, load) for q in solutions if q is not None])
    return np.array([[taus[:, j].min(), taus[:, j].max()] for j in (3, 4)])


def main() -> None:
    _compare()
    print("\n## D405(60 g)를 비켜 달 때, J6를 돌리면 손목 중력 토크가 어떻게 바뀌나 [N·m]")
    cases = (
        (
            "구형 손목, 플랜지에 대각선 4.5 cm",
            spherical(),
            PointMass("D405", FLANGE, (0.032, 0.032, 0.01), 0.06),
        ),
        (
            "UR형, 플랜지에 x로 4.5 cm",
            ur_type(),
            PointMass("D405", FLANGE, (0.045, 0.0, 0.01), 0.06),
        ),
        (
            "UR형, J6 앞 손목 하우징에 4.5 cm",
            ur_type(),
            PointMass("D405", 5, (-0.045, 0.0, UR_OFFSET), 0.06),
        ),
    )
    for label, robot, camera in cases:
        print(f"- {label}")
        for yaw in (-90, -45, 0, 45, 90):
            (j4_lo, j4_hi), (j5_lo, j5_hi) = camera_preload(robot, camera, yaw)
            print(
                f"  - J6 {yaw:+d}°: J4 {j4_lo:+.3f} ~ {j4_hi:+.3f}, J5 {j5_lo:+.3f} ~ {j5_hi:+.3f}"
            )


def _compare() -> None:
    for robot in (spherical(), ur_type()):
        ik = AnalyticIK(robot)
        load = masses(robot)
        poses = list(task_poses(0.30))
        chosen = [best_solution(robot, ik, T, load) for T in poses]
        reached = [q for q in chosen if q is not None]
        print(f"\n## {robot.name} (EAIK: {ik.family})")
        print(f"- 박스 쌓기 자세에 닿음: {len(reached)}/{len(poses)}")

        q30 = best_solution(robot, ik, tool_down(0.30, 0.0, 0.04), load)
        if q30 is not None:
            errors = backlash_errors(robot, q30, np.full(6, BACKLASH), tcp=(0.0, 0.0, LT)) * 1000
            print(
                f"- 손목 백래시가 흔드는 양 (반지름 30 cm) [mm]: J4 {errors[3]:.1f}, J5 {errors[4]:.1f}, J6 {errors[5]:.1f}"
            )

        taus = np.array([holding_torques(robot, q, load) for q in reached])
        print(
            "- 손목 중력 토크 [N·m]: "
            + ", ".join(
                f"J{j + 1} {taus[:, j].min():+.3f} ~ {taus[:, j].max():+.3f}" for j in (3, 4, 5)
            )
        )

        Q = np.array(reached)
        print(
            f"- 특이점과의 거리: 손목 |sin J5| 최소 {np.abs(np.sin(Q[:, 4])).min():.2f}, "
            f"팔꿈치 |sin J3| 최소 {np.abs(np.sin(Q[:, 2])).min():.2f}"
        )

        reachable = []
        for tilt_deg, azimuth_deg in itertools.product(range(0, 91, 10), range(0, 360, 30)):
            T = tilted(0.22, 0.10, tilt_deg * DEG, azimuth_deg * DEG)
            ok = any(within_limits(robot, q) for q in ik.solve(T))
            reachable.append((tilt_deg, ok))
        full_cone = max(
            (t for t in range(0, 91, 10) if all(ok for tt, ok in reachable if tt <= t)), default=-10
        )
        share = np.mean([ok for _, ok in reachable])
        print(
            f"- 포인터 시험 (TCP 반지름 22 cm, 높이 10 cm): 모든 방향으로 {full_cone}°까지 기울어짐, "
            f"0~90° 전체의 {share * 100:.0f}%에 닿음"
        )


if __name__ == "__main__":
    main()
