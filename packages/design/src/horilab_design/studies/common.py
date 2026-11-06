"""검토 코드가 같이 쓰는 가정과 도구: 막대 모델, 질량 가정, 박스 쌓기 목표 자세.

여기 있는 숫자는 골격 설계 전의 어림값이다. 실물을 재거나 설계가 정해지면 바꾼다.
"""

from __future__ import annotations

import itertools
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Joint, Mat, Robot, Vec3, rotation
from horilab_design.statics import FLANGE, PointMass, holding_torques

DEG = np.pi / 180
X: Vec3 = (1.0, 0.0, 0.0)
Y: Vec3 = (0.0, 1.0, 0.0)
Z: Vec3 = (0.0, 0.0, 1.0)

# 서보 사양 (docs/hardware.md, 12 V). 무게는 실물을 재면 바꾼다.
STS3215_MASS, STS3215_RATED = 0.055, 0.98  # [kg], [N·m] (정격 10 kg·cm)
STS3250_MASS, STS3250_RATED = 0.0745, 1.57  # [kg], [N·m] (정격 16 kg·cm)
RATED_MARGIN = 0.8  # 정지해 버틸 때 정격 토크의 이 비율까지만 쓴다. 나머지는 가속과 열의 몫이다
BELT_EFFICIENCY = 0.95  # 2:1 벨트 감속의 효율 (가정)
DUAL_SHARE = 0.9  # 서보 두 개가 한 관절을 같이 돌릴 때 나눠 갖는 비율 (가정, 서로 밀어내는 손실)
BACKLASH = 0.87 * DEG  # 이전 프로젝트에서 잰 STS3215 백래시 (±, docs/background.md). 다시 재야 한다

# 막대 모델의 고정 치수 [m]: 바닥→어깨 높이, 플랜지→TCP(흡착 그리퍼 가정)
H0, LT = 0.12, 0.06
L1, L2 = 0.20, 0.18  # 위팔, 아래팔(어깨 쪽 관절 → J4)
SPHERICAL_LW = 0.06  # 구형 손목: 손목 중심 → 플랜지
UR_OFFSET = 0.05  # UR형: J4→J5, J5→J6, J6→플랜지 오프셋 (STS3215 하나 크기 어림)

TASK_HEIGHTS = (0.04, 0.16)  # TCP 높이: 탁자 위 박스 윗면, 세 층 쌓은 박스 윗면 (가정)
TASK_YAWS_DEG = (-90, -45, 0, 45, 90)  # 박스를 돌려 놓는 각도 = 도구를 축 둘레로 돌리는 각도


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


def _joint(name: str, origin: Vec3, axis: Vec3, limit_deg: float) -> Joint:
    return Joint(name, origin, axis, -limit_deg * DEG, limit_deg * DEG, effort=1.0, velocity=4.0)


def _arm_joints(L1: float) -> tuple[Joint, ...]:
    return (
        _joint("j1", (0.0, 0.0, 0.05), Z, 175),
        _joint("j2", (0.0, 0.0, H0 - 0.05), Y, 120),
        _joint("j3", (0.0, 0.0, L1), Y, 150),
    )


def spherical_arm(L1: float = L1, L2: float = L2, lw: float = SPHERICAL_LW) -> Robot:
    """구형 손목(롤-피치-롤) 막대 모델. 영점에서 팔이 위로 곧게 선다."""
    joints = (
        *_arm_joints(L1),
        _joint("j4", (0.0, 0.0, L2), Z, 175),
        _joint("j5", (0.0, 0.0, 0.0), Y, 125),
        _joint("j6", (0.0, 0.0, lw), Z, 175),
    )
    return Robot("spherical", joints, np.eye(4))


def ur_arm(L1: float = L1, L2: float = L2, offset: float = UR_OFFSET) -> Robot:
    """UR형 막대 모델. J2, J3, J4가 나란하고(y), J5는 영점에서 아래팔 방향(z), J6는 다시 y다.

    서보는 한 바퀴(360°) 안에서만 돌므로, 관절마다 작업에 쓰는 각도가 범위 가운데에 오도록
    영점(서보 혼 방향)을 정한다. 도구가 아래를 향한 자세에서 J4는 180° 둘레를 쓰므로 범위를
    5°~355°로 둔다.
    """
    j4 = Joint("j4", (0.0, 0.0, L2), Y, 5 * DEG, 355 * DEG, effort=1.0, velocity=4.0)
    joints = (
        *_arm_joints(L1),
        j4,
        _joint("j5", (0.0, offset, 0.0), Z, 175),
        _joint("j6", (0.0, 0.0, offset), Y, 175),
    )
    flange = np.eye(4)
    # 플랜지 z(도구 방향)가 J6 축(y)을 보게 하고, 도구 축 둘레로 -90° 더 돌려서 기본 작업 자세
    # (박스 돌림 0°)에서 J6가 범위 가운데에 오게 한다. 그래야 박스를 양쪽으로 똑같이 돌릴 수 있다.
    flange[:3, :3] = rotation(X, -np.pi / 2) @ rotation(Z, -np.pi / 2)
    flange[:3, 3] = (0.0, offset, 0.0)
    return Robot("ur_type", joints, flange)


def point_masses(
    robot: Robot,
    case: MassCase,
    payload: float,
    *,
    elbow_mass: float = STS3250_MASS,
) -> list[PointMass]:
    """서보는 관절 원점에, 링크는 가운데에, 플랜지 끝 덩어리는 플랜지 3 cm 앞에, 짐은 TCP에 둔다."""
    j3, j4, j5, j6 = robot.joints[2:]
    upper, fore = np.linalg.norm(j3.origin), np.linalg.norm(j4.origin)
    return [
        PointMass("어깨 서보", 1, robot.joints[1].origin, STS3250_MASS + case.housing),
        PointMass("위팔", 2, tuple(np.asarray(j3.origin) / 2), case.link_per_m * upper),
        PointMass("팔꿈치 서보", 2, j3.origin, elbow_mass + case.housing),
        PointMass("아래팔", 3, tuple(np.asarray(j4.origin) / 2), case.link_per_m * fore),
        PointMass("J4 서보", 3, j4.origin, STS3215_MASS + case.housing),
        PointMass("J5 서보", 4, j5.origin, STS3215_MASS + case.housing),
        PointMass("J6 서보", 5, j6.origin, STS3215_MASS + 0.7 * case.housing),
        PointMass("플랜지 끝", FLANGE, (0.0, 0.0, 0.03), case.end_effector),
        PointMass("짐", FLANGE, (0.0, 0.0, LT), payload),
    ]


def tool_down(radius: float, height: float, yaw: float = 0.0) -> Mat:
    """TCP를 (반지름, 0, 높이)에 두고 도구가 아래를 향한 플랜지 자세.

    yaw = 0이면 플랜지 x가 베이스 쪽이다. yaw는 박스를 돌려 놓을 때처럼 도구를 축 둘레로 돌린다.
    방위각은 J1만 바꾸고 나머지 관절에는 영향이 없어서 0으로 둔다.
    """
    T = np.eye(4)
    T[:3, :3] = rotation(Y, np.pi) @ rotation(Z, yaw)
    T[:3, 3] = np.array([radius, 0.0, height]) - T[:3, :3] @ np.array([0.0, 0.0, LT])
    return T


def task_poses(r_max: float, *, r_min: float = 0.12, steps: int = 9) -> Iterator[Mat]:
    """박스 쌓기 목표: 반지름 r_min~r_max, 두 높이, 박스 돌림 각도들."""
    for r, z, yaw in itertools.product(
        np.linspace(r_min, r_max, steps), TASK_HEIGHTS, TASK_YAWS_DEG
    ):
        yield tool_down(float(r), z, yaw * DEG)


def configuration_flags(q: Mat, target_azimuth: float) -> tuple[bool, bool, bool]:
    """산업용 로봇의 자세 구분처럼 해의 모양을 나눈다: (베이스가 뒤를 봄, 팔꿈치가 아래, 손목이 뒤집힘).

    모두 False인 해가 기본 자세다: 베이스가 목표 쪽을 보고, 팔꿈치는 위로(J3 > 0), 손목은 J5 > 0.
    """
    facing = abs((q[0] - target_azimuth + np.pi) % (2 * np.pi) - np.pi) < np.pi / 2
    return (not facing, not q[2] > 0, not q[4] > 0)


def pick_solution(robot: Robot, ik: AnalyticIK, T: Mat, masses: list[PointMass]) -> Mat | None:
    """관절 한계 안의 해 중에서 기본 자세(configuration_flags가 모두 False)에 가장 가까운 해를 고른다.

    실제 로봇은 작업 중에 팔꿈치나 손목을 뒤집지 않는다. 토크만 보고 고르면 팔꿈치를 아래로 꺾거나
    베이스를 반대로 돌린 해가 섞여서, 중력 토크의 방향과 관절 범위가 실제와 다르게 나온다.
    모양이 같은 해가 여럿이면 어깨 토크가 작은 해를 쓴다.
    """
    solutions = list(ik.solve_in_limits(T))
    if not solutions:
        return None
    tcp = T[:3, :3] @ np.array([0.0, 0.0, LT]) + T[:3, 3]
    azimuth = float(np.arctan2(tcp[1], tcp[0]))
    return min(
        solutions,
        key=lambda q: (
            sum(configuration_flags(q, azimuth)),
            configuration_flags(q, azimuth),
            abs(holding_torques(robot, q, masses)[1]),
        ),
    )


def task_solutions(robot: Robot, masses: list[PointMass], r_max: float) -> list[Mat | None]:
    ik = AnalyticIK(robot)
    return [pick_solution(robot, ik, T, masses) for T in task_poses(r_max)]
