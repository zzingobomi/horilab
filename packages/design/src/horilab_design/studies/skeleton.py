"""골격 설계 검토: 0014 배치의 목표 자세로 링크 길이를 고른다.

    uv run python -m horilab_design.studies.skeleton

- 목표 자세는 docs/decisions/0014-workspace-layout.md의 배치다: 집는 곳 하나와 2 × 2 팔레트 4층.
  박스 윗면에 TCP를 두는 자세와, 내려가기 전 3 cm 위 자세를 같이 본다.
- 박스는 책상 축에 맞춰 놓는다. 박스는 90°마다 같은 모양이라, J6가 덜 돌도록 도구 돌림을 ±45° 안에서
  고른다.
- 팔은 params/arm.toml의 UR형이고, 링크 길이와 손목 오프셋만 바꿔 가며 본다. 해는 작업 자세(J5 > 0)만
  쓴다 (common.pick_solution). 질량은 common.MASS_CASES의 무거운 가정에 짐 100 g이다.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Robot
from horilab_design.statics import holding_torques
from horilab_design.studies.common import (
    MASS_CASES,
    RATED_MARGIN,
    pick_solution,
    point_masses,
    reach_band,
    tool_down,
)
from horilab_design.values import ArmParams, arm_robot, load_arm, load_servos

BOX = 0.04  # 박스 한 변 [m]
PICK = (0.20, math.radians(60))  # 집는 곳: 반지름 [m], 방위 [rad]
PALLET = (0.21, math.radians(-45))  # 팔레트 중심
PALLET_PITCH = 0.042  # 박스 40 mm + 틈 2 mm
LAYERS = 4
APPROACH = 0.03  # 내려가기 전 위쪽 자세의 높이 [m]
PAYLOAD = 0.1  # [kg]


def box_yaw(azimuth: float) -> float:
    """책상 축에 맞춘 박스를 잡는 도구 돌림 (방위 기준 좌표계). 90°마다 같으므로 ±45° 안으로 접는다."""
    return (-azimuth + math.pi / 4) % (math.pi / 2) - math.pi / 4


def spots() -> list[tuple[float, float, int]]:
    """박스 자리 (x, y, 층 수): 집는 곳 하나와 팔레트 네 자리."""
    out = [(PICK[0] * math.cos(PICK[1]), PICK[0] * math.sin(PICK[1]), 1)]
    cx, cy = PALLET[0] * math.cos(PALLET[1]), PALLET[0] * math.sin(PALLET[1])
    half = PALLET_PITCH / 2
    for dx, dy in itertools.product((-half, half), repeat=2):
        out.append((cx + dx, cy + dy, LAYERS))
    return out


def targets() -> list[tuple[float, float, float]]:
    """(반지름, 높이, 도구 돌림) 목록. 높이는 박스 윗면과 그 위 APPROACH."""
    poses = []
    for x, y, layers in spots():
        for layer in range(1, layers + 1):
            for lift in (0.0, APPROACH):
                poses.append((math.hypot(x, y), layer * BOX + lift, box_yaw(math.atan2(y, x))))
    return poses


def with_lengths(
    params: ArmParams,
    upper: float,
    fore: float,
    offsets: tuple[float, float, float],
    shoulder: float | None = None,
) -> ArmParams:
    """길이를 바꾼 팔 값. shoulder(설치면 → J2 축)를 주지 않으면 값 파일 그대로다."""
    update = {
        "upper_arm_mm": upper * 1000,
        "forearm_mm": fore * 1000,
        "j4_to_j5_mm": offsets[0] * 1000,
        "j5_to_j6_mm": offsets[1] * 1000,
        "j6_to_flange_mm": offsets[2] * 1000,
    }
    if shoulder is not None:
        update["shoulder_height_mm"] = shoulder * 1000
    return params.model_copy(update={"arm": params.arm.model_copy(update=update)})


@dataclass(frozen=True)
class Check:
    reached: int
    total: int
    sin_j3: float  # 팔꿈치가 펴진 정도: |sin J3|의 최솟값 (0이면 다 펴짐)
    shoulder: float  # 어깨 서보 정격 대비 최대 버팀 토크
    elbow: float  # 팔꿈치 서보 정격 대비
    reach_low: float  # 작업 자세로 TCP 높이 4 cm에서 닿는 끝 [m]
    reach_high: float  # 16 cm에서


def check(robot: Robot, params: ArmParams) -> Check:
    servos = load_servos()
    masses = point_masses(robot, MASS_CASES[1], PAYLOAD)
    ik = AnalyticIK(robot)
    poses = targets()
    reached = [
        q
        for radius, height, yaw in poses
        for q in [pick_solution(robot, ik, tool_down(radius, height, yaw), masses)]
        if q is not None
    ]
    Q = np.array(reached) if reached else np.zeros((0, 6))
    taus = np.abs(np.array([holding_torques(robot, q, masses) for q in reached]))
    shoulder_rated = servos[params.joints["j2"].servo].rated_torque
    elbow_rated = servos[params.joints["j3"].servo].rated_torque
    return Check(
        reached=len(reached),
        total=len(poses),
        sin_j3=float(np.abs(np.sin(Q[:, 2])).min()) if reached else 0.0,
        shoulder=float(taus[:, 1].max() / shoulder_rated) if reached else math.inf,
        elbow=float(taus[:, 2].max() / elbow_rated) if reached else math.inf,
        reach_low=reach_band(robot, 0.04)[1],
        reach_high=reach_band(robot, 0.16)[1],
    )


def main() -> None:
    base = load_arm()
    radii = [radius for radius, _, _ in targets()]
    print(
        f"## 목표 자세 {len(targets())}개: 반지름 {min(radii) * 100:.1f}~{max(radii) * 100:.1f} cm,"
        f" 높이 {BOX * 100:.0f}~{(LAYERS * BOX + APPROACH) * 100:.0f} cm"
    )
    for offsets in [(0.05, 0.05, 0.05), (0.04, 0.04, 0.03)]:
        label = "/".join(f"{o * 1000:.0f}" for o in offsets)
        print(
            f"\n## 손목 오프셋 {label} mm (J4→J5, J5→J6, J6→플랜지), 목표는 정격의 {RATED_MARGIN} 이하"
        )
        print(
            "| 위팔 | 아래팔 | 닿음 | J3의 sin 절댓값 최소 | 어깨 | 팔꿈치 |"
            " 닿는 끝 (높이 4 / 16 cm) |"
        )
        print("|---|---|---|---|---|---|---|")
        for upper, fore in itertools.product(
            np.arange(0.16, 0.225, 0.01), np.arange(0.14, 0.205, 0.01)
        ):
            params = with_lengths(base, float(upper), float(fore), offsets)
            c = check(arm_robot(params, load_servos()), params)
            print(
                f"| {upper * 100:.0f} | {fore * 100:.0f} | {c.reached}/{c.total} | {c.sin_j3:.2f}"
                f" | {c.shoulder:.2f} | {c.elbow:.2f}"
                f" | {c.reach_low * 100:.1f} / {c.reach_high * 100:.1f} |"
            )


if __name__ == "__main__":
    main()
