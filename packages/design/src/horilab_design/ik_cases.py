"""IK 대조 시험의 자세와 해를 만든다 (docs/decisions/0033-planning-and-simulation.md의 만드는 순서 2).

컨트롤러의 Rust IK(crates/controller/src/kinematics.rs)를 설계의 EAIK와 견준다. 관절 범위 안의 무작위 관절
각으로 플랜지 자세를 만들고, 그 자세의 EAIK 해를 모두 JSON으로 낸다. Rust 시험(crates/controller/tests/
ik_eaik.rs)이 이 명령을 띄워 읽는다:

uv run python -m horilab_design.ik_cases --count 300 --seed 7
"""

from __future__ import annotations

import argparse
import json
from typing import Any

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import flange_pose, link_poses
from horilab_design.values import load_robot

# 특이 자세 가까이는 해가 흔들려서 빼 둔다: 손목(J5가 0이나 π), 팔꿈치(J3가 0이나 π), 어깨(J6 원점이 J1 축에 가까움)
SINGULAR_SIN = 0.05
SHOULDER_MARGIN = 0.03  # [m]


def cases(count: int, seed: int) -> dict[str, Any]:
    robot = load_robot()
    ik = AnalyticIK(robot)
    rng = np.random.default_rng(seed)
    lower = np.array([joint.lower for joint in robot.joints])
    upper = np.array([joint.upper for joint in robot.joints])
    offset = sum(joint.origin[1] for joint in robot.joints[1:5])  # 나란한 축 방향의 거리
    found: list[dict[str, Any]] = []
    while len(found) < count:
        q = rng.uniform(lower, upper)
        if abs(np.sin(q[4])) < SINGULAR_SIN or abs(np.sin(q[2])) < SINGULAR_SIN:
            continue
        wrist = link_poses(robot, q)[5][:3, 3]
        if np.hypot(wrist[0], wrist[1]) < abs(offset) + SHOULDER_MARGIN:
            continue
        T = flange_pose(robot, q)
        found.append(
            {
                "q": q.tolist(),
                "rotation": T[:3, :3].tolist(),
                "position": T[:3, 3].tolist(),
                "solutions": ik.solve(T).tolist(),
            }
        )
    return {
        "origins": [list(joint.origin) for joint in robot.joints],
        "axes": [list(joint.axis) for joint in robot.joints],
        "flange_rotation": robot.T_last_flange[:3, :3].tolist(),
        "flange_position": robot.T_last_flange[:3, 3].tolist(),
        "cases": found,
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="IK 대조 시험의 자세와 EAIK 해 (0033)")
    parser.add_argument("--count", type=int, default=300)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)
    print(json.dumps(cases(args.count, args.seed)))


if __name__ == "__main__":
    main()
