"""로봇 기술로 MuJoCo 모델(MJCF)을 만든다. 지금은 골격 설계용 막대 모델이다.

MuJoCo의 MjSpec으로 코드에서 모델을 만든다 (docs/decisions/0006-data-and-pipeline.md).
바디 이름은 base, link1 … linkN, 플랜지는 사이트 flange다 (0010).
"""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import ArrayLike

from horilab_design.robot import Robot

STICK_RADIUS = 0.012  # [m] 막대 모델의 링크 굵기. 모양을 보려는 값이라 정밀할 필요는 없다


def build_spec(robot: Robot, stick_radius: float = STICK_RADIUS) -> mujoco.MjSpec:
    """막대 모델 MjSpec을 만든다. 링크마다 다음 관절(또는 플랜지)까지 막대를 하나 둔다."""
    spec = mujoco.MjSpec()
    spec.modelname = robot.name
    # MJCF 각도의 기본 단위는 도다. 로봇 기술은 라디안이라 바꿔 둔다.
    spec.compiler.degree = False

    body = spec.worldbody.add_body(name="base")
    _add_stick(body, robot.joints[0].origin, stick_radius)
    for i, joint in enumerate(robot.joints):
        body = body.add_body(name=f"link{i + 1}", pos=list(joint.origin))
        body.add_joint(
            name=joint.name,
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=list(joint.axis),
            range=[joint.lower, joint.upper],
            limited=mujoco.mjtLimited.mjLIMITED_TRUE,
        )
        is_last = i + 1 == len(robot.joints)
        end = robot.T_last_flange[:3, 3] if is_last else robot.joints[i + 1].origin
        _add_stick(body, end, stick_radius)

    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, robot.T_last_flange[:3, :3].flatten())
    body.add_site(name="flange", pos=list(robot.T_last_flange[:3, 3]), quat=list(quat))
    return spec


def _add_stick(body: mujoco.MjsBody, end: ArrayLike, radius: float) -> None:
    """바디 원점에서 end까지 캡슐을 단다. 길이가 0이면(축이 한 점에서 만나면) 구를 단다."""
    end = np.asarray(end, dtype=float)
    if np.linalg.norm(end) < 1e-9:
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[radius, 0.0, 0.0])
    else:
        body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0, 0.0, 0.0, *end],
            size=[radius, 0.0, 0.0],
        )
