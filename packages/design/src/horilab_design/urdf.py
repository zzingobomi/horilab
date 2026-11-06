"""로봇 기술로 URDF를 만든다. 표준 라이브러리 XML로 직접 쓴다 (0006, 0008).

링크 좌표계가 영점에서 베이스와 같은 방향이라, 관절의 origin rpy는 늘 0이다.
회전이 있는 건 마지막 링크에서 플랜지로 가는 고정 관절뿐이다.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import ArrayLike

from horilab_design.robot import Mat, Robot


def build_urdf(robot: Robot) -> str:
    """URDF 문자열을 만든다. 링크 이름은 base, link1 … linkN, flange다 (0010)."""
    root = ET.Element("robot", name=robot.name)
    ET.SubElement(root, "link", name="base")
    parent = "base"
    for i, joint in enumerate(robot.joints, start=1):
        child = f"link{i}"
        ET.SubElement(root, "link", name=child)
        element = ET.SubElement(root, "joint", name=joint.name, type="revolute")
        ET.SubElement(element, "parent", link=parent)
        ET.SubElement(element, "child", link=child)
        ET.SubElement(element, "origin", xyz=_vector(joint.origin), rpy="0 0 0")
        ET.SubElement(element, "axis", xyz=_vector(joint.axis))
        ET.SubElement(
            element,
            "limit",
            lower=repr(joint.lower),
            upper=repr(joint.upper),
            effort=repr(joint.effort),
            velocity=repr(joint.velocity),
        )
        parent = child

    ET.SubElement(root, "link", name="flange")
    element = ET.SubElement(root, "joint", name="flange_fixed", type="fixed")
    ET.SubElement(element, "parent", link=parent)
    ET.SubElement(element, "child", link="flange")
    ET.SubElement(
        element,
        "origin",
        xyz=_vector(robot.T_last_flange[:3, 3]),
        rpy=_vector(rpy_from_matrix(robot.T_last_flange[:3, :3])),
    )
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


def matrix_from_rpy(rpy: ArrayLike) -> Mat:
    """URDF의 rpy [rad]를 회전 행렬로 바꾼다. R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def rpy_from_matrix(R: ArrayLike) -> Mat:
    """회전 행렬을 URDF의 rpy [rad]로 바꾼다. pitch가 ±90°면 roll을 0으로 둔다."""
    R = np.asarray(R, dtype=float)
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
    if np.isclose(abs(R[2, 0]), 1.0, rtol=0.0, atol=1e-9):
        # 짐벌 잠금: roll과 yaw가 한 축으로 겹치므로 roll = 0으로 두고 yaw만 구한다.
        return np.array([0.0, pitch, float(np.arctan2(-R[0, 1], R[1, 1]))])
    roll = float(np.arctan2(R[2, 1], R[2, 2]))
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    return np.array([roll, pitch, yaw])


def _vector(values: ArrayLike) -> str:
    # repr는 다시 읽었을 때 같은 float가 되는 가장 짧은 표기다.
    return " ".join(repr(float(v)) for v in np.asarray(values, dtype=float))
