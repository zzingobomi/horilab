"""비교 모델 UR3e (0027): UR이 공개한 기구학과 충돌 형상으로 만들고, 호리봇 크기로 통째로 줄일 수 있다.

- 기구학: UR 홈페이지의 DH 값. 공개 모형(Universal_Robots_ROS2_Description)의 config/ur3e/default_kinematics.yaml과
  같다 (docs/references.md의 "기준 모델: UR3e").
- 충돌 형상: 공개 모형의 meshes/ur3e/collision/*.stl (reference/ur3e/, BSD-3-Clause). 링크 좌표계에서의 자리는
  config/ur3e/visual_parameters.yaml의 mesh_offset이다.
- 링크 좌표계는 이 레포 규칙(영점에서 모두 베이스와 같은 방향, 팔은 위로 곧게, robot.py)으로 둔다. 공개 모형의
  링크 좌표계는 관절마다 돌아 있어서, 공개 모형의 관절 각 0과 몸체 자리가 같아지는 관절 각(correspondence)을 찾아
  형상을 옮긴다.
- MuJoCo는 메시를 볼록 껍질로 보고 부딪힘을 잰다. 링크 형상에는 오목한 곳이 있어서, 가장 긴 축을 따라 얇은 판으로
  잘라 판마다 볼록 껍질로 둔다 (SLAB).
- 부딪힘을 보지 않는 링크 쌍은 UR의 MoveIt 설정(Universal_Robots_ROS2_Driver의
  ur_moveit_config/srdf/ur_macro.srdf.xacro)과 같다.
- 관절 범위: 사양은 플랜지가 끝없이 돌고 나머지는 ±360°다. 자세 하나씩 보는 시험에서는 한 바퀴를 넘는 범위가
  모든 각이 되는 것과 같아서 ±180°로 둔다. 팔꿈치는 어깨 관절에 막혀 실제로도 약 ±180°다 (공개 모형
  joint_limits.yaml의 주석). 막히는 것은 부딪힘 검사가 본다.
"""

from __future__ import annotations

import itertools
import math
from functools import cache
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike

from horilab_design.robot import Joint, Mat, Robot, link_poses, rotation
from horilab_design.studies import packaging as pk
from horilab_design.studies.packaging import Builder, Prim, Tool, X, Y, Z

MESH_DIR = Path(__file__).resolve().parent / "reference" / "ur3e" / "collision"

# DH 값 [m]: 설치면 → 어깨 축(d1), 위팔(a2), 아래팔(a3), J4 축 방향 옆 오프셋(d4), J5 축 방향(d5), J6 → 플랜지(d6)
D1, A2, A3, D4, D5, D6 = 0.15185, 0.24355, 0.2132, 0.13105, 0.08535, 0.0921
# 공개 모형의 관절 원점: 부모 링크 좌표계에서의 위치 [m]와 rpy [rad]. 관절은 자기 좌표계의 z 둘레로 돈다.
URDF_JOINTS: tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...] = (
    ((0.0, 0.0, D1), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (math.pi / 2, 0.0, 0.0)),
    ((-A2, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((-A3, 0.0, D4), (0.0, 0.0, 0.0)),
    ((0.0, -D5, 0.0), (math.pi / 2, 0.0, 0.0)),
    ((0.0, D6, 0.0), (math.pi / 2, math.pi, math.pi)),
)
# 링크마다 충돌 형상 파일과 링크 좌표계에서의 자리 (위치 [m], rpy [rad]). 0은 베이스(base_link_inertia)다.
MESHES: tuple[tuple[str, tuple[float, float, float], tuple[float, float, float]], ...] = (
    ("base.stl", (0.0, 0.0, 0.0), (0.0, 0.0, math.pi)),
    ("shoulder.stl", (0.0, 0.0, 0.0), (0.0, 0.0, math.pi)),
    ("upperarm.stl", (0.0, 0.0, 0.120), (math.pi / 2, 0.0, -math.pi / 2)),
    ("forearm.stl", (0.0, 0.0, 0.027), (math.pi / 2, 0.0, -math.pi / 2)),
    ("wrist1.stl", (0.0, 0.0, -0.104), (math.pi / 2, 0.0, 0.0)),
    ("wrist2.stl", (0.0, 0.0, -D5), (0.0, 0.0, 0.0)),
    ("wrist3.stl", (0.0, 0.0, -D6), (math.pi / 2, 0.0, 0.0)),
)
LINK_NAMES = ("베이스", "어깨", "위팔", "아래팔", "손목 1", "손목 2", "손목 3")
# UR의 MoveIt 설정이 부딪힘을 보지 않는 링크 쌍: 이웃한 링크, 그리고 손목 1과 손목 3(서로 닿을 수 없음).
EXCLUDED = ((0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (4, 6), (5, 6))
# 도구는 손목 3에 붙인 따로 된 몸체(7)라, 손목 3하고만 빼고 손목 1·2, 다른 링크, 책상과는 부딪힘을 본다.
TOOL_EXCLUDED = ((6, 7),)
SLAB = 0.008  # 형상을 자르는 판 두께 [m] (원본 크기 기준)
# 공개 모형 joint_limits.yaml: 관절 속도 [rad/s], 낼 수 있는 토크 [N·m]
VELOCITY = (math.pi, math.pi, math.pi, 2 * math.pi, 2 * math.pi, 2 * math.pi)
EFFORT = (54.0, 54.0, 28.0, 9.0, 9.0, 9.0)


def ur_robot(scale: float = 1.0) -> Robot:
    """UR3e의 로봇 기술. 영점에서 팔이 위로 곧게 서고, 축 배치와 오프셋 방향은 호리봇(values.arm_robot)과 같다."""
    s = scale
    axes = (Z, Y, Y, Y, Z, Y)
    origins = (
        (0.0, 0.0, 0.0),
        (0.0, 0.0, D1),
        (0.0, 0.0, A2),
        (0.0, 0.0, A3),
        (0.0, D4, 0.0),
        (0.0, 0.0, D5),
    )
    joints = tuple(
        Joint(
            f"j{k + 1}",
            (o[0] * s, o[1] * s, o[2] * s),
            axes[k],
            -math.pi,
            math.pi,
            EFFORT[k],
            VELOCITY[k],
        )
        for k, o in enumerate(origins)
    )
    flange = np.eye(4)
    flange[:3, :3] = rotation(X, -np.pi / 2) @ rotation(Z, -np.pi / 2)
    flange[:3, 3] = (0.0, D6 * s, 0.0)
    return Robot("ur3e" if s == 1.0 else f"ur3e×{s:.3f}", joints, flange)


def _tf(xyz: ArrayLike, rpy: tuple[float, float, float]) -> Mat:
    """URDF의 origin(xyz, rpy)을 4×4 행렬로. rpy는 고정 축 x, y, z 순서다."""
    roll, pitch, yaw = rpy
    T = np.eye(4)
    T[:3, :3] = rotation(Z, yaw) @ rotation(Y, pitch) @ rotation(X, roll)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def urdf_link_poses(q: ArrayLike) -> list[Mat]:
    """공개 모형의 관절 각 q [rad]에서 링크 좌표계 자세 (어깨 ~ 손목 3, 베이스는 base_link_inertia)."""
    T = np.eye(4)
    out: list[Mat] = []
    for (xyz, rpy), angle in zip(URDF_JOINTS, np.asarray(q, dtype=float), strict=True):
        spin = np.eye(4)
        spin[:3, :3] = rotation(Z, float(angle))
        T = T @ _tf(xyz, rpy) @ spin
        out.append(T)
    return out


def _same_line(p: np.ndarray, d: np.ndarray, p2: np.ndarray, d2: np.ndarray) -> int:
    """두 축 선이 같으면 방향 부호(1 또는 -1), 다르면 0."""
    if np.linalg.norm(np.cross(d, d2)) > 1e-9 or np.linalg.norm(np.cross(p2 - p, d)) > 1e-9:
        return 0
    return 1 if float(d @ d2) > 0 else -1


@cache
def correspondence() -> tuple[tuple[float, ...], tuple[int, ...]]:
    """공개 모형의 관절 각 0과 링크마다 몸체 자리가 같아지는 이 모델의 관절 각, 그리고 관절 축 방향의 부호.

    관절 i의 각은 다음 관절 축 선이 공개 모형과 겹치게 고른다 (90°씩 본다). 마지막 관절은 0이다.
    그러면 이 모델의 관절 각 = 찾은 각 + 부호 × 공개 모형의 관절 각이다.
    """
    robot = ur_robot()
    ur = urdf_link_poses(np.zeros(6))
    q = np.zeros(6)
    signs: list[int] = []

    def ours(i: int) -> tuple[np.ndarray, np.ndarray]:
        T = link_poses(robot, q)[i]
        return T[:3, 3], T[:3, :3] @ np.asarray(robot.joints[i].axis)

    for i in range(6):
        p, d = ours(i)
        sign = _same_line(p, d, ur[i][:3, 3], ur[i][:3, 2])
        if sign == 0:
            raise RuntimeError(f"관절 {i + 1}의 축 선이 공개 모형과 겹치지 않는다")
        signs.append(sign)
        if i == 5:
            break
        for angle in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            q[i] = angle
            p2, d2 = ours(i + 1)
            if _same_line(p2, d2, ur[i + 1][:3, 3], ur[i + 1][:3, 2]) == 1:
                break
        else:
            raise RuntimeError(f"관절 {i + 1}의 각을 찾지 못했다")
    return tuple(float(a) for a in q), tuple(signs)


def to_ours(q_urdf: ArrayLike) -> np.ndarray:
    """공개 모형의 관절 각을 이 모델의 관절 각으로."""
    offset, signs = correspondence()
    return np.asarray(offset) + np.asarray(signs) * np.asarray(q_urdf, dtype=float)


@cache
def mesh_placements() -> tuple[Mat, ...]:
    """링크마다 이 모델의 링크 좌표계에서 본 충돌 형상 파일의 좌표계 (원본 크기)."""
    offset, _ = correspondence()
    ours = [np.eye(4), *link_poses(ur_robot(), offset)]
    urs = [np.eye(4), *urdf_link_poses(np.zeros(6))]
    return tuple(
        np.linalg.inv(ours[i]) @ urs[i] @ _tf(xyz, rpy) for i, (_, xyz, rpy) in enumerate(MESHES)
    )


def load_stl(path: Path) -> np.ndarray:
    """이진 STL의 삼각형들 (n × 3 × 3) [m]. 공개 모형의 파일은 m 단위다."""
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path.name}: 이진 STL이 아니거나 잘렸다")
    rows = np.frombuffer(data, dtype=np.uint8, count=count * 50, offset=84).reshape(count, 50)
    return rows[:, 12:48].copy().view("<f4").reshape(count, 3, 3).astype(float)


def slab_points(tri: np.ndarray, thickness: float) -> list[np.ndarray]:
    """삼각형들을 가장 긴 축을 따라 판으로 자르고, 판마다 볼록 껍질을 이룰 점을 준다.

    점은 판 안의 꼭짓점과, 모서리가 판의 두 면을 지나는 점이다. 닫힌 형상이면 이 점들의 볼록 껍질이
    판 안 형상의 볼록 껍질과 같다.
    """
    pts = tri.reshape(-1, 3)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    k = int(np.argmax(hi - lo))
    n = max(1, math.ceil((hi[k] - lo[k]) / thickness))
    cuts = np.linspace(lo[k], hi[k], n + 1)
    edges = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
    e0, e1 = edges[:, 0], edges[:, 1]
    out: list[np.ndarray] = []
    for s0, s1 in itertools.pairwise(cuts):
        parts = [pts[(pts[:, k] >= s0) & (pts[:, k] <= s1)]]
        for s in (s0, s1):
            d0, d1 = e0[:, k] - s, e1[:, k] - s
            crossing = d0 * d1 < 0
            t = d0[crossing] / (d0[crossing] - d1[crossing])
            parts.append(e0[crossing] + t[:, None] * (e1[crossing] - e0[crossing]))
        p = np.unique(np.round(np.concatenate(parts), 9), axis=0)
        if len(p) >= 4 and np.linalg.matrix_rank(p - p.mean(axis=0), tol=1e-6) == 3:
            out.append(p)
    return out


@cache
def _link_triangles() -> tuple[np.ndarray, ...]:
    """링크마다 이 모델의 링크 좌표계로 옮긴 충돌 형상 삼각형들 (원본 크기)."""
    out = []
    for (fname, _, _), C in zip(MESHES, mesh_placements(), strict=True):
        tri = load_stl(MESH_DIR / fname)
        v = tri.reshape(-1, 3) @ C[:3, :3].T + C[:3, 3]
        out.append(v.reshape(-1, 3, 3))
    return tuple(out)


def ur_prims(scale: float = 1.0) -> list[Prim]:
    """UR3e 링크 형상을 판마다 볼록 껍질로 (링크 0~6). 무게는 넣지 않는다 (버팀 토크는 견주지 않는다)."""
    prims: list[Prim] = []
    for i, tri in enumerate(_link_triangles()):
        for k, verts in enumerate(slab_points(tri * scale, SLAB * scale)):
            prims.append(
                Prim(
                    f"ur3e/{i}#{k}",
                    i,
                    "hull",
                    np.zeros(3),
                    np.eye(3),
                    (0.0, 0.0, 0.0),
                    "metal",
                    0.0,
                    f"UR3e {LINK_NAMES[i]}",
                    verts,
                )
            )
    return prims


def ur_scene(scale: float = 1.0, tool: Tool | None = None) -> pk.Scene:
    """UR3e(scale배)와 도구(교체 어댑터 + 도구, 크기는 그대로)를 책상 위에 둔 장면."""
    robot = ur_robot(scale)
    b = Builder(ur_prims(scale))
    if tool is None:
        return pk.build_scene(robot, b.prims, exclude=EXCLUDED)
    pk.add_tool(b, 7, D6 * scale, tool)
    return pk.build_scene(robot, b.prims, exclude=EXCLUDED + TOOL_EXCLUDED)
