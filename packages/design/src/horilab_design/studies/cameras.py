"""카메라 자리 검토: 손목의 D405(eye-in-hand)와 책상 카메라 기둥의 USB 카메라(eye-to-hand)에서 무엇이 보이는지 본다.

    uv run python -m horilab_design.studies.cameras

- 기둥 카메라 자리 후보마다 본다:
  - 작업 자리(집는 곳 박스 윗면, 팔레트 네 층의 윗면, 앞쪽 잡기 자리)가 시야각과 거리 안에 드는지
  - 팔을 비켜 둔 자세에서 팔이 작업 자리를 가리지 않는지
  - 작업 자세(실제 방위)와 박스를 옮기는 길에서 팔이 기둥에 닿지 않는지
  - 플랜지에 단 보드로 캘리브레이션 자세가 몇 개 나오는지 (보드 두 가지)
- 손목 카메라는 책상 위 캘리브레이션 보드(7 × 5, 25 mm 칸)를 여러 방향에서 볼 수 있는지 본다.
- 책상과 작업 자리는 0014, 팔 형상과 손목 카메라 자리는 UR식 외형(covers.py)이다.
  책상 가장자리에서 베이스 중심까지는 아직 정하지 않아 어림값(DESK_X)이다.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import NDArray

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Mat, Robot, Vec3, link_poses, rotation
from horilab_design.studies import covers, skeleton
from horilab_design.studies import packaging as pk
from horilab_design.studies.packaging import DEG, Builder, Prim, X, Z

Mask = NDArray[np.bool_]


def arm_shapes(layout: pk.Layout, tool: pk.Tool) -> tuple[Robot, list[Prim], pk.Camera]:
    """UR식 외형의 팔 형상과 D405 자리."""
    robot, prims, _, cam = covers.build_ur_arm(layout, tool)
    return robot, prims, cam


D405_FOV = (84 * DEG, 58 * DEG)  # 깊이 시야 (가로, 세로). 사양서 값이다 (홈페이지는 가로 87°)
# [m] 848×480에서 깊이를 재는 가장 가까운 거리(1280×720은 0.10), 사양서가 정확도(±2%)를 주는 거리의 끝
D405_RANGE = (0.07, 0.50)
# 기둥의 USB 카메라 (innomaker U20CAM-720P, docs/hardware.md): 왜곡을 보정한 시야(이전 프로젝트에서 잼).
# 2D 카메라라 깊이 한계는 없다. 몸체는 32 × 32 mm 보드에 렌즈와 받침을 더한 어림이다.
POST_FOV = (94 * DEG, 57 * DEG)
POST_RANGE = (0.0, math.inf)
POST_BODY = (0.032, 0.032, 0.030)
POST_PIXELS = 1280  # 가로 화소 수
DESK_X = (-0.06, 0.28)  # 책상 앞뒤 끝 (베이스 중심 기준, 어림) [m]
DESK_Y = (-0.275, 0.275)
BOX = skeleton.BOX
WORK = (0.14, 0.0, 0.04)  # 기둥 카메라가 겨누는 점: 집는 곳과 팔레트 사이, 박스 1층 높이
CAL_BOARD = (0.20, 0.0)  # 손목 카메라 캘리브레이션 보드 가운데 (0014: 앞쪽 가운데)
CAL_GRID = (7, 5, 0.025)  # 칸 수, 칸 크기
VISIBLE = 0.999  # 광선이 목표점 앞에서 막히지 않았다고 보는 비율


@dataclass(frozen=True)
class PostCam:
    """기둥에 단 USB 카메라. 넓은 쪽(가로)은 수평으로 둔다. 위에서 곧장 내려다보면 가로가 책상 긴 변(y)이다."""

    name: str
    eye: Vec3  # 렌즈 가운데
    look: Vec3  # 광축이 지나는 점
    foot: tuple[float, float]  # 기둥이 서는 자리
    boom: bool = False  # 기둥 위에서 가로로 뻗은 팔 끝에 카메라를 단다

    def frame(self) -> Mat:
        """열: 가로축, 세로축, 광축."""
        a = np.subtract(self.look, self.eye)
        a = a / np.linalg.norm(a)
        side = np.cross(a, Z)
        side = (
            np.array([0.0, 1.0, 0.0])
            if np.linalg.norm(side) < 1e-6
            else side / np.linalg.norm(side)
        )
        return np.column_stack([side, np.cross(a, side), a])


def post_prims(cam: PostCam) -> list[Prim]:
    """기둥(30 × 30 mm), (있으면) 가로 팔(20 × 20 mm), 카메라."""
    b = Builder()
    eye, R = np.array(cam.eye), cam.frame()
    hw, hh, half_d = POST_BODY[0] / 2, POST_BODY[1] / 2, POST_BODY[2] / 2
    b.box(
        0,
        "post/camera",
        (-hw, -hh, -half_d),
        (hw, hh, half_d),
        material="camera",
        frame=(eye - R[:, 2] * half_d, R),
        group="기둥 카메라",
    )
    fx, fy = cam.foot
    top = eye[2] + (0.06 if cam.boom else 0.0)
    b.box(
        0, "post/column", (fx - 0.015, fy - 0.015, 0.0), (fx + 0.015, fy + 0.015, top), group="기둥"
    )
    if cam.boom:
        d = eye[:2] - np.array(cam.foot)
        heading = rotation(Z, math.atan2(d[1], d[0]))
        b.box(
            0,
            "post/boom",
            (0.0, -0.010, -0.020),
            (float(np.linalg.norm(d)), 0.010, 0.0),
            frame=(np.array([fx, fy, top]), heading),
            group="기둥",
        )
        b.box(
            0,
            "post/drop",
            (eye[0] - 0.010, eye[1] - 0.010, eye[2] + POST_BODY[2]),
            (eye[0] + 0.010, eye[1] + 0.010, top - 0.020),
            group="기둥",
        )
    return b.prims


# 0018의 기둥 자리 후보 중 가로 팔 안: 작업 자리 가운데보다 8 cm 맞은편, 렌즈 높이 45 cm에서 작업 자리 가운데를
# 겨눈다. 기둥은 맞은편 모서리. 손목 카메라 검토와 시험에서 기둥이 있을 때로 쓴다.
RECOMMENDED = PostCam(
    "8 cm 비켜 위 (기둥: 맞은편 왼쪽) 45",
    (0.22, 0.0, 0.45),
    (WORK[0], 0.0, 0.0),
    (0.26, 0.255),
    True,
)


def candidates() -> list[PostCam]:
    """위에서 보는 자리(가로 팔이 있는 기둥)와 비스듬히 보는 자리(기둥만).

    위에서 보는 자리는 작업 자리 가운데(x 14 cm) 바로 위와, 맞은편으로 비켜 작업 자리 가운데를 겨누는 자리다.
    """
    out = []
    corners = (("로봇 왼쪽 뒤", (-0.04, 0.24)), ("맞은편 왼쪽", (0.26, 0.255)))
    for h, (where, foot) in itertools.product((0.45, 0.50), corners):
        out.append(
            PostCam(
                f"바로 위 (기둥: {where}) {h * 100:.0f}",
                (WORK[0], 0.0, h),
                (WORK[0], 0.0, 0.0),
                foot,
                boom=True,
            )
        )
    for x, h, side in ((0.22, 0.42, 1), (0.22, 0.45, 1), (0.22, 0.45, -1), (0.24, 0.45, 1)):
        name = "왼쪽" if side > 0 else "오른쪽"
        out.append(
            PostCam(
                f"{(x - WORK[0]) * 100:.0f} cm 비켜 위 (기둥: 맞은편 {name}) {h * 100:.0f}",
                (x, 0.0, h),
                (WORK[0], 0.0, 0.0),
                (0.26, side * 0.255),
                boom=True,
            )
        )
    for h in (0.30, 0.40):
        for side, name in ((1, "왼쪽"), (-1, "오른쪽")):
            out.append(
                PostCam(
                    f"맞은편 {name} 모서리 {h * 100:.0f}",
                    (0.245, side * 0.235, h),
                    WORK,
                    (0.265, side * 0.255),
                )
            )
            out.append(
                PostCam(
                    f"{name} 옆 {h * 100:.0f}", (0.13, side * 0.235, h), WORK, (0.13, side * 0.26)
                )
            )
        out.append(PostCam(f"맞은편 가운데 {h * 100:.0f}", (0.25, 0.0, h), WORK, (0.27, 0.0)))
    return out


# ----------------------------------------------------------------------------------------------------
# 보이는지


def in_view(R: Mat, eye: Mat, pts: Mat) -> Mask:
    """기둥 카메라: 점마다 시야각 안이고 광축 방향 거리가 POST_RANGE 안인지."""
    c = (pts - eye) @ R
    z = c[:, 2]
    tx, ty = math.tan(POST_FOV[0] / 2), math.tan(POST_FOV[1] / 2)
    return (
        (z > 0)
        & (np.abs(c[:, 0]) <= z * tx)
        & (np.abs(c[:, 1]) <= z * ty)
        & (z >= POST_RANGE[0])
        & (z <= POST_RANGE[1])
    )


def pixel_mm(distance: float) -> float:
    """기둥 카메라에서 distance [m] 떨어진 면의 한 화소 크기 [mm] (왜곡을 보정한 시야로 어림)."""
    return 2 * distance * math.tan(POST_FOV[0] / 2) / POST_PIXELS * 1000


def in_cone(eye: Mat, axis: Mat, pts: Mat) -> Mask:
    """손목 카메라: 어느 쪽으로 눕힐지 정하기 전이라 좁은 쪽 시야(58°)를 원뿔로 본다."""
    rays = pts - eye
    depth = rays @ axis
    cos = depth / np.linalg.norm(rays, axis=1)
    return (cos >= math.cos(D405_FOV[1] / 2)) & (depth >= D405_RANGE[0]) & (depth <= D405_RANGE[1])


def unblocked(scene: pk.Scene, q: Mat, eye: Mat, pts: Mat, exclude: str) -> Mask:
    """eye에서 점마다 그은 광선이 점에 닿기 전에 다른 형상에 막히지 않는지 (exclude 몸체는 뺀다)."""
    m, d = scene.model, scene.data
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, exclude)
    geomid = np.zeros(1, dtype=np.int32)
    out = np.zeros(len(pts), dtype=bool)
    for i, p in enumerate(pts):
        hit = mujoco.mj_ray(m, d, eye, p - eye, None, 1, body, geomid)
        out[i] = hit < 0 or hit >= VISIBLE
    return out


def work_targets() -> dict[str, Mat]:
    """기둥 카메라가 봐야 할 점들. 책상 위 점은 2 mm 띄운다 (책상 면과 겹치지 않게)."""
    g = np.linspace(-0.45, 0.45, 5)
    spots = skeleton.spots()
    px, py, _ = spots[0]
    out = {"집는 곳": np.array([(px + BOX * a, py + BOX * b, BOX) for a in g for b in g])}
    cx = float(np.mean([s[0] for s in spots[1:]]))
    cy = float(np.mean([s[1] for s in spots[1:]]))
    for layer in range(1, skeleton.LAYERS + 1):
        out[f"팔레트 {layer}층"] = np.array(
            [(cx + 2 * BOX * a, cy + 2 * BOX * b, BOX * layer) for a in g for b in g]
        )
    out["잡기 자리"] = np.array(
        [(x, y, 0.002) for x in np.linspace(0.14, 0.26, 5) for y in np.linspace(-0.08, 0.08, 5)]
    )
    return out


def placed_stacking_poses(tool: pk.Tool) -> list[pk.Pose]:
    """박스 쌓기 자세를 실제 방위에 둔다 (packaging.stacking_poses는 반지름만 본다)."""
    out = []
    for x, y, layers in skeleton.spots():
        yaw = skeleton.box_yaw(math.atan2(y, x))
        for layer in range(1, layers + 1):
            for lift in (0.0, skeleton.APPROACH):
                h = layer * BOX + lift
                out.append(
                    pk.Pose(
                        f"박스 ({x * 100:.0f}, {y * 100:.0f}) h{h * 100:.0f}",
                        pk.tool_pose((x, y, h), 0.0, 0.0, yaw, tool.length),
                    )
                )
    return out


def transit_poses(tool: pk.Tool) -> list[pk.Pose]:
    """박스를 들고 집는 곳에서 팔레트로 가는 길: TCP 높이 24 cm, 반지름 20 cm, 방위 +60° → -45°."""
    out = []
    for az in range(60, -46, -5):
        p = (0.20 * math.cos(az * DEG), 0.20 * math.sin(az * DEG), 0.24)
        out.append(pk.Pose(f"옮기는 길 방위 {az}°", pk.tool_pose(p, 0.0, 0.0, 0.0, tool.length)))
    return out


TASK_SETS = (
    ("suction", placed_stacking_poses),
    ("suction", transit_poses),
    ("pointer", pk.pointer_poses),
    ("gripper", pk.grasp_poses),
)


# ----------------------------------------------------------------------------------------------------
# 팔을 비켜 둘 자세


def inside_desk(robot: Robot, q: Mat, tool_length: float) -> bool:
    """링크 원점과 TCP가 모두 책상 위에 있는지 (책상 밖으로 팔을 내밀지 않는다)."""
    poses = link_poses(robot, q)
    flange = poses[-1] @ robot.T_last_flange
    pts = [T[:3, 3] for T in poses] + [flange[:3, 3] + flange[:3, 2] * tool_length]
    return all(DESK_X[0] <= p[0] <= DESK_X[1] and DESK_Y[0] <= p[1] <= DESK_Y[1] for p in pts)


def park_candidates(scene: pk.Scene, tool: pk.Tool) -> list[Mat]:
    """책상 위에서 부딪힘 없는 자세들 (J5, J6는 가운데 각)."""
    robot = scene.robot
    out = []
    for q1, q2, q3, q4 in itertools.product(
        range(-165, 180, 15), range(-90, 91, 30), range(0, 119, 30), (120, 165, 210)
    ):
        q = np.array([q1, q2, q3, q4, 90, 10], float) * DEG
        if not inside_desk(robot, q, tool.length):
            continue
        if pk.collisions(pk.hits(scene, q)):
            continue
        out.append(q)
    return out


# ----------------------------------------------------------------------------------------------------
# 기둥 카메라 캘리브레이션: 플랜지에 단 보드를 여러 자세로 보여 준다


def _flange_face(key: str) -> Mat:
    """플랜지 좌표계에서 본 보드 겉면 (원점은 면 가운데, z는 면이 보는 쪽)."""
    T = np.eye(4)
    if key == "board":
        T[:3, 3] = (0.0, 0.0, pk.TOOLS["board"].length)
    else:
        T[:3, :3] = rotation(X, math.pi)
        T[:3, 3] = (0.0, pk.BOARD_SIDE_OFFSET, pk.ADAPTER_T)
    return T


def board_poses(cam: PostCam, key: str) -> list[tuple[str, Mat, Mat]]:
    """(이름, 플랜지 자세, 보드 겉면 자세): 보드 가운데를 시야 안 여러 곳에 두고 면을 카메라 쪽에서 0~40° 기울인다."""
    R, eye = cam.frame(), np.array(cam.eye)
    tx, ty = math.tan(POST_FOV[0] / 2), math.tan(POST_FOV[1] / 2)
    face_inv = np.linalg.inv(_flange_face(key))
    out = []
    for dist in (0.22, 0.30, 0.38):
        for u, v in ((0, 0), (0.35, 0), (-0.35, 0), (0, 0.35), (0, -0.35)):
            p = eye + R @ np.array([u * dist * tx, v * dist * ty, dist])
            to_cam = (eye - p) / np.linalg.norm(eye - p)
            e1 = np.cross(to_cam, Z)
            e1 = R[:, 0] if np.linalg.norm(e1) < 1e-6 else e1 / np.linalg.norm(e1)
            e2 = np.cross(to_cam, e1)
            for tilt, az in [(0, 0), *itertools.product((25, 40), (0, 90, 180, 270))]:
                k = math.cos(az * DEG) * e1 + math.sin(az * DEG) * e2
                n = rotation(k, tilt * DEG) @ to_cam
                x0 = e1 - n * (e1 @ n)
                x0 /= np.linalg.norm(x0)
                for spin in (0, 90, 180, 270):
                    x = rotation(n, spin * DEG) @ x0
                    face = np.eye(4)
                    face[:3, :3] = np.column_stack([x, np.cross(n, x), n])
                    face[:3, 3] = p
                    label = f"d{dist * 100:.0f} 기울기{tilt} 방향{az} 돌림{spin}"
                    out.append((label, face @ face_inv, face))
    return out


def board_points(face: Mat) -> Mat:
    """보드 겉면의 가운데와 네 모서리."""
    bx, by = pk.BOARD_HALF
    c = face[:3, 3]
    return np.array(
        [c]
        + [c + face[:3, 0] * sx * bx + face[:3, 1] * sy * by for sx in (1, -1) for sy in (1, -1)]
    )


@dataclass
class CalResult:
    total: int
    ok: list[str]

    def kinds(self) -> set[tuple[str, str]]:
        """된 자세의 (기울기, 방향) 종류. 0°는 한 종류, 25°·40°는 네 방향씩 해서 모두 9종류다."""
        return {(label.split()[1], label.split()[2]) for label in self.ok}


def eye_to_hand_calibration(cam: PostCam, layout: pk.Layout, key: str) -> CalResult:
    robot, prims, _ = arm_shapes(layout, pk.TOOLS[key])
    scene = pk.build_scene(robot, prims, post_prims(cam))
    ik = AnalyticIK(robot)
    R, eye = cam.frame(), np.array(cam.eye)
    poses = board_poses(cam, key)
    ok = []
    for label, T, face in poses:
        pts = board_points(face)
        if not in_view(R, eye, pts).all():
            continue
        for q in ik.solve_in_limits(T):
            if pk.collisions(pk.hits(scene, q)):
                continue
            if unblocked(scene, q, eye, pts, "fixture").all():
                ok.append(label)
                break
    return CalResult(len(poses), ok)


# ----------------------------------------------------------------------------------------------------
# 손목 카메라: 책상 위 보드를 여러 방향에서 본다


def camera_pose(robot: Robot, cam: pk.Camera, q: Mat) -> tuple[Mat, Mat]:
    T5 = link_poses(robot, q)[4]
    return T5[:3, 3] + T5[:3, :3] @ cam.lens, T5[:3, :3] @ cam.axis


def aim(robot: Robot, cam: pk.Camera, eye: Mat, target: Mat, seeds: list[Mat]) -> list[Mat]:
    """렌즈를 eye에 두고 광축이 target을 지나게 하는 J1~J5 (J6는 seed 값 그대로). 관절 한계 안의 해만."""
    want = (target - eye) / np.linalg.norm(target - eye)

    def residual(q: Mat) -> Mat:
        p, a = camera_pose(robot, cam, q)
        return np.concatenate([p - eye, 0.1 * np.cross(a, want)])

    out: list[Mat] = []
    for seed in seeds:
        q = seed.copy()
        for _ in range(40):
            r = residual(q)
            if np.linalg.norm(r) < 1e-8:
                break
            J = np.zeros((6, 5))
            for j in range(5):
                dq = np.zeros(6)
                dq[j] = 1e-6
                J[:, j] = (residual(q + dq) - r) / 1e-6
            step = np.linalg.lstsq(J, -r, rcond=None)[0]
            size = float(np.linalg.norm(step))
            q[:5] += step * min(1.0, 0.3 / size) if size > 0 else step
        p, a = camera_pose(robot, cam, q)
        if np.linalg.norm(p - eye) > 1e-4 or a @ want < math.cos(0.2 * DEG):
            continue
        q = (q + math.pi) % (2 * math.pi) - math.pi
        if q[3] < robot.lower[3]:
            q[3] += 2 * math.pi
        if np.any(q < robot.lower) or np.any(q > robot.upper):
            continue
        if all(np.abs(q - o).max() > 1e-3 for o in out):
            out.append(q)
    return out


def wrist_views() -> list[tuple[str, Mat]]:
    """손목 카메라 렌즈 자리: 보드 가운데에서 거리 25·35 cm, 보드 면에서 올려본 각 90·65·50°, 방위 8개.

    방위 0°는 렌즈가 보드보다 로봇에서 먼 쪽(바깥), 180°는 로봇 쪽이다.
    """
    b = np.array([*CAL_BOARD, 0.0])
    out = []
    for dist in (0.25, 0.35):
        for el, az in [(90, 0), *itertools.product((65, 50), range(0, 360, 45))]:
            u = np.array(
                [
                    math.cos(el * DEG) * math.cos(az * DEG),
                    math.cos(el * DEG) * math.sin(az * DEG),
                    math.sin(el * DEG),
                ]
            )
            out.append((f"d{dist * 100:.0f} 올려본 각{el} 방위{az}", b + u * dist))
    return out


def board_corners_on_desk() -> Mat:
    """책상 위 보드의 칸 모서리 (안쪽 모서리 6 × 4개)."""
    nx, ny, s = CAL_GRID
    xs = (np.arange(1, nx) - nx / 2) * s
    ys = (np.arange(1, ny) - ny / 2) * s
    return np.array([(CAL_BOARD[0] + x, CAL_BOARD[1] + y, 0.002) for x in xs for y in ys])


@dataclass
class WristResult:
    ok: list[str]
    out_of_range: list[str]  # 관절 범위 안의 해가 없다
    collided: list[str]  # 해는 있지만 모두 부딪힌다
    hidden: list[str]  # 부딪히지 않는 해가 있지만 보드가 시야 밖이거나 가린다


def eye_in_hand_calibration(
    layout: pk.Layout, tool: pk.Tool, fixed: list[Prim] | None = None
) -> WristResult:
    """보드 칸 모서리의 3/4 이상이 시야 안에 들고 가리지 않으면 된 것으로 본다. fixed는 책상의 기둥."""
    robot, prims, cam = arm_shapes(layout, tool)
    scene = pk.build_scene(robot, prims, fixed)
    b = np.array([*CAL_BOARD, 0.002])
    corners = board_corners_on_desk()
    seeds = [
        np.array(s, float) * DEG
        for s in itertools.product(
            (-30, 0, 30), (-20, 20, 50), (40, 80, 115), (140, 190), (60, 110), (10, 100)
        )
    ]
    res = WristResult([], [], [], [])
    for label, eye in wrist_views():
        sols = aim(robot, cam, eye, b, seeds)
        free = [q for q in sols if not pk.collisions(pk.hits(scene, q))]
        good = False
        for q in free:
            _, axis = camera_pose(robot, cam, q)
            seen = in_cone(eye, axis, corners) & unblocked(scene, q, eye, corners, "link5")
            if seen.mean() >= 0.75:
                good = True
                break
        if good:
            res.ok.append(label)
        elif not sols:
            res.out_of_range.append(label)
        elif not free:
            res.collided.append(label)
        else:
            res.hidden.append(label)
    return res


# ----------------------------------------------------------------------------------------------------
# 기둥 자리 후보 평가


@dataclass
class PostResult:
    cam: PostCam
    in_view: dict[str, float]
    depth: tuple[float, float]
    seen_parked: float
    lost: list[int]
    upright: int
    calibration: dict[str, CalResult]


UPRIGHT_J4 = (90, 135, 180, 225)  # [°] 곧게 선 자세에서 손목을 꺾는 각들
UPRIGHT_J1 = range(-175, 176, 5)


def upright_hits(scene: pk.Scene) -> int:
    """팔을 곧게 세우고(J2 = J3 = 0) J1을 돌릴 때 기둥에 닿는 (J1, J4) 조합 수."""
    count = 0
    for j4, j1 in itertools.product(UPRIGHT_J4, UPRIGHT_J1):
        q = np.array([j1, 0, 0, j4, 90, 10], float) * DEG
        found = pk.collisions(pk.hits(scene, q))
        count += any("기둥" in h.a + h.b for h in found)
    return count


def baseline_clear(layout: pk.Layout) -> list[int]:
    out = []
    for key, poses_of in TASK_SETS:
        tool = pk.TOOLS[key]
        robot, prims, _ = arm_shapes(layout, tool)
        scene = pk.build_scene(robot, prims)
        res = pk.check_set(scene, AnalyticIK(robot), poses_of(tool), tool.name, tool.symmetric)
        out.append(res.clear)
    return out


def evaluate_post(cam: PostCam, layout: pk.Layout, baseline: list[int]) -> PostResult:
    fixed = post_prims(cam)
    R, eye = cam.frame(), np.array(cam.eye)
    targets = work_targets()
    in_view_frac = {k: float(in_view(R, eye, pts).mean()) for k, pts in targets.items()}
    all_pts = np.concatenate(list(targets.values()))
    dz = (all_pts - eye) @ R[:, 2]

    lost = []
    for (key, poses_of), base in zip(TASK_SETS, baseline, strict=True):
        tool = pk.TOOLS[key]
        robot, prims, _ = arm_shapes(layout, tool)
        scene = pk.build_scene(robot, prims, fixed)
        res = pk.check_set(scene, AnalyticIK(robot), poses_of(tool), tool.name, tool.symmetric)
        lost.append(base - res.clear)
    robot, prims, _ = arm_shapes(layout, pk.TOOLS["gripper"])  # 가장 긴 도구
    upright = upright_hits(pk.build_scene(robot, prims, fixed))

    tool = pk.TOOLS["suction"]
    robot, prims, _ = arm_shapes(layout, tool)
    scene = pk.build_scene(robot, prims, fixed)
    visible_pts = all_pts[in_view(R, eye, all_pts)]
    best_seen = 0.0
    if len(visible_pts):
        for q in park_candidates(scene, tool):
            best_seen = max(
                best_seen, float(unblocked(scene, q, eye, visible_pts, "fixture").mean())
            )
            if best_seen >= 1.0:
                break
    return PostResult(
        cam,
        in_view_frac,
        (float(dz.min()), float(dz.max())),
        best_seen * len(visible_pts) / len(all_pts),
        lost,
        upright,
        {key: eye_to_hand_calibration(cam, layout, key) for key in ("board", "board_side")},
    )


def main() -> None:
    layout = pk.Layout()
    print(
        f"# 카메라 자리 검토 (기둥 USB 카메라 시야 {POST_FOV[0] / DEG:.0f}° × {POST_FOV[1] / DEG:.0f}°, "
        f"손목 D405 시야 {D405_FOV[0] / DEG:.0f}° × {D405_FOV[1] / DEG:.0f}°, "
        f"깊이 {D405_RANGE[0] * 100:.0f}~{D405_RANGE[1] * 100:.0f} cm)"
    )
    print(
        f"- 책상(어림): x {DESK_X[0] * 100:.0f}~{DESK_X[1] * 100:.0f} cm, y ±{DESK_Y[1] * 100:.1f} cm"
    )
    baseline = baseline_clear(layout)
    print(f"- 기둥 없이 부딪힘 없는 자세 (박스 쌓기/옮기는 길/포인터/잡기): {baseline}")
    print("\n## 기둥 카메라 (eye-to-hand). 높이는 렌즈 [cm]")
    n_upright = len(UPRIGHT_J4) * len(UPRIGHT_J1)
    print(
        "| 자리 | 시야 안: 집는 곳 / 팔레트 1층·4층 / 잡기 | 광축 거리 [cm] (먼 곳 한 화소) | 팔 비킨 뒤 보임 | "
        "기둥에 막히는 자세: 쌓기/옮기기/포인터/잡기 | 곧게 선 팔이 닿는 J1·J4 조합 | "
        "캘리브레이션 자세: 축 보드 · 옆 보드 (종류/9) |"
    )
    print("|---|---|---|---|---|---|---|")
    for cam in candidates():
        r = evaluate_post(cam, layout, baseline)
        iv = r.in_view
        view = f"{iv['집는 곳']:.0%} / {iv['팔레트 1층']:.0%}·{iv['팔레트 4층']:.0%} / {iv['잡기 자리']:.0%}"
        cal = " · ".join(
            f"{len(c.ok)}/{c.total} ({len(c.kinds())})" for c in r.calibration.values()
        )
        print(
            f"| {cam.name} | {view} | {r.depth[0] * 100:.0f}~{r.depth[1] * 100:.0f} "
            f"({pixel_mm(r.depth[1]):.2f} mm) | "
            f"{r.seen_parked:.0%} | {'/'.join(map(str, r.lost))} | {r.upright}/{n_upright} | {cal} |",
            flush=True,
        )
    print("\n## 손목 카메라 (eye-in-hand): 책상 위 보드(7 × 5, 25 mm)를 보는 자세")
    for key, fixed in (
        ("pointer", None),
        ("suction", None),
        ("gripper", None),
        ("suction", post_prims(RECOMMENDED)),
    ):
        r = eye_in_hand_calibration(layout, pk.TOOLS[key], fixed)
        total = len(r.ok) + len(r.out_of_range) + len(r.collided) + len(r.hidden)
        where = f", 기둥 '{RECOMMENDED.name}' 있음" if fixed else ""
        print(
            f"- {pk.TOOLS[key].name}{where}: 됨 {len(r.ok)}/{total}, 관절 범위 밖 {len(r.out_of_range)}, "
            f"부딪힘 {len(r.collided)}, 시야 밖이거나 가림 {len(r.hidden)}"
        )
        if key == "suction" and not fixed:
            print(f"  - 된 방향: {', '.join(r.ok)}")


if __name__ == "__main__":
    main()
