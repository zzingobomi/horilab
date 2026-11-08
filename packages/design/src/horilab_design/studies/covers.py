"""외형 검토: UR식 덮개를 배치 검토 형상에 넣고 크기, 부딪힘, 관절 범위, 무게, 프레임 휨을 본다.

    uv run python -m horilab_design.studies.covers

UR식 외형(docs/references.md의 "외형")을 지금 관절 구조(0015)와 골격(0017)에 맞춘 모양이다.
- 관절마다 관절 축을 감싸는 원통(관절 집)을 둔다. 가운데 고리는 부모 링크에, 양 끝 뚜껑은 자식 링크의
  브래킷 팔 바깥 면에 붙는다. 고리 안쪽 반지름은 브래킷이 쓸고 지나가는 반지름에 움직이는 틈을 더한 값이다.
- 손목(J4·J5·J6)은 자식 링크가 출력 쪽 끝으로 이어진다 (UR과 같다). 고리가 두 뚜껑 사이를 다 덮는다.
- 위팔과 아래팔은 한 면에 있어서(0017) 자식 링크가 고리를 뚫고 나갈 수 없다. J2·J3는 자식 링크가 브래킷 팔을
  늘인 판 두 장으로 원통을 빠져나가고, 고리는 두 판 사이에만 둔다. 판이 지나가는 자리가 검은 띠로 보인다.
- 링크 몸통은 관이다. 서보 먼 쪽을 감싸는 집(0015)은 관 끝에 그대로 둔다.
- J5·J6 고리는 다음 관절까지 힘을 전하는 구조라 벽을 구조 부품 두께로 둔다. 나머지 고리와 뚜껑은 덮개다.
- 손목 원통끼리 가까우면 이웃 뚜껑과 닿는 쪽은 뚜껑을 줄이거나 빼고 목(가는 기둥)으로 잇는다.

프레임 휨은 링크를 곧은 보로 보고 중력 모멘트를 적분한다. 서보, 혼, 나사 체결의 탄성은 넣지 않는다
(관절 하나 시험에서 따로 잰다). 출력한 PLA의 탄성 계수는 확인하지 않은 어림값이다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Mat, Robot
from horilab_design.statics import FLANGE, GRAVITY, PointMass, frame_poses, holding_torques
from horilab_design.studies import packaging as pk
from horilab_design.studies.common import RATED_MARGIN
from horilab_design.studies.packaging import (
    ARM_T,
    BRIDGE_T,
    DEG,
    GAP,
    MM,
    MOVING_GAP,
    TOOLS,
    WALL,
    Builder,
    Camera,
    Layout,
    Module,
    Prim,
    ServoShape,
    Tool,
    Y,
    Z,
)

D405_PLATE_T = 6.0 * MM  # D405 받침판 두께
E_PLA = 2.0e9  # [Pa] 출력한 PLA의 탄성 계수. 확인하지 않은 어림값이라 낮게 잡았다
G_PLA = E_PLA / (2 * (1 + 0.35))  # 푸아송 비 0.35 (어림)

THROUGH = (
    2,
    3,
)  # 자식 링크가 브래킷 팔을 늘인 판으로 원통을 빠져나가는 관절 (위팔과 아래팔이 한 면)
STRUCTURAL = (5, 6)  # 고리가 다음 관절까지 힘을 전하는 관절

# UR식에서 다른 모양으로 바꾸는 배치 검토 형상
REPLACED = {
    "shoulder_post",
    "upper_arm",
    "J3/arm0_root",
    "J3/arm1_root",
    "forearm",
    "wrist1_plate",
    "wrist1_web",
    "wrist2_plate",
    "camera_arm",
    "camera_link",
    "d405",
}


@dataclass(frozen=True)
class Look:
    """UR식 덮개 치수 [m]."""

    shell: float = 1.6 * MM  # 덮개 고리 벽 (0.20mm Standard로 뽑는 덮개)
    structural_shell: float = WALL  # 손목에서 힘을 전하는 고리 벽 (0.20mm Strength 벽 6겹)
    cap: float = 2.0 * MM  # 뚜껑 두께
    upper_tube: float = 0.020  # 위팔 관 바깥 반지름
    fore_tube: float = 0.018  # 아래팔 관 바깥 반지름
    tube_wall: float = WALL
    yoke: float = 6.0 * MM  # J2 원통 위에서 두 판을 잇는 가로판 두께
    root_in: float = 10.0 * MM  # 브래킷 팔을 늘인 판이 관 속으로 들어가는 길이
    column_r: float = 0.044  # J1 몸통 반지름 (돌림판과 같다)


@dataclass(frozen=True)
class Drum:
    """관절 하나의 원통. 축 방향 범위는 서보 좌표계 y [m]다. 반지름이 0인 뚜껑은 없다."""

    joint: int
    r_in: float
    r_out: float
    ring: tuple[float, float]
    cap_out: tuple[float, float]
    cap_back: tuple[float, float]
    cap_out_r: float
    cap_back_r: float

    @property
    def length(self) -> float:
        return self.cap_out[1] - self.cap_back[0]


def _arms(s: ServoShape) -> tuple[tuple[float, float], tuple[float, float]]:
    """브래킷 팔 두 장의 서보 좌표계 y 범위 (출력 쪽, 뒷면 쪽)."""
    return (s.horn_out[1], s.horn_out[1] + ARM_T), (s.horn_back[0] - ARM_T, s.horn_back[0])


def drums(layout: Layout, mods: list[Module], look: Look) -> dict[int, Drum]:
    out: dict[int, Drum] = {}
    for m in mods:
        s = m.shape
        r_in = m.sweep_r + MOVING_GAP
        r_out = r_in + (look.structural_shell if m.joint in STRUCTURAL else look.shell)
        (_, out_face), (back_face, _) = _arms(s)
        # 뚜껑은 브래킷 팔 바깥 면에 붙는다. 서보 먼 쪽을 감싼 집(부모)과도 움직이는 틈을 둔다.
        fy0, fy1 = s.far_half
        cap_out0 = max(out_face, fy1 + GAP + WALL + MOVING_GAP)
        cap_back1 = min(back_face, fy0 - GAP - WALL - MOVING_GAP)
        if m.joint in THROUGH:
            ring = (s.horn_back[0] + MOVING_GAP, s.horn_out[1] - MOVING_GAP)
        else:
            ring = (cap_back1 + MOVING_GAP, cap_out0 - MOVING_GAP)
        caps = (cap_out0, cap_out0 + look.cap), (cap_back1 - look.cap, cap_back1)
        out[m.joint] = Drum(m.joint, r_in, r_out, ring, *caps, r_out, r_out)
    d4, d5, d6 = out[4], out[5], out[6]
    # J5 → J6: J6 원통의 옆면이 J5 출력 쪽 끝을 마주보고, J6 뒷면 뚜껑(링크 6)이 J5 축 위로 지나간다.
    # 틈이 모자라면 J5 출력 쪽 뚜껑을 빼고, J6 뒷면 뚜껑은 J5 브래킷 팔과 틈을 두는 크기로 줄인다.
    if layout.d56 - d6.r_out < layout.mid5 + d5.cap_out[1] + MOVING_GAP:
        d5 = replace(d5, cap_out_r=0.0)
    # 서보가 두꺼우면 J5 고리의 출력 쪽 끝도 J6 고리 아래까지 올라오므로 그 앞에서 끊는다.
    ring5_end = min(d5.ring[1], layout.d56 - d6.r_out - MOVING_GAP - layout.mid5)
    d5 = replace(d5, ring=(d5.ring[0], ring5_end))
    top5 = layout.mid5 + (d5.cap_out[1] if d5.cap_out_r > 0 else d5.cap_out[0])
    d6 = replace(d6, cap_back_r=min(d6.r_out, layout.d56 - top5 - MOVING_GAP))
    # J4 → J5: J5 원통의 옆면이 J4 출력 쪽 끝을 마주보고, J5 뚜껑 두 장(링크 5)이 그쪽으로 다가온다.
    # 틈이 모자라면 J4 출력 쪽 뚜껑을 J5 뚜껑 사이로 들어가는 목 크기로 줄이고, J5 뒷면 뚜껑은
    # J4 브래킷 팔(다리 쪽이 J5 뒷면 뚜껑 높이까지 내려온다)과 틈을 두는 크기로 줄인다.
    if layout.d45 - d5.r_out < d4.cap_out[1] + MOVING_GAP:
        d4 = replace(d4, cap_out_r=neck4_radius(layout, d5))
        d5 = replace(d5, cap_back_r=min(d5.r_out, layout.d45 - d4.cap_out[0] - MOVING_GAP))
    out.update({4: d4, 5: d5, 6: d6})
    return out


def neck4_radius(layout: Layout, d5: Drum) -> float:
    """J4 출력 쪽에서 J5 원통으로 가는 목의 반지름: J5 뚜껑 두 장 사이에 틈을 두고 들어가는 크기."""
    lo = layout.mid5 + d5.cap_back[1] + MOVING_GAP
    hi = layout.mid5 + (d5.cap_out[0] if d5.cap_out_r > 0 else math.inf) - MOVING_GAP
    return min(-lo, hi)


def build_ur_arm(
    layout: Layout, tool: Tool, look: Look | None = None
) -> tuple[Robot, list[Prim], list[Module], Camera]:
    """배치 검토 형상(packaging.build_arm)에 UR식 원통, 뚜껑, 관 링크를 넣은 형상과 D405 자리."""
    look = look or Look()
    robot, prims, mods = pk.build_arm(layout, tool)
    by_joint = {m.joint: m for m in mods}
    ds = drums(layout, mods, look)
    b = Builder([p for p in prims if p.name not in REPLACED])

    for m in mods:
        d, j = ds[m.joint], m.joint
        y0, y1 = d.ring
        b.cyl(
            j - 1,
            f"J{j}/ring",
            (0.0, (y0 + y1) / 2, 0.0),
            Y,
            d.r_out,
            y1 - y0,
            inner=d.r_in,
            frame=m.frame(),
            group=f"J{j} 고리",
        )
        for name, (c0, c1), r in (
            ("cap_out", d.cap_out, d.cap_out_r),
            ("cap_back", d.cap_back, d.cap_back_r),
        ):
            if r > 0:
                b.cyl(
                    j,
                    f"J{j}/{name}",
                    (0.0, (c0 + c1) / 2, 0.0),
                    Y,
                    r,
                    c1 - c0,
                    frame=m.child_frame(),
                    group=f"J{j} 뚜껑",
                )

    # J1 몸통(1): 돌림판 위에서 J2 원통 아래까지. 속이 빈 원통이고 J2 서보의 먼 쪽이 그 안에 들어간다.
    # 이름에 "기둥"을 쓰지 않는다. 카메라 검토(cameras.py)가 그 이름으로 책상의 카메라 기둥을 찾는다.
    # 어깨가 낮은 비교용 골격(0017의 처음 길이)은 J2 원통이 돌림판까지 내려와 몸통이 없다.
    sh = robot.joints[1].origin[2]
    col0, col1 = pk.TURNTABLE_TOP, sh - ds[2].r_out - MOVING_GAP
    if col1 > col0:
        b.cyl(
            1,
            "column",
            (0.0, 0.0, (col0 + col1) / 2),
            Z,
            look.column_r,
            col1 - col0,
            inner=look.column_r - WALL,
            group="J1 몸통",
        )

    # 위팔(2): J2 브래킷 팔을 늘인 판 두 장이 J2 원통 위의 가로판에서 만나고, 그 위로 관이 J3 집까지 간다.
    j2, d2 = by_joint[2], ds[2]
    yoke0 = d2.r_out + MOVING_GAP
    yoke1 = yoke0 + look.yoke
    for k, (y0, y1) in enumerate(_arms(j2.shape)):
        b.box(
            2,
            f"J2/arm{k}_tine",
            (-j2.arm_r, y0, j2.bridge_in),
            (j2.arm_r, y1, yoke1),
            group="위팔",
        )
    b.box(
        2,
        "yoke",
        (-j2.arm_r, d2.cap_back[0], yoke0),
        (j2.arm_r, d2.cap_out[1], yoke1),
        group="위팔",
    )
    j3, d3 = by_joint[3], ds[3]
    top2 = layout.upper - j3.shape.far - GAP - WALL
    b.cyl(
        2,
        "upper_tube",
        (0.0, 0.0, (yoke1 + top2) / 2),
        Z,
        look.upper_tube,
        top2 - yoke1,
        inner=look.upper_tube - look.tube_wall,
        group="위팔",
    )

    # 아래팔(3): J3 브래킷 팔을 아래팔 쪽으로 늘인 판 두 장이 J3 원통 밖에서 관 속으로 들어간다.
    tube0 = d3.r_out + MOVING_GAP
    for k, (y0, y1) in enumerate(_arms(j3.shape)):
        b.box(
            3,
            f"J3/arm{k}_root",
            (-j3.arm_r, y0, 0.0),
            (j3.arm_r, y1, tube0 + look.root_in),
            group="아래팔",
        )
    top3 = layout.fore - by_joint[4].shape.far - GAP - WALL
    b.cyl(
        3,
        "fore_tube",
        (0.0, 0.0, (tube0 + top3) / 2),
        Z,
        look.fore_tube,
        top3 - tube0,
        inner=look.fore_tube - look.tube_wall,
        group="아래팔",
    )

    # 손목 1(4): J4 출력 쪽 브래킷 팔 바깥에서 J5 고리까지 가는 목.
    # J5 고리가 J4 브래킷 팔 바로 옆이면 목이 없다.
    d5 = ds[5]
    n0, n1 = _arms(by_joint[4].shape)[0][1], layout.d45 - d5.r_in
    if n1 > n0:
        r_neck = neck4_radius(layout, d5)
        b.cyl(4, "J4/neck", (0.0, (n0 + n1) / 2, 0.0), Y, r_neck, n1 - n0, group="손목 1")

    # 손목 2(5): J5 출력 쪽 브래킷 팔 바깥에서 J6 고리까지 가는 목. J6 뒷면 뚜껑을 피해 그 앞(+y)에 둔다.
    j5, d6 = by_joint[5], ds[6]
    z0, z1 = layout.mid5 + _arms(j5.shape)[0][1], layout.d56 - d6.r_in
    y_lo = layout.mid6 + d6.cap_back[1] + MOVING_GAP
    if z1 > z0:
        b.box(
            5,
            "J5/neck",
            (-j5.arm_r, y_lo, z0),
            (j5.arm_r, j5.bridge_in + BRIDGE_T, z1),
            group="손목 2",
        )
    # D405: J6 원통 옆(+x), 원통이 도는 뚜껑과 틈을 두고 단다. 렌즈 면 위치와 광축 기울기는 0017과 같다.
    # 받침판은 카메라 뒷면(평평한 곳 36 × 36 mm)의 M3 구멍 두 개(간격 20 mm, 링크 5의 z 방향)를 잡고
    # J6 고리에 붙는다. 두께 6 mm는 어림이고 부품 설계에서 강성으로 정한다. 볼트는 판 두께에
    # 카메라 나사 깊이(4 mm 이하)를 더한 길이로 고른다 (0018).
    x_cam = d6.r_out + MOVING_GAP
    lens_y = layout.d6f - layout.camera_back
    back = lens_y - 0.023
    b.box(
        5,
        "d405_plate",
        (d6.r_in, back - D405_PLATE_T, layout.d56 - 0.018),
        (x_cam + 0.039, back, layout.d56 + 0.018),
        group="D405 받침",
    )
    b.box(
        5,
        "d405",
        (x_cam, lens_y - 0.023, layout.d56 - 0.021),
        (x_cam + 0.042, lens_y, layout.d56 + 0.021),
        material="camera",
        mass=pk.D405_MASS,
        group="D405",
    )
    t = layout.camera_tilt
    cam = Camera(
        np.array([x_cam + 0.021, lens_y, layout.d56]), np.array([-math.sin(t), math.cos(t), 0.0])
    )
    return robot, b.prims, mods, cam


# ----------------------------------------------------------------------------------------------------
# 프레임 휨


@dataclass(frozen=True)
class Beam:
    """링크 하나의 곧은 구조 구간. 굽힘 강성은 보 축에 수직인 두 축(e1, e2 = 보 축 × e1) 둘레다."""

    name: str
    link: int
    a: np.ndarray  # 링크 좌표계 [m]
    b: np.ndarray
    e1: np.ndarray  # 링크 좌표계 방향, 보 축과 수직
    ei1: float  # [N·m²]
    ei2: float
    gj: float


def tube_beam(name: str, link: int, z0: float, z1: float, r: float, t: float) -> Beam:
    """링크 z축을 따라가는 둥근 관."""
    i = math.pi / 4 * (r**4 - (r - t) ** 4)
    return Beam(
        name,
        link,
        np.array([0, 0, z0]),
        np.array([0, 0, z1]),
        np.array([1.0, 0, 0]),
        E_PLA * i,
        E_PLA * i,
        G_PLA * 2 * i,
    )


def box_beam(name: str, link: int, z0: float, z1: float, wx: float, wy: float, t: float) -> Beam:
    """링크 z축을 따라가는 속 빈 상자 (바깥 wx × wy, 벽 t)."""
    ix = (wy * wx**3 - (wy - 2 * t) * (wx - 2 * t) ** 3) / 12  # y축 둘레 (x 방향으로 휨)
    iy = (wx * wy**3 - (wx - 2 * t) * (wy - 2 * t) ** 3) / 12
    area = (wx - t) * (wy - t)
    j = 4 * area**2 * t / (2 * (wx - t) + 2 * (wy - t))
    return Beam(
        name,
        link,
        np.array([0, 0, z0]),
        np.array([0, 0, z1]),
        np.array([0, 1.0, 0]),
        E_PLA * ix,
        E_PLA * iy,
        G_PLA * j,
    )


def plate_pair_beam(
    name: str, link: int, z0: float, z1: float, width: float, t: float, spacing: float
) -> Beam:
    """링크 z축을 따라가는 판 두 장 (폭은 x, 두께는 y, 판 가운데 사이 거리 spacing).

    두 판은 양 끝(브래킷 다리와 가로판, 또는 관)이 잡아 준다고 본다. 그래서 판 면 밖 굽힘은 한 단면처럼,
    비틀림은 두 판이 판 면 안에서 서로 반대로 휘는 것으로 계산한다.
    """
    strong = t * width**3 / 12
    lateral = 2 * (width * t**3 / 12 + width * t * (spacing / 2) ** 2)
    length = z1 - z0
    torsion = 6 * E_PLA * strong * spacing**2 / length**2 + G_PLA * 2 * width * t**3 / 3
    return Beam(
        name,
        link,
        np.array([0, 0, z0]),
        np.array([0, 0, z1]),
        np.array([0, 1.0, 0]),
        E_PLA * 2 * strong,
        E_PLA * lateral,
        torsion,
    )


def ur_beams(layout: Layout, look: Look, mods: list[Module]) -> list[Beam]:
    by_joint = {m.joint: m for m in mods}
    ds = drums(layout, mods, look)
    j2, j3 = by_joint[2], by_joint[3]
    yoke0 = ds[2].r_out + MOVING_GAP
    tube0 = ds[3].r_out + MOVING_GAP
    (a0, a1), (b0, b1) = _arms(j2.shape)
    (c0, c1), (e0, e1) = _arms(j3.shape)
    return [
        plate_pair_beam("J2 판 두 장", 2, 0.0, yoke0, 2 * j2.arm_r, ARM_T, (a0 + a1 - b0 - b1) / 2),
        tube_beam(
            "위팔 관",
            2,
            yoke0 + look.yoke,
            layout.upper - j3.shape.far - GAP - WALL,
            look.upper_tube,
            look.tube_wall,
        ),
        plate_pair_beam("J3 판 두 장", 3, 0.0, tube0, 2 * j3.arm_r, ARM_T, (c0 + c1 - e0 - e1) / 2),
        tube_beam(
            "아래팔 관",
            3,
            tube0,
            layout.fore - by_joint[4].shape.far - GAP - WALL,
            look.fore_tube,
            look.tube_wall,
        ),
    ]


def box_beams(layout: Layout, mods: list[Module]) -> list[Beam]:
    """비교용: 배치 검토의 상자 링크 (packaging.build_arm)."""
    by_joint = {m.joint: m for m in mods}
    j2, j3 = by_joint[2], by_joint[3]
    (a0, a1), (b0, b1) = _arms(j2.shape)
    (c0, c1), (e0, e1) = _arms(j3.shape)
    root = j3.sweep_r + 1.0 * MM
    return [
        plate_pair_beam(
            "J2 판 두 장", 2, 0.0, j2.bridge_in, 2 * j2.arm_r, ARM_T, (a0 + a1 - b0 - b1) / 2
        ),
        box_beam(
            "위팔 상자",
            2,
            j2.bridge_in + BRIDGE_T,
            layout.upper - j3.shape.far - GAP - WALL,
            0.032,
            0.040,
            WALL,
        ),
        plate_pair_beam("J3 판 두 장", 3, 0.0, root, 2 * j3.arm_r, ARM_T, (c0 + c1 - e0 - e1) / 2),
        box_beam(
            "아래팔 상자",
            3,
            root,
            layout.fore - by_joint[4].shape.far - GAP - WALL,
            0.030,
            0.0358,
            WALL,
        ),
    ]


def tcp_deflection(
    robot: Robot, q: Mat, beams: list[Beam], masses: list[PointMass], tcp: Mat
) -> tuple[np.ndarray, dict[str, float]]:
    """자세 q에서 프레임이 휘어 TCP가 움직이는 벡터 [m]와 보마다의 몫 [m].

    보의 단면마다 그 너머(끝 쪽)에 걸린 무게의 모멘트로 굽힘과 비틀림을 구하고, 그 작은 회전이
    TCP를 옮기는 양을 더한다. tcp는 플랜지 좌표계에서 본 위치다.
    """
    frames = frame_poses(robot, q)
    n = len(robot.joints)
    loads = []
    for pm in masses:
        index = n + 1 if pm.link == FLANGE else pm.link
        T = frames[index]
        where = T[:3, :3] @ np.asarray(pm.position) + T[:3, 3]
        loads.append((min(index, n + 1), where, np.array([0.0, 0.0, -GRAVITY * pm.mass])))
    p_tcp = frames[n + 1][:3, :3] @ tcp + frames[n + 1][:3, 3]
    total = np.zeros(3)
    parts: dict[str, float] = {}
    for beam in beams:
        T = frames[beam.link]
        a = T[:3, :3] @ beam.a + T[:3, 3]
        b = T[:3, :3] @ beam.b + T[:3, 3]
        length = float(np.linalg.norm(b - a))
        u = (b - a) / length
        e1 = T[:3, :3] @ beam.e1
        e2 = np.cross(u, e1)
        steps = 40
        ds = length / steps
        moved = np.zeros(3)
        for i in range(steps):
            s = a + u * (i + 0.5) * ds
            M = np.zeros(3)
            for index, where, force in loads:
                if index > beam.link or (index == beam.link and (where - s) @ u > 0):
                    M += np.cross(where - s, force)
            turn = (M @ e1) / beam.ei1 * e1 + (M @ e2) / beam.ei2 * e2 + (M @ u) / beam.gj * u
            moved += np.cross(turn * ds, p_tcp - s)
        parts[beam.name] = float(np.linalg.norm(moved))
        total += moved
    return total, parts


def prim_masses(prims: list[Prim]) -> list[PointMass]:
    return [PointMass(p.name, min(p.link, 6), tuple(p.pos), p.mass) for p in prims if p.mass > 0]


def extended_pose() -> np.ndarray:
    """위팔과 아래팔을 수평으로 다 펴고 도구를 아래로 향한 자세 (J2 90°, J3 0°, J4 180°, J5 90°)."""
    return np.radians([0.0, 90.0, 0.0, 180.0, 90.0, 0.0])


# ----------------------------------------------------------------------------------------------------
# 보고


def drum_table(layout: Layout, look: Look) -> None:
    _, _, mods = pk.build_arm(layout, TOOLS["gripper"])
    ds = drums(layout, mods, look)
    print("\n### 관절 원통 크기 [mm]")
    print(
        "| 관절 | 지름 | 길이 (뚜껑 바깥 사이) | 고리 길이 | 고리 벽 | 출력 쪽 뚜껑 | 뒷면 뚜껑 |"
    )
    print("|---|---|---|---|---|---|---|")
    for j, d in ds.items():

        def cap(r: float, full: float) -> str:
            if r <= 0:
                return "없음 (목으로 이음)"
            return "지름 같음" if math.isclose(r, full) else f"지름 {2 * r * 1000:.0f}로 줄임"

        print(
            f"| J{j} | {2 * d.r_out * 1000:.1f} | {d.length * 1000:.1f} | "
            f"{(d.ring[1] - d.ring[0]) * 1000:.1f} | {(d.r_out - d.r_in) * 1000:.1f} | "
            f"{cap(d.cap_out_r, d.r_out)} | {cap(d.cap_back_r, d.r_out)} |"
        )


def flange_margin(robot: Robot, prims: list[Prim], tool: Tool, qs: list[np.ndarray]) -> float:
    """qs의 어느 자세에서도 버팀 토크가 정격의 RATED_MARGIN배를 넘지 않도록 플랜지에 더 달 수 있는 무게 [kg].

    케이블, 나사, 흡착 튜브처럼 형상에 없는 무게가 얼마까지 괜찮은지 본다. 플랜지에 다는 것은 가장 나쁜 경우다.
    """
    rated = np.array([j.effort for j in robot.joints])
    masses = [*prim_masses(prims), PointMass("짐", FLANGE, (0.0, 0.0, tool.length), tool.payload)]
    unit = [PointMass("더하는 무게", FLANGE, (0.0, 0.0, 0.0), 1.0)]
    best = math.inf
    for q in qs:
        t0 = holding_torques(robot, q, masses)
        t1 = holding_torques(robot, q, unit)
        for j in range(len(rated)):
            if abs(t1[j]) < 1e-9:
                continue
            limit = RATED_MARGIN * rated[j]
            if abs(t0[j]) >= limit:
                return 0.0
            # t0 + m t1 = ±limit 중 m ≥ 0인 가장 작은 m
            best = min(
                best,
                *((s * limit - t0[j]) / t1[j] for s in (1, -1) if (s * limit - t0[j]) / t1[j] >= 0),
            )
    return best


def compare(layout: Layout, look: Look) -> None:
    """상자 링크(지금 배치 검토)와 UR식을 같은 자세로 비교한다."""
    print("\n### 작업 자세: 부딪힘 없이 되는 자세 수와 버팀 토크 / 정격 (상자 링크 → UR식)")
    print("| 작업 (도구) | 자세 | 상자 링크 | UR식 | 버팀 토크 상자 링크 | 버팀 토크 UR식 |")
    print("|---|---|---|---|---|---|")
    margins = []
    for name, key, poses_of in pk.POSE_SETS:
        tool = TOOLS[key]
        rows = []
        for build in (_box, _ur):
            robot, prims = build(layout, tool, look)
            scene = pk.build_scene(robot, prims)
            res = pk.check_set(scene, AnalyticIK(robot), poses_of(tool), name, tool.symmetric)
            ratio = pk.torque_ratios(robot, prims, tool, res.qs) if res.qs else np.zeros(6)
            rows.append((res, ratio))
            if res.clear_qs:
                margins.append((build is _ur, flange_margin(robot, prims, tool, res.clear_qs)))
        (rb, tb), (ru, tu) = rows
        print(
            f"| {name} ({tool.name}) | {rb.total} | {rb.clear} | {ru.clear} | "
            f"J2 {tb[1]:.2f} · J3 {tb[2]:.2f} · J4 {tb[3]:.2f} | J2 {tu[1]:.2f} · J3 {tu[2]:.2f} · J4 {tu[3]:.2f} |"
        )
        extra = [f for f in ru.failures if f not in rb.failures]
        for line in extra[:8]:
            print(f"  - UR식에서만: {line}")
    box_margin = min(m for ur, m in margins if not ur)
    ur_margin = min(m for ur, m in margins if ur)
    print(
        f"- 여유: 플랜지에 {ur_margin * 1000:.0f} g(상자 링크 {box_margin * 1000:.0f} g)까지 더 달아도 위 자세에서 "
        f"버팀 토크가 정격의 {RATED_MARGIN}배 안이다. 케이블, 나사, 흡착 튜브는 형상 무게에 없다."
    )


def _box(layout: Layout, tool: Tool, look: Look) -> tuple[Robot, list[Prim]]:
    robot, prims, _ = pk.build_arm(layout, tool)
    return robot, prims


def _ur(layout: Layout, tool: Tool, look: Look) -> tuple[Robot, list[Prim]]:
    robot, prims, _, _ = build_ur_arm(layout, tool, look)
    return robot, prims


def ranges_and_view(layout: Layout, look: Look) -> None:
    tool = TOOLS["gripper"]
    robot, prims, mods, cam = build_ur_arm(layout, tool, look)
    scene = pk.build_scene(robot, prims)
    ik = AnalyticIK(robot)
    home = pk.solve(ik, pk.tool_pose((0.20, 0.0, 0.10), 0.0, 0.0, 0.0, tool.length))[0]
    rb, pb, _ = pk.build_arm(layout, tool)
    box_ranges = pk.joint_ranges(pk.build_scene(rb, pb), home)
    print("\n### 관절 범위 (평행 그리퍼, 앞쪽 20 cm 높이 10 cm 자세에서 하나씩 돌림) [°]")
    print("| 관절 | 0017 범위 | 상자 링크 | UR식 | UR식에서 막는 것 |")
    print("|---|---|---|---|---|")
    for j, ((lo, hi, why_lo, why_hi), (blo, bhi, _, _)) in enumerate(
        zip(pk.joint_ranges(scene, home), box_ranges, strict=True)
    ):
        joint = robot.joints[j]
        print(
            f"| J{j + 1} | {joint.lower / DEG:.0f} ~ {joint.upper / DEG:.0f} | {blo / DEG:.0f} ~ {bhi / DEG:.0f} | "
            f"{lo / DEG:.0f} ~ {hi / DEG:.0f} | 아래: {why_lo} / 위: {why_hi} |"
        )
    print("\n### D405 시야 (앞쪽 20 cm 높이 10 cm 자세, TCP 둘레 ±30 mm)")
    print(
        f"- 렌즈 가운데는 J6 축에서 {cam.lens[0] * 1000:.1f} mm 옆 (상자 링크는 {pk.camera_of(layout, mods).lens[0] * 1000:.1f} mm)"
    )
    for key in ("gripper", "suction", "pointer"):
        t = TOOLS[key]
        rb2, rp2, _, cam2 = build_ur_arm(layout, t, look)
        sc = pk.build_scene(rb2, rp2)
        q0 = pk.solve(AnalyticIK(rb2), pk.tool_pose((0.20, 0.0, 0.10), 0.0, 0.0, 0.0, t.length))[0]
        for turn in (0, 90) if key == "gripper" else (0,):
            q = q0.copy()
            q[5] += turn * DEG
            seen, off, dist = pk.camera_view(sc, q, t, cam2)
            ahead, _, _ = pk.camera_view(sc, q, t, cam2, ahead=0.06)
            label = t.name + (f" (손가락 방향 {turn}° 돌림)" if key == "gripper" else "")
            print(
                f"- {label}: TCP 둘레 {seen * 100:.0f}%, 6 cm 앞 {ahead * 100:.0f}% 보임. "
                f"TCP가 광축에서 {off / DEG:.0f}°, 거리 {dist * 100:.1f} cm"
            )
    gap, part = pk.adapter_clearance(scene, home)
    print(f"- 교체 어댑터에서 가장 가까운 다른 부품: {part} {gap * 1000:.1f} mm")


def mass_report(layout: Layout, look: Look) -> None:
    tool = TOOLS["gripper"]
    _, box_prims, _ = pk.build_arm(layout, tool)
    _, ur_prims, _, _ = build_ur_arm(layout, tool, look)

    def beyond(prims: list[Prim], link: int) -> float:
        return sum(p.mass for p in prims if p.link >= link and p.material != "tool")

    print("\n### 무게 (도구 빼고) [g]")
    print("| 묶음 | 상자 링크 | UR식 |")
    print("|---|---|---|")
    for label, link in (
        ("J2가 드는 것 (링크 2부터)", 2),
        ("J3가 드는 것 (링크 3부터)", 3),
        ("J4가 드는 것 (링크 4부터)", 4),
    ):
        print(
            f"| {label} | {beyond(box_prims, link) * 1000:.0f} | {beyond(ur_prims, link) * 1000:.0f} |"
        )
    box_groups, ur_groups = pk.mass_table(box_prims), pk.mass_table(ur_prims)
    print("\n덮개 부품 (UR식에만 있는 것):")
    for group, m in sorted(ur_groups.items(), key=lambda kv: -kv[1]):
        if group not in box_groups and m > 0.0005:
            print(f"- {group}: {m * 1000:.0f} g")


def deflection_report(layout: Layout, look: Look) -> None:
    print(
        f"\n### 프레임 휨 (서보·혼·체결 탄성 제외, PLA 탄성 계수 {E_PLA / 1e9:.1f} GPa 어림): TCP가 움직이는 양 [mm]"
    )
    print("| 경우 | 상자 링크 | UR식 | UR식에서 가장 큰 몫 |")
    print("|---|---|---|---|")
    gripper = TOOLS["gripper"]
    q_ext = extended_pose()
    cases = [
        ("팔을 수평으로 다 폄, 평행 그리퍼 + 짐 150 g", q_ext, gripper, gripper.payload),
        ("팔을 수평으로 다 폄, 도구 없이 플랜지에 300 g", q_ext, None, 0.300),
    ]
    robot_ur, ur_prims, mods, _ = build_ur_arm(layout, gripper, look)
    res = pk.check_set(
        pk.build_scene(robot_ur, ur_prims),
        AnalyticIK(robot_ur),
        pk.grasp_poses(gripper),
        gripper.name,
        gripper.symmetric,
    )
    if res.clear_qs:
        worst = max(res.clear_qs, key=lambda q: abs(float(q[1])) + abs(float(q[2])))
        cases.append(
            ("잡기 자세 중 가장 편 자세, 평행 그리퍼 + 짐 150 g", worst, gripper, gripper.payload)
        )
    for label, q, tool, payload in cases:
        cells = []
        best = ""
        for build, beams_of in ((_box, box_beams), (_ur, None)):
            if tool is None:
                robot, prims = build(layout, TOOLS["pointer"], look)
                prims = [p for p in prims if p.material != "tool" and p.group != "교체 어댑터"]
                reach = 0.0
            else:
                robot, prims = build(layout, tool, look)
                reach = tool.length
            masses = [*prim_masses(prims), PointMass("짐", FLANGE, (0.0, 0.0, reach), payload)]
            beams = beams_of(layout, mods) if beams_of else ur_beams(layout, look, mods)
            moved, parts = tcp_deflection(robot, q, beams, masses, np.array([0.0, 0.0, reach]))
            cells.append(f"{np.linalg.norm(moved) * 1000:.2f}")
            if beams_of is None:
                name, value = max(parts.items(), key=lambda kv: kv[1])
                best = f"{name} {value * 1000:.2f}"
        print(f"| {label} | {cells[0]} | {cells[1]} | {best} |")


def offset_table(look: Look) -> None:
    print("\n### 손목 오프셋별: 손목 원통 뚜껑과 부딪힘 없는 자세 수 (박스 쌓기 / 포인터 / 잡기)")
    print("| J4→J5 | J5→J6 | J4 출력 쪽 | J5 출력 쪽 | 박스 쌓기 | 포인터 | 잡기 |")
    print("|---|---|---|---|---|---|---|")
    for d45, d56 in (
        (0.055, 0.055),
        (0.058, 0.058),
        (0.060, 0.060),
        (0.062, 0.062),
        (0.063, 0.063),
    ):
        layout = Layout(d45=d45, d56=d56)
        _, _, mods = pk.build_arm(layout, TOOLS["gripper"])
        ds = drums(layout, mods, look)

        def end(d: Drum) -> str:
            if d.cap_out_r <= 0:
                return "목만"
            return "뚜껑" if math.isclose(d.cap_out_r, d.r_out) else "작은 뚜껑"

        cells = []
        for name, key, poses_of in pk.POSE_SETS:
            tool = TOOLS[key]
            robot, prims, _, _ = build_ur_arm(layout, tool, look)
            res = pk.check_set(
                pk.build_scene(robot, prims),
                AnalyticIK(robot),
                poses_of(tool),
                name,
                tool.symmetric,
            )
            cells.append(f"{res.clear}/{res.total}")
        print(
            f"| {d45 * 1000:.0f} | {d56 * 1000:.0f} | {end(ds[4])} | {end(ds[5])} | "
            + " | ".join(cells)
            + " |"
        )


LETTER_HEIGHT = 8.0 * MM  # 관 글자 높이 (0019, 어림)
LETTER_DEPTH = 0.5 * MM  # 같은 색으로 파서 새기는 깊이 (0019)


def groove_stiffness_loss(
    r_out: float, wall: float, depth: float, height: float, center: float, n: int = 2000
) -> float:
    """관 겉면에 띠 모양 홈(폭 height, 깊이 depth)을 팠을 때 줄어드는 굽힘 강성의 비율.

    center는 굽힘 방향에서 잰 홈 가운데의 각 [rad]이다 (0이면 처지는 쪽 면, π/2면 옆면).
    글자가 띠를 다 채운다고 보아 크게 잡는다.
    """
    r = np.linspace(r_out - wall, r_out, n)
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    rr, tt = np.meshgrid(r, th)
    da = (r[1] - r[0]) * (th[1] - th[0]) * rr
    second = (rr * np.cos(tt)) ** 2 * da
    off = np.abs((tt - center + math.pi) % (2 * math.pi) - math.pi)
    band = (rr >= r_out - depth) & (off <= height / 2 / r_out)
    return float(second[band].sum() / second.sum())


def lettering_report(look: Look) -> None:
    print(
        f"\n### 관 글자: 높이 {LETTER_HEIGHT / MM:.0f} mm, 깊이 {LETTER_DEPTH / MM:.1f} mm 띠를 파냈을 때 "
        "줄어드는 굽힘 강성 (글자가 띠를 다 채운다고 봄)"
    )
    print("| 관 | 양쪽 옆면 | 위아래 면 하나 |")
    print("|---|---|---|")
    for name, r_out in (("위팔", look.upper_tube), ("아래팔", look.fore_tube)):
        side = 2 * groove_stiffness_loss(
            r_out, look.tube_wall, LETTER_DEPTH, LETTER_HEIGHT, math.pi / 2
        )
        top = groove_stiffness_loss(r_out, look.tube_wall, LETTER_DEPTH, LETTER_HEIGHT, 0.0)
        print(f"| {name} | {side * 100:.2f}% | {top * 100:.1f}% |")


def report(layout: Layout, look: Look) -> None:
    print("\n# UR식 외형 검토 (STS 서보 외형은 도면과 실측, packaging.shapes)")
    drum_table(layout, look)
    mass_report(layout, look)
    compare(layout, look)
    ranges_and_view(layout, look)
    deflection_report(layout, look)
    lettering_report(look)
    offset_table(look)


def main() -> None:
    report(Layout(), Look())


if __name__ == "__main__":
    main()
