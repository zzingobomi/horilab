"""UR식 관절 팔의 검토 모델: 관절 부품(parts/joint.py)의 쌓임과 무게로 팔 전체를 세워 본다 (0029, 0017).

개념 형상(studies/ur_joints.py)과 달리, 축 방향 쌓임은 관절 CAD의 값이고 무게는 뽑아서 잰 부품에 맞춘 어림이다
(printcheck.printed_mass_g). 어깨(parts/shoulder.py)는 서보가 오기 전이라 서보 치수가 공식 도면 값이다.

짜임 (영점 자세, 링크 좌표는 베이스와 같은 방향):
- 자식은 출력 원판 윗면에 앉는다. 링크의 납작한 면과 그 링크 끝 집의 원통 앞 끝이 한 평면이다.
  그래서 위팔, 아래팔, 손목이 관절마다 옆(y)으로 한 칸씩 비킨다.
- 손목 몸통은 서보 먼 끝이 앞 관절 쪽을 본다.

    uv run python -m horilab_design.studies.ur_arm               # 쌓임, 옆 자리, 무게, 세운 자세의 범위
    uv run python -m horilab_design.studies.ur_arm compare       # UR3e와 같은 자세 시험 (0027, 4분쯤)
    uv run python -m horilab_design.studies.ur_arm zone          # 주 작업 영역 지표 (0017)
    uv run python -m horilab_design.studies.ur_arm lengths       # 링크 길이 후보의 주 작업 영역 지표

뒤에 키=값을 붙여 바꿔 볼 수 있다 (길이는 mm, 무게는 g):
    shoulder=180 upper=230 fore=170 extra=30 tube_scale=0.62 ranges=,-160:160,-160:160,20:340,-160:160,-160:160
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from functools import cached_property

import numpy as np

from horilab_design.parts import joint as jp
from horilab_design.parts import links as lp
from horilab_design.parts import shoulder as sp
from horilab_design.parts import wrist as wp
from horilab_design.robot import Joint, Robot, rotation
from horilab_design.statics import FLANGE, GRAVITY, PointMass, frame_poses, holding_torques
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur_compare as uc
from horilab_design.studies.common import RATED_MARGIN
from horilab_design.studies.covers import D405_PLATE_T
from horilab_design.studies.packaging import (
    DEG,
    MM,
    MOVING_GAP,
    TOOLS,
    TURNTABLE_T,
    TURNTABLE_TOP,
    Builder,
    Prim,
    ServoShape,
    Tool,
    X,
    Y,
    Z,
)
from horilab_design.values import ServoModel, load_measured, load_servos, servo_mass

G = 1e-3  # g → kg
# 무게 [kg]. 출력 부품은 맞춘 어림이다 (uv run python -m horilab_design.parts.wrist, ...links).
# tests/test_ur_arm.py가 CAD 값과 견준다.
BEARING_MASS: dict[ServoModel, float] = {
    "STS3215": 0.024,
    "STS3250": 0.024,
    "ST-3120": 0.031,
}  # NSK 카탈로그
# 원통 속 부품과 나사: 누름 틀 4.1, 서보 받침 3.3, 뒷덮개 5.3, 벽 나사 3.9, 너트 1.5, 서보 뒷면 나사 0.8, 덮개 나사 1.8
INNER_MASS = 21.0 * G
WRIST_BODY_MASS = 24.9 * G  # 손목 몸통
# 자식 쪽: 출력 원판 11.1, 허브 판 1.4, 혼(가운데 나사 포함, 실측) 2.0, 혼 볼트 넷 5.4, 몸통 나사 넷 3.4, 너트 1.3,
# 멈춤 나사와 너트 1.2
OUT_MASS = 25.8 * G
# 어깨 (uv run python -m horilab_design.parts.shoulder).
# 부모 쪽: 집(J1 몸통의 윗부분) 54.7, 누름 틀 4.2, 서보 받침 9.2, 뒷덮개 12.3, 벽 나사 여섯 4.6, 너트 여섯 1.8,
# 서보 뒷면 나사 여섯 2.9, 덮개 나사 둘 2.0
SHOULDER_HOUSE_MASS = 91.7 * G
# 자식 쪽: 출력 원판 16.7, 허브 판 2.5, 혼 FT110M과 가운데 나사 6.9(도면 부피로 어림), 혼 볼트 여덟 11.5,
# 위팔 나사 여섯 5.3, 너트 여섯 2.1, 멈춤 나사와 너트 1.2
SHOULDER_OUT_MASS = 46.2 * G
# 위팔의 납작한 면(어깨 출력 원판 윗면)이 J1 축에서 떨어진 거리. 어깨 쌓임이 길어진 만큼 서보를 출력 반대쪽으로
# 물려서 이 값을 지킨다 (0029: 골격과 손목 오프셋이 그대로이고, 어깨 원통이 J1 축을 가운데로 거의 대칭이다)
SHOULDER_PLANE = 41.5 * MM
# 링크 한 부품을 세 덩어리로 나눈 값: 끝 집 쪽 22.8 g(끝 관절 축에서 10 mm 앞 관절 쪽), 관(길이당 무게 × 관 길이.
# 관은 발 상자 끝에서 끝 집의 목(축에서 40 mm)까지다), 발(앞 관절 축). 220 mm 위팔 92.9 g · 부피 중심 114.7 mm,
# 180 mm 아래팔 72.2 g · 88.3 mm와 맞는다. 길이 후보를 견줄 때 관만 길어진다.
LINK_HOUSE_MASS = 22.8 * G
LINK_TUBE: dict[
    str, tuple[float, float, float]
] = {  # (관 kg/m, 발 kg, 부피 중심의 서보 좌표 y [m])
    "upper": (0.320, 22.7 * G, 0.0129),
    "fore": (0.287, 16.7 * G, 0.0134),
}
D405_PLATE_MASS = 8.0 * G
SHELL = 1.008e3 * 2.6e-3  # 벽 2.6 mm 껍데기의 넓이당 무게 [kg/m²] (J1 몸통 기둥의 어림에만 쓴다)
HEAD = wp.HEAD * MM  # 출력 원판 윗면에서 다음 서보 먼 끝까지
FLANGE_T = 6.0 * MM  # 플랜지 원판 (도구 교체부는 그리퍼와 같이 그린다)
# 발 상자의 (반 길이, 반폭, 윗모서리를 깎은 깊이): 위팔은 어깨 출력 원판에, 아래팔은 STS 출력 원판에 앉는다
FEET = {
    key: (f.hx * MM, f.hw * MM, f.chamfer * MM)
    for key, f in (("upper", lp.SHOULDER_FOOT), ("fore", lp.STS_FOOT))
}
NECK_HW = jp.NECK_HW * MM  # STS 집의 목 반폭
LINK_NAMES = ("J1 몸통", "위팔", "아래팔", "손목 1", "손목 2", "손목 3")
DEFAULT_RANGES = (
    (0.0, 165.0),
    (0.0, 160.0),
    (0.0, 160.0),
    (180.0, 160.0),
    (0.0, 160.0),
    (0.0, 160.0),
)


@dataclass(frozen=True)
class Stack:
    """관절 하나의 축 방향 쌓임. 서보 좌표계 y(출력 쪽이 +) [m]."""

    radius: float
    back: float  # 뒷덮개 바깥 면
    bearing: tuple[float, float]
    rim: float  # 원통 앞 끝
    seam: float  # 출력 원판 밑면
    atop: float  # 출력 원판 윗면 = 자식 몸통이 앉는 면


def stack(shape: ServoShape) -> Stack:
    if shape.model != "ST-3120":
        return Stack(
            jp.R_DRUM * MM,
            jp.BACK_OUT * MM,
            (jp.B0 * MM, (jp.B0 + jp.BRG_B) * MM),
            jp.FRONT * MM,
            jp.SEAM * MM,
            jp.ATOP * MM,
        )
    # 어깨: parts/shoulder.py의 쌓임. 그 좌표의 원점은 몸체 두께의 가운데다
    mid = _body_mid(shape)
    return Stack(
        sp.R_DRUM * MM,
        mid + sp.BACK_OUT * MM,
        (mid + sp.B0 * MM, mid + (sp.B0 + sp.BRG_B) * MM),
        mid + sp.FRONT * MM,
        mid + sp.SEAM * MM,
        mid + sp.ATOP * MM,
    )


def _body_mid(shape: ServoShape) -> float:
    back_face = min(z[2] for z in shape.zones if z[0] <= 0.0 <= z[1])
    return (back_face + max(z[3] for z in shape.zones)) / 2


@dataclass(frozen=True)
class Concept:
    """팔 하나의 치수와 배치 [m]."""

    shoulder: float = 0.16
    upper: float = 0.22
    fore: float = 0.18
    upper_r: float = 0.020
    fore_r: float = 0.018
    column_r: float = 0.032
    sign4: int = 1  # J4 출력이 보는 쪽 (+1: 위팔 쪽으로 돌아온다)
    cam_side: int = 1  # D405를 다는 쪽 (링크 5의 x)
    tube_scale: float = 1.0  # 관 무게에 곱하는 값 (벽을 얇게 할 때)
    extra: float = 0.0  # 플랜지에 더 다는 무게 [kg] (형상에 없는 것: 선, 나사)
    camera_back: float = 0.005
    ranges_deg: tuple[tuple[float, float], ...] = DEFAULT_RANGES  # (가운데, 반폭)

    @cached_property
    def shapes(self) -> dict[ServoModel, ServoShape]:
        return pk.shapes()

    def st(self, model: ServoModel) -> Stack:
        return stack(self.shapes[model])

    @property
    def embed(self) -> float:
        return lp.EMBED * MM

    @property
    def mid(self) -> float:
        """손목 서보 원점의 축 방향 자리: 앞 관절 축이 뒷덮개 ~ 원통 앞 끝의 가운데를 지난다."""
        return -wp.AX_Y * MM

    @property
    def c2(self) -> float:
        """J2 서보 원점의 y: 출력 원판 윗면이 SHOULDER_PLANE에 오게 서보를 출력 반대쪽으로 물린다."""
        return SHOULDER_PLANE - self.st("ST-3120").atop

    @property
    def ya2(self) -> float:
        """위팔의 납작한 면(J2 출력 원판 윗면)의 y. J3 원통 앞 끝도 이 평면이다."""
        return self.c2 + self.st("ST-3120").atop

    @property
    def s2(self) -> float:
        return self.ya2 + self.upper_r - self.embed

    @property
    def c3(self) -> float:
        return self.ya2 + self.st("STS3250").rim

    @property
    def yb3(self) -> float:
        """아래팔의 납작한 면(J3 출력 원판 윗면)의 y. J4 원통 앞 끝도 이 평면이다."""
        return self.c3 - self.st("STS3250").atop

    @property
    def s3(self) -> float:
        return self.yb3 - (self.fore_r - self.embed)

    @property
    def c4(self) -> float:
        return self.yb3 - self.sign4 * self.st("STS3215").rim

    @property
    def yb4(self) -> float:
        return self.c4 + self.sign4 * self.st("STS3215").atop

    @property
    def far(self) -> float:
        return self.shapes["STS3215"].far

    @property
    def d4(self) -> float:
        return self.yb4 + self.sign4 * (HEAD + self.far)

    @property
    def d5(self) -> float:
        return self.mid + self.st("STS3215").atop + HEAD + self.far

    @property
    def d6(self) -> float:
        return self.mid + self.st("STS3215").seam + FLANGE_T


def robot_of(c: Concept) -> Robot:
    servos = load_servos()
    models: tuple[ServoModel, ...] = (
        "STS3215",
        "ST-3120",
        "STS3250",
        "STS3215",
        "STS3215",
        "STS3215",
    )
    origins = (
        (0.0, 0.0, 0.0),
        (0.0, 0.0, c.shoulder),
        (0.0, 0.0, c.upper),
        (0.0, 0.0, c.fore),
        (0.0, c.d4, 0.0),
        (0.0, 0.0, c.d5),
    )
    axes = (Z, Y, Y, Y, Z, Y)
    joints = tuple(
        Joint(
            f"j{k + 1}",
            origins[k],
            axes[k],
            (mid - half) * DEG,
            (mid + half) * DEG,
            servos[models[k]].rated_torque,
            servos[models[k]].max_speed,
        )
        for k, (mid, half) in enumerate(c.ranges_deg)
    )
    flange = np.eye(4)
    flange[:3, :3] = rotation(X, -np.pi / 2) @ rotation(Z, -np.pi / 2)
    flange[:3, 3] = (0.0, c.d6, 0.0)
    return Robot("horibot UR식 관절", joints, flange)


def build(c: Concept, tool: Tool) -> tuple[Robot, list[Prim]]:
    """검토 형상(상자, 원기둥, 볼록 껍질)과 무게. 좌표는 링크 좌표 [m]."""
    robot = robot_of(c)
    servos, measured = load_servos(), load_measured()
    sh = c.shapes
    b = Builder()

    def joint(
        j: int,
        model: ServoModel,
        origin: tuple[float, float, float],
        axis: tuple[float, float, float],
        near: tuple[float, float, float],
        out: int,
        mid: float,
        *,
        house_mass: float,
        wrist: bool = False,
        out_t: float = jp.BRIM_T * MM,
        neck_hw: float = NECK_HW,
        neck_x: float | None = None,
    ) -> None:
        shape = sh[model]
        m = pk.Module(j, shape, origin, axis, near, out, mid)
        s = c.st(model)
        f = m.frame()
        volumes = [(x1 - x0) * (y1 - y0) * (z1 - z0) for x0, x1, y0, y1, z0, z1 in shape.zones]
        mass = servo_mass(model, servos, measured)
        for k, (x0, x1, y0, y1, z0, z1) in enumerate(shape.zones):
            b.box(
                j - 1,
                f"J{j}/servo_{k}",
                (x0, y0, z0),
                (x1, y1, z1),
                material="servo",
                mass=mass * volumes[k] / sum(volumes),
                frame=f,
                group=f"J{j} 서보",
            )
        b.cyl(
            j - 1,
            f"J{j}/house",
            (0.0, (s.back + s.rim) / 2, 0.0),
            Y,
            s.radius,
            s.rim - s.back,
            mass=house_mass,
            frame=f,
            group=f"J{j} 집",
        )
        x0 = -(shape.far + (HEAD if wrist else 3.5 * MM)) if neck_x is None else neck_x
        x1 = -math.sqrt(max(s.radius**2 - neck_hw**2, 0.0))
        b.box(
            j - 1,
            f"J{j}/far_house",
            (x0, s.back, -neck_hw),
            (x1, s.rim, neck_hw),
            mass=0.0,
            frame=f,
            group=f"J{j} 집",
        )
        b.cyl(
            j - 1,
            f"J{j}/bearing",
            (0.0, (s.bearing[0] + s.bearing[1]) / 2, 0.0),
            Y,
            s.radius - 2.6 * MM,
            0.007,
            material="metal",
            mass=BEARING_MASS[model],
            frame=f,
            group=f"J{j} 집",
        )
        b.cyl(
            j,
            f"J{j}/cap",
            (0.0, s.seam + out_t / 2, 0.0),
            Y,
            s.radius,
            out_t,
            mass=SHOULDER_OUT_MASS if model == "ST-3120" else OUT_MASS,
            frame=m.child_frame(),
            group=f"J{j} 출력 원판",
        )

    def d_tube(
        link: int, name: str, y_center: float, radius: float, z0: float, z1: float, y_cut: float
    ) -> None:
        """z 방향 관. 관절 쪽은 y_cut 평면으로 납작하다. 볼록 껍질로 넣고, 무게는 link_body가 따로 넣는다."""
        side = 1.0 if y_center > y_cut else -1.0
        pts = []
        for k in range(48):
            a = 2 * math.pi * k / 48
            x, y = radius * math.cos(a), y_center + radius * math.sin(a)
            if (y - y_cut) * side >= 0:
                pts.append((x, y))
        half = math.sqrt(radius**2 - (y_cut - y_center) ** 2)
        pts += [(half, y_cut), (-half, y_cut)]
        verts = np.array([(x, y, z) for z in (z0, z1) for x, y in pts])
        center = np.array([0.0, y_center, (z0 + z1) / 2])
        b.prims.append(
            Prim(name, link, "hull", center, np.eye(3), (0.0, 0.0, 0.0), "pla", 0.0, "관", verts)
        )

    def link_body(link: int, key: str, length: float, c_y: float, out: int, group: str) -> None:
        """링크 한 부품: 발 상자의 형상과, 세 덩어리로 나눈 무게 (끝 집, 관, 발)."""
        k, m_foot, y_s = LINK_TUBE[key]
        y = c_y + out * y_s
        hx = FEET[key][0]
        tube_len = length - hx - 0.040
        lumps = (
            (LINK_HOUSE_MASS, length - 0.010),
            (k * tube_len * c.tube_scale, hx + tube_len / 2),
            (m_foot, 0.0),
        )
        for i, (m, z) in enumerate(lumps):
            lo, hi = (-0.002, y - 0.002, z - 0.002), (0.002, y + 0.002, z + 0.002)
            b.box(link, f"{key}_mass_{i}", lo, hi, mass=m, group=group)

    def foot(
        link: int, key: str, name: str, flat: float, out: int, height: float, group: str
    ) -> None:
        """발 상자: 납작한 면(flat)에서 out 쪽으로 height만큼 선다. 윗모서리 둘은 45°로 깎여 있다 (볼록 껍질)."""
        hx, hw, ch = FEET[key]
        top = flat + out * height
        section = [
            (-hw, flat),
            (hw, flat),
            (hw, top - out * ch),
            (hw - ch, top),
            (ch - hw, top),
            (-hw, top - out * ch),
        ]
        verts = np.array([(x, y, z) for z in (-hx, hx) for x, y in section])
        center = np.array([0.0, (flat + top) / 2, 0.0])
        b.prims.append(
            Prim(name, link, "hull", center, np.eye(3), (0.0, 0.0, 0.0), "pla", 0.0, group, verts)
        )

    st4 = c.st("STS3215")
    h, a2, a3 = c.shoulder, c.upper, c.fore

    base_h = TURNTABLE_TOP - TURNTABLE_T - 0.5 * MM
    b.cyl(0, "base", (0.0, 0.0, base_h / 2), Z, 0.046, base_h, mass=0.0, group="베이스")
    b.cyl(
        1,
        "turntable",
        (0.0, 0.0, TURNTABLE_TOP - TURNTABLE_T / 2),
        Z,
        0.044,
        TURNTABLE_T,
        group="돌림판",
    )
    # 어깨 집의 목은 서보 먼 끝 아래까지 내려오고, 어깨가 낮으면 돌림판 윗면에서 끝난다. 그 아래가 기둥이다
    neck_x = -min(-sp.NECK_X * MM, h - TURNTABLE_TOP)
    col_top = h + neck_x
    if col_top > TURNTABLE_TOP:
        col_h = col_top - TURNTABLE_TOP
        b.cyl(
            1,
            "column",
            (0.0, 0.0, (TURNTABLE_TOP + col_top) / 2),
            Z,
            c.column_r,
            col_h,
            mass=SHELL * 2 * math.pi * c.column_r * col_h,
            group="J1 몸통",
        )

    # J2: 서보 출력이 +y. 위팔 = 발 + 납작한 관 + J3 집
    joint(
        2,
        "ST-3120",
        (0.0, 0.0, h),
        Y,
        Z,
        1,
        c.c2,
        house_mass=SHOULDER_HOUSE_MASS,
        neck_hw=sp.NECK_HW * MM,
        neck_x=neck_x,
    )
    foot(2, "upper", "upper_foot", c.ya2, 1, 2 * c.upper_r - c.embed, "위팔")
    d_tube(2, "upper_tube", c.s2, c.upper_r, FEET["upper"][0], a2 - 0.036, c.ya2)
    link_body(2, "upper", a2, c.c3, -1, "위팔")
    # J3: 위팔 끝, 출력이 -y. 원통 앞 끝이 위팔의 납작한 면과 한 평면
    joint(3, "STS3250", (0.0, 0.0, a2), Y, Z, -1, c.c3, house_mass=INNER_MASS)
    foot(3, "fore", "fore_foot", c.yb3, -1, 2 * c.fore_r - c.embed, "아래팔")
    d_tube(3, "fore_tube", c.s3, c.fore_r, FEET["fore"][0], a3 - 0.036, c.yb3)
    link_body(3, "fore", a3, c.c4, c.sign4, "아래팔")
    # J4: 아래팔 끝. J5, J6은 손목 몸통 (서보 먼 끝이 앞 관절 쪽)
    joint(4, "STS3215", (0.0, 0.0, a3), Y, Z, c.sign4, c.c4, house_mass=INNER_MASS)
    wrist_house = WRIST_BODY_MASS + INNER_MASS
    near5 = (0.0, float(c.sign4), 0.0)
    joint(5, "STS3215", (0.0, c.d4, 0.0), Z, near5, 1, c.mid, house_mass=wrist_house, wrist=True)
    joint(
        6,
        "STS3215",
        (0.0, 0.0, c.d5),
        Y,
        Z,
        1,
        c.mid,
        house_mass=wrist_house,
        wrist=True,
        out_t=FLANGE_T,
    )

    # D405: J6 원통 옆(cam_side 쪽 x)에 움직이는 틈을 두고 단다
    sgn = c.cam_side
    x_in = st4.radius + MOVING_GAP
    lens_y = c.d6 - c.camera_back
    back = lens_y - 0.023

    def xr(a: float, bb: float) -> tuple[float, float]:
        lo_, hi_ = sorted((sgn * a, sgn * bb))
        return lo_, hi_

    px = xr(st4.radius - 2.6 * MM, x_in + 0.039)
    b.box(
        5,
        "d405_plate",
        (px[0], back - D405_PLATE_T, c.d5 - 0.018),
        (px[1], back, c.d5 + 0.018),
        mass=D405_PLATE_MASS,
        group="D405 받침",
    )
    cx = xr(x_in, x_in + 0.042)
    b.box(
        5,
        "d405",
        (cx[0], back, c.d5 - 0.021),
        (cx[1], lens_y, c.d5 + 0.021),
        material="camera",
        mass=pk.D405_MASS,
        group="D405",
    )
    pk.add_tool(b, 6, c.d6, tool)
    if c.extra > 0:
        lo, hi = (-0.002, c.d6 - 0.004, -0.002), (0.002, c.d6, 0.002)
        b.box(6, "extra", lo, hi, mass=c.extra, group="형상에 없는 무게")
    return robot, b.prims


def link_masses(prims: list[Prim]) -> list[float]:
    out = [0.0] * 7
    for p in prims:
        out[min(p.link, 6)] += p.mass
    return out


def hori(c: Concept) -> uc.Hori:
    robot, prims = build(c, uc.TOOL)
    ranges = (robot.lower, robot.upper)
    return uc.make_hori(robot, prims, ranges, ranges)


def describe(c: Concept) -> str:
    rng = " ".join(
        f"J{k + 1}[{mid - half:.0f},{mid + half:.0f}]" for k, (mid, half) in enumerate(c.ranges_deg)
    )
    return (
        f"어깨 {c.shoulder * 1000:.0f} 위팔 {c.upper * 1000:.0f} 아래팔 {c.fore * 1000:.0f} | "
        f"손목 {c.d4 / MM:.1f} · {c.d5 / MM:.1f} · {c.d6 / MM:.1f} | {rng}"
    )


def check(c: Concept, verbose: bool = True) -> dict:
    """쌓임, 옆 자리, 링크 무게와, 세운 자세에서 관절을 하나씩 돌려 부품이 닿기 전까지의 범위."""
    robot, prims = build(c, uc.TOOL)
    scene = pk.build_scene(robot, prims)
    q0 = np.array([mid * DEG for mid, _ in c.ranges_deg])
    q0[1] = q0[2] = q0[4] = 0.0
    bad = pk.collisions(pk.hits(scene, q0))
    ranges = pk.joint_ranges(scene, q0)
    masses = link_masses(prims)
    out = {
        "upright_hits": [f"{hit.a} ↔ {hit.b}" for hit in bad],
        "ranges": [
            ((lo - q0[j]) / DEG, (hi - q0[j]) / DEG, wl, wh)
            for j, (lo, hi, wl, wh) in enumerate(ranges)
        ],
        "masses": masses,
        "j2_load": sum(masses[2:]),
    }
    if not verbose:
        return out
    print("##", describe(c))
    for model in ("ST-3120", "STS3250", "STS3215"):
        s = c.st(model)
        print(
            f"- {model}: 원통 지름 {2 * s.radius / MM:.1f}, 뒷덮개 {s.back / MM:.1f} ~ 원통 앞 끝 {s.rim / MM:.1f} ~ "
            f"출력 원판 {s.seam / MM:.1f} ~ {s.atop / MM:.1f}, 베어링 {s.bearing[0] / MM:.1f} ~ {s.bearing[1] / MM:.1f}"
        )
    st3, st4 = c.st("STS3250"), c.st("STS3215")
    print(
        f"- 옆 자리 y [mm]: 위팔의 납작한 면 {c.ya2 / MM:.1f} (관 가운데 {c.s2 / MM:.1f}), 아래팔 {c.yb3 / MM:.1f} "
        f"({c.s3 / MM:.1f}), 손목 1 {c.yb4 / MM:.1f} | J3 뒷덮개 {(c.c3 - st3.back) / MM:.1f}, "
        f"J4 뒷덮개 {(c.c4 + c.sign4 * st4.back) / MM:.1f}"
    )
    print(f"- 세운 자세 부딪힘: {', '.join(out['upright_hits']) or '없음'}")
    for j, (lo, hi, wl, wh) in enumerate(out["ranges"]):
        print(f"  - J{j + 1}: {lo:.0f}° ~ {hi:.0f}° ({wl} / {wh})")
    print(
        "- 링크 무게 [g]: "
        + ", ".join(f"{n} {m * 1000:.0f}" for n, m in zip(LINK_NAMES, masses[1:], strict=True))
        + f" | J2가 드는 무게 {out['j2_load'] * 1000:.0f} g"
    )
    return out


def compare(c: Concept, every: int = 1) -> dict:
    """UR3e와 같은 자세 시험 (0027). UR3e가 되는 자세 중 호리봇도 되는 몫과, 못 하는 까닭."""
    cmp = uc.compare(hori=hori(c), every=every, label=describe(c))
    res = cmp.results
    ur_ok = [r for r in res if r.ur[0] == "됨"]
    both = [r for r in ur_ok if r.hori[0] == "됨"]
    only_hori = [r for r in res if r.ur[0] != "됨" and r.hori[0] == "됨"]
    reasons: dict[str, int] = {}
    detail: dict[str, dict[str, int]] = {}
    for r in ur_ok:
        if r.hori[0] != "됨":
            reasons[r.hori[0]] = reasons.get(r.hori[0], 0) + 1
            d = detail.setdefault(r.hori[0], {})
            d[r.hori[1]] = d.get(r.hori[1], 0) + 1
    return {
        "n": len(res),
        "ur_ok": len(ur_ok),
        "both": len(both),
        "share": len(both) / max(len(ur_ok), 1),
        "only_hori": len(only_hori),
        "reasons": reasons,
        "detail": detail,
    }


# ------------------------------------------------------------------------------------------------
# 주 작업 영역 지표 (0017: 반지름 16 ~ 30 cm, TCP 높이 2 ~ 18 cm, 도구가 아래)

RADII = [round(0.16 + 0.02 * i, 2) for i in range(8)]
HEIGHTS = [0.02, 0.06, 0.10, 0.14, 0.18]
# 관절마다 백래시 [rad] (docs/hardware.md의 다른 사람이 잰 값): STS3215 0.62°, ST-3120 0.5°(어림), STS3250 0.43°
BACKLASH = np.radians([0.62, 0.5, 0.43, 0.62, 0.62, 0.62])
# 서보를 스프링으로 본 강성 [N·m/rad]: STS3215 15 kg·cm에 2.6°, STS3250 20 kg·cm에 1.23° (다른 사람이 잰 처짐).
# ST-3120은 잰 값이 없어 정격 토크에 비례로 어림했다
STIFFNESS = np.array([31.0, 125.0, 91.0, 31.0, 31.0, 31.0])
BIAS_MIN = 0.03  # [N·m] 중력 토크가 이보다 크면 백래시가 한쪽에 붙는다고 본다 (0013)


def _jacobian(robot: Robot, q: np.ndarray, tool_len: float) -> tuple[np.ndarray, np.ndarray]:
    frames = frame_poses(robot, q)
    tf = frames[-1]
    p = tf[:3, 3] + tf[:3, 2] * tool_len
    jv, jw = np.zeros((3, 6)), np.zeros((3, 6))
    for i in range(6):
        t = frames[i + 1]
        z = t[:3, :3] @ np.asarray(robot.joints[i].axis)
        jv[:, i] = np.cross(z, p - t[:3, 3])
        jw[:, i] = z
    return jv, jw


def _mass_matrix(robot: Robot, q: np.ndarray, masses: list[PointMass]) -> np.ndarray:
    frames = frame_poses(robot, q)
    m = np.zeros((6, 6))
    for pm in masses:
        idx = 7 if pm.link == FLANGE else pm.link
        if idx == 0:
            continue
        t = frames[idx]
        p = t[:3, :3] @ np.asarray(pm.position) + t[:3, 3]
        jac = np.zeros((3, 6))
        for i in range(min(idx, 6)):
            ti = frames[i + 1]
            z = ti[:3, :3] @ np.asarray(robot.joints[i].axis)
            jac[:, i] = np.cross(z, p - ti[:3, 3])
        m += pm.mass * jac.T @ jac
    return m + np.eye(6) * 2e-5  # 부품 자체의 회전 관성 몫 (작은 값, 특이 방지)


def _tilt_moments(
    robot: Robot, q: np.ndarray, prims: list[Prim], masses: list[PointMass]
) -> dict[int, float]:
    """관절 2 ~ 6의 출력 면 베어링에 걸리는 기울이는 모멘트 [N·m] (축에 수직인 성분)."""
    frames = frame_poses(robot, q)
    out = {}
    for j in range(2, 7):
        brg = next(p for p in prims if p.name == f"J{j}/bearing")
        tp = frames[j - 1]
        center = tp[:3, :3] @ brg.pos + tp[:3, 3]
        axis = frames[j][:3, :3] @ np.asarray(robot.joints[j - 1].axis)
        moment = np.zeros(3)
        for pm in masses:
            idx = 7 if pm.link == FLANGE else pm.link
            if idx < j:
                continue
            t = frames[idx]
            p = t[:3, :3] @ np.asarray(pm.position) + t[:3, 3]
            moment += np.cross(p - center, np.array([0.0, 0.0, -GRAVITY * pm.mass]))
        out[j] = float(np.linalg.norm(moment - (moment @ axis) * axis))
    return out


def _branch(q: np.ndarray) -> str:
    front = abs((q[0] + math.pi) % (2 * math.pi) - math.pi) < math.pi / 2
    return ("앞" if front else "뒤") + ("위" if q[2] > 0 else "아래") + ("+" if q[4] > 0 else "-")


def fit_ranges(
    q: np.ndarray, lower: np.ndarray, upper: np.ndarray, symmetric: bool = True
) -> np.ndarray | None:
    """관절 범위 안에 드는 같은 자세의 각 (2π 차이, 대칭 도구면 J6 ± 180°). 없으면 None."""
    out = np.array(q, dtype=float)
    for j in range(6):
        turns = (0.0, math.pi, -math.pi) if (j == 5 and symmetric) else (0.0,)
        found = False
        for extra in turns:
            for k in (0, -1, 1):
                a = q[j] + extra + 2 * math.pi * k
                if lower[j] - 1e-9 <= a <= upper[j] + 1e-9:
                    out[j], found = a, True
                    break
            if found:
                break
        if not found:
            return None
    return out


def good_solutions(h: uc.Hori, pose: np.ndarray) -> list[np.ndarray]:
    """관절 범위 안, 부딪힘 없음, 버팀 토크가 정격의 0.8배 안인 해."""
    out = []
    for q in h.ik.solve(pose):
        fit = fit_ranges(q, h.robot.lower, h.robot.upper)
        if fit is None or pk.collisions(pk.hits(h.scene, fit)):
            continue
        if h.torque(fit)[1:].max() <= RATED_MARGIN:
            out.append(fit)
    return out


def zone(c: Concept) -> dict:
    """주 작업 영역 40칸의 지표. 도구는 평행 그리퍼(+ 짐 150 g), 도구 돌림 0°와 90°가 다 되는 칸만 센다."""
    tool = TOOLS["gripper"]
    h = hori(c)
    robot, prims = h.robot, h.prims
    by_branch: dict[str, dict[tuple[float, float], list[np.ndarray]]] = {}
    any_ok = 0
    for r in RADII:
        for z in HEIGHTS:
            per: dict[str, list[tuple[float, np.ndarray]]] = {}
            for yaw in (0.0, 90 * DEG):
                pose = pk.tool_pose((r, 0.0, z), 0.0, 0.0, yaw, tool.length)
                for q in good_solutions(h, pose):
                    per.setdefault(_branch(q), []).append((yaw, q))
            ok = False
            for name, lst in per.items():
                if len({yaw for yaw, _ in lst}) == 2:
                    by_branch.setdefault(name, {})[(r, z)] = [q for _, q in lst]
                    ok = True
            any_ok += ok
    if not by_branch:
        return {"cells": len(RADII) * len(HEIGHTS), "any": 0}
    best = max(by_branch, key=lambda name: len(by_branch[name]))
    torque = np.zeros(6)
    sin3 = 1.0
    lash_free = 0.0
    acc_tcp, f1 = math.inf, math.inf
    tilt = {j: 0.0 for j in range(2, 7)}
    for qs in by_branch[best].values():
        q = min(qs, key=lambda q: float(h.torque(q)[1:].max()))
        torque = np.maximum(torque, h.torque(q))
        sin3 = min(sin3, abs(math.sin(q[2])))
        jv, jw = _jacobian(robot, q, tool.length)
        tau = holding_torques(robot, q, h.masses)
        free = np.abs(tau) < BIAS_MIN
        lash_free = max(lash_free, float(np.linalg.norm(jv * BACKLASH, axis=0)[free].sum()))
        m = _mass_matrix(robot, q, h.masses)
        jac = np.vstack([jv, jw])
        try:
            jinv = np.linalg.inv(jac)
        except np.linalg.LinAlgError:
            jinv = np.linalg.pinv(jac)
        for u in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            v = m @ jinv @ np.array([*u, 0.0, 0.0, 0.0])
            for i in range(6):
                if abs(v[i]) > 1e-12:
                    lim = (
                        (h.rated[i] - tau[i]) / v[i] if v[i] > 0 else (-h.rated[i] - tau[i]) / v[i]
                    )
                    acc_tcp = min(acc_tcp, float(lim))
        w2 = np.linalg.eigvals(np.linalg.solve(m, np.diag(STIFFNESS))).real
        f1 = min(f1, float(math.sqrt(max(w2.min(), 0.0)) / (2 * math.pi)))
        for j, v in _tilt_moments(robot, q, prims, h.masses).items():
            tilt[j] = max(tilt[j], v)
    return {
        "cells": len(RADII) * len(HEIGHTS),
        "any": any_ok,
        "branches": {k: len(v) for k, v in sorted(by_branch.items(), key=lambda kv: -len(kv[1]))},
        "best": best,
        "torque": [round(float(t), 3) for t in torque],
        "sin3": round(sin3, 3),
        "lash_free_mm": round(lash_free * 1000, 2),
        "acc_tcp": round(acc_tcp, 2),
        "f1_hz": round(f1, 2),
        "tilt_nm": {j: round(v, 2) for j, v in tilt.items()},
    }


def tilts(c: Concept) -> dict[int, tuple[int, int]]:
    """포인터 끝을 한 점에 두고 30°·45°로 8방향 기울이기: (8방향이 다 되는 칸 수, 6방향 이상 되는 칸 수)."""
    tool = TOOLS["pointer"]
    robot, prims = build(c, tool)
    ranges = (robot.lower, robot.upper)
    h = uc.make_hori(robot, prims, ranges, ranges)
    h.masses = [m for m in h.masses if m.label != "짐"]  # 포인터는 짐이 없다
    out = {}
    for tilt in (30, 45):
        full = six = 0
        for r in RADII:
            for z in HEIGHTS:
                n = sum(
                    bool(
                        good_solutions(
                            h, pk.tool_pose((r, 0.0, z), tilt * DEG, k * 45 * DEG, 0.0, tool.length)
                        )
                    )
                    for k in range(8)
                )
                full += n == 8
                six += n >= 6
        out[tilt] = (full, six)
    return out


def print_zone(c: Concept) -> None:
    z, t = zone(c), tilts(c)
    print("##", describe(c))
    print(
        f"- 주 작업 영역 {z['cells']}칸 중 되는 칸 {z['any']}, 작업 갈래 {z.get('best')} {z.get('branches')}"
    )
    if "torque" in z:
        print(
            f"- 버팀 토크 / 정격 J2 ~ J4 {z['torque'][1:4]}, 가장 펴진 팔꿈치 |sin J3| {z['sin3']}, "
            f"중력이 안 누르는 관절의 백래시 끝 오차 {z['lash_free_mm']} mm"
        )
        print(
            f"- TCP 가속 여유(가장 나쁜 방향) {z['acc_tcp']} m/s², 첫 흔들림 {z['f1_hz']} Hz, "
            f"베어링을 기울이는 모멘트 [N·m] {z['tilt_nm']}"
        )
    print(
        f"- 포인터 기울이기: 30° 8방향 {t[30][0]}칸, 45° 8방향 {t[45][0]}칸 (6방향 이상 {t[45][1]}칸)"
    )


LENGTHS = (
    (160, 220, 180),
    (160, 210, 190),
    (160, 230, 170),
    (160, 240, 180),
    (140, 220, 180),
    (180, 220, 180),
)


def shoulder_room(c: Concept) -> float:
    """어깨 서보의 먼 끝과 돌림판 윗면 사이 [m]. 음수면 서보가 돌림판에 닿아 그 어깨 높이로는 만들 수 없다."""
    return c.shoulder - TURNTABLE_TOP - c.shapes["ST-3120"].far


def print_lengths(c: Concept) -> None:
    print(
        "| 어깨 / 위팔 / 아래팔 [mm] | 작업 갈래 칸 | 버팀 토크 J2 · J3 | 가장 펴진 팔꿈치 | 30° 8방향 | 45° 8방향 |"
    )
    print("|---|---|---|---|---|---|")
    for s, u, f in LENGTHS:
        ci = replace(c, shoulder=s * MM, upper=u * MM, fore=f * MM)
        if shoulder_room(ci) < MOVING_GAP:
            room = shoulder_room(ci) / MM
            print(
                f"| {s} / {u} / {f} | 어깨 서보의 먼 끝이 돌림판에 닿는다 (틈 {room:.1f} mm) | | | | |"
            )
            continue
        z, t = zone(ci), tilts(ci)
        n = z["branches"][z["best"]]
        print(
            f"| {s} / {u} / {f} | {n} | {z['torque'][1]:.2f} · {z['torque'][2]:.2f} | {z['sin3']:.2f} | {t[30][0]} | {t[45][0]} |"
        )


def parse(args: list[str]) -> Concept:
    c = Concept()
    for a in args:
        k, v = a.split("=")
        if k in ("shoulder", "upper", "fore", "upper_r", "fore_r", "column_r", "camera_back"):
            c = replace(c, **{k: float(v) * MM})
        elif k in ("sign4", "cam_side"):
            c = replace(c, **{k: int(v)})
        elif k == "tube_scale":
            c = replace(c, tube_scale=float(v))
        elif k == "extra":
            c = replace(c, extra=float(v) * G)
        elif k == "ranges":
            rng = list(c.ranges_deg)
            for j, part in enumerate(v.split(",")):
                if part:
                    lo, hi = (float(x) for x in part.split(":"))
                    rng[j] = ((lo + hi) / 2, (hi - lo) / 2)
            c = replace(c, ranges_deg=tuple(rng))
        else:
            raise SystemExit(f"모르는 키: {k}")
    return c


def main() -> None:
    args = sys.argv[1:]
    mode = args.pop(0) if args and "=" not in args[0] else "check"
    c = parse(args)
    if mode == "check":
        check(c)
    elif mode == "compare":
        info = check(c, verbose=False)
        s = compare(c)
        print("##", describe(c), f"| J2가 드는 무게 {info['j2_load'] * 1000:.0f} g")
        rs = ", ".join(f"{k} {v}" for k, v in sorted(s["reasons"].items(), key=lambda kv: -kv[1]))
        print(
            f"- UR3e가 되는 {s['ur_ok']}개 중 {s['both']}개 ({s['share'] * 100:.1f}%), 호리봇만 되는 자세 {s['only_hori']}개"
        )
        print(f"- 못 하는 까닭: {rs}")
        for k, d in s["detail"].items():
            top = sorted(d.items(), key=lambda kv: -kv[1])[:5]
            print(f"  - {k}: " + ", ".join(f"{a or '-'} {n}" for a, n in top))
    elif mode == "zone":
        print_zone(c)
    elif mode == "lengths":
        print_lengths(c)
    else:
        raise SystemExit(f"모르는 일: {mode} (check, compare, zone, lengths)")


if __name__ == "__main__":
    main()
