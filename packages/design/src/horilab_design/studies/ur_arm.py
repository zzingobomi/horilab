"""UR식 관절 팔의 검토 모델: 관절 부품(parts/joint.py)의 쌓임과 무게로 팔 전체를 세워 본다 (0029, 0017).

개념 형상(studies/ur_joints.py)과 달리, 축 방향 쌓임은 관절 CAD의 값이고 무게는 뽑아서 잰 부품에 맞춘 어림이다
(printcheck.printed_mass_g). 어깨(parts/shoulder.py)는 서보가 오기 전이라 서보 치수가 공식 도면 값이다.

짜임 (영점 자세, 링크 좌표는 베이스와 같은 방향):
- 자식은 출력 원판 윗면에 앉는다. 링크의 납작한 면과 그 링크 끝 집의 원통 앞 끝이 한 평면이다.
  그래서 위팔, 아래팔, 손목이 관절마다 옆(y)으로 한 칸씩 비킨다.
- 손목 몸통은 서보 먼 끝이 앞 관절 쪽을 본다.
- 베이스 몸통, 돌림판, J1 몸통의 바닥판은 J1 관절 CAD(parts/base.py)의 치수다.
- 관절마다 원통 밖으로 나온 것도 넣는다: 자식 몸통의 귀와 그 위의 나사 머리, 멈춤 기둥과 그 귀, 영점 핀 귀,
  원통 옆의 벽 나사 머리. 집의 목에는 멈춤 기둥이 도는 홈과 가운데 살을 넣어, 관절이 멈춤 각(163°)에서 막힌다.
- 손목 끝은 그린 부품의 자리다: 손목 몸통 옆의 돋움, 기울인 D405와 받침과 플러그(parts/camera_mount.py),
  플랜지 원판의 영점 핀 귀(parts/flange.py), 평행 그리퍼(parts/gripper.py). 포인터는 아직 그리지 않아
  자리표시(packaging.TOOLS)다.
- 선은 무게만 넣는다 (studies/wiring.py: 서보 선과 D405 선을 링크마다 한 덩어리로). 느슨한 고리의 형상은 없다.

    uv run python -m horilab_design.studies.ur_arm               # 쌓임, 옆 자리, 무게, 세운 자세의 범위
    uv run python -m horilab_design.studies.ur_arm compare       # UR3e와 같은 자세 시험 (0027, 5분쯤)
    uv run python -m horilab_design.studies.ur_arm zone          # 주 작업 영역 지표 (0017)
    uv run python -m horilab_design.studies.ur_arm lengths       # 링크 길이 후보의 주 작업 영역 지표
    uv run python -m horilab_design.studies.ur_arm view          # D405가 도구 끝점 둘레를 보는 몫
    uv run python -m horilab_design.studies.ur_arm calib         # 손목 카메라의 캘리브레이션 자세 (0018)

뒤에 키=값을 붙여 바꿔 볼 수 있다 (길이는 mm, 무게는 g):
    shoulder=180 upper=230 fore=170 extra=30 tube_scale=0.62 plug=none wires=0
    ranges=,-160:160,-160:160,20:340,-160:160,-160:160
"""

from __future__ import annotations

import itertools
import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass, replace
from functools import cached_property, partial

import numpy as np

from horilab_design.parts import base as bp
from horilab_design.parts import camera_mount as cmt
from horilab_design.parts import flange as fp
from horilab_design.parts import gripper as gp
from horilab_design.parts import joint as jp
from horilab_design.parts import links as lp
from horilab_design.parts import shoulder as sp
from horilab_design.parts import wrist as wp
from horilab_design.robot import Joint, Robot, Vec3, rotation
from horilab_design.statics import FLANGE, GRAVITY, PointMass, frame_poses, holding_torques
from horilab_design.studies import cameras as cm
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur_compare as uc
from horilab_design.studies import wiring as wr
from horilab_design.studies.common import RATED_MARGIN
from horilab_design.studies.packaging import (
    DEG,
    MM,
    MOVING_GAP,
    Axis,
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
# 원통 속 부품과 나사: 누름 틀 4.1, 서보 받침 3.2, 뒷덮개 5.3, 벽 나사 3.9, 너트 1.5, 서보 뒷면 나사 0.8, 덮개 나사 1.8
INNER_MASS = 21.0 * G
WRIST_BODY_MASS = 26.6 * G  # 손목 몸통 (원통 옆의 돋움 둘 포함)
# 자식 쪽: 출력 원판 11.8, 허브 판 1.3, 혼(가운데 나사 포함, 실측) 2.0, 혼 볼트 넷 5.4, 몸통 나사 넷 3.4, 너트 1.3,
# 멈춤 나사와 너트 1.2
OUT_MASS = 26.4 * G
# 어깨 (uv run python -m horilab_design.parts.shoulder).
# 부모 쪽: 집(J1 몸통의 윗부분) 54.7, 누름 틀 4.2, 서보 받침 9.2, 뒷덮개 12.3, 벽 나사 여섯 4.6, 너트 여섯 1.8,
# 서보 뒷면 나사 여섯 2.9, 덮개 나사 둘 2.0
SHOULDER_HOUSE_MASS = 91.7 * G
# 자식 쪽: 출력 원판 17.6, 허브 판 2.5, 혼 FT110M과 가운데 나사 6.9(도면 부피로 어림), 혼 볼트 여덟 11.5,
# 위팔 나사 여섯 5.3, 너트 여섯 2.1, 멈춤 나사와 너트 1.2
SHOULDER_OUT_MASS = 47.1 * G
# 위팔의 납작한 면(어깨 출력 원판 윗면)이 J1 축에서 떨어진 거리. 어깨 쌓임이 길어진 만큼 서보를 출력 반대쪽으로
# 물려서 이 값을 지킨다 (0029: 골격과 손목 오프셋이 그대로이고, 어깨 원통이 J1 축을 가운데로 거의 대칭이다)
SHOULDER_PLANE = sp.J1_PLANE * MM
# J1 관절 (uv run python -m horilab_design.parts.base). 돌림판 윗면의 높이는 그 쌓임에서 나온다
TABLE_TOP = bp.TOP_H * MM
# J1의 도는 쪽: 돌림판 34.5, 허브 4.2, 사이 고리 11.8, 혼 2.0, 조임 볼트와 와셔·혼 나사·멈춤 나사·J1 몸통 나사와 너트 23.9
J1_TABLE_MASS = 76.4 * G
# J1 몸통(89.8)에서 어깨 집(54.7, SHOULDER_HOUSE_MASS에 들었다)을 뺀 몫: 돌림판까지 내려온 목과 바닥판
J1_FOOT_MASS = 35.1 * G
# 링크 한 부품을 세 덩어리로 나눈 값: 끝 집 쪽 22.8 g(끝 관절 축에서 10 mm 앞 관절 쪽), 관(길이당 무게 × 관 길이.
# 관은 발 상자 끝에서 끝 집의 목(축에서 40 mm)까지다), 발(앞 관절 축). 220 mm 위팔 92.7 g · 부피 중심 114.4 mm,
# 180 mm 아래팔 72.0 g · 87.9 mm와 맞는다. 길이 후보를 견줄 때 관만 길어진다.
LINK_HOUSE_MASS = 22.8 * G
LINK_TUBE: dict[
    str, tuple[float, float, float]
] = {  # (관 kg/m, 발 kg, 부피 중심의 서보 좌표 y [m])
    "upper": (0.320, 22.4 * G, 0.0128),
    "fore": (0.287, 16.7 * G, 0.0133),
}
D405_PLATE_MASS = 12.1 * G  # D405 받침 8.5 (parts/camera_mount.py)과 나사 넷, 너트 둘 3.6
# J6의 자식 쪽 (uv run python -m horilab_design.parts.flange): 플랜지 원판 12.6, 허브 판 1.3, 혼 2.0, 혼 볼트 넷 5.4,
# 너트 둘 0.7, 멈춤 나사와 너트 1.2, 도구 핀 둘 2.2. 손나사 둘과 손잡이는 도구의 무게에 들어 있다
FLANGE_MASS = 25.4 * G
# 평행 그리퍼의 무게 덩어리 (uv run python -m horilab_design.parts.gripper): (무게 kg, 도구 좌표의 무게 중심 mm).
# 도구 좌표는 x가 손가락이 벌어지는 쪽(J6의 멈춤 귀 쪽), H가 플랜지 면에서 도구 쪽, z가 D405의 반대쪽이다
GRIPPER_LUMPS: dict[str, tuple[float, Vec3]] = {
    "몸통, 손나사, 선": (25.8 * G, (-6.9, 12.7, 0.7)),
    "서보와 혼": (58.65 * G, (-12.8, 24.5, 0.0)),
    "랙 집과 속 부품, 나사": (37.0 * G, (1.0, 47.2, 0.0)),
    "손가락": (7.2 * G, (0.0, 71.5, 0.0)),
}
# D405의 옆면에서 선이 굽기 시작하는 곳까지 [m] (가정: parts/camera_mount.py의 PLUG_*). 가진 기본 선의 곧은
# 플러그다 (새 선은 사지 않는다, 0031)
PLUGS = {
    "straight": (cmt.PLUG_LEN + cmt.PLUG_RELIEF) * MM,
    "none": 0.0,
}
PLUG_NAMES = {"straight": "곧은 것", "none": "없음"}
HEAD = wp.HEAD * MM  # 출력 원판 윗면에서 다음 서보 먼 끝까지
# 발 상자의 (반 길이, 반폭, 윗모서리를 깎은 깊이): 위팔은 어깨 출력 원판에, 아래팔은 STS 출력 원판에 앉는다
FEET = {
    key: (f.hx * MM, f.hw * MM, f.chamfer * MM)
    for key, f in (("upper", lp.SHOULDER_FOOT), ("fore", lp.STS_FOOT))
}
NECK_HW = jp.NECK_HW * MM  # STS 집의 목 반폭
SCREW_HEAD = 1.65 * MM  # M3 둥근머리의 높이
EAR_T = jp.EAR_T * MM  # 자식 몸통의 귀 (출력 원판과 같은 원)
PIN_UP = 3.0 * MM  # 영점 핀이 귀 위로 나온 길이
PIN_HW = 4.4 * MM  # 영점 핀 귀의 반폭
STOP_SCREW = (1.8 * MM, 10.0 * MM)  # 멈춤 나사 M3 × 10: 머리 자리의 깊이, 길이
# 도구 좌표 (x, H, z)를 링크 6 좌표 (-z, 플랜지 면 + H, x)로 옮기는 회전
TOOL_ROT = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
LINK_NAMES = ("J1 몸통", "위팔", "아래팔", "손목 1", "손목 2", "손목 3")
DEFAULT_RANGES = (
    (0.0, 160.0),
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
    sign4: int = 1  # J4 출력이 보는 쪽 (+1: 위팔 쪽으로 돌아온다)
    tube_scale: float = 1.0  # 관 무게에 곱하는 값 (벽을 얇게 할 때)
    extra: float = 0.0  # 플랜지에 더 다는 무게 [kg] (형상에 없는 것: 선, 나사)
    # D405의 플러그 (PLUGS): "straight"(가진 곧은 플러그 선), "none"(넣지 않음).
    # 커넥터가 있는 옆면은 J5 쪽을 본다 (0029)
    plug: str = "straight"
    wires: bool = True  # 선의 무게를 넣는다 (0031)
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
        """플랜지 면(J6 출력 원판의 윗면)의 y."""
        return self.mid + self.st("STS3215").atop


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


def draw_gripper(b: Builder, link: int, yf: float, opening: float) -> None:
    """그린 평행 그리퍼(parts/gripper.py)의 겉모양과 무게. opening은 손가락의 열림 폭 [mm]이다.

    치수는 도구 좌표 [mm]다 (GRIPPER_LUMPS). 플랜지에 앉는 판과 서보를 품는 틀, 귀와 돋움, 손나사 손잡이,
    랙 집(바닥판 포함), 손가락 둘을 넣는다. 서보는 틀 속에 있어 형상은 없고 무게만 넣는다.
    """
    f = (np.array([0.0, yf, 0.0]), TOOL_ROT)

    def tbox(name: str, x: tuple[float, float], h: tuple[float, float], hz: float) -> None:
        lo, hi = (x[0] * MM, h[0] * MM, -hz * MM), (x[1] * MM, h[1] * MM, hz * MM)
        b.box(link, name, lo, hi, material="tool", mass=0.0, group="도구", frame=f)

    def tprism(name: str, pts: list[tuple[float, float]], axis: Axis, half: float) -> None:
        scaled = [(u * MM, v * MM) for u, v in pts]
        b.prism(
            link, name, scaled, axis, -half * MM, half * MM, material="tool", group="도구", frame=f
        )

    t = fp.ADAPTER_T
    b.cyl(
        link,
        "adapter",
        (0.0, t / 2 * MM, 0.0),
        Y,
        fp.ADAPTER_R * MM,
        t * MM,
        material="tool",
        mass=0.0,
        group="교체 어댑터",
        frame=f,
    )
    for k, (x, z) in enumerate(fp.TOOL_SCREWS):
        h0, h1 = t + 0.3, t + fp.KNOB[1]
        b.cyl(
            link,
            f"knob{k}",
            (x * MM, (h0 + h1) / 2 * MM, z * MM),
            Y,
            fp.KNOB[0] * MM,
            (h1 - h0) * MM,
            material="tool",
            mass=0.0,
            group="도구",
            frame=f,
        )
    x0, x1 = gp.CRADLE_X[0] - gp.WALL, gp.CRADLE_X[1] + gp.WALL
    top = gp.H_FRONT
    # 틀: 서보 먼 끝 쪽의 밑은 45°로 깎여 있다 (J5 출력 원판의 멈춤 귀 끝 위를 지나간다)
    frame = [(x1, 0.0), (x1, top), (x0, top), (x0, -gp.RELIEF_X - x0), (-gp.RELIEF_X, 0.0)]
    tprism("gripper_frame", frame, "z", gp.CRADLE_Z + gp.WALL)
    # 서보 먼 끝 쪽의 귀와 가까운 끝 쪽의 돋움 (밑은 45°로 받친다)
    lug = [(x0, gp.LUG_H0 - (gp.HOUSE_HL + x0)), (x0, top), (-gp.HOUSE_HL, top)]
    tprism("gripper_lug", [*lug, (-gp.HOUSE_HL, gp.LUG_H0)], "z", gp.LUG_HW)
    boss = [(x1, gp.BOSS_H0 - (gp.BOSS_X1 - x1)), (x1, top), (gp.BOSS_X1, top)]
    tprism("gripper_boss", [*boss, (gp.BOSS_X1, gp.BOSS_H0)], "z", gp.FS[1] + 3.6)
    # 바닥판과 랙 집: 밑의 긴 모서리 둘은 깎여 있다 (단면은 (H, z))
    hw, cut, bottom = gp.HOUSE_HW, gp.TUB_CHAMFER, gp.H_BOTTOM
    house = [(top, -hw), (top, hw), (bottom - cut, hw), (bottom, hw - cut)]
    tprism("gripper_house", [*house, (bottom, cut - hw), (bottom - cut, -hw)], "x", gp.HOUSE_HL)
    xi = opening / 2
    knee = gp.H_ROOT + gp.ROOT_H + (gp.LC - gp.SHANK_U)
    parts = (
        ("root", gp.LC, (bottom, knee), gp.FINGER_HW),
        ("shank", gp.SHANK_U, (knee, gp.H_SHOULDER), gp.FINGER_HW),
        ("tip", gp.BARB_U, (gp.H_SHOULDER, gp.H_TIP), gp.TIP_HW),
    )
    for sg in (1, -1):
        for part, thick, h, hz in parts:
            xa, xb = sorted((sg * xi, sg * (xi + thick)))
            tbox(f"finger{sg:+d}_{part}", (xa, xb), h, hz)
    for i, (mass, (x, h, z)) in enumerate(GRIPPER_LUMPS.values()):
        lo, hi = (
            ((x - 2) * MM, (h - 2) * MM, (z - 2) * MM),
            ((x + 2) * MM, (h + 2) * MM, (z + 2) * MM),
        )
        b.box(link, f"gripper_mass_{i}", lo, hi, material="tool", mass=mass, group="도구", frame=f)


def gripper(opening: float | None = None) -> Tool:
    """그린 평행 그리퍼. 길이는 플랜지 면에서 도구 끝점까지, 박스를 잡으면 박스 윗면이 손가락 어깨에 닿는다.

    opening은 형상에 넣을 손가락의 열림 폭 [mm]이다. 주지 않으면 박스를 잡은 폭이다 (무게와 길이는 같다).
    """
    width = 2 * gp.grasp_x() if opening is None else opening
    return Tool(
        "평행 그리퍼",
        gp.H_TCP * MM,
        sum(m for m, _ in GRIPPER_LUMPS.values()),
        0.15,
        True,
        box_depth=(gp.H_TCP - gp.H_SHOULDER) * MM,
        draw=partial(draw_gripper, opening=width),
    )


GRIPPER = gripper()
# 이 팔에 다는 도구. 그리지 않은 도구는 packaging의 자리표시 그대로다
TOOLS: dict[str, Tool] = {**pk.TOOLS, "gripper": GRIPPER}


def _j6_frame(c: Concept) -> tuple[np.ndarray, np.ndarray]:
    """링크 5 좌표에서 본 J6의 서보 좌표 (parts/joint.py의 좌표: y가 J6 축, -x가 J5 쪽, D405는 -z 쪽)."""
    return pk.Module(6, c.shapes["STS3215"], (0.0, 0.0, c.d5), Y, Z, 1, c.mid).frame()


def camera_frame(c: Concept) -> tuple[np.ndarray, np.ndarray]:
    """링크 5 좌표에서 본 D405의 좌표: 원점이 렌즈 면 가운데, x가 가로, y가 세로 바깥쪽, z가 광축."""
    origin, rot = _j6_frame(c)
    axis, wide = np.array(cmt.AXIS), np.array(X)
    local = np.column_stack([wide, np.cross(axis, wide), axis])
    return origin + rot @ (np.array(cmt.LENS) * MM), rot @ local


def camera(c: Concept) -> pk.Camera:
    """링크 5 좌표에서 본 D405의 렌즈 면 가운데와 광축 (parts/camera_mount.py의 자리)."""
    origin, rot = camera_frame(c)
    return pk.Camera(origin, rot[:, 2])


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
        out_mass: float = OUT_MASS,
        wrist: bool = False,
        neck_hw: float = NECK_HW,
        neck_x: float | None = None,
        child: str = "",
    ) -> None:
        """child는 출력 원판에 앉는 자식: "wrist"(손목 몸통), "upper"·"fore"(링크), ""(플랜지: 도구가 앉는다)."""
        shape = sh[model]
        m = pk.Module(j, shape, origin, axis, near, out, mid)
        s = c.st(model)
        f = m.frame()
        volumes = [(x1 - x0) * (y1 - y0) * (z1 - z0) for x0, x1, y0, y1, z0, z1 in shape.zones]
        mass = servo_mass(model, servos, measured)
        # STS 서보의 외형(packaging.py)은 혼 윗면을 두께 32인 뒷면에서 잰 것으로 읽은 자리다. 관절 CAD는 낮은
        # 뒷면에서 잰 것으로 읽어(docs/hardware.md의 "혼 높이") 몸통이 그보다 뒤에 있다
        dy = 0.0 if model == "ST-3120" else (jp.L1 - pk.STS_BACK) * MM
        for k, (x0, x1, y0, y1, z0, z1) in enumerate(shape.zones):
            b.box(
                j - 1,
                f"J{j}/servo_{k}",
                (x0, y0 + dy, z0),
                (x1, y1 + dy, z1),
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
        cad = sp if model == "ST-3120" else jp
        off = s.atop - cad.ATOP * MM  # 이 모델의 서보 좌표와 CAD 좌표의 차이
        # 목. 앞면에는 멈춤 기둥이 도는 홈이 있다 (원통 벽에서 반지름 CH_R1까지, 깊이는 원통 앞 끝에서 CH_Y까지)
        x0 = -(shape.far + (HEAD if wrist else 3.5 * MM)) if neck_x is None else neck_x
        x1 = -math.sqrt(max(s.radius**2 - neck_hw**2, 0.0))
        y_ch, r_ch = off + cad.CH_Y * MM, cad.CH_R1 * MM
        house = f"J{j} 집"
        b.box(
            j - 1,
            f"J{j}/far_house",
            (x0, s.back, -neck_hw),
            (x1, y_ch, neck_hw),
            mass=0.0,
            frame=f,
            group=house,
        )
        # 홈 바깥에 남는 목 끝: 홈의 둥근 벽을 15°씩 나눈 띠로 넣는다 (띠가 홈 안으로 들어오지 않게 잡는다)
        edge = math.asin(min(1.0, neck_hw / r_ch))
        n = max(1, math.ceil(edge / (15 * DEG)))
        for k in range(n):
            a0, a1 = edge * k / n, edge * (k + 1) / n
            z0, z1, xs = r_ch * math.sin(a0), r_ch * math.sin(a1), -r_ch * math.cos(a0)
            if x0 >= xs:
                continue
            for side, (lo_z, hi_z) in enumerate(((-z1, z1),) if k == 0 else ((z0, z1), (-z1, -z0))):
                b.box(
                    j - 1,
                    f"J{j}/neck_end{k}{'ab'[side]}",
                    (x0, y_ch, lo_z),
                    (xs, s.rim, hi_z),
                    mass=0.0,
                    frame=f,
                    group=house,
                )
        # 홈 가운데의 살: 멈춤 기둥이 여기에 닿아 선다
        half = cad.WEB_HALF * DEG
        ri = s.radius - 1.0 * MM
        web = [(-r * math.cos(a), r * math.sin(a)) for r, a in ((ri, -half), (r_ch, -half))]
        web += [(-r * math.cos(a), r * math.sin(a)) for r, a in ((r_ch, half), (ri, half))]
        b.prism(j - 1, f"J{j}/web", web, "y", y_ch, s.rim, group=house, frame=f)
        if wrist:
            # 원통 옆의 돋움 둘 (parts/wrist.py): 손목 2에서는 D405 받침이 앉는다. 앞쪽은 비스듬히 원통으로 잇는다
            ya, yb = wp.MOUNT_Y
            zf, zr = -wp.MOUNT_Z, -(jp.R_DRUM - 6.6)
            ramp = yb + 1.1 * (zr - zf)
            ramp_pts = [
                (ya * MM, zf * MM),
                (yb * MM, zf * MM),
                (ramp * MM, zr * MM),
                (ya * MM, zr * MM),
            ]
            for k, (x, _y) in enumerate(wp.mount_xy()):
                xa, xb = (x - wp.MOUNT_HW) * MM, (x + wp.MOUNT_HW) * MM
                b.prism(j - 1, f"J{j}/boss{k}", ramp_pts, "x", xa, xb, group=house, frame=f)
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
            (0.0, (s.seam + s.atop) / 2, 0.0),
            Y,
            s.radius,
            s.atop - s.seam,
            mass=out_mass,
            frame=m.child_frame(),
            group=f"J{j} 출력 원판",
        )
        # ---- 원통 밖으로 나온 것 (무게는 위의 덩어리에 들어 있다)
        fc = m.child_frame()
        # 출력 원판에 붙은 것(멈춤 귀와 기둥)의 좌표: 범위의 가운데에서 기둥이 살의 맞은편(서보 가까운 끝 쪽)에
        # 온다. J4는 범위의 가운데가 180°라, 원판을 손목 1 몸통에 반 바퀴 돌려 단다 (0029)
        fs = replace(m, neutral=c.ranges_deg[j - 1][0] * DEG).child_frame()
        group = f"J{j} 귀와 나사"
        # 벽 나사 머리: 원통 옆으로 1 mm 나온다. 나사 줄 둘을 덮는 띠로 넣는다
        y0, y1 = off + (cad.CAR_SCREW_Y - 4.0) * MM, off + (cad.RET_SCREW_Y + 4.0) * MM
        b.cyl(
            j - 1,
            f"J{j}/wallscrews",
            (0.0, (y0 + y1) / 2, 0.0),
            Y,
            s.radius + SCREW_HEAD - 0.6 * MM,
            y1 - y0,
            mass=0.0,
            frame=f,
            group=group,
        )
        # 멈춤 기둥의 귀: 출력 원판 테에서 가까운 끝 쪽(관절 각 0)으로 나온다
        r_post = cad.R_POST * MM
        ear_hw = cad.EAR_HW * MM  # 멈춤 기둥 귀의 반폭
        b.box(
            j,
            f"J{j}/stopear",
            (s.radius - 3.6 * MM, s.seam, -ear_hw),
            (r_post + ear_hw, s.atop, ear_hw),
            mass=0.0,
            frame=fs,
            group=group,
        )
        # 멈춤 기둥: 귀 밑으로 내려와 목의 홈 속을 돈다. 돋움과 너트, 그 밑으로 나온 나사 끝
        nut = off + cad.POST_TIP * MM - jp.NUT_T * MM
        tip = s.atop - sum(STOP_SCREW)
        for name, radius, ya, yb in (
            ("post", jp.BOSS_R * MM, nut, s.seam),
            ("post_tip", 1.5 * MM, tip, nut),
        ):
            b.cyl(
                j,
                f"J{j}/{name}",
                (r_post, (ya + yb) / 2, 0.0),
                Y,
                radius,
                yb - ya,
                mass=0.0,
                frame=fs,
                group=group,
            )
        if not child:
            # 플랜지 원판의 영점 핀 귀 (parts/flange.py). 멈춤 귀의 맞은편이다
            b.box(
                j,
                f"J{j}/pinear",
                (-(cad.R_PIN * MM + PIN_HW), s.seam, -PIN_HW),
                (-(s.radius - 3.6 * MM), s.atop, PIN_HW),
                mass=0.0,
                frame=fs,
                group=group,
            )
            return
        # 자식 몸통의 귀 둘과 그 위의 나사 머리: 출력 원판과 같은 원에서, 자식의 목이나 발 상자의 옆벽 바깥쪽만이다.
        # 가운데는 자식의 목과 발 상자가 따로 들어 있다 (손목 몸통은 그 자리에 다음 관절의 멈춤 홈이 있다)
        side = NECK_HW if child == "wrist" else FEET[child][1]
        a0 = math.asin(side / s.radius)
        arc = [a0 + (math.pi - 2 * a0) * k / 8 for k in range(9)]
        for k, sg in enumerate((1, -1)):
            seg = [(s.radius * math.cos(a), sg * s.radius * math.sin(a)) for a in arc]
            b.prism(
                j,
                f"J{j}/ears{k}",
                seg,
                "y",
                s.atop,
                s.atop + EAR_T + SCREW_HEAD,
                group=group,
                frame=fc,
            )
        # 영점 핀 귀: 손목 몸통은 옆(-z), 링크는 발 끝(-x, 먼 끝 쪽)에 있다. 핀이 귀 위로 3 mm 나온다
        r_pin = cad.R_PIN * MM
        top = s.atop + EAR_T + PIN_UP
        if child == "wrist":
            lo, hi = (-PIN_HW, s.atop, -(r_pin + PIN_HW)), (PIN_HW, top, -(s.radius - 3.0 * MM))
        else:
            lo, hi = (
                (-(r_pin + PIN_HW), s.atop, -PIN_HW),
                (-(FEET[child][0] - 1.0 * MM), top, PIN_HW),
            )
        b.box(j, f"J{j}/pinear", lo, hi, mass=0.0, frame=fc, group=group)

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

    h, a2, a3 = c.shoulder, c.upper, c.fore

    # 베이스 몸통(0)과 돌림판(1). J1 서보와 베어링은 몸통 속이라 충돌 검사에서 뺀다
    base_h = (bp.FRONT - bp.Y_DESK) * MM
    b.cyl(0, "base", (0.0, 0.0, base_h / 2), Z, bp.R_CORE * MM, base_h, mass=0.0, group="베이스")
    # 선이 감기는 띠의 위아래 턱 (0031). 띠에 감긴 선의 두께는 아직 형상에 없다
    lip_h = 1.1 * bp.BAND_LIP * MM
    for name, top in (("base_lip", base_h), ("base_rib", base_h - bp.BAND_H * MM + lip_h)):
        b.cyl(
            0,
            name,
            (0.0, 0.0, top - lip_h),
            Z,
            (bp.R_CORE + bp.BAND_LIP) * MM,
            2 * lip_h,
            mass=0.0,
            group="베이스",
        )
    # 책상에 무는 판의 날개와, 그 위를 무는 클램프 둘. 클램프는 가진 금속 클램프의 위 턱(사진으로 어림:
    # 두께 약 7, 폭 약 10, 가장자리에서 28 mm)을 넉넉히 덮는 상자다
    wing_top = (bp.PLATE_T + bp.RING_H) * MM
    for sg in (1, -1):
        y0, y1 = sorted((sg * bp.WING_IN * MM, sg * bp.WING_Z * MM))
        b.box(
            0,
            f"wing{sg:+d}",
            (-bp.EDGE * MM, y0, 0.0),
            (bp.WING_X * MM, y1, wing_top),
            mass=0.0,
            group="베이스",
        )
        c0, c1 = sorted(
            (sg * (bp.CLAMP_Z - bp.CLAMP_HW) * MM, sg * (bp.CLAMP_Z + bp.CLAMP_HW) * MM)
        )
        b.box(
            0,
            f"clamp{sg:+d}",
            ((-bp.EDGE - 14.0) * MM, c0, wing_top),
            ((-bp.EDGE + bp.CLAMP_IN + 8.0) * MM, c1, wing_top + 0.012),
            mass=0.0,
            group="클램프",
        )
    table0 = (bp.SEAM - bp.Y_DESK) * MM
    b.cyl(
        1,
        "turntable",
        (0.0, 0.0, (table0 + TABLE_TOP) / 2),
        Z,
        (bp.R_TABLE + bp.FLARE) * MM,
        TABLE_TOP - table0,
        mass=J1_TABLE_MASS,
        group="돌림판",
    )
    # J1 몸통: 어깨 집의 목이 돌림판 윗면까지 내려오고, 그 아래 끝의 앞뒤로 둥근 귀가 나온다
    foot_t = jp.EAR_T * MM
    b.cyl(
        1,
        "j1_foot",
        (0.0, 0.0, TABLE_TOP + foot_t / 2),
        Z,
        bp.R_TABLE * MM,
        foot_t,
        mass=J1_FOOT_MASS,
        group="J1 몸통",
    )
    neck_x = -(h - TABLE_TOP)

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
        out_mass=SHOULDER_OUT_MASS,
        neck_hw=sp.NECK_HW * MM,
        neck_x=neck_x,
        child="upper",
    )
    foot(2, "upper", "upper_foot", c.ya2, 1, 2 * c.upper_r - c.embed, "위팔")
    d_tube(2, "upper_tube", c.s2, c.upper_r, FEET["upper"][0], a2 - 0.036, c.ya2)
    link_body(2, "upper", a2, c.c3, -1, "위팔")
    # J3: 위팔 끝, 출력이 -y. 원통 앞 끝이 위팔의 납작한 면과 한 평면
    joint(3, "STS3250", (0.0, 0.0, a2), Y, Z, -1, c.c3, house_mass=INNER_MASS, child="fore")
    foot(3, "fore", "fore_foot", c.yb3, -1, 2 * c.fore_r - c.embed, "아래팔")
    d_tube(3, "fore_tube", c.s3, c.fore_r, FEET["fore"][0], a3 - 0.036, c.yb3)
    link_body(3, "fore", a3, c.c4, c.sign4, "아래팔")
    # J4: 아래팔 끝. J5, J6은 손목 몸통 (서보 먼 끝이 앞 관절 쪽)
    joint(4, "STS3215", (0.0, 0.0, a3), Y, Z, c.sign4, c.c4, house_mass=INNER_MASS, child="wrist")
    wrist_house = WRIST_BODY_MASS + INNER_MASS
    near5 = (0.0, float(c.sign4), 0.0)
    joint(
        5,
        "STS3215",
        (0.0, c.d4, 0.0),
        Z,
        near5,
        1,
        c.mid,
        house_mass=wrist_house,
        wrist=True,
        child="wrist",
    )
    joint(
        6,
        "STS3215",
        (0.0, 0.0, c.d5),
        Y,
        Z,
        1,
        c.mid,
        house_mass=wrist_house,
        out_mass=FLANGE_MASS,
        wrist=True,
    )

    # D405와 받침 (parts/camera_mount.py): 손목 2 몸통의 돋움에 달고, 광축을 도구 축 쪽으로 기울인다
    f6, fcam = _j6_frame(c), camera_frame(c)
    half = cmt.MOUNT_W / 2 * MM
    pr = [(y * MM, z * MM) for y, z in cmt.profile()]
    root = (pr[5][0] + cmt.FILLET * MM, pr[5][1])  # 판 뒷면이 발의 바깥 면과 만나는 곳
    b.prism(
        5, "d405_seat", [pr[0], pr[1], root, pr[6]], "x", -half, half, group="D405 받침", frame=f6
    )
    b.prism(
        5,
        "d405_plate",
        [pr[1], pr[2], pr[3], root],
        "x",
        -half,
        half,
        mass=D405_PLATE_MASS,
        group="D405 받침",
        frame=f6,
    )
    for k, (x, y) in enumerate(wp.mount_xy()):  # 발 나사의 머리
        z = -(wp.MOUNT_Z + cmt.FOOT_T) * MM - SCREW_HEAD / 2
        b.cyl(
            5,
            f"d405_screw{k}",
            (x * MM, y * MM, z),
            Z,
            2.85 * MM,
            SCREW_HEAD,
            mass=0.0,
            group="D405 받침",
            frame=f6,
        )
    cw, ch, cd = cmt.CAM_W / 2 * MM, cmt.CAM_H / 2 * MM, cmt.CAM_D * MM
    b.box(
        5,
        "d405",
        (-cw, -ch, -cd),
        (cw, ch, 0.0),
        material="camera",
        mass=pk.D405_MASS,
        group="D405",
        frame=fcam,
    )
    if PLUGS[c.plug] > 0:
        # 플러그 몸통이 차지하는 자리: 커넥터가 있는 옆면(J5 쪽)에서 선이 굽기 시작하는 곳까지
        zc, pw, pt = -cd + cmt.CONN_BACK * MM, cmt.PLUG_W / 2 * MM, cmt.PLUG_T / 2 * MM
        b.box(
            5,
            "d405_plug",
            (-cw - PLUGS[c.plug], -pw, zc - pt),
            (-cw, pw, zc + pt),
            material="camera",
            mass=0.0,
            group="D405 플러그",
            frame=fcam,
        )
    if c.wires:
        # 선의 무게 (0031): 링크마다 관 가운데와 끝 집, 손목 몸통 둘. J6 → 그리퍼 선은 도구의 무게에 들어 있다
        wire = wr.wire_masses()
        spots: dict[str, tuple[int, Vec3]] = {
            "upper_mid": (2, (0.0, c.s2, a2 / 2)),
            "upper_end": (2, (0.0, c.c3, a2)),
            "fore_mid": (3, (0.0, c.s3, a3 / 2)),
            "fore_end": (3, (0.0, c.c4, a3)),
            "wrist1": (4, (0.0, c.d4 * 0.6, 0.0)),
            "wrist2": (5, (0.0, 0.0, c.d5 * 0.6)),
        }
        for key, (link, (x, y, z)) in spots.items():
            lo, hi = (x - 0.002, y - 0.002, z - 0.002), (x + 0.002, y + 0.002, z + 0.002)
            b.box(link, f"wire_{key}", lo, hi, mass=wire[key], group="선")
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


def hori(c: Concept, tool: Tool = GRIPPER) -> uc.Hori:
    """도구를 달고 그 도구의 짐을 든 팔 (견주기와 지표에 쓴다)."""
    robot, prims = build(c, tool)
    ranges = (robot.lower, robot.upper)
    return uc.make_hori(robot, prims, ranges, ranges, tool)


def describe(c: Concept) -> str:
    rng = " ".join(
        f"J{k + 1}[{mid - half:.0f},{mid + half:.0f}]" for k, (mid, half) in enumerate(c.ranges_deg)
    )
    return (
        f"어깨 {c.shoulder * 1000:.0f} 위팔 {c.upper * 1000:.0f} 아래팔 {c.fore * 1000:.0f} | "
        f"손목 {c.d4 / MM:.1f} · {c.d5 / MM:.1f} · {c.d6 / MM:.1f} | {rng} | "
        f"D405 플러그 {PLUG_NAMES[c.plug]}"
    )


def check(c: Concept, verbose: bool = True) -> dict:
    """쌓임, 옆 자리, 링크 무게와, 세운 자세에서 관절을 하나씩 돌려 부품이 닿기 전까지의 범위."""
    robot, prims = build(c, GRIPPER)
    scene = pk.build_scene(robot, prims)
    q0 = np.array([mid * DEG for mid, _ in c.ranges_deg])
    q0[1] = q0[2] = q0[4] = 0.0
    bad = pk.collisions(pk.hits(scene, q0))
    ranges = pk.joint_ranges(scene, q0)
    masses = link_masses(prims)
    wire_load = sum(p.mass for p in prims if p.group == "선" and p.link >= 2)
    out = {
        "upright_hits": [f"{hit.a} ↔ {hit.b}" for hit in bad],
        "ranges": [
            ((lo - q0[j]) / DEG, (hi - q0[j]) / DEG, wl, wh)
            for j, (lo, hi, wl, wh) in enumerate(ranges)
        ],
        "masses": masses,
        "j2_load": sum(masses[2:]),
        "j2_wires": wire_load,
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
        + f" | J2가 드는 무게 {out['j2_load'] * 1000:.0f} g (선 {wire_load * 1000:.0f} g 포함)"
    )
    return out


NO_SOLUTION_INSIDE = "범위 안의 해 없음"


def outcome(h: uc.Hori, pose: np.ndarray) -> tuple[str, str]:
    """호리봇이 한 자세를 하는지: ("됨", "") 또는 (까닭, 자세한 것).

    IK의 해마다 관절 범위 안의 같은 자세(2π 차이, 대칭 도구의 J6 ± 180°)로 옮긴 각에서 부딪힘과 버팀 토크를 본다.
    까닭은 이렇게 가른다:
    - "관절 범위": 범위 안의 해는 없거나 부딪히는데, 범위 밖의 해로는 부딪히지 않는다. 그 해가 넘는 관절과,
      범위 안의 해를 막은 것을 "J4: 책상 ↔ J4 귀와 나사"처럼 적는다 (범위 안의 해가 없으면 "J4: 범위 안의 해 없음").
      범위 밖의 해에서 자기 관절의 멈춤에 닿는 것은 부딪힘으로 세지 않는다 (멈춤이 곧 범위다).
    - "부딪힘": 어느 해로도 부딪힌다. 가장 덜 겹치는 해의 가장 깊은 쌍을 적는다.
    - "토크": 부딪히지 않는 해는 있지만 버팀 토크가 정격의 0.8배를 넘는다.
    """
    sols = list(h.ik.solve(pose))
    if not sols:
        return "못 닿음", ""
    lower, upper = h.robot.lower, h.robot.upper
    free: list[np.ndarray] = []  # 범위 안에서 부딪히지 않는 해
    beyond: list[np.ndarray] = []  # 범위 밖에서 부딪히지 않는 해
    blocked: list[pk.Hit] = []  # 해마다 가장 깊이 겹친 쌍
    blocked_inside: list[pk.Hit] = []  # 그 가운데 범위 안의 해의 것
    for raw in sols:
        q = fit_ranges(raw, lower, upper)
        bad = pk.collisions(pk.hits(h.scene, raw if q is None else q))
        if q is None:
            bad = [hit for hit in bad if not uc.same_joint(hit)]
        if bad:
            blocked.append(min(bad, key=lambda hit: hit.dist))
            if q is not None:
                blocked_inside.append(blocked[-1])
        elif q is None:
            beyond.append(raw)
        else:
            free.append(q)
    if free:
        best = min((h.torque(q) for q in free), key=lambda t: float(t[1:].max()))
        if best[1:].max() <= RATED_MARGIN:
            return "됨", ""
        return "토크", f"J{int(np.argmax(best[1:])) + 2}"
    if beyond:
        out = min((~uc.fits(q, lower, upper) for q in beyond), key=lambda miss: int(miss.sum()))
        joints = "·".join(f"J{j + 1}" for j in range(6) if out[j])
        if not blocked_inside:
            return "관절 범위", f"{joints}: {NO_SOLUTION_INSIDE}"
        worst = max(blocked_inside, key=lambda hit: hit.dist)
        return "관절 범위", f"{joints}: {worst.a} ↔ {worst.b}"
    worst = max(blocked, key=lambda hit: hit.dist)
    return "부딪힘", f"{worst.a} ↔ {worst.b}"


def compare(c: Concept, every: int = 1) -> dict:
    """UR3e와 같은 자세 시험 (0027). UR3e가 되는 자세 중 호리봇도 되는 몫과, 못 하는 까닭."""
    cmp = uc.compare(hori=hori(c), every=every, label=describe(c), tool=GRIPPER, outcome=outcome)
    res = cmp.results
    ur_ok = [r for r in res if r.ur[0] == "됨"]
    both = [r for r in ur_ok if r.hori[0] == "됨"]
    only_hori = [r for r in res if r.ur[0] != "됨" and r.hori[0] == "됨"]
    reasons: dict[str, int] = {}
    detail: dict[str, dict[str, int]] = {}
    inside: dict[str, int] = {}  # 관절 범위로 못 하는 자세에서, 범위 안의 해를 막은 것
    for r in ur_ok:
        if r.hori[0] != "됨":
            why, what = r.hori
            what, _, block = what.partition(": ")
            reasons[why] = reasons.get(why, 0) + 1
            d = detail.setdefault(why, {})
            d[what] = d.get(what, 0) + 1
            if block:
                inside[block] = inside.get(block, 0) + 1
    return {
        "n": len(res),
        "ur_ok": len(ur_ok),
        "both": len(both),
        "share": len(both) / max(len(ur_ok), 1),
        "only_hori": len(only_hori),
        "reasons": reasons,
        "detail": detail,
        "inside": inside,
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
    tool = GRIPPER
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
    h = hori(c, tool)  # 포인터는 짐이 없다
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
    """어깨 서보의 먼 끝과 J1 몸통의 바닥판 윗면 사이 [m]. 음수면 서보가 바닥판에 닿아 그 어깨 높이로는 만들 수 없다."""
    return c.shoulder - TABLE_TOP - bp.FLOOR_T * MM - c.shapes["ST-3120"].far


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
                f"| {s} / {u} / {f} | 어깨 서보의 먼 끝이 J1 몸통의 바닥판에 닿는다 (틈 {room:.1f} mm) | | | | |"
            )
            continue
        z, t = zone(ci), tilts(ci)
        n = z["branches"][z["best"]]
        print(
            f"| {s} / {u} / {f} | {n} | {z['torque'][1]:.2f} · {z['torque'][2]:.2f} | {z['sin3']:.2f} | {t[30][0]} | {t[45][0]} |"
        )


VIEW_TURNS = (
    0,
    45,
    90,
)  # D405 화면을 볼 J6 각 [°]. 0°에서 손가락이 벌어지는 방향이 카메라 쪽과 직각이다
VIEW_AHEAD = gp.VIEW_AHEAD * MM  # 잡기 전 높이 (0017)


def views(
    c: Concept, tool: Tool = GRIPPER, turns: Sequence[int] = VIEW_TURNS
) -> dict[int, tuple[float, float, float, float]]:
    """D405가 도구 끝점 둘레(±30 mm)를 보는 몫. 앞쪽 20 cm, 높이 10 cm에서 도구가 아래를 볼 때다.

    J6 각마다 (잡기 전 높이에서 보는 몫, 끝점 높이에서 보는 몫, 끝점이 광축에서 벗어난 각 [rad], 렌즈에서 끝점까지
    [m]). 평행 그리퍼의 실제 형상으로 센 값은 parts/gripper.py의 view_share에 있다 (J6 0°).
    """
    h = hori(c, tool)
    pose = pk.tool_pose((0.20, 0.0, 0.10), 0.0, 0.0, 0.0, tool.length)
    q0 = next(q for q in good_solutions(h, pose) if _branch(q) == "앞위+")
    cam = camera(c)
    out = {}
    for turn in turns:
        q = q0.copy()
        q[5] = turn * DEG
        ahead, _, _ = pk.camera_view(h.scene, q, tool, cam, ahead=VIEW_AHEAD)
        near, off, dist = pk.camera_view(h.scene, q, tool, cam)
        out[turn] = (ahead, near, off, dist)
    return out


def print_views(c: Concept) -> None:
    print("##", describe(c))
    lens, axis = camera(c).lens, camera(c).axis
    print(
        f"- D405 렌즈 면 가운데 (링크 5) [mm]: ({lens[0] / MM:.1f}, {lens[1] / MM:.1f}, {lens[2] / MM:.1f}), "
        f"플랜지 면보다 {(c.d6 - lens[1]) / MM:.1f} 뒤, 광축은 도구 축 쪽으로 "
        f"{math.degrees(math.atan2(-axis[0], axis[1])):.0f}°"
    )
    print(
        f"- 도구 끝점 둘레를 보는 몫: 잡기 전 높이(끝점보다 {VIEW_AHEAD / MM:.0f} mm 앞) / 끝점 높이. "
        "J6 0°에서 손가락이 벌어지는 방향이 카메라 쪽과 직각이다"
    )
    rows = (
        ("평행 그리퍼 (박스 구멍에 넣는 폭)", GRIPPER, VIEW_TURNS),
        ("평행 그리퍼 (다 벌림)", gripper(gp.OPEN_MAX), VIEW_TURNS),
        ("포인터 (자리표시)", TOOLS["pointer"], (0,)),
        ("흡착 (자리표시)", TOOLS["suction"], (0,)),
    )
    for label, tool, turns in rows:
        seen = views(c, tool, turns)
        _, _, off, dist = seen[0]
        print(
            f"  - {label}: "
            + ", ".join(
                f"J6 {turn}° {ahead * 100:.0f}% / {near * 100:.0f}%"
                for turn, (ahead, near, _o, _d) in seen.items()
            )
            + f" | 끝점이 광축에서 {off / DEG:.0f}°, 거리 {dist * 100:.1f} cm"
        )


# 손목 카메라의 캘리브레이션 자세를 찾을 때의 시작 각 [°] (J1 ~ J6). J5는 양쪽을 다 본다. J6는 시작 각 그대로
# 쓴다: 0°에서 그리퍼의 좁은 쪽이 카메라를 보고(손가락이 벌어지는 방향이 카메라 쪽과 직각), 90°는 견줄 값이다
CALIB_SEEDS = [
    np.array(s, float) * DEG
    for s in itertools.product(
        (-30, 0, 30), (-20, 20, 50), (40, 80, 115), (140, 190), (60, 110, -60, -110), (0, 90)
    )
]
# 캘리브레이션 자세를 볼 도구: 포인터, 손가락을 다 벌린 평행 그리퍼
CALIB_TOOLS = (("포인터", TOOLS["pointer"]), ("평행 그리퍼 (다 벌림)", gripper(gp.OPEN_MAX)))


def calibration(c: Concept, tool: Tool) -> cm.WristResult:
    """손목 카메라의 캘리브레이션 자세 (0018): 책상 앞쪽 가운데의 보드를 34방향에서 볼 때 되는 방향."""
    robot, prims = build(c, tool)
    return cm.wrist_calibration(robot, prims, camera(c), seeds=CALIB_SEEDS)


def print_calibration(c: Concept) -> None:
    print("##", describe(c))
    for name, tool in CALIB_TOOLS:
        r = calibration(c, tool)
        total = len(r.ok) + len(r.out_of_range) + len(r.collided) + len(r.hidden)
        print(
            f"- {name}: {total}방향 중 {len(r.ok)}방향이 된다 (관절 범위 밖 {len(r.out_of_range)}, "
            f"부딪힘 {len(r.collided)}, 가림 {len(r.hidden)})"
        )
        for why, labels in (
            ("관절 범위 밖", r.out_of_range),
            ("부딪힘", r.collided),
            ("가림", r.hidden),
        ):
            if labels:
                print(f"  - {why}: " + "; ".join(labels))


def parse(args: list[str]) -> Concept:
    c = Concept()
    for a in args:
        k, v = a.split("=")
        if k in ("shoulder", "upper", "fore", "upper_r", "fore_r"):
            c = replace(c, **{k: float(v) * MM})
        elif k == "sign4":
            c = replace(c, sign4=int(v))
        elif k == "plug":
            if v not in PLUGS:
                raise SystemExit(f"모르는 플러그: {v} ({', '.join(PLUGS)})")
            c = replace(c, plug=v)
        elif k == "wires":
            c = replace(c, wires=bool(int(v)))
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
        if s["inside"]:
            top = sorted(s["inside"].items(), key=lambda kv: -kv[1])[:6]
            print(
                "  - 관절 범위로 못 하는 자세에서 범위 안의 해를 막는 것: "
                + ", ".join(f"{a} {n}" for a, n in top)
            )
    elif mode == "zone":
        print_zone(c)
    elif mode == "lengths":
        print_lengths(c)
    elif mode == "view":
        print_views(c)
    elif mode == "calib":
        print_calibration(c)
    else:
        raise SystemExit(f"모르는 일: {mode} (check, compare, zone, lengths, view, calib)")


if __name__ == "__main__":
    main()
