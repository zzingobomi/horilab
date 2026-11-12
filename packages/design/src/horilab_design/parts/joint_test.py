"""옛 관절 하나 시험의 시험대와 출력 목록, 검사 (0028에서 시험을 없애 다시 설계하며 지운다).

    uv run python -m horilab_design.parts.joint_test          # 검사 결과를 보이고 build/joint_test/에 STL
    uv run python -m horilab_design.parts.joint_test --check  # 검사만

팔꿈치 부품(elbow.py)에 시험용 부품을 더한다: 위팔 관 대신 받침 기둥, 아래팔 대신 시험 링크,
다이얼 게이지 받침, 프레임만의 휨을 잴 서보 모형. 받침 기둥과 게이지 받침은 책상 가장자리에 C 클램프로 문다.
좌표는 elbow.py와 같은 서보 좌표계(mm)이고, 시험 자세는 J3 = 90°(시험 링크가 수평, 서보 x가 위)다.
"""

from __future__ import annotations

import argparse
import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from build123d import Compound, Location, Plane, Polygon, Pos, Rot, export_stl, extrude

from horilab_design.parts import elbow as e
from horilab_design.parts.shapes import box, cyl_x, cyl_y, nut_trap, solid, teardrop
from horilab_design.values import REPO_ROOT, load_servos

OUT_DIR = REPO_ROOT / "build" / "joint_test"
TEST_ANGLE = 90.0  # [deg] 시험 링크가 수평인 자세
PLA_DENSITY = 1.26e-3  # [g/mm³]
STRENGTH_WALL = 2.6  # 0.20mm Strength 프리셋의 벽 6겹 (docs/manufacturing.md)
STRENGTH_INFILL = 0.25

# --- 시험대 배치 [mm] ---------------------------------------------------------------------------
AXIS_H = 140.0  # 책상 위 J3 축 높이. 시험 링크 아래로 짐(500 g 안팎)이 매달릴 높이
X_DESK = -AXIS_H
EDGE_Y = -40.0  # 책상 가장자리. 책상은 +y 쪽이고, 시험 링크는 가장자리를 따라 -z로 뻗는다
BASE_T = 6.0
BASE_Z = (-60.0, 60.0)
COLLAR_OD, COLLAR_L = e.TUBE_OD, 22.0  # 받침 기둥 위끝은 위팔 관과 같은 바깥지름의 갈라진 고리
COLLAR_SLIT = 1.6  # 고리를 가르는 틈. 죄면 둘레가 이만큼까지 줄어든다
POST_W = 50.0
LINK_LENGTH = 215.0  # J3 축에서 시험 링크 끝까지
LINK_DEPTH, LINK_WIDTH = 30.0, 24.0
HOOK_R = LINK_LENGTH - 5.0  # 짐을 거는 구멍 (J3 축에서)
RING_HEAD_H = 3.0  # 고리 나사 머리가 고리 위로 솟는 높이 (원통머리 3.0과 둥근머리 1.65 중 큰 것)
LINK_TOP = LINK_DEPTH / 2  # 시험 자세에서 링크 윗면의 x
# --- 게이지 받침 ---------------------------------------------------------------------------------
# 다이얼 게이지 Mitutoyo 2046A (미츠토요 자료): 줄기 지름 8 mm(h6), 길이 16.9 mm, 바늘을 누르지 않았을 때 줄기 끝에서
# 접촉점까지 19.8 mm, 바늘 움직임 10 mm, 미는 힘 0.4~1.4 N, 눈금판 지름 57 mm, 무게 144 g.
# 받침은 서보 좌표로 그리고 세로 줄기 축이 z = 0에 온다 (시험대에서는 z = -GAUGE_R로 옮긴다).
# - 기둥은 링크 뒤(책상 안쪽, +y)에 선다. 발은 링크 밑을 지나 책상 가장자리까지 나와서, 클램프가 발 앞쪽을 막힘
#   없이 문다. 기둥이 발 앞쪽을 막으면 가진 클램프(나사가 등에서 약 2 cm)로 물 자리가 없다.
# - 기둥과 발은 짐(물병) 쪽(-z)이 아니라 관절 쪽(+z)으로 비켜서 매단 물병과 떨어진다. 위 팔이 링크 위로 와서
#   세로 집게가 링크 윗면 가운데(축에서 GAUGE_R)를 잰다. 옆 집게는 기둥에서 게이지를 가로로 잡아 링크 뒷면을
#   잰다 (흔들림).
# - 집게는 줄기 구멍에 M3 × 10 나사를 바로 돌려 넣어(자가탭) 나사 끝으로 줄기를 누른다. 갈라진 고리는 PLA가 틈만큼
#   휘어야 해서 금이 갈 수 있고, 너트를 넣는 일도 는다.
# - 출력은 관절 쪽 옆면(z = GAUGE_BED_Z)을 바닥에 두고 눕힌다. 모든 부분이 그 면에서 -z로 곧게 올라오는 기둥꼴이라
#   공중에서 시작하는 층이 없다. 두 줄기 구멍은 출력할 때 옆으로 누우므로 위(-z)로 45° 지붕을 낸다.
GAUGE_STEM_D, GAUGE_STEM_L, GAUGE_TIP = 8.0, 16.9, 19.8
GAUGE_STEM_HOLE = 8.4  # 가로 구멍이라 작게 뽑히는 몫(0.2)과 미끄러질 여유
GAUGE_ABOVE = 22.0  # 링크 윗면 → 세로 집게 가운데. 줄기를 위아래로 밀어 바늘이 5 mm쯤 눌리게 맞춘다
GAUGE_CLAMP_H = 16.0  # 세로 집게 높이 (x)
GAUGE_CLAMP_FRONT = -12.5  # 세로 집게 앞면 (y). 누름 나사가 들어오는 면
GAUGE_CLAMP_BACK = 9.0  # 세로 집게 뒷면 (y). 줄기 구멍 뒤 벽 4.8 mm
GAUGE_BED_Z = 38.0  # 출력 바닥면 = 받침의 관절 쪽 옆면
GAUGE_ARM_Z0 = -9.0  # 위 팔(세로 집게 몸통)의 짐 쪽 면
GAUGE_COL_Z = (22.0, GAUGE_BED_Z)  # 기둥과 옆 집게 (z). 가운데가 옆 줄기 축
GAUGE_COL_Y = (20.0, 34.0)  # 기둥 (y). 링크 뒷면(y = 12)에서 8 mm
GAUGE_SIDE_HX = 11.0  # 옆 집게 반높이 (x). 누름 나사는 윗면(+x)에서 들어온다
# 옆 집게 뒷면. 게이지 목(줄기 위 굵은 부분)이 여기 닿을 때까지 줄기를 넣으면 바늘이 링크 뒷면을 약 2 mm 누른다
GAUGE_SIDE_Y1 = 46.5
GAUGE_FOOT_T = 8.0
GAUGE_FOOT_Y = (EDGE_Y, 44.0)  # 발: 책상 가장자리에서 기둥 뒤까지
GAUGE_FOOT_Z = (-5.0, GAUGE_BED_Z)  # 발 폭 43 mm
GAUGE_SET_SCREW = 10.0  # 누름 나사 M3 × 10 (두 집게)
GAUGE_SET_TAP = 3.0  # 누름 나사 자가탭 구멍. 출력할 때 가로 구멍이라 세로 값(2.8)에 0.2를 더한다
GAUGE_CLAMP_FOOT_PAD = (
    EDGE_Y + 20.0,
    (GAUGE_FOOT_Z[0] + GAUGE_FOOT_Z[1]) / 2,
)  # 클램프 패드 (y, z)


def _nut(
    center: tuple[float, float, float],
    axis: tuple[float, float, float],
    slide: tuple[float, float, float],
    length: float,
) -> Compound:
    return nut_trap(center, axis, slide, length, e.FITS.nut_width, e.FITS.nut_height)


def test_link() -> Compound:
    """시험 링크: 두 원판의 혀 사이에 끼는 뿌리 + 곧은 막대. J3 = 0일 때 +x로 뻗는다.

    출력 방향: 아랫면(-z)을 바닥에 눕힌다. 휨은 층을 따라 걸린다. 속은 슬라이서 채움(25%)에 맡긴다.
    """
    y0, y1 = e.Y_DISC_BACK, e.Y_DISC_OUT
    root = box(e.R_CYL + e.MOVING_GAP, e.TONGUE_R, y0, y1, -LINK_DEPTH / 2, LINK_DEPTH / 2)
    beam = box(
        e.TONGUE_R - 1,
        LINK_LENGTH,
        -LINK_WIDTH / 2,
        LINK_WIDTH / 2,
        -LINK_DEPTH / 2,
        LINK_DEPTH / 2,
    )
    link = root + beam
    for x, z in e.TONGUE_BOLTS:  # 혀에서 들어오는 M3 × 12, 너트는 위아래(z) 면에서 밀어 넣는다
        sz = math.copysign(1, z)
        for face, inward in ((y0, 1.0), (y1, -1.0)):
            link -= cyl_y(
                e.FITS.m3_hole / 2,
                min(face, face + inward * 9.5),
                max(face, face + inward * 9.5),
                x,
                z,
            )
            nut_y = face + inward * (5.0 + e.FITS.nut_height / 2)
            link -= _nut((x, nut_y, z), (0, 1, 0), (0, 0, sz), LINK_DEPTH / 2 - abs(z) + 1)
    # 고리 나사 머리(고리 맨 위) 위를 지나가는 홈. 머리와 움직이는 틈(2 mm)을 둔다.
    ys = e.RING_SCREWS_Y
    link -= box(
        e.R_CYL + e.MOVING_GAP - 1.0,
        e.R_CYL + RING_HEAD_H + e.MOVING_GAP,
        min(ys) - 4.5,
        max(ys) + 4.5,
        -LINK_DEPTH / 2 - 1.0,
        LINK_DEPTH / 2 + 1.0,
    )
    link -= cyl_y(2.5, -60.0, 60.0, HOOK_R, 0.0)
    return solid(link)


def stand() -> Compound:
    """받침 기둥: 책상 가장자리에 무는 바닥판, 사각 기둥, 집의 꽂이를 무는 갈라진 고리 (위팔 관 위끝과 같다).

    출력 방향: 바닥판을 바닥에 둔다. 속은 슬라이서 채움에 맡긴다. 케이블 A는 꽂이 속에서 기둥 가운데
    구멍으로 내려가 바닥판 밑 홈으로 나온다.
    """
    x_top = e.X_SHOULDER
    x_base = X_DESK + BASE_T
    x_collar = x_top - COLLAR_L
    s = box(X_DESK, x_base, EDGE_Y, 30.0, *BASE_Z)
    s += box(x_base, x_collar, -POST_W / 2, POST_W / 2, -POST_W / 2, POST_W / 2)
    s += cyl_x(COLLAR_OD / 2, x_collar, x_top)
    # 조임 귀 (+z 쪽): 아래는 45° 쐐기로 받쳐 서포트 없이 뽑힌다
    ear_z0, ear_z1 = COLLAR_OD / 2 - 3, COLLAR_OD / 2 + 8
    ear_x0 = x_top - 14
    s += box(ear_x0, x_top, -6.0, 6.0, ear_z0, ear_z1)
    s += box(ear_x0 - (ear_z1 - ear_z0), ear_x0, -6.0, 6.0, ear_z0, ear_z1) & wedge_below(
        ear_x0, ear_z0, ear_z1
    )
    s -= cyl_x(e.TUBE_ID / 2, x_top - e.SPIGOT_L - 1, x_top + 1)  # 꽂이 자리
    slit = COLLAR_SLIT / 2  # 가르는 틈
    s -= box(x_top - e.SPIGOT_L - 4, x_top + 1, -slit, slit, 0.0, ear_z1 + 1)
    s -= cyl_y(e.FITS.m3_hole_horizontal / 2, -7.0, 7.0, x_top - 7, COLLAR_OD / 2 + 3.5)
    s -= _nut((x_top - 7, -3.6, COLLAR_OD / 2 + 3.5), (0, 1, 0), (0, 0, 1), 6.0)
    s -= cyl_x(10.0, X_DESK - 1, x_top - e.SPIGOT_L)  # 케이블 길
    s -= box(X_DESK - 1, X_DESK + 4.0, -6.0, 6.0, 0.0, 61.0)  # 바닥판 밑 홈 (+z 끝으로)
    return solid(s)


def wedge_below(x_top: float, z0: float, z1: float) -> Compound:
    """조임 귀 밑을 받치는 45° 쐐기 (x_top 아래, z0 쪽이 높다)."""
    h = z1 - z0
    tri = Polygon((x_top, z0), (x_top, z1), (x_top - h, z0), align=None)  # xz 평면의 삼각형
    return extrude(tri, amount=12.0).moved(
        Location(Plane(origin=(0, 6.0, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)))
    )


GAUGE_CLAMP_X = LINK_TOP + GAUGE_ABOVE  # 세로 집게 가운데 (x)
GAUGE_SIDE_Z = (
    GAUGE_COL_Z[0] + GAUGE_COL_Z[1]
) / 2  # 옆 줄기 축 (z). 시험대에서 축으로부터 GAUGE_R - 이 값
GAUGE_SIDE_SCREW_Y = (GAUGE_COL_Y[1] + GAUGE_SIDE_Y1) / 2  # 옆 집게 누름 나사 자리 (y)


def gauge_bracket() -> Compound:
    """다이얼 게이지 받침 (위 '게이지 받침' 설명). 세로 줄기 축이 z = 0으로 링크 윗면 가운데 위에 온다.

    발, 링크 뒤 기둥, 링크 위로 오는 팔과 세로 집게, 기둥 뒤로 이어진 옆 집게. 모두 z = GAUGE_BED_Z 면에서 -z로
    곧게 이어지는 기둥꼴이다.
    """
    roof = (0.0, 0.0, -1.0)  # 출력할 때 위
    x_foot = X_DESK + GAUGE_FOOT_T
    x0, x1 = GAUGE_CLAMP_X - GAUGE_CLAMP_H / 2, GAUGE_CLAMP_X + GAUGE_CLAMP_H / 2
    hx = GAUGE_SIDE_HX
    g = box(X_DESK, x_foot, *GAUGE_FOOT_Y, *GAUGE_FOOT_Z)  # 발 (책상 가장자리까지)
    g += box(x_foot, x1, *GAUGE_COL_Y, *GAUGE_COL_Z)  # 기둥 (링크 뒤)
    # 세로 집게는 바닥면(z = GAUGE_BED_Z)까지 이어지고, 위 팔은 기둥 높이(z)로만 집게와 기둥을 잇는다
    g += box(x0, x1, GAUGE_CLAMP_FRONT, GAUGE_CLAMP_BACK, GAUGE_ARM_Z0, GAUGE_BED_Z)
    g += box(x0, x1, GAUGE_CLAMP_BACK, GAUGE_COL_Y[1], *GAUGE_COL_Z)
    g += box(-hx, hx, GAUGE_COL_Y[0], GAUGE_SIDE_Y1, *GAUGE_COL_Z)  # 옆 집게 (기둥 뒤로 이어진다)
    # 세로 줄기 구멍과 누름 나사 (앞면에서 +y로)
    g -= teardrop((x0 - 1.0, 0.0, 0.0), (1.0, 0.0, 0.0), GAUGE_CLAMP_H + 2.0, GAUGE_STEM_HOLE, roof)
    g -= cyl_y(GAUGE_SET_TAP / 2, GAUGE_CLAMP_FRONT - 1.0, 0.0, GAUGE_CLAMP_X, 0.0)
    # 옆 줄기 구멍 (y, 기둥을 지나 링크 뒷면 쪽으로)과 누름 나사 (윗면에서 -x로)
    side_len = GAUGE_SIDE_Y1 - GAUGE_COL_Y[0] + 2.0
    g -= teardrop(
        (0.0, GAUGE_COL_Y[0] - 1.0, GAUGE_SIDE_Z), (0.0, 1.0, 0.0), side_len, GAUGE_STEM_HOLE, roof
    )
    g -= cyl_x(GAUGE_SET_TAP / 2, 0.0, hx + 1.0, y=GAUGE_SIDE_SCREW_Y, z=GAUGE_SIDE_Z)
    return solid(g)


def dummy() -> Compound:
    """서보 모형: 서보와 같은 외형에 두 혼의 바깥 면까지 찬 덩어리. 원판이 바로 나사로 붙어 관절이 잠긴다.

    프레임(집, 원판, 링크, 받침)만의 휨을 잰다. 채움 100%로 뽑는다. 출력 방향: 옆면(z)을 바닥에 눕힌다.
    """
    s = box(-e.FAR, e.NEAR, e.BACK, e.FRONT, -e.HW, e.HW)
    s += box(-e.FAR, e.PAD_X, e.PAD_Y, e.BACK, -e.HW, e.HW)
    s += cyl_y(e.HORN_R, e.FRONT, e.HORN_TOP)
    s += cyl_y(e.HORN_R, e.BACK_HORN[0], e.BACK)
    s += box(-e.HORN_R, e.HORN_R, e.BACK_HORN[0], e.BACK, -e.HW, 0.0)  # 바닥까지 내려 서포트 없이
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        x, z = e.HORN_BOLT_R * math.cos(a), e.HORN_BOLT_R * math.sin(a)
        s -= cyl_y(e.FITS.m3_tap / 2, e.HORN_TOP - 8, e.HORN_TOP + 1, x, z)
        s -= cyl_y(e.FITS.m3_tap / 2, e.BACK_HORN[0] - 1, e.BACK_HORN[0] + 8, x, z)
    for x, z in e.FRONT_HOLES:  # 집의 M2 나사가 모형에 바로 조여 들어간다
        s -= cyl_y(0.8, e.FRONT - 6, e.FRONT + 1, x, z)
    return solid(s)


# ------------------------------------------------------------------------------------------------
# 출력 목록


@dataclass(frozen=True)
class PartSpec:
    name: str  # 파일 이름 (영어)
    label: str  # 사람이 보는 이름
    build: Callable[[], Compound]
    color: str  # "black" 구조색, "white" 덮개색, "any" 시험용
    child: bool  # 자식 링크(관절과 같이 돈다)
    print_plane: Plane  # 이 평면이 출력 바닥이 되게 눕힌다 (평면의 z가 위)
    count: int = 1
    preset: str = "0.20mm Strength"
    note: str = ""


def _plane(
    origin: tuple[float, float, float],
    z_dir: tuple[float, float, float],
    x_dir: tuple[float, float, float],
) -> Plane:
    return Plane(origin=origin, x_dir=x_dir, z_dir=z_dir)


PARTS: tuple[PartSpec, ...] = (
    PartSpec(
        "elbow_house",
        "서보 집",
        e.house,
        "black",
        False,
        _plane((0, 0, e.X_SHOULDER - e.SPIGOT_L), (1, 0, 0), (0, 1, 0)),
    ),
    PartSpec(
        "elbow_ring",
        "가운데 고리",
        e.ring,
        "white",
        False,
        _plane((0, e.Y_DISC_BACK + e.MOVING_GAP, 0), (0, 1, 0), (1, 0, 0)),
        preset="0.20mm Standard",
    ),
    PartSpec(
        "elbow_disc_out",
        "출력 쪽 원판",
        lambda: e.disc("out"),
        "black",
        True,
        _plane((0, e.Y_DISC_OUT + e.DISC_T, 0), (0, -1, 0), (1, 0, 0)),
    ),
    PartSpec(
        "elbow_disc_back",
        "뒷면 쪽 원판",
        lambda: e.disc("back"),
        "black",
        True,
        _plane((0, e.Y_DISC_BACK - e.DISC_T, 0), (0, 1, 0), (1, 0, 0)),
    ),
    PartSpec(
        "test_link",
        "시험 링크",
        test_link,
        "any",
        True,
        _plane((0, 0, -LINK_DEPTH / 2), (0, 0, 1), (1, 0, 0)),
    ),
    PartSpec(
        "test_stand",
        "받침 기둥",
        stand,
        "any",
        False,
        _plane((0, 0, 0), (1, 0, 0), (0, 1, 0)),
        note="바닥판이 바닥",
    ),
    PartSpec(
        "gauge_bracket",
        "게이지 받침",
        gauge_bracket,
        "any",
        False,
        _plane((0, 0, GAUGE_BED_Z), (0, 0, -1), (1, 0, 0)),
        note="관절 쪽 옆면이 바닥",
    ),
    PartSpec(
        "servo_dummy",
        "서보 모형",
        dummy,
        "any",
        False,
        _plane((0, 0, -e.HW), (0, 0, 1), (1, 0, 0)),
        note="채움 100%",
    ),
)


def placed_for_print(spec: PartSpec) -> Compound:
    """출력 방향으로 눕히고 바닥(z = 0)에 내려놓는다."""
    part = spec.build()
    to_plane = Location(spec.print_plane).inverse()
    laid = part.moved(to_plane)
    bb = laid.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * laid


# ------------------------------------------------------------------------------------------------
# 볼트와 나사 (시험 하나에 드는 것)


@dataclass(frozen=True)
class Fastener:
    item: str
    count: int
    where: str


FASTENERS: tuple[Fastener, ...] = (
    Fastener(
        "M2 × 5 둥근머리 십자볼트 (서보 나사 구멍 바닥에 닿으면 M2 × 4)", 4, "서보 출력 면 → 집"
    ),
    Fastener("M3 × 10 둥근머리 렌치볼트", 10, "원판 → 혼 (한 원판에 넷), 게이지 누름 나사 둘"),
    Fastener(
        "M3 × 12 둥근머리 렌치볼트",
        9,
        "원판 혀 → 시험 링크 넷, 서보 모형 뒷면 넷, 받침 기둥 고리 조임",
    ),
    Fastener("M3 × 8 둥근머리 렌치볼트", 2, "고리 → 집"),
    Fastener("M3 육각너트", 7, "너트 홈 (가진 것)"),
    Fastener("영점 핀 MSH3-20", 1, "J3 = 0에서 원판 → 집 (주문한 것)"),
)


# ------------------------------------------------------------------------------------------------
# 검사


def rot_y(q_deg: float) -> Rot:
    return Rot(0, q_deg, 0)


def ring_screw_heads() -> Compound:
    """고리를 집에 다는 나사(M3) 머리 둘. 지름 6, 높이 RING_HEAD_H로 둥근머리와 원통머리를 다 덮는다."""
    heads = [cyl_x(3.0, e.R_CYL - 0.5, e.R_CYL + RING_HEAD_H, y=y) for y in e.RING_SCREWS_Y]
    return solid(heads[0] + heads[1])


def clashes(
    q_values: list[float], min_volume: float = 0.5
) -> dict[float, list[tuple[str, str, float]]]:
    """관절 각마다 자식 부품과 부모 부품(서보, 커넥터, 고리 나사 머리 포함)이 겹치는 부피 [mm³]."""
    parents = {
        "servo": e.servo(),
        "plugs": e.plugs(),
        "house": e.house(),
        "ring": e.ring(),
        "ring_screws": ring_screw_heads(),
    }
    children = {"disc_out": e.disc("out"), "disc_back": e.disc("back"), "test_link": test_link()}
    out: dict[float, list[tuple[str, str, float]]] = {}
    for q in q_values:
        hits = []
        for cn, cp in children.items():
            moved = rot_y(q) * cp
            for pn, pp in parents.items():
                v = (moved & pp).volume
                if v > min_volume:
                    hits.append((cn, pn, v))
        out[q] = hits
    return out


def pin_interference() -> dict[str, float]:
    """J3 = 0에서 영점 핀(지름 3, 길이 20)을 끝까지 꽂았을 때 부품과 겹치는 부피 [mm³]. 0이어야 한다."""
    pin_bottom = e.Y_OUT - e.PIN_DEPTH
    pin = cyl_y(1.5, pin_bottom, pin_bottom + 20.0, *e.PIN_XZ)
    return {"house": (pin & e.house()).volume, "disc_out": (pin & e.disc("out")).volume}


def pin_grip() -> float:
    """핀을 끝까지 꽂았을 때 원판 바깥으로 나와 손으로 잡는 길이 [mm]."""
    return (e.Y_OUT - e.PIN_DEPTH + 20.0) - (e.Y_DISC_OUT + e.DISC_T)


def screw_depths() -> dict[str, float]:
    """혼 나사가 혼에 들어가는 길이 [mm]. 출력 혼 4.5, 뒷면 혼 3.1 안에서 끝나야 한다."""
    out_head = e.Y_DISC_OUT + e.DISC_T - e.OUT_SCREW_HEAD
    back_head = e.Y_DISC_BACK - e.DISC_T + e.BACK_SCREW_HEAD
    return {
        "출력 혼 (M3 × 10)": e.HORN_TOP - (out_head - 10.0),
        "뒷면 혼 (M3 × 10)": (back_head + 10.0) - e.BACK_HORN[0],
    }


def ring_screw_past_nut() -> float:
    """고리 나사(M3 × 8)의 끝이 서보 집 막대 속 너트를 지나간 길이 [mm]. 둥근머리(지름 5.7)는 고리의 둥근 바깥 면에
    가운데가 떠서 앉는다."""
    head_r = 5.7 / 2
    seat = e.R_CYL + (e.R_CYL - math.sqrt(e.R_CYL**2 - head_r**2))
    tip = seat - e.RING_SCREW_LENGTH
    return (e.RING_NUT_X - e.FITS.nut_height / 2) - tip


def gauge_set_screws() -> dict[str, float]:
    """게이지 누름 나사(M3 × 10)가 줄기를 누를 때 [mm]: 나사가 PLA에 물리는 길이와, 나사 머리가 면에서 떨어진 거리.

    줄기는 나사에 밀려 구멍의 먼 벽에 붙는다. 머리가 면에 닿기 전에 나사 끝이 줄기에 닿아야 조인다.
    """
    near = GAUGE_STEM_D - GAUGE_STEM_HOLE / 2  # 줄기가 먼 벽에 붙었을 때 가까운 면 (구멍 축에서)
    out: dict[str, float] = {}
    for name, face in (("세로 집게", -GAUGE_CLAMP_FRONT), ("옆 집게", GAUGE_SIDE_HX)):
        out[f"{name}: 물리는 길이"] = face - GAUGE_STEM_HOLE / 2
        out[f"{name}: 머리와 면 사이"] = GAUGE_SET_SCREW - (face - near)
    return out


def gauge_preload() -> dict[str, tuple[float, float]]:
    """게이지 바늘이 링크를 누르는 깊이 [mm]의 범위 (줄기를 집게 안에서 밀어 맞출 수 있는 범위).

    세로 집게: 게이지 목이 집게 윗면에 닿을 때(가장 깊이)부터 줄기가 집게에 8 mm만 남을 때까지.
    옆 집게: 목이 옆 집게 뒷면에 닿을 때(가장 깊이)부터 줄기가 8 mm만 남을 때까지. 링크 뒷면은 y = LINK_WIDTH / 2.
    """
    top = GAUGE_CLAMP_X + GAUGE_CLAMP_H / 2
    v_deep = LINK_TOP - (top - GAUGE_STEM_L - GAUGE_TIP)
    v_shallow = LINK_TOP - (top - 8.0 - GAUGE_TIP)
    back = LINK_WIDTH / 2
    s_deep = (GAUGE_SIDE_Y1 - GAUGE_STEM_L - GAUGE_TIP) * -1 + back
    s_shallow = back - (GAUGE_SIDE_Y1 - 8.0 - GAUGE_TIP)
    return {"세로 집게": (v_shallow, v_deep), "옆 집게": (s_shallow, s_deep)}


# --- 짐 -----------------------------------------------------------------------------------------
# 500 ml 생수병을 링크와 직각(y)으로 눕혀 링크 아래에 단다. 링크 끝 구멍(지름 5 mm)에 꿴 끈의 앞 끝은 병목에, 뒤 끝은
# 병 바닥 쪽 몸통에 묶는다. 두 곳에 묶어서 병이 링크 방향으로 돌지 않는다. 저울(500 g까지)로 끈까지 재며 물을
# 따라 내서 무게를 맞춘다. 짐을 뺄 때는 병 밑에 받침(책)을 넣어 끈을 느슨하게 한다 (링크를 손으로 건드리지 않는다).
LOAD_BOTTLE_D, LOAD_BOTTLE_L = (
    65.0,
    210.0,
)  # 500 ml 생수병 어림 (지름, 길이). 다른 물건도 이 안에 들면 된다
LOAD_GAP = 35.0  # 병 아래 → 책상


def load_envelope() -> Compound:
    """매단 물병이 차지하는 자리 (시험 자세, 서보 좌표). 링크 방향으로 ±10 mm 흔들리는 몫을 더한다."""
    r = LOAD_BOTTLE_D / 2
    xc = X_DESK + LOAD_GAP + r
    swing = 10.0
    body = cyl_y(r, -LOAD_BOTTLE_L / 2, LOAD_BOTTLE_L / 2, xc, -HOOK_R)
    return solid(body + body.moved(Location((0, 0, swing))) + body.moved(Location((0, 0, -swing))))


def rig_gaps() -> dict[str, float]:
    """시험대에서 서로 움직이지 않는 부품 사이의 겹침 부피 [mm³] (0이어야 한다)와 짐까지 거리 [mm].

    고리와 서보 집처럼 나사로 고정하는 부품끼리도 본다. 게이지 받침은 시험 자리(z = -GAUGE_R)에 둔다.
    """
    house, ring, link = e.house(), e.ring(), rot_y(TEST_ANGLE) * test_link()
    gauge = gauge_bracket().moved(Location((0, 0, -GAUGE_R)))
    load = load_envelope()
    pairs = {
        "고리 / 서보 집": (ring, house),
        "서보 / 서보 집": (e.servo(), house),
        "서보 모형 / 서보 집": (dummy(), house),
        "받침 기둥 / 서보 집": (stand(), house),
        "게이지 받침 / 시험 링크": (gauge, link),
        "게이지 받침 / 받침 기둥": (gauge, stand()),
        "짐 / 게이지 받침": (load, gauge),
        "짐 / 받침 기둥": (load, stand()),
    }
    out = {k: (a & b).volume for k, (a, b) in pairs.items()}
    out["짐 아래 → 책상"] = LOAD_GAP
    return out


def estimated_mass_g(part: Compound, preset: str = "0.20mm Strength", full: bool = False) -> float:
    """출력 무게 어림 [g] (docs/decisions/0006의 "질량"): 겉넓이 × 벽 두께 + 안쪽 × 채움."""
    volume = part.volume
    if full:
        return PLA_DENSITY * volume
    wall = STRENGTH_WALL if preset == "0.20mm Strength" else 0.9
    infill = STRENGTH_INFILL if preset == "0.20mm Strength" else 0.15
    shell = min(part.area * wall, volume)
    return PLA_DENSITY * (shell + (volume - shell) * infill)


TEST_LOAD_G = 500.0  # 링크 끝 구멍에 매다는 짐 (저울로 잰다, 저울은 500 g까지)
GAUGE_R = 160.0  # 게이지를 대는 반지름 (짐 구멍과 5 cm 띄운다)
E_PLA_MPA = 2000.0  # 출력한 PLA의 탄성 계수 어림 (studies/covers.py와 같은 낮은 값)


def test_load_torque() -> tuple[float, float]:
    """시험 자세(링크 수평)에서 J3가 버티는 토크 [N·m]와 STS3250 정격 대비 비율. 링크 자체 무게도 넣는다."""
    link = test_link()
    link_g = estimated_mass_g(link)
    link_r = link.center().X  # 부피 중심의 J3 축에서 거리 [mm]
    torque = 9.81e-3 * (TEST_LOAD_G * HOOK_R + link_g * link_r) * 1e-3
    return torque, torque / load_servos()[e.SERVO].rated_torque


def link_bending_mm(r: float = GAUGE_R) -> float:
    """짐 때문에 시험 링크 자체가 휘어 반지름 r에서 내려가는 양 [mm] (뿌리에서 외팔보, 속은 벽 2.6 mm 상자로 본다).

    서보 모형으로 잰 프레임 휨에 이 값이 들어간다. 링크는 관절 부품이 아니라 따로 빼서 본다.
    """
    t = STRENGTH_WALL
    b, h = LINK_WIDTH, LINK_DEPTH  # 시험 자세에서 휘는 방향 높이가 h
    inertia = (b * h**3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12
    a = HOOK_R - e.TONGUE_R  # 뿌리에서 짐까지
    x = min(r, HOOK_R) - e.TONGUE_R
    force = TEST_LOAD_G * 9.81e-3  # [N]
    return force * x**2 * (3 * a - x) / (6 * E_PLA_MPA * inertia)


def report() -> None:
    print(f"공차: {e.FILAMENT}, 서보 자리 여유 {e.FITS.seat_clearance} mm")
    if e.FITS.provisional:
        print(f"  임시 값 (3차 시험편 결과로 바꾼다): {', '.join(e.FITS.provisional)}")
    print("\n부품 (출력 크기와 무게 어림)")
    total_child = total_parent = 0.0
    for spec in PARTS:
        part = placed_for_print(spec)
        bb = part.bounding_box()
        m = estimated_mass_g(spec.build(), spec.preset, full="100%" in spec.note)
        if spec.name.startswith("elbow"):
            if spec.child:
                total_child += m
            else:
                total_parent += m
        print(
            f"  {spec.label:8s} {spec.color:5s} {bb.size.X:6.1f} × {bb.size.Y:6.1f} × {bb.size.Z:6.1f} mm  {m:6.1f} g  {spec.preset}"
        )
    print(f"  팔꿈치 부모(집, 고리) {total_parent:.0f} g, 자식(원판 둘) {total_child:.0f} g")
    overlap = {k: round(v, 3) for k, v in pin_interference().items()}
    print(f"\n영점 핀 (J3 = 0): 부품과 겹침 {overlap} mm³, 원판 밖으로 {pin_grip():.1f} mm 나옴")
    print("\n혼 나사가 혼에 들어가는 길이")
    for k, v in screw_depths().items():
        print(f"  {k}: {v:.1f} mm")
    q_values = [float(q) for q in np.arange(-50.0, 141.0, 2.0)]
    hits = clashes(q_values)
    free = [q for q, h in hits.items() if not h]
    first_hit = min((q for q, h in hits.items() if h and q > 0), default=None)
    print(
        f"\n부딪힘: -50°~{first_hit - 2 if first_hit else 140:.0f}°에서 없음 ({len(free)}/{len(q_values)}개 각)"
    )
    if first_hit is not None:
        print(f"  처음 닿는 각 {first_hit:.0f}°: {hits[first_hit]}")
    torque, ratio = test_load_torque()
    print(
        f"\n시험 짐 {TEST_LOAD_G:.0f} g (구멍 {HOOK_R:.0f} mm) + 링크 무게: J3 토크 {torque:.2f} N·m, "
        f"정격의 {ratio:.2f}배"
    )
    print(f"  링크 자체 휨 (게이지 자리 {GAUGE_R:.0f} mm): {link_bending_mm():.2f} mm")
    print("\n시험대 (움직이지 않는 부품끼리 겹침 mm³, 짐까지)")
    for k, v in rig_gaps().items():
        print(f"  {k}: {v:.2f}")
    print(f"  고리 나사(M3 × 8)가 너트를 지나간 길이: {ring_screw_past_nut():.2f} mm")
    print("\n게이지")
    for k, v in gauge_set_screws().items():
        print(f"  누름 나사 {k}: {v:.1f} mm")
    for k, (lo, hi) in gauge_preload().items():
        print(f"  바늘이 눌리는 깊이 {k}: {lo:.1f} ~ {hi:.1f} mm")
    print("\n볼트와 나사")
    for f in FASTENERS:
        print(f"  {f.item} × {f.count}  ({f.where})")


def export(out_dir: Path = OUT_DIR) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for spec in PARTS:
        path = out_dir / f"{spec.name}.stl"
        export_stl(placed_for_print(spec), str(path))
        paths.append(path)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description="관절 하나 시험(J3) 부품 검사와 STL 내보내기")
    parser.add_argument("--check", action="store_true", help="검사만 하고 STL은 만들지 않는다")
    parser.add_argument("--out", type=Path, default=OUT_DIR)
    args = parser.parse_args()
    report()
    if not args.check:
        for path in export(args.out):
            print(path)


if __name__ == "__main__":
    main()
