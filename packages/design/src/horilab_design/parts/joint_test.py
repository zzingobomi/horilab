"""관절 하나 시험(docs/decisions/0021-elbow-joint-test.md)의 시험대와 출력 목록, 검사.

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
from build123d import Box, Compound, Location, Plane, Polygon, Pos, Rot, export_stl, extrude

from horilab_design.parts import elbow as e
from horilab_design.parts.shapes import box, cyl_x, cyl_y, nut_trap, solid
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
POST_W = 50.0
LINK_LENGTH = 215.0  # J3 축에서 시험 링크 끝까지
LINK_DEPTH, LINK_WIDTH = 30.0, 24.0
HOOK_R = LINK_LENGTH - 5.0  # 짐을 거는 구멍 (J3 축에서)
RING_HEAD_H = 3.0  # 고리 나사 머리가 고리 위로 솟는 높이 (원통머리 3.0과 둥근머리 1.65 중 큰 것)
LINK_TOP = LINK_DEPTH / 2  # 시험 자세에서 링크 윗면의 x
GAUGE_ABOVE = 22.0  # 링크 윗면 → 게이지 줄기 잡는 고리 가운데 (줄기를 고리에서 밀어 맞춘다)
GAUGE_T = 16.0  # 게이지 받침 두께 (z)
# 게이지 줄기(지름 8, h6)를 무는 갈라진 고리의 구멍. 가로 구멍이라 작게 뽑히는 몫(0.2)과 여유를 두고 넉넉히 낸다.
# 틈(1.6 mm)을 다 죄면 지름이 약 0.5 mm 줄어 줄기를 문다. 그래서 공차 시험편이 필요 없다.
GAUGE_STEM_HOLE = 8.4
GAUGE_CLAMP_HW = 8.0  # 집게 몸통 반폭 (y)
GAUGE_SLIT = 1.6  # 집게 틈
GAUGE_EAR_H = 10.0  # 조임 귀가 받침 윗면(+z) 위로 솟는 높이
GAUGE_BOLT_Z = GAUGE_T / 2 + GAUGE_EAR_H / 2  # 조임 볼트(y 방향) 높이
GAUGE_NUT_Y = -4.4  # 너트 가운데 (-y 쪽 귀 안, 바깥 면과 틈에서 각각 약 2.3 mm)
GAUGE_BOLT_LENGTH = 15.0  # M3 × 15


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
    s -= box(x_top - e.SPIGOT_L - 4, x_top + 1, -0.8, 0.8, 0.0, ear_z1 + 1)  # 가르는 틈
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


def _stem_hole(x0: float, x1: float) -> Compound:
    """게이지 줄기 구멍 (x 방향 가로 구멍). 출력할 때 위(+z)로 45° 지붕을 내서 서포트 없이 뽑힌다."""
    r = GAUGE_STEM_HOLE / 2
    c = r / math.sqrt(2)
    diamond = Pos((x0 + x1) / 2, 0, 0) * (Rot(45, 0, 0) * Box(x1 - x0, 2 * r, 2 * r))
    roof = solid(diamond & box(x0, x1, -2 * r, 2 * r, c, 2 * r))
    return solid(cyl_x(r, x0, x1) + roof)


def gauge_bracket() -> Compound:
    """다이얼 게이지 받침: 책상 가장자리에 무는 발, 기둥, 링크 위로 뻗은 팔, 줄기(지름 8)를 잡는 갈라진 집게.

    어느 반지름에 두든 같은 모양이다 (C 클램프로 가장자리를 따라 옮긴다). z를 위로 납작하게 뽑는다.
    집게의 틈은 줄기 구멍에서 +z로 갈라져 출력할 때 세로로 서고, 조이는 귀는 +z로 솟는다. 그래서 공중에서
    시작하는 층이 없어 서포트 없이 뽑힌다.
    """
    t = GAUGE_T / 2
    x_base = X_DESK + BASE_T
    x_clamp = LINK_TOP + GAUGE_ABOVE
    yc = EDGE_Y + 14.0
    g = box(X_DESK, x_base, EDGE_Y, EDGE_Y + 40, -t, t)
    col = box(x_base, x_clamp + 8, yc - 7, yc + 7, -t, t)
    col -= box(x_base + 3, x_clamp + 5, yc - 4.4, yc + 4.4, -t + 2.4, t - 2.4)
    g += col + box(x_clamp - 8, x_clamp + 8, yc, -6.0, -t, t)
    hw = GAUGE_CLAMP_HW
    g += box(x_clamp - 8, x_clamp + 8, -hw, hw, -t, t + GAUGE_EAR_H)  # 집게 몸통과 조임 귀
    g -= _stem_hole(x_clamp - 10, x_clamp + 10)
    g -= box(x_clamp - 9, x_clamp + 9, -GAUGE_SLIT / 2, GAUGE_SLIT / 2, 0.0, t + GAUGE_EAR_H + 1)
    g -= cyl_y(e.FITS.m3_hole_horizontal / 2, -hw - 1, hw + 1, x_clamp, GAUGE_BOLT_Z)
    # 볼트는 +y 면에서 넣고, 너트는 -y 쪽 귀에 위(+z)에서 떨어뜨려 넣는다. M3 × 15가 너트를 지나간다.
    g -= _nut(
        (x_clamp, GAUGE_NUT_Y, GAUGE_BOLT_Z),
        (0, 1, 0),
        (0, 0, 1),
        t + GAUGE_EAR_H - GAUGE_BOLT_Z + 1,
    )
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
        _plane((0, 0, -GAUGE_T / 2), (0, 0, 1), (1, 0, 0)),
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
    Fastener("M3 × 10 둥근머리 렌치볼트", 8, "원판 → 혼 (한 원판에 넷)"),
    Fastener(
        "M3 × 12 둥근머리 렌치볼트",
        9,
        "원판 혀 → 시험 링크 넷, 서보 모형 뒷면 넷, 받침 기둥 고리 조임",
    ),
    Fastener("M3 × 8 둥근머리 렌치볼트", 2, "고리 → 집"),
    Fastener("M3 × 15 둥근머리 렌치볼트 (16 mm는 팔지 않는다)", 1, "게이지 고리 조임"),
    Fastener("M3 육각너트", 8, "너트 홈 (가진 것)"),
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


def gauge_clamp_fit() -> dict[str, float]:
    """게이지 조임 볼트(+y 면에서 넣는다)와 너트 자리 [mm]: 너트를 지나간 길이, 볼트 끝에서 -y 면까지,
    너트 홈과 바깥 면·틈 사이 벽."""
    tip = GAUGE_CLAMP_HW - GAUGE_BOLT_LENGTH
    nut_lo = GAUGE_NUT_Y - e.FITS.nut_height / 2
    nut_hi = GAUGE_NUT_Y + e.FITS.nut_height / 2
    return {
        "너트를 지나간 길이": nut_lo - tip,
        "볼트 끝 → -y 면": tip + GAUGE_CLAMP_HW,
        "너트 홈 → -y 면 벽": nut_lo + GAUGE_CLAMP_HW,
        "너트 홈 → 틈 벽": -GAUGE_SLIT / 2 - nut_hi,
    }


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
