"""공차 시험편 (docs/manufacturing.md의 "공차").

칸마다 치수를 조금씩 바꾸고 윗면에 그 치수를 새긴다. 실물(볼트, 너트, 서보, 혼)을 끼워 보고
가장 잘 맞는 칸의 치수를 data/measured.toml에 필라멘트마다 적는다.

- 1차: M3 볼트 구멍과 너트 홈
- 2차: 서보 몸체 자리와 혼 자리. 도면 치수(params/servos.toml)에 여유를 더해 간다
- 3차: 영점 핀 구멍(MISUMI MSH3, 0015), J1 베어링 6806의 바깥 링 자리와 안쪽 링 축(0015)

    uv run python -m horilab_design.coupons    # build/coupons/에 STL을 만든다

모두 놓인 그대로(글자가 위) 서포트 없이 뽑는다. 출력 설정은 docs/manufacturing.md의 "출력 설정"이고,
높이 방향 치수는 0.2 mm 층에 맞춘다 (첫 층도 0.2 mm). 부품 코드라서 단위는 mm다.
"""

from __future__ import annotations

import argparse
import math
from collections.abc import Callable
from pathlib import Path

from build123d import (
    Align,
    Box,
    Compound,
    Cylinder,
    Part,
    Pos,
    RegularPolygon,
    Rot,
    Text,
    export_stl,
    extrude,
)

from horilab_design.values import REPO_ROOT, ServoSpec, load_servos

OUT_DIR = REPO_ROOT / "build" / "coupons"

LAYER = 0.2  # 층 높이 [mm]
M3_NUT_THICKNESS = 2.4  # ISO 4032 M3 육각너트 높이
BOLT_HOLE = 3.4  # 너트 홈 시험편의 볼트 구멍. 구멍 시험 결과가 나오면 그 값으로 바꾼다

TAP_HOLES = (2.5, 2.6, 2.7, 2.8)  # M3 볼트를 플라스틱에 바로 조일 구멍
CLEARANCE_HOLES = (3.1, 3.2, 3.3, 3.4, 3.5)  # M3 볼트가 지나갈 구멍
NUT_POCKETS = (5.5, 5.6, 5.7, 5.8, 5.9, 6.0)  # 위에서 끼우는 육각 홈의 맞변 거리
POCKET_DEPTH = M3_NUT_THICKNESS + LAYER
SLOT_HEIGHT = 2.8  # 옆에서 밀어 넣는 홈의 기본 높이 (너트 2.4 + 층 두 개, 홈 천장이 조금 처진다)
NUT_SLOTS = (  # 옆에서 밀어 넣는 홈 (폭, 높이). 앞 넷은 폭을, 뒤 둘은 높이를 바꾼다
    (5.6, SLOT_HEIGHT),
    (5.7, SLOT_HEIGHT),
    (5.8, SLOT_HEIGHT),
    (5.9, SLOT_HEIGHT),
    (5.8, 2.6),
    (5.8, 3.0),
)

SEAT_CLEARANCES = (0.1, 0.2, 0.3, 0.4, 0.5)  # 서보 단면의 길이와 폭에 더하는 여유
HORN_CLEARANCES = (0.1, 0.2, 0.3, 0.4)  # 혼 지름에 더하는 여유
FRAME_WALL = 2.4
FRAME_HEIGHT = 6.0
LABEL_TAB = 7.0  # 틀의 한쪽 벽을 넓혀 글자를 새긴다
CORNER_RELIEF = 2.0  # 안쪽 모서리를 둥글게 더 파서 각진 서보 모서리가 걸리지 않게 한다 (지름)
HORN_RECESS_DEPTH = 2.0
HORN_SCREW_RELIEF = 7.0  # 혼 가운데 나사 머리가 들어갈 구멍

# 3차: 손으로 꽂았다 빼는 영점 핀(지름 3, h7: 2.990~3.000). 핀 20 mm 중 절반이 들어가는 두께로 본다
# 세로 구멍이 가로보다 더 작게 뽑힌다. 흰색 첫 세로판(3.00~3.25)에는 핀이 한 칸도 들어가지 않았고,
# 가로는 3.25가 뻑뻑하고 3.30이 맞았다 (2026-11-10). 그래서 세로판 칸을 올렸다.
# 1차의 M3 볼트(세로 3.1, 가로 3.3에서 지나감)는 나사산이 플라스틱을 밀고 지나가서 핀과 견줄 수 없었다
PIN_HOLES = (3.30, 3.35, 3.40, 3.45, 3.50, 3.55)
PIN_HOLES_HORIZONTAL = (3.20, 3.25, 3.30, 3.35, 3.40, 3.45)
PIN_DEPTH = 10.0
# 3차: 6806 (30 × 42 × 7). 바깥 링 공차 0/-0.011, 안쪽 링 0/-0.010. 손으로 밀어 넣어 놀지 않는 칸을 고른다
# 큰 세로 구멍도 0.35 작게 뽑혀서(흰색 관 고리 34.8 → 34.45) 맞는 칸이 위 끝에 올 수 있어 42.5를 더했다
BEARING_BORES = (42.0, 42.1, 42.2, 42.3, 42.4, 42.5)  # 바깥 링이 들어갈 구멍
BEARING_SHAFTS = (29.8, 29.9, 30.0, 30.1, 30.2)  # 안쪽 링이 끼워질 축
SEAT_DEPTH = 7.2  # 베어링 폭 7 mm + 층 하나
LIP_HEIGHT = 1.6  # 바깥 링이 얹히는 바닥 턱
LIP_BORE = 37.0  # 턱 안지름. 바깥 링 면에만 닿고 안쪽 링(바깥지름 약 34)은 비킨다
RING_WALL = 5.0
SHOULDER_D = 33.0  # 안쪽 링이 얹히는 턱 지름. 안쪽 링 면에만 닿고 바깥 링(안지름 약 38)은 비킨다
SHOULDER_H = 0.8
STUB_BASE = 2.0  # 축 받침판 두께
STUB_HEIGHT = 8.0  # 베어링 폭보다 1 mm 길게

PITCH = 11.0  # 칸 간격
MARGIN = 2.0
LABEL_SIZE = 3.5
LABEL_DEPTH = 2 * LAYER
CORNER = (Align.MIN, Align.MIN, Align.MIN)  # 원점에 모서리를 둔다


def _x(i: int) -> float:
    return MARGIN + PITCH * (i + 0.5)


def _length(cells: int) -> float:
    return 2 * MARGIN + PITCH * cells


def _label(text: str, x: float, y: float, top: float) -> Part:
    return extrude(Pos(x, y, top) * Text(text, font_size=LABEL_SIZE), amount=-LABEL_DEPTH)


def _vertical_hole(diameter: float, x: float, y: float, height: float) -> Part:
    return Pos(x, y, -1.0) * Cylinder(
        diameter / 2, height + 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )


def _horizontal_hole(diameter: float, x: float, z: float, length: float) -> Part:
    """y 방향 둥근 가로 구멍.

    1차 시험편의 가로 구멍은 이 모양으로 뽑아서 맞춰 봤다 (지름 5 mm 안팎까지는 서포트 없이 뽑힌다).
    """
    return Pos(x, length + 1.0, z) * (
        Rot(90, 0, 0)
        * Cylinder(diameter / 2, length + 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )


def vertical_holes() -> Compound:
    """세로 구멍판 (6 mm 두께). 아랫줄은 조여 넣는 구멍, 윗줄은 지나가는 구멍."""
    height, row = 6.0, 14.0
    plate: Compound = Box(_length(len(CLEARANCE_HOLES)), 2 * row + MARGIN, height, align=CORNER)
    for r, holes in enumerate((TAP_HOLES, CLEARANCE_HOLES)):
        y = MARGIN + r * row + 4.0
        for i, d in enumerate(holes):
            plate -= _vertical_hole(d, _x(i), y, height)
            plate -= _label(f"{d:.1f}", _x(i), y + 6.0, height)
    return plate


def horizontal_holes() -> Compound:
    """가로 구멍 막대. 구멍은 y 방향 둥근 구멍이다."""
    depth, height, z = 8.0, 10.0, 4.5
    block: Compound = Box(_length(len(CLEARANCE_HOLES)), depth, height, align=CORNER)
    for i, d in enumerate(CLEARANCE_HOLES):
        block -= _horizontal_hole(d, _x(i), z, depth)
        block -= _label(f"{d:.1f}", _x(i), depth / 2, height)
    return block


def nut_pockets() -> Compound:
    """위에서 너트를 눌러 끼우는 육각 홈. 맞변 거리를 바꾼다."""
    height, y = 6.0, 6.0
    plate: Compound = Box(_length(len(NUT_POCKETS)), 16.0, height, align=CORNER)
    for i, across_flats in enumerate(NUT_POCKETS):
        hexagon = RegularPolygon(across_flats / 2, 6, major_radius=False)
        plate -= Pos(_x(i), y, height - POCKET_DEPTH) * extrude(hexagon, amount=POCKET_DEPTH + 1)
        plate -= _vertical_hole(BOLT_HOLE, _x(i), y, height)
        plate -= _label(f"{across_flats:.1f}", _x(i), y + 7.0, height)
    return plate


def nut_slots() -> Compound:
    """앞면(y = 0)에서 너트를 밀어 넣는 홈. 볼트는 세로로 지나간다.

    홈 끝은 육각이라 너트가 볼트 구멍 가운데에서 멈춘다. 육각의 맞변이 홈 벽과 나란하다.
    """
    depth, height, floor = 16.0, 9.0, 3.0
    bolt_y = 8.0
    block: Compound = Box(_length(len(NUT_SLOTS)), depth, height, align=CORNER)
    for i, (width, slot_height) in enumerate(NUT_SLOTS):
        x = _x(i)
        channel = Pos(x, -1.0, floor) * Box(
            width, bolt_y + 1.0, slot_height, align=(Align.CENTER, Align.MIN, Align.MIN)
        )
        hexagon = RegularPolygon(width / 2, 6, major_radius=False, rotation=30)
        end = Pos(x, bolt_y, floor) * extrude(hexagon, amount=slot_height)
        block -= channel + end
        block -= _vertical_hole(BOLT_HOLE, x, bolt_y, height)
        text = f"{width:.1f}" if slot_height == SLOT_HEIGHT else f"t{slot_height:.1f}"
        block -= _label(text, x, bolt_y + 4.5, height)
    return block


def servo_frames(spec: ServoSpec) -> Compound:
    """서보 몸체가 들어갈 틀. 안쪽은 도면 단면(길이 × 폭)에 여유를 더한 크기이고, 틀 하나에 여유 하나다.

    서보를 출력축 방향으로 넣어 본다. 안쪽 모서리는 둥글게 더 파서 각진 모서리도 걸리지 않는다.
    """
    columns = 3
    frames: list[Compound] = []
    for i, clearance in enumerate(SEAT_CLEARANCES):
        length = spec.body_length_mm + clearance
        width = spec.body_width_mm + clearance
        outer_x, outer_y = length + 2 * FRAME_WALL, width + FRAME_WALL + LABEL_TAB
        x0 = (i % columns) * (outer_x + 4.0)
        y0 = (i // columns) * (outer_y + 4.0)
        frame: Compound = Pos(x0, y0, 0) * Box(outer_x, outer_y, FRAME_HEIGHT, align=CORNER)
        inner_x, inner_y = x0 + FRAME_WALL, y0 + LABEL_TAB
        frame -= Pos(inner_x, inner_y, -1.0) * Box(length, width, FRAME_HEIGHT + 2, align=CORNER)
        for cx in (inner_x, inner_x + length):
            for cy in (inner_y, inner_y + width):
                frame -= _vertical_hole(CORNER_RELIEF, cx, cy, FRAME_HEIGHT)
        frame -= _label(f"+{clearance:.1f}", x0 + outer_x / 2, y0 + LABEL_TAB / 2, FRAME_HEIGHT)
        frames.append(frame)
    return Compound(frames)


def horn_recesses(spec: ServoSpec) -> Compound:
    """혼이 들어갈 얕은 자리. 지름은 혼 지름에 여유를 더하고, M3 구멍이 혼의 구멍 원 위에 있다.

    서보에 달린 혼 위에 시험판을 엎어 얹어 본다.
    """
    pitch = spec.horn_diameter_mm + 6.0
    height = HORN_RECESS_DEPTH + 2.0
    y = MARGIN + pitch / 2
    plate: Compound = Box(
        2 * MARGIN + pitch * len(HORN_CLEARANCES), pitch + LABEL_TAB, height, align=CORNER
    )
    for i, clearance in enumerate(HORN_CLEARANCES):
        x = MARGIN + pitch * (i + 0.5)
        recess = Cylinder((spec.horn_diameter_mm + clearance) / 2, HORN_RECESS_DEPTH + 1.0)
        plate -= Pos(x, y, height - HORN_RECESS_DEPTH + (HORN_RECESS_DEPTH + 1.0) / 2) * recess
        plate -= _vertical_hole(HORN_SCREW_RELIEF, x, y, height)
        r = spec.horn_bolt_circle_mm / 2
        for k in range(spec.horn_bolts):
            angle = 2 * math.pi * k / spec.horn_bolts
            plate -= _vertical_hole(
                BOLT_HOLE, x + r * math.cos(angle), y + r * math.sin(angle), height
            )
        plate -= _label(f"+{clearance:.1f}", x, y + pitch / 2 + LABEL_TAB / 2 - 1.0, height)
    return plate


def pin_holes_vertical() -> Compound:
    """영점 핀이 세로로 지나가는 구멍판. 핀을 손으로 꽂았다 빼 본다."""
    plate: Compound = Box(_length(len(PIN_HOLES)), 16.0, PIN_DEPTH, align=CORNER)
    for i, d in enumerate(PIN_HOLES):
        plate -= _vertical_hole(d, _x(i), 5.5, PIN_DEPTH)
        plate -= _label(f"{d:.2f}", _x(i), 12.0, PIN_DEPTH)
    return plate


def pin_holes_horizontal() -> Compound:
    """영점 핀이 가로로 지나가는 막대. 구멍은 y 방향 둥근 구멍이다."""
    height, z = 10.0, 4.5
    block: Compound = Box(_length(len(PIN_HOLES_HORIZONTAL)), PIN_DEPTH, height, align=CORNER)
    for i, d in enumerate(PIN_HOLES_HORIZONTAL):
        block -= _horizontal_hole(d, _x(i), z, PIN_DEPTH)
        block -= _label(f"{d:.2f}", _x(i), PIN_DEPTH / 2, height)
    return block


def bearing_bores() -> Compound:
    """6806 바깥 링이 들어갈 고리. 고리 하나에 지름 하나이고, 바닥 턱에 바깥 링이 얹힌다.

    베어링을 위에서 손으로 밀어 넣어 본다. 고리 옆 혀에 지름을 새긴다.
    """
    columns, height = 3, LIP_HEIGHT + SEAT_DEPTH
    rings: list[Compound] = []
    for i, d in enumerate(BEARING_BORES):
        outer = BEARING_BORES[-1] + 2 * RING_WALL
        x = (i % columns) * (outer + 4.0) + outer / 2
        y = (i // columns) * (outer + LABEL_TAB + 4.0) + outer / 2
        up = (Align.CENTER, Align.CENTER, Align.MIN)
        ring: Compound = Pos(x, y, 0) * Cylinder(outer / 2, height, align=up)
        ring += Pos(x - 7.0, y + outer / 2 - 2.0, 0) * Box(
            14.0, LABEL_TAB + 2.0, height, align=CORNER
        )
        ring -= Pos(x, y, LIP_HEIGHT) * Cylinder(d / 2, SEAT_DEPTH + 1.0, align=up)
        ring -= _vertical_hole(LIP_BORE, x, y, height)
        ring -= _label(f"{d:.1f}", x, y + outer / 2 + LABEL_TAB / 2 - 1.0, height)
        rings.append(ring)
    return Compound(rings)


def bearing_shafts() -> Compound:
    """6806 안쪽 링이 끼워질 축. 받침판 위 턱에 안쪽 링이 얹힌다. 베어링을 위에서 끼워 본다."""
    columns, size = 3, 40.0
    stubs: list[Compound] = []
    up = (Align.CENTER, Align.CENTER, Align.MIN)
    for i, d in enumerate(BEARING_SHAFTS):
        x0 = (i % columns) * (size + 4.0)
        y0 = (i // columns) * (size + LABEL_TAB + 4.0)
        cx, cy = x0 + size / 2, y0 + size / 2
        stub: Compound = Pos(x0, y0, 0) * Box(size, size + LABEL_TAB, STUB_BASE, align=CORNER)
        stub += Pos(cx, cy, STUB_BASE) * Cylinder(SHOULDER_D / 2, SHOULDER_H, align=up)
        stub += Pos(cx, cy, STUB_BASE + SHOULDER_H) * Cylinder(d / 2, STUB_HEIGHT, align=up)
        stub -= _label(f"{d:.1f}", cx, y0 + size + LABEL_TAB / 2, STUB_BASE)
        stubs.append(stub)
    return Compound(stubs)


COUPONS: dict[str, Callable[[], Compound]] = {
    "fit1_vertical_holes": vertical_holes,
    "fit1_horizontal_holes": horizontal_holes,
    "fit1_nut_pockets": nut_pockets,
    "fit1_nut_slots": nut_slots,
    # STS3215와 STS3250은 도면이 같아서 같은 시험편으로 본다
    "fit2_sts_frames": lambda: servo_frames(load_servos()["STS3215"]),
    "fit2_sts_horn": lambda: horn_recesses(load_servos()["STS3215"]),
    # ST-3120은 도면 치수로 미리 만들어 두고, 서보가 오면 뽑아 끼워 본다
    "fit2_st3120_frames": lambda: servo_frames(load_servos()["ST-3120"]),
    "fit2_st3120_horn": lambda: horn_recesses(load_servos()["ST-3120"]),
    "fit3_pin_vertical": pin_holes_vertical,
    "fit3_pin_horizontal": pin_holes_horizontal,
    "fit3_6806_bores": bearing_bores,
    "fit3_6806_shafts": bearing_shafts,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="공차 시험편 STL을 만든다.")
    parser.add_argument("--out", type=Path, default=OUT_DIR)
    args = parser.parse_args()
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)
    for name, build in COUPONS.items():
        path = out / f"{name}.stl"
        export_stl(build(), str(path))
        print(path)


if __name__ == "__main__":
    main()
