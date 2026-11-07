"""공차 시험편 (docs/manufacturing.md의 "공차").

칸마다 치수를 조금씩 바꾸고 윗면에 그 치수를 새긴다. 실물(볼트, 너트, 서보, 혼)을 끼워 보고
가장 잘 맞는 칸의 치수를 data/measured.toml에 필라멘트마다 적는다.

- 1차: M3 볼트 구멍과 너트 홈
- 2차: 서보 몸체 자리와 혼 자리. 도면 치수(params/servos.toml)에 여유를 더해 간다

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
    Circle,
    Compound,
    Cylinder,
    Part,
    Polygon,
    Pos,
    RegularPolygon,
    Rot,
    Text,
    Wire,
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


def _teardrop_hole(diameter: float, x: float, z: float, length: float) -> Compound | Wire:
    """y 방향 가로 구멍. 위쪽을 45° 지붕으로 막아 서포트 없이 뽑힌다 (manufacturing.md)."""
    r = diameter / 2
    roof = Polygon(  # xy 평면, 지붕이 +y
        (-r * math.cos(math.pi / 4), r * math.sin(math.pi / 4)),
        (0.0, r * math.sqrt(2)),
        (r * math.cos(math.pi / 4), r * math.sin(math.pi / 4)),
        align=None,
    )
    span = length + 2.0
    solid = extrude(Circle(r), amount=span) + extrude(roof, amount=span)  # +z 쪽으로
    # x축으로 90° 돌리면 지붕(+y)은 +z로, 뽑은 방향(+z)은 -y로 간다
    return Pos(x, length + 1.0, z) * (Rot(90, 0, 0) * solid)


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
    """가로 구멍 막대. 구멍은 y 방향이고 물방울 모양이다."""
    depth, height, z = 8.0, 10.0, 4.5
    block: Compound = Box(_length(len(CLEARANCE_HOLES)), depth, height, align=CORNER)
    for i, d in enumerate(CLEARANCE_HOLES):
        block -= _teardrop_hole(d, _x(i), z, depth)
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


COUPONS: dict[str, Callable[[], Compound]] = {
    "fit1_vertical_holes": vertical_holes,
    "fit1_horizontal_holes": horizontal_holes,
    "fit1_nut_pockets": nut_pockets,
    "fit1_nut_slots": nut_slots,
    # STS3215와 STS3250은 도면이 같아서 같은 시험편으로 본다
    "fit2_sts_frames": lambda: servo_frames(load_servos()["STS3215"]),
    "fit2_sts_horn": lambda: horn_recesses(load_servos()["STS3215"]),
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
