"""부품 코드가 같이 쓰는 형상 도우미 (단위 mm).

상자와 원기둥을 좌표 범위로 만들고, 볼트 구멍, 너트 홈, 물방울 구멍을 낸다. 너트 홈 치수는 공차 시험편으로
고른 값(data/measured.toml)을 쓴다 (docs/decisions/0003-fastening-nut-traps.md).
"""

from __future__ import annotations

import math
from typing import cast

from build123d import (
    Align,
    Box,
    Circle,
    Compound,
    Cone,
    Cylinder,
    Location,
    Plane,
    Polygon,
    Pos,
    RegularPolygon,
    Rot,
    Sketch,
    Vector,
    extrude,
)

Point = tuple[float, float, float]

# ISO 4032 M3 육각너트 두께. 홈 높이는 1차 시험편에서 고른 값을 쓴다.
M3_NUT_THICKNESS = 2.4


def solid(shape: object) -> Compound:
    """불 연산 결과를 Compound로 받는다. build123d의 연산자 타입은 선이나 면이 섞인 합집합이라 여기서 좁힌다."""
    if not isinstance(shape, Compound):
        raise TypeError(f"입체가 아니다: {type(shape).__name__}")
    return shape


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Compound:
    """좌표 범위로 만든 상자."""
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _up(radius: float, length: float) -> Compound:
    return Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl_x(radius: float, x0: float, x1: float, y: float = 0.0, z: float = 0.0) -> Compound:
    """x 방향 원기둥 (x0 → x1)."""
    return Pos(x0, y, z) * (Rot(0, 90, 0) * _up(radius, x1 - x0))


def cyl_y(radius: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0) -> Compound:
    """y 방향 원기둥 (y0 → y1)."""
    return Pos(x, y0, z) * (Rot(-90, 0, 0) * _up(radius, y1 - y0))


def cyl_z(radius: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> Compound:
    """z 방향 원기둥 (z0 → z1)."""
    return Pos(x, y, z0) * _up(radius, z1 - z0)


def cone_x(r0: float, r1: float, x0: float, x1: float) -> Compound:
    """x 방향 원뿔대 (x0에서 반지름 r0, x1에서 r1)."""
    return Pos(x0, 0, 0) * (
        Rot(0, 90, 0) * Cone(r0, r1, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )


def _pt(v: Vector) -> Point:
    return (v.X, v.Y, v.Z)


def rod(a: Point, b: Point, radius: float) -> Compound:
    """a에서 b까지 원기둥."""
    va, vb = Vector(*a), Vector(*b)
    return _up(radius, (vb - va).length).moved(Location(Plane(origin=va, z_dir=vb - va)))


def slot(a: Point, b: Point, radius: float, half_width: float, side: Point) -> Compound:
    """a에서 b까지 가는 납작한 길: 반지름 radius의 원기둥 둘을 side 방향으로 ±half_width 띄우고 그 사이를 채운다.

    5264 플러그처럼 납작한 것이 지나가는 길이다. side는 a→b와 직각인 단위 벡터다.
    """
    va, vb, vs = Vector(*a), Vector(*b), Vector(*side)
    d = vb - va
    ends = [rod(_pt(va + vs * s), _pt(vb + vs * s), radius) for s in (-half_width, half_width)]
    web = Box(2 * radius, 2 * half_width, d.length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    plane = Plane(origin=va, x_dir=d.cross(vs), z_dir=d)
    return solid(ends[0] + ends[1] + web.moved(Location(plane)))


def nut_trap(
    center: Point, axis: Point, slide: Point, length: float, width: float, height: float
) -> Compound:
    """옆에서 밀어 넣는 너트 홈. 볼트는 axis 방향, 너트는 slide 쪽 입구에서 length만큼 밀려 들어온다.

    홈 끝은 육각이라 너트가 볼트 구멍 가운데에서 멈추고 돌지 않는다 (1차 시험편과 같은 모양).
    width는 맞변 거리(홈 폭), height는 볼트 방향 홈 높이다.
    """
    hexagon = extrude(
        RegularPolygon(width / 2, 6, major_radius=False), amount=height / 2, both=True
    )
    channel = Pos(length / 2, 0, 0) * Box(length, width, height)
    return solid((hexagon + channel).moved(Location(Plane(origin=center, x_dir=slide, z_dir=axis))))


def teardrop(origin: Point, axis: Point, length: float, diameter: float, roof: Point) -> Compound:
    """origin에서 axis 방향으로 length만큼 가는 가로 구멍. roof 방향(출력할 때 위, axis와 직각인 단위 벡터)으로
    45° 지붕을 내서 서포트 없이 뽑힌다."""
    r = diameter / 2
    c = r * math.cos(math.pi / 4)
    # 지붕 삼각형은 반시계 방향으로 준다. 시계 방향이면 면이 뒤집혀 원과 반대쪽으로 밀린다
    profile = Circle(r) + Polygon((-c, c), (c, c), (0.0, r * math.sqrt(2)), align=None)
    body = extrude(cast(Sketch, profile), amount=length)  # 지붕은 지역 +Y, 민 방향은 지역 +Z
    va, vr = Vector(*axis), Vector(*roof)
    # 지역 z = axis, 지역 y = roof가 되도록 지역 x = roof × axis (y = z × x)
    plane = Plane(origin=origin, x_dir=vr.cross(va), z_dir=va)
    return solid(body.moved(Location(plane)))


def teardrop_y(diameter: float, y0: float, y1: float, x: float, z: float, roof: Point) -> Compound:
    """y 방향 가로 구멍. roof 방향(출력할 때 위, xz 평면의 단위 벡터)으로 45° 지붕을 낸다.

    서포트 없이 뽑히고, 지붕 쪽으로만 틈이 생긴다. 영점 핀처럼 한 방향만 정확하면 되는 구멍은 지붕이
    그 방향과 직각이 되게 둔다.
    """
    r = diameter / 2
    c = r * math.cos(math.pi / 4)
    profile = Circle(r) + Polygon((-c, c), (c, c), (0.0, r * math.sqrt(2)), align=None)
    body = extrude(cast(Sketch, profile), amount=y1 - y0)  # 지붕은 +Y, 민 방향은 +Z
    rx, _, rz = roof
    x_dir = (-rz, 0.0, rx)  # roof × (0, 1, 0): 이 틀에서 지역 y가 roof, 지역 z가 세계 y다
    return solid(body.moved(Location(Plane(origin=(x, y0, z), x_dir=x_dir, z_dir=(0.0, 1.0, 0.0)))))


# 바깥 상자는 빠른 어림이라 곡면에서 실제보다 조금 작을 수 있다. 이만큼 더 떨어진 쌍만 건너뛴다
BOX_MARGIN = 1.0


def far_apart(a: Compound, b: Compound, limit: float) -> bool:
    """두 형상의 바깥 상자가 limit + BOX_MARGIN보다 멀리 떨어져 있다.

    두 형상 사이의 실제 틈은 바깥 상자 사이 거리보다 작지 않아서, 그런 쌍은 틈이 limit보다 크다. 틈 검사에서 먼 쌍의
    거리 계산(부품 하나에 0.1 ~ 0.5초)을 건너뛰는 데 쓴다. 결과는 거르지 않을 때와 같다.
    """
    ba, bb = a.bounding_box(optimal=False), b.bounding_box(optimal=False)
    dx = max(0.0, bb.min.X - ba.max.X, ba.min.X - bb.max.X)
    dy = max(0.0, bb.min.Y - ba.max.Y, ba.min.Y - bb.max.Y)
    dz = max(0.0, bb.min.Z - ba.max.Z, ba.min.Z - bb.max.Z)
    return math.sqrt(dx * dx + dy * dy + dz * dz) > limit + BOX_MARGIN
