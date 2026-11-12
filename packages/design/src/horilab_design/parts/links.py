"""위팔·아래팔 링크: 앞 관절의 출력 원판에 앉는 발 + 납작한 관 + 다음 관절의 집. 한 부품이다 (0029).

좌표는 링크 끝의 집이 품는 서보의 좌표다 (parts/joint.py). 앞 관절 축은 x = -length에서 y와 나란하다.
링크의 납작한 면(뽑을 때 바닥)은 y = FRONT이고, 원통 앞 끝과 한 평면이다. 그래서 두 관절 축이 나란한 정도가
한 부품 안에서, 바닥에 닿아 뽑힌 면으로 정해진다. 앞 관절의 출력 원판은 이 평면 바깥(+y)에서 발을 받친다.
발의 나사 자리는 손목 몸통과 같다 (축에서 ±5, 폭 ±19.4).

    uv run python -m horilab_design.parts.links
"""

from __future__ import annotations

import math

from build123d import Compound

from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_x, cyl_y, solid
from horilab_design.printcheck import overhangs, printed_mass_g

WALL = 2.6  # 0.20mm Strength의 벽 6겹
FOOT_HX = 26.8  # 발 상자의 반 길이 (관 방향)
EMBED = 6.0  # 관을 납작하게 깎는 깊이: 바닥에서 45° 안으로 선다
NECK_END = -40.0  # 집의 목이 관으로 이어지는 x
BED = 256.0  # 출력 범위 [mm]

# 링크마다 (두 관절 축 사이 길이, 관 반지름, 앞 관절의 영점 핀 반지름).
# 위팔의 앞 관절은 어깨(ST-3120)라 출력 원판이 커서 핀이 더 멀다 (어깨는 서보가 오면 재서 그린다).
LINKS: dict[str, tuple[float, float, float]] = {
    "upper_arm": (220.0, 20.0, 33.0),
    "forearm": (180.0, 18.0, j.R_PIN),
}


def _chamfer_x(x0: float, x1: float, y_top: float, hz: float, c: float) -> Compound:
    """x 방향 상자의 뒷면 쪽(-y) 두 모서리를 45°로 깎는 쐐기 둘 (y_top이 상자 윗면, 반폭 hz)."""
    out = None
    for sg in (1, -1):
        pts = [
            (y_top - 0.01, sg * (hz + 0.01)),
            (y_top + c, sg * (hz + 0.01)),
            (y_top - 0.01, sg * (hz - c)),
        ]
        w = j.prism_x(pts, x0, x1)
        out = w if out is None else solid(out + w)
    assert out is not None
    return out


def link(length: float, tube_r: float, pin_r: float = j.R_PIN) -> Compound:
    """링크 하나 (흰색). 납작한 면(y = FRONT)을 바닥에 두고 뽑는다."""
    xa = -length  # 앞 관절 축
    h_t = 2 * tube_r - EMBED  # 관과 발 상자의 높이
    y_top = j.FRONT - h_t
    yc = j.FRONT - (tube_r - EMBED)  # 관 가운데
    hw = j.NECK_IN
    # ---- 겉: 집, 관, 발 상자, 귀, 핀 귀
    tube = cyl_x(tube_r, xa + FOOT_HX - 1, NECK_END + 4, yc, 0.0)
    tube &= box(xa, 0, y_top - 1, j.FRONT, -tube_r - 1, tube_r + 1)
    foot = box(xa - FOOT_HX, xa + FOOT_HX, y_top, j.FRONT, -j.NECK_HW, j.NECK_HW)
    foot -= _chamfer_x(xa - FOOT_HX - 1, xa + FOOT_HX + 1, y_top, j.NECK_HW, 8.0)
    ears = cyl_y(j.R_DRUM, j.FRONT - j.EAR_T, j.FRONT, xa, 0.0)
    ears -= box(
        xa - 30, xa + 30, j.FRONT - j.EAR_T - 1, j.FRONT + 1, -j.NECK_HW + 0.01, j.NECK_HW - 0.01
    )
    tab = box(xa - pin_r, xa - FOOT_HX + 0.01, j.FRONT - j.EAR_T, j.FRONT, -4.4, 4.4)
    tab += cyl_y(4.4, j.FRONT - j.EAR_T, j.FRONT, xa - pin_r, 0.0)
    body = solid(j.drum_shell(NECK_END) + tube + foot + ears + tab)
    # ---- 속: 목은 관 높이까지 관 속으로 이어지고, 그 위(서보 뒷면 쪽)는 목 끝 벽에서 막힌다
    inner = j.drum_cavity(NECK_END, roof_y=y_top + WALL, roof_x=NECK_END + WALL)
    # 관 속은 누워서 뽑히므로 윗부분을 45° 면 둘과 평평한 다리로 잇는다 (둥근 천장은 처진다)
    r_in = tube_r - WALL
    u45 = r_in / math.sqrt(2)
    u_top = r_in * 0.91
    v_top = r_in * math.sqrt(2) - u_top
    x0t, x1t = xa + FOOT_HX - 4, NECK_END + 6
    tube_in = cyl_x(r_in, x0t, x1t, yc, 0.0) & box(xa, 0, yc - u45, j.FRONT - WALL, -tube_r, tube_r)
    roof = [
        (yc - u45 + 0.01, u45),
        (yc - u_top, v_top),
        (yc - u_top, -v_top),
        (yc - u45 + 0.01, -u45),
    ]
    tube_in += j.prism_x(roof, x0t, x1t)
    foot_in = box(xa - FOOT_HX + WALL, xa + FOOT_HX + 2, y_top + WALL, j.FRONT - WALL, -hw, hw)
    foot_in -= _chamfer_x(xa - FOOT_HX, xa + FOOT_HX + 3, y_top + WALL, hw, 6.5)
    h = solid(body - solid(inner + tube_in + foot_in))
    # ---- 발: 혼 볼트 머리 위 구멍, 몸통 나사 구멍, 영점 핀 구멍, 선이 들어오는 홈
    h = solid(h - cyl_y(11.0, j.FRONT - WALL - 1, j.FRONT + 1, xa, 0.0))
    for dx, z in j.RIM:
        h = solid(h - cyl_y(j.M3_FREE / 2, j.FRONT - j.EAR_T - 1, j.FRONT + 1, xa + dx, z))
    h = solid(h - cyl_y(j.PIN_HOLE / 2, j.FRONT - j.EAR_T - 1, j.FRONT + 1, xa - pin_r, 0.0))
    h = solid(
        h - box(xa - FOOT_HX - 1, xa - FOOT_HX + WALL + 1, j.FRONT - 2.4, j.FRONT + 1, 6.5, 11.5)
    )
    return j.cut_joint_features(h, window=True)


def link_cover() -> Compound:
    """링크 끝 집의 뒷덮개 (검정). 목이 관으로 내려가는 자리까지 덮는다."""
    return j.back_cover(NECK_END, end_skirt=True)


def fits_bed(shape: Compound) -> tuple[bool, float]:
    """출력 판(256 × 256)에 들어가는지와, 대각선으로 놓았을 때 필요한 한 변 [mm]."""
    bb = shape.bounding_box()
    diagonal = (bb.size.X + bb.size.Z) / math.sqrt(2)
    return max(bb.size.X, bb.size.Z) <= BED or diagonal <= BED, diagonal


def report() -> None:
    up = (0.0, -1.0, 0.0)
    for name, (length, tube_r, pin_r) in LINKS.items():
        part = link(length, tube_r, pin_r)
        bb = part.bounding_box()
        c = part.center()
        over = overhangs(part, up)
        ok, diagonal = fits_bed(part)
        print(f"{name}: 길이 {length:.0f}, 관 Ø{2 * tube_r:.0f}")
        print(
            f"  크기 {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm, 무게 {printed_mass_g(part, up):.1f} g, "
            f"부피 중심 x {c.X:.1f} y {c.Y:.1f}"
        )
        fit = (
            "곧게 들어간다" if bb.size.X <= BED else f"대각선으로 놓는다 (한 변 {diagonal:.0f} mm)"
        )
        print(f"  출력 판: {fit if ok else '들어가지 않는다'}")
        print(
            f"  걸침 {over.total:.0f} mm² (거의 수평 {over.flat:.0f}, 가장 넓은 한 자리 {over.largest:.0f})"
        )
        for a, hgt, u, v, flat in over.spots[:6]:
            print(
                f"    높이 {hgt:4.0f}  자리 ({u:6.1f}, {v:5.1f})  넓이 {a:6.1f}  {'수평' if flat else '비스듬'}"
            )


if __name__ == "__main__":
    report()
