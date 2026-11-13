"""어깨 관절(J2, ST-3120)의 부품 (0029의 "어깨 관절").

STS 관절(parts/joint.py)과 같은 짜임이고, 다른 점은 이렇다:
- 베어링은 6808이고, 혼 볼트는 여덟이다 (혼 FT110M의 M3 나사 구멍, 지름 24 원).
- 누름 틀의 다리와 벽 나사는 서보 앞면과 베어링 사이에 있다. 서보 옆을 지나지 않는다.
- 서보 받침은 뒷면 M2.5 여섯에 달고 벽 나사 셋으로 잠근다. 커넥터가 옆면에 있어 뒷면에는 선이 없다.
- 집의 목은 J1 몸통이다. 옆면 커넥터의 플러그와 선이 들어가게 목이 원통만큼 넓고, 서보 먼 끝은 목 속의
  받침 턱 둘이 옆에서 잡는다 (서보가 미는 토크의 뒷받침). 돌림판에 앉는 바닥판까지 더한 J1 몸통은
  parts/base.py가 그린다.

좌표(mm): x는 출력축에서 서보의 가까운 끝 쪽(+15.9), y는 출력축 방향(혼 쪽이 +), z = x × y.
원점은 출력축 위, 몸체 두께(48)의 가운데다. 서보 먼 끝(-x)이 아래(J1 쪽)를 본다.
방위각은 x-z 평면에서 +x가 0°, +z가 90°다.

서보가 오기 전이라 서보 치수는 공식 도면 값이다 (docs/hardware.md의 "기구 치수"). PROVISIONAL의 값은 재서 바꾼다.

조립 순서: 베어링 → 누름 틀과 앞줄 벽 나사 셋 → 출력 원판(멈춤 나사와 너트를 먼저 조여 둔다, 앞에서)과 허브 판(뒤에서)
→ 선을 꽂고 받침을 단 서보 → 혼 볼트 여덟 → 뒷줄 벽 나사 셋(서보 잠금) → 뒷덮개 → 위팔.
옆면 커넥터는 집 속에서 꽂을 수 없다 (옆 틈 9.5 mm, 플러그 9.6 mm). 서보를 넣기 전에 꽂는다.

    uv run python -m horilab_design.parts.shoulder
"""

from __future__ import annotations

import math

from build123d import Compound, Cone, Pos, Rot

from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_x, cyl_y, cyl_z, solid
from horilab_design.printcheck import overhangs, printed_mass_g

# ---- 서보 ST-3120 (공식 도면) ----
NEAR, FAR = 15.9, 49.1
HW, HT = 15.0, 24.0  # 반폭(z), 반두께(y)
COLLAR_R, COLLAR_TOP = 8.5, HT + 0.9  # 출력축 둘레 턱
HORN_R, HORN_T, HUB_R = 14.95, 2.5, 7.0  # 혼 FT110M: 얇은 테와 허브
HORN_TOP = HT + 9.9  # 혼 윗면 (임시: 도면으로 9.9 ~ 10.2)
HORN_SCREW = (2.9, 1.8)  # 혼 가운데 나사 머리: 반지름, 혼 윗면 위로 나온 높이 (임시)
BOSS_R, BOSS_TIP = 3.0, -HT - 4.4  # 뒷면 보스
BACK_HOLES = (
    (12.9, 12.0),
    (12.9, -12.0),
    (-22.1, 12.0),
    (-22.1, -12.0),
    (-46.1, 12.0),
    (-46.1, -12.0),
)  # 뒷면 M2.5 (x, z)
CONN_X, CONN_Y = (
    (-17.6, -7.3),
    (-20.0, -15.2),
)  # 옆면(z = ±15) 커넥터 (임시: 도면 그림에서 읽은 값, ±1)
PLUG_OUT = 4.2  # 꽂은 플러그가 옆면에서 나온 길이 (STS 서보에서 본 값)
WIRE_OUT = 9.2  # 플러그 끝에서 꺾여 내려가는 선의 바깥 (옆면에서)

# ---- 공차 (6808 자리는 시험편이 없어 6806의 임시 값과 같은 여유를 둔다) ----
BEARING_BORE = 52.1  # 6808 바깥 링 자리 (임시)
JOURNAL_D = 39.9  # 안쪽 링 속 저널 (임시)
M25_FREE = 2.9  # M2.5 볼트가 지나가는 구멍
M25_HEAD = 2.0  # M2.5 둥근머리의 높이
PROVISIONAL = (
    "HORN_TOP",
    "HORN_SCREW",
    "CONN_X, CONN_Y",
    "BEARING_BORE",
    "JOURNAL_D",
)

# ---- 베어링과 축 방향 쌓임 ----
BRG_D, BRG_OD, BRG_B = 40.0, 52.0, 7.0
SEAT = HORN_TOP  # 허브 판이 혼 윗면에 앉는 높이
SEAT_R0, SEAT_R1 = (
    7.3,
    14.4,
)  # 허브 판 돋움이 혼에 닿는 고리 (혼 앞면 가운데의 지름 14 자리를 피한다)
RELIEF = 1.4  # 돋움의 높이. 테(휘는 살)가 돋움 가장자리에서 시작하게 한다
# 허브 판의 테: 두께 1.4, 틈 0.2면 누르는 힘 140 ~ 190 N, 휨 응력 27 ~ 37 MPa다 (studies/clamp_plate.py)
FLANGE_T = 1.4
B0 = SEAT + RELIEF + FLANGE_T  # 베어링 뒷면
FRONT = B0 + BRG_B + j.LIP_T  # 원통 앞 끝 (집을 뽑을 때 바닥)
SEAM = FRONT + j.GAP  # 출력 원판 테의 밑면
ATOP = SEAM + j.BRIM_T  # 출력 원판 윗면 = 위팔이 앉는 면
# 위팔의 납작한 면(출력 원판 윗면)이 J1 축에서 떨어진 거리. 쌓임이 길어진 만큼 서보를 출력 반대쪽으로 물려서
# 이 값을 지킨다 (0029: 골격과 손목 오프셋이 그대로다). J1 축은 이 좌표에서 y = J1_AXIS_Y를 지난다
J1_PLANE = 41.5
J1_AXIS_Y = ATOP - J1_PLANE
R_BORE = BEARING_BORE / 2
WALL = 2.55
R_DRUM = R_BORE + WALL  # 원통 바깥 반지름 28.6
R_LIP = 24.5
R_STEP = 21.5  # 허브 판의 테와 출력 원판의 단: 안쪽 링 앞뒤 면만 누른다
R_JOURNAL = JOURNAL_D / 2
BOLT_R = 12.0  # 혼 볼트 여덟
HEAD_HOLE_R = 4.0  # 허브 판 가운데 구멍 (혼 가운데 나사 머리가 들어온다)
COVER_T = 2.0
# 뒷덮개 안쪽 면: 뒷면 보스 끝과 받침 나사 머리 중 더 나온 것에서 COVER_GAP
BACK_IN = min(BOSS_TIP, -HT - j.PLATE_T - M25_HEAD) - j.COVER_GAP
BACK_END = BACK_IN + 0.8  # 원통 뒤 끝. 덮개는 서보 받침을 따라 앉으므로 0.8 띄운다
COVER_SKIRT = 2.5
COVER_R = R_DRUM + 0.4 + 1.6
BACK_OUT = BACK_IN - COVER_T  # 뒷덮개 바깥 면

# ---- 원통 속 부품 ----
SPOT_R = R_DRUM - 0.6  # 벽 나사 머리가 앉는 면
RET_Y = (22.0, 32.6)  # 누름 틀 다리의 범위 (다리는 서보 옆면보다 5 mm 바깥이다)
RET_SCREW_Y = 27.3
CAR_Y1 = -4.0  # 서보 받침 날개의 끝
CAR_SCREW_Y = -10.0
CAR_SLOT = 1.5  # 받침 벽 나사 구멍의 반 길이 (혼 높이 9.9 ~ 10.2와 조립 오차)
LEAF_X = (-4.6, 12.0)  # 옆 날개의 x 범위 (옆면 커넥터의 플러그를 비킨다)
COVER_BOSS = ((0.0, 19.0), (0.0, -19.0))  # 뒷덮개 나사 자리 (x, z)

# ---- 목 (J1 몸통의 윗부분) ----
NECK_TOP_X = -8.0  # 목 상자가 원통 속에서 시작하는 x
NECK_IN = 24.5  # 목 안쪽 반폭: 옆면 15 + 플러그 4.2 + 꺾인 선 5 + 틈
NECK_HW = NECK_IN + 2.6  # 목 바깥 반폭 27.1
NECK_FRONT_IN = FRONT - 2.4  # 목 앞벽의 안쪽 면
# 목의 아래 끝. 어깨만 따로 볼 때의 값이다. 뽑는 부품은 목이 돌림판까지 내려오고 바닥판이 달린 J1 몸통이고,
# 그 길이는 J1 관절의 쌓임에서 나온다 (parts/base.py의 NECK_X와 j1_body)
NECK_X = -58.0
PAD_X = (-FAR, -36.0)  # 서보 먼 끝을 옆에서 잡는 턱
PAD_Y = (-8.0, HT)
PAD_GAP = 0.35

# ---- 기계 멈춤, 영점 핀, 위팔 나사 ----
R_POST = R_DRUM + 4.4  # 멈춤 기둥 중심 33.0
POST_TIP = SEAM - 4.5  # 기둥 돋움의 끝. 그 아래 너트 2.4
WEB_HALF = 180.0 - j.STOP_DEG - math.degrees(math.asin(j.BOSS_R / R_POST))  # 살의 반각 11.4°
CH_R1 = R_POST + 4.8  # 홈 바깥 반지름
CH_Y = FRONT - 7.0  # 홈 바닥. 멈춤 나사 끝과 1.0 띄운다
CH_SPAN = 80.0  # 홈이 살에서 양쪽으로 가는 각 (목 폭 전체를 지난다)
TUB_Y = CH_Y - 1.6  # 홈 밑을 막는 살의 밑면
TUB_X = -(CH_R1 + 1.8)  # 그 살이 끝나는 x
R_PIN = R_DRUM + 5.4  # 영점 핀 34.0 (위팔 발의 끝벽에서 0.7 바깥. STS 관절과 같은 관계다)
RIM_R = 24.4
RIM = (
    (0.0, RIM_R),
    (0.0, -RIM_R),
    *(
        (sx * RIM_R * math.sin(math.radians(26.0)), sz * RIM_R * math.cos(math.radians(26.0)))
        for sx in (1, -1)
        for sz in (1, -1)
    ),
)  # 위팔 나사 M3 × 10 여섯 (x, z): 마찰로 버티는 토크가 서보가 끝까지 미는 토크의 2배가 넘게
WIN_X = (-51.0, -41.4)  # 목 앞면의 선 창
WIN_Z = 3.0


def bolt_xz() -> list[tuple[float, float]]:
    """혼 볼트 여덟 (혼의 M3 나사 구멍). 원기둥 면의 이음선(0°)을 피해 22.5°부터 둔다."""
    return [j.polar(BOLT_R, 22.5 + 45.0 * k) for k in range(8)]


# ------------------------------------------------------------------------------------------------
# 검사용 서보 외형 (출력하지 않는다)


def servo() -> dict[str, Compound]:
    """서보 몸통, 혼, 옆면(+z)에 꽂은 플러그와 아래로 꺾인 선이 차지하는 자리."""
    body = box(-FAR, NEAR, -HT, HT, -HW, HW)
    body += cyl_y(COLLAR_R, HT - 0.01, COLLAR_TOP)
    body += cyl_y(BOSS_R, BOSS_TIP, -HT + 0.01)
    horn = cyl_y(HUB_R, COLLAR_TOP + 0.3, HORN_TOP - HORN_T + 0.01)
    horn += cyl_y(HORN_R, HORN_TOP - HORN_T, HORN_TOP)
    horn += cyl_y(HORN_SCREW[0], HORN_TOP - 0.01, HORN_TOP + HORN_SCREW[1])
    plug = box(CONN_X[0], CONN_X[1], CONN_Y[0], CONN_Y[1], HW - 0.5, HW + PLUG_OUT)
    wire = box(CONN_X[0] - 14.0, CONN_X[1], CONN_Y[0], CONN_Y[1], HW + PLUG_OUT, HW + WIRE_OUT)
    return {"body": solid(body), "horn": solid(horn), "plug": solid(plug), "wire": solid(wire)}


# ------------------------------------------------------------------------------------------------
# 집 (J1 몸통의 윗부분: 원통과 목)


def house_shell(neck_x: float = NECK_X) -> Compound:
    return solid(
        cyl_y(R_DRUM, BACK_END, FRONT) + box(neck_x, NECK_TOP_X, BACK_END, FRONT, -NECK_HW, NECK_HW)
    )


def house_cavity(neck_x: float = NECK_X) -> Compound:
    """원통과 목의 속. 서보 앞면 위(TUB_Y부터)는 멈춤 홈 밑을 막는 살로 채우고 x < TUB_X만 비운다."""
    c = cyl_y(R_BORE, BACK_END - 1, B0 + BRG_B) + cyl_y(R_LIP, B0, FRONT + 1)
    c += box(neck_x - 1, NECK_TOP_X, BACK_END - 1, TUB_Y, -NECK_IN, NECK_IN)
    c += box(neck_x - 1, TUB_X, TUB_Y - 0.5, NECK_FRONT_IN, -NECK_IN, NECK_IN)
    return solid(c)


def _pads() -> Compound:
    """서보 먼 끝을 옆에서 잡는 턱 둘. 앞쪽(+y, 뽑을 때 아래)은 벽에서 45°로 선다."""
    z_in, z_wall = HW + PAD_GAP, NECK_IN + 0.01
    out = None
    for sg in (1, -1):
        pts = [
            (PAD_Y[0], sg * z_wall),
            (PAD_Y[0], sg * z_in),
            (PAD_Y[1], sg * z_in),
            (PAD_Y[1] + (z_wall - z_in), sg * z_wall),
        ]
        p = j.prism_x(pts, PAD_X[0], PAD_X[1])
        out = p if out is None else solid(out + p)
    assert out is not None
    return out


def cut_features(h: Compound) -> Compound:
    """집에 멈춤 홈과 살, 영점 핀 구멍, 선 창, 벽 나사 구멍을 낸다. 불 연산은 하나씩 한다 (joint.py와 같다)."""
    h = solid(h - j.sector(R_DRUM + 0.4, CH_R1, 180 + WEB_HALF, 180 + CH_SPAN, CH_Y, FRONT + 1))
    h = solid(h - j.sector(R_DRUM + 0.4, CH_R1, 180 - CH_SPAN, 180 - WEB_HALF, CH_Y, FRONT + 1))
    h = solid(h - cyl_y(j.PIN_HOLE / 2, TUB_Y - 1, FRONT + 1, -R_PIN, 0.0))
    h = solid(h - box(WIN_X[0], WIN_X[1], NECK_FRONT_IN - 1, FRONT + 1, -WIN_Z, WIN_Z))
    r = j.M3_FREE_H / 2
    lo, hi = 18.0, R_DRUM + 1
    for s in (1, -1):
        z0, z1 = (lo, hi) if s > 0 else (-hi, -lo)
        zs0, zs1 = (SPOT_R, hi) if s > 0 else (-hi, -SPOT_R)
        h = solid(h - cyl_z(r, z0, z1, 0.0, RET_SCREW_Y))
        h = solid(h - box(-3.4, 3.4, RET_SCREW_Y - 3.4, RET_SCREW_Y + 3.4, zs0, zs1))
        # 서보 받침 나사 구멍은 축 방향으로 길다 (서보가 찾은 자리 그대로 잠근다)
        slot = cyl_z(r, z0, z1, 0.0, CAR_SCREW_Y - CAR_SLOT)
        slot += cyl_z(r, z0, z1, 0.0, CAR_SCREW_Y + CAR_SLOT)
        slot += box(-r, r, CAR_SCREW_Y - CAR_SLOT, CAR_SCREW_Y + CAR_SLOT, z0, z1)
        h = solid(h - solid(slot))
        spot = CAR_SLOT + 4.2
        h = solid(h - box(-4.2, 4.2, CAR_SCREW_Y - spot, CAR_SCREW_Y + spot, zs0, zs1))
    h = solid(h - cyl_x(r, lo, hi, RET_SCREW_Y, 0.0))
    h = solid(h - box(SPOT_R, hi, RET_SCREW_Y - 3.4, RET_SCREW_Y + 3.4, -3.4, 3.4))
    slot = cyl_x(r, lo, hi, CAR_SCREW_Y - CAR_SLOT, 0.0) + cyl_x(
        r, lo, hi, CAR_SCREW_Y + CAR_SLOT, 0.0
    )
    slot += box(lo, hi, CAR_SCREW_Y - CAR_SLOT, CAR_SCREW_Y + CAR_SLOT, -r, r)
    h = solid(h - solid(slot))
    spot = CAR_SLOT + 4.2
    h = solid(h - box(SPOT_R, hi, CAR_SCREW_Y - spot, CAR_SCREW_Y + spot, -4.2, 4.2))
    if not h.is_valid or h.volume < 1000:
        raise RuntimeError(f"집 형상이 깨졌다: 부피 {h.volume:.1f}")
    return h


def house(neck_x: float = NECK_X) -> Compound:
    """어깨 집 (흰색, J1 몸통의 윗부분). 원통 앞 끝(y = FRONT)을 바닥에 두고 뽑는다."""
    h = solid(house_shell(neck_x) - house_cavity(neck_x))
    h = solid(h + _pads())
    return cut_features(h)


# ------------------------------------------------------------------------------------------------
# 원통 속 부품 (검정)


def bearing() -> Compound:
    return j.ring(BRG_OD / 2, BRG_D / 2, B0, B0 + BRG_B)


def hub_plate() -> Compound:
    """허브 판: 돋움이 혼 윗면에 앉고, 넓은 테가 안쪽 링 뒤를 받친다. 테 윗면을 바닥에 두고 뽑는다."""
    h = j.ring(SEAT_R1, SEAT_R0, SEAT, SEAT + RELIEF + 0.01)
    h += j.ring(R_STEP, HEAD_HOLE_R, SEAT + RELIEF, B0)
    for x, z in bolt_xz():
        h -= cyl_y(j.M3_FREE / 2, SEAT - 1, B0 + 1, x, z)
    return solid(h)


def out_disc() -> Compound:
    """출력 원판: 저널(안쪽 링 속) + 단(안쪽 링 앞면) + 테(원통 앞 끝 위) + 멈춤 기둥 귀.

    테 윗면을 바닥에 두고 뽑는다. 혼 볼트 머리는 그 면에 앉는다.
    """
    a = cyl_y(R_JOURNAL, B0 + j.CLAMP_GAP, B0 + BRG_B + 0.01)
    a += cyl_y(R_STEP, B0 + BRG_B, SEAM + 0.01)
    a += cyl_y(R_DRUM, SEAM, ATOP)
    # 선이 틈으로 빠지지 않게 막는 테두리 (joint.py의 출력 원판과 같다)
    a += j.ring(R_DRUM, R_DRUM - j.LIP_W, FRONT + j.LIP_GAP, SEAM + 0.01)
    a += box(R_DRUM - 3.6, R_POST, SEAM, ATOP, -4.6, 4.6) + cyl_y(4.6, SEAM, ATOP, R_POST, 0.0)
    a += cyl_y(j.BOSS_R, POST_TIP, SEAM + 0.01, R_POST, 0.0)  # 멈춤 기둥 (속에 M3 나사)
    a -= cyl_y(j.M3_FREE / 2, POST_TIP - 1, ATOP + 1, R_POST, 0.0)
    a -= cyl_y(3.1, ATOP - 1.8, ATOP + 1, R_POST, 0.0)  # 기둥 나사 머리 자리
    for x, z in bolt_xz():
        a -= cyl_y(j.M3_FREE / 2, B0, ATOP + 1, x, z)
    for x, z in RIM:
        a -= cyl_y(j.M3_FREE / 2, SEAM - 1, ATOP + 1, x, z)
        a -= j.hex_y(x, z, SEAM - 0.01, SEAM + j.NUT_T + 0.1)  # 밑면에서 끼우는 너트 홈
    return solid(a)


def retainer() -> Compound:
    """누름 틀: 바깥 링 뒷면을 받치는 고리와 다리 셋 (벽 나사의 너트를 품는다). 고리를 바닥에 두고 뽑는다."""
    r_out = R_BORE - 0.15
    r_ring = R_STEP + 1.3
    leg_in = SPOT_R - j.SCREW_LEN  # 나사 끝이 닿는 면 = 다리 안쪽 면 (20.0)
    part = solid(Rot(0, 45, 0) * j.ring(r_out, r_ring, RET_Y[1], B0))
    seg = solid(Rot(0, 45, 0) * cyl_y(r_out, RET_Y[0], RET_Y[1] + 0.01))
    legs = box(-8.0, 8.0, RET_Y[0], RET_Y[1] + 0.01, leg_in, 40)
    legs += box(-8.0, 8.0, RET_Y[0], RET_Y[1] + 0.01, -40, -leg_in)
    legs += box(leg_in, 40, RET_Y[0], RET_Y[1] + 0.01, -7.0, 7.0)
    part += seg & legs
    r = j.M3_FREE_H / 2
    for s in (1, -1):
        z0, z1 = (
            (leg_in - 0.01, leg_in + j.NUT_T + 0.3)
            if s > 0
            else (-leg_in - j.NUT_T - 0.3, -leg_in + 0.01)
        )
        part -= j.hex_z(0.0, RET_SCREW_Y, z0, z1, j.NUT_AF_UPRIGHT)
        part -= (
            cyl_z(r, 15, 32, 0.0, RET_SCREW_Y) if s > 0 else cyl_z(r, -32, -15, 0.0, RET_SCREW_Y)
        )
    part -= j.hex_x(RET_SCREW_Y, 0.0, leg_in - 0.01, leg_in + j.NUT_T + 0.3, j.NUT_AF_UPRIGHT)
    part -= cyl_x(r, 15, 32, RET_SCREW_Y, 0.0)
    # 다리가 고리 안쪽으로 걸치지 않게, 다리 밑을 45° 원뿔로 깎는다 (joint.py와 같다)
    y_lo, y_hi = RET_Y[1] - (r_ring - leg_in) - 0.3, RET_Y[1] + 0.05
    cone = Cone(leg_in - 0.3, r_ring + 0.05, y_hi - y_lo)
    cut = Pos(0, (y_lo + y_hi) / 2, 0) * (Rot(-90, 0, 0) * cone)
    return solid(solid(part) - cut)


def carrier() -> Compound:
    """서보 받침: 서보 뒷면 M2.5 여섯에 다는 판과 벽을 따라 선 날개 셋. 판 뒷면을 바닥에 두고 뽑는다."""
    y0, y1 = -HT - j.PLATE_T, -HT
    r_out = R_BORE - 0.15
    # 원기둥 면의 이음선(0°)을 날개와 나사 구멍 자리에서 45° 돌려 둔다 (이음선 위에서 자르면 형상이 깨진다)
    arc = solid(Rot(0, 45, 0) * cyl_y(r_out - 0.4, y0, y1))
    plate = box(-FAR + 0.6, NEAR - 0.6, y0, y1, -HW + 0.4, HW - 0.4)
    for sg in (1, -1):  # 옆 날개까지 가는 귀
        ear = box(
            LEAF_X[0] - 0.4,
            LEAF_X[1] + 0.4,
            y0,
            y1,
            min(sg * (HW - 1), sg * 40),
            max(sg * (HW - 1), sg * 40),
        )
        plate += ear & arc
    plate += box(NEAR - 1.0, 40, y0, y1, -8.0, 8.0) & arc  # 가까운 끝 쪽 날개까지 가는 귀
    part = plate
    leaf_in = r_out - j.LEAF_T
    boss_in = SPOT_R - j.SCREW_LEN + 1.6  # 너트 자리 입구: 나사 끝이 너트를 1.6 지나간다
    y_lo, y_hi = CAR_SCREW_Y - 4.6, CAR_SCREW_Y + 4.6
    dz = leaf_in - boss_in
    r = j.M3_FREE_H / 2
    leaf = solid(Rot(0, 45, 0) * j.ring(r_out, leaf_in, y1 - 0.01, CAR_Y1))
    boss = solid(Rot(0, 45, 0) * cyl_y(r_out, y_lo, y_hi))
    for sg in (1, -1):
        zs = (min(sg * 10, sg * 40), max(sg * 10, sg * 40))
        part += leaf & box(LEAF_X[0], LEAF_X[1], y1 - 0.01, CAR_Y1, *zs)
        zb = (min(sg * boss_in, sg * 40), max(sg * boss_in, sg * 40))
        part += boss & box(-4.6, 4.6, y_lo, y_hi, *zb)
        # 판 뒷면을 바닥에 두고 뽑을 때 돋움 밑이 걸치지 않게 45°로 받친다
        under = [
            (y_lo - dz, sg * leaf_in),
            (y_lo + 0.01, sg * leaf_in),
            (y_lo + 0.01, sg * boss_in),
        ]
        part += j.prism_x(under, -4.6, 4.6)
        z0, z1 = (
            (boss_in - 0.01, boss_in + j.NUT_T + 0.3)
            if sg > 0
            else (-boss_in - j.NUT_T - 0.3, -boss_in + 0.01)
        )
        part -= j.hex_z(0.0, CAR_SCREW_Y, z0, z1, j.NUT_AF_UPRIGHT)
        part -= (
            cyl_z(r, 15, 32, 0.0, CAR_SCREW_Y) if sg > 0 else cyl_z(r, -32, -15, 0.0, CAR_SCREW_Y)
        )
    # 가까운 끝(+x) 쪽 날개
    part += leaf & box(10, 40, y1 - 0.01, CAR_Y1, -8.0, 8.0)
    part += boss & box(boss_in, 40, y_lo, y_hi, -4.6, 4.6)
    under = [(y_lo - dz, leaf_in), (y_lo + 0.01, leaf_in), (y_lo + 0.01, boss_in)]
    part += solid(Rot(0, 90, 0) * j.prism_x(under, -4.6, 4.6))  # (y, z) 단면을 (y, x)로 돌린다
    part -= j.hex_x(CAR_SCREW_Y, 0.0, boss_in - 0.01, boss_in + j.NUT_T + 0.3, j.NUT_AF_UPRIGHT)
    part -= cyl_x(r, 15, 32, CAR_SCREW_Y, 0.0)
    for x, z in COVER_BOSS:  # 뒷덮개 나사가 물리는 기둥
        part += cyl_y(3.2, y1 - 0.01, y1 + 5.5, x, z)
        part -= cyl_y(j.M3_TAP / 2, y0 - 1, y1 + 5.5 + 1, x, z)
    part -= cyl_y(BOSS_R + 0.6, y0 - 1, y1 + 1)  # 뒷면 보스
    for x, z in BACK_HOLES:
        part -= cyl_y(M25_FREE / 2, y0 - 1, y1 + 1, x, z)
    return solid(part)


def back_cover(neck_x: float = NECK_X) -> Compound:
    """뒷덮개: 판 + 원통과 목의 뒤 끝을 겹쳐 가리는 테 + 서보 받침에 닿는 나사 기둥 둘. 바깥 면을 바닥에 두고 뽑는다."""
    y_skirt = BACK_END + COVER_SKIRT
    outline = cyl_y(COVER_R, BACK_OUT, y_skirt)
    outline += box(neck_x, NECK_TOP_X, BACK_OUT, y_skirt, -NECK_HW - 2.0, NECK_HW + 2.0)
    inner = cyl_y(R_DRUM + 0.4, BACK_IN, y_skirt + 1)
    inner += box(neck_x - 1, NECK_TOP_X, BACK_IN, y_skirt + 1, -NECK_HW - 0.4, NECK_HW + 0.4)
    c = solid(outline - inner)
    for x, z in COVER_BOSS:
        c = solid(c + cyl_y(3.2, BACK_IN - 0.01, -HT - j.PLATE_T, x, z))
        c = solid(c - cyl_y(j.M3_FREE_H / 2, BACK_OUT - 1, -HT - j.PLATE_T + 1, x, z))
        c = solid(c - cyl_y(3.1, BACK_OUT - 0.01, BACK_OUT + 1.2, x, z))  # 나사 머리 자리
    return c


# ------------------------------------------------------------------------------------------------
# 조립 검사에 쓰는 부품 목록


def _wall_screw(axis: str, s: int, y: float) -> Compound:
    """벽 나사 M3 × 8: 머리가 SPOT_R 면에 앉고 축 쪽으로 들어간다."""
    a, b = SPOT_R - j.SCREW_LEN, SPOT_R
    if axis == "x":
        return solid(cyl_x(1.5, a, b, y, 0.0) + cyl_x(2.85, b, b + 1.65, y, 0.0))
    if s > 0:
        return solid(cyl_z(1.5, a, b, 0.0, y) + cyl_z(2.85, b, b + 1.65, 0.0, y))
    return solid(cyl_z(1.5, -b, -a, 0.0, y) + cyl_z(2.85, -b - 1.65, -b, 0.0, y))


def hardware() -> list[j.Part]:
    """벽 나사, 혼 볼트, 멈춤 나사, 너트, 서보 뒷면 나사, 덮개 나사 (검사와 3D 페이지용)."""
    out: list[j.Part] = []
    for y, name in ((RET_SCREW_Y, "누름 틀"), (CAR_SCREW_Y, "서보 받침")):
        for k, (axis, s) in enumerate((("z", 1), ("z", -1), ("x", 1)), start=1):
            shape = _wall_screw(axis, s, y)
            out.append(j.Part(f"벽 나사 ({name}) {k}", shape, j.STEEL, color="metal"))
    for k, (x, z) in enumerate(bolt_xz()):
        shape = solid(cyl_y(1.5, ATOP - 20.0, ATOP, x, z) + cyl_y(2.85, ATOP, ATOP + 1.65, x, z))
        out.append(j.Part(f"혼 볼트 M3x20 {k + 1}", shape, j.STEEL, True, "metal"))
    seat = ATOP - 1.8
    stop = solid(
        cyl_y(1.5, seat - 10.0, seat, R_POST, 0.0) + cyl_y(2.85, seat, seat + 1.65, R_POST, 0.0)
    )
    out.append(j.Part("멈춤 나사 M3x10", stop, j.STEEL, True, "metal"))
    nut = j.hex_y(R_POST, 0.0, POST_TIP - j.NUT_T, POST_TIP, 5.4) - cyl_y(
        1.5, POST_TIP - 3, POST_TIP + 1, R_POST, 0.0
    )
    out.append(j.Part("너트 (멈춤 기둥)", solid(nut), j.STEEL, True, "metal"))
    for k, (x, z) in enumerate(RIM):
        nut = j.hex_y(x, z, SEAM, SEAM + j.NUT_T, 5.4) - cyl_y(
            1.5, SEAM - 1, SEAM + j.NUT_T + 1, x, z
        )
        out.append(j.Part(f"너트 (출력 원판) {k + 1}", solid(nut), j.STEEL, True, "metal"))
    y = -HT - j.PLATE_T
    for k, (x, z) in enumerate(BACK_HOLES):
        shape = cyl_y(1.25, y, y + 6.0, x, z) + cyl_y(2.25, y - M25_HEAD, y, x, z)
        out.append(j.Part(f"M2.5x6 {k + 1}", solid(shape), j.STEEL, False, "metal"))
    y = BACK_OUT + 1.2
    for k, (x, z) in enumerate(COVER_BOSS):
        shape = cyl_y(1.5, y, y + 12.0, x, z) + cyl_y(2.85, y - 1.65, y, x, z)
        out.append(j.Part(f"덮개 나사 M3x12 {k + 1}", solid(shape), j.STEEL, False, "metal"))
    leg_in = SPOT_R - j.SCREW_LEN
    for name, y, z_in in (
        ("누름 틀", RET_SCREW_Y, leg_in),
        ("서보 받침", CAR_SCREW_Y, leg_in + 1.6),
    ):
        for sg in (1, -1):
            z0, z1 = (z_in, z_in + j.NUT_T) if sg > 0 else (-z_in - j.NUT_T, -z_in)
            out.append(
                j.Part(
                    f"너트 ({name})", j.hex_z(0.0, y, z0, z1, 5.4), j.STEEL * 0.72, False, "metal"
                )
            )
        nut_x = j.hex_x(y, 0.0, z_in, z_in + j.NUT_T, 5.4)
        out.append(j.Part(f"너트 ({name})", nut_x, j.STEEL * 0.72, False, "metal"))
    return out


def inner_parts(neck_x: float = NECK_X) -> list[j.Part]:
    """집 속 부품과 서보, 출력 원판 (집과 위팔은 빼고)."""
    sv = servo()
    return [
        j.Part("베어링 6808", bearing(), color="metal"),
        j.Part("허브 판", hub_plate(), moving=True, color="cap"),
        j.Part("출력 원판", out_disc(), moving=True, color="cap"),
        j.Part("누름 틀", retainer(), color="frame"),
        j.Part("서보 받침", carrier(), color="frame"),
        j.Part("뒷덮개", back_cover(neck_x), color="cap"),
        j.Part("서보 몸통", sv["body"], color="servo"),
        j.Part("혼", sv["horn"], moving=True, color="metal"),
        j.Part("플러그", sv["plug"], color="plug"),
        j.Part("선 자리", sv["wire"], color="wire"),
    ]


def as_child(link_shape: Compound, length: float, q_deg: float = 0.0) -> Compound:
    """위팔(링크 좌표: 끝 관절의 서보 좌표, 앞 관절 축이 x = -length)을 어깨의 서보 좌표로 옮긴다.

    링크의 납작한 면(y = joint.FRONT)이 출력 원판 윗면(ATOP)에 앉고, 관절 각 0에서 위팔이 +x(위)를 본다.
    링크의 y는 어깨의 -y다 (x축 둘레 180°).
    """
    placed = Pos(length, ATOP + j.FRONT, 0) * (Rot(180, 0, 0) * link_shape)
    return j.turned(solid(placed), q_deg)


def child_hardware(length: float) -> list[j.Part]:
    """위팔 나사 M3 × 10 여섯과 영점 핀 (핀은 관절 각 0에 꽂는다)."""
    out: list[j.Part] = []
    top = ATOP + j.EAR_T
    for k, (x, z) in enumerate(RIM):
        s = cyl_y(1.5, top - j.BODY_SCREW, top, x, z) + cyl_y(2.85, top, top + 1.65, x, z)
        out.append(j.Part(f"위팔 나사 M3x10 {k + 1}", solid(s), j.STEEL, True, "metal"))
    pin = cyl_y(1.5, top + 3.0 - 20.0, top + 3.0, -R_PIN, 0.0)
    out.append(j.Part("영점 핀 MSH3-20", solid(pin), j.STEEL, False, "pin"))
    return out


def assembly(child: Compound, length: float, neck_x: float = NECK_X) -> list[j.Part]:
    """어깨 조립: 집과 속 부품, 출력 원판, 위팔, 나사와 핀."""
    parts = [j.Part("어깨 집", house(neck_x), color="body")]
    parts += inner_parts(neck_x)
    parts.append(j.Part("위팔", as_child(child, length), moving=True, color="body"))
    return parts + hardware() + child_hardware(length)


def clearances(
    parts: list[j.Part], q_deg: float, limit: float = 2.5
) -> dict[tuple[str, str], float]:
    """관절 각 q에서 도는 부품과 서 있는 부품 사이 틈 [mm] (limit 아래만). 영점 핀과 벽 나사는 뺀다."""
    static = [
        p
        for p in parts
        if not p.moving
        and "베어링" not in p.name
        and "핀" not in p.name
        and "벽 나사" not in p.name
    ]
    out: dict[tuple[str, str], float] = {}
    for a in (p for p in parts if p.moving):
        name = a.name.rstrip("0123456789 ").strip()
        shape = j.turned(a.shape, q_deg)
        for b in static:
            if b.name == "서보 몸통" and name.startswith("혼"):
                continue  # 혼과 서보는 한 몸이다
            d = shape.distance_to(b.shape)
            if d < limit:
                out[(name, b.name)] = min(d, out.get((name, b.name), limit))
    return out


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋).
# 집과 뒷덮개는 여기 없다. 뽑는 것은 목이 돌림판까지 내려오고 바닥판이 달린 J1 몸통과 그 뒷덮개다
# (parts/base.py의 printed: j1_body, j1_cover)
def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {
        "shoulder_out_disc": (out_disc(), (0.0, -1.0, 0.0), "black", "strength"),
        "shoulder_hub_plate": (hub_plate(), (0.0, -1.0, 0.0), "black", "strength"),
        "shoulder_retainer": (retainer(), (0.0, -1.0, 0.0), "black", "strength"),
        "shoulder_carrier": (carrier(), (0.0, 1.0, 0.0), "black", "strength"),
    }


def report() -> None:
    from horilab_design.parts import links  # 링크가 이 모듈의 발 치수를 쓰므로 여기서 부른다

    length, tube_r, foot = links.LINKS["upper_arm"]
    child = links.link(length, tube_r, foot)
    parts = assembly(child, length)
    print(
        f"어깨 관절 (ST-3120, 6808): 원통 지름 {2 * R_DRUM:.1f}, 뒷덮개 {BACK_OUT:.1f} ~ 원통 앞 끝 {FRONT:.1f} ~ "
        f"출력 원판 {SEAM:.1f} ~ {ATOP:.1f}, 베어링 {B0:.1f} ~ {B0 + BRG_B:.1f}, 길이 {ATOP - BACK_OUT:.1f}"
    )
    print(f"임시 값: {', '.join(PROVISIONAL)}")
    print("\n뽑는 부품 (무게는 뽑아서 잰 부품에 맞춘 어림, 걸침은 바닥에 닿지 않고 아래를 보는 면)")
    for name, (shape, up, color, preset) in printed().items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name:19s} {color:5s} {bb.size.X:5.1f} × {bb.size.Y:5.1f} × {bb.size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f}, "
            f"가장 넓은 한 자리 {over.largest:.0f})"
        )
    metal = sum(p.mass_g for p in parts if p.density > 0 and "핀" not in p.name)
    print(f"나사와 너트 {metal:.1f} g")
    hits = j.overlaps([p for p in parts if "핀" not in p.name])
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    print("도는 부품과 서 있는 부품 사이 틈 [mm] (2.5 아래만)")
    for q in (0.0, 90.0, j.SOFT_DEG, j.STOP_DEG, -90.0, -j.SOFT_DEG, -j.STOP_DEG):
        gaps = sorted((d, a, b) for (a, b), d in clearances(parts, q).items())
        print(f"  {q:7.1f}°: " + ", ".join(f"{a}↔{b} {d:.2f}" for d, a, b in gaps[:5]))
    body = next(p for p in parts if p.name == "어깨 집")
    arm = next(p for p in parts if p.name == "위팔")
    pin = next(p for p in parts if "핀" in p.name)
    print(
        f"영점 핀 (관절 각 0°): 집과 틈 {pin.shape.distance_to(body.shape):.2f}, "
        f"위팔과 틈 {pin.shape.distance_to(arm.shape):.2f} (구멍 틈 0.15 ~ 0.18)"
    )


if __name__ == "__main__":
    report()
