"""J1 관절: 베이스 몸통과 돌림판, 그 위에 앉는 J1 몸통의 발 (0028의 J1, 0029의 "J1 관절").

STS 관절(parts/joint.py)을 세워 놓고 베어링을 둘로 늘린 짜임이다. 서보, 누름 틀, 서보 받침은 STS 관절과 같다.
다른 점:
- 6806 둘을 가운데 사이 30 mm로 띄운다. 돌림판의 저널 하나가 안쪽 링 둘을 지나서, 두 링이 한 원기둥에 앉는다.
- 허브는 혼에 짧은 나사 넷으로 먼저 달고(책상 위에서), 돌림판 위에서 내려온 긴 볼트 넷을 허브 옆 홈의 너트에
  조인다. 볼트는 돌림판 단 - 위 안쪽 링 - 사이 고리 - 아래 안쪽 링 - 허브를 한 덩어리로 문다. 저널 끝은 허브에
  닿지 않는다. 혼의 얇은 나사산(2.5 mm)에 긴 볼트를 걸지 않고, 볼트 길이가 혼 높이에 매이지 않는다.
- 바깥 링은 위쪽이 몸통의 턱, 아래쪽이 누름 틀이다. 누름 틀을 밀어 붙인 채 벽 나사를 잠근다.
- 몸통은 두꺼운 원기둥이다 (무게가 문제 되지 않는 자리라 베어링 둘레를 두껍게 했다). 벽 나사는 굴 속에 앉는다.
- 기계 멈춤 기둥은 돌림판 밑면에, 홈과 살은 몸통 윗면에 있다. 영점 핀은 J1 몸통의 뒤쪽 귀에서 그 살에 꽂는다.
- J1 몸통은 돌림판에 축 가까이(반지름 21 mm)의 나사 넷으로 단다. 나사는 돌림판 밑에서 올라와 J1 몸통의 두꺼운
  바닥판 속 너트에 물린다 (책상 위에서 먼저 조립한다). 돌림판의 얇은 테가 팔의 기울이는 힘에 휘지 않게 하려는 것이다.
- 몸통 윗면과 J1 몸통 사이(돌림판 테 + 발 귀, 높이 12 mm, 반지름 39 mm)가 선이 감기는 자리다.

좌표(mm)는 joint.py의 서보 좌표다: y가 J1 축(위), x는 서보의 가까운 끝 쪽(로봇 앞), z = x × y (로봇 오른쪽).
서보 먼 끝(-x)이 로봇 뒤(책상 가장자리 쪽)를 본다. 책상 윗면은 y = Y_DESK다.
J1 몸통은 어깨 집(parts/shoulder.py)에 발을 단 한 부품이고, 어깨의 서보 좌표로 그린 뒤 as_child로 옮긴다.

책상에 무는 판(날개)과 선이 드나드는 자리는 책상 사진, 클램프 치수, 배선을 정한 뒤에 그린다 (PROVISIONAL).

    uv run python -m horilab_design.parts.base
"""

from __future__ import annotations

import math

from build123d import Compound, Cone, Location, Plane, Pos, Rot

from horilab_design.parts import joint as j
from horilab_design.parts import shoulder as sh
from horilab_design.parts.shapes import box, cyl_x, cyl_y, cyl_z, nut_trap, rod, solid, teardrop
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.values import ServoModel

# ---- 축 방향 쌓임 (혼 윗면 j.SEAT에서 위로) ----
LABEL_GAP = 1.2  # 허브 밑면과 서보 앞면 라벨 판 사이에 둘 틈
# 허브 밑면을 혼 바깥에서 들어 올리는 높이. 혼이 라벨 판보다 LABEL_GAP 넘게 높으면 0이다 (혼 높이 값에 달렸다)
HUB_RELIEF = max(0.0, LABEL_GAP - (j.SEAT - j.LABEL - 0.17))
HUB_BODY = 6.0  # 혼 바깥에서의 허브 두께: 너트 위 살 1.6 + 옆 홈 2.8 + 밑 살 1.6
HUB_T = HUB_BODY + HUB_RELIEF  # 혼 윗면 ~ 아래 베어링 밑면
SPAN = 30.0  # 두 베어링의 가운데 사이 (0028)
SPACER = SPAN - j.BRG_B  # 사이 고리의 길이 23
B0 = j.SEAT + HUB_T  # 아래 베어링 밑면 = 허브 윗면
B1 = B0 + j.BRG_B + SPACER  # 위 베어링 밑면
FRONT = B1 + j.BRG_B + j.LIP_T  # 몸통 윗면 (뽑을 때 바닥)
SEAM = FRONT + j.GAP  # 돌림판 테의 밑면
WASHER_T = (
    0.5  # 조임 볼트 머리 밑의 평와셔 (M3 × 8 × 0.5): 머리가 PLA를 누르는 넓이를 2.4배로 넓힌다
)
# 돌림판 테. 와셔를 낀 M3 × 50 조임 볼트가 허브의 너트를 0.5 지나가는 두께다
# (2.7 ~ 4.7이면 볼트가 너트에 다 물리고 끝이 허브 속에서 끝난다)
BRIM_T = 4.2
ATOP = SEAM + BRIM_T  # 돌림판 윗면 = J1 몸통이 앉는 면
JOURNAL_GAP = 0.6  # 저널 끝과 허브 사이. 베어링 폭과 출력 오차를 다 더해도 닿지 않는다
PLATE_T = 4.0  # 몸통 밑에 책상에 무는 판이 들어갈 두께 (임시)
Y_FLOOR = j.WIRE_TOP - j.COVER_GAP  # 몸통 아래 끝 = 판 윗면. 꽂아 눕힌 선과 1.0 띄운다
Y_DESK = Y_FLOOR - PLATE_T  # 책상 윗면
TOP_H = ATOP - Y_DESK  # 돌림판 윗면의 높이 (책상에서)
SHOULDER_H = 160.0  # 어깨 축 높이 (0017의 골격)
NECK_X = -(SHOULDER_H - TOP_H)  # 어깨 좌표에서 J1 몸통의 발 바닥 (목의 아래 끝)

# ---- 반지름 ----
R_CORE = 40.5  # 몸통 바깥. 서보 먼 끝 모서리(37.2) 밖으로 벽 2.4가 남는다
R_TABLE = 39.0  # 돌림판 테와 J1 몸통 발의 바깥 = 선이 감기는 반지름
R_CLAMP = 11.6  # 조임 볼트 넷 (저널 벽 1.75가 남는다)
R_HORN = 7.0  # 혼 나사 넷 (혼의 M3 나사 구멍)
HEAD_RECESS = 1.5  # 허브 윗면의 혼 나사 머리 자리 (머리가 0.15 나오고, 저널 끝과는 0.45 뜬다)
HORN_SCREW = 8.0  # 혼 나사 M3 × 8
HORN_FLANGE = 2.5  # 혼의 테 두께 (M3 나사 구멍이 뚫려 있다, docs/hardware.md)
NUT_ROOF = 1.6  # 허브 옆 홈의 너트 위에 남는 살
SLOT_W, SLOT_H = 5.6, 2.8  # 옆에서 밀어 넣는 너트 홈 (임시: 흰색 값. 검정 시험편 값으로 바꾼다)
CLAMP_LEN = 50.0  # 조임 볼트 M3 × 50 (살 것)
SPOT_R = j.SPOT_R  # 벽 나사 머리가 앉는 면. STS 관절과 같아서 누름 틀과 서보 받침을 그대로 쓴다
SPACER_R = j.R_BORE - 0.5  # 사이 고리의 몸통: 저널이 들어오기 전에 구멍 가운데 가까이 있게 한다
JOURNAL_TIP = 0.8  # 저널 끝의 모따기 (사이 고리가 0.5 비켜 있어도 들어간다)
SPACER_BORE = j.BRG_D / 2 + 0.15

# ---- 기계 멈춤, 영점 핀, J1 몸통 나사 ----
R_POST = 33.0  # 멈춤 기둥 (돌림판 밑면, 앞쪽)
POST_TIP = SEAM - 4.5
CH_R = (R_POST - 4.4, R_POST + 4.4)  # 몸통 윗면의 홈
CH_Y = FRONT - 7.0  # 홈 바닥
WEB_HALF = 180.0 - j.STOP_DEG - math.degrees(math.asin(j.BOSS_R / R_POST))  # 살의 반각 11.3°
R_PIN = 33.0  # 영점 핀 (뒤쪽 살의 가운데)
# J1 몸통 나사 M3 × 12 넷 (x, z). 돌림판의 단(반지름 16.5)에서 4.5 mm 밖이라 테가 휘는 길이가 짧다.
# 가장자리(반지름 35)에 두면 테가 팔의 기울이는 힘(약 3 N·m)에 0.05° 넘게 휜다 (어림)
R_FOOT = 21.0
FOOT = tuple(j.polar(R_FOOT, a) for a in (45.0, 135.0, 225.0, 315.0))
FOOT_SCREW = 12.0
FOOT_RECESS = 1.8  # 돌림판 밑면의 나사 머리 자리
FOOT_NUT = 2.6  # J1 몸통 바닥판 윗면의 너트 자리 깊이
# J1 몸통의 바닥판 두께 (목 안쪽). 돌림판 밑에서 올라온 M3 × 12가 바닥판 속 너트에 다 물린다.
# 목 앞뒤로 나온 귀는 j.EAR_T(6.2)로 얇아서 영점 핀이 살 속에 4.6 들어간다
FLOOR_T = 9.8
POCKET_TOP = j.LABEL + 1.4  # 서보 먼 끝 자리의 천장
FAR_GAP = 1.2  # 서보 먼 끝(눕힌 선은 0.9 더 나온다)과 벽 사이
WIRE_EXIT = (
    6.0,
    7.0,
)  # 몸통 뒤 아래의 선 구멍: 반폭, 높이 (플러그 9.6 × 5.4가 지나간다. 자리는 임시)
FLARE = 1.3  # 돌림판 테의 아래 가장자리를 45°로 넓힌 턱: 감긴 선이 몸통과의 틈으로 빠지지 않는다
STRING_D = 1.0  # 돌림판 테 둘레의 스프링 줄 홈 깊이 (V 모양이라 뽑을 때 걸치지 않는다)
STRING_Y = SEAM + 2.5  # 줄 홈의 가운데 높이
M2_TAP = 1.7  # 줄을 묶는 M2 나사를 조여 넣는 구멍 (임시: 시험편 값이 없다)
FOOT_HOLE = 16.0  # J1 몸통 바닥판 가운데 구멍의 반지름 (조임 볼트 머리가 들어온다)
STRING_DEG = 10.0  # 줄 끝을 묶는 나사의 방위각 (앞쪽. 멈춤 기둥을 비킨다)

PROVISIONAL = (
    "PLATE_T (책상에 무는 판과 날개)",
    "SLOT_W, SLOT_H (검정 너트 홈 시험편)",
    "선이 몸통에서 나오는 자리와 J1 몸통으로 들어가는 홈 (배선을 그리며 정한다)",
    *j.PROVISIONAL[:2],
)


def clamp_xz() -> list[tuple[float, float]]:
    """조임 볼트 넷."""
    return [j.polar(R_CLAMP, a) for a in (0, 90, 180, 270)]


# ------------------------------------------------------------------------------------------------
# 베이스 몸통 (검정)


def _tunnel(axis: str, sign: int, y: float, half_y: float, half_w: float) -> Compound:
    """벽 나사 머리가 앉는 면에서 몸통 밖까지 뚫은 네모 굴 (나사와 렌치가 드나든다)."""
    lo, hi = SPOT_R, R_CORE + 1
    if axis == "x":
        return box(lo, hi, y - half_y, y + half_y, -half_w, half_w)
    z0, z1 = (lo, hi) if sign > 0 else (-hi, -lo)
    return box(-half_w, half_w, y - half_y, y + half_y, z0, z1)


def core() -> Compound:
    """베이스 몸통. 윗면(y = FRONT)을 바닥에 두고 뽑는다. 속은 아래(뽑을 때 위)로 열려 있다."""
    h = solid(Rot(0, 45, 0) * cyl_y(R_CORE, Y_FLOOR, FRONT))
    cav = cyl_y(j.R_BORE, Y_FLOOR - 1, B1 + j.BRG_B) + cyl_y(j.R_LIP, B1, FRONT + 1)
    cav += box(-(j.FAR + FAR_GAP), -12.0, Y_FLOOR - 1, POCKET_TOP, -j.NECK_IN, j.NECK_IN)
    h = solid(h - solid(cav))
    hw, ht = WIRE_EXIT
    h = solid(h - box(-R_CORE - 1, -j.FAR, Y_FLOOR - 1, Y_FLOOR + ht, -hw, hw))
    # 멈춤 홈: 윗면을 한 바퀴 돌고, 뒤쪽(180°)에 살만 남긴다
    band = j.ring(CH_R[1], CH_R[0], CH_Y, FRONT + 1)
    web = band & j.wedge(180 - WEB_HALF, 180 + WEB_HALF, CH_R[1] + 5, CH_Y - 1, FRONT + 2)
    h = solid(h - solid(band - web))
    h = solid(h - cyl_y(j.PIN_HOLE / 2, CH_Y - 2.0, FRONT + 1, -R_PIN, 0.0))
    # 벽 나사: 누름 틀 셋 (양옆과 앞), 서보 받침 둘 (양옆, 축 방향으로 긴 구멍)
    r = j.M3_FREE_H / 2
    for s in (1, -1):
        z0, z1 = (15.0, SPOT_R + 0.01) if s > 0 else (-SPOT_R - 0.01, -15.0)
        h = solid(h - cyl_z(r, z0, z1, 0.0, j.RET_SCREW_Y))
        h = solid(h - _tunnel("z", s, j.RET_SCREW_Y, 3.4, 3.4))
        slot = cyl_z(r, z0, z1, 0.0, j.CAR_SCREW_Y - 1.0) + cyl_z(
            r, z0, z1, 0.0, j.CAR_SCREW_Y + 1.0
        )
        slot += box(-r, r, j.CAR_SCREW_Y - 1.0, j.CAR_SCREW_Y + 1.0, z0, z1)
        h = solid(h - solid(slot))
        h = solid(h - _tunnel("z", s, j.CAR_SCREW_Y, 5.2, 4.2))
    h = solid(h - cyl_x(r, 15.0, SPOT_R + 0.01, j.RET_SCREW_Y, 0.0))
    h = solid(h - _tunnel("x", 1, j.RET_SCREW_Y, 3.4, 3.4))
    if not h.is_valid or h.volume < 1000:
        raise RuntimeError(f"베이스 몸통 형상이 깨졌다: 부피 {h.volume:.1f}")
    return h


# ------------------------------------------------------------------------------------------------
# 돌림판, 허브, 사이 고리 (검정, 서보와 함께 돈다)


def bearings() -> tuple[Compound, Compound]:
    lower = j.ring(j.BRG_OD / 2, j.BRG_D / 2, B0, B0 + j.BRG_B)
    upper = j.ring(j.BRG_OD / 2, j.BRG_D / 2, B1, B1 + j.BRG_B)
    return lower, upper


def hub() -> Compound:
    """허브: 혼 윗면에 M3 × 8 넷으로 달고, 넓은 윗면이 아래 안쪽 링을 받친다. 혼 쪽 면을 바닥에 두고 뽑는다.

    조임 볼트의 너트는 옆면에서 밀어 넣는다. 볼트 끝은 허브 속에서 끝나서 혼에 닿지 않는다.
    """
    h = solid(Rot(0, 22.5, 0) * cyl_y(j.R_STEP, j.SEAT, B0))
    if HUB_RELIEF > 0:  # 혼 바깥에서 밑면을 들어 라벨 판 위로 띄운다
        h = solid(h - j.ring(j.R_STEP + 1, j.HUB_SEAT_R, j.SEAT - 1, j.SEAT + HUB_RELIEF))
    h = solid(h - cyl_y(3.6, j.SEAT - 1, j.SEAT + 2.0))  # 혼 가운데 나사 머리
    for x, z in j.bolt_xz():  # 혼 나사 (45°부터)
        h = solid(h - cyl_y(j.M3_FREE / 2, j.SEAT - 1, B0 + 1, x, z))
        h = solid(h - cyl_y(3.1, B0 - HEAD_RECESS, B0 + 1, x, z))
    y_nut = B0 - NUT_ROOF - SLOT_H / 2
    for a, (x, z) in zip((0, 90, 180, 270), clamp_xz(), strict=True):
        h = solid(h - cyl_y(j.M3_FREE / 2, j.SEAT - 1, B0 + 1, x, z))
        out = (math.cos(math.radians(a)), 0.0, math.sin(math.radians(a)))
        trap = nut_trap((x, y_nut, z), (0.0, 1.0, 0.0), out, j.R_STEP - R_CLAMP + 1, SLOT_W, SLOT_H)
        h = solid(h - trap)
    return h


def spacer() -> Compound:
    """사이 고리: 두 안쪽 링 사이. 끝의 좁은 면(반지름 16.5까지)만 안쪽 링에 닿고, 몸통은 45°로 물러난다."""
    y0, y1 = B0 + j.BRG_B, B1
    c = 1.1 * (SPACER_R - j.R_STEP)  # 뽑을 때 45°보다 조금 서게
    s = cyl_y(j.R_STEP, y0, y1) + cyl_y(SPACER_R, y0 + c, y1 - c)
    lower = Pos(0, y0 + c / 2, 0) * (Rot(-90, 0, 0) * Cone(j.R_STEP, SPACER_R, c))
    upper = Pos(0, y1 - c / 2, 0) * (Rot(-90, 0, 0) * Cone(SPACER_R, j.R_STEP, c))
    s = solid(s + lower + upper)
    return solid(s - cyl_y(SPACER_BORE, y0 - 1, y1 + 1))


def turntable() -> Compound:
    """돌림판: 저널(안쪽 링 둘과 사이 고리 속) + 단(위 안쪽 링 윗면) + 테 + 멈춤 기둥. 테 윗면을 바닥에 두고 뽑는다.

    조임 볼트 머리는 테 윗면(바닥에 닿아 평평한 면)에 앉는다.
    """
    y_tip = B0 + JOURNAL_GAP
    a = cyl_y(j.R_JOURNAL, y_tip + JOURNAL_TIP, B1 + j.BRG_B + 0.01)
    a += Pos(0, y_tip + JOURNAL_TIP / 2, 0) * (
        Rot(-90, 0, 0) * Cone(j.R_JOURNAL - JOURNAL_TIP, j.R_JOURNAL, JOURNAL_TIP + 0.01)
    )
    a += cyl_y(j.R_STEP, B1 + j.BRG_B, SEAM + 0.01)
    brim = cyl_y(R_TABLE, SEAM, ATOP) + Pos(0, SEAM + FLARE / 2, 0) * (
        Rot(-90, 0, 0) * Cone(R_TABLE + FLARE, R_TABLE, FLARE)
    )
    a = solid(a + solid(Rot(0, 45, 0) * solid(brim)))
    a = solid(a + cyl_y(j.BOSS_R, POST_TIP, SEAM + 0.01, R_POST, 0.0))  # 멈춤 기둥 (속에 M3 나사)
    a = solid(a - cyl_y(j.M3_FREE / 2, POST_TIP - 1, ATOP + 1, R_POST, 0.0))
    a = solid(a - cyl_y(3.1, ATOP - 1.8, ATOP + 1, R_POST, 0.0))  # 기둥 나사 머리 자리
    for x, z in clamp_xz():
        a = solid(a - cyl_y(j.M3_FREE / 2, B0, ATOP + 1, x, z))
    for (
        x,
        z,
    ) in FOOT:  # J1 몸통 나사: 머리가 밑면의 자리에 앉는다 (뽑을 때 위라 자리 바닥이 평평하다)
        a = solid(a - cyl_y(j.M3_FREE / 2, SEAM - 1, ATOP + 1, x, z))
        a = solid(a - cyl_y(3.1, SEAM - 1, SEAM + FOOT_RECESS, x, z))
    a = solid(a - cyl_y(2.0, SEAM - 1, ATOP + 1, -R_PIN, 0.0))  # 영점 핀이 지나가는 구멍
    # 스프링 자리 (0011): 테 둘레의 줄 홈과, 줄 끝을 묶는 M2 나사 구멍 (옆에서 조여 넣는다). 스프링이나 추는
    # 다 만든 팔의 반복 정밀도가 모자랄 때만 단다
    d, hh = STRING_D, 1.1 * STRING_D  # 홈의 두 면을 45°보다 조금 세운다
    groove = j.ring(R_TABLE + 0.01, R_TABLE - d, STRING_Y - hh, STRING_Y + hh)
    groove -= Pos(0, STRING_Y - hh / 2, 0) * (Rot(-90, 0, 0) * Cone(R_TABLE, R_TABLE - d, hh))
    groove -= Pos(0, STRING_Y + hh / 2, 0) * (Rot(-90, 0, 0) * Cone(R_TABLE - d, R_TABLE, hh))
    a = solid(a - solid(Rot(0, 45, 0) * solid(groove)))
    sx, sz = j.polar(1.0, STRING_DEG)
    r0, r1 = R_TABLE - 5.5, R_TABLE + 1
    a = solid(a - rod((sx * r0, STRING_Y, sz * r0), (sx * r1, STRING_Y, sz * r1), M2_TAP / 2))
    if not a.is_valid:
        raise RuntimeError("돌림판 형상이 깨졌다")
    return a


def retainer() -> Compound:
    """누름 틀: STS 관절의 것에서 고리만 길다 (허브가 두껍다)."""
    return j.retainer(B0)


# ------------------------------------------------------------------------------------------------
# J1 몸통 (흰색): 어깨 집 + 목 + 돌림판에 앉는 발. 어깨의 서보 좌표로 그린다


def j1_body() -> Compound:
    """J1 몸통. 어깨 원통 앞 끝(어깨 좌표의 y = FRONT)을 바닥에 두고 뽑는다.

    목의 아래 끝을 두꺼운 바닥판(FLOOR_T)이 막고, 목 앞뒤로 둥근 귀(돌림판과 같은 반지름, 두께 6.2)가 나온다.
    바닥판은 돌림판 밑에서 올라온 나사 넷으로 달리고, 뒤쪽 귀에 영점 핀 구멍이 있다.
    돌림판의 테와 귀의 둘레가 선이 감기는 면이다.
    """
    h = sh.house(NECK_X)
    x0 = NECK_X
    x_ear, x_floor = x0 + j.EAR_T, x0 + FLOOR_T
    lim = R_TABLE + 5
    foot = cyl_x(R_TABLE, x0, x_ear, sh.J1_AXIS_Y, 0.0) & box(
        x0 - 1, x_ear + 1, -lim, sh.FRONT, -lim, lim
    )
    wall = sh.NECK_HW - 0.01
    # 뽑을 때 귀의 아랫변이 벽에서 45°로 서게 깎는다 (원호는 벽 가까이에서 44°로 조금 눕는다)
    y_w = sh.J1_AXIS_Y + math.sqrt(R_TABLE**2 - sh.NECK_HW**2) - 0.3
    for sg in (1, -1):
        cut = j.prism_x(
            [
                (y_w, sg * wall),
                (y_w - 20.0, sg * (wall + 20.0)),
                (y_w + 25.0, sg * (wall + 20.0)),
                (y_w + 25.0, sg * wall),
            ],
            x0 - 1,
            x_ear + 1,
        )
        foot = solid(foot - cut)
    # 목 안쪽의 두꺼운 바닥판: 목의 단면 전체를 막는다 (뽑을 때 선 벽이다)
    foot = solid(foot + box(x0, x_floor, sh.BACK_END, sh.FRONT, -sh.NECK_HW, sh.NECK_HW))
    h = solid(h + foot)
    # 가운데 구멍: 누워서 뽑히는 큰 가로 구멍이라 위를 45° 지붕으로 낸다
    hole = teardrop(
        (x0 - 1, sh.J1_AXIS_Y, 0.0), (1.0, 0.0, 0.0), FLOOR_T + 2, 2 * FOOT_HOLE, (0.0, -1.0, 0.0)
    )
    h = solid(h - hole)
    for x, z in FOOT:  # 베이스의 (x, z) → 어깨의 (y, z) = (J1_AXIS_Y - z, -x)
        y_h, z_h = sh.J1_AXIS_Y - z, -x
        h = solid(h - cyl_x(j.M3_FREE_H / 2, x0 - 1, x_floor + 1, y_h, z_h))
        h = solid(h - j.hex_x(y_h, z_h, x_floor - FOOT_NUT, x_floor + 0.01, j.NUT_AF_UPRIGHT))
    h = solid(h - cyl_x(j.PIN_HOLE_H / 2, x0 - 1, x_ear + 1, sh.J1_AXIS_Y, R_PIN))
    if not h.is_valid:
        raise RuntimeError("J1 몸통 형상이 깨졌다")
    return h


def as_child(shape: Compound, q_deg: float = 0.0) -> Compound:
    """어깨의 서보 좌표로 그린 것을 베이스 좌표로 옮긴다 (J1 각 q).

    어깨의 x(위) → 베이스의 y, 어깨의 y(어깨 축, 출력 쪽) → 베이스의 -z(로봇 왼쪽), 어깨의 z → 베이스의 -x(뒤).
    """
    plane = Plane(
        origin=(0.0, Y_DESK + SHOULDER_H, sh.J1_AXIS_Y),
        x_dir=(0.0, 1.0, 0.0),
        z_dir=(-1.0, 0.0, 0.0),
    )
    return j.turned(solid(shape.moved(Location(plane))), q_deg)


# ------------------------------------------------------------------------------------------------
# 조립 검사에 쓰는 부품 목록


def _screw_down(x: float, z: float, seat: float, length: float, head: float = 1.65) -> Compound:
    """머리가 위(+y)에 있는 M3 볼트. 머리 밑면이 seat."""
    return solid(cyl_y(1.5, seat - length, seat, x, z) + cyl_y(2.85, seat, seat + head, x, z))


def hardware(model: ServoModel) -> list[j.Part]:
    """조임 볼트, 혼 나사, 너트, 벽 나사, 멈춤 나사, J1 몸통 나사, 영점 핀, 서보 뒷면 나사."""
    dy = j.servo_dy(model)
    out: list[j.Part] = []
    for k, (x, z) in enumerate(clamp_xz()):
        bolt = _screw_down(x, z, ATOP + WASHER_T, CLAMP_LEN)
        out.append(j.Part(f"조임 볼트 M3x50 {k + 1}", bolt, j.STEEL, True, "metal"))
        washer = j.ring(4.0, 1.6, ATOP, ATOP + WASHER_T)
        out.append(
            j.Part(
                f"와셔 (조임 볼트) {k + 1}", solid(Pos(x, 0, z) * washer), j.STEEL, True, "metal"
            )
        )
        nut_top = B0 - NUT_ROOF
        nut = j.hex_y(x, z, nut_top - j.NUT_T, nut_top, 5.4) - cyl_y(
            1.5, nut_top - 3, nut_top + 1, x, z
        )
        out.append(j.Part(f"너트 (허브) {k + 1}", solid(nut), j.STEEL, True, "metal"))
    for k, (x, z) in enumerate(j.bolt_xz()):
        seat = B0 - HEAD_RECESS
        out.append(
            j.Part(
                f"혼 나사 M3x8 {k + 1}", _screw_down(x, z, seat, HORN_SCREW), j.STEEL, True, "metal"
            )
        )
    out.append(
        j.Part(
            "멈춤 나사 M3x10", _screw_down(R_POST, 0.0, ATOP - 1.8, 10.0), j.STEEL, True, "metal"
        )
    )
    nut = j.hex_y(R_POST, 0.0, POST_TIP - j.NUT_T, POST_TIP, 5.4) - cyl_y(
        1.5, POST_TIP - 3, POST_TIP + 1, R_POST, 0.0
    )
    out.append(j.Part("너트 (멈춤 기둥)", solid(nut), j.STEEL, True, "metal"))
    top = ATOP + j.EAR_T
    seat = SEAM + FOOT_RECESS
    for k, (x, z) in enumerate(FOOT):  # 돌림판 밑에서 올라오는 나사와 바닥판 속 너트
        shape = cyl_y(1.5, seat, seat + FOOT_SCREW, x, z) + cyl_y(2.85, seat - 1.65, seat, x, z)
        out.append(j.Part(f"J1 몸통 나사 M3x12 {k + 1}", solid(shape), j.STEEL, True, "metal"))
        y0 = ATOP + FLOOR_T - FOOT_NUT
        nut = j.hex_y(x, z, y0, y0 + j.NUT_T, 5.4) - cyl_y(1.5, y0 - 1, y0 + j.NUT_T + 1, x, z)
        out.append(j.Part(f"너트 (J1 몸통) {k + 1}", solid(nut), j.STEEL, True, "metal"))
    pin = cyl_y(1.5, top + 3.0 - 20.0, top + 3.0, -R_PIN, 0.0)
    out.append(j.Part("영점 핀 MSH3-20", solid(pin), j.STEEL, False, "pin"))
    out += j.wall_screws()
    leg_in = j.SPOT_R - j.SCREW_LEN
    for sg in (1, -1):
        z0, z1 = (leg_in, leg_in + j.NUT_T) if sg > 0 else (-leg_in - j.NUT_T, -leg_in)
        nut_r = j.hex_z(0.0, j.RET_SCREW_Y, z0, z1, 5.4)
        out.append(j.Part("너트 (누름 틀)", nut_r, j.STEEL * 0.72, False, "metal"))
        b0, b1 = (
            (leg_in + 1.6, leg_in + 1.6 + j.NUT_T)
            if sg > 0
            else (-leg_in - 1.6 - j.NUT_T, -leg_in - 1.6)
        )
        nut_c = solid(Pos(0, dy, 0) * j.hex_z(0.0, j.CAR_SCREW_Y, b0, b1, 5.4))
        out.append(j.Part("너트 (서보 받침)", nut_c, j.STEEL * 0.72, False, "metal"))
    nut_x = j.hex_x(j.RET_SCREW_Y, 0.0, leg_in, leg_in + j.NUT_T, 5.4)
    out.append(j.Part("너트 (누름 틀)", nut_x, j.STEEL * 0.72, False, "metal"))
    for k, (x, z) in enumerate(j.BACK_HOLES):
        y = j.L1 - j.PLATE_T + dy
        shape = cyl_y(1.0, y, y + 6.0, x, z) + cyl_y(1.9, y - 2.0, y, x, z)
        out.append(j.Part(f"M2x6 {k + 1}", solid(shape), j.STEEL, False, "metal"))
    return out


def assembly(model: ServoModel = "STS3215") -> list[j.Part]:
    """J1 조립: 베이스 몸통과 속 부품, 돌림판, J1 몸통, 어깨 서보(틈 검사용), 나사와 핀."""
    dy = j.servo_dy(model)
    sv = j.servo(model)
    lower, upper = bearings()
    shoulder = sh.servo()
    parts = [
        j.Part("베이스 몸통", core(), color="cap"),
        j.Part("베어링 6806 (아래)", lower, color="metal"),
        j.Part("베어링 6806 (위)", upper, color="metal"),
        j.Part("사이 고리", spacer(), moving=True, color="frame"),
        j.Part("허브", hub(), moving=True, color="cap"),
        j.Part("돌림판", turntable(), moving=True, color="cap"),
        j.Part("누름 틀", retainer(), color="frame"),
        j.Part("서보 받침", solid(Pos(0, dy, 0) * j.carrier()), color="frame"),
        j.Part("서보 몸통", sv["body"], color="servo"),
        j.Part("혼", sv["horn"], moving=True, color="metal"),
        j.Part("플러그", sv["plugs"], color="plug"),
        j.Part("선 자리", sv["wires"], color="wire"),
        j.Part("J1 몸통", as_child(j1_body()), moving=True, color="body"),
        j.Part("어깨 서보", as_child(shoulder["body"]), moving=True, color="servo"),
    ]
    return parts + hardware(model)


# 서로 닿게 그린 쌍: 안쪽 링을 무는 부품들, 혼에 앉는 허브
_TOUCHING = {
    frozenset(p)
    for p in (
        ("서보 몸통", "혼"),
        ("플러그", "서보 몸통"),
        ("플러그", "선 자리"),
    )
}


def overlaps(parts: list[j.Part], tol: float = 0.05) -> dict[tuple[str, str], float]:
    """출력 부품과 서보끼리 겹친 부피 [mm³] (tol 넘는 것만). 나사와 너트는 구멍 속에 있어 보지 않는다."""
    solids = [p for p in parts if p.density == 0.0]
    out: dict[tuple[str, str], float] = {}
    for i, a in enumerate(solids):
        for b in solids[i + 1 :]:
            if frozenset((a.name, b.name)) in _TOUCHING:
                continue
            v = solid(a.shape & b.shape).volume if a.shape.distance_to(b.shape) < 1e-6 else 0.0
            if v > tol:
                out[(a.name, b.name)] = v
    return out


def clearances(
    parts: list[j.Part], q_deg: float, limit: float = 2.5
) -> dict[tuple[str, str], float]:
    """J1 각 q에서 도는 부품과 서 있는 부품 사이 틈 [mm] (limit 아래만). 영점 핀, 벽 나사, 베어링은 뺀다."""
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


def screw_checks(model: ServoModel) -> dict[str, float]:
    """나사가 물린 길이와 끝의 여유 [mm]."""
    dy = j.servo_dy(model)
    horn_tip = B0 - HEAD_RECESS - HORN_SCREW  # 혼 나사의 끝
    nut_top = B0 - NUT_ROOF
    clamp_tip = ATOP + WASHER_T - CLAMP_LEN
    foot_tip = SEAM + FOOT_RECESS + FOOT_SCREW
    foot_nut = ATOP + FLOOR_T - FOOT_NUT  # 바닥판 속 너트의 밑면
    return {
        "혼 나사가 혼 테(2.5)에 물린 길이": min(j.SEAT - horn_tip, HORN_FLANGE),
        "혼 나사 끝과 서보 앞면 사이": horn_tip - (j.F29 + dy),
        "조임 볼트가 너트(2.4)에 물린 길이": min(nut_top - clamp_tip, j.NUT_T),
        "조임 볼트 끝과 허브 밑면 사이": clamp_tip - (j.SEAT + HUB_RELIEF),
        "J1 몸통 나사가 너트(2.4)에 물린 길이": min(foot_tip - foot_nut, j.NUT_T),
        "J1 몸통 나사 끝과 바닥판 윗면 사이": (ATOP + FLOOR_T) - foot_tip,
        "J1 몸통 바닥판과 어깨 서보 먼 끝 사이": (Y_DESK + SHOULDER_H - sh.FAR) - (ATOP + FLOOR_T),
    }


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋). J1 몸통과 그 뒷덮개(어깨 원통의 뒷면)는 어깨의 서보 좌표다
def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {
        "base_core": (core(), (0.0, -1.0, 0.0), "black", "strength"),
        "base_turntable": (turntable(), (0.0, -1.0, 0.0), "black", "strength"),
        # 허브: 밑면이 평평하면 혼 쪽 면이 바닥이다. 혼 바깥을 들어 올린 모양이면 뒤집어 뽑는다
        "base_hub": (hub(), (0.0, 1.0 if HUB_RELIEF == 0 else -1.0, 0.0), "black", "strength"),
        "base_spacer": (spacer(), (0.0, 1.0, 0.0), "black", "strength"),
        "base_retainer": (retainer(), (0.0, -1.0, 0.0), "black", "strength"),
        "j1_body": (j1_body(), (0.0, -1.0, 0.0), "white", "strength"),
        "j1_cover": (sh.back_cover(NECK_X), (0.0, 1.0, 0.0), "black", "standard"),
    }


def report(model: ServoModel = "STS3215") -> None:
    parts = assembly(model)
    print(
        f"J1 관절 ({model}, 6806 둘): 책상 {Y_DESK:.2f}, 몸통 {Y_FLOOR:.2f} ~ {FRONT:.2f}, 베어링 {B0:.2f} ~ "
        f"{B0 + j.BRG_B:.2f} · {B1:.2f} ~ {B1 + j.BRG_B:.2f}, 돌림판 {SEAM:.2f} ~ {ATOP:.2f}"
    )
    print(
        f"책상에서: 몸통 윗면 {FRONT - Y_DESK:.1f}, 돌림판 윗면 {TOP_H:.1f}, 어깨 축 {SHOULDER_H:.0f} "
        f"(어깨 서보 먼 끝 {SHOULDER_H - sh.FAR:.1f}), 목 아래 끝 x {NECK_X:.1f}"
    )
    print(f"임시 값: {', '.join(PROVISIONAL)}")
    print("\n뽑는 부품 (무게는 뽑아서 잰 부품에 맞춘 어림, 걸침은 바닥에 닿지 않고 아래를 보는 면)")
    for name, (shape, up, color, preset) in printed().items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name:15s} {color:5s} {bb.size.X:5.1f} × {bb.size.Y:5.1f} × {bb.size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f}, "
            f"가장 넓은 한 자리 {over.largest:.0f})"
        )
    metal = sum(p.mass_g for p in parts if p.density > 0 and "핀" not in p.name)
    print(f"나사와 너트 {metal:.1f} g")
    for name, value in screw_checks(model).items():
        print(f"  {name}: {value:.2f}")
    hits = overlaps([p for p in parts if "핀" not in p.name])
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    print("도는 부품과 서 있는 부품 사이 틈 [mm] (2.5 아래만)")
    for q in (0.0, 90.0, j.SOFT_DEG, j.STOP_DEG, -90.0, -j.SOFT_DEG, -j.STOP_DEG):
        gaps = sorted((d, a, b) for (a, b), d in clearances(parts, q).items())
        print(f"  {q:7.1f}°: " + ", ".join(f"{a}↔{b} {d:.2f}" for d, a, b in gaps[:6]))
    body = next(p for p in parts if p.name == "베이스 몸통")
    arm = next(p for p in parts if p.name == "J1 몸통")
    pin = next(p for p in parts if "핀" in p.name)
    print(
        f"영점 핀 (J1 각 0°): 몸통과 틈 {pin.shape.distance_to(body.shape):.2f}, "
        f"J1 몸통과 틈 {pin.shape.distance_to(arm.shape):.2f} (구멍 틈 0.15 ~ 0.18)"
    )


if __name__ == "__main__":
    report()
