"""J1 관절: 베이스 몸통과 돌림판, 그 위에 앉는 J1 몸통의 발 (0028의 J1, 0029의 "J1 관절").

STS 관절(parts/joint.py)을 세워 놓고 베어링을 둘로 늘린 짜임이다. 서보, 누름 틀, 서보 받침은 STS 관절과 같다.
다른 점:
- 6806 둘을 가운데 사이 30 mm로 띄운다. 돌림판의 저널 하나가 안쪽 링 둘을 지나서, 두 링이 한 원기둥에 앉는다.
- 허브는 혼에 짧은 나사 넷으로 먼저 달고(책상 위에서), 돌림판에서 내려온 긴 볼트 넷(가진 M3 × 45)을 허브 옆
  홈의 너트에 조인다. 볼트는 돌림판 단 - 위 안쪽 링 - 사이 고리 - 아래 안쪽 링 - 허브를 한 덩어리로 문다. 저널
  끝은 허브에 닿지 않는다. 혼의 얇은 나사산(2.5 mm)에 긴 볼트를 걸지 않고, 볼트 길이가 혼 높이에 매이지 않는다.
  볼트 머리와 와셔는 돌림판 윗면에서 5.6 mm 내려간 자리에 앉는다 (위 안쪽 링의 윗면보다 2.4 mm 위).
- 바깥 링은 위쪽이 몸통의 턱, 아래쪽이 누름 틀이다. 누름 틀을 밀어 붙인 채 벽 나사를 잠근다.
- 몸통은 두꺼운 원기둥이다 (무게가 문제 되지 않는 자리라 베어링 둘레를 두껍게 했다). 벽 나사는 굴 속에 앉는다.
- 기계 멈춤 기둥은 돌림판 밑면에, 홈과 살은 몸통 윗면에 있다. 영점 핀은 J1 몸통의 뒤쪽 귀에서 그 살에 꽂는다.
- J1 몸통은 돌림판에 축 가까이(반지름 21 mm)의 나사 넷으로 단다. 나사는 돌림판 밑에서 올라와 J1 몸통의 두꺼운
  바닥판 속 너트에 물린다 (책상 위에서 먼저 조립한다). 돌림판의 얇은 테가 팔의 기울이는 힘에 휘지 않게 하려는 것이다.
- 선은 몸통 윗부분의 띠(높이 16 mm, 반지름 40.5 mm)에 감긴다 (0031). 몸통 쪽 누르개와 J1 몸통 뒤쪽 귀 위의
  누르개가 범위 가운데에서 같은 방위각(뒤에서 왼쪽으로 28°)에 온다.

좌표(mm)는 joint.py의 서보 좌표다: y가 J1 축(위), x는 서보의 가까운 끝 쪽(로봇 앞), z = x × y (로봇 오른쪽).
서보 먼 끝(-x)이 로봇 뒤(책상 가장자리 쪽)를 본다. 책상 윗면은 y = Y_DESK다.
J1 몸통은 어깨 집(parts/shoulder.py)에 발을 단 한 부품이고, 어깨의 서보 좌표로 그린 뒤 as_child로 옮긴다.

책상에 무는 판(날개)은 몸통 밑에 깔리고, 책상 뒤 가장자리에서 금속 클램프 둘로 문다. 책상 두께(40 mm)와 가장자리에서
J1 축까지(60 mm)는 문서의 값이다(0014, 0018). 금속 클램프는 다 벌린 틈을 쟀고 나머지 치수는 사진으로 어림했다
(hardware.md).

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
    0.5  # 조임 볼트 머리 밑의 평와셔 (M3 × 8 × 0.5): 머리가 PLA를 누르는 넓이를 2.7배로 넓힌다
)
# 돌림판 테. J1 몸통 나사의 머리 자리(1.8) 위로 살 2.4가 남는다. 두꺼워지면 돌림판 윗면이 올라가 어깨 서보
# 밑의 자리(5.0)가 준다
BRIM_T = 4.2
ATOP = SEAM + BRIM_T  # 돌림판 윗면 = J1 몸통이 앉는 면
JOURNAL_GAP = 0.6  # 저널 끝과 허브 사이. 베어링 폭과 출력 오차를 다 더해도 닿지 않는다
PLATE_T = 4.0  # 몸통 밑에 깔리는 판의 바닥 두께
Y_FLOOR = j.WIRE_TOP - j.COVER_GAP  # 몸통 아래 끝 = 판 윗면. 꽂아 눕힌 선과 1.0 띄운다
Y_DESK = Y_FLOOR - PLATE_T  # 책상 윗면
TOP_H = ATOP - Y_DESK  # 돌림판 윗면의 높이 (책상에서)
SHOULDER_H = 160.0  # 어깨 축 높이 (0017의 골격)
NECK_X = -(SHOULDER_H - TOP_H)  # 어깨 좌표에서 J1 몸통의 발 바닥 (목의 아래 끝)

# ---- 반지름 ----
R_CORE = 40.5  # 몸통 바깥. 서보 먼 끝 모서리(37.2) 밖으로 벽 2.4가 남는다
R_TABLE = 39.0  # 돌림판 테와 J1 몸통 발의 바깥 = 선이 감기는 반지름
R_CLAMP = 11.6  # 조임 볼트 넷 (저널 벽 1.75가 남고, 볼트가 혼의 바깥(반지름 9.98)을 지난다)
R_HORN = 7.0  # 혼 나사 넷 (혼의 M3 나사 구멍)
HEAD_RECESS = 1.5  # 허브 윗면의 혼 나사 머리 자리 (머리가 0.15 나오고, 저널 끝과는 0.45 뜬다)
HORN_SCREW = 8.0  # 혼 나사 M3 × 8
HORN_FLANGE = 2.5  # 혼의 테 두께 (M3 나사 구멍이 뚫려 있다, docs/hardware.md)
NUT_ROOF = 1.6  # 허브 옆 홈의 너트 위에 남는 살
SLOT_W, SLOT_H = 5.6, 2.8  # 옆에서 밀어 넣는 너트 홈 (임시: 흰색 값. 검정 시험편 값으로 바꾼다)
CLAMP_LEN = 45.0  # 조임 볼트: 가진 유두 렌치볼트 M3 × 45
CLAMP_HEAD_R, CLAMP_HEAD_H = 2.75, 3.0  # 그 머리 (지름 5.5, 높이 3.0, 2.5 mm 육각렌치)
# 조임 볼트의 머리 자리: 돌림판 윗면에서 SINK만큼 내려간 곳에 와셔가 앉는다. 볼트 끝이 허브의 너트를 1.1 지나고
# 허브 밑면보다 0.9 위에서 끝나는 깊이다 (4.5 ~ 6.5이면 너트에 다 물리고 끝이 허브 속에서 끝난다).
# 자리의 천장은 뽑을 때 허공에 걸친다 (0.2 mm 층으로 28층 위). 막 한 층(SKIN)이 볼트 구멍까지 덮어서 줄이
# 끊기지 않고 걸친다. 막은 조립 전에 볼트 구멍 자리만 머리 자리 쪽에서 뚫는다
SINK = 5.6
SINK_R = 4.3  # 머리 자리의 반지름: 와셔(지름 8)에 틈 0.3
SKIN = 0.2
# 머리 자리가 단 속으로 1.4 내려가므로, 그 둘레에 살 1.6이 남게 단을 넓힌다. 위 베어링 윗면에서 0.8 위부터라
# 베어링에 닿지 않고, 몸통의 턱 구멍(반지름 19.5)과는 2.0 뜬다
STEP_WIDE_R = 17.5
STEP_WIDE_UP = 0.8
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
)  # 몸통 뒤 아래의 선 구멍: 반폭, 높이 (플러그 9.6 × 5.4가 지나간다). J1 서보의 선 둘이 여기로 나온다
FLARE = 1.3  # 돌림판 테의 아래 가장자리를 45°로 넓힌 턱: 감긴 선이 몸통과의 틈으로 빠지지 않는다
STRING_D = 1.0  # 돌림판 테 둘레의 스프링 줄 홈 깊이 (V 모양이라 뽑을 때 걸치지 않는다)
STRING_Y = SEAM + 2.5  # 줄 홈의 가운데 높이
M2_TAP = 1.7  # 줄을 묶는 M2 나사를 조여 넣는 구멍 (임시: 시험편 값이 없다)
FOOT_HOLE = 16.0  # J1 몸통 바닥판 가운데 구멍의 반지름 (조임 볼트와 와셔를 넣고 렌치가 드나든다)
STRING_DEG = 10.0  # 줄 끝을 묶는 나사의 방위각 (앞쪽. 멈춤 기둥을 비킨다)

# ---- 선 (0031) ----
# J1을 넘는 선(서보 선 둘, D405 선)은 몸통 윗부분의 띠에 감긴다. 띠는 서 있는 몸통의 둥근 면이라, J1 몸통의
# 모서리(축에서 47.5 mm까지 나온다)와 상관없이 선이 한 원에 감긴다. 멈춤 기둥은 돌림판 밑에 숨어 있어 선과 만나지 않는다.
# 선은 몸통 쪽 누르개(방위각 WIRE_AZ)와 J1 몸통 뒤쪽 귀 위의 누르개 사이에서 감겼다 풀린다. 범위 가운데에서 두
# 누르개가 같은 방위각에 온다
BAND_H = 16.0  # 띠의 높이 (몸통 윗면에서 아래로)
BAND_LIP = 1.5  # 띠 위아래의 턱 (45°): 감긴 선이 띠 밖으로 넘어가지 않는다
WIRE_AZ = 208.0  # 선이 띠로 올라오는 방위각 (뒤에서 왼쪽으로 28°)
BAND_NOTCH = 8.0  # 아래 턱을 튼 자리의 반폭 (선이 밑에서 띠로 올라온다)
CLAMP_DROP = 6.0  # 아래 턱에서 몸통 쪽 누르개 나사 구멍까지
CLAMP_DEPTH = 8.0  # 누르개 나사(M3 × 8)를 조여 넣는 구멍의 깊이
EAR_CLAMP = (-31.5, -17.0)  # J1 몸통 뒤쪽 귀의 누르개 나사 자리 (x, z). 방위각이 WIRE_AZ다
TABLE_LIP = (
    38.6,
    39.8,
)  # 돌림판 밑의 테두리 (반지름 범위): 몸통 윗면과의 틈을 joint.LIP_GAP으로 좁힌다

# ---- 책상에 무는 판 ----
EDGE = 60.0  # 책상 뒤 가장자리에서 J1 축까지 (0018의 검토에 쓴 값)
DESK_T = 40.0  # 책상 상판 두께 (0014)
SEAT_GAP = 0.2  # 몸통과 판의 자리 사이
RING_H = 6.0  # 몸통 둘레의 턱과 날개가 바닥 위로 올라온 높이 (날개 두께는 바닥까지 10)
RING_W = 5.5  # 턱의 폭
WING_Z = 76.0  # 판의 반폭
WING_IN = 30.0  # 날개가 시작하는 |z| (그 사이는 선이 나가는 바닥이다)
WING_X = -15.0  # 날개의 앞 끝
# 클램프가 무는 자리: 날개 위에 비워 둔 폭 20 mm의 띠다 (가장자리에서 35 mm까지).
# 몸통(반지름 40.5)을 비키는 가장 가까운 자리라, 뒤쪽 판 나사와 클램프 사이가 짧다
CLAMP_Z = 50.0
CLAMP_HW = 10.0
CLAMP_IN = 25.0  # 책상 가장자리에서 띠의 가운데까지 (design.md: 2 ~ 3 cm 안쪽)
# 가진 금속 클램프 (다 벌린 틈은 캘리퍼스, 나머지는 사진으로 어림한 값, hardware.md). 등을 책상 가장자리에 붙이면
# 위 턱이 가장자리에서 JAW_REACH까지 날개를 누르고, 나사가 그 밑(JAW_AXIS)에서 받침으로 책상 밑면을 민다
JAW_REACH = 28.0  # 위 턱이 등 안쪽 면에서 나온 길이
JAW_AXIS = 22.0  # 등 안쪽 면에서 나사 축까지
JAW_HW = 5.0  # 각재의 반폭
JAW_T, SPINE_T = 7.0, 9.0  # 위 턱의 두께, 등과 아래 턱의 두께
PAD_R, PAD_T = 20.0, 10.7  # 나사 끝 받침의 반지름, 받침의 높이 (나사를 다 풀었을 때 아래 턱 위로)
CLAMP_OPEN = 55.88  # 금속 클램프를 다 벌린 틈 (2026-11-13 캘리퍼스, hardware.md)
R_PLATE = (
    33.0  # 몸통을 판에 묶는 나사 넷 (45° 대각선: 서보 먼 끝 자리와 선 구멍, 벽 나사 굴을 비킨다)
)
PLATE_SCREW = 10.0  # M3 × 10
PLATE_RECESS = 2.2  # 판 밑면의 머리 자리: 둥근머리 1.65 + 와셔 0.5 (머리가 밑면보다 0.05 안에 있다)
PLATE_NUT_UP = 4.2  # 몸통 밑면에서 너트 홈까지의 살
PLATE_HOLE = 8.7  # 몸통 밑면에서 나사 구멍의 깊이 (나사 끝 7.7에 1.0 여유)
ACC_X = (-28.0, -52.0)  # 날개 바깥 가장자리의 부속 자리 (스프링 기둥, 선 정리 부품): 날개마다 M3 둘
ACC_Z = 69.0
ACC_NUT = 2.6  # 그 밑면의 너트 자리 깊이

PROVISIONAL = (
    "SLOT_W, SLOT_H (검정 너트 홈 시험편)",
    "선 누르개의 모양 (쓸 선의 굵기가 정해진 뒤에 그린다. 나사 자리는 냈다)",
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
    # 책상에 무는 판의 나사 넷: 밑에서 올라와 옆에서 밀어 넣은 너트에 물린다
    y_nut = Y_FLOOR + PLATE_NUT_UP + SLOT_H / 2
    for a, (x, z) in zip((45.0, 135.0, 225.0, 315.0), plate_xz(), strict=True):
        h = solid(h - cyl_y(j.M3_FREE / 2, Y_FLOOR - 1, Y_FLOOR + PLATE_HOLE, x, z))
        out = (math.cos(math.radians(a)), 0.0, math.sin(math.radians(a)))
        trap = nut_trap((x, y_nut, z), (0.0, 1.0, 0.0), out, R_CORE - R_PLATE + 1, SLOT_W, SLOT_H)
        h = solid(h - trap)
    h = solid(h + _band_lips())
    # 몸통 쪽 누르개 나사 구멍 (아래 턱 밑, 옆에서 조여 넣는다)
    y_clamp = FRONT - BAND_H - CLAMP_DROP
    hole = cyl_x(j.M3_TAP / 2, R_CORE - CLAMP_DEPTH, R_CORE + 1, y_clamp, 0.0)
    h = solid(h - j.turned(solid(hole), WIRE_AZ))
    if not h.is_valid or h.volume < 1000:
        raise RuntimeError(f"베이스 몸통 형상이 깨졌다: 부피 {h.volume:.1f}")
    return h


def _band_lips() -> Compound:
    """선이 감기는 띠의 위아래 턱. 위 턱은 몸통 윗면(뽑을 때 바닥)에서 가장 넓고, 아래 턱은 양쪽이 45°보다 조금 선다.

    아래 턱은 WIRE_AZ에서 터 있다 (선이 밑에서 띠로 올라오는 자리).
    """
    lip, c = BAND_LIP, 1.1 * BAND_LIP
    top = Pos(0, FRONT - c / 2, 0) * (Rot(-90, 0, 0) * Cone(R_CORE - 0.01, R_CORE + lip, c))
    y_rib = FRONT - BAND_H
    rib = Pos(0, y_rib - c / 2, 0) * (Rot(-90, 0, 0) * Cone(R_CORE - 0.01, R_CORE + lip, c))
    rib += Pos(0, y_rib + c / 2, 0) * (Rot(-90, 0, 0) * Cone(R_CORE + lip, R_CORE - 0.01, c))
    notch = box(
        R_CORE - 1.0, R_CORE + lip + 1, y_rib - c - 1, y_rib + c + 1, -BAND_NOTCH, BAND_NOTCH
    )
    rib = solid(solid(Rot(0, 45, 0) * solid(rib)) - j.turned(notch, WIRE_AZ))
    # 원뿔의 안쪽은 몸통과 겹친다. 몸통 밖으로 나온 고리만 남긴다
    lips = solid(solid(Rot(0, 45, 0) * solid(top)) + rib)
    return solid(lips - cyl_y(R_CORE - 0.5, Y_FLOOR - 1, FRONT + 1))


# ------------------------------------------------------------------------------------------------
# 책상에 무는 판 (검정): 몸통이 앉는 자리 + 책상 뒤 가장자리까지 나온 날개 둘


def plate_xz() -> list[tuple[float, float]]:
    """몸통을 판에 묶는 나사 넷."""
    return [j.polar(R_PLATE, a) for a in (45.0, 135.0, 225.0, 315.0)]


def mount_xz() -> list[tuple[float, float]]:
    """날개 바깥 가장자리의 부속 자리 넷 (x, z)."""
    return [(x, s * ACC_Z) for s in (1, -1) for x in ACC_X]


def plate() -> Compound:
    """책상에 무는 판. 밑면(책상에 닿는 면)을 바닥에 두고 뽑는다.

    바닥(PLATE_T) 위로 몸통 둘레의 턱과 날개가 RING_H만큼 올라와 있다. 몸통은 턱 안에 앉아 자리를 잡고,
    밑에서 올라온 나사 넷이 몸통 속 너트에 물린다. 클램프는 날개 윗면을 책상 가장자리에서 문다.
    """
    r_seat = R_CORE + SEAT_GAP
    r_out = r_seat + RING_W
    y0, y1, y2 = Y_DESK, Y_FLOOR, Y_FLOOR + RING_H
    floor = cyl_y(r_out, y0, y1) + box(-EDGE, WING_X, y0, y1, -WING_Z, WING_Z)
    ring = j.ring(r_out, r_seat, y1 - 0.01, y2)
    wings = box(-EDGE, WING_X, y1 - 0.01, y2, WING_IN, WING_Z)
    wings += box(-EDGE, WING_X, y1 - 0.01, y2, -WING_Z, -WING_IN)
    raised = solid(ring + solid(solid(wings) - cyl_y(r_seat, y1 - 1, y2 + 1)))
    p = solid(solid(floor) + raised)
    # 선이 나가는 길: 몸통 뒤 아래의 선 구멍 앞에서 턱을 튼다
    hw = WIRE_EXIT[0] + 1.0
    p = solid(p - box(-r_out - 1, -r_seat + 1, y1, y2 + 1, -hw, hw))
    for x, z in plate_xz():  # 나사 구멍과 밑면의 머리 자리. 그 사이에 막 한 층이 남는다
        p = solid(p - cyl_y(j.M3_FREE / 2, y0 + PLATE_RECESS + SKIN, y1 + 1, x, z))
        p = solid(p - cyl_y(SINK_R, y0 - 1, y0 + PLATE_RECESS, x, z))
    for x, z in mount_xz():  # 부속 자리: 위에서 조이는 M3 나사와 밑면의 너트 자리
        p = solid(p - cyl_y(j.M3_FREE / 2, y0 + ACC_NUT + SKIN, y2 + 1, x, z))
        p = solid(p - j.hex_y(x, z, y0 - 1, y0 + ACC_NUT, j.NUT_AF))
    if not p.is_valid:
        raise RuntimeError("책상에 무는 판 형상이 깨졌다")
    return p


def desk_parts() -> list[j.Part]:
    """책상과 금속 클램프 둘 (그림에만 쓴다). 클램프는 사진으로 어림한 치수다. 등을 책상 가장자리에 붙여 그린다."""
    top = Y_FLOOR + RING_H
    under = Y_DESK - DESK_T  # 책상 밑면
    desk = box(-EDGE, 150.0, under, Y_DESK, -130.0, 130.0)
    out = [j.Part("책상", desk, color="desk")]
    arm_top = top - (CLAMP_OPEN + PAD_T)  # 아래 턱의 윗면
    x_back, x_tip, x_axis = -EDGE - SPINE_T, -EDGE + JAW_REACH, -EDGE + JAW_AXIS
    for k, s in enumerate((1, -1), start=1):
        z0, z1 = sorted((s * (CLAMP_Z - JAW_HW), s * (CLAMP_Z + JAW_HW)))
        jaw = box(x_back, x_tip, top, top + JAW_T, z0, z1)
        spine = box(x_back, -EDGE, arm_top - SPINE_T, top + JAW_T, z0, z1)
        low = box(x_back, x_tip, arm_top - SPINE_T, arm_top, z0, z1)
        screw = cyl_y(4.0, arm_top - SPINE_T - 45.0, under - 4.0, x_axis, s * CLAMP_Z)
        knob = cyl_y(9.0, arm_top - SPINE_T - 57.0, arm_top - SPINE_T - 45.0, x_axis, s * CLAMP_Z)
        pad = cyl_y(PAD_R, under - 4.0, under, x_axis, s * CLAMP_Z)
        shape = solid(jaw + spine + low + screw + knob + pad)
        out.append(j.Part(f"클램프 {k}", shape, color="metal"))
    return out


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

    조임 볼트의 머리와 와셔는 테 윗면에서 SINK만큼 내려간 자리에 앉는다. 자리의 천장은 허공에 걸쳐 뽑히고,
    막 한 층이 볼트 구멍을 덮는다.
    """
    y_tip = B0 + JOURNAL_GAP
    a = cyl_y(j.R_JOURNAL, y_tip + JOURNAL_TIP, B1 + j.BRG_B + 0.01)
    a += Pos(0, y_tip + JOURNAL_TIP / 2, 0) * (
        Rot(-90, 0, 0) * Cone(j.R_JOURNAL - JOURNAL_TIP, j.R_JOURNAL, JOURNAL_TIP + 0.01)
    )
    a += cyl_y(j.R_STEP, B1 + j.BRG_B, SEAM + 0.01)
    a += cyl_y(STEP_WIDE_R, B1 + j.BRG_B + STEP_WIDE_UP, SEAM + 0.01)
    brim = cyl_y(R_TABLE, SEAM, ATOP) + Pos(0, SEAM + FLARE / 2, 0) * (
        Rot(-90, 0, 0) * Cone(R_TABLE + FLARE, R_TABLE, FLARE)
    )
    a = solid(a + solid(Rot(0, 45, 0) * solid(brim)))
    # 테 밑의 테두리: 몸통 윗면과의 틈을 좁혀, 띠에서 올라오는 선이 돌림판 밑으로 들어가지 않게 한다
    lip = j.ring(TABLE_LIP[1], TABLE_LIP[0], FRONT + j.LIP_GAP, SEAM + 0.01)
    a = solid(a + solid(Rot(0, 45, 0) * lip))
    a = solid(a + cyl_y(j.BOSS_R, POST_TIP, SEAM + 0.01, R_POST, 0.0))  # 멈춤 기둥 (속에 M3 나사)
    a = solid(a - cyl_y(j.M3_FREE / 2, POST_TIP - 1, ATOP + 1, R_POST, 0.0))
    a = solid(a - cyl_y(3.1, ATOP - 1.8, ATOP + 1, R_POST, 0.0))  # 기둥 나사 머리 자리
    for x, z in clamp_xz():  # 볼트 구멍과 머리 자리. 그 사이에 막 한 층이 남는다
        a = solid(a - cyl_y(j.M3_FREE / 2, B0, ATOP - SINK - SKIN, x, z))
        a = solid(a - cyl_y(SINK_R, ATOP - SINK, ATOP + 1, x, z))
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
    # 뒤쪽 귀의 누르개 나사 구멍 (0031): J1을 넘는 선을 여기서 잡는다. 베이스의 (x, z) → 어깨의 (y, z)
    cx, cz = EAR_CLAMP
    h = solid(h - cyl_x(j.M3_TAP / 2, x0 - 1, x_ear + 1, sh.J1_AXIS_Y - cz, -cx))
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


def _screw_down(
    x: float, z: float, seat: float, length: float, head: float = 1.65, head_r: float = 2.85
) -> Compound:
    """머리가 위(+y)에 있는 M3 볼트. 머리 밑면이 seat. 기본 머리는 둥근머리 렌치볼트다."""
    return solid(cyl_y(1.5, seat - length, seat, x, z) + cyl_y(head_r, seat, seat + head, x, z))


def hardware(model: ServoModel) -> list[j.Part]:
    """조임 볼트, 혼 나사, 너트, 벽 나사, 멈춤 나사, J1 몸통 나사, 영점 핀, 서보 뒷면 나사."""
    dy = j.servo_dy(model)
    out: list[j.Part] = []
    pocket = ATOP - SINK  # 머리 자리의 바닥 (와셔가 앉는 면)
    for k, (x, z) in enumerate(clamp_xz()):
        bolt = _screw_down(x, z, pocket + WASHER_T, CLAMP_LEN, CLAMP_HEAD_H, CLAMP_HEAD_R)
        out.append(j.Part(f"조임 볼트 M3x45 {k + 1}", bolt, j.STEEL, True, "metal"))
        washer = j.ring(4.0, 1.6, pocket, pocket + WASHER_T)
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
    head = Y_DESK + PLATE_RECESS - WASHER_T  # 판 나사 머리의 윗면 (와셔 밑)
    for k, (x, z) in enumerate(plate_xz(), start=1):
        s = cyl_y(1.5, head, head + PLATE_SCREW, x, z) + cyl_y(2.85, head - 1.65, head, x, z)
        out.append(j.Part(f"판 나사 M3x10 {k}", solid(s), j.STEEL, False, "metal"))
        washer = j.ring(4.0, 1.6, head, head + WASHER_T)
        out.append(
            j.Part(f"와셔 (판 나사) {k}", solid(Pos(x, 0, z) * washer), j.STEEL, False, "metal")
        )
        y0 = Y_FLOOR + PLATE_NUT_UP
        nut = j.hex_y(x, z, y0, y0 + j.NUT_T, 5.4) - cyl_y(1.5, y0 - 1, y0 + j.NUT_T + 1, x, z)
        out.append(j.Part(f"너트 (판) {k}", solid(nut), j.STEEL, False, "metal"))
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
        j.Part("책상에 무는 판", plate(), color="frame"),
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
    clamp_seat = ATOP - SINK + WASHER_T  # 조임 볼트 머리의 밑면
    clamp_tip = clamp_seat - CLAMP_LEN
    foot_tip = SEAM + FOOT_RECESS + FOOT_SCREW
    foot_nut = ATOP + FLOOR_T - FOOT_NUT  # 바닥판 속 너트의 밑면
    plate_tip = Y_DESK + PLATE_RECESS - WASHER_T + PLATE_SCREW  # 판 나사의 끝
    return {
        "혼 나사가 혼 테(2.5)에 물린 길이": min(j.SEAT - horn_tip, HORN_FLANGE),
        "혼 나사 끝과 서보 앞면 사이": horn_tip - (j.F29 + dy),
        "조임 볼트가 너트(2.4)에 물린 길이": min(nut_top - clamp_tip, j.NUT_T),
        "조임 볼트 끝과 허브 밑면 사이": clamp_tip - (j.SEAT + HUB_RELIEF),
        "조임 볼트 머리 윗면과 돌림판 윗면 사이": ATOP - (clamp_seat + CLAMP_HEAD_H),
        "조임 볼트 머리 자리와 위 안쪽 링 윗면 사이의 살": (ATOP - SINK) - (B1 + j.BRG_B),
        "J1 몸통 나사가 너트(2.4)에 물린 길이": min(foot_tip - foot_nut, j.NUT_T),
        "J1 몸통 나사 끝과 바닥판 윗면 사이": (ATOP + FLOOR_T) - foot_tip,
        "J1 몸통 바닥판과 어깨 서보 먼 끝 사이": (Y_DESK + SHOULDER_H - sh.FAR) - (ATOP + FLOOR_T),
        "판 나사가 너트(2.4)에 물린 길이": min(plate_tip - (Y_FLOOR + PLATE_NUT_UP), j.NUT_T),
        "판 나사 끝과 나사 구멍 바닥 사이": (Y_FLOOR + PLATE_HOLE) - plate_tip,
        "판 나사 머리 자리 위에 남은 판의 살": PLATE_T - PLATE_RECESS,
        "클램프가 벌어지는 틈에서 책상과 날개를 뺀 나머지": CLAMP_OPEN
        - DESK_T
        - (PLATE_T + RING_H),
    }


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋). J1 몸통과 그 뒷덮개(어깨 원통의 뒷면)는 어깨의 서보 좌표다
def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {
        "base_core": (core(), (0.0, -1.0, 0.0), "black", "strength"),
        "base_plate": (plate(), (0.0, 1.0, 0.0), "black", "strength"),
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
