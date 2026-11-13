"""STS 관절(J3~J6) 하나의 부품 (0028의 UR식 관절을 부품으로 나눈 것, 0029).

서보 좌표계(mm)로 그린다: x는 출력축에서 서보의 가까운 끝 쪽, y는 출력축 방향(출력 혼 쪽이 +), z = x × y.
원점은 출력축 위, 몸통 두께의 가운데다: y = -14.5가 출력축 둘레의 낮은 뒷면(두께 29), -16.0이 두께 32인
뒷면이다 (docs/hardware.md의 "기구 치수"). 출력 혼 윗면은 그 낮은 뒷면에서 잰 값이다 ("혼 높이").
방위각은 x-z 평면에서 +x가 0°, +z가 90°다. 집의 목(서보 먼 끝)은 180° 쪽이다.

구조:
- 부모: 집(원통과 목). 원통 앞 끝의 턱이 한 몸이고, 베어링(6806)은 뒤에서 넣는다. 누름 틀이 바깥 링 뒷면을
  받치고 벽 나사 셋으로 잠긴다. 서보는 뒷면에 단 서보 받침의 날개를 벽 나사 둘로 맨 나중에 잠근다. 벽 구멍이
  축 방향으로 길어서, 베어링이 정한 자리 그대로 잠긴다 (0028: 베어링이 축을 정한다).
- 자식: 허브 판(혼 윗면에 앉고 안쪽 링 뒤를 받침)과 출력 원판(안쪽 링 속 저널, 원통 앞 끝 위의 테)이 혼 볼트
  넷으로 안쪽 링을 물고 혼에 묶인다. 자식 몸통은 출력 원판 윗면에 M3 × 10 넷으로 단다 (parts/wrist.py, parts/links.py).
- 기계 멈춤: 출력 원판 귀의 기둥(속에 M3 나사)이 목 앞면의 홈을 돌다가 가운데 살에 닿는다 (±STOP_DEG).
- 영점 핀: 자식 몸통의 귀와 그 살의 구멍에 꽂는다.

부품을 품는 몸통(손목 몸통, 링크)은 이 모듈의 원통과 목, 홈을 가져다 쓴다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Compound, Cone, Polygon, Pos, RegularPolygon, Rot, extrude

from horilab_design.parts.shapes import box, cyl_x, cyl_y, cyl_z, far_apart, solid
from horilab_design.values import ServoModel, load_measured

PLA = 1.24e-3  # [g/mm³] 나사와 너트의 무게를 어림할 때만 쓴다. 출력 무게는 printcheck.printed_mass_g가 낸다
STEEL = 7.85e-3

_MEASURED = load_measured()

# ---- 서보 (STS3215와 STS3250은 도면이 같다. 공식 2D 도면의 옆모습 29 / 32 / 35와 실측) ----
HW = 12.36  # 반폭
NEAR, FAR = 10.11, 35.11
L0, F29 = -14.5, 14.5  # 출력축 둘레의 낮은 면 (두께 29)
L1, F32 = -16.0, 16.0  # 두께 32인 뒷면과 앞면 (고정 구멍과 커넥터가 이 면에 있다)
STEP = -6.0  # 29 → 32 단이 끝나는 x
LABEL = F32 + 1.1  # 앞면 라벨 판 (x -34.1 ~ -9.2, z ±7.0)
PLATEAU = L1 - 1.9  # 뒷면 볼록한 판 (x -29.7 ~ -17.33, z ±9.24)
BOSS_TIP = L0 - 4.1  # 뒷면 보스 끝 (도면. STS3215의 실측 혼 윗면 ~ 보스 끝 37.32와 맞는다)
HORN_R = 9.975
STS_MODELS: tuple[ServoModel, ...] = ("STS3215", "STS3250")
# 출력 혼 윗면: 낮은 뒷면에서 잰 값이다. 혼이 앞면 라벨 판보다 1.6 ~ 1.8 높다 (깊이 막대로 잰 1.5 ~ 1.75와 맞는다)
HORN_TOP: dict[ServoModel, float] = {
    m: L0 + _MEASURED.servo_body[m].horn_to_back_mm for m in STS_MODELS
}
assert _MEASURED.servo_depth is not None
# 혼 가운데 나사의 머리: 지름과, 혼 윗면 위로 나온 높이 (1.7 ~ 2.0으로 오르내린 값 중 큰 값)
HEAD_D = _MEASURED.servo_depth.horn_screw_head_diameter_mm
HEAD_H = max(_MEASURED.servo_depth.horn_screw_head_mm)
_WIRE = _MEASURED.servo_body["STS3250"]
assert _WIRE.horn_to_wire_mm is not None
WIRE_TOP = HORN_TOP["STS3250"] - _WIRE.horn_to_wire_mm  # 꽂아 눕힌 선 (혼 윗면에서 잰 값, -20.12)
BACK_HOLES = ((-8.30, 10.25), (-8.30, -10.25), (-32.75, 10.25), (-32.75, -10.25))

# ---- 공차 (시험편 값. 검정과 6806은 아직 재지 않아 임시 값이다) ----
_WHITE = _MEASURED.fit["pla_basic_white"]
assert _WHITE.pin_hole_mm is not None and _WHITE.pin_hole_horizontal_mm is not None
PIN_HOLE = _WHITE.pin_hole_mm  # 세로 구멍 (몸통의 살)
PIN_HOLE_H = _WHITE.pin_hole_horizontal_mm  # 가로 구멍 (손목 몸통의 귀는 뽑을 때 눕는다)
M3_FREE = 3.2  # 볼트가 지나가는 세로 구멍: 시험편 3.1에 구멍 자리 오차 몫 0.1
M3_FREE_H = 3.4  # 가로 구멍: 시험편 3.3에 0.1
M3_TAP = 2.8  # 조여 넣는 구멍 (뒷덮개 나사)
NUT_AF, NUT_T = 5.6, 2.4  # 위에서 눌러 끼우는 육각 홈의 맞변 거리, 너트 두께 (출력 원판 밑면)
# 누름 틀과 서보 받침의 너트 자리: 뽑을 때 너트가 서고(축이 가로) 홈 천장이 허공에 걸친다. 위에서 끼우는 홈과
# 뽑히는 방향이 달라 그 값이 옮겨지지 않는다 (흰 받침 기둥 귀에서 5.6에 너트가 손으로 들어가지 않았다).
# 임시: 검정 fit1_nut_slots_upright 결과(세로 폭)로 바꾼다
NUT_AF_UPRIGHT = 5.6
BEARING_BORE = 42.1  # 6806 바깥 링 자리 (임시: 흰색 fit3_6806_bores 결과로 바꾼다)
JOURNAL_D = 29.9  # 안쪽 링 속 저널 (임시: 검정 fit3_6806_shafts 결과로 바꾼다)
PROVISIONAL = (
    "BEARING_BORE",
    "JOURNAL_D",
    "NUT_AF_UPRIGHT",
    "검정 부품의 구멍과 너트 홈 (흰색 값을 씀)",
)

# ---- 베어링과 축 방향 쌓임 ----
BRG_D, BRG_OD, BRG_B = 30.0, 42.0, 7.0
SEAT = max(
    HORN_TOP.values()
)  # 허브 판이 혼 윗면에 앉는 높이 (STS3250 기준. STS3215는 몸통이 0.17 앞으로 온다)
# 허브 판이 혼 위에서 두꺼운 높이 (혼 바깥에서는 밑면이 이만큼 들려 있다). 혼 가운데 나사 머리(혼 윗면 위로
# HEAD_H)가 허브 판 속에 다 들어가고, 저널 끝과 0.8 뜬다. 앞면 라벨 판과는 3.2 ~ 3.4 뜬다
RELIEF = 1.6
HUB_SEAT_R = 9.8  # 허브 판이 혼 윗면에 앉는 돋움의 반지름 (테는 이 가장자리에서 휜다)
# 허브 판의 테 (안쪽 링 뒤). 혼 볼트를 조이면 이 테가 CLAMP_GAP만큼 휘면서 안쪽 링을 누르고, 그 뒤에는
# 저널 끝이 허브 판에 닿아 볼트 힘이 저널로 간다. 테가 스프링이라 온도와 크리프에도 누르는 힘이 남는다.
# 두께 1.2, 틈 0.2면 누르는 힘 80 ~ 110 N, 휨 응력 28 ~ 38 MPa다 (탄성 계수 2.0 ~ 2.75 GPa, 고리 판 계산:
# studies/clamp_plate.py). 두께 2.0, 틈 0.3이면 69 ~ 104 MPa로 PLA의 굽힘 강도(약 76 MPa)에 닿는다
FLANGE_T = 1.2
# 조이기 전에 저널 끝과 허브 판 사이에 남는 틈. 출력 원판을 뽑을 때 저널 끝이 층에 맞는 값이다
CLAMP_GAP = 0.2
B0 = SEAT + RELIEF + FLANGE_T  # 베어링 뒷면
LIP_T = 1.8
FRONT = B0 + BRG_B + LIP_T  # 원통 앞 끝 (몸통을 뽑을 때 바닥)
GAP = 2.0  # 원통 앞 끝과 출력 원판 테 사이 (0017의 움직이는 틈)
SEAM = FRONT + GAP  # 출력 원판 테의 밑면
BRIM_T = 3.8  # M3 × 20 혼 볼트 머리가 테 윗면(뽑을 때 바닥 면)에 바로 앉고 혼에 2.6 들어가는 두께
ATOP = SEAM + BRIM_T  # 출력 원판 윗면 = 자식 몸통이 앉는 면
R_BORE = BEARING_BORE / 2
WALL = 2.55
R_DRUM = R_BORE + WALL  # 원통 바깥 반지름 23.6
R_LIP = 19.5
R_STEP = 16.5  # 허브 판의 테와 출력 원판의 단: 안쪽 링 앞뒤 면만 누른다
R_JOURNAL = JOURNAL_D / 2
COVER_GAP = 1.0
BACK_IN = WIRE_TOP - COVER_GAP  # 뒷덮개 안쪽 면
COVER_T = 1.5
BACK_END = BACK_IN + 0.8  # 원통 뒤 끝. 덮개는 서보 받침을 따라 앉으므로 0.8 띄운다
COVER_SKIRT = 2.5  # 덮개 테가 원통 뒤 끝을 겹쳐 가리는 길이
COVER_R = R_DRUM + 0.4 + 1.6
BACK_OUT = BACK_IN - COVER_T  # 뒷덮개 바깥 면

# ---- 원통 속 부품 ----
RET_Y = (5.0, 18.7)  # 누름 틀 다리의 범위
RET_SCREW_Y = 12.0
CAR_Y = (L1, L1 + 17.7)  # 서보 받침 날개의 범위
CAR_SCREW_Y = L1 + 12.5  # 서보를 잠그는 벽 나사 (받침 판에서 12.5)
LEAF_T = 1.4  # 날개 두께: 반지름 쪽으로 무르고 돌리는 쪽으로 단단하다
PLATE_T = 3.0
COVER_BOSS = ((3.0, 16.0), (3.0, -16.0))  # 뒷덮개 나사 자리 (x, z)
COVER_SCREW = 12.0  # 뒷덮개 나사 M3 × 12
# 서보 뒷면 나사(M2)의 머리는 받침 판 속 자리에 든다: 뒷덮개 안쪽 면이 판에서 2.1 mm라, 머리가 판 위로 나오면
# 덮개에 닿고 그 위로 선이 지나가지 못한다. 자리 밑에 남는 판의 살은 PLATE_T - M2_SINK = 1.2 mm다
M2_SINK = 1.8
M2_HEAD_R = 2.2  # 머리 자리의 반지름 (머리 지름 3.5 ~ 4.0)
M2_SCREW = 4.0  # M2 × 4: 서보에 2.8 mm 물린다
# 먼 가로대의 가운데는 뒷면을 이만큼 낮춘다: 꽂아 눕힌 선(밑이 판 뒷면보다 0.4 낮다)이 잰 높이 그대로 지나간다
WIRE_GROOVE = 0.8
SPOT_R = R_DRUM - 0.6  # 벽 나사 머리가 앉는 면
SCREW_LEN = 8.0  # 벽 나사 M3 × 8

# ---- 목 (서보 먼 끝을 품는다) ----
NECK_HW = HW + 0.34 + 2.6  # 목 바깥 반폭 15.3
NECK_IN = HW + 0.34  # 목 안쪽 반폭
NECK_FRONT_IN = FRONT - 2.4  # 목 앞벽의 안쪽 면

# ---- 기계 멈춤, 영점 핀, 자식 몸통 나사 ----
R_POST = 28.0  # 멈춤 기둥 중심
BOSS_R = 3.25
# 멈춤 기둥 귀의 반폭. 귀의 옆면은 멈춤 각에서 살 가운데(선이 넘는 자리) 가까이 온다. 4.0이면 테의 면에서
# 살 가운데까지 3.1 mm가 남아 선(반폭 2.4)과 0.7 mm 뜬다 (4.6이면 2.5 mm가 남아 0.1 mm다)
EAR_HW = 4.0
POST_TIP = SEAM - 4.5  # 기둥 돋움의 끝. 그 아래 너트 2.4
STOP_DEG = 163.0  # 기둥이 살에 닿는 각. 프로그램 한계는 SOFT_DEG
SOFT_DEG = 160.0
WEB_HALF = 180.0 - STOP_DEG - math.degrees(math.asin(BOSS_R / R_POST))  # 살의 반각 10.3°
CH_R1 = 32.8  # 홈 바깥 반지름
CH_Y = 23.5  # 홈 바닥 (원통 앞 끝에서 7.0). 멈춤 나사 끝(24.5)과 1.0 띄운다
TUB_Y = CH_Y - 1.6  # 홈 밑을 막는 살의 밑면. 서보 앞면의 라벨 판(17.3)과 4.6 띄운다
TUB_X = -34.6  # 그 살이 끝나는 x
R_PIN = 29.0  # 영점 핀
RIM = ((5.0, 19.4), (-5.0, 19.4), (5.0, -19.4), (-5.0, -19.4))  # 자식 몸통 나사 (x, z)
BODY_SCREW = 10.0  # M3 × 10
EAR_T = (
    6.2  # 자식 몸통 귀의 두께: 나사 10 = 귀 6.2 + 출력 원판 살 1.3 + 너트 2.4 (끝이 0.1 모자라게)
)
WIN_X = (-45.0, -35.4)  # 링크의 선 창 (플러그 9.58 × 4.75가 지나간다)
WIN_Z = 3.0

# ---- 선 (0031) ----
# 서보 선 한 줄이 차지하는 자리: 세 가닥을 나란히 놓은 폭과 두께 (딸려 온 20 cm 선을 잰 값, 4.75 × 1.5)
assert _MEASURED.servo_wire is not None
WIRE_W, WIRE_T = _MEASURED.servo_wire.width_mm, _MEASURED.servo_wire.thickness_mm
# 꽂은 플러그의 뒤 끝 (선이 나오는 곳): 눕힌 선의 밑이다. 커넥터가 있는 뒷면(L1)에서 2.6 나온다
PLUG_TOP = WIRE_TOP + WIRE_T
# 선은 살(180°) 위에서 출력 원판의 테로 건너가 테 둘레에 감긴다. 테 밑의 틈(GAP)으로 선이 빠지면 도는 원판과
# 서 있는 원통 사이에 낀다. 테 밑면 가장자리에 테두리를 내려 틈을 LIP_GAP으로 좁힌다 (선 두께보다 좁다)
LIP_GAP = 0.7
LIP_W = 1.2


# ------------------------------------------------------------------------------------------------
# 형상 도우미


def ring(r_out: float, r_in: float, y0: float, y1: float) -> Compound:
    return solid(cyl_y(r_out, y0, y1) - cyl_y(r_in, y0 - 0.01, y1 + 0.01))


def hex_z(x: float, y: float, z0: float, z1: float, af: float = NUT_AF) -> Compound:
    """z 방향 육각 기둥 (너트 자리)."""
    h = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=z1 - z0)
    return solid(Pos(x, y, z0) * h)


def hex_x(y: float, z: float, x0: float, x1: float, af: float = NUT_AF) -> Compound:
    h = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=x1 - x0)
    return solid(Pos(x0, y, z) * (Rot(0, 90, 0) * h))


def hex_y(x: float, z: float, y0: float, y1: float, af: float = NUT_AF) -> Compound:
    h = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=y1 - y0)
    return solid(Pos(x, y1, z) * (Rot(90, 0, 0) * h))


def polar(r: float, a: float) -> tuple[float, float]:
    return r * math.cos(math.radians(a)), r * math.sin(math.radians(a))


def wedge(a0: float, a1: float, r: float, y0: float, y1: float) -> Compound:
    """방위각 a0 ~ a1 [°], 반지름 r까지의 부채꼴 기둥 (y0 ~ y1)."""
    n = max(2, int(abs(a1 - a0) / 6) + 1)
    pts = [(0.0, 0.0)] + [polar(r, a0 + (a1 - a0) * k / n) for k in range(n + 1)]
    prism = extrude(Polygon(*pts, align=None), amount=y1 - y0)
    return solid(Pos(0, y1, 0) * (Rot(90, 0, 0) * prism))


def sector(r0: float, r1: float, a0: float, a1: float, y0: float, y1: float) -> Compound:
    """방위각 a0 ~ a1, 반지름 r0 ~ r1의 고리 조각."""
    band = cyl_y(r1, y0, y1) - cyl_y(r0, y0 - 0.01, y1 + 0.01)
    return solid(band & wedge(a0, a1, r1 + 5, y0 - 0.02, y1 + 0.02))


def prism_x(pts: list[tuple[float, float]], x0: float, x1: float) -> Compound:
    """(y, z) 다각형을 x0 ~ x1로 뽑은 기둥."""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1], strict=True))
    if area < 0:
        pts = pts[::-1]  # 반시계 방향으로 맞춘다 (면이 뽑는 쪽을 보게)
    body = extrude(Polygon(*pts, align=None), amount=x1 - x0)
    # 스케치 (u, v, w) → (x, y, z) = (w, u, v): z축 둘레 90°, 이어서 y축 둘레 90°
    return solid(Pos(x0, 0, 0) * (Rot(0, 90, 0) * (Rot(0, 0, 90) * body)))


def turned(shape: Compound, q_deg: float) -> Compound:
    """자식 쪽 부품을 관절 각 q만큼 돌린다 (+y 축 둘레, 방위각이 커지는 쪽)."""
    return solid(Rot(0, -q_deg, 0) * shape)


# ------------------------------------------------------------------------------------------------
# 검사용 서보 외형 (출력하지 않는다)


def servo(model: ServoModel = "STS3250") -> dict[str, Compound]:
    """서보 몸통, 혼, 꽂은 플러그, 눕힌 선이 차지하는 자리. 몸통은 혼 윗면이 SEAT에 오게 옮긴다."""
    dy = SEAT - HORN_TOP[model]
    top = HORN_TOP[model]
    body = box(-FAR, NEAR, L0, F29, -HW, HW)  # 두께 29
    body += box(-34.5, STEP, L1, F32, -HW, HW)  # 두께 32 (앞뒤로 1.5씩)
    body += box(-34.1, -9.2, F32, LABEL, -7.0, 7.0)  # 앞면 라벨 판
    body += box(-29.7, -17.33, PLATEAU, L1, -9.24, 9.24)  # 뒷면 볼록한 판
    body -= cyl_y(10.8, F29, LABEL + 1)  # 혼 둘레의 둥근 자리
    body += cyl_y(3.1, BOSS_TIP, L0)  # 뒷면 보스
    horn = cyl_y(HORN_R, F29 + 0.3, top)
    horn += cyl_y(HEAD_D / 2, top, top + HEAD_H)  # 혼과 가운데 나사 머리
    plugs = box(-16.6, -11.85, PLUG_TOP, L1 + 0.5, 0.45, 9.46)
    plugs += box(-16.6, -11.85, PLUG_TOP, L1 + 0.5, -9.46, -0.45)
    wires = box(-36.0, -11.85, WIRE_TOP, PLUG_TOP, -8.5, 8.5)  # 먼 끝 쪽으로 눕힌 선
    out = {"body": solid(body), "horn": solid(horn), "plugs": solid(plugs), "wires": solid(wires)}
    return {k: solid(Pos(0, dy, 0) * v) for k, v in out.items()}


def servo_dy(model: ServoModel) -> float:
    """서보 몸통과 거기 달린 부품(서보 받침, 뒷덮개)이 축 방향으로 옮겨지는 값."""
    return SEAT - HORN_TOP[model]


# ------------------------------------------------------------------------------------------------
# 원통과 목 (몸통이 가져다 쓰는 조각)


def drum_shell(neck_x: float) -> Compound:
    """원통과 목의 겉. 목은 x = neck_x까지다."""
    return solid(
        cyl_y(R_DRUM, BACK_END, FRONT) + box(neck_x, -12.0, BACK_END, FRONT, -NECK_HW, NECK_HW)
    )


def drum_cavity(
    neck_x: float, roof_y: float | None = None, roof_x: float | None = None
) -> Compound:
    """원통과 목의 속. 서보 앞면 위(TUB_Y부터)는 멈춤 홈 밑을 막는 살로 채우고 x < TUB_X만 비운다.

    roof_y를 주면 목의 뒷면 쪽(y < roof_y)은 x = roof_x에서 막힌다 (링크: 목이 낮은 관으로 이어질 때).
    """
    c = cyl_y(R_BORE, BACK_END - 1, B0 + BRG_B) + cyl_y(R_LIP, B0, FRONT + 1)
    if roof_y is None or roof_x is None:
        c += box(neck_x - 1, -12.0, BACK_END - 1, TUB_Y, -NECK_IN, NECK_IN)
    else:
        c += box(neck_x - 1, -12.0, roof_y, TUB_Y, -NECK_IN, NECK_IN)
        c += box(roof_x, -12.0, BACK_END - 1, roof_y + 0.01, -NECK_IN, NECK_IN)
    c += box(neck_x - 1, TUB_X, TUB_Y - 0.5, NECK_FRONT_IN, -NECK_IN, NECK_IN)
    return solid(c)


def cut_joint_features(h: Compound, window: bool) -> Compound:
    """몸통에 이 관절의 멈춤 홈과 살, 영점 핀 구멍, 벽 나사 구멍을 낸다. window면 링크의 선 창도 낸다.

    불 연산을 하나씩 하고 그때마다 입체로 좁힌다. 한꺼번에 빼면 형상 계산이 깨지는 일이 있었다.
    """
    # 멈춤 홈: 원통 벽 0.4 밖부터 CH_R1까지, 목 폭 전체. 가운데 살만 남긴다
    h = solid(h - sector(R_DRUM + 0.4, CH_R1, 180 + WEB_HALF, 240, CH_Y, FRONT + 1))
    h = solid(h - sector(R_DRUM + 0.4, CH_R1, 120, 180 - WEB_HALF, CH_Y, FRONT + 1))
    h = solid(h - cyl_y(PIN_HOLE / 2, TUB_Y - 1, FRONT + 1, -R_PIN, 0.0))
    if window:
        h = solid(h - box(WIN_X[0], WIN_X[1], NECK_FRONT_IN - 1, FRONT + 1, -WIN_Z, WIN_Z))
    r = M3_FREE_H / 2
    for s in (1, -1):
        z0, z1 = (15.0, R_DRUM + 1) if s > 0 else (-R_DRUM - 1, -15.0)
        zs0, zs1 = (SPOT_R, R_DRUM + 1) if s > 0 else (-R_DRUM - 1, -SPOT_R)
        h = solid(h - cyl_z(r, z0, z1, 0.0, RET_SCREW_Y))
        h = solid(h - box(-3.4, 3.4, RET_SCREW_Y - 3.4, RET_SCREW_Y + 3.4, zs0, zs1))
        # 서보 받침 나사 구멍은 축 방향으로 길다 (서보가 찾은 자리 그대로 잠근다)
        slot = cyl_z(r, z0, z1, 0.0, CAR_SCREW_Y - 1.0) + cyl_z(r, z0, z1, 0.0, CAR_SCREW_Y + 1.0)
        slot += box(-r, r, CAR_SCREW_Y - 1.0, CAR_SCREW_Y + 1.0, z0, z1)
        h = solid(h - solid(slot))
        h = solid(h - box(-4.2, 4.2, CAR_SCREW_Y - 5.2, CAR_SCREW_Y + 5.2, zs0, zs1))
    h = solid(h - cyl_x(r, 15, R_DRUM + 1, RET_SCREW_Y, 0.0))
    h = solid(h - box(SPOT_R, R_DRUM + 1, RET_SCREW_Y - 3.4, RET_SCREW_Y + 3.4, -3.4, 3.4))
    if not h.is_valid or h.volume < 1000:
        raise RuntimeError(f"몸통 형상이 깨졌다: 부피 {h.volume:.1f}")
    return h


# ------------------------------------------------------------------------------------------------
# 원통 속 부품 (검정)


def bearing() -> Compound:
    return ring(BRG_OD / 2, BRG_D / 2, B0, B0 + BRG_B)


def bolt_xz() -> list[tuple[float, float]]:
    """혼 볼트 넷. 자식 몸통 속을 지나는 선(|z| 8 ~ 12)과 머리가 비키게 45° 돌려 둔다."""
    return [polar(7.0, a) for a in (45, 135, 225, 315)]


def hub_plate() -> Compound:
    """허브 판: 혼 윗면에 앉고(가운데 r 9.8), 넓은 테가 안쪽 링 뒤를 받친다. 테 윗면을 바닥에 두고 뽑는다.

    가운데 구멍은 혼 가운데 나사 머리(지름 5.15, 높이 2.0까지)의 자리다. 판을 뚫어서, 머리가 판 두께(2.8)
    안에 들고 그 위의 저널 끝과 0.8 뜬다.
    """
    h = cyl_y(HUB_SEAT_R, SEAT, SEAT + RELIEF + 0.01) + cyl_y(R_STEP, SEAT + RELIEF, B0)
    h -= cyl_y(3.6, SEAT - 1, B0 + 1)  # 혼 가운데 나사 머리
    for x, z in bolt_xz():
        h -= cyl_y(M3_FREE / 2, SEAT - 1, B0 + 1, x, z)
    return solid(h)


def out_disc() -> Compound:
    """출력 원판: 저널(안쪽 링 속) + 단(안쪽 링 앞면) + 테(원통 앞 끝 위) + 멈춤 기둥 귀.

    테 윗면을 바닥에 두고 뽑는다. 혼 볼트 머리는 그 면(바닥에 닿아 평평한 면)에 앉는다.
    테 밑면 가장자리의 테두리는 원통 앞 끝과 LIP_GAP만 띄워, 테에 감기는 선이 틈으로 빠지지 않게 한다.
    """
    # 허브 판과 CLAMP_GAP만큼 띄운다 (안쪽 링을 사이에 두고 조인다)
    a = cyl_y(R_JOURNAL, B0 + CLAMP_GAP, B0 + BRG_B + 0.01)
    a += cyl_y(R_STEP, B0 + BRG_B, SEAM + 0.01)
    a += cyl_y(R_DRUM, SEAM, ATOP)
    a += ring(
        R_DRUM, R_DRUM - LIP_W, FRONT + LIP_GAP, SEAM + 0.01
    )  # 선이 틈으로 빠지지 않게 막는 테두리
    a += box(20.0, R_POST, SEAM, ATOP, -EAR_HW, EAR_HW)
    a += cyl_y(EAR_HW, SEAM, ATOP, R_POST, 0.0)  # 귀
    a += cyl_y(BOSS_R, POST_TIP, SEAM + 0.01, R_POST, 0.0)  # 멈춤 기둥 (속에 M3 나사)
    a -= cyl_y(M3_FREE / 2, POST_TIP - 1, ATOP + 1, R_POST, 0.0)
    a -= cyl_y(3.1, ATOP - 1.8, ATOP + 1, R_POST, 0.0)  # 기둥 나사 머리 자리
    for x, z in bolt_xz():
        a -= cyl_y(M3_FREE / 2, B0, ATOP + 1, x, z)
    for x, z in RIM:
        a -= cyl_y(M3_FREE / 2, SEAM - 1, ATOP + 1, x, z)
        a -= hex_y(x, z, SEAM - 0.01, SEAM + NUT_T + 0.1)  # 밑면에서 끼우는 너트 홈
    return solid(a)


def retainer(b0: float = B0) -> Compound:
    """누름 틀: 바깥 링 뒷면을 받치는 고리와 다리 셋 (벽 나사의 너트를 품는다). 고리를 바닥에 두고 뽑는다.

    b0은 고리가 받치는 베어링 뒷면이다 (J1은 허브가 두꺼워 고리가 더 길다, parts/base.py).
    """
    r_out = R_BORE - 0.15
    leg_in = SPOT_R - SCREW_LEN  # 나사 끝이 닿는 면 = 다리 안쪽 면 (15.0)
    # 원기둥 면의 이음선(방위각 0°)을 나사 구멍 자리에서 45° 돌려 둔다. 이음선 위에 구멍을 내면 형상이 깨진다
    part = solid(Rot(0, 45, 0) * ring(r_out, R_STEP + 1.3, RET_Y[1], b0))
    seg = solid(Rot(0, 45, 0) * cyl_y(r_out, RET_Y[0], RET_Y[1] + 0.01))
    legs = box(-8.0, 8.0, RET_Y[0], RET_Y[1] + 0.01, leg_in, 30)
    legs += box(-8.0, 8.0, RET_Y[0], RET_Y[1] + 0.01, -30, -leg_in)
    legs += box(leg_in, 30, RET_Y[0], RET_Y[1] + 0.01, -7.0, 7.0)
    part += seg & legs
    r = M3_FREE_H / 2
    for s in (1, -1):
        z0, z1 = (
            (leg_in - 0.01, leg_in + NUT_T + 0.3)
            if s > 0
            else (-leg_in - NUT_T - 0.3, -leg_in + 0.01)
        )
        part -= hex_z(0.0, RET_SCREW_Y, z0, z1, NUT_AF_UPRIGHT)
        part -= (
            cyl_z(r, 10, 25, 0.0, RET_SCREW_Y) if s > 0 else cyl_z(r, -25, -10, 0.0, RET_SCREW_Y)
        )
    part -= hex_x(RET_SCREW_Y, 0.0, leg_in - 0.01, leg_in + NUT_T + 0.3, NUT_AF_UPRIGHT)
    part -= cyl_x(r, 10, 25, RET_SCREW_Y, 0.0)
    # 앞면 라벨 판 위를 0.75 띄운다 (STS3215는 몸통이 0.17 앞이다). 고리(RET_Y[1]부터)는 그보다 위라 깎이지 않는다
    part -= box(-30, -9.0, 0, LABEL + 0.17 + 0.75, -7.7, 7.7)
    # 다리가 고리 안쪽으로 2.8 걸치지 않게, 다리 밑을 45° 원뿔로 깎는다.
    # 원뿔은 다리 안쪽 면보다 0.3 안에서 시작하고(면에 접하면 형상이 깨진다), 고리 속으로 0.05 올라간다
    r_ring = R_STEP + 1.3
    y_lo, y_hi = RET_Y[1] - (r_ring - leg_in) - 0.3, RET_Y[1] + 0.05
    cone = Cone(leg_in - 0.3, r_ring + 0.05, y_hi - y_lo)
    cut = Pos(0, (y_lo + y_hi) / 2, 0) * (Rot(-90, 0, 0) * cone)
    return solid(solid(part) - cut)


def carrier() -> Compound:
    """서보 받침: 서보 뒷면 M2 넷에 다는 틀 모양 판과 벽을 따라 선 날개 둘. 판 뒷면을 바닥에 두고 뽑는다.

    판은 뒷면의 볼록한 판과 커넥터를 비킨다. 가까운 쪽 가로대의 가운데는 터서, 플러그 옆면을 잡을 수 있다.
    M2 나사의 머리는 판 속 자리에 든다 (M2_SINK). 선은 먼 가로대 위를 어디로든 지나간다.
    """
    y0, y1 = L1 - PLATE_T, L1
    r_out = R_BORE - 0.15
    hw = HW - 0.06  # 목 안쪽 벽과 0.4
    plate = box(-35.0, -30.2, y0, y1, -hw, hw)  # 먼 가로대
    plate -= box(-36.0, -29.2, y0 - 1, y0 + WIRE_GROOVE, -8.6, 8.6)  # 선이 지나는 자리
    for sg in (1, -1):
        plate += box(
            -10.4, -6.1, y0, y1, min(sg * 7.5, sg * hw), max(sg * 7.5, sg * hw)
        )  # 나사 자리
        plate += box(-35.0, -6.1, y0, y1, min(sg * 10.0, sg * hw), max(sg * 10.0, sg * hw))  # 옆 살
        wing = box(-10.4, 10.0, y0, y1, min(sg * hw, sg * 30), max(sg * hw, sg * 30))
        plate += wing & cyl_y(r_out - 0.4, y0, y1)
    part = plate
    leaf_in = r_out - LEAF_T
    boss_in = SPOT_R - SCREW_LEN + 1.6  # 너트 자리 입구: 나사 끝이 너트를 1.6 지나간다
    r = M3_FREE_H / 2
    for sg in (1, -1):
        side = box(-10.0, 10.0, y1 - 0.01, CAR_Y[1], min(sg * 10, sg * 30), max(sg * 10, sg * 30))
        part += ring(r_out, leaf_in, y1 - 0.01, CAR_Y[1]) & side
        y_lo, y_hi = CAR_SCREW_Y - 4.6, CAR_SCREW_Y + 4.6
        bz = box(-4.6, 4.6, y_lo, y_hi, min(sg * boss_in, sg * 30), max(sg * boss_in, sg * 30))
        part += cyl_y(r_out, y_lo, y_hi) & bz
        # 판 뒷면을 바닥에 두고 뽑을 때 돋움 밑이 걸치지 않게 45°로 받친다
        dz = leaf_in - boss_in
        under = [
            (y_lo - dz, sg * leaf_in),
            (y_lo + 0.01, sg * leaf_in),
            (y_lo + 0.01, sg * boss_in),
        ]
        part += prism_x(under, -4.6, 4.6)
        z0, z1 = (
            (boss_in - 0.01, boss_in + NUT_T + 0.3)
            if sg > 0
            else (-boss_in - NUT_T - 0.3, -boss_in + 0.01)
        )
        part -= hex_z(0.0, CAR_SCREW_Y, z0, z1, NUT_AF_UPRIGHT)
        part -= (
            cyl_z(r, 10, 25, 0.0, CAR_SCREW_Y) if sg > 0 else cyl_z(r, -25, -10, 0.0, CAR_SCREW_Y)
        )
    # 뒷덮개 나사가 물리는 기둥: 나사 끝(덮개의 머리 자리에서 COVER_SCREW)보다 0.4 길다
    pillar = BACK_OUT + 1.2 + COVER_SCREW + 0.4
    for x, z in COVER_BOSS:
        part += cyl_y(3.2, y1 - 0.01, pillar, x, z)
        part -= cyl_y(M3_TAP / 2, y0 - 1, pillar + 1, x, z)
    for x, z in BACK_HOLES:
        part -= cyl_y(1.15, y0 - 1, y1 + 1, x, z)
        part -= cyl_y(M2_HEAD_R, y0 - 1, y0 + M2_SINK, x, z)  # 머리 자리
    return solid(part)


def back_cover(neck_x: float, end_skirt: bool) -> Compound:
    """뒷덮개: 판 + 원통 뒤 끝을 겹쳐 가리는 테 + 서보 받침에 닿는 나사 기둥 둘. 바깥 면을 바닥에 두고 뽑는다.

    서보 받침에 조이므로 받침을 따라 축 방향으로 조금 다르게 앉는다. 테가 그 틈을 가린다.
    end_skirt가 거짓이면 목 끝 쪽 테가 없다 (손목 몸통: 앞 관절의 출력 원판을 넘지 않는다).
    """
    y_skirt = BACK_END + COVER_SKIRT
    x_out = neck_x - 2.0 if end_skirt else neck_x
    x_in = neck_x - 0.4 if end_skirt else neck_x - 1.0
    outline = cyl_y(COVER_R, BACK_OUT, y_skirt)
    outline += box(x_out, -12.0, BACK_OUT, y_skirt, -NECK_HW - 2.0, NECK_HW + 2.0)
    inner = cyl_y(R_DRUM + 0.4, BACK_IN, y_skirt + 1)
    inner += box(x_in, -12.0, BACK_IN, y_skirt + 1, -NECK_HW - 0.4, NECK_HW + 0.4)
    c = solid(outline - inner)
    for x, z in COVER_BOSS:
        c = solid(c + cyl_y(3.2, BACK_IN - 0.01, L1 - PLATE_T, x, z))
        c = solid(c - cyl_y(M3_FREE_H / 2, BACK_OUT - 1, L1 - PLATE_T + 1, x, z))
        c = solid(c - cyl_y(3.1, BACK_OUT - 0.01, BACK_OUT + 1.2, x, z))  # 나사 머리 자리
    return c


# ------------------------------------------------------------------------------------------------
# 조립 검사에 쓰는 부품 목록


@dataclass(frozen=True)
class Part:
    name: str
    shape: Compound
    density: float = 0.0  # [g/mm³] 나사와 너트만 준다
    moving: bool = False  # 자식과 함께 도는 부품
    color: str = "body"  # 3D 페이지의 색 구실

    @property
    def mass_g(self) -> float:
        return self.shape.volume * self.density


def _screw_y(x: float, z: float, seat: float, length: float) -> Compound:
    """머리가 +y 쪽에 있는 M3 둥근머리 볼트 (머리 밑면이 seat)."""
    return solid(cyl_y(1.5, seat - length, seat, x, z) + cyl_y(2.85, seat, seat + 1.65, x, z))


def wall_screws() -> list[Part]:
    """벽 나사 M3 × 8 다섯: 1·3·5번은 누름 틀(앞줄), 2·4번은 서보 받침(뒷줄)."""
    out: list[Part] = []

    def one(axis: str, s: int, y: float) -> None:
        if axis == "z":
            if s > 0:
                shape = cyl_z(1.5, SPOT_R - SCREW_LEN, SPOT_R, 0.0, y) + cyl_z(
                    2.85, SPOT_R, SPOT_R + 1.65, 0.0, y
                )
            else:
                shape = cyl_z(1.5, -SPOT_R, -(SPOT_R - SCREW_LEN), 0.0, y)
                shape += cyl_z(2.85, -SPOT_R - 1.65, -SPOT_R, 0.0, y)
        else:
            shape = cyl_x(1.5, SPOT_R - SCREW_LEN, SPOT_R, y, 0.0) + cyl_x(
                2.85, SPOT_R, SPOT_R + 1.65, y, 0.0
            )
        out.append(Part(f"벽 나사 {len(out) + 1}", solid(shape), STEEL, color="metal"))

    for s in (1, -1):
        one("z", s, RET_SCREW_Y)
        one("z", s, CAR_SCREW_Y)
    one("x", 1, RET_SCREW_Y)
    return out


def inner_hardware(model: ServoModel) -> list[Part]:
    """혼 볼트, 멈춤 나사, 너트, 서보 뒷면 나사, 덮개 나사 (검사와 3D 페이지용)."""
    dy = servo_dy(model)
    out: list[Part] = []
    for k, (x, z) in enumerate(bolt_xz()):
        out.append(Part(f"혼 볼트 M3x20 {k + 1}", _screw_y(x, z, ATOP, 20.0), STEEL, True, "metal"))
    out.append(
        Part("멈춤 나사 M3x10", _screw_y(R_POST, 0.0, ATOP - 1.8, 10.0), STEEL, True, "metal")
    )
    nut = hex_y(R_POST, 0.0, POST_TIP - NUT_T, POST_TIP, 5.4) - cyl_y(
        1.5, POST_TIP - 3, POST_TIP + 1, R_POST, 0.0
    )
    out.append(Part("너트 (멈춤 기둥)", solid(nut), STEEL, True, "metal"))
    for k, (x, z) in enumerate(RIM):
        nut = hex_y(x, z, SEAM, SEAM + NUT_T, 5.4) - cyl_y(1.5, SEAM - 1, SEAM + NUT_T + 1, x, z)
        out.append(Part(f"너트 (출력 원판) {k + 1}", solid(nut), STEEL, True, "metal"))
    for k, (x, z) in enumerate(BACK_HOLES):
        y = L1 - PLATE_T + M2_SINK + dy  # 머리 밑면 (판 속 자리의 바닥)
        shape = cyl_y(1.0, y, y + M2_SCREW, x, z) + cyl_y(1.9, y - 2.0, y, x, z)
        out.append(Part(f"M2x4 {k + 1}", solid(shape), STEEL, False, "metal"))
    for k, (x, z) in enumerate(COVER_BOSS):
        y = BACK_OUT + 1.2 + dy
        shape = cyl_y(1.5, y, y + COVER_SCREW, x, z) + cyl_y(2.85, y - 1.65, y, x, z)
        out.append(Part(f"덮개 나사 M3x12 {k + 1}", solid(shape), STEEL, False, "metal"))
    leg_in = SPOT_R - SCREW_LEN
    boss_in = leg_in + 1.6
    for sg in (1, -1):
        z0, z1 = (leg_in, leg_in + NUT_T) if sg > 0 else (-leg_in - NUT_T, -leg_in)
        out.append(
            Part(
                "너트 (누름 틀)", hex_z(0.0, RET_SCREW_Y, z0, z1, 5.4), STEEL * 0.72, False, "metal"
            )
        )
        z0, z1 = (boss_in, boss_in + NUT_T) if sg > 0 else (-boss_in - NUT_T, -boss_in)
        nut_c = solid(Pos(0, dy, 0) * hex_z(0.0, CAR_SCREW_Y, z0, z1, 5.4))
        out.append(Part("너트 (서보 받침)", nut_c, STEEL * 0.72, False, "metal"))
    nut_x = hex_x(RET_SCREW_Y, 0.0, leg_in, leg_in + NUT_T, 5.4)
    out.append(Part("너트 (누름 틀)", nut_x, STEEL * 0.72, False, "metal"))
    return out


def inner_parts(model: ServoModel, neck_x: float, end_skirt: bool) -> list[Part]:
    """원통 속 부품과 서보, 출력 원판, 벽 나사 (몸통과 자식 몸통은 빼고)."""
    dy = servo_dy(model)
    sv = servo(model)
    parts = [
        Part("베어링 6806", bearing(), color="metal"),
        Part("허브 판", hub_plate(), moving=True, color="cap"),
        Part("출력 원판", out_disc(), moving=True, color="cap"),
        Part("누름 틀", retainer(), color="frame"),
        Part("서보 받침", solid(Pos(0, dy, 0) * carrier()), color="frame"),
        Part("뒷덮개", solid(Pos(0, dy, 0) * back_cover(neck_x, end_skirt)), color="cap"),
        Part("서보 몸통", sv["body"], color="servo"),
        Part("혼", sv["horn"], moving=True, color="metal"),
        Part("플러그", sv["plugs"], color="plug"),
        Part("선 자리", sv["wires"], color="wire"),
    ]
    return parts + wall_screws()


# 서로 닿거나 겹쳐 그린 것으로 치는 쌍 (서보 안의 조각끼리)
_SAME_BODY = {
    frozenset(p) for p in (("서보 몸통", "혼"), ("플러그", "서보 몸통"), ("플러그", "선 자리"))
}


def overlaps(parts: list[Part], tol: float = 0.05) -> dict[tuple[str, str], float]:
    """출력 부품과 서보끼리 겹친 부피 [mm³] (tol 넘는 것만). 나사와 너트는 구멍 속에 있어 보지 않는다."""
    solids = [p for p in parts if p.density == 0.0]
    out: dict[tuple[str, str], float] = {}
    for i, a in enumerate(solids):
        for b in solids[i + 1 :]:
            if frozenset((a.name, b.name)) in _SAME_BODY:
                continue
            if far_apart(a.shape, b.shape, 0.0):
                continue
            v = solid(a.shape & b.shape).volume if a.shape.distance_to(b.shape) < 1e-6 else 0.0
            if v > tol:
                out[(a.name, b.name)] = v
    return out
