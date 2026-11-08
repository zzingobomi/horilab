"""팔꿈치(J3) 관절 부품. 관절 하나 시험(docs/decisions/0021-elbow-joint-test.md)에 그대로 쓴다.

서보 좌표계(mm)로 그린다: x는 출력축에서 서보의 가까운 끝 쪽, y는 출력축 방향(출력 혼 쪽), z = x × y.
원점은 출력축 위, y = 0은 서보 뒷면에서 14.5 mm 앞이다 (studies/packaging.py와 같다).
팔에 달면 x가 위팔 방향(위), y가 링크 좌표계의 y다. 자식 부품(원판)은 J3 = 0(아래팔이 위팔과 한 줄)일
때의 자리로 그리고, 관절 각 q만큼 +y 둘레로 돌린다. 그때 아래팔 방향은 (cos q, 0, -sin q)다.

구조 (0021):
- 부모(위팔): 서보 집이 서보를 통째로 감싼다. 뒷면 쪽이 열려 있어 서보를 뒤에서 밀어 넣고, 출력 면의
  M2 나사 넷으로 잡는다. 아래쪽 꽂이가 위팔 관(시험에서는 받침 기둥)에 꽂힌다. 흰 고리가 원통 가운데를
  덮는다.
- 자식(아래팔): 양쪽 혼에 원판을 하나씩 단다. 원판은 원통의 양 끝 뚜껑을 겸한다. 원판의 혀 두 개 사이에
  아래팔 뿌리(시험에서는 시험 링크)를 볼트로 끼운다.
- 서보 뒷면과 뒷면 원판 사이(약 9 mm)는 커넥터와, J3를 지나는 케이블의 고리가 지나가는 자리다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Compound

from horilab_design.parts.shapes import box, cone_x, cyl_x, cyl_y, nut_trap, slot, solid
from horilab_design.values import Filament, PrintFit, load_measured, load_servos

SERVO = "STS3250"
FILAMENT: Filament = "pla_basic_white"  # 시험 부품은 공차를 잰 흰색으로 뽑는다


@dataclass(frozen=True)
class Fits:
    """공차 시험편으로 고른 값 [mm]. 3차 시험편 값이 아직 없으면 임시 값을 쓰고 provisional에 적는다."""

    seat_clearance: float  # 서보 몸체 단면의 길이·폭에 더하는 여유 (양쪽 합)
    m3_hole: float
    m3_hole_horizontal: float
    m3_tap: float
    nut_width: float
    nut_height: float
    pin_hole: float  # 세로 구멍 (원판)
    pin_hole_horizontal: float  # 가로 구멍 (집: 세워 뽑는다)
    provisional: tuple[str, ...]


# 3차 시험편(영점 핀)을 재기 전의 임시 값. 가로 구멍은 1차 시험편처럼 세로보다 0.2 크게 잡는다.
# 핀 구멍이 들어간 부품(집, 출력 원판)은 시험편 결과를 넣은 뒤에 뽑는다.
PROVISIONAL_PIN_HOLE = 3.10
PROVISIONAL_PIN_HOLE_HORIZONTAL = 3.30
M2_HOLE = 2.6  # M2 나사 구멍. 시험하지 않아서 도면 위치 오차까지 넉넉히 둔다
M2_HEAD_D, M2_HEAD_DEPTH = 4.4, 1.6  # M2 둥근머리 십자볼트(머리 지름 약 3.5~4, 높이 1.3~1.6) 자리
BUTTON_D = 6.0  # M3 둥근머리 렌치볼트(ISO 7380, 머리 지름 5.7, 높이 1.65) 자리


def load_fits(filament: Filament = FILAMENT) -> Fits:
    fit: PrintFit = load_measured().fit[filament]
    provisional: list[str] = []

    def need(value: float | None, name: str) -> float:
        if value is None:
            raise ValueError(f"{filament}의 {name}이 data/measured.toml에 없다")
        return value

    def third(value: float | None, name: str, fallback: float) -> float:
        if value is None:
            provisional.append(name)
            return fallback
        return value

    pin = third(fit.pin_hole_mm, "pin_hole_mm", PROVISIONAL_PIN_HOLE)
    pin_h = third(
        fit.pin_hole_horizontal_mm, "pin_hole_horizontal_mm", PROVISIONAL_PIN_HOLE_HORIZONTAL
    )
    return Fits(
        seat_clearance=need(fit.seat_clearance_mm.get(SERVO), "seat_clearance_mm"),
        m3_hole=need(fit.m3_hole_mm, "m3_hole_mm"),
        m3_hole_horizontal=need(fit.m3_hole_horizontal_mm, "m3_hole_horizontal_mm"),
        m3_tap=need(fit.m3_tap_hole_mm, "m3_tap_hole_mm"),
        nut_width=need(fit.m3_nut_slot_width_mm, "m3_nut_slot_width_mm"),
        nut_height=need(fit.m3_nut_slot_height_mm, "m3_nut_slot_height_mm"),
        pin_hole=pin,
        pin_hole_horizontal=pin_h,
        provisional=tuple(provisional),
    )


FITS = load_fits()

# --- 서보 (STS3250, 도면과 실측: docs/hardware.md의 "기구 치수") --------------------------------
_spec = load_servos()[SERVO]
_body = load_measured().servo_body[SERVO]
NEAR = _spec.shaft_to_end_mm  # 축 → 가까운 끝
FAR = _spec.body_length_mm - NEAR  # 축 → 먼 끝
HW = _spec.body_width_mm / 2
BACK = -14.5  # 뒷면 (서보 좌표계의 정의)
FRONT = BACK + 32.0  # 앞면 (출력 면). 몸통 두께 32는 도면 값이고 먼 쪽 실측과 맞는다
PAD_X = -15.9  # 뒷면 판이 시작하는 자리 (먼 끝까지)
PAD_Y = FRONT - _body.far_thickness_mm
HORN_TOP = BACK + _body.horn_to_back_mm
HORN_R = _spec.horn_diameter_mm / 2
HORN_T = 4.5
HORN_BOLT_R = _spec.horn_bolt_circle_mm / 2
BACK_HORN_T = 3.1
# 뒷면 혼은 돌기(지름 6, 4.1 나옴)에 끼워 돌고, 뒷면과 돌기 끝 나사 머리 사이에서 약 1 mm 움직인다.
# 그 가운데(뒷면에서 0.5 mm)에 둔다. 원판 간격이 출력 오차로 조금 달라도 혼이 미끄러져 받아 준다.
BACK_HORN = (BACK - 0.5 - BACK_HORN_T, BACK - 0.5)
BOSS_TIP = BACK - 4.1
# 커넥터 두 개(사진으로 어림): 축에서 먼 쪽으로 10~16 mm, 꽂은 플러그와 꺾인 선이 뒷면에서 6 mm까지
CONNECTOR_X = (-16.0, -10.0)
CONNECTOR_Y = BACK - 6.0
FRONT_HOLES = ((-8.30, 10.25), (-8.30, -10.25), (-29.00, 10.25), (-29.00, -10.25))

# --- 팔꿈치 배치 [mm] -------------------------------------------------------------------------
WALL = 2.6  # 0.20mm Strength 벽 6겹
MOVING_GAP = 2.0  # 서로 움직이는 부품 사이 (0017)
T_OUT = 3.2  # 출력 쪽 벽: M2 머리를 묻고 1.6 mm가 남는다
Y_OUT = FRONT + T_OUT  # 출력 쪽 벽 바깥 면
DISC_T = 5.0
OUT_HUB = 3.8  # 출력 혼 윗면 → 출력 원판 안쪽 면. 벽과 원판 사이가 움직이는 틈만큼 된다
BACK_HUB = 5.6  # 뒷면 혼 바깥 면 → 뒷면 원판 안쪽 면. 커넥터와 케이블 고리 자리
Y_DISC_OUT = HORN_TOP + OUT_HUB
Y_DISC_BACK = BACK_HORN[0] - BACK_HUB
R_CYL = 29.7  # 원통 바깥 반지름 (0019의 팔꿈치 원통 지름 59.4)
RING_T = 1.6  # 덮개 벽 (0019)
R_RING_IN = R_CYL - RING_T
HUB_R = 9.4  # 혼 나사 넷을 감싸고 커넥터(축에서 10 mm부터)를 비키는 반지름
HORN_HOLE_R = HORN_R + 0.35  # 출력 쪽 벽 구멍: 서보를 넣을 때 혼이 지나간다
SEAT = FITS.seat_clearance / 2
# 집은 x를 세워 뽑아서, 서보 가까운 끝 위 지붕은 허공에 걸쳐 뽑히고 조금 처진다. 그쪽에 여유를 더 두고,
# 서보는 정확하게 뽑히는 바닥(먼 쪽 벽 윗면)과 옆벽이 잡는다. 조립할 때 서보를 바닥에 닿게 누르고 조인다.
ROOF_SAG = 0.4
CX0, CX1, CZ = -FAR - SEAT, NEAR + SEAT + ROOF_SAG, HW + SEAT  # 서보 자리 (x 범위, z 반폭)
X_BOT = CX0 - WALL
X_ROOF = CX1 + WALL
Z_SIDE = CZ + WALL
# 집 아래: 45° 깔때기가 집 바닥을 관 바깥지름(어깨)까지 줄이고, 그 아래 꽂이가 관 속으로 들어간다.
# 관 끝은 어깨에 닿고, 관 끝의 갈라진 고리를 나사로 죄어 꽂이를 문다 (시험에서는 받침 기둥 위끝).
TUBE_OD, TUBE_ID = 40.0, 34.8  # 위팔 관 (0019: 지름 40, 벽 2.6)
SHOULDER_R = TUBE_OD / 2
FUNNEL_H = 5.8  # 집 바닥 모서리(축에서 약 25.6)까지 45°로 넓히는 높이
X_SHOULDER = X_BOT - FUNNEL_H
SPIGOT_R = TUBE_ID / 2 - 0.15
SPIGOT_BORE, SPIGOT_L = 12.0, 15.0
Y_BACK_EDGE = PAD_Y - 0.3  # 먼 쪽 옆벽이 뒷면 판을 덮는 끝
PIN_XZ = (22.0, 0.0)  # 영점 핀 (J3 = 0에서 원판 구멍과 맞는다)
PIN_DEPTH = 8.0
BAR_X = (X_ROOF - 1.0, R_RING_IN - 0.1)
BAR_Y = (-10.0, Y_OUT)
BAR_Z = 5.0
RING_SCREWS_Y = (-4.0, 8.0)
CABLE_A_Z = 5.5  # 케이블 A 길의 반폭 (5264 플러그가 약 10 × 5 mm)
TONGUE_W, TONGUE_R = 26.0, 46.0
TONGUE_BOLTS = ((39.0, -8.0), (39.0, 8.0))
OUT_SCREW_HEAD = 1.8  # 출력 원판의 혼 나사 머리 자리 깊이 (M3 × 10이 혼에 3.0 mm 들어간다)
BACK_SCREW_HEAD = 3.0  # 뒷면 원판의 혼 나사 머리 자리 깊이 (M3 × 10이 혼에 2.4 mm 들어간다)


def _nut(
    center: tuple[float, float, float],
    axis: tuple[float, float, float],
    slide: tuple[float, float, float],
    length: float,
) -> Compound:
    return nut_trap(center, axis, slide, length, FITS.nut_width, FITS.nut_height)


# ------------------------------------------------------------------------------------------------
# 검사용 서보 외형 (출력하지 않는다)


def servo() -> Compound:
    """서보 몸통, 두 혼, 뒷면 돌기와 혼을 잡는 나사."""
    s = box(-FAR, NEAR, BACK, FRONT, -HW, HW)
    s += box(-FAR, PAD_X, PAD_Y, BACK, -HW, HW)
    s -= cyl_y(11.0, FRONT - 3.1, FRONT + 1)  # 혼이 앉는 오목한 자리 (어림)
    s += cyl_y(HORN_R, HORN_TOP - HORN_T, HORN_TOP)
    s += cyl_y(2.8, HORN_TOP, HORN_TOP + 2.0)  # 출력 혼 가운데 나사 머리
    s += cyl_y(3.0, BOSS_TIP, BACK)
    s += cyl_y(HORN_R, *BACK_HORN)
    s += cyl_y(3.5, BOSS_TIP - 2.5, BOSS_TIP)  # 뒷면 혼을 잡는 나사와 와셔 (어림)
    return solid(s)


def plugs() -> Compound:
    """꽂은 커넥터와 꺾인 선이 차지하는 자리 (어림)."""
    return box(CONNECTOR_X[0], CONNECTOR_X[1], CONNECTOR_Y, BACK, -11.5, 11.5)


# ------------------------------------------------------------------------------------------------
# 부모 (위팔 쪽)


def house() -> Compound:
    """서보 집 (검정). 출력 쪽 벽, 양 옆벽, 먼 쪽 벽, 가까운 쪽 지붕, 영점 핀 막대, 깔때기와 꽂이.

    출력 방향: 꽂이를 바닥에 두고 x를 위로 세운다. 서보 자리는 뒷면 쪽(-y)으로 열려 있어 서보를 뒤에서
    밀어 넣는다. 깔때기(45°)가 집 바닥을 받쳐 서포트 없이 뽑힌다.
    """
    outer = box(X_BOT, X_ROOF, BACK, Y_OUT, -Z_SIDE, Z_SIDE)
    outer += box(
        X_BOT, -24.0, Y_BACK_EDGE, BACK, -Z_SIDE, Z_SIDE
    )  # 먼 쪽 옆벽이 뒷면 판까지 덮는다
    outer += box(*BAR_X, *BAR_Y, -BAR_Z, BAR_Z)  # 영점 핀 막대
    footprint = box(X_SHOULDER, X_BOT, Y_BACK_EDGE, Y_OUT, -Z_SIDE, Z_SIDE)
    outer += footprint & cone_x(SHOULDER_R, SHOULDER_R + FUNNEL_H, X_SHOULDER, X_BOT)
    outer += cyl_x(SPIGOT_R, X_SHOULDER - SPIGOT_L, X_SHOULDER)
    part = outer - box(CX0, CX1, -60.0, FRONT, -CZ, CZ)
    # 꽂이 속: 케이블이 관으로 내려가는 길. 위는 45°로 좁혀 다리 없이 뽑힌다.
    part -= cyl_x(SPIGOT_BORE, X_SHOULDER - SPIGOT_L - 1, X_SHOULDER - 2.0)
    part -= cone_x(SPIGOT_BORE, 4.0, X_SHOULDER - 2.0, X_SHOULDER - 2.0 + SPIGOT_BORE - 4.0)
    # 출력 쪽 벽의 혼 구멍: 위는 지붕까지 곧게 터서 서보를 넣을 때 혼이 걸리지 않는다
    part -= cyl_y(HORN_HOLE_R, FRONT - 1, Y_OUT + 1)
    part -= box(0.0, CX1, FRONT - 1, Y_OUT + 1, -HORN_HOLE_R, HORN_HOLE_R)
    for x, z in FRONT_HOLES:  # 서보 출력 면의 M2 나사
        part -= cyl_y(M2_HOLE / 2, FRONT - 1, Y_OUT + 1, x, z)
        part -= cyl_y(M2_HEAD_D / 2, Y_OUT - M2_HEAD_DEPTH, Y_OUT + 1, x, z)
    part -= cyl_y(FITS.pin_hole_horizontal / 2, Y_OUT - PIN_DEPTH, Y_OUT + 1, *PIN_XZ)
    for s in (1, -1):  # 옆 창: 서보 옆면의 바람길
        part -= box(-27.0, -3.0, BACK + 4.0, FRONT - 4.0, s * Z_SIDE - 4, s * Z_SIDE + 4)
    for y in RING_SCREWS_Y:  # 고리 나사: 막대 위에서 -x로, 너트는 +z 옆에서
        part -= cyl_x(FITS.m3_hole / 2, BAR_X[1] - 9.0, BAR_X[1] + 1, y=y)
        part -= _nut((BAR_X[1] - 4.5, y, 0.0), (1, 0, 0), (0, 0, 1), BAR_Z + 1)
    # 케이블 A: 서보 먼 끝 뒤에서 깔때기를 비스듬히 지나 꽂이 속으로 (5264 플러그가 지나가는 폭)
    part -= slot(
        (X_BOT + 0.5, Y_BACK_EDGE - 1.5, 0.0),
        (X_SHOULDER - 8.0, -8.0, 0.0),
        3.0,
        CABLE_A_Z - 3.0,
        (0.0, 0.0, 1.0),
    )
    return solid(part)


def ring() -> Compound:
    """가운데 고리 (흰색 덮개). 아래가 열린 C자라 위에서 내려 끼운다. 축을 세워 뽑는다."""
    band = cyl_y(R_CYL, Y_DISC_BACK + MOVING_GAP, Y_OUT) - cyl_y(R_RING_IN, -60.0, 60.0)
    half = Z_SIDE + 0.5
    band -= box(-60.0, -15.0, -60.0, 60.0, -half, half)  # 집이 원통 밖으로 나가는 자리
    for y in RING_SCREWS_Y:
        band -= cyl_x(FITS.m3_hole_horizontal / 2, 20.0, 40.0, y=y)
    return band


# ------------------------------------------------------------------------------------------------
# 자식 (아래팔 쪽)


def disc(side: str) -> Compound:
    """혼에 다는 원판 (검정). side는 "out"(출력 혼) 또는 "back"(뒷면 혼). J3 = 0일 때 혀가 +x다.

    출력 방향: 바깥 면을 바닥에 두고 허브가 위로 오게 납작하게 뽑는다.
    """
    out = side == "out"
    if out:
        y_in, y_out = Y_DISC_OUT, Y_DISC_OUT + DISC_T
        hub = (HORN_TOP, y_in)
        head_depth = OUT_SCREW_HEAD
    else:
        y_in, y_out = Y_DISC_BACK, Y_DISC_BACK - DISC_T
        hub = (y_in, BACK_HORN[0])
        head_depth = BACK_SCREW_HEAD
    ya, yb = sorted((y_in, y_out))
    d = cyl_y(R_CYL, ya, yb)
    d += box(0.0, TONGUE_R, ya, yb, -TONGUE_W / 2, TONGUE_W / 2)
    d += cyl_y(HUB_R, min(hub), max(hub))
    # 가운데 나사 머리 자리. 나사 종류(냄비머리·원통머리, 와셔)를 아직 보지 못해 넉넉히 판다.
    if out:  # 출력 혼 가운데 나사 머리 (원통머리 높이 3까지)
        d -= cyl_y(3.5, HORN_TOP - 1, HORN_TOP + 3.4)
    else:  # 돌기 끝(혼 위 0.5), 와셔(지름 8.6까지), 나사 머리
        d -= cyl_y(4.5, BACK_HORN[0] - 4.2, BACK_HORN[0] + 1)

    def counterbore(x: float, z: float, depth: float) -> Compound:
        if out:
            return cyl_y(BUTTON_D / 2, y_out - depth, y_out + 1, x, z)
        return cyl_y(BUTTON_D / 2, y_out - 1, y_out + depth, x, z)

    for k in range(4):  # 혼의 M3 넷 (지름 14 원)
        a = math.pi / 4 + k * math.pi / 2
        x, z = HORN_BOLT_R * math.cos(a), HORN_BOLT_R * math.sin(a)
        d -= cyl_y(FITS.m3_hole / 2, -60.0, 60.0, x, z)
        d -= counterbore(x, z, head_depth)
    if out:
        d -= cyl_y(FITS.pin_hole / 2, -60.0, 60.0, *PIN_XZ)
    for x, z in TONGUE_BOLTS:
        d -= cyl_y(FITS.m3_hole / 2, -60.0, 60.0, x, z)
        d -= counterbore(x, z, 1.8)
    if not out:  # 케이블 B를 혀에 묶는 끈 구멍
        for z in (-4.0, 4.0):
            d -= cyl_y(1.6, -60.0, 60.0, 33.0, z)
    return solid(d)
