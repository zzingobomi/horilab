"""평행 그리퍼 (첫 그리퍼, docs/design.md의 "도구 순서", 0030).

STS3215 하나가 피니언을 돌려 랙 둘을 반대로 밀고, 손가락 둘이 도구 축을 가운데로 대칭으로 벌어지고 오므라든다.
박스는 윗면 구멍에 손가락 끝을 넣고 벌려 잡고(0014), 가벼운 물체는 손가락 안쪽 면으로 바깥에서 잡는다.

짜임 (플랜지 면에서 도구 쪽으로):
- 몸통: 플랜지에 앉는 판(도구 어댑터 판과 같은 구멍)과 서보를 품는 틀이 한 부품이다. 판의 -x 쪽을 터서
  서보 뒷면 커넥터에 선을 꽂는다 (도구를 플랜지에 달기 전에 꽂는다). 틀의 -x 쪽 밑은 45°로 깎여 있다:
  J6가 돌 때 그 밑이 앞 관절(J5) 출력 원판의 멈춤 귀 끝 위를 지나간다 (RELIEF_X).
- 서보는 세워 둔다 (출력축이 도구 축, 혼이 손가락 쪽). 몸통의 누름 자리와 바닥판 사이에 물린다.
- 바닥판 밑이 통로다. 통로의 위층에 피니언과 랙의 이가 있고, 아래층에 손가락 받침의 판이 있다.
  받침(랙 팔 + 판)은 바닥판과 덮개 사이에 끼여 x로만 미끄러진다. 랙 팔이 길어서 손가락이 받는 힘에 받침이
  기울지 않는다.
- 랙 집: 통로의 옆벽, 끝벽, 덮개가 한 부품이다. 덮개 가운데의 긴 틈으로 손가락의 목이 나온다.
- 손가락: 받침의 판에 나사 하나로 단다. 덮개를 사이에 두고 판(안)과 손가락 어깨(밖)가 덮개를 문다.

좌표는 joint.py의 서보 좌표다 (y가 J6 축, 플랜지 면이 y = ATOP, 손가락은 x로 벌어진다, D405는 -z 쪽).
높이 H는 플랜지 면에서 도구 쪽으로 잰 거리다 (y = ATOP + H).

    uv run python -m horilab_design.parts.gripper
"""

from __future__ import annotations

import math

from build123d import Axis, Compound, Cone, Polygon, Pos, Rot, extrude

from horilab_design.parts import flange, wrist
from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_y, far_apart, nut_trap, solid
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.values import ServoModel

MODEL: ServoModel = "STS3215"
Pt = tuple[float, float]

# ---- 이 (모듈 1.25, 20개: 혼 볼트 머리 넷이 이뿌리 원 안에 든다) ----
MODULE = 1.25
TEETH = 20
PA = math.radians(20.0)  # 압력각
RP = MODULE * TEETH / 2  # 피치 반지름 12.5
ADD, DED = MODULE, 1.25 * MODULE
PITCH = math.pi * MODULE
MESH_GAP = 0.2  # 랙의 피치선을 피니언에서 띄운 거리 (이 사이 틈 약 0.15)
PITCH_Z = RP + MESH_GAP  # 랙의 피치선
RACK_TIP_Z = PITCH_Z - ADD  # 11.45
RACK_ROOT_Z = PITCH_Z + DED  # 14.26
BACKING = 2.2  # 이뿌리 뒤의 살
ARM_BACK_Z = RACK_ROOT_Z + BACKING  # 랙 팔의 바깥 면 16.46
LANE_GAP = 0.25  # 랙 팔과 옆벽 사이
WALL_IN = ARM_BACK_Z + LANE_GAP  # 옆벽 안쪽 면
TUB_WALL = 1.6
HOUSE_HW = WALL_IN + TUB_WALL  # 랙 집의 반폭 18.31

# ---- 높이 H (플랜지 면에서) ----
ADAPTER_T = flange.ADAPTER_T  # 6.2
BACK_GAP = 2.0  # 판 밑면과 서보 뒷면 사이 (서보 뒷면 보스는 판 속 자리에 든다)
H_BACK = ADAPTER_T + BACK_GAP  # 서보의 두께 32인 뒷면 8.2
H_PLUG = H_BACK - (j.L1 - j.PLUG_TOP)  # 꽂은 플러그의 뒤 끝 (뒷면에서 2.6 나온다) 5.6
H_FRONT = H_BACK + (j.F32 - j.L1)  # 서보의 두께 32인 앞면 = 바닥판 윗면 40.2
H_HORN = H_BACK + (j.HORN_TOP[MODEL] - j.L1)  # 혼 윗면 42.91 (바닥판 밑면보다 0.3 나온다)
FLOOR_T = 2.4
H_FLOOR = H_FRONT + FLOOR_T  # 바닥판 밑면 = 통로 천장 42.6
SLIDE_GAP = 0.1  # 받침과 통로의 위아래 틈 (한쪽)
RACK_H = 6.2  # 통로의 위층: 이
PLATE_T = 4.0  # 통로의 아래층: 받침의 판
H_ARM0 = H_FLOOR + SLIDE_GAP  # 랙 팔 윗면 42.7
H_PLATE0 = H_ARM0 + RACK_H  # 판 윗면 48.9
H_ARM1 = H_PLATE0 + PLATE_T  # 받침 밑면 52.9
H_COVER = H_ARM1 + SLIDE_GAP  # 덮개 윗면 53.0
COVER_T = 2.4
H_BOTTOM = H_COVER + COVER_T  # 몸통 밑면 55.4
PIN_FACE = (H_FLOOR + 0.4, H_PLATE0 - 0.4)  # 피니언 이의 높이 범위 43.0 ~ 48.5
HUB_R = 10.0  # 피니언이 혼에 앉는 목 (혼 볼트 구멍 둘레를 덮는다)
PINION_SCREWS = (8.0, 6.0)  # 혼 볼트로 쓸 길이 후보 (가진 것)
HEAD_REC = 1.95  # 나사 머리(높이 1.65)가 면 밑으로 들어가는 깊이
# 피니언 밑의 혼 가운데 나사 머리 자리: 반지름과 깊이 (머리는 지름 5.15, 혼 윗면 위로 2.0까지)
HORN_HEAD_REC = (3.3, j.HEAD_H + 0.5)

# ---- 손가락 ----
STROKE = 30.0  # 손가락 하나가 가는 거리 (열림 폭 60 mm)
OPEN_MAX = 2 * STROKE
LC = 10.0  # 받침의 판과 손가락 뿌리의 길이 (x)
ARM_U = (
    -(STROKE + 7.5),
    LC,
)  # 랙 팔의 범위 (손가락 안쪽 면에서): 다 벌려도 피니언에 이 셋이 물린다
ARM_LOW_Z = 10.8  # 랙 팔 아랫부분의 안쪽 면 (맞은편 받침의 판과 0.5 띄운다)
PLATE_EDGE = 10.3  # 판이 맞은편 쪽으로 뻗은 끝
SLOT_HW = 3.2  # 덮개 틈의 반폭
TENON_HW = 3.0  # 손가락 목의 반폭
SHOULDER_GAP = 0.2  # 손가락 어깨와 덮개 밑면 사이
H_ROOT = H_BOTTOM + SHOULDER_GAP  # 손가락 어깨 55.6
FINGER_LEN = (
    40.0  # 몸통 밑면에서 손가락 끝까지. D405가 잡기 전 높이에서 도구 끝 둘레를 80% 넘게 보는 길이
)
H_TIP = H_BOTTOM + FINGER_LEN  # 95.4
FINGER_HW = 6.5  # 손가락의 반폭 (z)
SHANK_U = 8.0  # 손가락 줄기의 두께 (x)
ROOT_H = 9.0  # 뿌리(두께 LC)의 높이
FINGER_SCREW_U = 5.0  # 손가락 나사의 자리
FINGER_SCREW = 12.0  # M3 × 12
FINGER_NUT = (58.2, 61.0)  # 손가락 속 너트 홈의 높이 범위
# 손가락 끝: 박스 구멍(BOX_*)에 들어가 벌려 잡는다
TIP_LEN = 9.5  # 어깨(박스 윗면에 닿는 면)에서 끝까지
H_SHOULDER = H_TIP - TIP_LEN  # 85.9
TIP_HW = 4.0  # 끝의 반폭. 턱의 양 모서리가 구멍의 원뿔에 닿는다
NECK_HW = 3.6  # 목(구멍 입구를 지나는 곳)의 반폭
NECK_U = 2.8  # 목의 두께
BARB_U = 5.4  # 턱의 가장 두꺼운 곳
BARB_DEPTH = 5.3  # 어깨에서 턱의 가장 두꺼운 곳까지
BARB_SLOPE = math.radians(
    55.0
)  # 턱 윗면이 세로에서 누운 각. 구멍의 원뿔(45°)보다 누워서 턱의 끝 모서리가 닿는다
TIP_END_U = 3.0  # 끝의 두께 (끝으로 가며 45°로 얇아진다)
TCP_UP = 5.0  # 도구 끝점(TCP)이 손가락 끝에서 올라온 거리
H_TCP = H_TIP - TCP_UP  # 90.4

# ---- 박스 윗면 구멍 (0014). 박스는 구멍을 위로 두고 뽑는다: 원뿔이 45°라 받침 없이 뽑힌다 ----
BOX_LIP_R = 8.0  # 입구 반지름 (지름 16)
BOX_LIP_T = 3.0  # 입구의 곧은 깊이
BOX_CAVITY_R = 11.2  # 속의 반지름 (입구 밑에서 45°로 넓어진다)
BOX_HOLE_DEPTH = 12.0

# ---- 몸통 ----
SERVO_GAP = 0.3  # 서보 둘레 여유
CRADLE_X = (-j.FAR - SERVO_GAP, j.NEAR + SERVO_GAP)  # 틀 안쪽 (-35.41, 10.41)
CRADLE_Z = j.HW + SERVO_GAP  # 12.66
WALL = 2.0
# 틀의 서보 먼 끝 쪽 밑을 45°로 깎기 시작하는 자리 (축에서). 앞 관절(J5) 출력 원판의 멈춤 귀 끝이 플랜지 면
# 바로 밑(0.6 mm)까지, 축에서 37.7 mm부터 온다. J6가 돌 때 틀의 밑이 그 위를 2 mm 넘게 띄우고 지나가게 한다
RELIEF_X = 31.0
NOTCH_X = -10.8  # 판을 튼 자리의 안쪽 끝 (커넥터 구멍은 x -16.3 ~ -11.7)
NOTCH_HW = 10.5
PAD_X = (-18.5, -6.3)  # 서보 뒷면(두께 32인 면)을 누르는 자리
BOSS_REC = (3.7, 2.7)  # 서보 뒷면 보스가 드는 자리: 반지름, 깊이
PLUG_Z = (0.45, 9.46)  # 꽂는 커넥터 (+z 쪽 것)
# 몸통과 바닥판, 랙 집을 묶는 나사
FS = (17.2, 7.5)  # +x 쪽 바닥판 나사 (밑에서, M3 × 8): 머리가 통로 위층의 빈 가운데에 있다
BOSS_X1 = 21.2  # 그 너트를 품는 돋움의 바깥 끝
BOSS_H0 = 33.2
BOSS_NUT_H = 36.5  # 너트 홈의 가운데 높이
END_T = 4.0  # 랙 집의 끝벽
IN_HL = STROKE + LC + 0.3  # 통로 안쪽 반 길이 40.3
HOUSE_HL = IN_HL + END_T  # 랙 집의 반 길이 44.3
TS = (IN_HL + END_T / 2, 7.5)  # 끝벽 나사 (위에서, 끝벽의 플라스틱에 조인다)
LUG_H0 = H_FRONT - 4.0  # -x 쪽 귀의 윗면 (나사 머리가 앉는다)
LUG_HW = 11.5
TUB_CHAMFER = 2.0  # 랙 집 밑 긴 모서리를 깎은 길이 (D405가 그 모서리 너머로 본다)
SLOT_NUT = (5.6, 2.8)  # 옆에서 밀어 넣는 너트 홈 (임시: 검정 시험편 값으로 바꾼다)
PROVISIONAL = (
    "SLOT_NUT (검정 fit1 너트 홈)",
    "가진 M3 × 12 (손가락, 귀)와 M3 × 8 수량",
    *j.PROVISIONAL[3:],
)


def _y(h: float) -> float:
    return j.ATOP + h


def _ccw(pts: list[Pt]) -> list[Pt]:
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1], strict=True))
    return pts if area > 0 else pts[::-1]


def _prism_y(pts: list[Pt], y0: float, y1: float) -> Compound:
    """(x, z) 다각형을 y0 ~ y1로 뽑은 기둥."""
    body = extrude(Polygon(*_ccw(pts), align=None), amount=y1 - y0)
    return solid(Pos(0, y1, 0) * (Rot(90, 0, 0) * body))


def _prism_z(pts: list[Pt], z0: float, z1: float) -> Compound:
    """(x, y) 다각형을 z0 ~ z1로 뽑은 기둥."""
    body = extrude(Polygon(*_ccw(pts), align=None), amount=z1 - z0)
    return solid(Pos(0, 0, z0) * body)


def _flip(shape: Compound) -> Compound:
    """도구 축 둘레로 반 바퀴 돌린 것 (맞은편 받침과 손가락)."""
    return solid(Rot(0, 180, 0) * shape)


# ------------------------------------------------------------------------------------------------
# 이


def _inv(a: float) -> float:
    return math.tan(a) - a


def pinion_outline() -> list[Pt]:
    """피니언의 이 윤곽 (x, z). 이 하나가 +z를 본다. 인벌류트, 이뿌리는 곧게 잇는다."""
    rb = RP * math.cos(PA)
    r_tip, r_root = RP + ADD, RP - DED
    half = math.pi / (2 * TEETH)

    def psi(r: float) -> float:
        return half + _inv(PA) - _inv(math.acos(min(1.0, rb / r)))

    flank = [(r_root, psi(rb))] + [
        (r, psi(r)) for r in (rb + (r_tip - rb) * k / 8 for k in range(9))
    ]
    pts: list[Pt] = []
    for n in range(TEETH):
        c = math.pi / 2 + 2 * math.pi * n / TEETH
        pts += [(r * math.cos(c - p), r * math.sin(c - p)) for r, p in flank]
        pts += [(r * math.cos(c + p), r * math.sin(c + p)) for r, p in reversed(flank)]
    return pts


def rack_outline(u0: float, u1: float) -> list[Pt]:
    """랙 팔 윗부분의 윤곽 (u, z): 뒤의 살과 이. 이의 가운데는 피치의 반 칸 자리다 (이 사이가 0, ±피치, ...)."""
    ht = PITCH / 4 - ADD * math.tan(PA)
    hr = PITCH / 4 + DED * math.tan(PA)
    k0 = math.ceil((u0 + hr + 0.3) / PITCH - 0.5)
    k1 = math.floor((u1 - hr - 0.3) / PITCH - 0.5)
    pts: list[Pt] = [(u0, ARM_BACK_Z), (u1, ARM_BACK_Z), (u1, RACK_ROOT_Z)]
    for k in range(k1, k0 - 1, -1):
        c = (k + 0.5) * PITCH
        pts += [
            (c + hr, RACK_ROOT_Z),
            (c + ht, RACK_TIP_Z),
            (c - ht, RACK_TIP_Z),
            (c - hr, RACK_ROOT_Z),
        ]
    pts.append((u0, RACK_ROOT_Z))
    return pts


def pinion_turn(opening: float) -> float:
    """열림 폭이 opening일 때 피니언이 돈 각 [°] (j.turned에 주는 값. 다 닫혔을 때 0)."""
    return -math.degrees(opening / 2 / RP)


def pinion_screw() -> tuple[float, float]:
    """피니언을 혼에 묶는 나사의 (길이, 혼에 물린 길이). 혼의 테는 2.5 mm이고 그 뒤로 1.7 mm가 비어 있다 (STS3215)."""
    length = PIN_FACE[1] - H_HORN
    for screw in PINION_SCREWS:
        bite = screw + HEAD_REC - length
        if 2.0 <= bite <= 3.4:
            return screw, bite
    raise ValueError(f"피니언 길이 {length:.2f}에 맞는 나사가 없다")


# ------------------------------------------------------------------------------------------------
# 뽑는 부품


def frame() -> Compound:
    """몸통 (검정): 플랜지에 앉는 판 + 서보를 품는 틀. 플랜지에 닿는 면을 바닥에 두고 뽑는다."""
    ya, yd, yf = _y(0.0), _y(ADAPTER_T), _y(H_FRONT)
    x0, x1 = CRADLE_X[0] - WALL, CRADLE_X[1] + WALL
    zo = CRADLE_Z + WALL

    def under(start: float) -> Compound:
        """x = -start에서 시작해 서보 먼 끝 쪽으로 45° 올라가는 면의 아래."""
        far = x0 - 1
        return _prism_z(
            [(-start + 1, ya - 1), (far, ya - start - far), (far, ya - 1)], -zo - 1, zo + 1
        )

    # 서보 먼 끝 쪽의 밑은 45°로 깎고(RELIEF_X), 끝벽은 그 경사를 따라 눕힌다. 바닥에서부터 이어져 걸치지 않는다
    hollow = solid(
        box(CRADLE_X[0], CRADLE_X[1], ya - 1, yf + 1, -CRADLE_Z, CRADLE_Z)
        - under(RELIEF_X - WALL * math.sqrt(2))
    )
    walls = solid(solid(box(x0, x1, ya, yf, -zo, zo) - under(RELIEF_X)) - hollow)
    deck = solid(Rot(0, 45, 0) * cyl_y(flange.ADAPTER_R, ya, yd))
    pads = box(PAD_X[0], PAD_X[1], yd - 0.01, _y(H_BACK), -CRADLE_Z - 0.01, CRADLE_Z + 0.01)
    f = solid(walls + deck + pads)
    # 서보 선을 꽂는 틈 (판과 누름 자리를 지난다)
    f = solid(
        f - box(-flange.ADAPTER_R - 1, NOTCH_X, ya - 1, _y(H_BACK) + 0.01, -NOTCH_HW, NOTCH_HW)
    )
    f = solid(f - cyl_y(BOSS_REC[0], yd - BOSS_REC[1], yd + 0.01))
    f = flange.adapter_cuts(f)
    # 손나사 손잡이가 드는 창, 옆벽의 창 셋 (윗변은 45°로 좁혀 걸치는 길이를 줄인다)
    for x, z in flange.TOOL_SCREWS:
        s = 1.0 if z > 0 else -1.0
        za, zb = sorted((s * (CRADLE_Z - 0.5), s * (zo + 0.5)))
        f = solid(f - box(x - 4.6, x + 4.6, yd, yd + flange.KNOB[1] + 1.4, za, zb))
    for wx0, wx1 in ((-31.0, -20.5), (-18.0, -7.5), (-5.0, 5.5)):
        h0, h1, c = 16.5, 35.0, 3.0
        win = [(wx0, _y(h0)), (wx1, _y(h0)), (wx1, _y(h1 - c)), (wx1 - c, _y(h1))]
        win += [(wx0 + c, _y(h1)), (wx0, _y(h1 - c))]
        f = solid(f - _prism_z(win, -zo - 1, zo + 1))
    # -x 쪽 귀: 나사 둘이 귀, 바닥판을 지나 랙 집의 끝벽에 조인다. 귀 밑(뽑을 때)은 45°로 받친다
    run = HOUSE_HL + x0
    lug = box(-HOUSE_HL, x0 + 0.01, _y(LUG_H0), yf, -LUG_HW, LUG_HW)
    lug += _prism_z(
        [
            (x0 + 0.01, _y(LUG_H0 - run)),
            (x0 + 0.01, _y(LUG_H0) + 0.01),
            (-HOUSE_HL, _y(LUG_H0) + 0.01),
        ],
        -LUG_HW,
        LUG_HW,
    )
    f = solid(f + solid(lug))
    for sz in (1, -1):
        f = solid(f - cyl_y(j.M3_FREE / 2, _y(LUG_H0 - run - 1), yf + 1, -TS[0], sz * TS[1]))
        f = solid(f - cyl_y(3.2, _y(LUG_H0 - run - 1), _y(LUG_H0), -TS[0], sz * TS[1]))
    # +x 쪽 돋움 둘: 밑에서 올라온 바닥판 나사의 너트를 품는다
    bx0 = x1 - 0.01
    for sz in (1, -1):
        za, zb = sorted((sz * (FS[1] - 3.6), sz * (FS[1] + 3.6)))
        boss = box(bx0, BOSS_X1, _y(BOSS_H0), yf, za, zb)
        boss += _prism_z(
            [
                (bx0, _y(BOSS_H0 - (BOSS_X1 - bx0))),
                (bx0, _y(BOSS_H0) + 0.01),
                (BOSS_X1, _y(BOSS_H0) + 0.01),
            ],
            za,
            zb,
        )
        f = solid(f + solid(boss))
        f = solid(f - cyl_y(j.M3_FREE / 2, _y(BOSS_NUT_H - 2.6), yf + 1, FS[0], sz * FS[1]))
        f = solid(
            f
            - nut_trap(
                (FS[0], _y(BOSS_NUT_H), sz * FS[1]),
                (0.0, 1.0, 0.0),
                (1.0, 0.0, 0.0),
                BOSS_X1 - FS[0] + 0.5,
                SLOT_NUT[0],
                SLOT_NUT[1],
            )
        )
    if not f.is_valid or len(f.solids()) != 1:
        raise RuntimeError("그리퍼 몸통 형상이 깨졌다")
    return f


def floor() -> Compound:
    """바닥판 (검정): 서보 앞면이 앉고, 밑면이 통로의 천장이다. 납작하게 뽑는다."""
    y0, y1 = _y(H_FRONT), _y(H_FLOOR)
    f = box(-HOUSE_HL, HOUSE_HL, y0, y1, -HOUSE_HW, HOUSE_HW)
    f = solid(f - cyl_y(HUB_R + 0.3, y0 - 1, y1 + 1))  # 혼과 피니언의 목
    f = solid(f - box(-34.5, -8.8, y0 - 1, y1 + 1, -7.4, 7.4))  # 서보 앞면의 라벨 판
    for x, z in screw_xz():
        f = solid(f - cyl_y(j.M3_FREE / 2, y0 - 1, y1 + 1, x, z))
    return f


def screw_xz() -> list[Pt]:
    """바닥판을 지나는 나사 여섯의 자리 (x, z): +x 바닥판 나사 둘, 끝벽 나사 넷."""
    out = [(FS[0], s * FS[1]) for s in (1, -1)]
    return out + [(sx * TS[0], sz * TS[1]) for sx in (1, -1) for sz in (1, -1)]


def tub() -> Compound:
    """랙 집 (검정): 통로의 옆벽과 끝벽, 덮개. 덮개를 바닥에 두고 뽑는다."""
    y0, yc, y1 = _y(H_FLOOR), _y(H_COVER), _y(H_BOTTOM)
    t = box(-HOUSE_HL, HOUSE_HL, y0, y1, -HOUSE_HW, HOUSE_HW)
    t = solid(t - box(-IN_HL, IN_HL, y0 - 1, yc, -WALL_IN, WALL_IN))
    t = solid(t - box(-IN_HL, IN_HL, yc - 1, y1 + 1, -SLOT_HW, SLOT_HW))
    c = TUB_CHAMFER
    for s in (1, -1):
        cut = [
            (y1 - c, s * HOUSE_HW),
            (y1 + 0.5, s * HOUSE_HW),
            (y1 + 0.5, s * (HOUSE_HW - c - 0.5)),
        ]
        t = solid(t - j.prism_x(cut, -HOUSE_HL - 1, HOUSE_HL + 1))
    for sx in (1, -1):
        for sz in (1, -1):
            t = solid(t - cyl_y(j.M3_TAP / 2, y0 - 1, _y(H_ARM1 - 1.5), sx * TS[0], sz * TS[1]))
    if not t.is_valid or len(t.solids()) != 1:
        raise RuntimeError("랙 집 형상이 깨졌다")
    return t


def pinion() -> Compound:
    """피니언 (검정): 혼 윗면에 앉는 목과 이. 나사 머리 쪽 면을 바닥에 두고 뽑는다. 이 하나가 +z를 본다."""
    y0, y1 = _y(PIN_FACE[0]), _y(PIN_FACE[1])
    p = _prism_y(pinion_outline(), y0, y1) + cyl_y(HUB_R, _y(H_HORN), y0 + 0.01)
    p = solid(p)
    for x, z in j.bolt_xz():
        p = solid(p - cyl_y(j.M3_FREE / 2, _y(H_HORN) - 1, y1 + 1, x, z))
        p = solid(p - cyl_y(3.1, y1 - HEAD_REC, y1 + 1, x, z))
    r_head, d_head = HORN_HEAD_REC
    p = solid(p - cyl_y(r_head, _y(H_HORN) - 1, _y(H_HORN) + d_head))  # 혼 가운데 나사 머리
    return p


def carriage(x_in: float = 0.0) -> Compound:
    """손가락 받침 (+x 쪽 것, 검정): 랙 팔과 판. 밑면을 바닥에 두고 뽑는다. x_in은 손가락 안쪽 면의 x다."""
    yt, ym, yb = _y(H_ARM0), _y(H_PLATE0), _y(H_ARM1)
    u0, u1 = ARM_U
    c = box(u0, u1, ym, yb, ARM_LOW_Z, ARM_BACK_Z) + box(0.0, LC, ym, yb, -PLATE_EDGE, ARM_BACK_Z)
    c = solid(c + _prism_y(rack_outline(u0, u1), yt, ym + 0.01))
    c = solid(c - cyl_y(j.M3_FREE / 2, ym - 1, yb + 1, FINGER_SCREW_U, 0.0))
    c = solid(c - cyl_y(3.1, ym - 1, ym + HEAD_REC, FINGER_SCREW_U, 0.0))
    return solid(Pos(x_in, 0, 0) * c)


def finger(x_in: float = 0.0) -> Compound:
    """손가락 (+x 쪽 것, 검정). 안쪽 면을 바닥에 두고 뽑는다: 휘는 힘이 층을 따라 걸린다."""
    neck1 = H_SHOULDER + BARB_DEPTH - (BARB_U - NECK_U) / math.tan(BARB_SLOPE)
    prof = [
        (0.0, H_ARM1),
        (0.0, H_TIP),
        (TIP_END_U, H_TIP),
        (BARB_U, H_TIP - (BARB_U - TIP_END_U)),
        (BARB_U, H_SHOULDER + BARB_DEPTH),
        (NECK_U, neck1),
        (NECK_U, H_SHOULDER),
        (SHANK_U, H_SHOULDER),
        (SHANK_U, H_ROOT + ROOT_H + (LC - SHANK_U)),
        (LC, H_ROOT + ROOT_H),
        (LC, H_ARM1),
    ]
    f = _prism_z([(u, _y(h)) for u, h in prof], -FINGER_HW, FINGER_HW)
    big = FINGER_HW + 1
    for s in (1, -1):
        cuts = [
            (H_ARM1 - 1, H_ROOT, TENON_HW),  # 목: 덮개 틈을 지난다
            (H_SHOULDER, H_TIP + 1, TIP_HW),  # 끝
            (H_SHOULDER, neck1 - 0.3, NECK_HW),  # 구멍 입구를 지나는 곳
        ]
        for h0, h1, hw in cuts:
            za, zb = sorted((s * hw, s * big))
            f = solid(f - box(-1, LC + 1, _y(h0), _y(h1), za, zb))
        lead = [(_y(H_TIP - 2.4), s * (TIP_HW + 0.01)), (_y(H_TIP) + 0.01, s * (TIP_HW + 0.01))]
        lead.append((_y(H_TIP) + 0.01, s * (TIP_HW - 1.5)))
        f = solid(f - j.prism_x(lead, -1, LC + 1))
    f = solid(
        f - cyl_y(j.M3_FREE_H / 2, _y(H_ARM1) - 1, _y(FINGER_NUT[1] + 2.4), FINGER_SCREW_U, 0.0)
    )
    half = SLOT_NUT[0] / 2
    f = solid(
        f - box(FINGER_SCREW_U - 3.3, LC + 1, _y(FINGER_NUT[0]), _y(FINGER_NUT[1]), -half, half)
    )
    if not f.is_valid or len(f.solids()) != 1:
        raise RuntimeError("손가락 형상이 깨졌다")
    return solid(Pos(x_in, 0, 0) * f)


def box_top(h_top: float = H_SHOULDER, half: float = 20.0) -> Compound:
    """박스의 윗부분 (윗면이 h_top). 손가락 끝이 구멍에 맞는지 볼 때 쓴다."""
    y0 = _y(h_top)
    b = box(-half, half, y0, y0 + BOX_HOLE_DEPTH + 2.0, -half, half)
    b = solid(b - cyl_y(BOX_LIP_R, y0 - 1, y0 + BOX_LIP_T + 0.01))
    rise = BOX_CAVITY_R - BOX_LIP_R
    cone = Pos(0, y0 + BOX_LIP_T + rise / 2, 0) * (
        Rot(-90, 0, 0) * Cone(BOX_LIP_R, BOX_CAVITY_R, rise)
    )
    b = solid(b - cone)
    return solid(b - cyl_y(BOX_CAVITY_R, y0 + BOX_LIP_T + rise - 0.01, y0 + BOX_HOLE_DEPTH))


def tip(x_in: float = 0.0) -> Compound:
    """손가락 둘의 끝(어깨 밑)만. 박스 구멍과의 틈을 볼 때 쓴다 (어깨는 박스 윗면에 닿아 있다)."""
    f = finger(x_in)
    region = box(-60, 60, _y(H_SHOULDER + 0.05), _y(H_TIP + 1), -20, 20)
    return solid(solid(f & region) + solid(_flip(f) & region))


def grasp_x() -> float:
    """박스를 잡았을 때 손가락 안쪽 면의 x: 턱의 끝 모서리(z = ±TIP_HW)가 구멍의 원뿔에 닿는 자리."""
    r = BOX_LIP_R + (BARB_DEPTH - BOX_LIP_T)
    return math.sqrt(r * r - TIP_HW * TIP_HW) - BARB_U


# ------------------------------------------------------------------------------------------------
# 조립 검사


def _screw(x: float, z: float, seat: float, length: float, down: bool) -> Compound:
    """M3 둥근머리 볼트. seat은 머리 밑면의 y다. down이면 머리가 플랜지 쪽이고 몸이 도구 쪽으로 간다."""
    if down:
        return solid(cyl_y(1.5, seat, seat + length, x, z) + cyl_y(2.85, seat - 1.65, seat, x, z))
    return j._screw_y(x, z, seat, length)


def _nut(x: float, z: float, y0: float) -> Compound:
    return solid(j.hex_y(x, z, y0, y0 + j.NUT_T, 5.4) - cyl_y(1.5, y0 - 1, y0 + j.NUT_T + 1, x, z))


def servo_parts() -> dict[str, Compound]:
    """서보 몸통, 혼, 꽂은 플러그 (검사와 그림에 쓴다)."""
    sv = j.servo(MODEL)
    dy = _y(H_HORN) - j.SEAT
    plug = box(-16.6, -11.85, _y(H_PLUG), _y(H_BACK + 0.5), PLUG_Z[0], PLUG_Z[1])
    return {
        "body": solid(Pos(0, dy, 0) * sv["body"]),
        "horn": solid(Pos(0, dy, 0) * sv["horn"]),
        "plug": solid(plug),
    }


def hardware(opening: float = 0.0) -> list[j.Part]:
    """나사와 너트."""
    out: list[j.Part] = []
    length, _ = pinion_screw()
    turn = pinion_turn(opening)
    for k, (x, z) in enumerate(j.bolt_xz(), start=1):
        s = j.turned(_screw(x, z, _y(PIN_FACE[1] - HEAD_REC), length, False), turn)
        out.append(j.Part(f"피니언 나사 M3x{length:.0f} {k}", s, j.STEEL, True, "metal"))
    for k, sz in enumerate((1, -1), start=1):
        s = _screw(FS[0], sz * FS[1], _y(H_FLOOR), 8.0, False)
        out.append(j.Part(f"바닥판 나사 M3x8 {k}", s, j.STEEL, True, "metal"))
        n = _nut(FS[0], sz * FS[1], _y(BOSS_NUT_H) - j.NUT_T / 2)
        out.append(j.Part(f"너트 (몸통 돋움) {k}", n, j.STEEL, True, "metal"))
        s = _screw(TS[0], sz * TS[1], _y(H_FRONT), 8.0, True)
        out.append(j.Part(f"끝벽 나사 M3x8 {k}", s, j.STEEL, True, "metal"))
        s = _screw(-TS[0], sz * TS[1], _y(LUG_H0), 12.0, True)
        out.append(j.Part(f"귀 나사 M3x12 {k}", s, j.STEEL, True, "metal"))
    x = opening / 2
    for k, flip in enumerate((False, True), start=1):
        s = _screw(x + FINGER_SCREW_U, 0.0, _y(H_PLATE0 + HEAD_REC), FINGER_SCREW, True)
        n = _nut(x + FINGER_SCREW_U, 0.0, _y(FINGER_NUT[0] + 0.2))
        if flip:
            s, n = _flip(s), _flip(n)
        out.append(j.Part(f"손가락 나사 M3x12 {k}", s, j.STEEL, True, "metal"))
        out.append(j.Part(f"너트 (손가락) {k}", n, j.STEEL, True, "metal"))
    return out


def gripper_parts(opening: float = 0.0) -> list[j.Part]:
    """그리퍼만: 뽑는 부품 여덟, 서보, 나사."""
    x = opening / 2
    sv = servo_parts()
    car, fin = carriage(x), finger(x)
    parts = [
        j.Part("그리퍼 몸통", frame(), moving=True, color="frame"),
        j.Part("바닥판", floor(), moving=True, color="frame"),
        j.Part("랙 집", tub(), moving=True, color="cap"),
        j.Part("피니언", j.turned(pinion(), pinion_turn(opening)), moving=True, color="metal"),
        j.Part("손가락 받침 1", car, moving=True, color="frame"),
        j.Part("손가락 받침 2", _flip(car), moving=True, color="frame"),
        j.Part("손가락 1", fin, moving=True, color="cap"),
        j.Part("손가락 2", _flip(fin), moving=True, color="cap"),
        j.Part("그리퍼 서보", sv["body"], moving=True, color="servo"),
        j.Part("그리퍼 혼", sv["horn"], moving=True, color="metal"),
        j.Part("그리퍼 플러그", sv["plug"], moving=True, color="plug"),
    ]
    return parts + hardware(opening)


def assembly(opening: float = 0.0) -> list[j.Part]:
    """J6 조립(parts/flange.py)의 도구 어댑터 판 자리에 그리퍼를 단 것. 영점 핀은 뺀다 (도구를 떼고 꽂는다)."""
    keep = [
        p
        for p in flange.assembly(MODEL)
        if p.name != "도구 어댑터 판" and not p.name.startswith("영점 핀")
    ]
    return keep + gripper_parts(opening)


_SAME = {
    frozenset(p)
    for p in (("그리퍼 서보", "그리퍼 혼"), ("그리퍼 서보", "그리퍼 플러그"), ("서보 몸통", "혼"))
}
_SAME |= {frozenset(p) for p in (("플러그", "서보 몸통"), ("플러그", "선 자리"))}


def overlaps(parts: list[j.Part], tol: float = 0.05) -> dict[tuple[str, str], float]:
    """뽑는 부품과 서보끼리 겹친 부피 [mm³] (tol 넘는 것만)."""
    solids = [
        p for p in parts if p.density == 0.0 and "핀" not in p.name and "손잡이" not in p.name
    ]
    out: dict[tuple[str, str], float] = {}
    for i, a in enumerate(solids):
        for b in solids[i + 1 :]:
            if frozenset((a.name, b.name)) in _SAME:
                continue
            if far_apart(a.shape, b.shape, 0.0) or a.shape.distance_to(b.shape) > 1e-6:
                continue
            v = solid(a.shape & b.shape).volume
            if v > tol:
                out[(a.name, b.name)] = v
    return out


SLIDE_PAIRS = (
    ("손가락 받침 1", "랙 집"),
    ("손가락 받침 1", "바닥판"),
    ("손가락 받침 1", "피니언"),
    ("손가락 받침 2", "피니언"),
    ("손가락 받침 1", "바닥판 나사 M3x8 1"),
    ("손가락 받침 2", "바닥판 나사 M3x8 2"),
    ("손가락 1", "랙 집"),
    ("피니언", "바닥판"),
    ("피니언", "그리퍼 서보"),
    ("피니언", "바닥판 나사 M3x8 1"),
    ("그리퍼 플러그", "그리퍼 몸통"),
)


def slide_gaps(opening: float) -> dict[tuple[str, str], float]:
    """움직이는 부품(받침, 손가락, 피니언)과 그 둘레 사이의 틈 [mm]."""
    by = {p.name: p.shape for p in gripper_parts(opening)}
    return {(a, b): by[a].distance_to(by[b]) for a, b in SLIDE_PAIRS}


def sweep_gaps() -> dict[str, float]:
    """J6를 멈춤 각까지 돌릴 때, 그리퍼의 윗부분과 서 있는 것 사이의 가장 좁은 틈.

    서 있는 것은 손목 몸통, D405와 받침과 플러그, 그리고 손목 몸통이 앉은 앞 관절(J5)의 출력 원판이다.
    """
    from horilab_design.parts import camera_mount

    fixed = [
        p
        for p in camera_mount.assembly(MODEL)
        if p.name in ("손목 몸통", "D405", "D405 받침", "D405 플러그")
    ]
    fixed += wrist.parent_disc(MODEL)
    names = ("그리퍼 몸통", "그리퍼 서보", "바닥판", "랙 집")
    moving = [p for p in gripper_parts(OPEN_MAX) if p.name in names]
    out: dict[str, float] = {}
    for q in range(-165, 166, 5):
        turn = float(max(-j.STOP_DEG, min(j.STOP_DEG, q)))
        for a in moving:
            shape = j.turned(a.shape, turn)
            for b in fixed:
                key = f"{a.name} ↔ {b.name}"
                if far_apart(shape, b.shape, out.get(key, 99.0)):
                    continue  # 이 각에서는 지금까지의 가장 좁은 틈보다 멀다
                out[key] = min(out.get(key, 99.0), shape.distance_to(b.shape))
    return out


VIEW_AHEAD = 60.0  # 잡기 전 높이: 도구 끝점보다 6 cm 앞의 평면 (0017의 D405 시야 기준)


def view_share(opening: float, ahead: float = VIEW_AHEAD) -> float:
    """D405가 도구 끝점 둘레(±30 mm, 끝점보다 ahead만큼 앞의 평면)를 보는 몫. J6 각 0°(손가락이 카메라와 직각).

    렌즈 가운데(parts/camera_mount.py)에서 7 × 7점으로 곧은 선을 긋고, 그리퍼의 실제 형상에 가리면 못 본 것으로 센다.
    """
    from horilab_design.parts import camera_mount

    lens = camera_mount.LENS
    shapes = [p.shape for p in gripper_parts(opening) if p.density == 0.0]
    grid = [-30.0 + 10.0 * k for k in range(7)]
    seen = 0
    for a in grid:
        for b in grid:
            d = (a - lens[0], _y(H_TCP + ahead) - lens[1], b - lens[2])
            length = math.sqrt(sum(c * c for c in d))
            u = (d[0] / length, d[1] / length, d[2] / length)
            axis = Axis(lens, u)
            blocked = False
            for shape in shapes:
                for pt, _normal in shape.find_intersection_points(axis):
                    along = (
                        (pt.X - lens[0]) * u[0] + (pt.Y - lens[1]) * u[1] + (pt.Z - lens[2]) * u[2]
                    )
                    if 1e-6 < along < length - 1e-6:
                        blocked = True
                        break
                if blocked:
                    break
            seen += not blocked
    return seen / len(grid) ** 2


def screw_checks() -> dict[str, float]:
    """나사가 물린 길이와 여유 [mm]."""
    length, bite = pinion_screw()
    lug_tip = LUG_H0 + 12.0
    end_tip = H_FRONT + 8.0
    fin_tip = H_PLATE0 + HEAD_REC + FINGER_SCREW
    return {
        f"피니언 나사 M3 × {length:.0f}이 혼에 물린 길이 (테 2.5)": bite,
        "피니언 나사 끝과 서보 앞면 사이": 2.5 + 1.7 - bite,
        "피니언 나사 머리 밑의 살": PIN_FACE[1] - HEAD_REC - H_HORN,
        "바닥판 나사가 너트(2.4)를 지나간 길이": (BOSS_NUT_H - j.NUT_T / 2) - (H_FLOOR - 8.0),
        "귀 나사가 끝벽에 물린 길이": lug_tip - H_FLOOR,
        "끝벽 나사가 끝벽에 물린 길이": end_tip - H_FLOOR,
        "끝벽 나사 끝과 구멍 바닥 사이": (H_ARM1 - 1.5) - max(lug_tip, end_tip),
        "손가락 나사가 너트(2.4)를 지나간 길이": fin_tip - (FINGER_NUT[0] + 0.2 + j.NUT_T),
        "손가락 나사 머리 밑의 판 살": PLATE_T - HEAD_REC,
        "받침과 통로 사이 위아래 틈 (합)": 2 * SLIDE_GAP,
    }


def travel() -> dict[str, float]:
    """움직임과 힘 (서보 사양은 docs/hardware.md)."""
    deg = math.degrees(STROKE / RP)
    return {
        "열림 폭 [mm]": OPEN_MAX,
        "피니언이 도는 각 [°]": deg,
        "서보 한 칸에 바뀌는 열림 폭 [mm]": 2 * RP * 2 * math.pi / 4096,
        "정격 토크(0.98 N·m)에서 랙 하나의 힘 [N]": 0.98 / (2 * RP / 1000),
        "스톨 토크(2.94 N·m)에서 랙 하나의 힘 [N]": 2.94 / (2 * RP / 1000),
        "박스를 잡을 때의 열림 폭 [mm]": 2 * grasp_x(),
    }


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋, 개수)
LABELS = {  # 뽑는 부품의 한글 이름 (출력 안내에 쓴다, horilab_design.slicing)
    "gripper_frame": "그리퍼 몸통",
    "gripper_floor": "그리퍼 바닥판",
    "gripper_tub": "랙 집",
    "gripper_pinion": "피니언",
    "gripper_slider": "손가락 받침",
    "gripper_finger": "손가락",
}


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str, int]]:
    return {
        "gripper_frame": (frame(), (0.0, 1.0, 0.0), "black", "strength", 1),
        "gripper_floor": (floor(), (0.0, 1.0, 0.0), "black", "strength", 1),
        "gripper_tub": (tub(), (0.0, -1.0, 0.0), "black", "strength", 1),
        "gripper_pinion": (pinion(), (0.0, -1.0, 0.0), "black", "strength", 1),
        "gripper_slider": (carriage(), (0.0, -1.0, 0.0), "black", "strength", 2),
        "gripper_finger": (finger(), (1.0, 0.0, 0.0), "black", "strength", 2),
    }


SERVO_G = 57.2 + 1.45  # 서보 몸통과 출력 혼 (잰 값, data/measured.toml)
CABLE_G = 3.5  # 서보 선 20 cm


def mass_g() -> dict[str, float]:
    """그리퍼 무게 [g]: 뽑는 부품(어림식), 서보, 나사와 너트, 손나사와 손잡이, 선."""
    out = {
        name: printed_mass_g(shape, up, preset) * n
        for name, (shape, up, _c, preset, n) in printed().items()
    }
    out["서보와 혼"] = SERVO_G
    out["나사와 너트"] = sum(p.mass_g for p in hardware())
    out["손나사와 손잡이"] = sum(
        p.mass_g for p in flange.hardware() if p.name.startswith(("손나사", "손잡이"))
    )
    out["선"] = CABLE_G
    return out


Lump = tuple[float, tuple[float, float, float]]


def mass_centers() -> dict[str, Lump]:
    """덩어리 넷의 (무게 [g], 무게 중심 (x, H, z) [mm]). 팔 전체 모델(studies/ur_arm.py)의 무게 자리와 견준다.

    뽑는 부품은 그 부품의 무게를 부피 중심에 둔다. 선은 꽂는 자리에 둔다. 받침과 손가락은 둘이 축을 가운데로
    마주 보므로 열림 폭이 바뀌어도 중심이 그대로다.
    """
    made = printed()
    grams = {name: printed_mass_g(s, up, preset) for name, (s, up, _c, preset, _n) in made.items()}

    def at(shape: Compound) -> tuple[float, float, float]:
        c = shape.center()
        return (c.X, c.Y - j.ATOP, c.Z)

    def lump(items: list[Lump]) -> Lump:
        total = sum(m for m, _ in items)
        x, h, z = (sum(m * c[k] for m, c in items) / total for k in range(3))
        return total, (x, h, z)

    sv = servo_parts()
    car, fin = carriage(), finger()
    hand = [p for p in flange.hardware() if p.name.startswith(("손나사", "손잡이"))]
    plug_at = (-14.2, H_PLUG, (PLUG_Z[0] + PLUG_Z[1]) / 2)
    body = [(grams["gripper_frame"], at(made["gripper_frame"][0])), (CABLE_G, plug_at)]
    house = [(grams[n], at(made[n][0])) for n in ("gripper_floor", "gripper_tub", "gripper_pinion")]
    house += [(grams["gripper_slider"], at(c)) for c in (car, _flip(car))]
    return {
        "몸통, 손나사, 선": lump(body + [(p.mass_g, at(p.shape)) for p in hand]),
        "서보와 혼": lump([(SERVO_G - 1.45, at(sv["body"])), (1.45, at(sv["horn"]))]),
        "랙 집과 속 부품, 나사": lump(house + [(p.mass_g, at(p.shape)) for p in hardware()]),
        "손가락": lump([(grams["gripper_finger"], at(f)) for f in (fin, _flip(fin))]),
    }


def report() -> None:
    print(
        f"평행 그리퍼: 서보 뒷면 H {H_BACK:.1f}, 바닥판 {H_FRONT:.1f} ~ {H_FLOOR:.1f}, 통로 ~ {H_COVER:.1f}, "
        f"몸통 밑면 {H_BOTTOM:.1f}, 손가락 끝 {H_TIP:.1f}, TCP {H_TCP:.1f} | 랙 집 {2 * HOUSE_HL:.1f} × "
        f"{2 * HOUSE_HW:.1f} mm"
    )
    print(f"임시 값: {', '.join(PROVISIONAL)}")
    print("\n뽑는 부품 (무게는 어림, 걸침은 바닥에 닿지 않고 아래를 보는 면)")
    for name, (shape, up, color, preset, n) in printed().items():
        size = shape.bounding_box().size
        over = overhangs(shape, up)
        print(
            f"  {name:15s} × {n} {color:5s} {size.X:5.1f} × {size.Y:5.1f} × {size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f}, "
            f"가장 넓은 한 자리 {over.largest:.0f})"
        )
    m = mass_g()
    print(
        "무게 [g]: "
        + ", ".join(f"{k} {v:.1f}" for k, v in m.items())
        + f" | 합 {sum(m.values()):.1f}"
    )
    lumps = mass_centers()
    total = sum(g for g, _ in lumps.values())
    cx, ch = (sum(g * c[k] for g, c in lumps.values()) / total for k in (0, 1))
    print(
        f"무게 중심: 플랜지 면에서 {ch:.1f} mm, 축에서 서보 먼 끝 쪽으로 {-cx:.1f} mm | "
        + ", ".join(f"{k} {g:.1f} g (x {c[0]:.1f}, H {c[1]:.1f})" for k, (g, c) in lumps.items())
    )
    for name, value in {**travel(), **screw_checks()}.items():
        print(f"  {name}: {value:.2f}")
    for opening in (0.0, 2 * grasp_x(), 30.0, OPEN_MAX):
        hits = overlaps(assembly(opening))
        text = ", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음"
        print(f"열림 {opening:5.1f} mm 겹침 [mm³]: {text}")
    for opening in (0.0, OPEN_MAX):
        gaps = sorted((d, a, b) for (a, b), d in slide_gaps(opening).items())
        print(
            f"열림 {opening:5.1f} mm 틈 [mm]: "
            + ", ".join(f"{a}↔{b} {d:.2f}" for d, a, b in gaps[:7])
        )
    print(
        "J6를 돌릴 때 가장 좁은 틈 [mm]: "
        + ", ".join(
            f"{k} {d:.1f}" for k, d in sorted(sweep_gaps().items(), key=lambda kv: kv[1])[:6]
        )
    )
    print(
        "D405가 도구 끝점 둘레를 보는 몫 (잡기 전 높이 / 끝점 높이): "
        + ", ".join(
            f"열림 {w:.0f} mm {view_share(w) * 100:.0f}% / {view_share(w, 2.0) * 100:.0f}%"
            for w in (0.0, 40.0, OPEN_MAX)
        )
    )
    gx = grasp_x()
    top = box_top()
    for label, x in (("닫고 넣을 때", 0.0), ("잡기 0.1 mm 전", gx - 0.1), ("잡은 자리", gx)):
        print(
            f"박스 구멍과 손가락 끝 사이 ({label}, 안쪽 면 x {x:.2f}): {tip(x).distance_to(top):.2f} mm"
        )


if __name__ == "__main__":
    report()
