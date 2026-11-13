"""오차 예산 (0012, 0028): 팔 끝의 반복 정밀도와 상대 정확도에 무엇이 얼마나 드는지 셈한다.

관절 각의 오차와 기구의 오차를 TCP의 오차로 옮긴다 (야코비안). 자세는 도구가 아래를 향한 작업 갈래(앞, 팔꿈치 위,
J5 > 0)이고, 주 작업 영역(0017)의 반지름을 따라 본다. 팔은 J1 둘레로 돌기만 하므로 방위 0에서 셈하면
X는 반지름 쪽, Y는 둘레 쪽, Z는 높이다. 팔의 형상과 무게는 studies/ur_arm.py의 값이다.

몫마다 가정이 다르다 (docs/hardware.md의 "다른 사람이 잰 값", 0027의 베어링 계산, 출력 오차의 어림). 잰 값이
아니라 설계할 때의 어림이고, 다 만든 팔에서 잰 값으로 바꾼다.

    uv run python -m horilab_design.studies.error_budget
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from horilab_design.parts import joint as jp
from horilab_design.parts import links as lp
from horilab_design.parts import shoulder as sp
from horilab_design.robot import Robot
from horilab_design.statics import FLANGE, GRAVITY, frame_poses, holding_torques
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur_arm as ua
from horilab_design.studies import ur_compare as uc
from horilab_design.studies.packaging import DEG, MM

STEP = 2 * math.pi / 4096  # 서보 값 한 칸 [rad] (0.088°)
Z_PLANE = 0.02  # 보는 높이 [m]: 책상 위 물체 (주 작업 영역의 아래 끝)
BRANCH = "앞위+"  # 작업 갈래 (0013)
PAIR = (0.20, 0.30)  # 반지름 쪽으로 100 mm 떨어진 두 점 [m]
ARC_R, ARC_D = 0.26, 0.10  # 둘레 쪽으로 100 mm 떨어진 두 점: 반지름과 거리 [m]

# 반복 정밀도: 관절이 같은 목표에 설 때마다 흩어지는 폭 [칸]. 0.5는 분해능만, 1과 2는 불감대와 마찰 몫
BANDS = (0.5, 1.0, 2.0)
# 0012의 식(ISO 9283: 평균점에서의 거리 l의 평균 + 3 × l의 표준편차)을 한 축에 쓰면, 위치가 표준편차 σ인
# 정규 분포일 때 l = |x|라서 평균 0.80σ + 3 × 0.60σ = 2.61σ다
REPEAT_SIGMAS = math.sqrt(2 / math.pi) + 3.0 * math.sqrt(1 - 2 / math.pi)
# 서보 처짐을 보정한 뒤에도 남는 몫 (포인터로 두 점 사이 거리를 잴 때)
REMAINING = (
    "영점 (핀 구멍 자리 ±0.1 mm)",
    "링크 길이 (±0.2 mm)",
    "베어링 기울기 (넓게)",
    "관의 휨 (포인터)",
)
# 기울인 자세를 볼 자리 (0017의 포인터 시험: 앞쪽 가운데)와 기울기 [°]
TILT_POINT = (0.24, 0.05)
TILTS = (30, 45)
# 영점: 출력한 핀 구멍의 자리 오차 [m] (구멍 하나, 어림). 핀은 부모의 살과 자식의 귀 두 구멍에 꽂힌다
HOLE_POSITION = 0.1 * MM
# 관절 축에서 영점 핀까지 [m]. J2는 출력 원판이 커서 33 mm (parts/links.py), 나머지는 29 mm (parts/joint.py)
PIN_RADIUS = (sp.R_PIN * MM, jp.R_PIN * MM, jp.R_PIN * MM, jp.R_PIN * MM)  # J2 ~ J5
# 링크 길이와 손목 오프셋의 출력 오차 [m] (어림)
LENGTH_ERROR = 0.2 * MM
# 출력 면 베어링의 기울기 틈 [rad]: 한쪽 링을 잡았을 때 다른 링이 기울 수 있는 각 (0027의 계산, 좁게와 넓게).
# 무게가 한쪽으로 붙이면 가운데에서 그 절반만큼 기운 채 돈다
TILT_PLAY = (0.13 * DEG, 0.38 * DEG)
# J1 축의 기울기 [rad]: 6806 두 개를 30 mm 띄운 받침의 틈 0.05°와, 자리 구멍이 0.02 mm 클 때의 0.04° (0028, 계산)
J1_TILT = 0.09 * DEG
J1_BEARING_Z = 0.07  # J1 받침의 높이 어림 [m] (베이스를 그리면 바꾼다)
E_PLA = 2.0e9  # [Pa] 출력한 PLA의 탄성 계수. 확인하지 않은 어림이라 낮게 잡았다 (studies/covers.py와 같다)


@dataclass(frozen=True)
class Pose:
    """한 자리의 자세와, 거기서 쓰는 값."""

    r: float
    q: np.ndarray
    tcp: np.ndarray
    jv: np.ndarray  # 관절 각 1 rad에 TCP가 움직이는 벡터 (3 × 6)
    frames: list[np.ndarray]


@dataclass
class Arm:
    hori: uc.Hori
    tool: pk.Tool
    concept: ua.Concept

    @property
    def robot(self) -> Robot:
        return self.hori.robot


def arm(tool_key: str, payload: bool) -> Arm:
    """도구를 단 팔. payload가 참이면 그 도구의 짐도 든다.

    선의 무게(0031)는 넣지 않는다. 선의 무게는 아직 가정이고(D405 선을 재지 않았다), 넣으면 가장 먼 낮은 칸의
    어깨 버팀 토크가 한계(정격의 0.8배)에 걸려 그 칸의 작업 자세를 고를 수 없다. 선을 잰 뒤 넣어서 다시 낸다
    """
    c = ua.Concept(wires=False)
    tool = ua.TOOLS[tool_key]
    h = ua.hori(c, tool)
    if not payload:
        h.masses = [m for m in h.masses if m.label != "짐"]
    return Arm(h, tool, c)


def _branch(q: np.ndarray) -> str:
    front = abs((q[0] + math.pi) % (2 * math.pi) - math.pi) < math.pi / 2
    return ("앞" if front else "뒤") + ("위" if q[2] > 0 else "아래") + ("+" if q[4] > 0 else "-")


def pose_at(
    a: Arm, r: float, z: float = Z_PLANE, tilt: float = 0.0, tilt_dir: float = 0.0
) -> Pose | None:
    """TCP를 (r, 0, z)에 두고 도구가 아래에서 tilt만큼 tilt_dir 쪽으로 기운 자세. 없으면 None.

    작업 갈래의 해를 먼저 쓰고, 기울인 자세에서 작업 갈래로 안 되면 되는 해 가운데 고른다.
    """
    target = pk.tool_pose((r, 0.0, z), tilt, tilt_dir, 0.0, a.tool.length)
    found = ua.good_solutions(a.hori, target)
    sols = [q for q in found if _branch(q) == BRANCH] or (found if tilt > 0 else [])
    if not sols:
        return None
    q = min(sols, key=lambda s: float(a.hori.torque(s)[1:].max()))
    frames = frame_poses(a.robot, q)
    flange = frames[-1]
    tcp = flange[:3, 3] + flange[:3, 2] * a.tool.length
    jv = np.zeros((3, 6))
    for i in range(6):
        t = frames[i + 1]
        axis = t[:3, :3] @ np.asarray(a.robot.joints[i].axis)
        jv[:, i] = np.cross(axis, tcp - t[:3, 3])
    return Pose(r, q, tcp, jv, frames)


def _mass_points(a: Arm, p: Pose) -> list[tuple[int, np.ndarray, float]]:
    """(링크 번호, 베이스 좌표의 위치, 무게). 플랜지에 붙은 것은 링크 6으로 센다."""
    out = []
    for pm in a.hori.masses:
        frame = p.frames[7 if pm.link == FLANGE else pm.link]
        link = 6 if pm.link == FLANGE else pm.link
        out.append((link, frame[:3, :3] @ np.asarray(pm.position) + frame[:3, 3], pm.mass))
    return out


# ------------------------------------------------------------------------------------------------
# 몫마다: TCP가 움직이는 벡터 [m]


def sag(a: Arm, p: Pose) -> np.ndarray:
    """서보가 짐에 눌려 목표보다 처진 채 버티는 몫. 서보를 스프링으로 본다 (ur_arm.STIFFNESS).

    위치 서보는 오차가 있어야 힘을 내서, 버티는 토크만큼 목표에서 벗어난다. 서보는 그 벗어남을 스스로 읽는다.
    """
    hold = holding_torques(a.robot, p.q, a.hori.masses)
    return p.jv @ (-hold / ua.STIFFNESS)


def zero_moves(p: Pose) -> np.ndarray:
    """영점 오차 몫 (3 × 4, J2 ~ J5). 핀 구멍 둘의 자리 오차가 서로 겹쳐 만드는 각이다. J1의 영점은 좌표계를
    돌릴 뿐이고, J6는 도구 축 둘레라 TCP를 옮기지 않는다."""
    angles = np.array([math.sqrt(2) * HOLE_POSITION / r for r in PIN_RADIUS])
    return p.jv[:, 1:5] * angles


def length_moves(p: Pose) -> np.ndarray:
    """링크 길이와 손목 오프셋의 오차 몫 (3 × 5: 위팔, 아래팔, J4→J5, J5→J6, J6→TCP)."""
    f = p.frames
    directions = [
        f[2][:3, 2],  # 위팔: 링크 2의 z
        f[3][:3, 2],  # 아래팔: 링크 3의 z
        f[4][:3, 1],  # J4 → J5: 링크 4의 y
        f[5][:3, 2],  # J5 → J6: 링크 5의 z
        f[7][:3, 2],  # J6 → 플랜지 → TCP: 도구 축
    ]
    return np.array(directions).T * LENGTH_ERROR


def tilt_moves(a: Arm, p: Pose) -> np.ndarray:
    """출력 면 베어링이 1 rad 기울 때 TCP가 움직이는 벡터 (3 × 5, J2 ~ J6). 무게가 기울이는 쪽으로 기운다."""
    points = _mass_points(a, p)
    prims = {prim.name: prim for prim in a.hori.prims}
    out = np.zeros((3, 5))
    for k, j in enumerate(range(2, 7)):
        parent = p.frames[j - 1]
        center = parent[:3, :3] @ prims[f"J{j}/bearing"].pos + parent[:3, 3]
        axis = p.frames[j][:3, :3] @ np.asarray(a.robot.joints[j - 1].axis)
        moment = np.zeros(3)
        for link, position, mass in points:
            if link >= j:
                moment += np.cross(position - center, np.array([0.0, 0.0, -GRAVITY * mass]))
        across = moment - (moment @ axis) * axis
        size = float(np.linalg.norm(across))
        if size > 1e-9:
            out[:, k] = np.cross(across / size, p.tcp - center)
    return out


def bending(a: Arm, p: Pose) -> np.ndarray:
    """위팔과 아래팔의 관이 휘는 몫. 관을 한쪽이 잡힌 곧은 보로 본다. 관절 집, 베어링, 체결의 탄성과
    비틀림은 넣지 않았다."""
    points = _mass_points(a, p)
    c = a.concept
    total = np.zeros(3)
    for link, length, radius in ((2, c.upper, c.upper_r), (3, c.fore, c.fore_r)):
        frame = p.frames[link]
        root, along = frame[:3, 3], frame[:3, 2]
        end = root + along * length
        bend_axis = frame[:3, :3] @ np.asarray(a.robot.joints[link - 1].axis)  # 관절 축 (가로)
        normal = np.cross(bend_axis, along)
        force, moment = 0.0, 0.0
        for owner, position, mass in points:
            if owner > link:
                weight = np.array([0.0, 0.0, -GRAVITY * mass])
                force += float(weight @ normal)
                moment += float(np.cross(position - end, weight) @ bend_axis)
        d_out, d_in = 2 * radius, 2 * (radius - lp.WALL * MM)
        stiffness = E_PLA * math.pi / 64 * (d_out**4 - d_in**4)
        deflection = force * length**3 / (3 * stiffness) + moment * length**2 / (2 * stiffness)
        slope = force * length**2 / (2 * stiffness) + moment * length / stiffness
        total += deflection * normal + slope * np.cross(bend_axis, p.tcp - end)
    return total


def base_tilt(p: Pose) -> np.ndarray:
    """J1 축이 팔 쪽으로 기우는 몫 (팔의 무게가 받침의 틈을 한쪽으로 붙인다)."""
    center = np.array([0.0, 0.0, J1_BEARING_Z])
    return J1_TILT * np.cross(np.array([0.0, 1.0, 0.0]), p.tcp - center)


# ------------------------------------------------------------------------------------------------
# 모으기


def repeatability(p: Pose, band: float) -> np.ndarray:
    """관절마다 ±band 칸 안에서 고르게 흩어질 때, 0012의 식으로 본 축마다의 반복 정밀도 [m] (X, Y, Z)."""
    sigma_joint = band * STEP / math.sqrt(3)
    sigma = np.sqrt(((p.jv * sigma_joint) ** 2).sum(axis=1))
    return REPEAT_SIGMAS * sigma


def relative(moves_by_r: dict[float, np.ndarray]) -> tuple[float, float]:
    """반지름마다의 오차 벡터(들)에서 두 점 사이 거리의 오차 [m]: (반지름 쪽 100 mm, 둘레 쪽 100 mm).

    벡터가 여럿(3 × n, 부호를 모르는 서로 다른 몫)이면 몫마다 셈해 제곱합의 제곱근으로 모은다.
    둘레 쪽 두 점은 같은 자세를 J1로 돌린 것이라, 반지름 쪽 오차가 반지름을 바꾼 만큼 현의 길이가 달라진다.
    """
    near, far, arc = (np.atleast_2d(moves_by_r[r].T).T for r in (*PAIR, ARC_R))
    radial = far[0] - near[0]
    tangential = ARC_D * arc[0] / ARC_R
    return float(np.linalg.norm(radial)), float(np.linalg.norm(tangential))


def tilted(a: Arm) -> dict[int, dict]:
    """도구를 기울인 자세(포인터 시험의 자리, 8방향)에서의 값. 기울기 [°]마다:

    되는 방향 수, 관절이 ±1칸 흩어질 때의 반복 정밀도(방향 중 가장 나쁜 값), J1 말고 중력이 누르지 않는 관절과
    그 관절들이 백래시 안에서 떠 있을 때 TCP가 움직이는 폭, 서보 처짐의 크기.
    """
    r, z = TILT_POINT
    out = {}
    for tilt in TILTS:
        poses = [pose_at(a, r, z, tilt * DEG, k * 45 * DEG) for k in range(8)]
        poses = [p for p in poses if p is not None]
        if not poses:
            out[tilt] = {"reached": 0}
            continue
        free: set[int] = set()
        floating = 0.0
        for p in poses:
            hold = holding_torques(a.robot, p.q, a.hori.masses)
            loose = [i for i in range(1, 5) if abs(hold[i]) < ua.BIAS_MIN]  # J2 ~ J5
            free.update(i + 1 for i in loose)
            moves = np.linalg.norm(p.jv[:, loose] * ua.BACKLASH[loose] / 2, axis=0)
            floating = max(floating, float(moves.sum()))
        out[tilt] = {
            "reached": len(poses),
            "repeat": np.max([repeatability(p, 1.0) for p in poses], axis=0),
            "free_joints": sorted(free),
            "floating": floating,
            "sag": max(float(np.linalg.norm(sag(a, p))) for p in poses),
        }
    return out


def budget() -> dict:
    """도구가 아래인 자세, 높이 Z_PLANE에서의 오차 예산. 길이는 m다."""
    pointer = arm("pointer", payload=False)
    empty, loaded = arm("gripper", payload=False), arm("gripper", payload=True)
    radii = sorted({*ua.RADII, *PAIR, ARC_R})

    def poses(a: Arm) -> dict[float, Pose]:
        found = {r: pose_at(a, r) for r in radii}
        missing = [r for r, p in found.items() if p is None]
        if missing:
            raise RuntimeError(f"{a.tool.name}: 작업 갈래로 닿지 않는 반지름이 있다: {missing}")
        return {r: p for r, p in found.items() if p is not None}

    pp, pe, pl = poses(pointer), poses(empty), poses(loaded)
    far = max(ua.RADII)
    hold = holding_torques(pointer.robot, pp[far].q, pointer.hori.masses)

    def source(moves: dict[float, np.ndarray]) -> dict:
        at_far = np.atleast_2d(moves[far].T).T
        size = np.sqrt((at_far**2).sum(axis=1))  # 몫이 여럿이면 축마다 제곱합의 제곱근
        radial, tangential = relative(moves)
        return {"far": size, "radial": radial, "tangential": tangential}

    top = pose_at(pointer, far, max(ua.HEIGHTS))
    sources = {
        "서보 처짐 (포인터)": source({r: sag(pointer, p) for r, p in pp.items()}),
        "서보 처짐 (그리퍼)": source({r: sag(empty, p) for r, p in pe.items()}),
        "서보 처짐 (그리퍼 + 짐 150 g)": source({r: sag(loaded, p) for r, p in pl.items()}),
        "짐 150 g을 들 때 더 처지는 양": source(
            {r: sag(loaded, pl[r]) - sag(empty, pe[r]) for r in radii}
        ),
        "영점 (핀 구멍 자리 ±0.1 mm)": source({r: zero_moves(p) for r, p in pp.items()}),
        "링크 길이 (±0.2 mm)": source({r: length_moves(p) for r, p in pp.items()}),
        "베어링 기울기 (좁게)": source(
            {r: tilt_moves(pointer, p) * TILT_PLAY[0] / 2 for r, p in pp.items()}
        ),
        "베어링 기울기 (넓게)": source(
            {r: tilt_moves(pointer, p) * TILT_PLAY[1] / 2 for r, p in pp.items()}
        ),
        "관의 휨 (포인터)": source({r: bending(pointer, p) for r, p in pp.items()}),
        "관의 휨 (그리퍼 + 짐)": source({r: bending(loaded, p) for r, p in pl.items()}),
        "J1 축 기울기": source({r: base_tilt(p) for r, p in pp.items()}),
    }
    # 처짐을 보정한 뒤 남는 몫 (포인터로 잴 때): 서로 상관없다고 보고 제곱합의 제곱근으로 모은다
    left = [sources[name] for name in REMAINING]
    return {
        "far": far,
        "remaining": tuple(
            math.sqrt(sum(s[key] ** 2 for s in left)) for key in ("radial", "tangential")
        ),
        "repeat": {band: repeatability(pp[far], band) for band in BANDS},
        "repeat_top": None if top is None else repeatability(top, 1.0),
        "tilted": tilted(pointer),
        "step_far": np.abs(pp[far].jv) * STEP,
        "free_joints": [i + 1 for i in range(6) if abs(hold[i]) < ua.BIAS_MIN],
        "j1_float": np.abs(pp[far].jv[:, 0]) * ua.BACKLASH[0] / 2,
        "sources": sources,
    }


def report() -> None:
    b = budget()
    mm = 1000.0
    print(
        f"## 오차 예산: 도구가 아래, TCP 높이 {Z_PLANE * 100:.0f} cm, 반지름 {b['far'] * 100:.0f} cm "
        "(X 반지름 쪽, Y 둘레 쪽, Z 높이) [mm]"
    )
    print("\n### 반복 정밀도 (같은 자세로 돌아올 때 흩어지는 양, 0012의 식)")
    print("| 관절이 흩어지는 폭 | X | Y | Z |")
    print("|---|---|---|---|")
    for band, value in b["repeat"].items():
        x, y, z = value * mm
        print(f"| ±{band:g}칸 | {x:.2f} | {y:.2f} | {z:.2f} |")
    step = b["step_far"] * mm
    print(
        "- 관절이 한 칸 움직일 때 TCP: "
        + ", ".join(
            f"J{i + 1} ({step[0, i]:.2f}, {step[1, i]:.2f}, {step[2, i]:.2f})" for i in range(6)
        )
    )
    if b["repeat_top"] is not None:
        x, y, z = b["repeat_top"] * mm
        print(f"- 높이 {max(ua.HEIGHTS) * 100:.0f} cm에서 ±1칸이면 ({x:.2f}, {y:.2f}, {z:.2f})")
    x, y, z = b["j1_float"] * mm
    print(
        f"- 중력이 누르지 않는 관절: {', '.join(f'J{j}' for j in b['free_joints'])}. "
        f"J1이 백래시({ua.BACKLASH[0] / DEG:.2f}°) 안에서 떠 있으면 Y로 ±{y:.2f}"
    )
    r, z = TILT_POINT
    print(
        f"\n### 기울인 자세 (포인터, 반지름 {r * 100:.0f} cm, 높이 {z * 100:.0f} cm, 8방향 중 가장 나쁜 값)"
    )
    print(
        "| 기울기 | 되는 방향 | 반복 정밀도 ±1칸 (X, Y, Z) | 중력이 누르지 않는 관절 (J1, J6 말고) "
        "| 그 관절이 떠 있으면 | 서보 처짐 |"
    )
    print("|---|---|---|---|---|---|")
    for tilt, t in b["tilted"].items():
        if t["reached"] == 0:
            print(f"| {tilt}° | 0 | - | - | - | - |")
            continue
        x, y, z = t["repeat"] * mm
        free = ", ".join(f"J{j}" for j in t["free_joints"]) or "없음"
        print(
            f"| {tilt}° | {t['reached']} | {x:.2f}, {y:.2f}, {z:.2f} | {free} | "
            f"±{t['floating'] * mm:.2f} | {t['sag'] * mm:.2f} |"
        )
    print("\n### 정확도 (자세마다 정해지는 오차)")
    print(
        f"| 몫 | X | Y | Z | 반지름 쪽 100 mm ({PAIR[0] * 100:.0f} → {PAIR[1] * 100:.0f} cm) "
        f"| 둘레 쪽 100 mm (반지름 {ARC_R * 100:.0f} cm) |"
    )
    print("|---|---|---|---|---|---|")
    for name, s in b["sources"].items():
        x, y, z = s["far"] * mm
        print(
            f"| {name} | {x:.2f} | {y:.2f} | {z:.2f} | {s['radial'] * mm:.2f} | {s['tangential'] * mm:.2f} |"
        )
    radial, tangential = b["remaining"]
    print(
        f"- 서보 처짐을 보정한 뒤 남는 몫({', '.join(REMAINING)})의 제곱합의 제곱근: "
        f"반지름 쪽 {radial * mm:.2f}, 둘레 쪽 {tangential * mm:.2f}"
    )


if __name__ == "__main__":
    report()
