"""기구학 검토: 로봇이 할 수 있는 작업 범위에서 어깨 높이와 링크 길이를 고른다 (0017).

    uv run python -m horilab_design.studies.reach

- 작업 범위는 책상이 아니라 로봇에서 정한다. 먼저 서보가 버티는 반지름을 본다 (가장 무거운 도구인 평행 그리퍼
  + 짐 150 g, 정격의 0.8배). 그 안을 주 작업 영역으로 둔다: 반지름 16~30 cm, TCP 높이 2~18 cm.
- 고른 골격의 추가 확인(서보 부담 내역과 가속 여유, 특이점, 칸 사이와 경로)은 `... reach checks`로 돌린다.
- 팔은 J1 둘레로 돌기만 하므로, 닿는지와 자세를 바꿀 수 있는지는 J1 축에서 TCP까지의 반지름 r과 TCP 높이 z로
  정해진다. 그래서 (r, z) 격자에서 보고, 어디에 둘지는 마지막에 따로 본다.
- 형상은 UR식 외형(covers.py, 0019)이고, 부딪힘과 관절 범위(params/arm.toml)를 넣는다. 손목 오프셋은 0019 그대로다.
- 칸마다 본다:
  - 도구가 아래를 향해 닿는지 (흡착, 평행 그리퍼)
  - 끝을 그 점에 둔 채 30°·45°로 8방향 기울일 수 있는지 (포인터)
  - 팔꿈치가 펴진 정도 |sin J3|. 0이면 팔이 다 펴진 특이점이다
  - 버팀 토크 / 정격 (흡착 + 짐 100 g, 평행 그리퍼 + 짐 150 g, 도구가 아래)
  - 백래시가 TCP를 흔드는 가장 나쁜 양 (관절마다 다른 사람이 잰 값, docs/hardware.md)
"""

from __future__ import annotations

import itertools
import math
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, replace

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Mat, Robot, link_poses
from horilab_design.statics import (
    FLANGE,
    PointMass,
    backlash_displacements,
    frame_poses,
    holding_torques,
    worst_backlash_error,
)
from horilab_design.studies import covers as cv
from horilab_design.studies import packaging as pk
from horilab_design.studies.common import RATED_MARGIN
from horilab_design.studies.packaging import DEG, TOOLS, Layout, Tool
from horilab_design.values import KGCM, load_servos

RADII = tuple(round(0.10 + 0.02 * i, 2) for i in range(16))  # 10~40 cm
HEIGHTS = (0.02, 0.06, 0.10, 0.14, 0.18, 0.22)
TILTS = (30 * DEG, 45 * DEG)
# 주 작업 영역: 서보가 버티는 반지름 안에서 베이스 가까이 16 cm부터, 높이는 책상 위 물체(2 cm)부터
# 박스 4층 윗면 위 접근 높이(18 cm)까지. 반지름 15 cm는 낮은 곳에서 팔꿈치 접힘 한계에 걸린다 (continuity_report)
MAIN_R = (0.16, 0.30)
MAIN_Z = (0.02, 0.18)
# 관절마다 백래시 [rad]: STS3215 0.62°(논문, 하중 없이, 원문 재확인 못 함), ST-3120 0.5°(판매처),
# STS3250 0.43°(robonine, 서보 한 개를 한 번 잰 값). 출처와 잰 방법은 docs/hardware.md의 "다른 사람이 잰 값"
BACKLASH = np.radians([0.62, 0.5, 0.43, 0.62, 0.62, 0.62])
BEFORE = Layout(shoulder=0.12, upper=0.20, fore=0.17)  # 0017의 처음 길이 (박스 쌓기 자세로 골랐다)
CANDIDATES = [
    *itertools.product(
        (0.12, 0.14, 0.16, 0.18), (0.20, 0.21, 0.22, 0.23), (0.16, 0.17, 0.18, 0.19)
    ),
    # 아래팔이나 위팔이 더 긴 쪽과 어깨가 더 높은 쪽의 경향
    (0.16, 0.20, 0.21),
    (0.16, 0.22, 0.21),
    (0.16, 0.24, 0.18),
    (0.16, 0.24, 0.21),
    (0.20, 0.22, 0.18),
]
DESK = (0.55, 0.34)  # 작업대 가로 × 세로 [m] (goals.md)
BASE_FROM_EDGE = 0.06  # 책상 뒤 가장자리에서 베이스 중심까지 [m] (0018의 어림)
DIRECTIONS = (
    "바깥",
    "바깥 왼쪽",
    "왼쪽",
    "안쪽 왼쪽",
    "안쪽",
    "안쪽 오른쪽",
    "오른쪽",
    "바깥 오른쪽",
)


@dataclass
class Cell:
    down: bool = False  # 흡착 도구가 아래를 향해 닿는다
    grip: bool = False  # 평행 그리퍼(35 mm 더 길다)도 아래를 향해 닿는다
    tilts: tuple[int, int] = (0, 0)  # 30°, 45°로 기울일 수 있는 방향 수 (8개 중)
    sin3: float = 0.0
    # 닿는 도구 중 가장 나쁜 J2, J3, J4 버팀 토크 / 정격 (흡착 + 100 g, 평행 그리퍼 + 150 g)
    torque: tuple[float, float, float] = (0.0, 0.0, 0.0)
    backlash: float = 0.0  # [m]


@dataclass
class Scenes:
    robot: Robot
    ik: AnalyticIK
    by_tool: dict[str, tuple[pk.Scene, list[pk.Prim]]]


def scenes(layout: Layout) -> Scenes:
    by_tool = {}
    robot = None
    for key in ("suction", "pointer", "gripper"):
        robot, prims, _, _ = cv.build_ur_arm(layout, TOOLS[key])
        by_tool[key] = (pk.build_scene(robot, prims), prims)
    assert robot is not None
    return Scenes(robot, AnalyticIK(robot), by_tool)


def free_solution(s: Scenes, key: str, T: Mat, tool: Tool) -> np.ndarray | None:
    scene, _ = s.by_tool[key]
    for q in pk.solve(s.ik, T, tool.symmetric):
        if not pk.collisions(pk.hits(scene, q)):
            return q
    return None


def down_pose(r: float, z: float, tool: Tool) -> Mat:
    return pk.tool_pose((r, 0.0, z), 0.0, 0.0, 0.0, tool.length)


def tilted_pose(r: float, z: float, tilt: float, k: int) -> Mat:
    """포인터 끝을 (r, z)에 두고 k번째 방향(0이 바깥, 45°씩 왼쪽으로 돈다)으로 기울인 자세."""
    return pk.tool_pose((r, 0.0, z), tilt, k * 45 * DEG, 0.0, TOOLS["pointer"].length)


def cell(s: Scenes, r: float, z: float) -> Cell:
    out = Cell()
    suction, pointer, gripper = TOOLS["suction"], TOOLS["pointer"], TOOLS["gripper"]
    q = free_solution(s, "suction", down_pose(r, z, suction), suction)
    if q is not None:
        out.down = True
        out.sin3 = abs(math.sin(float(q[2])))
        moves = backlash_displacements(s.robot, q, BACKLASH, (0.0, 0.0, suction.length))
        out.backlash = worst_backlash_error(moves)
    n30, n45 = (
        sum(
            free_solution(s, "pointer", tilted_pose(r, z, tilt, k), pointer) is not None
            for k in range(8)
        )
        for tilt in TILTS
    )
    out.tilts = (n30, n45)
    qg = free_solution(s, "gripper", down_pose(r, z, gripper), gripper)
    out.grip = qg is not None
    for key, tool, qt in (("suction", suction, q), ("gripper", gripper, qg)):
        if qt is None:
            continue
        _, prims = s.by_tool[key]
        ratio = pk.torque_ratios(s.robot, prims, tool, [qt])
        a, b, c = out.torque
        out.torque = (max(a, float(ratio[1])), max(b, float(ratio[2])), max(c, float(ratio[3])))
    return out


def in_main(r: float, z: float) -> bool:
    return MAIN_R[0] <= r <= MAIN_R[1] + 1e-9 and MAIN_Z[0] <= z <= MAIN_Z[1] + 1e-9


@dataclass(frozen=True)
class Summary:
    layout: Layout
    grid: dict[tuple[float, float], Cell]

    def main_cells(self) -> list[Cell]:
        return [c for (r, z), c in self.grid.items() if in_main(r, z)]

    def share(self, test: Callable[[Cell], bool]) -> float:
        cells = self.main_cells()
        return sum(bool(test(c)) for c in cells) / len(cells)

    def worst(self, value: Callable[[Cell], float]) -> float:
        cells = [c for c in self.main_cells() if c.down]
        return max((value(c) for c in cells), default=math.inf)

    def reach(self, z: float) -> tuple[float, float]:
        rs = [r for (r, zz), c in self.grid.items() if zz == z and c.down]
        return (min(rs), max(rs)) if rs else (0.0, 0.0)

    def missed(self) -> list[str]:
        """주 작업 영역에서 도구를 아래로 향해 닿지 않는 칸 (r,z [cm], s 흡착, g 평행 그리퍼)."""
        return [
            f"{r * 100:.0f},{z * 100:.0f}{'' if c.down else 's'}{'' if c.grip else 'g'}"
            for (r, z), c in sorted(self.grid.items())
            if in_main(r, z) and not (c.down and c.grip)
        ]

    def full(self) -> bool:
        return self.share(lambda c: c.down and c.grip) == 1.0

    def torque_ok(self) -> bool:
        return max(self.worst(lambda c, j=j: c.torque[j]) for j in range(3)) <= RATED_MARGIN


def evaluate(layout: Layout) -> Summary:
    s = scenes(layout)
    return Summary(layout, {(r, z): cell(s, r, z) for r, z in itertools.product(RADII, HEIGHTS)})


def label(layout: Layout) -> str:
    return f"{layout.shoulder * 100:.0f} / {layout.upper * 100:.0f} / {layout.fore * 100:.0f}"


def servo_reach(layout: Layout) -> None:
    """반지름마다 버팀 토크 / 정격. 서보가 버티는 작업 반지름을 본다."""
    s = scenes(layout)
    gripper = TOOLS["gripper"]
    _, prims = s.by_tool["gripper"]
    print(
        f"\n## 서보가 버티는 반지름 ({label(layout)}): 평행 그리퍼 + 짐 150 g, 도구 아래, "
        f"버팀 토크 / 정격 (J2 · J3), 목표는 {RATED_MARGIN} 이하"
    )
    radii = [round(0.26 + 0.02 * i, 2) for i in range(6)]
    print("| 높이 \\ 반지름 [cm] | " + " | ".join(f"{r * 100:.0f}" for r in radii) + " |")
    print("|---|" + "---|" * len(radii))
    for z in (0.02, 0.10, 0.18):
        row = []
        for r in radii:
            q = free_solution(s, "gripper", down_pose(r, z, gripper), gripper)
            if q is None:
                row.append("닿지 않음")
                continue
            t = pk.torque_ratios(s.robot, prims, gripper, [q])
            row.append(f"{t[1]:.2f} · {t[2]:.2f}")
        print(f"| {z * 100:.0f} | " + " | ".join(row) + " |")


def grid_map(sm: Summary) -> None:
    """칸마다 한 글자: '#' 45° 8방향, '+' 30° 8방향, 'o' 도구 아래로만, '.' 닿지 않음. '!'는 버팀 토크가 목표를 넘음."""
    print(f"\n### 지도: {label(sm.layout)} (가로 반지름 cm, 세로 TCP 높이 cm)")
    print("```")
    print("높이 \\ 반지름 " + " ".join(f"{r * 100:>3.0f}" for r in RADII))
    for z in reversed(HEIGHTS):
        row = []
        for r in RADII:
            c = sm.grid[(r, z)]
            mark = "#" if c.tilts[1] == 8 else "+" if c.tilts[0] == 8 else "o" if c.down else "."
            if c.down and max(c.torque) > RATED_MARGIN:
                mark = "!"
            row.append(f"{mark:>3}")
        print(f"{z * 100:>12.0f} " + " ".join(row))
    print("```")


def candidates_table(base: Layout) -> list[Summary]:
    print(
        f"\n## 후보 비교: 주 작업 영역 (반지름 {MAIN_R[0] * 100:.0f}~{MAIN_R[1] * 100:.0f} cm, "
        f"높이 {MAIN_Z[0] * 100:.0f}~{MAIN_Z[1] * 100:.0f} cm)"
    )
    print(
        "| 어깨 높이 / 위팔 / 아래팔 [cm] | 도구 아래로 닿음 (그리퍼) | 닿지 않는 칸 (r,z) | 30° 8방향 | "
        "45° 6방향 이상 (8방향) | 가장 펴진 팔꿈치 \\|sin J3\\| | 버팀 토크 J2 · J3 · J4 | "
        "백래시 끝 오차 [mm] | 닿는 끝 (높이 2 cm) [cm] |"
    )
    print("|---|---|---|---|---|---|---|---|---|")
    out = []
    for shoulder, upper, fore in CANDIDATES:
        sm = evaluate(replace(base, shoulder=shoulder, upper=upper, fore=fore))
        out.append(sm)
        t = [sm.worst(lambda c, j=j: c.torque[j]) for j in range(3)]
        print(
            f"| {label(sm.layout)} | {sm.share(lambda c: c.down) * 100:.0f}% "
            f"({sm.share(lambda c: c.grip) * 100:.0f}%) | {' '.join(sm.missed()) or '-'} | "
            f"{sm.share(lambda c: c.tilts[0] == 8) * 100:.0f}% | {sm.share(lambda c: c.tilts[1] >= 6) * 100:.0f}% "
            f"({sm.share(lambda c: c.tilts[1] == 8) * 100:.0f}%) | "
            f"{min((c.sin3 for c in sm.main_cells() if c.down), default=0):.2f} | "
            f"{t[0]:.2f} · {t[1]:.2f} · {t[2]:.2f} | {sm.worst(lambda c: c.backlash) * 1000:.2f} | "
            f"{sm.reach(0.02)[1] * 100:.0f} |",
            flush=True,
        )
    return out


def _widened(robot: Robot) -> Robot:
    """관절마다 범위를 지금 가운데에서 한 바퀴로 넓힌 팔. 무엇이 막는지 나눌 때만 쓴다."""
    joints = []
    for j in robot.joints:
        mid = (j.lower + j.upper) / 2
        joints.append(replace(j, lower=mid - math.pi + 1e-6, upper=mid + math.pi - 1e-6))
    return Robot(robot.name, tuple(joints), robot.T_last_flange)


def _overrun(robot: Robot, q: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, np.maximum(robot.lower - q, q - robot.upper))


def blocker(s: Scenes, wide_ik: AnalyticIK, T: Mat) -> tuple[str, str]:
    """포인터를 기울인 자세 하나가 왜 안 되는지와, 부딪히면 그 쌍."""
    tool = TOOLS["pointer"]
    scene, _ = s.by_tool["pointer"]
    sols = pk.solve(s.ik, T, tool.symmetric)
    found = [pk.collisions(pk.hits(scene, q)) for q in sols]
    if any(not bad for bad in found):
        return "됨", ""
    if sols:
        # 가장 덜 겹치는 해에서 가장 깊이 겹친 쌍
        bad = max(found, key=lambda b: min(h.dist for h in b))
        h = min(bad, key=lambda h: h.dist)
        return "부딪힘", f"{h.a} ↔ {h.b}"
    wide = pk.solve(wide_ik, T, tool.symmetric)
    if wide:
        best = min(wide, key=lambda q: float(_overrun(s.robot, q).sum()))
        return f"J{int(np.argmax(_overrun(s.robot, best))) + 1} 범위 밖", ""
    if len(wide_ik.solve_in_limits(T)):
        return "손목을 뒤집어야 닿음", ""
    return "못 닿음", ""


def blockers_table(layout: Layout, tilt: float = 45 * DEG) -> None:
    """주 작업 영역의 칸마다 방향별로, 기울이기가 안 되는 까닭을 센다."""
    s = scenes(layout)
    wide_ik = AnalyticIK(_widened(s.robot))
    cells = [(r, z) for r, z in itertools.product(RADII, HEIGHTS) if in_main(r, z)]
    counts: list[Counter[str]] = []
    pairs: Counter[str] = Counter()
    for k in range(8):
        c: Counter[str] = Counter()
        for r, z in cells:
            why, pair = blocker(s, wide_ik, tilted_pose(r, z, tilt, k))
            c[why] += 1
            if pair:
                pairs[pair] += 1
        counts.append(c)
    reasons = [
        "됨",
        "J3 범위 밖",
        "J4 범위 밖",
        "J5 범위 밖",
        "손목을 뒤집어야 닿음",
        "부딪힘",
        "못 닿음",
    ]
    reasons += sorted({why for c in counts for why in c} - set(reasons))
    print(
        f"\n## {tilt / DEG:.0f}° 기울이기를 막는 것 ({label(layout)}, 주 작업 영역 {len(cells)}칸): "
        "방향은 도구 끝이 향하는 쪽, 바깥은 베이스 반대쪽"
    )
    print("| 방향 | " + " | ".join(reasons) + " |")
    print("|---|" + "---|" * len(reasons))
    for k, c in enumerate(counts):
        print(f"| {DIRECTIONS[k]} | " + " | ".join(str(c.get(why, 0)) for why in reasons) + " |")
    print("- 부딪히는 쌍: " + ", ".join(f"{p} {n}" for p, n in pairs.most_common()))


def desk_share(width: float, depth: float, base_from_edge: float, n: int = 400) -> float:
    """주 작업 영역(베이스 앞쪽 반원 고리)의 넓이 중 책상 안에 드는 비율. 베이스는 긴 변 가운데에 둔다."""
    r = np.linspace(MAIN_R[0], MAIN_R[1], n)
    th = np.linspace(-math.pi / 2, math.pi / 2, n)
    rr, tt = np.meshgrid(r, th)
    inside = (rr * np.cos(tt) <= depth - base_from_edge) & (np.abs(rr * np.sin(tt)) <= width / 2)
    return float((inside * rr).sum() / rr.sum())


# ----------------------------------------------------------------------------------------------------
# 고른 골격의 추가 확인: 서보 부담의 내역과 가속 여유, 특이점, 칸 사이와 경로의 연속성

# 버팀 토크를 나눠 보는 묶음 (링크 번호 → 이름). 7은 도구다 (형상은 링크 6에 붙어 있다).
PARTS = (
    ((2,), "위팔 (J3 서보 포함)"),
    ((3,), "아래팔 (J4 서보 포함)"),
    ((4, 5, 6), "손목 (J5·J6 서보, D405, 플랜지, 교체부)"),
    ((7,), "도구 + 짐"),
)


def _part(p: pk.Prim) -> int:
    return 7 if p.material == "tool" else p.link


def _masses(
    prims: list[pk.Prim], tool: Tool, links: tuple[int, ...] | None = None
) -> list[PointMass]:
    out = [
        PointMass(
            p.name, min(p.link, 6), (float(p.pos[0]), float(p.pos[1]), float(p.pos[2])), p.mass
        )
        for p in prims
        if p.mass > 0 and (links is None or _part(p) in links)
    ]
    if tool.payload and (links is None or 7 in links):
        out.append(PointMass("짐", FLANGE, (0.0, 0.0, tool.length), tool.payload))
    return out


def _worst_down(s: Scenes, key: str, joint: int) -> tuple[float, float, np.ndarray] | None:
    """주 작업 영역에서 도구가 아래일 때 관절 joint의 버팀 토크가 가장 큰 칸과 그 자세 (닿는 칸이 없으면 None)."""
    tool = TOOLS[key]
    _, prims = s.by_tool[key]
    masses = _masses(prims, tool)
    best: tuple[float, float, float, np.ndarray] | None = None
    for r, z in itertools.product(RADII, HEIGHTS):
        if not in_main(r, z):
            continue
        q = free_solution(s, key, down_pose(r, z, tool), tool)
        if q is None:
            continue
        tau = abs(float(holding_torques(s.robot, q, masses)[joint]))
        if best is None or tau > best[0]:
            best = (tau, r, z, q)
    return None if best is None else (best[1], best[2], best[3])


def _inertia(robot: Robot, q: np.ndarray, masses: list[PointMass], joint: int) -> float:
    """관절 축에 대한 관성 [kg·m²]. 부품을 점 질량으로 보고, 그 관절이 움직이는 것만 센다."""
    frames = frame_poses(robot, q)
    n = len(robot.joints)
    origin = frames[joint + 1][:3, 3]
    axis = frames[joint + 1][:3, :3] @ np.asarray(robot.joints[joint].axis)
    total = 0.0
    for m in masses:
        index = n + 1 if m.link == FLANGE else m.link
        if index <= joint:
            continue
        T = frames[index]
        d = T[:3, :3] @ np.asarray(m.position) + T[:3, 3] - origin
        total += m.mass * float(np.sum((d - (d @ axis) * axis) ** 2))
    return total


def torque_report(layouts: tuple[Layout, ...]) -> None:
    """어깨(J2)와 팔꿈치(J3)의 가장 나쁜 칸에서 버팀 토크를 부품 묶음별로 나누고, 정격까지 남은 몫으로 낼 수 있는 가속을 본다."""
    key = "gripper"
    tool = TOOLS[key]
    print(
        f"\n## 서보 부담의 내역과 가속 여유 (평행 그리퍼 + 짐 {tool.payload * 1000:.0f} g, 도구 아래, 주 작업 영역에서 가장 나쁜 칸)"
    )
    print(
        "버팀 토크 / 정격을 부품 묶음별로 나눈다. 가속 여유는 정격까지 남은 토크를 관절 축에 대한 관성(부품을 점 질량으로 어림, "
        "서보 안 모터의 관성은 자료가 없어 뺌)으로 나눈 각가속도다."
    )
    header = "| 골격 | 관절 | 칸 (r, z) [cm] | " + " | ".join(name for _, name in PARTS)
    print(header + " | 합 | 관성 [kg·m²] | 가속 여유 [°/s²] | 60°/s까지 [s] |")
    print("|---|---|---|" + "---|" * len(PARTS) + "---|---|---|---|")
    for layout in layouts:
        s = scenes(layout)
        _, prims = s.by_tool[key]
        for joint, name in ((1, "J2 어깨"), (2, "J3 팔꿈치")):
            worst = _worst_down(s, key, joint)
            if worst is None:
                # 어깨가 돌림판 윗면(packaging.TURNTABLE_TOP)보다 많이 낮은 골격은 J2 원통이 돌림판에 닿는다
                print(
                    f"| {label(layout)} | {name} | 닿는 칸 없음 | "
                    + " | ".join("-" for _ in PARTS)
                    + " | - | - | - | - |"
                )
                continue
            r, z, q = worst
            rated = s.robot.joints[joint].effort
            parts = [
                abs(float(holding_torques(s.robot, q, _masses(prims, tool, links))[joint])) / rated
                for links, _ in PARTS
            ]
            total = abs(float(holding_torques(s.robot, q, _masses(prims, tool))[joint])) / rated
            inertia = _inertia(s.robot, q, _masses(prims, tool), joint)
            accel = (1.0 - total) * rated / inertia
            print(
                f"| {label(layout)} | {name} | {r * 100:.0f}, {z * 100:.0f} | "
                + " | ".join(f"{v:.2f}" for v in parts)
                + f" | {total:.2f} | {inertia:.4f} | {math.degrees(accel):.0f} | "
                f"{math.radians(60) / accel:.2f} |"
            )


SERVO_OF_JOINT = ("STS3215", "ST-3120", "STS3250", "STS3215", "STS3215", "STS3215")
# 서보 줄을 어댑터의 두 단자로 나눈다 (0016): 단자 1은 J1→J2, 단자 2는 J3→J4→J5→J6→그리퍼
PORTS = ((0, 1), (2, 3, 4, 5))


def holding_currents(layout: Layout) -> tuple[np.ndarray, np.ndarray, tuple[str, float, float]]:
    """주 작업 영역에서 두 도구가 아래일 때 버팀 전류 [A]: (관절마다 가장 큰 값, 합이 가장 큰 자세의 관절별 값, 그 자세).

    전류는 토크에 비례한다고 보고 스톨 토크와 스톨 전류로 센다. 서보의 대기 전류는 뺀다.
    """
    servos = load_servos()
    stall_t = np.array([servos[n].stall_torque_kgcm * KGCM for n in SERVO_OF_JOINT])
    stall_i = np.array([servos[n].stall_current_a for n in SERVO_OF_JOINT])
    s = scenes(layout)
    worst, at_sum, where = np.zeros(6), np.zeros(6), ("", 0.0, 0.0)
    for key in ("suction", "gripper"):
        tool = TOOLS[key]
        _, prims = s.by_tool[key]
        masses = _masses(prims, tool)
        for r, z in itertools.product(RADII, HEIGHTS):
            if not in_main(r, z):
                continue
            q = free_solution(s, key, down_pose(r, z, tool), tool)
            if q is None:
                continue
            amps = stall_i * np.abs(holding_torques(s.robot, q, masses)) / stall_t
            worst = np.maximum(worst, amps)
            if amps.sum() > at_sum.sum():
                at_sum, where = amps, (key, r, z)
    return worst, at_sum, where


def current_report(layout: Layout) -> None:
    worst, at_sum, (key, r, z) = holding_currents(layout)
    print("\n## 버팀 전류 (주 작업 영역, 도구 아래. 토크에 비례로 셈, 대기 전류와 그리퍼 서보 뺌)")
    print("| | J1 | J2 | J3 | J4 | J5 | J6 | 합 |")
    print("|---|---|---|---|---|---|---|---|")
    print("| 관절마다 가장 큰 값 [A] | " + " | ".join(f"{a:.2f}" for a in worst) + " | - |")
    print(
        f"| 합이 가장 큰 자세 ({'평행 그리퍼' if key == 'gripper' else '흡착'}, r {r * 100:.0f}, "
        f"z {z * 100:.0f} cm) [A] | "
        + " | ".join(f"{a:.2f}" for a in at_sum)
        + f" | {at_sum.sum():.2f} |"
    )
    ports = ", ".join(
        f"단자 {k + 1}(J{'·J'.join(str(j + 1) for j in joints)}) {at_sum[list(joints)].sum():.2f} A"
        for k, joints in enumerate(PORTS)
    )
    print(f"- 한 줄이면 첫 구간에 {at_sum.sum():.2f} A가 흐른다. 두 단자로 나누면 {ports}다.")


def _free_solutions(s: Scenes, key: str, T: Mat, tool: Tool) -> list[np.ndarray]:
    scene, _ = s.by_tool[key]
    return [q for q in pk.solve(s.ik, T, tool.symmetric) if not pk.collisions(pk.hits(scene, q))]


@dataclass
class Track:
    blocked: int  # 부딪힘 없는 작업 자세의 해가 없는 걸음 수
    step: float  # 한 걸음에 가장 크게 바뀐 관절 각 [rad]
    joint: int  # 그 관절 (1~6)


def track(s: Scenes, key: str, poses: list[Mat]) -> Track:
    """자세를 차례로 따라가며 앞 걸음의 해에 가장 가까운 해를 고른다. 관절이 갑자기 크게 바뀌면 이어지지 않는 것이다."""
    tool = TOOLS[key]
    prev: np.ndarray | None = None
    out = Track(0, 0.0, 0)
    for T in poses:
        sols = _free_solutions(s, key, T, tool)
        if not sols:
            out.blocked += 1
            prev = None
            continue
        if prev is None:
            q = sols[0]
        else:
            p = prev
            q = min(sols, key=lambda c: float(np.abs(c - p).max()))
            d = np.abs(q - p)
            j = int(np.argmax(d))
            if d[j] > out.step:
                out.step, out.joint = float(d[j]), j + 1
        prev = q
    return out


def singularity_report(layout: Layout) -> None:
    """UR형 팔의 특이점은 셋이다: 팔꿈치(J3 = 0), 손목(J5 = 0), 어깨(J6 점이 J1 축 쪽 평면에 옴).
    주 작업 영역의 된 자세들에서 셋과 얼마나 떨어져 있는지 본다."""
    s = scenes(layout)
    rows: dict[str, list[tuple[float, float, float, float, float]]] = {}
    for r, z in itertools.product(RADII, HEIGHTS):
        if not in_main(r, z):
            continue
        poses: list[tuple[str, str, Mat]] = [
            ("흡착, 도구 아래", "suction", down_pose(r, z, TOOLS["suction"])),
            ("평행 그리퍼, 도구 아래", "gripper", down_pose(r, z, TOOLS["gripper"])),
        ]
        for tilt in TILTS:
            for k in range(8):
                poses.append(
                    (f"포인터 {tilt / DEG:.0f}° 기울임", "pointer", tilted_pose(r, z, tilt, k))
                )
        for kind, key, T in poses:
            q = free_solution(s, key, T, TOOLS[key])
            if q is None:
                continue
            p6 = link_poses(s.robot, q)[5][:3, 3]
            reach6 = math.sqrt(max(float(np.hypot(p6[0], p6[1])) ** 2 - layout.d45**2, 0.0))
            rows.setdefault(kind, []).append(
                (abs(math.sin(float(q[2]))), abs(math.sin(float(q[4]))), reach6, r, z)
            )
    print(
        f"\n## 특이점과의 거리 ({label(layout)}, 주 작업 영역의 된 자세들 중 가장 가까운 값). "
        "팔꿈치·손목은 sin 절댓값(1이 가장 멀다), 어깨는 J6 점이 팔 방향으로 J1 축에서 떨어진 거리"
    )
    print(
        "| 자세 | 자세 수 | 팔꿈치 \\|sin J3\\| (그 칸 r, z) | 팔꿈치 0.3 아래인 자세 | 손목 \\|sin J5\\| | 어깨 [cm] |"
    )
    print("|---|---|---|---|---|---|")
    for kind, vals in rows.items():
        a = np.array(vals)
        i = int(np.argmin(a[:, 0]))
        print(
            f"| {kind} | {len(a)} | {a[i, 0]:.2f} ({a[i, 3] * 100:.0f}, {a[i, 4] * 100:.0f}) | "
            f"{int((a[:, 0] < 0.3).sum())} | {a[:, 1].min():.2f} | {a[:, 2].min() * 100:.1f} |"
        )


def continuity_report(layout: Layout) -> None:
    """검사한 칸 사이도 되는지 본다: 1 cm 격자로 다시 보고, 직선 이동과 끝을 둔 채 방향 바꾸기를 잘게 따라간다."""
    s = scenes(layout)
    r0, r1 = MAIN_R
    rs = [round(r0 + 0.01 * i, 2) for i in range(round((r1 - r0) / 0.01) + 1)]
    zs = [round(MAIN_Z[0] + 0.01 * i, 2) for i in range(round((MAIN_Z[1] - MAIN_Z[0]) / 0.01) + 1)]
    miss = []
    for r, z in itertools.product(rs, zs):
        for key in ("suction", "gripper"):
            tool = TOOLS[key]
            if free_solution(s, key, down_pose(r, z, tool), tool) is None:
                miss.append(f"{r * 100:.0f},{z * 100:.0f}{key[0]}")
    print(
        f"\n## 칸 사이와 경로 ({label(layout)})\n"
        f"- 1 cm 격자 {len(rs) * len(zs)}칸(반지름 {r0 * 100:.0f}~{r1 * 100:.0f}, 높이 "
        f"{MAIN_Z[0] * 100:.0f}~{MAIN_Z[1] * 100:.0f} cm)에서 도구를 아래로 향해 닿지 않는 칸: "
        f"{' '.join(miss) or '없음'} (s 흡착, g 그리퍼)"
    )
    inner = []
    for z in (0.02, 0.06, 0.10, 0.14, 0.18):
        near = [
            r
            for r in np.arange(0.10, r0 + 1e-9, 0.005)
            if all(
                free_solution(s, key, down_pose(float(r), z, TOOLS[key]), TOOLS[key]) is not None
                for key in ("suction", "gripper")
            )
        ]
        inner.append(
            f"높이 {z * 100:.0f} cm {near[0] * 100:.1f} cm"
            if near
            else f"높이 {z * 100:.0f} cm 없음"
        )
    print("- 베이스 쪽으로 두 도구가 다 닿는 가장 가까운 반지름 (0.5 cm 간격): " + ", ".join(inner))
    suction = TOOLS["suction"]
    lines = (
        ("가까이 → 멀리, 높이 2 cm", (r0, 0.02), (r1, 0.02)),
        ("가까이 → 멀리, 높이 18 cm", (r0, 0.18), (r1, 0.18)),
        (f"아래 → 위, 반지름 {r0 * 100:.0f} cm", (r0, 0.02), (r0, 0.18)),
        (f"아래 → 위, 반지름 {r1 * 100:.0f} cm", (r1, 0.02), (r1, 0.18)),
        ("가까운 위 → 먼 아래", (r0, 0.18), (r1, 0.02)),
    )
    print("- 흡착을 아래로 향한 채 5 mm 간격으로 직선 이동 (앞 걸음에 가장 가까운 해를 따라감):")
    for name, (ra, za), (rb, zb) in lines:
        n = math.ceil(math.hypot(rb - ra, zb - za) / 0.005) + 1
        poses = [
            down_pose(ra + (rb - ra) * t, za + (zb - za) * t, suction) for t in np.linspace(0, 1, n)
        ]
        t = track(s, "suction", poses)
        print(
            f"  - {name}: {n}걸음, 막힌 걸음 {t.blocked}, 한 걸음 최대 J{t.joint} {math.degrees(t.step):.1f}°"
        )
    pointer = TOOLS["pointer"]
    print("- 포인터 끝을 한 점에 둔 채 30° 기울인 방향을 5°씩 한 바퀴 돌림:")
    for r, z in ((0.24, 0.02), (0.25, 0.02), (0.26, 0.05)):
        poses = [
            pk.tool_pose((r, 0.0, z), 30 * DEG, a * DEG, 0.0, pointer.length)
            for a in range(0, 361, 5)
        ]
        t = track(s, "pointer", poses)
        print(
            f"  - 반지름 {r * 100:.0f} cm, 높이 {z * 100:.0f} cm: 막힌 걸음 {t.blocked}/73, "
            f"5°마다 최대 J{t.joint} {math.degrees(t.step):.1f}°"
        )


def checks() -> None:
    base = Layout()
    print("# 고른 골격의 추가 확인")
    torque_report((BEFORE, base))
    current_report(base)
    singularity_report(base)
    continuity_report(base)


def main() -> None:
    import sys

    if sys.argv[1:] == ["checks"]:
        checks()
        return
    base = Layout()  # params/arm.toml의 골격
    print("# 기구학 검토: 로봇의 작업 범위에서 고른 골격")
    servo_reach(base)
    for layout in (BEFORE, base):
        sm = evaluate(layout)
        grid_map(sm)
        low = [c for (r, z), c in sm.grid.items() if z == MAIN_Z[0] and in_main(r, z)]
        print(
            f"- 책상 높이({MAIN_Z[0] * 100:.0f} cm)에서 기울일 수 있는 방향 수의 합 ({len(low)}칸 × 8방향 중): "
            f"30° {sum(c.tilts[0] for c in low)}, 45° {sum(c.tilts[1] for c in low)}"
        )
    summaries = candidates_table(base)
    good = [sm for sm in summaries if sm.full() and sm.torque_ok()]
    good.sort(
        key=lambda sm: (
            -sm.share(lambda c: c.tilts[1] >= 6),
            -sm.share(lambda c: c.tilts[0] == 8),
            -min(c.sin3 for c in sm.main_cells()),
        )
    )
    print(
        "\n## 두 도구가 주 작업 영역에 다 닿고 버팀 토크가 목표 안인 후보 (45° 6방향 이상이 넓은 순)"
    )
    for sm in good:
        print(
            f"- {label(sm.layout)}: 45° 6방향 이상 {sm.share(lambda c: c.tilts[1] >= 6) * 100:.0f}%, "
            f"30° 8방향 {sm.share(lambda c: c.tilts[0] == 8) * 100:.0f}%, "
            f"|sin J3| 최소 {min(c.sin3 for c in sm.main_cells()):.2f}, "
            f"J3 {sm.worst(lambda c: c.torque[1]):.2f}"
        )
    blockers_table(base)
    share = desk_share(*DESK, BASE_FROM_EDGE)
    print(
        f"\n## 책상: {DESK[0] * 100:.0f} × {DESK[1] * 100:.0f} cm, 베이스 중심이 뒤 가장자리에서 "
        f"{BASE_FROM_EDGE * 100:.0f} cm일 때 주 작업 영역의 {share * 100:.0f}%가 책상 위다. 다 들어가려면 가로 "
        f"{2 * MAIN_R[1] * 100:.0f} cm, 세로 {(BASE_FROM_EDGE + MAIN_R[1]) * 100:.0f} cm가 필요하다."
    )


if __name__ == "__main__":
    main()
