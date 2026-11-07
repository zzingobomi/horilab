"""기구학 검토: 로봇이 할 수 있는 작업 범위에서 어깨 높이와 링크 길이를 고른다 (0017).

    uv run python -m horilab_design.studies.reach

- 작업 범위는 책상이 아니라 로봇에서 정한다. 먼저 서보가 버티는 반지름을 본다 (가장 무거운 도구인 평행 그리퍼
  + 짐 150 g, 정격의 0.8배). 그 안을 주 작업 영역으로 둔다: 반지름 15~30 cm, TCP 높이 2~18 cm.
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
from horilab_design.robot import Mat, Robot
from horilab_design.statics import backlash_displacements, worst_backlash_error
from horilab_design.studies import covers as cv
from horilab_design.studies import packaging as pk
from horilab_design.studies.common import RATED_MARGIN
from horilab_design.studies.packaging import DEG, TOOLS, Layout, Tool

RADII = tuple(round(0.10 + 0.02 * i, 2) for i in range(16))  # 10~40 cm
HEIGHTS = (0.02, 0.06, 0.10, 0.14, 0.18, 0.22)
TILTS = (30 * DEG, 45 * DEG)
# 주 작업 영역: 서보가 버티는 반지름 안에서 베이스 가까이 15 cm부터, 높이는 책상 위 물체(2 cm)부터
# 박스 4층 윗면 위 접근 높이(18 cm)까지
MAIN_R = (0.15, 0.30)
MAIN_Z = (0.02, 0.18)
# 관절마다 백래시 [rad]: STS3215 0.62°(하중 없이), ST-3120 0.5°(판매처), STS3250 0.43°(잰 범위의 큰 쪽)
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


def main() -> None:
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
