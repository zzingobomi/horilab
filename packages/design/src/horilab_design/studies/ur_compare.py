"""UR3e와 견주기 (0027): 호리봇 크기로 줄인 UR3e와 지금의 호리봇에 같은 자세 시험을 돌린다.

    uv run python -m horilab_design.studies.ur_compare          # 견주기
    uv run python -m horilab_design.studies.ur_compare check    # 원본 UR3e 모델이 공개 값과 맞는지
    uv run python -m horilab_design.studies.ur_compare free     # 가정: 호리봇 관절 구조가 막지 않으면
    uv run python -m horilab_design.studies.ur_compare bare     # 덮개 없이 서보가 드러난 ㄷ자 브래킷

- 줄이는 비율은 위팔 + 아래팔 길이를 맞춘다 (0027의 견주는 방법, 제안).
- 두 팔에 같은 도구(교체 어댑터 + 평행 그리퍼)를 달고, 같은 책상 면 위에 둔다. 부딪힘은 형상으로 본다.
  UR3e는 UR이 공개한 충돌 형상(ur3e.py), 호리봇은 UR식 외형 검토 형상(covers.py)이다.
- TCP 자리는 베이스 축에서 반지름 r, 책상에서 높이 z인 점이다. 팔이 J1 둘레로 돌므로 한 방위만 본다. 방위는
  왼쪽(90°)이다. 호리봇 J1의 두 해(목표를 마주 봄, 어깨 위로 넘겨 뒤로 뻗음)가 다 J1 범위 안에 드는 방위다.
- 도구 방향은 아래(0°)부터 위(180°)까지 30°마다, 기울이는 방향은 8개다 (0은 바깥, 45°씩 왼쪽으로 돈다).
  도구 축 둘레 돌림은 0°와 90°다. 평행 그리퍼는 180°마다 모양이 같아서 J6를 180° 돌린 해도 같은 자세로 본다.
- 해는 IK가 주는 것을 모두 본다 (어깨 앞뒤, 팔꿈치 위아래, 손목 뒤집기).
- 호리봇의 관절 범위는 셋으로 나눠 본다:
  - 지금: params/arm.toml. 다만 J1은 서보 한 바퀴의 경계 여유(0024) 때문에 가운데에서 ±135°까지만 된다.
  - 270°: 관절마다 지금 범위의 가운데에서 ±135°. 서보 한 바퀴에 경계 여유 45°를 두고 쓸 수 있는 가장 넓은 범위다.
  - 한 바퀴: 범위는 보지 않고 부딪힘만 본다.
- 버팀 토크는 호리봇만 본다: 평행 그리퍼 + 짐 150 g, J2~J6가 정격의 0.8배 안 (0027의 하드웨어 기준).
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Mat, Robot, flange_pose, link_poses, rotation
from horilab_design.statics import FLANGE, PointMass, holding_torques
from horilab_design.studies import covers as cv
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur3e
from horilab_design.studies.common import RATED_MARGIN
from horilab_design.studies.packaging import DEG, TOOLS, Layout, Y, Z
from horilab_design.values import load_servos

TOOL = TOOLS["gripper"]
RADII = tuple(round(0.04 * i, 2) for i in range(13))  # 0~48 cm
HEIGHTS = tuple(round(0.02 + 0.04 * i, 2) for i in range(13))  # 2~50 cm
TILTS = (0, 30, 60, 90, 120, 150, 180)  # 도구가 연직 아래에서 기운 각 [°]
YAWS = (0, 90)  # 도구 축 둘레 돌림 [°]
AZIMUTH = math.pi / 2
WINDOW = 135 * DEG  # 서보 한 바퀴(360°)에서 경계 여유 45°를 양쪽에 두고 남는 반 범위 (0024)
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
SERVO_OF_JOINT = ("STS3215", "ST-3120", "STS3250", "STS3215", "STS3215", "STS3215")
REASONS = (
    ("못 닿음", "IK 해가 없음 (길이, 구조)"),
    (
        "관절 구조",
        "해는 있으나 서보가 자기 관절의 브래킷에 닿음 (설계, 0015의 관절 구조가 도는 범위)",
    ),
    ("부딪힘", "해는 있으나 다른 부품끼리나 책상에 닿음 (설계)"),
    ("한 바퀴", "부딪히지 않는 해가 서보가 허락하는 범위보다 더 돌아야 함 (하드웨어 후보)"),
    ("지금 범위", "서보가 허락하는 범위 안이지만 지금 관절 범위 밖 (설계)"),
    ("토크", "버팀 토크가 정격의 0.8배를 넘음 (하드웨어)"),
)


def scale() -> float:
    """호리봇 크기로 줄이는 비율: 위팔 + 아래팔 길이를 맞춘다."""
    layout = Layout()
    return (layout.upper + layout.fore) / (ur3e.A2 + ur3e.A3)


@dataclass(frozen=True)
class Target:
    r: float
    z: float
    tilt: int  # [°]
    direction: int  # 0~7. 기울기 0°와 180°는 0 하나만 쓴다
    yaw: int  # [°]
    T: Mat  # 플랜지 자세


def flange_target(
    r: float, z: float, tilt: float, direction: int, yaw: float, length: float
) -> Mat:
    """TCP를 (반지름 r, 높이 z)에 두고, 도구를 연직 아래에서 tilt만큼 direction 쪽으로 기울인 플랜지 자세.

    tilt = 0일 때는 common.tool_down, packaging.tool_pose와 같다. 회전축이 기울기와 상관없이 수평이라
    도구가 위(180°)를 향해도 정해진다.
    """
    p = np.array([r * math.cos(AZIMUTH), r * math.sin(AZIMUTH), z])
    phi = AZIMUTH + direction * 45 * DEG
    R = rotation(Z, AZIMUTH) @ rotation(Y, math.pi) @ rotation(Z, yaw)
    if tilt > 0:
        R = rotation((math.sin(phi), -math.cos(phi), 0.0), tilt) @ R
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p - R @ np.array([0.0, 0.0, length])
    return T


def targets(length: float = TOOL.length) -> list[Target]:
    out = []
    for r in RADII:
        for z in HEIGHTS:
            for tilt in TILTS:
                for direction in range(1 if tilt in (0, 180) else 8):
                    for yaw in YAWS:
                        T = flange_target(r, z, tilt * DEG, direction, yaw * DEG, length)
                        out.append(Target(r, z, tilt, direction, yaw, T))
    return out


@dataclass
class Arm:
    robot: Robot
    ik: AnalyticIK
    scene: pk.Scene


def same_joint(h: pk.Hit) -> bool:
    """서보가 자기 관절의 브래킷·뚜껑·집에 닿는 쌍 (예: "J4 서보 ↔ J4 브래킷"). 관절 구조가 도는 범위다."""
    ja, jb = h.a.split(" ")[0], h.b.split(" ")[0]
    return ja == jb and ja.startswith("J")


def free_solutions(
    arm: Arm, T: Mat, ignore: Callable[[pk.Hit], bool] | None = None
) -> tuple[list[np.ndarray], pk.Hit | None, int]:
    """부딪히지 않는 해들, 부딪히는 해만 있을 때 가장 덜 겹치는 해의 가장 깊은 쌍, 해의 수.

    ignore가 참인 쌍은 부딪힘으로 세지 않는다 (가정해 보는 실행에만 쓴다).
    """
    sols = arm.ik.solve(T)
    free: list[np.ndarray] = []
    blocked: list[pk.Hit] = []
    for q in sols:
        bad = pk.collisions(pk.hits(arm.scene, q))
        if ignore is not None:
            bad = [h for h in bad if not ignore(h)]
        if bad:
            blocked.append(min(bad, key=lambda h: h.dist))
        else:
            free.append(q)
    worst = max(blocked, key=lambda h: h.dist) if blocked and not free else None
    return free, worst, len(sols)


def fits(q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """관절마다 범위 안에 드는지. 2π 차이 나는 각과, J6는 180° 돌린 각(평행 그리퍼)도 본다."""
    ok = np.zeros(6, dtype=bool)
    for j in range(6):
        turns = (0.0, math.pi) if j == 5 else (0.0,)
        for extra in turns:
            for k in (-1, 0, 1):
                a = q[j] + extra + 2 * math.pi * k
                ok[j] |= bool(lower[j] - 1e-9 <= a <= upper[j] + 1e-9)
    return ok


@dataclass
class Hori(Arm):
    prims: list[pk.Prim]
    now: tuple[np.ndarray, np.ndarray]
    window: tuple[np.ndarray, np.ndarray]
    rated: np.ndarray
    masses: list[PointMass]

    def torque(self, q: np.ndarray) -> np.ndarray:
        """버팀 토크 / 정격 (관절마다)."""
        return np.abs(holding_torques(self.robot, q, self.masses)) / self.rated


def make_hori(
    robot: Robot,
    prims: list[pk.Prim],
    now: tuple[np.ndarray, np.ndarray],
    window: tuple[np.ndarray, np.ndarray],
) -> Hori:
    """형상(prims)과 관절 범위 둘(지금, 270°처럼 서보가 허락하는 가장 넓은 범위)로 견줄 호리봇을 만든다."""
    servos = load_servos()
    rated = np.array([servos[s].rated_torque for s in SERVO_OF_JOINT])
    masses = [PointMass(p.name, min(p.link, 6), tuple(p.pos), p.mass) for p in prims if p.mass > 0]
    masses.append(PointMass("짐", FLANGE, (0.0, 0.0, TOOL.length), TOOL.payload))
    return Hori(
        robot, AnalyticIK(robot), pk.build_scene(robot, prims), prims, now, window, rated, masses
    )


def hori_arm(layout: Layout | None = None, *, covers: bool = True) -> Hori:
    """지금 설계(UR식 외형, covers.py). J1은 서보 한 바퀴의 경계 여유 때문에 가운데 ±135°까지만 된다.

    covers가 False면 덮개 없이 서보가 드러난 ㄷ자 브래킷 형상(packaging.py)이다.
    """
    if covers:
        robot, prims, _, _ = cv.build_ur_arm(layout or Layout(), TOOL)
    else:
        robot, prims, _ = pk.build_arm(layout or Layout(), TOOL)
    mid = (robot.lower + robot.upper) / 2
    window = (mid - WINDOW, mid + WINDOW)
    now = (np.maximum(robot.lower, window[0]), np.minimum(robot.upper, window[1]))
    return make_hori(robot, prims, now, window)


def ur_arm(s: float) -> Arm:
    robot = ur3e.ur_robot(s)
    return Arm(robot, AnalyticIK(robot), ur3e.ur_scene(s, TOOL))


def _joints(ok: np.ndarray) -> str:
    return "·".join(f"J{j + 1}" for j in range(6) if not ok[j])


def hori_outcome(h: Hori, T: Mat, *, free_joints: bool = False) -> tuple[str, str]:
    """호리봇이 한 자세를 하는지: ("됨", "") 또는 (까닭, 자세한 것).

    free_joints는 관절 구조가 막지 않는다고 가정한다: 서보와 자기 브래킷이 닿는 것을 세지 않고, 지금 범위
    대신 270°를 쓴다. 관절 구조와 다른 설계 중 어느 쪽 탓이 큰지 가를 때 쓴다.
    """
    free, worst, n = free_solutions(h, T, same_joint if free_joints else None)
    if n == 0:
        return "못 닿음", ""
    if not free:
        assert worst is not None
        if same_joint(worst):
            return "관절 구조", worst.a.split(" ")[0]
        return "부딪힘", f"{worst.a} ↔ {worst.b}"
    if free_joints:
        h = replace(h, now=h.window)
    now = [q for q in free if fits(q, *h.now).all()]
    if now:
        ratios = [h.torque(q) for q in now]
        best = min(ratios, key=lambda t: float(t[1:].max()))
        if best[1:].max() <= RATED_MARGIN:
            return "됨", ""
        return "토크", f"J{int(np.argmax(best[1:])) + 2}"
    in_window = [q for q in free if fits(q, *h.window).all()]
    if in_window:
        best = min(in_window, key=lambda q: int((~fits(q, *h.now)).sum()))
        return "지금 범위", _joints(fits(best, *h.now))
    best = min(free, key=lambda q: int((~fits(q, *h.window)).sum()))
    return "한 바퀴", _joints(fits(best, *h.window))


def ur_outcome(u: Arm, T: Mat) -> tuple[str, str]:
    free, worst, n = free_solutions(u, T)
    if free:
        return "됨", ""
    if n == 0:
        return "못 닿음", ""
    assert worst is not None
    return "부딪힘", f"{worst.a} ↔ {worst.b}"


@dataclass
class Result:
    target: Target
    ur: tuple[str, str]
    hori: tuple[str, str]


@dataclass
class Comparison:
    scale: float
    free_joints: bool = False
    label: str = "지금 설계"
    results: list[Result] = field(default_factory=list)

    def count(self, test: Callable[[Result], bool]) -> int:
        return sum(1 for r in self.results if test(r))


def compare(
    layout: Layout | None = None,
    every: int = 1,
    *,
    free_joints: bool = False,
    hori: Hori | None = None,
    label: str = "지금 설계",
) -> Comparison:
    """every > 1이면 자세를 그만큼 건너뛰며 본다 (빠른 확인용). free_joints는 hori_outcome의 가정이다.

    hori를 주면 그 호리봇(다른 관절 구조 등)으로 견준다.
    """
    s = scale()
    u, h = ur_arm(s), hori or hori_arm(layout)
    out = Comparison(s, free_joints, label)
    for t in targets()[::every]:
        out.results.append(
            Result(t, ur_outcome(u, t.T), hori_outcome(h, t.T, free_joints=free_joints))
        )
    return out


def _pct(n: int, total: int) -> str:
    return f"{100 * n / total:.0f}%" if total else "-"


def report(c: Comparison) -> None:
    res = c.results
    n = len(res)
    print(
        f"\n## UR3e와 견주기 ({c.label}): UR3e ×{c.scale:.3f} (위팔 + 아래팔을 호리봇에 맞춤), "
        "평행 그리퍼 + 어댑터"
    )
    if c.free_joints:
        print(
            "- 가정: 호리봇의 관절 구조가 막지 않는다 (서보와 자기 브래킷이 닿는 것을 세지 않고, 관절 범위는 270°)"
        )
    print(
        f"- 자세 {n}개: TCP 자리 {len(RADII) * len(HEIGHTS)}곳(반지름 {RADII[0] * 100:.0f}~{RADII[-1] * 100:.0f} cm, "
        f"높이 {HEIGHTS[0] * 100:.0f}~{HEIGHTS[-1] * 100:.0f} cm, 4 cm 간격) × 도구 방향 "
        f"{2 + 8 * (len(TILTS) - 2)} × 도구 돌림 {len(YAWS)}"
    )
    ur_ok = {id(r) for r in res if r.ur[0] == "됨"}
    hori_ok = {id(r) for r in res if r.hori[0] == "됨"}
    both = len(ur_ok & hori_ok)
    print("\n| 갈래 | 자세 수 | 비율 |\n|---|---|---|")
    for name, k in (
        ("둘 다 됨", both),
        ("UR3e만 됨", len(ur_ok - hori_ok)),
        ("호리봇만 됨", len(hori_ok - ur_ok)),
        ("둘 다 안 됨", n - len(ur_ok | hori_ok)),
    ):
        print(f"| {name} | {k} | {_pct(k, n)} |")
    print(
        f"- UR3e가 되는 자세 {len(ur_ok)}개 중 호리봇이 되는 것은 {both}개({_pct(both, len(ur_ok))})다."
    )

    only_ur = [r for r in res if id(r) in ur_ok and id(r) not in hori_ok]
    print("\n### UR3e만 되는 자세에서 호리봇이 못 하는 까닭")
    print("| 까닭 | 뜻 | 자세 수 | 많이 나온 것 |\n|---|---|---|---|")
    for key, meaning in REASONS:
        group = [r for r in only_ur if r.hori[0] == key]
        top = Counter(r.hori[1] for r in group if r.hori[1]).most_common(4)
        detail = ", ".join(f"{d} {k}" for d, k in top) or "-"
        print(f"| {key} | {meaning} | {len(group)} | {detail} |")

    print("\n### 도구 방향별: UR3e가 되는 자세 중 호리봇이 되는 비율")
    print("| 기울기 | " + " | ".join(DIRECTIONS) + " |\n|---|" + "---|" * len(DIRECTIONS))
    for tilt in TILTS:
        cells = []
        for d in range(8):
            group = [r for r in res if r.target.tilt == tilt and r.target.direction == d]
            if not group:
                cells.append("")
                continue
            ok_ur = [r for r in group if id(r) in ur_ok]
            cells.append(f"{_pct(sum(id(r) in hori_ok for r in ok_ur), len(ok_ur))} ({len(ok_ur)})")
        print(f"| {tilt}° | " + " | ".join(cells) + " |")
    print("- 괄호는 UR3e가 되는 자세 수다. 기울기 0°(아래)와 180°(위)는 방향이 하나다.")

    print(
        "\n### 자리별 지도: UR3e가 되는 자세 중 호리봇이 되는 비율 (10%씩 0~9, #는 100%, .은 UR3e도 안 됨)"
    )
    print("```")
    print("높이 \\ 반지름 " + " ".join(f"{r * 100:>3.0f}" for r in RADII))
    for z in reversed(HEIGHTS):
        row = []
        for r_ in RADII:
            group = [r for r in res if r.target.r == r_ and r.target.z == z and id(r) in ur_ok]
            if not group:
                row.append(".")
                continue
            share = sum(id(r) in hori_ok for r in group) / len(group)
            row.append("#" if share == 1.0 else str(min(9, int(share * 10))))
        print(f"{z * 100:>12.0f} " + " ".join(f"{m:>3}" for m in row))
    print("```")

    only_hori = [r for r in res if id(r) in hori_ok and id(r) not in ur_ok]
    print(f"\n### 호리봇만 되는 자세 {len(only_hori)}개: UR3e가 못 하는 까닭")
    top = Counter(f"{r.ur[0]} {r.ur[1]}".strip() for r in only_hori).most_common(5)
    print("- " + (", ".join(f"{d} {k}" for d, k in top) or "-"))


def check() -> None:
    """원본 UR3e 모델이 공개 값과 맞는지 본다."""
    print("\n## 원본 UR3e 모델 확인")
    offset, signs = ur3e.correspondence()
    print(
        f"- 공개 모형의 관절 각 0 = 이 모델의 ({', '.join(f'{a / DEG:.0f}' for a in offset)})°, "
        f"축 방향 부호 {signs}"
    )
    robot = ur3e.ur_robot()
    rng = np.random.default_rng(0)
    o0 = link_poses(robot, np.array(offset))
    u0 = ur3e.urdf_link_poses(np.zeros(6))
    worst = 0.0
    for _ in range(200):
        qu = rng.uniform(-math.pi, math.pi, 6)
        ours, urs = link_poses(robot, ur3e.to_ours(qu)), ur3e.urdf_link_poses(qu)
        for i in range(6):
            K = np.linalg.inv(o0[i]) @ u0[i]
            worst = max(worst, float(np.abs(ours[i] @ K - urs[i]).max()))
    print(f"- 아무 관절 각 200개에서 링크 자리가 공개 모형과 다른 가장 큰 값: {worst:.1e}")
    T = flange_pose(robot, np.array(offset))
    print(
        f"- 공개 모형의 관절 각 0에서 플랜지 [mm]: ({', '.join(f'{v * 1000:.2f}' for v in T[:3, 3])}). "
        "DH로는 (-456.75, -223.15, 66.50)이다"
    )
    scene = ur3e.ur_scene()
    upright = ur3e.to_ours([0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0])
    print("- 팔을 위로 세운 자세에서 관절 하나씩 돌려 부딪히기 전까지 (도구 없이, 책상은 뺀다):")
    for j, (lo, hi, why_lo, why_hi) in enumerate(pk.joint_ranges(scene, upright)):
        print(
            f"  - J{j + 1}: {(lo - upright[j]) / DEG:.0f}° ~ {(hi - upright[j]) / DEG:.0f}° ({why_lo} / {why_hi})"
        )
    best = (0.0, 0.0, 0.0)
    for j4 in range(0, 360, 30):
        for j5 in range(0, 360, 90):
            q = ur3e.to_ours([0.0, -math.pi / 2, 0.0, j4 * DEG, j5 * DEG, 0.0])
            for step in (DEG, -DEG):
                fold = _fold(scene, q, step)
                if abs(fold) > abs(best[0]):
                    best = (fold, j4, j5)
    print(
        f"- 팔꿈치가 가장 많이 접히는 손목 자세: {best[0] / DEG:.0f}° (공개 모형의 J4 {best[1]}°, J5 {best[2]}°). "
        "공개 모형 joint_limits.yaml의 주석은 '어깨에 막혀 약 ±180°'다"
    )


def _fold(scene: pk.Scene, q0: np.ndarray, step: float) -> float:
    """팔꿈치(J3)를 step 방향으로 부딪히기 전까지 돌린 각 (책상은 뺀다)."""
    q = q0.copy()
    angle = 0.0
    while abs(angle) < math.pi:
        q[2] = q0[2] + angle + step
        if [h for h in pk.collisions(pk.hits(scene, q)) if "책상" not in (h.a, h.b)]:
            break
        angle += step
    return angle


def main() -> None:
    args = sys.argv[1:]
    if args == ["check"]:
        check()
    elif args == ["free"]:
        report(compare(free_joints=True))
    elif args == ["bare"]:
        report(compare(hori=hori_arm(covers=False), label="덮개 없는 ㄷ자 브래킷"))
    else:
        report(compare())


if __name__ == "__main__":
    main()
