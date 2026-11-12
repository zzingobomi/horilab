"""관절 구조 대안: UR식 관절 (0027의 결정 3). 개념 형상으로 UR3e와 견준다.

    uv run python -m horilab_design.studies.ur_joints            # UR3e와 견주기 (관절 범위 ±165°)
    uv run python -m horilab_design.studies.ur_joints 135        # 같은 견주기, 관절 범위 ±135°
    uv run python -m horilab_design.studies.ur_joints cable      # 관절마다 선 칸 예산을 넣고 견주기
    uv run python -m horilab_design.studies.ur_joints check      # 치수, 관절마다 도는 범위, 무게 (cable을 더 줄 수 있다)

처음 관절(ㄷ자 브래킷)은 자식 링크의 ㄷ자 브래킷이 서보의 두 혼을 양쪽에서 잡는다. 브래킷 다리가 서보 몸통에
닿기 전까지만 돌아서 관절마다 약 180~220°다. UR식 관절은 이렇다:
- 부모 링크의 원통(집)이 서보를 품는다. 서보 뒷면에 꽂는 커넥터와 선이 들어갈 자리를 집 안에 둔다.
- 자식 링크는 출력 쪽 뚜껑으로만 잇는다. 뚜껑은 출력 혼에 볼트로 묶고, 출력 면에 둔 베어링의 안쪽 링에
  앉는다. 출력 쪽 굽힘은 베어링이 받아서 서보 축이 혼자 받지 않는다 (design.md의 원칙). 베어링 축과 서보 축은
  J1처럼 서보 몸체 나사를 나중에 조여 맞춘다. 뒷면 혼은 쓰지 않는다.
- 부모와 자식이 관절 축 방향으로 비켜 있어서 서로 닿지 않고 돈다. 그래서 위팔, 아래팔, 손목이 축 방향으로
  번갈아 비켜 놓인다 (UR3e의 옆 오프셋, docs/references.md의 "기준 모델: UR3e").
- 형상은 원기둥과 상자로 만든 개념 형상이다. 원통과 관은 속이 찼다고 보고 부딪힘을 본다. 무게는 PLA 벽
  두께로 어림하고, 베어링 무게도 어림이다. 선, 커넥터 바깥, 볼트 머리는 아직 없다.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from functools import cached_property

import numpy as np

from horilab_design.robot import Robot
from horilab_design.studies import covers as cv
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur_compare as uc
from horilab_design.studies.covers import D405_PLATE_T
from horilab_design.studies.packaging import (
    CONNECTOR_DEPTH,
    DEG,
    GAP,
    MM,
    MOVING_GAP,
    PLA_DENSITY,
    TURNTABLE_T,
    TURNTABLE_TOP,
    WALL,
    Builder,
    Layout,
    Module,
    Prim,
    ServoShape,
    Tool,
    Y,
    Z,
)
from horilab_design.values import ServoModel, load_measured, load_servos, servo_mass

# 출력 면 베어링 (안지름, 바깥지름, 폭 [m], 무게 [kg]). 출력 혼(STS 지름 20, ST-3120 지름 30)을 감싸는 뚜껑의
# 허브가 안쪽 링에 들어간다. 무게는 카탈로그를 보기 전의 어림이다.
BEARINGS: dict[ServoModel, tuple[float, float, float, float]] = {
    "STS3215": (0.030, 0.042, 0.007, 0.019),  # 6806 (61806)
    "STS3250": (0.030, 0.042, 0.007, 0.019),
    "ST-3120": (0.040, 0.052, 0.007, 0.026),  # 6808 (61808)
}
SEAT_WALL = 3.5 * MM  # 베어링 바깥 링을 잡는 집 벽
SHOULDER_T = 1.5 * MM  # 베어링 뒤를 받치는 턱 (서보 앞면과 0.5 mm 띄움)
CAP_T = 4.0 * MM  # 뚜껑 판 두께
INNER_EXTRA = 1.2  # 집 안의 서보 자리, 베어링 자리, 나사 기둥 몫으로 벽 무게에 곱하는 값 (어림)
LINK_NAMES = ("J1 몸통", "위팔", "아래팔", "손목 1", "손목 2", "손목 3")


@dataclass(frozen=True)
class Stack:
    """관절 하나의 축 방향 쌓임. 서보 좌표계 y(출력 쪽이 +) [m]."""

    radius: float  # 원통 바깥 반지름
    back: float  # 집 뒷면 바깥 (뒷면 커넥터와 선 자리, 벽 포함)
    bearing: tuple[float, float]  # 베어링 y 범위
    front: float  # 집 앞면 (베어링 앞면, 선 칸이 있으면 그 앞)
    cap: tuple[float, float]  # 자식 뚜껑


def stack(shape: ServoShape, cable_r: float = 0.0, cable_a: float = 0.0) -> Stack:
    """서보 외형과 베어링에서 관절의 쌓임을 정한다.

    서보 뒷면(축 둘레)에서 커넥터와 선이 CONNECTOR_DEPTH만큼 솟고 1 mm를 더 띄운다. 베어링은 서보 앞면에서
    턱만큼 띄우고, 뚜껑은 베어링 앞면과 출력 혼 윗면 중 바깥 것에서 0.5 mm 띄운다.
    cable_r, cable_a는 관절을 지나는 선이 감기고 풀릴 칸의 예산이다: 원통 반지름을 cable_r만큼 키우고,
    베어링과 뚜껑 사이를 cable_a만큼 띄운다.
    """
    back_face = min(z[2] for z in shape.zones if z[0] <= 0.0 <= z[1])
    front_face = max(z[3] for z in shape.zones)
    _, od, width, _ = BEARINGS[shape.model]
    b0 = front_face + 0.5 * MM + SHOULDER_T
    b1 = b0 + width
    cap0 = max(b1, shape.horn_out[1]) + 0.5 * MM + cable_a
    return Stack(
        radius=od / 2 + SEAT_WALL + cable_r,
        back=back_face - (CONNECTOR_DEPTH + 1.0) * MM - WALL,
        bearing=(b0, b1),
        front=b1 + cable_a,
        cap=(cap0, cap0 + CAP_T),
    )


def _body_mid(shape: ServoShape) -> float:
    """서보 몸통 가운데의 y (축 둘레 구간)."""
    back_face = min(z[2] for z in shape.zones if z[0] <= 0.0 <= z[1])
    return (back_face + max(z[3] for z in shape.zones)) / 2


@dataclass(frozen=True)
class Concept:
    """UR식 관절 팔의 개념 치수 [m]. 링크 길이는 지금 골격(0017)과 같게 두고, 옆 오프셋은 관절 쌓임에서 정한다."""

    shoulder: float = 0.16
    upper: float = 0.22
    fore: float = 0.18
    upper_r: float = 0.020  # 위팔 관 바깥 반지름 (0019)
    fore_r: float = 0.018
    column_r: float = 0.044  # J1 몸통 반지름 (0019)
    camera_back: float = 0.005  # D405 렌즈 면을 플랜지 면보다 뒤로 물리는 거리 (0017)
    cable_r: float = 0.0  # 관절마다 선 칸 예산: 원통 반지름을 키우는 양 (stack)
    cable_a: float = 0.0  # 관절마다 선 칸 예산: 베어링과 뚜껑 사이를 띄우는 양 (stack)
    ranges_deg: tuple[tuple[float, float], ...] = (
        (0, 165),
        (0, 165),
        (0, 165),
        (180, 165),
        (0, 165),
        (0, 165),
    )  # 관절마다 (가운데, 반 범위) [°]. 기계 멈춤이 이 밖에 있다고 본다

    @cached_property
    def shapes(self) -> dict[ServoModel, ServoShape]:
        return pk.shapes()

    def st(self, model: ServoModel) -> Stack:
        return stack(self.shapes[model], self.cable_r, self.cable_a)

    # 옆(y) 자리: 관절 축 방향. J1 축을 지나는 면이 0이다.
    @property
    def c2(self) -> float:
        """J2 서보 좌표계 원점의 y. 서보 몸통이 J1 몸통 가운데에 오게 둔다."""
        return -_body_mid(self.shapes["ST-3120"])

    @property
    def s2(self) -> float:
        """위팔 관 가운데의 y: J2 뚜껑 바깥 면에 붙는다."""
        return self.c2 + self.st("ST-3120").cap[1] + self.upper_r

    @property
    def c3(self) -> float:
        """J3 서보 원점의 y. 출력이 -y를 보고, 서보 몸통 가운데가 위팔 관 가운데에 온다."""
        return self.s2 + _body_mid(self.shapes["STS3250"])

    @property
    def s3(self) -> float:
        """아래팔 관 가운데의 y: J3 뚜껑 바깥 면에 붙는다."""
        return self.c3 - self.st("STS3250").cap[1] - self.fore_r

    @property
    def c4(self) -> float:
        """J4 서보 원점의 y. 출력이 +y를 보고, 서보 몸통 가운데가 아래팔 관 가운데에 온다."""
        return self.s3 - _body_mid(self.shapes["STS3215"])

    @property
    def d4(self) -> float:
        """J5 축의 y (UR의 d4): J5 원통이 J4 뚜껑과 움직이는 틈을 둔다."""
        s4 = self.st("STS3215")
        return self.c4 + s4.cap[1] + MOVING_GAP + s4.radius

    @property
    def d5(self) -> float:
        """J5 점 → J6 축 (UR의 d5): J6 서보의 먼 쪽(아래)과 집 벽이 J5 뚜껑과 움직이는 틈을 둔다."""
        s = self.st("STS3215")
        j6 = self.shapes["STS3215"]
        return max(s.cap[1] + MOVING_GAP + j6.far + GAP + WALL, s.cap[1] + MOVING_GAP + s.radius)

    @property
    def d6(self) -> float:
        """J6 점 → 플랜지 면 (UR의 d6): J6 뚜껑 바깥 면."""
        return self.st("STS3215").cap[1]

    def layout(self) -> Layout:
        return Layout(
            shoulder=self.shoulder,
            upper=self.upper,
            fore=self.fore,
            d45=self.d4,
            d56=self.d5,
            d6f=self.d6,
        )


def robot_of(c: Concept) -> Robot:
    """링크 길이와 옆 오프셋은 Concept에서, 관절 범위는 ranges_deg에서."""
    base = pk.arm(c.layout())
    joints = tuple(
        replace(j, lower=(mid - half) * DEG, upper=(mid + half) * DEG)
        for j, (mid, half) in zip(base.joints, c.ranges_deg, strict=True)
    )
    return Robot("horibot UR식 관절", joints, base.T_last_flange)


def _shell_mass(radius: float, length: float, ends: int) -> float:
    """PLA 벽(WALL)으로 만든 원통 겉면의 무게: 옆면과 끝면 ends개."""
    area = 2 * math.pi * radius * length + ends * math.pi * radius**2
    return PLA_DENSITY * WALL * area


def build(c: Concept, tool: Tool) -> tuple[Robot, list[Prim]]:
    """UR식 관절 팔의 개념 형상: 베이스, J1 몸통, 관절 원통(집 + 뚜껑), 관 링크, 손목, D405, 도구."""
    robot = robot_of(c)
    servos, measured = load_servos(), load_measured()
    sh = c.shapes
    b = Builder()

    def joint(
        j: int,
        model: ServoModel,
        origin: tuple[float, float, float],
        axis: tuple[float, float, float],
        near: tuple[float, float, float],
        out: int,
        mid: float,
    ) -> Module:
        """관절 j의 서보, 집(부모), 먼 쪽 집, 베어링, 뚜껑(자식, 출력 혼 무게 포함)."""
        shape = sh[model]
        m = Module(j, shape, origin, axis, near, out, mid)
        s = c.st(model)
        f = m.frame()
        volumes = [(x1 - x0) * (y1 - y0) * (z1 - z0) for x0, x1, y0, y1, z0, z1 in shape.zones]
        mass = servo_mass(model, servos, measured)
        for k, (x0, x1, y0, y1, z0, z1) in enumerate(shape.zones):
            b.box(
                j - 1,
                f"J{j}/servo_{k}",
                (x0, y0, z0),
                (x1, y1, z1),
                material="servo",
                mass=mass * volumes[k] / sum(volumes),
                frame=f,
                group=f"J{j} 서보",
            )
        length = s.front - s.back
        b.cyl(
            j - 1,
            f"J{j}/house",
            (0.0, (s.back + s.front) / 2, 0.0),
            Y,
            s.radius,
            length,
            mass=_shell_mass(s.radius, length, 1) * INNER_EXTRA,
            frame=f,
            group=f"J{j} 집",
        )
        # 원통 밖으로 나오는 서보 먼 쪽을 감싸는 집. 서보 앞뒤로 GAP과 벽을 둔다.
        hw = shape.half_width + GAP + WALL
        y0 = min(z[2] for z in shape.zones) - GAP - WALL
        y1 = max(z[3] for z in shape.zones) + GAP + WALL
        x0 = -(shape.far + GAP + WALL)
        x1 = -math.sqrt(max(s.radius**2 - hw**2, 0.0))
        if x1 > x0:
            area = 2 * ((x1 - x0) * (y1 - y0) + (x1 - x0) * 2 * hw) + (y1 - y0) * 2 * hw
            b.box(
                j - 1,
                f"J{j}/far_house",
                (x0, y0, -hw),
                (x1, y1, hw),
                mass=PLA_DENSITY * WALL * area,
                frame=f,
                group=f"J{j} 집",
            )
        bore, od, width, bearing_mass = BEARINGS[model]
        b.cyl(
            j - 1,
            f"J{j}/bearing",
            (0.0, (s.bearing[0] + s.bearing[1]) / 2, 0.0),
            Y,
            od / 2,
            width,
            material="metal",
            mass=bearing_mass,
            frame=f,
            group=f"J{j} 집",
        )
        cf = m.child_frame()
        cap_mass = PLA_DENSITY * (
            math.pi * s.radius**2 * CAP_T + math.pi * ((bore / 2) ** 2 - shape.horn_r**2) * width
        )
        b.cyl(
            j,
            f"J{j}/cap",
            (0.0, (s.cap[0] + s.cap[1]) / 2, 0.0),
            Y,
            s.radius,
            s.cap[1] - s.cap[0],
            mass=cap_mass + shape.horn_mass / 2,
            frame=cf,
            group=f"J{j} 뚜껑",
        )
        return m

    def tube(
        link: int,
        name: str,
        center: tuple[float, float, float],
        axis: tuple[float, float, float],
        radius: float,
        length: float,
        group: str,
        ends: int = 0,
    ) -> None:
        b.cyl(
            link,
            name,
            center,
            axis,
            radius,
            length,
            mass=_shell_mass(radius, length, ends),
            group=group,
        )

    st2, st3, st4 = c.st("ST-3120"), c.st("STS3250"), c.st("STS3215")
    h, a2, a3, d4, d5, d6 = c.shoulder, c.upper, c.fore, c.d4, c.d5, c.d6

    # 베이스(0), 돌림판과 J1 몸통(1). J2 서보의 먼 쪽(아래)은 몸통 안이다.
    base_h = TURNTABLE_TOP - TURNTABLE_T - 0.5 * MM
    b.cyl(0, "base", (0.0, 0.0, base_h / 2), Z, 0.046, base_h, mass=0.0, group="베이스")
    b.cyl(
        1,
        "turntable",
        (0.0, 0.0, TURNTABLE_TOP - TURNTABLE_T / 2),
        Z,
        0.044,
        TURNTABLE_T,
        group="돌림판",
    )
    col_top = h - st2.radius - MOVING_GAP
    tube(
        1,
        "column",
        (0.0, 0.0, (TURNTABLE_TOP + col_top) / 2),
        Z,
        c.column_r,
        col_top - TURNTABLE_TOP,
        "J1 몸통",
        ends=1,
    )

    # J2: 서보는 J1 몸통 위에서 출력이 +y를 본다. 위팔(2)은 뚜껑 바깥에 붙어 옆으로 비켜 있다.
    joint(2, "ST-3120", (0.0, 0.0, h), Y, Z, 1, c.c2)
    y_lo, y_hi = c.c2 + st2.cap[1], c.s2 + c.upper_r
    tube(2, "upper_root", (0.0, (y_lo + y_hi) / 2, 0.0), Y, st2.radius, y_hi - y_lo, "위팔", 1)
    tube(2, "upper_tube", (0.0, c.s2, a2 / 2), Z, c.upper_r, a2, "위팔")

    # J3: 서보는 위팔 위 끝에서 출력이 -y를 본다. 아래팔(3)은 뚜껑 바깥(-y)에 붙는다.
    joint(3, "STS3250", (0.0, 0.0, a2), Y, Z, -1, c.c3)
    y_hi, y_lo = c.c3 - st3.cap[1], c.s3 - c.fore_r
    tube(3, "fore_root", (0.0, (y_lo + y_hi) / 2, 0.0), Y, st3.radius, y_hi - y_lo, "아래팔", 1)
    tube(3, "fore_tube", (0.0, c.s3, a3 / 2), Z, c.fore_r, a3, "아래팔")

    # J4: 서보는 아래팔 위 끝에서 출력이 +y를 본다. 손목 1(4)은 뚜껑에서 J5 원통까지 가는 목이다.
    joint(4, "STS3215", (0.0, 0.0, a3), Y, Z, 1, c.c4)
    y_lo = c.c4 + st4.cap[1]
    tube(4, "neck4", (0.0, (y_lo + d4) / 2, 0.0), Y, st4.radius, d4 - y_lo, "손목 1")

    # J5: 서보는 손목 1 안에서 출력이 위(+z)를 본다. 먼 쪽은 J4 반대쪽(+y)으로 뻗는다. J4 쪽으로 뻗으면
    # J4 원통과 겹쳐서 d4가 약 80 mm로 늘어난다.
    joint(5, "STS3215", (0.0, d4, 0.0), Z, (0.0, -1.0, 0.0), 1, 0.0)
    z_lo = st4.cap[1]
    tube(5, "column5", (0.0, 0.0, (z_lo + d5) / 2), Z, st4.radius, d5 - z_lo, "손목 2")

    # J6: 서보는 손목 2 위에서 출력이 +y(플랜지 쪽)를 보고, 먼 쪽은 아래(-z)로 손목 2 기둥 안에 든다.
    joint(6, "STS3215", (0.0, 0.0, d5), Y, Z, 1, 0.0)

    # D405: J6 원통 옆(+x)에 움직이는 틈을 두고 단다. 렌즈 면은 플랜지 면보다 camera_back 뒤다 (0017, 0019).
    x_cam = st4.radius + MOVING_GAP
    lens_y = d6 - c.camera_back
    back = lens_y - 0.023
    b.box(
        5,
        "d405_plate",
        (st4.radius - WALL, back - D405_PLATE_T, d5 - 0.018),
        (x_cam + 0.039, back, d5 + 0.018),
        group="D405 받침",
    )
    b.box(
        5,
        "d405",
        (x_cam, back, d5 - 0.021),
        (x_cam + 0.042, lens_y, d5 + 0.021),
        material="camera",
        mass=pk.D405_MASS,
        group="D405",
    )

    # 손목 3(6): J6 뚜껑이 플랜지다. 그 앞에 교체 어댑터와 도구.
    pk.add_tool(b, 6, d6, tool)
    return robot, b.prims


def link_masses(prims: list[Prim]) -> list[float]:
    """링크마다 무게 [kg] (0 베이스 ~ 6 손목 3, 도구 포함)."""
    out = [0.0] * 7
    for p in prims:
        out[min(p.link, 6)] += p.mass
    return out


def hori(c: Concept, half_deg: float | None = None) -> uc.Hori:
    """견주기에 쓰는 호리봇. half_deg를 주면 관절 범위를 가운데 ±half_deg로 바꾼다."""
    if half_deg is not None:
        c = replace(c, ranges_deg=tuple((mid, half_deg) for mid, _ in c.ranges_deg))
    robot, prims = build(c, uc.TOOL)
    ranges = (robot.lower, robot.upper)
    return uc.make_hori(robot, prims, ranges, ranges)


def check(c: Concept) -> None:
    """치수, 관절마다 도는 범위, 무게, 버팀 토크."""
    print("\n## UR식 관절 개념 치수")
    for model in ("ST-3120", "STS3250", "STS3215"):
        s = c.st(model)
        print(
            f"- {model}: 원통 지름 {2 * s.radius / MM:.0f} mm, 축 방향 집 {(s.front - s.back) / MM:.1f} mm "
            f"+ 뚜껑 {(s.cap[1] - s.cap[0]) / MM:.0f} mm (서보 좌표 {s.back / MM:.1f} ~ {s.cap[1] / MM:.1f} mm), "
            f"베어링 {BEARINGS[model][0] / MM:.0f}×{BEARINGS[model][1] / MM:.0f}×{BEARINGS[model][2] / MM:.0f}"
        )
    print(
        f"- 옆 자리 [mm]: 위팔 관 {c.s2 / MM:.1f}, 아래팔 관 {c.s3 / MM:.1f}, J5 축 d4 {c.d4 / MM:.1f}. "
        f"d5 {c.d5 / MM:.1f}, d6 {c.d6 / MM:.1f} (지금 골격 63 · 63 · 50)"
    )
    robot, prims = build(c, uc.TOOL)
    scene = pk.build_scene(robot, prims)
    q0 = np.array([mid * DEG for mid, _ in c.ranges_deg])
    q0[1] = 0.0  # 팔을 세운 자세
    bad = pk.collisions(pk.hits(scene, q0))
    print(f"- 팔을 세운 자세의 부딪힘: {', '.join(f'{h.a} ↔ {h.b}' for h in bad) or '없음'}")
    print("- 팔을 세운 자세에서 관절 하나씩 돌려 부딪히기 전까지 (책상은 뺀다):")
    for j, (lo, hi, why_lo, why_hi) in enumerate(pk.joint_ranges(scene, q0)):
        print(
            f"  - J{j + 1}: {(lo - q0[j]) / DEG:.0f}° ~ {(hi - q0[j]) / DEG:.0f}° ({why_lo} / {why_hi})"
        )
    masses = link_masses(prims)
    print(
        "- 링크 무게 [g]: "
        + ", ".join(
            f"{name} {m * 1000:.0f}" for name, m in zip(LINK_NAMES, masses[1:], strict=True)
        )
        + f". J2가 드는 무게(링크 2~6, 도구 포함) {sum(masses[2:]) * 1000:.0f} g"
    )
    _, now_prims, _, _ = cv.build_ur_arm(Layout(), uc.TOOL)
    now = link_masses(now_prims)
    print(f"  - 지금 설계(UR식 외형, 0019)의 J2가 드는 무게: {sum(now[2:]) * 1000:.0f} g")


CABLE_BUDGET = (6.0 * MM, 6.0 * MM)  # 선 칸 예산 (반지름, 축 방향). 태엽처럼 감기는 칸의 어림


def main() -> None:
    """인자: check(치수와 범위), cable(선 칸 예산을 넣음), 숫자(관절 반 범위 [°]). 섞어 쓸 수 있다."""
    args = sys.argv[1:]
    c = Concept()
    if "cable" in args:
        c = replace(c, cable_r=CABLE_BUDGET[0], cable_a=CABLE_BUDGET[1])
    if "check" in args:
        check(c)
        return
    numbers = [float(a) for a in args if a.replace(".", "", 1).isdigit()]
    half = numbers[0] if numbers else None
    h = hori(c, half)
    budget = ", 선 칸 예산" if "cable" in args else ""
    title = f"UR식 관절 (관절 범위 가운데 ±{half or 165:.0f}°{budget})"
    uc.report(uc.compare(hori=h, label=title))


if __name__ == "__main__":
    main()
