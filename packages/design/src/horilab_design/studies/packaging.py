"""골격 배치 검토: 실제 서보 외형으로 관절을 배치하고, 작업 자세에서 부딪히는지와 관절 범위, 토크, 시야를 본다.

    uv run python -m horilab_design.studies.packaging

- 서보 외형은 Feetech 공식 도면 값과 실측이다 (docs/hardware.md의 기구 치수, data/measured.toml).
  STS3215와 STS3250은 도면이 같다: 몸통 32, 출력축 둘레는 혼이 앉는 오목한 자리, 먼 쪽은 뒷면 판까지 35 mm.
- 관절 구조는 처음 관절 구조(ㄷ자 브래킷)다. UR식 관절(0028)은 studies/ur_joints.py에 있다. 자식 링크의 ㄷ자 브래킷이 출력 혼과 뒷면 혼을 잡고,
  다리(브래킷의 두 팔을 잇는 판)는 서보의 가까운 끝 너머에 있다. 부모 링크는 서보 몸체의 먼 쪽만 감싼다.
  브래킷이 도는 동안 쓸고 지나가는 자리에는 부모 링크의 형상을 두지 않는다.
- 모양은 상자와 원기둥으로 만든 개념 형상이다. 부딪힘(MuJoCo 충돌 검사), 관절 범위, 무게, 시야를 어림한다.
  부품을 설계하면 같은 검사를 실제 형상으로 다시 한다 (docs/design.md의 "출력 전 확인").
- 도구는 docs/design.md의 도구 순서 세 가지(포인터, 평행 그리퍼, 흡착)와 교체 어댑터(두께 10 mm)다.
  박스 쌓기는 평행 그리퍼가 박스 윗면 구멍 안에서 잡는다 (0014).
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from functools import cache
from typing import Literal

import mujoco
import numpy as np

from horilab_design.ik import AnalyticIK
from horilab_design.robot import Mat, Robot, Vec3, rotation
from horilab_design.statics import FLANGE, PointMass, holding_torques
from horilab_design.studies.common import working_solutions
from horilab_design.studies.skeleton import targets as stacking_targets
from horilab_design.studies.skeleton import with_lengths
from horilab_design.values import (
    ServoBody,
    ServoModel,
    arm_robot,
    load_arm,
    load_measured,
    load_servos,
    servo_mass,
)

MM = 1e-3
DEG = math.pi / 180
X: Vec3 = (1.0, 0.0, 0.0)
Y: Vec3 = (0.0, 1.0, 0.0)
Z: Vec3 = (0.0, 0.0, 1.0)

WALL = 2.6 * MM  # 0.20mm Strength 프리셋의 벽 6겹 (docs/manufacturing.md)
GAP = 0.3 * MM  # 출력 부품과 서보 사이 여유 (공차 시험편으로 정하기 전의 어림)
ARM_T = 4.0 * MM  # 브래킷 팔 두께
BRIDGE_T = 4.0 * MM  # 브래킷 다리 두께
BRIDGE_GAP = 1.5 * MM  # 다리와 서보 가까운 끝 모서리 사이
FLANGE_T = 6.0 * MM  # 로봇 쪽 플랜지 판 두께
ADAPTER_T = 10.0 * MM  # 그리퍼 교체 어댑터 두께 (docs/design.md: 10 mm 이내)
PLA_DENSITY = 1260.0  # [kg/m³] 뱀부랩 PLA Basic 필라멘트 프리셋
D405_MASS = 0.060  # [kg] 홈페이지 값 (사양서는 58 g, docs/hardware.md)
CONTACT_MARGIN = 2.0 * MM  # 이 거리 안이면 "빠듯함"으로 본다
PENETRATION = 0.2 * MM  # 이보다 깊이 겹치면 부딪힘으로 본다
MOVING_GAP = 2.0 * MM  # 서로 움직이는 다른 링크 형상 사이에 두는 틈
# 돌림판 윗면 높이. 베이스 안에 J1 서보와 J1 베어링이 들어간다 (0028). 아래부터 바닥판 4, J1 서보와 혼 36,
# 안쪽 링 받침 4, 6806 두 개(30 mm 띄움, 바깥 면 사이 37), 바깥 링 누름판 2, 틈 0.5, 돌림판 9.5 [mm].
# RU42(두께 12)일 때는 62 mm였다.
TURNTABLE_TOP = 0.093
TURNTABLE_T = 9.5 * MM


@dataclass(frozen=True)
class ServoShape:
    """서보 외형 (서보 좌표계: x는 축에서 가까운 끝 쪽, y는 출력 축 방향, z는 x × y, 원점은 축 위).

    몸통은 상자 여러 개다: (x0, x1, y0, y1, z0, z1) [m].
    """

    model: ServoModel
    near: float  # 축 → 가까운 끝
    far: float  # 축 → 먼 끝
    half_width: float
    zones: tuple[tuple[float, float, float, float, float, float], ...]
    horn_r: float
    horn_out: tuple[float, float]  # 출력 혼의 y 범위
    horn_back: tuple[
        float, float
    ]  # 뒷면 혼(또는 베어링)의 y 범위. 돌기 끝과 나사 머리는 브래킷 팔의 홈으로 받는다
    horn_mass: float  # 혼 두 개 [kg]

    @property
    def far_half(self) -> tuple[float, float]:
        """먼 쪽 구간(집이 감싸는 곳)의 y 범위."""
        far_zones = [z for z in self.zones if z[0] <= -self.far + 1e-9]
        return min(z[2] for z in far_zones), max(z[3] for z in far_zones)


Zone = tuple[float, float, float, float, float, float]
Span = tuple[float, float]


def _zone(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Zone:
    """mm 값을 m로: 몸통 상자 하나 (x0, x1, y0, y1, z0, z1)."""
    return (x0 * MM, x1 * MM, y0 * MM, y1 * MM, z0 * MM, z1 * MM)


def _span(a: float, b: float) -> Span:
    return (a * MM, b * MM)


# STS3215·STS3250 외형 [mm]: Feetech 도면(docs/hardware.md의 기구 치수)과 실측(data/measured.toml, 2026-11-08).
# 서보 좌표계의 y 원점은 뒷면(축 가까운 쪽)에서 14.5 mm 앞이다. 실측 전 외형 모델과 뒷면 혼, 먼 쪽 자리가 같다.
# - 몸통 두께는 32다 (뒷면 -14.5 ~ 앞면 +17.5). 앞면은 끝까지 평평하다.
# - 출력축 둘레(축에서 -11 ~ 가까운 끝)는 혼이 앉는 둥근 오목한 자리다 (도면의 "축 근처 29"). 바닥은 혼 아래
#   HORN_GAP에 두고, 양옆(|z| ≥ STS_WALL_Z)은 앞면 높이까지 찬 것으로 둔다.
# - 먼 쪽(축에서 -15.9 ~ 먼 끝)은 뒷면에 볼록한 판이 있어 두께가 실측 far_thickness다 (도면 35).
# - 출력 혼(두께 4.5) 윗면은 뒷면에서 실측 horn_to_back이다. 뒷면 혼(두께 3.1)은 뒷면 보스에 끼워 뒷면에서
#   HORN_GAP 띄운다.
STS_BACK = -14.5
STS_BODY = 32.0
STS_NEAR, STS_FAR = 10.11, 35.11
STS_HALF_WIDTH = 12.36
STS_RECESS_X = -11.0  # 혼 자리가 시작하는 x (가까운 끝까지)
STS_WALL_Z = 10.0  # 혼 자리 옆이 앞면 높이인 |z|
STS_PAD_X = -15.9  # 뒷면 판이 시작하는 x (먼 끝까지)
HORN_T = 4.5
BACK_HORN_T = 3.1
HORN_GAP = 0.3
# 실측이 없을 때 쓰는 도면 값: 혼 자리 바닥 29 + 틈 0.35 + 혼 4.5, 먼 쪽 35
DRAWING_BODY = ServoBody(far_thickness_mm=35.0, horn_to_back_mm=33.85)
# 혼 두 개의 무게를 저울로 재기 전의 어림 [kg] (재면 data/measured.toml의 servo_horn_mass_g)
HORN_MASS_GUESS: dict[ServoModel, float] = {"STS3215": 0.008, "STS3250": 0.008, "ST-3120": 0.025}
# 뒷면 커넥터 두 개가 꽂히는 자리 (사진으로 어림): 축에서 x 범위, 뒷면에서 플러그와 선이 솟는 깊이 [mm]
CONNECTOR_X = (-16.0, -9.0)
CONNECTOR_DEPTH = 6.0


def sts_shape(model: ServoModel, body: ServoBody, horn_mass: float) -> ServoShape:
    front = STS_BACK + STS_BODY
    horn_top = STS_BACK + body.horn_to_back_mm
    floor = horn_top - HORN_T - HORN_GAP
    hw = STS_HALF_WIDTH
    return ServoShape(
        model,
        near=STS_NEAR * MM,
        far=STS_FAR * MM,
        half_width=hw * MM,
        zones=(
            _zone(-STS_FAR, STS_PAD_X, front - body.far_thickness_mm, front, -hw, hw),
            _zone(STS_PAD_X, STS_RECESS_X, STS_BACK, front, -hw, hw),
            _zone(STS_RECESS_X, STS_NEAR, STS_BACK, floor, -hw, hw),
            _zone(STS_RECESS_X, STS_NEAR, floor, front, STS_WALL_Z, hw),
            _zone(STS_RECESS_X, STS_NEAR, floor, front, -hw, -STS_WALL_Z),
        ),
        horn_r=9.975 * MM,
        horn_out=_span(horn_top - HORN_T, horn_top),
        horn_back=_span(STS_BACK - HORN_GAP - BACK_HORN_T, STS_BACK - HORN_GAP),
        horn_mass=horn_mass,
    )


def horn_mass(model: ServoModel) -> float:
    """혼 두 개의 무게 [kg]. 저울로 잰 값이 있으면 그 값을, 없으면 어림을 쓴다."""
    grams = load_measured().servo_horn_mass_g.get(model)
    return HORN_MASS_GUESS[model] if grams is None else grams * 1e-3


@cache
def shapes() -> dict[ServoModel, ServoShape]:
    measured = load_measured().servo_body
    return {
        "STS3215": sts_shape(
            "STS3215", measured.get("STS3215", DRAWING_BODY), horn_mass("STS3215")
        ),
        "STS3250": sts_shape(
            "STS3250", measured.get("STS3250", DRAWING_BODY), horn_mass("STS3250")
        ),
        # 65 × 30 × 48, 축에서 가까운 끝 15.9. 혼(FT110M)은 지름 29.9, 두께 9로 스플라인(7.4 나옴) 위에 끼운다.
        # 뒷면은 돌기(4.4 나옴)를 혼이나 MR106ZZ로 받친다 (처음 관절 구조). 혼 무게는 사양에 없어 재기 전에는 어림이다.
        "ST-3120": ServoShape(
            "ST-3120",
            near=15.9 * MM,
            far=49.1 * MM,
            half_width=15.0 * MM,
            zones=(_zone(-49.1, 15.9, -24.0, 24.0, -15.0, 15.0),),
            horn_r=14.95 * MM,
            horn_out=_span(24.9, 33.9),
            horn_back=_span(-28.4, -24.4),
            horn_mass=horn_mass("ST-3120"),
        ),
    }


Material = Literal["pla", "servo", "metal", "camera", "tool"]
Axis = Literal["x", "y", "z"]


@dataclass(frozen=True)
class Prim:
    """링크 좌표계에 둔 상자, 원기둥, 고리(속이 빈 원기둥), 볼록 껍질 하나. 원기둥과 고리의 축은 형상 좌표계의 z다."""

    name: str
    link: int  # 0 베이스, i 관절 i가 움직이는 링크, 7 도구(링크 6에 붙은 따로 된 몸체)
    shape: Literal["box", "cyl", "ring", "hull"]
    pos: np.ndarray
    rot: np.ndarray
    # 상자: 반 크기 / 원기둥: (반지름, 반 길이, 0) / 고리: (바깥 반지름, 반 길이, 안쪽 반지름) / 껍질: 쓰지 않음
    size: tuple[float, float, float]
    material: Material
    mass: float
    group: str  # 보고용 묶음 (예: "J4 서보")
    verts: np.ndarray | None = None  # 볼록 껍질을 이루는 점들 (n × 3, 형상 좌표계)


def _box_mass(half: np.ndarray, hollow: bool) -> float:
    full = float(np.prod(2 * half))
    if not hollow or np.min(2 * half) <= 2 * WALL:
        return PLA_DENSITY * full
    inner = float(np.prod(np.maximum(2 * half - 2 * WALL, 0.0)))
    return PLA_DENSITY * (full - inner)


@dataclass
class Builder:
    """링크마다 형상을 모은다. 좌표는 링크 좌표계 (영점에서 베이스와 같은 방향, 원점은 관절 점)."""

    prims: list[Prim] = field(default_factory=list)

    def box(
        self,
        link: int,
        name: str,
        lo: Vec3,
        hi: Vec3,
        *,
        material: Material = "pla",
        hollow: bool = False,
        mass: float | None = None,
        group: str = "",
        frame: tuple[np.ndarray, np.ndarray] | None = None,
    ) -> None:
        """frame(원점, 회전)으로 주어진 좌표계에서 lo~hi 상자. frame이 없으면 링크 좌표계다."""
        lo_, hi_ = np.asarray(lo, float), np.asarray(hi, float)
        center, half = (lo_ + hi_) / 2, (hi_ - lo_) / 2
        origin, R = frame if frame is not None else (np.zeros(3), np.eye(3))
        m = mass if mass is not None else (_box_mass(half, hollow) if material == "pla" else 0.0)
        self.prims.append(
            Prim(name, link, "box", origin + R @ center, R, tuple(half), material, m, group or name)
        )

    def cyl(
        self,
        link: int,
        name: str,
        center: Vec3,
        axis: Vec3,
        radius: float,
        length: float,
        *,
        material: Material = "pla",
        mass: float | None = None,
        group: str = "",
        frame: tuple[np.ndarray, np.ndarray] | None = None,
        inner: float = 0.0,
    ) -> None:
        """inner > 0이면 안쪽 반지름이 inner인 고리다."""
        origin, R = frame if frame is not None else (np.zeros(3), np.eye(3))
        z = R @ np.asarray(axis, float)
        z /= np.linalg.norm(z)
        helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x = np.cross(helper, z)
        x /= np.linalg.norm(x)
        rot = np.column_stack([x, np.cross(z, x), z])
        vol = math.pi * (radius**2 - inner**2) * length
        m = mass if mass is not None else (PLA_DENSITY * vol if material == "pla" else 0.0)
        pos = origin + R @ np.asarray(center, float)
        shape: Literal["cyl", "ring"] = "ring" if inner > 0 else "cyl"
        self.prims.append(
            Prim(
                name, link, shape, pos, rot, (radius, length / 2, inner), material, m, group or name
            )
        )

    def prism(
        self,
        link: int,
        name: str,
        pts: Sequence[tuple[float, float]],
        axis: Axis,
        a0: float,
        a1: float,
        *,
        material: Material = "pla",
        mass: float = 0.0,
        group: str = "",
        frame: tuple[np.ndarray, np.ndarray] | None = None,
    ) -> None:
        """볼록한 다각형 pts를 axis 방향으로 a0 ~ a1까지 뽑은 기둥 (볼록 껍질).

        pts는 axis가 x면 (y, z), y면 (x, z), z면 (x, y)다. 무게는 꼭짓점들의 가운데에 둔다.
        """
        origin, R = frame if frame is not None else (np.zeros(3), np.eye(3))
        k = "xyz".index(axis)
        local = np.array([np.insert(np.asarray(p, float), k, a) for a in (a0, a1) for p in pts])
        verts = origin + local @ R.T
        self.prims.append(
            Prim(
                name,
                link,
                "hull",
                verts.mean(axis=0),
                np.eye(3),
                (0.0, 0.0, 0.0),
                material,
                mass,
                group or name,
                verts,
            )
        )


@dataclass(frozen=True)
class Module:
    """서보 하나로 도는 관절. 부모 링크에 서보 몸체와 집을, 자식 링크에 혼과 ㄷ자 브래킷을 둔다."""

    joint: int  # 1~6
    shape: ServoShape
    origin: Vec3  # 부모 링크 좌표계에서 관절 점
    axis: Vec3  # 관절 축 (부모 좌표계)
    near: Vec3  # 축에서 서보 가까운 끝 방향 (부모 좌표계, 축과 수직)
    out: int = 1  # 출력 혼이 +축(1) 쪽인지 -축(-1) 쪽인지
    mid: float = 0.0  # 관절 점에서 서보 몸통 가운데까지 축 방향 거리
    neutral: float = 0.0  # 다리가 서보 가까운 끝 위에 오는 관절 각 [rad]

    def frame(self) -> tuple[np.ndarray, np.ndarray]:
        """서보 좌표계 (부모 링크 좌표계 기준 원점, 회전)."""
        ex = np.asarray(self.near, float)
        ey = np.asarray(self.axis, float) * self.out
        ez = np.cross(ex, ey)
        origin = np.asarray(self.origin, float) + np.asarray(self.axis, float) * self.mid
        return origin, np.column_stack([ex, ey, ez])

    @property
    def bridge_in(self) -> float:
        s = self.shape
        return math.hypot(s.near, s.half_width) + BRIDGE_GAP

    @property
    def arm_r(self) -> float:
        return self.shape.horn_r + 3.0 * MM

    @property
    def sweep_r(self) -> float:
        """브래킷이 돌며 쓸고 지나가는 반지름 (다리 바깥 모서리)."""
        return math.hypot(self.bridge_in + BRIDGE_T, self.arm_r) + 1.0 * MM

    def child_frame(self) -> tuple[np.ndarray, np.ndarray]:
        """자식 링크 좌표계에서 본 서보 좌표계 (관절이 neutral일 때 다리가 가까운 끝 위에 오도록)."""
        origin, R = self.frame()
        back = rotation(self.axis, -self.neutral)
        return back @ (origin - np.asarray(self.origin, float)), back @ R

    def build(self, b: Builder, servo_mass: float) -> None:
        s, j = self.shape, self.joint
        parent, child = j - 1, j
        f = self.frame()
        g_servo, g_house, g_bracket = f"J{j} 서보", f"J{j} 집", f"J{j} 브래킷"
        volumes = [(x1 - x0) * (y1 - y0) * (z1 - z0) for x0, x1, y0, y1, z0, z1 in s.zones]
        for k, (x0, x1, y0, y1, z0, z1) in enumerate(s.zones):
            b.box(
                parent,
                f"J{j}/servo_{k}",
                (x0, y0, z0),
                (x1, y1, z1),
                material="servo",
                frame=f,
                group=g_servo,
                mass=servo_mass * volumes[k] / sum(volumes),
            )
        # 부모: 먼 쪽만 감싸는 집. 옆벽(z)과 앞뒤 벽(y)은 브래킷이 쓸고 지나가는 반지름 밖에서 시작한다.
        fy0, fy1 = s.far_half
        end = -s.far - GAP
        side_z = s.half_width + GAP
        r = self.sweep_r
        x_side = -math.sqrt(max(r**2 - (side_z + WALL) ** 2, 0.0))
        x_face = min(x_side, -r)
        b.box(
            parent,
            f"J{j}/house_end",
            (end - WALL, fy0 - GAP - WALL, -side_z - WALL),
            (end, fy1 + GAP + WALL, side_z + WALL),
            frame=f,
            group=g_house,
        )
        for sign in (1, -1):
            b.box(
                parent,
                f"J{j}/house_side{sign:+d}",
                (end, fy0, min(sign * side_z, sign * (side_z + WALL))),
                (x_side, fy1, max(sign * side_z, sign * (side_z + WALL))),
                frame=f,
                group=g_house,
            )
        b.box(
            parent,
            f"J{j}/house_face+1",
            (end, fy1 + GAP, -side_z - WALL),
            (x_face, fy1 + GAP + WALL, side_z + WALL),
            frame=f,
            group=g_house,
        )
        b.box(
            parent,
            f"J{j}/house_face-1",
            (end, fy0 - GAP - WALL, -side_z - WALL),
            (x_face, fy0 - GAP, side_z + WALL),
            frame=f,
            group=g_house,
        )
        # 자식: 혼 두 개, 브래킷 팔 두 개(축 둘레 원판 + 다리까지 판), 다리.
        cf = self.child_frame()
        for name, (y0, y1) in (("horn_out", s.horn_out), ("horn_back", s.horn_back)):
            b.cyl(
                child,
                f"J{j}/{name}",
                (0.0, (y0 + y1) / 2, 0.0),
                Y,
                s.horn_r,
                y1 - y0,
                material="metal",
                mass=s.horn_mass / 2,
                frame=cf,
                group=g_bracket,
            )
        arms = ((s.horn_out[1], s.horn_out[1] + ARM_T), (s.horn_back[0] - ARM_T, s.horn_back[0]))
        for k, (y0, y1) in enumerate(arms):
            b.cyl(
                child,
                f"J{j}/arm{k}_round",
                (0.0, (y0 + y1) / 2, 0.0),
                Y,
                self.arm_r,
                y1 - y0,
                frame=cf,
                group=g_bracket,
            )
            b.box(
                child,
                f"J{j}/arm{k}_plate",
                (0.0, y0, -self.arm_r),
                (self.bridge_in, y1, self.arm_r),
                frame=cf,
                group=g_bracket,
            )
        b.box(
            child,
            f"J{j}/bridge",
            (self.bridge_in, arms[1][0], -self.arm_r),
            (self.bridge_in + BRIDGE_T, arms[0][1], self.arm_r),
            frame=cf,
            group=g_bracket,
        )


@dataclass(frozen=True)
class Layout:
    """골격 후보. 길이는 [m], 각도는 [rad]."""

    shoulder: float = 0.16  # 설치면 → J2 축
    upper: float = 0.22
    fore: float = 0.18
    d45: float = 0.063  # J4 → J5 (y). UR식 원통이 들어가는 값 (0019, covers.py)
    d56: float = 0.063  # J5 → J6 (z)
    d6f: float = 0.05  # J6 → 플랜지 (y)
    camera_tilt: float = 20 * DEG  # D405 광축을 도구 축 쪽으로 기울이는 각
    camera_back: float = (
        0.005  # D405 렌즈 면을 플랜지 면보다 뒤로 물리는 거리 (어댑터를 손으로 잡을 자리)
    )
    neutral3: float = 75 * DEG
    neutral4: float = 165 * DEG
    neutral5: float = 90 * DEG
    neutral6: float = (
        10 * DEG
    )  # 대칭이 아닌 도구로도 잡기 자세가 J6 범위 안에 들도록 가운데를 옮긴 값
    mid5: float = 0.0  # J5 서보 축 원점의 z (J5 점 기준)

    @property
    def mid6(self) -> float:
        """J6 서보 축 원점의 y. 플랜지 판이 출력 쪽 팔 바깥에 붙도록 정한다."""
        return self.d6f - shapes()["STS3215"].horn_out[1] - ARM_T - FLANGE_T


@dataclass(frozen=True)
class Tool:
    """도구 하나. 길이는 플랜지에서 TCP까지 (교체 어댑터 포함).

    symmetric이면 도구를 축 둘레로 180° 돌려도 같은 일을 하므로 J6 ± 180° 해도 쓴다
    (흡착은 어느 각도든, 평행 그리퍼는 180°마다 같다).
    box_depth는 박스를 잡을 때 TCP가 박스 윗면 아래로 들어가는 깊이다. 평행 그리퍼는 박스 윗면 구멍
    (어림 지름 16, 깊이 12 mm, 0014)에 손가락 끝을 넣고 바깥으로 벌려 잡는다. 구멍이 둥글어서 도구 축 둘레
    돌림은 흡착처럼 자유롭다.
    draw가 있으면 그 함수가 도구의 형상과 무게를 넣는다 (builder, 링크, 플랜지 면의 y). 그린 도구에 쓴다.
    없으면 add_tool이 이름으로 고른 자리표시 형상을 넣는다.
    """

    name: str
    length: float
    mass: float
    payload: float
    symmetric: bool
    box_depth: float = 0.0
    draw: Callable[[Builder, int, float], None] | None = None


TOOLS = {
    "pointer": Tool("포인터", ADAPTER_T + 0.080, 0.025, 0.0, True),
    "suction": Tool("흡착", ADAPTER_T + 0.060, 0.035, 0.10, True),
    "gripper": Tool("평행 그리퍼", ADAPTER_T + 0.095, 0.140, 0.15, True, box_depth=0.006),
    # 기둥 카메라 캘리브레이션(eye-to-hand)용 보드: 100 × 80 × 4 mm 판.
    # "board"는 판 면이 도구 축 방향을 보고, TCP는 판 겉면 가운데다.
    # "board_side"는 판을 어댑터 옆(플랜지 y 쪽 BOARD_SIDE_OFFSET)에 두고 판 면이 플랜지 쪽(-z)을 본다.
    # 도구가 아래를 향하면 판 면이 위를 본다.
    "board": Tool("보드", ADAPTER_T + 0.004, 0.030, 0.0, False),
    "board_side": Tool("옆 보드", ADAPTER_T, 0.040, 0.0, False),
}
BOARD_HALF = (0.050, 0.040)  # 보드 판의 반 크기 (플랜지 x, 플랜지 y) [m]
BOARD_SIDE_OFFSET = 0.090  # 옆 보드 가운데의 플랜지 y [m]


def arm(layout: Layout) -> Robot:
    params = with_lengths(
        load_arm(),
        layout.upper,
        layout.fore,
        (layout.d45, layout.d56, layout.d6f),
        layout.shoulder,
    )
    return arm_robot(params, load_servos())


def modules(layout: Layout, robot: Robot) -> list[Module]:
    o = [j.origin for j in robot.joints]
    sh = shapes()
    return [
        Module(2, sh["ST-3120"], o[1], Y, Z),
        Module(3, sh["STS3250"], o[2], Y, Z, neutral=layout.neutral3),
        Module(4, sh["STS3215"], o[3], Y, Z, neutral=layout.neutral4),
        Module(
            5, sh["STS3215"], o[4], Z, (-1.0, 0.0, 0.0), mid=layout.mid5, neutral=layout.neutral5
        ),
        Module(
            6, sh["STS3215"], o[5], Y, (0.0, 0.0, -1.0), mid=layout.mid6, neutral=layout.neutral6
        ),
    ]


@dataclass(frozen=True)
class Camera:
    """링크 5 좌표계에서 본 D405 렌즈 가운데와 광축."""

    lens: np.ndarray
    axis: np.ndarray


def camera_of(layout: Layout, mods: list[Module]) -> Camera:
    j6 = next(m for m in mods if m.joint == 6)
    reach6 = j6.sweep_r + MOVING_GAP
    lens = np.array([reach6 + 0.021, layout.d6f - layout.camera_back, layout.d56])
    t = layout.camera_tilt
    return Camera(lens, np.array([-math.sin(t), math.cos(t), 0.0]))


def build_arm(layout: Layout, tool: Tool) -> tuple[Robot, list[Prim], list[Module]]:
    """골격 후보의 형상 전체: 베이스, 관절 모듈, 링크 몸통, 손목 연결, D405, 플랜지, 도구."""
    robot = arm(layout)
    servos, measured = load_servos(), load_measured()
    mods = modules(layout, robot)
    by_joint = {m.joint: m for m in mods}
    b = Builder()
    for m in mods:
        m.build(b, servo_mass(m.shape.model, servos, measured))
    sh = robot.joints[1].origin[2]  # 설치면 → J2 축

    # 베이스(0)와 돌림판(1). J1 서보와 베어링은 베이스 안이라 충돌 검사에서 뺀다.
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
    j2 = by_joint[2]
    s2 = j2.shape
    house_bottom = sh - s2.far - GAP - WALL
    fy0, fy1 = s2.far_half
    b.box(
        1,
        "shoulder_post",
        (-(s2.half_width + GAP + WALL), fy0 - GAP - WALL, TURNTABLE_TOP),
        (s2.half_width + GAP + WALL, fy1 + GAP + WALL, house_bottom),
        group="J2 집",
    )

    # 위팔(2): J2 다리 위에서 J3 집 아래까지 상자 단면.
    top2 = j2.bridge_in + BRIDGE_T
    j3 = by_joint[3]
    bottom3 = layout.upper - j3.shape.far - GAP - WALL
    b.box(
        2, "upper_arm", (-0.016, -0.020, top2), (0.016, 0.020, bottom3), hollow=True, group="위팔"
    )

    # 아래팔(3): J3 브래킷에서 아래팔 방향(+z)으로 이어 주는 판 두 장, 그다음 상자 단면.
    root = j3.sweep_r + 1.0 * MM
    s3 = j3.shape
    for k, (y0, y1) in enumerate(
        ((s3.horn_out[1], s3.horn_out[1] + ARM_T), (s3.horn_back[0] - ARM_T, s3.horn_back[0]))
    ):
        # 브래킷 팔을 아래팔 쪽으로 늘인 판이라 이름을 J3 팔로 둔다 (서보와의 틈은 팔과 같다).
        b.box(3, f"J3/arm{k}_root", (-j3.arm_r, y0, 0.0), (j3.arm_r, y1, root), group="아래팔")
    j4 = by_joint[4]
    bottom4 = layout.fore - j4.shape.far - GAP - WALL
    b.box(
        3, "forearm", (-0.015, -0.0179, root), (0.015, 0.0179, bottom4), hollow=True, group="아래팔"
    )

    # 손목 1(4): J4 브래킷 → J5 집. J5 브래킷 뒷면 팔 아래(-z)로 돌아가는 판이라 J5 브래킷이 쓸고 지나가는 자리를 피한다.
    j5 = by_joint[5]
    s5 = j5.shape
    z_plate = layout.mid5 + s5.horn_back[0] - ARM_T - MOVING_GAP
    j4_bridge = j4.child_frame()
    tip = j4_bridge[0] + j4_bridge[1] @ np.array([j4.bridge_in + BRIDGE_T, 0.0, 0.0])
    s4 = j4.shape
    arm4_back = s4.horn_back[0] - ARM_T  # J4 뒷면 쪽 브래킷 팔의 바깥 면 (링크 4의 y)
    arm4_out = s4.horn_out[1] + ARM_T  # J4 출력 쪽 브래킷 팔의 바깥 면
    b.box(
        4,
        "wrist1_plate",
        (-0.015, arm4_back - ARM_T, z_plate - 4.0 * MM),
        (s5.far + GAP + WALL, layout.d45 + s5.half_width + GAP + WALL, z_plate),
        group="손목 1",
    )
    if tip[2] > z_plate:
        b.box(
            4,
            "wrist1_web",
            (-0.015, arm4_back - ARM_T, z_plate),
            (0.015, arm4_out, tip[2]),
            group="손목 1",
        )

    # 손목 2(5): J5 브래킷의 출력 쪽 팔 위에서 J6 브래킷 뒷면 팔 뒤(-y)로 돌아 J6 집까지 가는 판.
    j6 = by_joint[6]
    s6 = j6.shape
    y_plate = layout.mid6 + s6.horn_back[0] - ARM_T - MOVING_GAP
    top5 = layout.mid5 + s5.horn_out[1] + ARM_T
    reach6 = j6.sweep_r + MOVING_GAP  # J6 브래킷이 쓸고 지나가는 원 밖
    b.box(
        5,
        "wrist2_plate",
        (-0.015, y_plate - 4.0 * MM, top5),
        (0.015, y_plate, layout.d56 + s6.far + GAP + WALL),
        group="손목 2",
    )
    # D405 (42 × 42 × 23): J5 축에서 옆(+x)으로 비켜, J6 브래킷이 쓸고 지나가는 원 밖에 단다 (0013).
    # 렌즈 면(42 × 42)은 도구 방향(+y)을 보고, 플랜지 면보다 camera_back만큼 뒤에 둔다. 광축 기울기는 camera_of.
    lens_y = layout.d6f - layout.camera_back
    b.box(
        5,
        "camera_arm",
        (reach6, y_plate - 4.0 * MM, layout.d56 - 0.015),
        (reach6 + 4.0 * MM, lens_y - 0.023, layout.d56 + 0.015),
        group="손목 2",
    )
    b.box(
        5,
        "camera_link",
        (0.015, y_plate - 4.0 * MM, layout.d56 - 0.015),
        (reach6 + 4.0 * MM, y_plate, layout.d56 + 0.015),
        group="손목 2",
    )
    b.box(
        5,
        "d405",
        (reach6, lens_y - 0.023, layout.d56 - 0.021),
        (reach6 + 0.042, lens_y, layout.d56 + 0.021),
        material="camera",
        mass=D405_MASS,
        group="D405",
    )

    # 손목 3(6): 플랜지 판, 그다음 도구 (도구 좌표 z = 링크 6의 +y).
    yf = layout.d6f
    b.cyl(6, "flange", (0.0, yf - FLANGE_T / 2, 0.0), Y, 0.022, FLANGE_T, group="플랜지")
    add_tool(b, 6, yf, tool)
    return robot, b.prims, mods


def add_tool(b: Builder, link: int, yf: float, tool: Tool) -> None:
    """교체 어댑터와 도구. yf는 링크 6 좌표계에서 플랜지 면의 y이고, 도구는 +y로 나간다.

    link가 7이면 링크 6에 붙은 따로 된 몸체에 둔다 (좌표계는 링크 6과 같다). 다른 팔(UR3e)에 같은 도구를 달 때 쓴다.
    """
    if tool.draw is not None:
        tool.draw(b, link, yf)
        return
    b.cyl(
        link,
        "adapter",
        (0.0, yf + ADAPTER_T / 2, 0.0),
        Y,
        0.022,
        ADAPTER_T,
        mass=0.015,
        group="교체 어댑터",
    )
    tool_lo = yf + ADAPTER_T
    if tool.name == "포인터":
        b.cyl(
            link,
            "pointer",
            (0.0, tool_lo + 0.040, 0.0),
            Y,
            0.005,
            0.080,
            material="tool",
            mass=tool.mass,
            group="도구",
        )
    elif tool.name == "흡착":
        b.cyl(
            link,
            "suction",
            (0.0, tool_lo + 0.030, 0.0),
            Y,
            0.009,
            0.060,
            material="tool",
            mass=tool.mass,
            group="도구",
        )
    elif tool.name == "보드":
        # 링크 6 좌표계에서 플랜지 x는 +z, 플랜지 y는 +x다 (T_last_flange).
        bx, by = BOARD_HALF
        b.box(
            link,
            "board",
            (-by, tool_lo, -bx),
            (by, tool_lo + 0.004, bx),
            material="tool",
            mass=tool.mass,
            group="도구",
        )
    elif tool.name == "옆 보드":
        bx, by = BOARD_HALF
        c = BOARD_SIDE_OFFSET
        b.box(
            link,
            "board_arm",
            (0.020, tool_lo - 0.004, -0.010),
            (c - by, tool_lo + 0.004, 0.010),
            material="tool",
            mass=tool.mass * 0.25,
            group="도구",
        )
        b.box(
            link,
            "board",
            (c - by, tool_lo, -bx),
            (c + by, tool_lo + 0.004, bx),
            material="tool",
            mass=tool.mass * 0.75,
            group="도구",
        )
    else:
        # STS3215 하나로 랙·피니언을 돌리는 몸통, 그 아래 손가락 두 개 (열린 폭 40 mm).
        b.box(
            link,
            "gripper_body",
            (-0.032, tool_lo, -0.018),
            (0.032, tool_lo + 0.050, 0.018),
            material="tool",
            mass=tool.mass * 0.8,
            group="도구",
        )
        for sign in (1, -1):
            x0, x1 = sorted((sign * 0.020, sign * 0.028))
            b.box(
                link,
                f"finger{sign:+d}",
                (x0, tool_lo + 0.050, -0.010),
                (x1, tool_lo + 0.097, 0.010),
                material="tool",
                mass=tool.mass * 0.1,
                group="도구",
            )


# ----------------------------------------------------------------------------------------------------
# MuJoCo 모델과 충돌 검사


@dataclass
class Scene:
    robot: Robot
    prims: list[Prim]
    model: mujoco.MjModel
    data: mujoco.MjData
    geom_prim: dict[int, Prim]


RING_SEGMENTS = 32  # 고리를 나누는 상자 수. 상자 바깥 모서리는 고리보다 0.15 mm쯤 나온다


def _geom_pieces(p: Prim) -> list[tuple[str, int, list[float], np.ndarray, np.ndarray]]:
    """MuJoCo geom으로 쓸 조각들 (이름, 종류, 크기, 위치, 회전). MuJoCo에 속 빈 원기둥이 없어서
    고리는 둘레를 상자로 나눈다. 상자 안쪽 면은 고리 안쪽 원 밖에 있어서 안쪽 틈은 줄지 않는다."""
    box, cyl = int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_CYLINDER)
    if p.shape == "box":
        return [(p.name, box, list(p.size), p.pos, p.rot)]
    if p.shape == "cyl":
        return [(p.name, cyl, [p.size[0], p.size[1], 0.0], p.pos, p.rot)]
    r_out, half, r_in = p.size
    mid, width = (r_in + r_out) / 2, r_out * math.tan(math.pi / RING_SEGMENTS)
    pieces = []
    for k in range(RING_SEGMENTS):
        a = 2 * math.pi * k / RING_SEGMENTS
        pos = p.pos + p.rot @ np.array([mid * math.cos(a), mid * math.sin(a), 0.0])
        rot = p.rot @ rotation(Z, a)
        pieces.append((f"{p.name}#{k}", box, [(r_out - r_in) / 2, width, half], pos, rot))
    return pieces


def build_scene(
    robot: Robot,
    prims: list[Prim],
    fixed: list[Prim] | None = None,
    *,
    exclude: tuple[tuple[int, int], ...] = (),
) -> Scene:
    """fixed는 책상에 고정된 물건(카메라 기둥 등)이다. 베이스와 따로 "fixture" 몸체에 둔다.

    이웃한 링크끼리도 부딪힘을 본다. exclude는 부딪힘을 보지 않을 링크 쌍이다 (0 베이스 ~ 6, 7은 도구 몸체).
    """
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    spec.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    world = spec.worldbody
    world.add_geom(
        name="desk",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[1.0, 1.0, 0.01],
        contype=2,
        conaffinity=0,
    )
    bodies = [world.add_body(name="base")]
    for i, joint in enumerate(robot.joints):
        body = bodies[-1].add_body(name=f"link{i + 1}", pos=list(joint.origin))
        body.add_joint(name=joint.name, type=mujoco.mjtJoint.mjJNT_HINGE, axis=list(joint.axis))
        bodies.append(body)
    if any(p.link == 7 for p in prims) or any(7 in pair for pair in exclude):
        bodies.append(bodies[-1].add_body(name="tool"))
    for a, b in exclude:
        spec.add_exclude(bodyname1=bodies[a].name, bodyname2=bodies[b].name)
    fixture = world.add_body(name="fixture")
    fixed = fixed or []
    placed = [(p, bodies[p.link], p.link == 0) for p in prims] + [(p, fixture, True) for p in fixed]
    by_name: dict[str, Prim] = {}
    for p, body, static in placed:
        if p.shape == "hull":
            # MuJoCo는 면 없이 점만 준 메시를 볼록 껍질로 만든다.
            assert p.verts is not None
            spec.add_mesh(name=f"{p.name}@mesh", uservert=p.verts.flatten().tolist())
            body.add_geom(
                name=p.name,
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=f"{p.name}@mesh",
                contype=1,
                conaffinity=1 if static else 3,
                margin=CONTACT_MARGIN,
            )
            by_name[p.name] = p
            continue
        for name, gtype, size, pos, rot in _geom_pieces(p):
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, rot.flatten())
            body.add_geom(
                name=name,
                type=mujoco.mjtGeom(gtype),
                size=size,
                pos=list(pos),
                quat=list(quat),
                contype=1,
                conaffinity=1 if static else 3,
                margin=CONTACT_MARGIN,
            )
            by_name[name] = p
    all_prims = [*prims, *fixed]
    model = spec.compile()
    data = mujoco.MjData(model)
    geom_prim = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name): p for name, p in by_name.items()
    }
    return Scene(robot, all_prims, model, data, geom_prim)


@dataclass(frozen=True)
class Hit:
    a: str
    b: str
    dist: float
    designed: bool = False  # 설계상 가까운 쌍이라 "빠듯함" 보고에서 뺀다 (겹치면 부딪힘으로 센다)


def hits(scene: Scene, q: np.ndarray) -> list[Hit]:
    """자세 q에서 서로 다른 링크의 형상이 CONTACT_MARGIN 안으로 가까워진 쌍들 (desk 포함)."""
    m, d = scene.model, scene.data
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    out = []
    for c in d.contact[: d.ncon]:
        pa, pb = scene.geom_prim.get(c.geom1), scene.geom_prim.get(c.geom2)
        a = pa.group if pa else "책상"
        bname = pb.group if pb else "책상"
        designed = bool(pa and pb and _by_design(pa, pb))
        out.append(Hit(a, bname, float(c.dist), designed))
    return out


def _by_design(a: Prim, b: Prim) -> bool:
    """설계상 아주 가까운 쌍: 같은 관절의 서보와 그 혼·브래킷 팔·다리 (틈 0.3~2 mm), 베이스와 돌림판(베어링)."""
    if {a.name, b.name} == {"base", "turntable"}:
        return True
    ja, jb = a.name.split("/")[0], b.name.split("/")[0]
    if ja != jb or not ja.startswith("J"):
        return False
    kinds = {a.name.split("/")[1].split("_")[0], b.name.split("/")[1].split("_")[0]}
    return "servo" in kinds and bool(kinds & {"horn", "arm0", "arm1", "bridge"})


def collisions(found: list[Hit]) -> list[Hit]:
    return [h for h in found if h.dist < -PENETRATION]


# ----------------------------------------------------------------------------------------------------
# 자세


def tool_pose(p: Vec3, tilt: float, tilt_dir: float, yaw: float, length: float) -> Mat:
    """TCP를 p에 두고 도구가 아래(-z)에서 tilt만큼 tilt_dir(방위) 쪽으로 기운 플랜지 자세.

    yaw는 도구 축 둘레 돌림이다 (tilt = 0일 때 common.tool_down과 같은 기준, 방위는 p 쪽).
    """
    azimuth = math.atan2(p[1], p[0])
    R0 = rotation(Z, azimuth) @ rotation(Y, math.pi) @ rotation(Z, yaw)
    if tilt > 0:
        d = np.array(
            [
                math.sin(tilt) * math.cos(tilt_dir),
                math.sin(tilt) * math.sin(tilt_dir),
                -math.cos(tilt),
            ]
        )
        k = np.cross([0.0, 0.0, -1.0], d)
        R0 = rotation(k / np.linalg.norm(k), tilt) @ R0
    T = np.eye(4)
    T[:3, :3] = R0
    T[:3, 3] = np.asarray(p, float) - R0 @ np.array([0.0, 0.0, length])
    return T


@dataclass(frozen=True)
class Pose:
    label: str
    T: Mat


def stacking_poses(tool: Tool) -> list[Pose]:
    """박스 쌓기 (0014): 박스 윗면(과 그 위 접근 높이)에서 도구가 박스를 잡는 깊이만큼 아래가 TCP다."""
    return [
        Pose(
            f"박스 r{r * 100:.0f} h{h * 100:.0f}",
            tool_pose((r, 0.0, h - tool.box_depth), 0.0, 0.0, yaw, tool.length),
        )
        for r, h, yaw in stacking_targets()
    ]


def pointer_poses(tool: Tool, r: float = 0.22) -> list[Pose]:
    """포인터 시험: 끝을 앞쪽 가운데 한 점(반지름 r, 높이 2 cm)에 두고 40°까지 여러 방향으로 기울인다."""
    p = (r, 0.0, 0.02)
    out = [Pose("포인터 0°", tool_pose(p, 0.0, 0.0, 0.0, tool.length))]
    for tilt in (20 * DEG, 40 * DEG):
        for k in range(8):
            out.append(
                Pose(
                    f"포인터 {tilt / DEG:.0f}° 방향 {k * 45}°",
                    tool_pose(p, tilt, k * 45 * DEG, 0.0, tool.length),
                )
            )
    return out


def grasp_poses(tool: Tool) -> list[Pose]:
    """비스듬히 잡기: 책상 위 물체를 도구를 기울여 잡는다 (TCP 높이 2.5 cm)."""
    out = []
    for r in (0.16, 0.22, 0.28):
        for yaw in (0.0, 90 * DEG):
            out.append(
                Pose(
                    f"잡기 r{r * 100:.0f} 수직 돌림{yaw / DEG:.0f}",
                    tool_pose((r, 0.0, 0.025), 0.0, 0.0, yaw, tool.length),
                )
            )
            for tilt in (30 * DEG, 45 * DEG):
                for k, name in enumerate(("바깥", "왼쪽", "안쪽", "오른쪽")):
                    out.append(
                        Pose(
                            f"잡기 r{r * 100:.0f} {tilt / DEG:.0f}° {name} 돌림{yaw / DEG:.0f}",
                            tool_pose((r, 0.0, 0.025), tilt, k * 90 * DEG, yaw, tool.length),
                        )
                    )
    return out


def solve(ik: AnalyticIK, T: Mat, symmetric: bool = False) -> list[np.ndarray]:
    """작업 자세의 해. symmetric이면 J6를 180° 돌린 해도 넣는다 (J6 범위 안일 때만)."""
    sols = [np.asarray(q) for q in working_solutions(ik, T)]
    if symmetric:
        j6 = ik.robot.joints[5]
        for q in list(sols):
            for turn in (math.pi, -math.pi):
                if j6.lower <= q[5] + turn <= j6.upper:
                    flipped = q.copy()
                    flipped[5] += turn
                    sols.append(flipped)
    return sols


# ----------------------------------------------------------------------------------------------------
# 보고


@dataclass
class SetResult:
    name: str
    total: int = 0
    reached: int = 0
    clear: int = 0
    worst: Hit | None = None
    tight: Hit | None = None
    failures: list[str] = field(default_factory=list)
    qs: list[np.ndarray] = field(default_factory=list)
    clear_qs: list[np.ndarray] = field(default_factory=list)


def check_set(
    scene: Scene, ik: AnalyticIK, poses: list[Pose], name: str, symmetric: bool = False
) -> SetResult:
    res = SetResult(name, total=len(poses))
    for pose in poses:
        sols = solve(ik, pose.T, symmetric)
        if not sols:
            res.failures.append(f"{pose.label}: 작업 자세로 닿지 않음")
            continue
        res.reached += 1
        best: tuple[float, np.ndarray, list[Hit]] | None = None
        for q in sols:
            found = hits(scene, q)
            worst = min((h.dist for h in found), default=CONTACT_MARGIN)
            if best is None or worst > best[0]:
                best = (worst, q, found)
        assert best is not None
        _, q, found = best
        res.qs.append(q)
        bad = collisions(found)
        if bad:
            h = min(bad, key=lambda h: h.dist)
            angles = " ".join(f"{v / DEG:.0f}" for v in q)
            res.failures.append(f"{pose.label}: {h.a} ↔ {h.b} {h.dist * 1000:.1f} mm (q {angles})")
            if res.worst is None or h.dist < res.worst.dist:
                res.worst = h
        else:
            res.clear += 1
            res.clear_qs.append(q)
            near = [h for h in found if h.dist < CONTACT_MARGIN and not h.designed]
            if near:
                h = min(near, key=lambda h: h.dist)
                if res.tight is None or h.dist < res.tight.dist:
                    res.tight = h
    return res


def joint_ranges(scene: Scene, q0: np.ndarray) -> list[tuple[float, float, str, str]]:
    """q0에서 관절 하나씩만 움직여 부딪히기 전까지 돌 수 있는 범위 [rad]와 막는 쌍 (±180° 안, 책상은 뺀다)."""
    out = []
    for j in range(6):
        ends: list[tuple[float, str]] = []
        for step in (1 * DEG, -1 * DEG):
            q = q0.copy()
            angle, why = q0[j], "막힘 없음"
            while abs(angle - q0[j]) < math.pi:
                q[j] = angle + step
                bad = [h for h in collisions(hits(scene, q)) if "책상" not in (h.a, h.b)]
                if bad:
                    h = min(bad, key=lambda h: h.dist)
                    why = f"{h.a} ↔ {h.b}"
                    break
                angle += step
            ends.append((angle, why))
        out.append((ends[1][0], ends[0][0], ends[1][1], ends[0][1]))
    return out


def torque_ratios(robot: Robot, prims: list[Prim], tool: Tool, qs: list[np.ndarray]) -> np.ndarray:
    """설계 형상의 무게(출력물은 PLA 밀도, 서보는 사양)로 계산한 버팀 토크 / 정격 토크의 최댓값."""
    servos = load_servos()
    rated = np.array(
        [
            servos[s].rated_torque
            for s in ("STS3215", "ST-3120", "STS3250", "STS3215", "STS3215", "STS3215")
        ]
    )
    masses = [PointMass(p.name, min(p.link, 6), tuple(p.pos), p.mass) for p in prims if p.mass > 0]
    if tool.payload:
        masses.append(PointMass("짐", FLANGE, (0.0, 0.0, tool.length), tool.payload))
    worst = np.zeros(6)
    for q in qs:
        worst = np.maximum(worst, np.abs(holding_torques(robot, q, masses)))
    return worst / rated


D405_HALF_FOV = 29 * DEG  # 깊이 시야 세로 58°의 절반 (사양서와 홈페이지가 같다. docs/hardware.md)
D405_MIN_RANGE = 0.07  # [m] 848×480에서 깊이를 재는 가장 가까운 거리 (1280×720은 10 cm)


def camera_view(
    scene: Scene, q: np.ndarray, tool: Tool, cam: Camera, ahead: float = 0.002
) -> tuple[float, float, float]:
    """D405가 TCP 둘레(±30 mm, TCP보다 ahead만큼 앞 평면)를 얼마나 보는지.

    (보이는 점 비율, TCP가 광축에서 벗어난 각 [rad], 렌즈에서 TCP까지 거리 [m])를 준다.
    도구가 가리면 안 보이는 것으로 센다. 렌즈가 달린 링크 5는 빼고 본다.
    """
    m, d = scene.model, scene.data
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    link5 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link5")
    R5, p5 = d.xmat[link5].reshape(3, 3), d.xpos[link5]
    lens, axis = p5 + R5 @ cam.lens, R5 @ cam.axis
    link6 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link6")
    T6 = np.eye(4)
    T6[:3, :3], T6[:3, 3] = d.xmat[link6].reshape(3, 3), d.xpos[link6]
    Tf = T6 @ scene.robot.T_last_flange
    tool_axis = Tf[:3, 2]
    tcp = Tf[:3, 3] + tool_axis * tool.length
    u = Tf[:3, 0]
    v = Tf[:3, 1]
    visible = total = 0
    geomid = np.zeros(1, dtype=np.int32)
    for a in np.linspace(-0.03, 0.03, 7):
        for b in np.linspace(-0.03, 0.03, 7):
            target = tcp + tool_axis * ahead + u * a + v * b
            ray = target - lens
            total += 1
            hit = mujoco.mj_ray(m, d, lens, ray, None, 1, link5, geomid)
            if hit < 0 or hit >= 0.999:
                visible += 1
    to_tcp = tcp - lens
    off = math.acos(float(np.clip(axis @ to_tcp / np.linalg.norm(to_tcp), -1.0, 1.0)))
    return visible / total, off, float(np.linalg.norm(to_tcp))


def adapter_clearance(scene: Scene, q: np.ndarray) -> tuple[float, str]:
    """교체 어댑터 둘레의 가장 가까운 다른 링크 형상까지 거리 (손가락으로 잡아 돌릴 자리)."""
    m, d = scene.model, scene.data
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    adapter = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "adapter")
    best, name = math.inf, ""
    fromto = np.zeros(6)
    for g, p in scene.geom_prim.items():
        if p.link >= 6:
            continue
        dist = mujoco.mj_geomDistance(m, d, adapter, g, 0.1, fromto)
        if dist < best:
            best, name = dist, p.group
    return best, name


def mass_table(prims: list[Prim]) -> dict[str, float]:
    out: dict[str, float] = {}
    for p in prims:
        out[p.group] = out.get(p.group, 0.0) + p.mass
    return out


# (이름, 도구, 자세): 박스 쌓기는 첫 그리퍼인 평행 그리퍼로 한다 (docs/design.md의 도구 순서)
POSE_SETS = (
    ("박스 쌓기", "gripper", stacking_poses),
    ("포인터", "pointer", pointer_poses),
    ("잡기", "gripper", grasp_poses),
)


def evaluate(layout: Layout) -> list[SetResult]:
    out = []
    for name, key, poses_of in POSE_SETS:
        tool = TOOLS[key]
        robot, prims, _ = build_arm(layout, tool)
        scene = build_scene(robot, prims)
        out.append(check_set(scene, AnalyticIK(robot), poses_of(tool), name, tool.symmetric))
    return out


def _segment_overlap(a: tuple[float, float], b: tuple[float, float]) -> float:
    return min(a[1], b[1]) - max(a[0], b[0])


def rect_overlaps(corners: np.ndarray, x: tuple[float, float], z: tuple[float, float]) -> bool:
    """볼록한 사각형(꼭짓점 4개, x–z 평면)이 축에 맞춘 사각형 x × z와 겹치는지 (분리축 검사)."""
    box = np.array([[x[0], z[0]], [x[1], z[0]], [x[1], z[1]], [x[0], z[1]]])
    axes = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
    for k in range(4):
        edge = corners[(k + 1) % 4] - corners[k]
        axes.append(np.array([-edge[1], edge[0]]))
    for axis in axes:
        pa, pb = corners @ axis, box @ axis
        if _segment_overlap((pa.min(), pa.max()), (pb.min(), pb.max())) <= 0:
            return False
    return True


@dataclass(frozen=True)
class ConnectorCheck:
    joint: int
    part: str
    reach_mm: float  # 커넥터 자리 안으로 x 방향으로 들어간 깊이 (둥근 부분) [mm]
    depth_mm: tuple[float, float]  # 겹치는 곳의 뒷면에서 깊이 범위 [mm]
    angles: tuple[float, float] | None  # 겹치는 관절 각 범위 [°] (둥근 부분은 None: 늘 겹침)


def connector_checks(layout: Layout) -> list[ConnectorCheck]:
    """STS 관절마다 뒷면 커넥터 자리(플러그와 꺾인 선, CONNECTOR_X × CONNECTOR_DEPTH)에 자식 링크의 뒷면 쪽
    부품(뒷면 혼, 브래킷 팔, 다리)이 관절 범위 안에서 들어오는지 본다. 커넥터 자리는 충돌 검사에 넣지 않고 따로 본다.
    """
    robot = arm(layout)
    out = []
    for m in modules(layout, robot):
        s = m.shape
        if s.model == "ST-3120":
            continue
        back = STS_BACK * MM
        keep_y = (back - CONNECTOR_DEPTH * MM, back)
        keep_x = (CONNECTOR_X[0] * MM, CONNECTOR_X[1] * MM)
        keep_z = (-s.half_width, s.half_width)
        arm_back = (s.horn_back[0] - ARM_T, s.horn_back[0])
        arm_out_face = s.horn_out[1] + ARM_T
        discs = (("뒷면 혼", s.horn_r, s.horn_back), ("뒷면 쪽 팔의 둥근 판", m.arm_r, arm_back))
        for part, r, ys in discs:
            dy = (max(ys[0], keep_y[0]), min(ys[1], keep_y[1]))
            if dy[1] > dy[0] and -r < keep_x[1]:
                out.append(
                    ConnectorCheck(
                        m.joint,
                        part,
                        (keep_x[1] + r) / MM,
                        ((back - dy[1]) / MM, (back - dy[0]) / MM),
                        None,
                    )
                )
        boxes = (
            ("뒷면 쪽 팔의 판", (0.0, m.bridge_in), arm_back),
            ("다리", (m.bridge_in, m.bridge_in + BRIDGE_T), (arm_back[0], arm_out_face)),
        )
        joint = robot.joints[m.joint - 1]
        for part, (x0, x1), ys in boxes:
            dy = (max(ys[0], keep_y[0]), min(ys[1], keep_y[1]))
            if dy[1] <= dy[0]:
                continue
            hit = []
            for deg in range(round(joint.lower / DEG), round(joint.upper / DEG) + 1):
                # 자식 링크가 관절 각 q에서 서보 좌표계 y 둘레로 (q - neutral)만큼 돈 자리 (child_frame 기준)
                turn = (deg * DEG - m.neutral) * m.out
                c, sn = math.cos(turn), math.sin(turn)
                local = np.array([[x0, -m.arm_r], [x1, -m.arm_r], [x1, m.arm_r], [x0, m.arm_r]])
                # y축 둘레 회전: x' = x cos + z sin, z' = -x sin + z cos
                corners = np.column_stack(
                    [local[:, 0] * c + local[:, 1] * sn, -local[:, 0] * sn + local[:, 1] * c]
                )
                if rect_overlaps(corners, keep_x, keep_z):
                    hit.append(deg)
            if hit:
                out.append(
                    ConnectorCheck(
                        m.joint,
                        part,
                        0.0,
                        ((back - dy[1]) / MM, (back - dy[0]) / MM),
                        (float(min(hit)), float(max(hit))),
                    )
                )
    return out


def outward_tilt_table(layout: Layout) -> None:
    """도구를 바깥(베이스 반대쪽)으로 기울여 가까운 곳을 잡을 때 J3가 얼마나 접히는지, 부딪히는지."""
    tool = TOOLS["gripper"]
    robot, prims, _ = build_arm(layout, tool)
    scene = build_scene(robot, prims)
    ik = AnalyticIK(robot)
    print(
        "\n### 바깥으로 기울여 잡기 (평행 그리퍼, TCP 높이 2.5 cm): J3 [°], × 부딪힘, - 닿지 않음"
    )
    radii = [0.16 + 0.02 * i for i in range(8)]
    print("| 기울기 | " + " | ".join(f"r {r * 100:.0f}" for r in radii) + " |")
    print("|---|" + "---|" * len(radii))
    for tilt in (20 * DEG, 30 * DEG, 40 * DEG, 45 * DEG):
        cells = []
        for r in radii:
            sols = solve(
                ik, tool_pose((r, 0.0, 0.025), tilt, 0.0, 0.0, tool.length), tool.symmetric
            )
            if not sols:
                cells.append("-")
                continue
            ok = [q for q in sols if not collisions(hits(scene, q))]
            q = (ok or sols)[0]
            cells.append(f"{q[2] / DEG:.0f}" + ("" if ok else " ×"))
        print(f"| {tilt / DEG:.0f}° | " + " | ".join(cells) + " |")


def main() -> None:
    report(Layout())


def connector_report(layout: Layout) -> None:
    print(
        f"\n### 뒷면 커넥터 자리 (축에서 {-CONNECTOR_X[1]:.0f}~{-CONNECTOR_X[0]:.0f} mm, 뒷면에서 "
        f"{CONNECTOR_DEPTH:.0f} mm까지)와 자식 링크의 뒷면 쪽 부품"
    )
    found = connector_checks(layout)
    if not found:
        print("- 관절 범위 안에서 들어오는 부품 없음")
    for c in found:
        where = (
            f"관절 각과 상관없이 {c.reach_mm:.1f} mm 들어옴"
            if c.angles is None
            else f"관절 각 {c.angles[0]:.0f}°~{c.angles[1]:.0f}°에서 들어옴"
        )
        print(
            f"- J{c.joint} {c.part}: {where} (뒷면에서 깊이 {c.depth_mm[0]:.1f}~{c.depth_mm[1]:.1f} mm)"
        )


def report(layout: Layout) -> None:
    sh = shapes()
    print(
        "\n# STS 서보 외형: 도면과 실측 (몸통 32, 먼 쪽 뒷면 판까지 "
        f"{(sh['STS3215'].far_half[1] - sh['STS3215'].far_half[0]) / MM:.2f} mm, 출력 혼 윗면 ~ 뒷면 "
        f"STS3215 {(sh['STS3215'].horn_out[1] / MM - STS_BACK):.2f} · "
        f"STS3250 {(sh['STS3250'].horn_out[1] / MM - STS_BACK):.2f} mm)"
    )
    print(
        f"## 골격 후보: 위팔 {layout.upper * 1000:.0f}, 아래팔 {layout.fore * 1000:.0f}, 손목 오프셋 {layout.d45 * 1000:.0f} · {layout.d56 * 1000:.0f} · {layout.d6f * 1000:.0f} mm"
    )
    results = evaluate(layout)
    for (name, key, poses_of), res in zip(POSE_SETS, results, strict=True):
        tool = TOOLS[key]
        robot, prims, _ = build_arm(layout, tool)
        ratio = torque_ratios(robot, prims, tool, res.qs) if res.qs else np.zeros(6)
        print(f"\n### {name}: {tool.name} ({len(poses_of(tool))}개 자세)")
        print(f"- 닿음 {res.reached}/{res.total}, 부딪힘 없음 {res.clear}/{res.reached}")
        if res.worst:
            print(
                f"- 가장 깊은 부딪힘: {res.worst.a} ↔ {res.worst.b} {res.worst.dist * 1000:.1f} mm"
            )
        if res.tight:
            print(f"- 가장 빠듯한 곳: {res.tight.a} ↔ {res.tight.b} {res.tight.dist * 1000:.1f} mm")
        print("- 버팀 토크 / 정격: " + ", ".join(f"J{i + 1} {r:.2f}" for i, r in enumerate(ratio)))
        for line in res.failures[:12]:
            print(f"  - {line}")
        if len(res.failures) > 12:
            print(f"  - … 외 {len(res.failures) - 12}개")
    tool = TOOLS["gripper"]
    robot, prims, _ = build_arm(layout, tool)
    scene = build_scene(robot, prims)
    ik = AnalyticIK(robot)
    home = solve(ik, tool_pose((0.20, 0.0, 0.10), 0.0, 0.0, 0.0, tool.length))[0]
    print("\n### 관절 범위 (평행 그리퍼, 앞쪽 20 cm 높이 10 cm 자세에서 하나씩 돌림)")
    ranges = joint_ranges(scene, home)
    for j, (lo, hi, why_lo, why_hi) in enumerate(ranges):
        print(
            f"- J{j + 1}: {lo / DEG:.0f}° ~ {hi / DEG:.0f}° (지금 {home[j] / DEG:.0f}°) 아래: {why_lo} / 위: {why_hi}"
        )
    used = np.array([q for res in results for q in res.clear_qs])
    print("\n### 부딪힘 없이 된 자세들이 쓴 관절 범위 vs 돌 수 있는 범위 [°]")
    print("| 관절 | 쓴 범위 | 돌 수 있는 범위 |")
    print("|---|---|---|")
    for j in range(6):
        print(
            f"| J{j + 1} | {used[:, j].min() / DEG:.0f} ~ {used[:, j].max() / DEG:.0f} | "
            f"{ranges[j][0] / DEG:.0f} ~ {ranges[j][1] / DEG:.0f} |"
        )
    print("\n### 포인터 시험 자리 (끝 높이 2 cm, 40°까지 8방향): 부딪힘 없는 자세 수")
    pointer = TOOLS["pointer"]
    rp, pp, _ = build_arm(layout, pointer)
    sp = build_scene(rp, pp)
    ikp = AnalyticIK(rp)
    for r in (0.20, 0.22, 0.24, 0.26, 0.28):
        res_p = check_set(sp, ikp, pointer_poses(pointer, r), "포인터", pointer.symmetric)
        print(f"- 반지름 {r * 100:.0f} cm: {res_p.clear}/{res_p.total}")
    # 대칭이 아닌 도구는 J6 ± 180° 대안이 없다. 평행 그리퍼 자세들을 그런 도구로 할 때 J6가 범위 안인지 본다.
    lo6, hi6 = ranges[5][0], ranges[5][1]
    needed = [q[5] for pose in grasp_poses(tool) for q in solve(ik, pose.T)[:1]]
    inside = sum(lo6 <= v <= hi6 for v in needed)
    print(
        f"- 대칭이 아닌 도구라면 (J6 ± 180° 대안 없음): 잡기 자세 {inside}/{len(needed)}개가 J6 범위 안, "
        f"필요한 J6 {min(needed) / DEG:.0f}° ~ {max(needed) / DEG:.0f}°"
    )
    cam = camera_of(layout, modules(layout, robot))
    print("\n### D405 시야 (앞쪽 20 cm 높이 10 cm 자세, TCP 둘레 ±30 mm)")
    print(
        f"- 광축은 도구 축 쪽으로 {layout.camera_tilt / DEG:.0f}° 기울임, 렌즈 면은 플랜지 면보다 "
        f"{layout.camera_back * 1000:.0f} mm 뒤. 시야 절반 {D405_HALF_FOV / DEG:.0f}°(좁은 쪽), 최소 거리 {D405_MIN_RANGE * 100:.0f} cm(848×480)"
    )
    for key in ("gripper", "suction", "pointer"):
        t = TOOLS[key]
        rb, rp, _ = build_arm(layout, t)
        sc = build_scene(rb, rp)
        q0 = solve(AnalyticIK(rb), tool_pose((0.20, 0.0, 0.10), 0.0, 0.0, 0.0, t.length))[0]
        for turn in (0, 45, 90) if key == "gripper" else (0,):
            q = q0.copy()
            q[5] += turn * DEG
            seen, off, dist = camera_view(sc, q, t, cam)
            ahead, _, _ = camera_view(sc, q, t, cam, ahead=0.06)
            label = f"{t.name}" + (f" (손가락 방향 {turn}° 돌림)" if key == "gripper" else "")
            print(
                f"- {label}: TCP 둘레 {seen * 100:.0f}%, 6 cm 앞(잡기 전 높이) {ahead * 100:.0f}% 보임. "
                f"TCP가 광축에서 {off / DEG:.0f}°, 거리 {dist * 100:.1f} cm"
            )
    gap, part = adapter_clearance(scene, home)
    print(f"- 교체 어댑터에서 가장 가까운 다른 부품: {part} {gap * 1000:.1f} mm")
    connector_report(layout)
    outward_tilt_table(layout)
    print("\n### 손목 오프셋 비교: 부딪힘 없는 자세 수 (박스 쌓기 / 포인터 / 잡기)")
    print("| J4→J5 | J5→J6 | J6→플랜지 | 박스 쌓기 | 포인터 | 잡기 |")
    print("|---|---|---|---|---|---|")
    for d45, d56, d6f in (
        (0.05, 0.05, 0.05),
        (0.055, 0.05, 0.05),
        (0.05, 0.055, 0.05),
        (0.055, 0.055, 0.05),
        (0.055, 0.058, 0.05),
        (0.055, 0.06, 0.05),
        (0.06, 0.06, 0.05),
        (0.05, 0.05, 0.04),
        (0.045, 0.045, 0.04),
    ):
        variant = Layout(d45=d45, d56=d56, d6f=d6f)
        cells = [f"{r.clear}/{r.total}" for r in evaluate(variant)]
        print(
            f"| {d45 * 1000:.0f} | {d56 * 1000:.0f} | {d6f * 1000:.0f} | "
            + " | ".join(cells)
            + " |"
        )
    print("\n### 무게 (평행 그리퍼 포함)")
    for group, m in sorted(mass_table(prims).items(), key=lambda kv: -kv[1]):
        if m > 0.002:
            print(f"- {group}: {m * 1000:.0f} g")


if __name__ == "__main__":
    main()
