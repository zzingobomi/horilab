"""손목 몸통: 앞 관절의 출력 원판에 앉는 발 + 목 + 다음 관절의 집. 손목 1과 손목 2에 같은 부품을 쓴다 (0029).

좌표는 이 몸통이 품는 서보의 좌표다 (parts/joint.py). 앞 관절의 축은 x 방향이고 (y, z) = (AX_Y, 0)을 지난다.
발 바닥(앞 관절 출력 원판의 윗면)은 x = X_FOOT이다. 서보 먼 끝이 앞 관절 쪽을 본다.

첫 관절(0028: 실제 팔 부품으로 뽑아 맞춰 본다)은 이 몸통 둘로 조립한다. 하나는 집, 하나는 자식이다.

    uv run python -m horilab_design.parts.wrist
"""

from __future__ import annotations

import math

from build123d import Compound, Pos, Rot

from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_x, solid
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.values import ServoModel

HEAD = (
    2.6  # 서보 먼 끝과 앞 관절 출력 원판 사이: 혼 볼트 머리(1.65)와 선 두 가닥(두께 1.4)이 지나간다
)
X_FOOT = -(j.FAR + HEAD)  # 발 바닥
AX_Y = 3.68  # 앞 관절 축이 지나는 y: 뒷덮개 바깥 면 ~ 원통 앞 끝의 가운데
PIN_Z = j.R_PIN  # 영점 핀 귀 (+z 쪽)
PIN_POSE_DEG = -90.0  # 영점 핀을 꽂는 관절 각: 자식의 핀 귀가 부모의 살 위에 온다
NOTCH = (6.5, 11.5)  # 앞벽의 발 쪽 끝에 낸 선 홈의 |z| 범위


def _ear_outline(side: int, pin: bool) -> list[tuple[float, float]]:
    """발의 귀 윤곽 (y, z). 앞 관절 출력 원판 반지름 안, 목 옆면 바깥. 바닥 쪽(+y) 아랫변은 45°로 선다."""
    r, wall = j.R_DRUM, j.NECK_HW - 0.01
    half = math.sqrt(r * r - wall * wall)
    pts: list[tuple[float, float]] = []
    if not pin:
        pts.append((AX_Y + half, side * wall))
        u = 2.66  # 벽에서 45°로 올라가다 원호를 만나는 자리
        pts.append((AX_Y + half - u, side * (wall + u)))
        a0 = math.atan2(half - u, wall + u)
        a1 = -math.atan2(half, wall)
        pts += [(AX_Y + r * math.sin(a), side * r * math.cos(a)) for a in _steps(a0, a1, 24)]
        return pts
    # 핀 귀: 핀 둘레 원(반지름 4.4)과 원판을 잇고, 아랫변은 핀 원에서 벽까지 45°
    pr = 4.4
    t = pr / math.sqrt(2)
    pts.append((AX_Y + t + (PIN_Z + t - wall), side * wall))
    for a in _steps(math.radians(45.0), math.radians(-131.5), 12, first=True):
        pts.append((AX_Y + pr * math.sin(a), side * (PIN_Z + pr * math.cos(a))))
    phi = math.asin((r - pr) / PIN_Z)
    pts.append((AX_Y - r * math.cos(phi), side * r * math.sin(phi)))  # 원판 위쪽 접점
    for a in _steps(phi, math.atan2(wall, half), 8):
        pts.append((AX_Y - r * math.cos(a), side * r * math.sin(a)))
    return pts


def _steps(a0: float, a1: float, n: int, first: bool = False) -> list[float]:
    return [a0 + (a1 - a0) * k / n for k in range(0 if first else 1, n + 1)]


def wrist_body() -> Compound:
    """손목 몸통 (흰색). 원통 앞 끝(y = FRONT)을 바닥에 두고 뽑는다."""
    h = solid(j.drum_shell(X_FOOT) - j.drum_cavity(X_FOOT))
    # 발: 양옆 귀 (+z 쪽에 영점 핀 귀). 뽑을 때 선 판이다
    h = solid(h + j.prism_x(_ear_outline(1, True), X_FOOT, X_FOOT + j.EAR_T))
    h = solid(h + j.prism_x(_ear_outline(-1, False), X_FOOT, X_FOOT + j.EAR_T))
    for x, z in j.RIM:
        h = solid(h - cyl_x(j.M3_FREE_H / 2, X_FOOT - 1, X_FOOT + j.EAR_T + 1, AX_Y + x, z))
    h = solid(h - cyl_x(j.PIN_HOLE_H / 2, X_FOOT - 1, X_FOOT + j.EAR_T + 1, AX_Y, PIN_Z))
    # 선 홈: 앞벽의 발 쪽 끝 (앞 관절에서 들어오는 선, 다음 관절로 나가는 선)
    for sg in (1, -1):
        z0, z1 = (NOTCH[0], NOTCH[1]) if sg > 0 else (-NOTCH[1], -NOTCH[0])
        h = solid(h - box(X_FOOT - 1, X_FOOT + 2.4, j.NECK_FRONT_IN - 1, j.FRONT + 1, z0, z1))
    return j.cut_joint_features(h, window=False)


def wrist_cover() -> Compound:
    """손목 몸통의 뒷덮개 (검정). 발 쪽 끝은 발 바닥에서 끝난다."""
    return j.back_cover(X_FOOT, end_skirt=False)


def as_child(shape: Compound, q_deg: float = 0.0) -> Compound:
    """자식 손목 몸통의 좌표를 부모 관절의 서보 좌표로 옮긴다.

    자식의 x → 부모의 +y(출력축), 자식의 y → 부모의 +x (관절 각 0에서 자식의 앞이 부모의 가까운 끝 쪽), z → -z.
    자식의 발 바닥이 부모 출력 원판 윗면(ATOP)에 앉고, 자식의 (y, z) = (AX_Y, 0) 선이 부모 축이다.
    """
    placed = Pos(-AX_Y, j.ATOP - X_FOOT, 0) * (Rot(0, 180, 0) * (Rot(0, 0, 90) * shape))
    return j.turned(solid(placed), q_deg)


def child_hardware() -> list[j.Part]:
    """자식 몸통 나사 M3 × 10 넷과 영점 핀 (핀은 PIN_POSE_DEG에 꽂은 자리)."""
    out: list[j.Part] = []
    x = X_FOOT + j.EAR_T
    for k, (dy, z) in enumerate(j.RIM):
        s = cyl_x(1.5, x - j.BODY_SCREW, x, AX_Y + dy, z) + cyl_x(2.85, x, x + 1.65, AX_Y + dy, z)
        out.append(j.Part(f"몸통 나사 M3x10 {k + 1}", as_child(solid(s)), j.STEEL, True, "metal"))
    pin = cyl_x(1.5, x + 3.0 - 20.0, x + 3.0, AX_Y, PIN_Z)
    out.append(j.Part("영점 핀 MSH3-20", as_child(solid(pin), PIN_POSE_DEG), j.STEEL, False, "pin"))
    return out


def first_joint(model: ServoModel = "STS3215") -> list[j.Part]:
    """첫 관절 조립: 손목 몸통(부모, 집)과 속 부품, 출력 원판, 손목 몸통(자식), 나사와 핀."""
    body = wrist_body()
    parts = [j.Part("손목 몸통 (부모)", body, color="body")]
    parts += j.inner_parts(model, X_FOOT, end_skirt=False)
    parts.append(j.Part("손목 몸통 (자식)", as_child(body), moving=True, color="body"))
    return parts + j.inner_hardware(model) + child_hardware()


def clearances(model: ServoModel, q_deg: float, limit: float = 2.5) -> dict[tuple[str, str], float]:
    """관절 각 q에서 도는 부품과 서 있는 부품 사이 틈 [mm] (limit 아래만). 영점 핀은 뺀다."""
    parts = first_joint(model)
    static = [p for p in parts if not p.moving and p.name != "베어링 6806" and "핀" not in p.name]
    static = [p for p in static if not p.name.startswith("벽 나사")]
    out: dict[tuple[str, str], float] = {}
    for a in (p for p in parts if p.moving):
        name = a.name.rstrip("0123456789 ").strip()
        shape = j.turned(a.shape, q_deg)
        for b in static:
            if b.name == "서보 몸통" and (name.startswith("혼") or name.startswith("허브")):
                continue  # 혼과 혼 볼트 끝은 서보 앞면과 0.3 (서보 자체의 틈), 허브 판은 앞면 판과 1.7 ~ 1.8
            d = shape.distance_to(b.shape)
            if d < limit:
                out[(name, b.name)] = min(d, out.get((name, b.name), limit))
    return out


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋)
def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {
        "wrist_body": (wrist_body(), (0.0, -1.0, 0.0), "white", "strength"),
        "out_disc": (j.out_disc(), (0.0, -1.0, 0.0), "black", "strength"),
        "hub_plate": (j.hub_plate(), (0.0, -1.0, 0.0), "black", "strength"),
        "retainer": (j.retainer(), (0.0, -1.0, 0.0), "black", "strength"),
        "carrier": (j.carrier(), (0.0, 1.0, 0.0), "black", "strength"),
        "wrist_cover": (wrist_cover(), (0.0, 1.0, 0.0), "black", "standard"),
    }


def report(model: ServoModel = "STS3215") -> None:
    parts = first_joint(model)
    print(
        f"첫 관절 (손목 몸통 둘, {model}): 발 바닥 x {X_FOOT:.2f}, 앞 관절 축 y {AX_Y}, 귀 두께 {j.EAR_T}"
    )
    print(f"임시 값: {', '.join(j.PROVISIONAL)}")
    print("\n뽑는 부품 (무게는 뽑아서 잰 부품에 맞춘 어림, 걸침은 바닥에 닿지 않고 아래를 보는 면)")
    for name, (shape, up, color, preset) in printed().items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name:12s} {color:5s} {bb.size.X:5.1f} × {bb.size.Y:5.1f} × {bb.size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f}, "
            f"가장 넓은 한 자리 {over.largest:.0f})"
        )
    hits = j.overlaps([p for p in parts if "핀" not in p.name])
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    print("도는 부품과 서 있는 부품 사이 틈 [mm] (2.5 아래만)")
    for q in (0.0, 90.0, j.SOFT_DEG, j.STOP_DEG, -90.0, -j.SOFT_DEG, -j.STOP_DEG):
        gaps = sorted((d, a, b) for (a, b), d in clearances(model, q).items())
        print(f"  {q:7.1f}°: " + ", ".join(f"{a}↔{b} {d:.2f}" for d, a, b in gaps[:5]))
    body = wrist_body()
    pin = next(p for p in parts if "핀" in p.name)
    print(
        f"영점 핀 (관절 각 {PIN_POSE_DEG:.0f}°): 부모 몸통과 틈 {pin.shape.distance_to(body):.2f}, "
        f"자식 몸통과 틈 {pin.shape.distance_to(as_child(body, PIN_POSE_DEG)):.2f} (구멍 틈 0.15 ~ 0.18)"
    )


if __name__ == "__main__":
    report()
