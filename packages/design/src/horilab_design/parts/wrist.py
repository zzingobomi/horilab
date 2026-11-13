"""손목 몸통: 앞 관절의 출력 원판에 앉는 발 + 목 + 다음 관절의 집. 손목 1과 손목 2에 같은 부품을 쓴다 (0029).

좌표는 이 몸통이 품는 서보의 좌표다 (parts/joint.py). 앞 관절의 축은 x 방향이고 (y, z) = (AX_Y, 0)을 지난다.
발 바닥(앞 관절 출력 원판의 윗면)은 x = X_FOOT이다. 서보 먼 끝이 앞 관절 쪽을 본다.

첫 관절(0028: 실제 팔 부품으로 뽑아 맞춰 본다)은 이 몸통 둘로 조립한다. 하나는 집, 하나는 자식이다.

    uv run python -m horilab_design.parts.wrist
"""

from __future__ import annotations

import math

from build123d import Compound, Location, Pos, Rot

from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_x, cyl_y, cyl_z, nut_trap, solid
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.values import ServoModel

HEAD = (
    2.6  # 서보 먼 끝과 앞 관절 출력 원판 사이: 혼 볼트 머리(1.65)와 선 두 가닥(두께 1.4)이 지나간다
)
X_FOOT = -(j.FAR + HEAD)  # 발 바닥
AX_Y = 3.68  # 앞 관절 축이 지나는 y: 뒷덮개 바깥 면 ~ 원통 앞 끝의 가운데
PIN_Z = j.R_PIN  # 영점 핀 귀 (+z 쪽)
PIN_POSE_DEG = -90.0  # 영점 핀을 꽂는 관절 각: 자식의 핀 귀가 부모의 살 위에 온다
# 앞벽의 발 쪽 끝에 낸 선 홈 (0031). 가운데 홈은 다음 관절로 나가는 선이 살 위로 나가는 자리다: 멈춤 기둥과 귀가
# 지나가지 않는 폭(멈춤 각에서 살 가운데 ± 2.5 mm) 안에 둔다. 옆 홈 둘은 앞 관절에서 들어오는 선의 자리다
# (이 몸통의 앞이 앞 관절 원판의 살 쪽에 올 때: 손목 1). 손목 2는 들어오는 선이 뒷덮개 쪽 모서리로 들어온다
NOTCH = (6.5, 11.5)  # 옆 홈의 |z| 범위
EXIT_HW = 2.6  # 가운데 홈의 반폭 (선 폭 3.9에 여유)
SIDE_LANE = 10.0  # 서보 먼 끝 밑(앞 관절 원판 위)에서 선이 지나는 |z|: 혼 볼트 머리(7.8까지)와 목 안쪽 벽(12.7) 사이
# 원통 옆(-z 쪽, 영점 핀 귀의 반대쪽)의 돋움 둘: 손목 2에서는 D405 받침(parts/camera_mount.py)을, 손목 1에서는
# D405 선의 클립을 M3 나사 둘로 단다. 너트는 뒤(-y, 뽑을 때 위)에서 떨어뜨려 넣는다
MOUNT_X = 12.0  # 돋움 가운데의 |x|
MOUNT_HW = 4.5  # 돋움의 반폭
MOUNT_Y = (-17.0, -7.0)  # 돋움의 y 범위: 뒷덮개 테(-17.8)와 서보를 잠그는 벽 나사(-2) 사이
MOUNT_Z = 27.6  # 돋움 면의 |z| (원통 옆면에서 4.0)
MOUNT_SCREW_Y = -12.0
MOUNT_NUT = (
    5.6,
    2.8,
    1.6,
)  # 너트 홈의 폭과 두께, 돋움 면에서 홈까지의 살 (임시: 흰색 fit1_nut_drops 값으로 바꾼다)
MOUNT_HOLE = 6.0  # 나사 구멍의 깊이 (돋움 면에서. 원통 벽에는 닿지 않는다)


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


def mount_xy() -> list[tuple[float, float]]:
    """돋움 둘의 나사 자리 (x, y)."""
    return [(MOUNT_X, MOUNT_SCREW_Y), (-MOUNT_X, MOUNT_SCREW_Y)]


def _mount_bosses() -> Compound:
    """원통 옆의 돋움 둘. 원통 밖에만 살을 더한다. 뽑을 때 아래를 보는 면(+y 쪽)은 45°보다 조금 서게 깎는다."""
    y0, y1 = MOUNT_Y
    deep = 15.0  # 원통 속까지 넉넉히 그린 뒤 원통 바깥 면으로 자른다
    ramp = 1.1 * (MOUNT_Z - deep)
    out: Compound | None = None
    for x, _y in mount_xy():
        b = box(x - MOUNT_HW, x + MOUNT_HW, y0, y1, -MOUNT_Z, -deep)
        b += j.prism_x(
            [(y1, -MOUNT_Z), (y1 + ramp, -deep), (y1, -deep)], x - MOUNT_HW, x + MOUNT_HW
        )
        out = solid(b) if out is None else solid(out + b)
    assert out is not None
    return solid(out - cyl_y(j.R_DRUM - 0.01, y0 - 1, y1 + ramp + 1))


def _mount_cuts() -> Compound:
    """돋움의 너트 홈(뒤에서 떨어뜨려 넣는다)과 나사 구멍."""
    w, t, roof = MOUNT_NUT
    z_nut = -(MOUNT_Z - roof - t / 2)
    out: Compound | None = None
    for x, y in mount_xy():
        trap = nut_trap((x, y, z_nut), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), y - MOUNT_Y[0] + 1, w, t)
        cut = trap + cyl_z(j.M3_FREE_H / 2, -MOUNT_Z - 1, -MOUNT_Z + MOUNT_HOLE, x, y)
        out = solid(cut) if out is None else solid(out + cut)
    assert out is not None
    return out


def wrist_body() -> Compound:
    """손목 몸통 (흰색). 원통 앞 끝(y = FRONT)을 바닥에 두고 뽑는다."""
    h = solid(j.drum_shell(X_FOOT) - j.drum_cavity(X_FOOT))
    h = solid(solid(h + _mount_bosses()) - _mount_cuts())
    # 발: 양옆 귀 (+z 쪽에 영점 핀 귀). 뽑을 때 선 판이다
    h = solid(h + j.prism_x(_ear_outline(1, True), X_FOOT, X_FOOT + j.EAR_T))
    h = solid(h + j.prism_x(_ear_outline(-1, False), X_FOOT, X_FOOT + j.EAR_T))
    for x, z in j.RIM:
        h = solid(h - cyl_x(j.M3_FREE_H / 2, X_FOOT - 1, X_FOOT + j.EAR_T + 1, AX_Y + x, z))
    h = solid(h - cyl_x(j.PIN_HOLE_H / 2, X_FOOT - 1, X_FOOT + j.EAR_T + 1, AX_Y, PIN_Z))
    # 선 홈: 앞벽의 발 쪽 끝. 가운데는 다음 관절로 나가는 선, 양옆은 앞 관절에서 들어오는 선
    spans = [(-EXIT_HW, EXIT_HW), (NOTCH[0], NOTCH[1]), (-NOTCH[1], -NOTCH[0])]
    for z0, z1 in spans:
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
    return j.turned(solid(_child_at() * shape), q_deg)


def _child_at() -> Location:
    """자식 손목 몸통의 좌표를 부모 관절의 서보 좌표로 옮기는 자리 (관절 각 0)."""
    return Pos(-AX_Y, j.ATOP - X_FOOT, 0) * (Rot(0, 180, 0) * Rot(0, 0, 90))


def parent_disc(model: ServoModel = "STS3215") -> list[j.Part]:
    """이 몸통이 앉은 앞 관절의 출력 원판과 멈춤 나사, 너트를 이 몸통의 서보 좌표로 옮긴 것 (이 몸통과 함께 서 있다).

    앞 관절의 멈춤 귀는 이 몸통의 출력 쪽(+y)을 보고, 그 끝이 이 몸통의 출력 원판 윗면과 같은 높이까지 온다
    (손목 2에 단 도구가 J6를 따라 돌 때 그 위를 지나간다). J4의 원판은 손목 1에 반 바퀴 돌려 달아서 귀가 반대쪽이다.
    """
    back = _child_at().inverse()
    parts = [j.Part("앞 관절의 출력 원판", solid(back * j.out_disc()), color="cap")]
    for p in j.inner_hardware(model):
        if p.name.startswith(("멈춤 나사", "너트 (멈춤")):
            parts.append(
                j.Part(f"앞 관절의 {p.name}", solid(back * p.shape), j.STEEL, False, "metal")
            )
    return parts


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
