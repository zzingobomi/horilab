"""J6의 플랜지 원판과 도구 어댑터 판 (docs/design.md의 "그리퍼 교체 구조", 0029의 "플랜지").

플랜지 원판은 STS 관절의 출력 원판(parts/joint.py)에 둘을 더한 것이다:
- 도구 자리를 잡는 강철 핀 둘 (MSH3-20, 눌러 박는다. 윗면 위로 6 mm 나온다).
- J6의 영점 핀 귀. 다른 관절은 영점 핀 구멍이 자식 몸통에 있지만, J6의 자식은 갈아 끼우는 도구라서 원판에 둔다.

도구는 어댑터 판(두께 6.2 mm)으로 플랜지 윗면에 앉는다. 핀 둘(±z)이 자리와 돌림을 잡고(하나는 둥근 구멍,
하나는 핀 사이 방향으로 긴 구멍), 손나사 둘이 원판 밑면의 너트에 물린다. 기울기는 맞닿은 두 면이 정한다.
어댑터 판은 도구마다 그 도구의 몸통과 한 부품으로 뽑는다 (여기서는 판만 그린다. 구멍은 adapter_cuts가 낸다).

도구의 방향: 손가락이 벌어지는 쪽을 멈춤 기둥의 귀(0°)와 영점 핀 귀(180°)를 잇는 방향(x)으로 둔다. J6 각이 0°
근처일 때 두 귀가 D405(옆, ±90°)의 화면에 들지 않는다 (0017: 물체를 볼 때는 손가락이 벌어지는 방향을 카메라
쪽과 직각으로 둔다). 도구 핀 둘은 그와 직각인 z 위에 있다.

도구가 비킬 자리: 손목 2 몸통이 앉은 J5 출력 원판의 멈춤 귀는 도구 쪽(+y)을 보고, 그 끝이 플랜지 면과
같은 높이까지 온다 (축에서 -x 쪽으로 37.7 ~ 41.5 mm, parts/wrist.py의 parent_disc). J6에 단 것은 그 위를 돌며
지나가므로, 도구의 밑이 축에서 35 mm 밖으로 나올 때는 그 귀와 2 mm 넘게 띄운다.

좌표는 joint.py의 서보 좌표다 (y가 J6 축, 원판 윗면이 y = ATOP).

    uv run python -m horilab_design.parts.flange
"""

from __future__ import annotations

from build123d import Compound, Rot

from horilab_design.parts import joint as j
from horilab_design.parts import wrist
from horilab_design.parts.shapes import box, cyl_y, solid
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.values import ServoModel

TOOL_PIN_R = 11.5  # 도구 핀 둘이 축에서 떨어진 거리. 핀 구멍이 혼 볼트 머리와 3.7, 저널 바깥 면과 1.8 떨어진다
# 핀 둘은 ±z에 둔다 (멈춤 기둥 귀와 영점 핀 귀를 잇는 x와 직각). 평행 그리퍼는 서보 선을 꽂을 틈을 어댑터 판의
# -x 쪽에 내는데, 핀이 x 위에 있으면 그 틈과 겹친다 (parts/gripper.py)
TOOL_PINS = (
    (0.0, TOOL_PIN_R),
    (0.0, -TOOL_PIN_R),
)  # (x, z): 앞의 것이 둥근 구멍, 뒤의 것이 긴 구멍
TOOL_PIN_DEPTH = 14.0  # 핀을 박는 깊이 (테 3.8 + 단 3.8 + 저널 속 6.4)
TOOL_PIN_UP = 20.0 - TOOL_PIN_DEPTH  # 윗면 위로 나온 길이 6.0 (어댑터 판 윗면보다 0.2 낮다)
PIN_PRESS = 3.30  # 핀을 눌러 박는 세로 구멍 (임시: 흰색에서 뻑뻑했던 값. 검정 시험편 값으로 바꾼다)
EAR_HW = 4.4  # 영점 핀 귀의 반폭
ADAPTER_R = 22.0  # 어댑터 판의 반지름 (원판 23.6 안)
ADAPTER_T = j.EAR_T  # 6.2: 손나사 M3 × 10이 원판 밑면의 너트에 다 물리는 두께
SLOT_HALF = 0.5  # 긴 구멍이 핀 사이 방향으로 늘어난 반 길이 (두 핀 사이 거리의 출력 오차)
HEAD_POCKET = (3.2, 2.0)  # 어댑터 밑면에서 혼 볼트 머리를 비키는 자리: 반지름, 깊이
TOOL_SCREWS = (
    j.RIM[0],
    j.RIM[3],
)  # 손나사 둘: 원판의 너트 자리 넷 중 마주 보는 둘 (5, 19.4), (-5, -19.4)
KNOB = (6.0, 7.0)  # 손나사 머리에 씌우는 손잡이: 반지름, 높이
PROVISIONAL = ("PIN_PRESS (검정 핀 구멍 시험편)", *j.PROVISIONAL)


def flange_disc() -> Compound:
    """플랜지 원판 (검정). 테 윗면을 바닥에 두고 뽑는다. 핀 구멍 셋은 세로 구멍이다."""
    a = j.out_disc()
    ear = box(-j.R_PIN, -20.0, j.SEAM, j.ATOP, -EAR_HW, EAR_HW)
    ear += cyl_y(EAR_HW, j.SEAM, j.ATOP, -j.R_PIN, 0.0)
    a = solid(a + ear)
    a = solid(a - cyl_y(j.PIN_HOLE / 2, j.SEAM - 1, j.ATOP + 1, -j.R_PIN, 0.0))
    for x, z in TOOL_PINS:
        a = solid(a - cyl_y(PIN_PRESS / 2, j.ATOP - TOOL_PIN_DEPTH, j.ATOP + 1, x, z))
    if not a.is_valid:
        raise RuntimeError("플랜지 원판 형상이 깨졌다")
    return a


def adapter_cuts(t: Compound) -> Compound:
    """도구 쪽 판(두께 ADAPTER_T)에 핀 구멍 둘, 손나사 구멍 둘, 혼 볼트 머리 자리를 낸다.

    도구마다 판의 윤곽이 달라서(평행 그리퍼는 판이 몸통의 윗부분이다) 구멍만 따로 낸다.
    """
    y0, y1 = j.ATOP, j.ATOP + ADAPTER_T
    r = j.PIN_HOLE / 2
    (x0, z0), (x1, z1) = TOOL_PINS
    t = solid(t - cyl_y(r, y0 - 1, y1 + 1, x0, z0))  # 둥근 구멍: 자리를 잡는다
    slot = cyl_y(r, y0 - 1, y1 + 1, x1, z1 - SLOT_HALF) + cyl_y(
        r, y0 - 1, y1 + 1, x1, z1 + SLOT_HALF
    )
    slot += box(x1 - r, x1 + r, y0 - 1, y1 + 1, z1 - SLOT_HALF, z1 + SLOT_HALF)
    t = solid(t - solid(slot))  # 긴 구멍: 돌림만 잡는다
    for x, z in TOOL_SCREWS:
        t = solid(t - cyl_y(j.M3_FREE / 2, y0 - 1, y1 + 1, x, z))
    for x, z in j.bolt_xz():
        t = solid(t - cyl_y(HEAD_POCKET[0], y0 - 1, y0 + HEAD_POCKET[1], x, z))
    return t


def tool_adapter() -> Compound:
    """도구 어댑터 판 (도구 몸통의 밑부분). 플랜지에 닿는 밑면을 바닥에 두고 뽑는다."""
    disc = solid(Rot(0, 45, 0) * cyl_y(ADAPTER_R, j.ATOP, j.ATOP + ADAPTER_T))
    return adapter_cuts(disc)


# ------------------------------------------------------------------------------------------------
# 조립 검사


def hardware() -> list[j.Part]:
    """도구 핀 둘, 손나사 둘과 손잡이, J6 영점 핀."""
    out: list[j.Part] = []
    for k, (x, z) in enumerate(TOOL_PINS, start=1):
        pin = cyl_y(1.5, j.ATOP - TOOL_PIN_DEPTH, j.ATOP + TOOL_PIN_UP, x, z)
        out.append(j.Part(f"도구 핀 MSH3-20 {k}", solid(pin), j.STEEL, True, "pin"))
    top = j.ATOP + ADAPTER_T
    for k, (x, z) in enumerate(TOOL_SCREWS, start=1):
        screw = cyl_y(1.5, top - j.BODY_SCREW, top, x, z) + cyl_y(2.85, top, top + 1.65, x, z)
        out.append(j.Part(f"손나사 M3x10 {k}", solid(screw), j.STEEL, True, "metal"))
        knob = cyl_y(KNOB[0], top + 0.3, top + KNOB[1], x, z)
        out.append(j.Part(f"손잡이 {k}", solid(knob), j.PLA, True, "cap"))
    # 영점 핀: 귀(3.8)와 틈(2.0)을 지나 살의 구멍 바닥에 닿는다. 귀 위로 4.6 나온다
    bottom = j.TUB_Y - 1.0
    pin = cyl_y(1.5, bottom, bottom + 20.0, -j.R_PIN, 0.0)
    out.append(j.Part("영점 핀 MSH3-20", solid(pin), j.STEEL, False, "pin"))
    return out


def assembly(model: ServoModel = "STS3215") -> list[j.Part]:
    """J6 조립: 손목 몸통(손목 2)과 속 부품, 플랜지 원판, 도구 어댑터 판, 핀과 손나사."""
    parts = [j.Part("손목 몸통", wrist.wrist_body(), color="body")]
    for p in j.inner_parts(model, wrist.X_FOOT, end_skirt=False):
        if p.name == "출력 원판":
            p = j.Part("플랜지 원판", flange_disc(), moving=True, color="cap")
        parts.append(p)
    parts.append(j.Part("도구 어댑터 판", tool_adapter(), moving=True, color="frame"))
    # 원판의 너트 넷 중 손나사가 쓰는 둘만 조립 그림에 남긴다
    used = {f"너트 (출력 원판) {k + 1}" for k, xz in enumerate(j.RIM) if xz in TOOL_SCREWS}
    inner = [
        p
        for p in j.inner_hardware(model)
        if not p.name.startswith("너트 (출력 원판)") or p.name in used
    ]
    return parts + inner + hardware()


def clearances(
    parts: list[j.Part], q_deg: float, limit: float = 2.5
) -> dict[tuple[str, str], float]:
    """J6 각 q에서 도는 부품과 서 있는 부품 사이 틈 [mm] (limit 아래만). 영점 핀, 벽 나사, 베어링은 뺀다."""
    static = [
        p
        for p in parts
        if not p.moving
        and "베어링" not in p.name
        and "영점 핀" not in p.name
        and "벽 나사" not in p.name
    ]
    out: dict[tuple[str, str], float] = {}
    for a in (p for p in parts if p.moving):
        name = a.name.rstrip("0123456789 ").strip()
        shape = j.turned(a.shape, q_deg)
        for b in static:
            if b.name == "서보 몸통" and (name.startswith("혼") or name.startswith("허브")):
                continue  # 혼과 혼 볼트 끝, 허브 판은 서보 앞면과 가깝다 (parts/wrist.py와 같다)
            d = shape.distance_to(b.shape)
            if d < limit:
                out[(name, b.name)] = min(d, out.get((name, b.name), limit))
    return out


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋)
def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {
        "flange_disc": (flange_disc(), (0.0, -1.0, 0.0), "black", "strength"),
        "tool_adapter": (tool_adapter(), (0.0, 1.0, 0.0), "black", "strength"),
    }


def report(model: ServoModel = "STS3215") -> None:
    parts = assembly(model)
    print(
        f"J6 플랜지 ({model}): 플랜지 면 y {j.ATOP:.1f}, 어댑터 판 윗면 {j.ATOP + ADAPTER_T:.1f}, "
        f"도구 핀 z ±{TOOL_PIN_R} (윗면 위로 {TOOL_PIN_UP:.1f}), 영점 핀 귀 축에서 {j.R_PIN:.0f}"
    )
    print(f"임시 값: {', '.join(PROVISIONAL)}")
    print("\n뽑는 부품 (무게는 뽑아서 잰 부품에 맞춘 어림, 걸침은 바닥에 닿지 않고 아래를 보는 면)")
    for name, (shape, up, color, preset) in printed().items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name:13s} {color:5s} {bb.size.X:5.1f} × {bb.size.Y:5.1f} × {bb.size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f}, "
            f"가장 넓은 한 자리 {over.largest:.0f})"
        )
    hits = j.overlaps([p for p in parts if "핀" not in p.name and "손잡이" not in p.name])
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    print("도는 부품과 서 있는 부품 사이 틈 [mm] (2.5 아래만)")
    for q in (0.0, 90.0, j.SOFT_DEG, j.STOP_DEG, -90.0, -j.SOFT_DEG, -j.STOP_DEG):
        gaps = sorted((d, a, b) for (a, b), d in clearances(parts, q).items())
        print(f"  {q:7.1f}°: " + ", ".join(f"{a}↔{b} {d:.2f}" for d, a, b in gaps[:5]))
    body = next(p for p in parts if p.name == "손목 몸통")
    disc = next(p for p in parts if p.name == "플랜지 원판")
    plate = next(p for p in parts if p.name == "도구 어댑터 판")
    pin = next(p for p in parts if p.name.startswith("영점 핀"))
    print(
        f"영점 핀 (J6 각 0°): 손목 몸통과 틈 {pin.shape.distance_to(body.shape):.2f}, "
        f"플랜지 원판과 틈 {pin.shape.distance_to(disc.shape):.2f} (구멍 틈 0.15 ~ 0.18)"
    )
    for p in parts:
        if p.name.startswith("도구 핀"):
            print(
                f"{p.name}: 원판과 틈 {p.shape.distance_to(disc.shape):.2f} (눌러 박는 구멍), "
                f"어댑터 판과 틈 {p.shape.distance_to(plate.shape):.2f}"
            )


if __name__ == "__main__":
    report()
