"""손목 D405 받침: 손목 2 몸통(J6의 집) 옆의 돋움 둘에 다는 꺾인 판 (0018, 0029의 "D405 받침").

D405는 J6 원통 옆(-z 쪽, 영점 핀 귀의 반대쪽)에 있고, 광축을 도구 축 쪽으로 20° 기울인다 (0017). 카메라 뒷면의
M3 구멍 둘(간격 20 mm)에 받침의 뒤에서 나사를 조인다. 받침의 발은 손목 몸통의 돋움 둘(parts/wrist.py)에 M3 나사
둘로 단다. 커넥터가 있는 옆면은 J5 쪽(-x)을 본다.

카메라의 치수는 공식 사양서와 CAD에서 읽은 값이고(docs/hardware.md의 D405), 플러그의 폭과 두께는 USB 3.0 규격의
한계다. 플러그는 가진 기본 선의 곧은 플러그이고, 꺾임 보호의 길이는 가정이다 (PLUG_*: 가진 선을 재서 확인한다).

좌표는 joint.py의 서보 좌표다 (y가 J6 축, 플랜지 면이 y = ATOP, -x가 J5 쪽).

    uv run python -m horilab_design.parts.camera_mount
"""

from __future__ import annotations

import math

from build123d import Compound, Location, Plane

from horilab_design.parts import flange, wrist
from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_z, far_apart, solid
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.values import ServoModel

# ---- D405 (docs/hardware.md: 공식 사양서와 CAD) ----
CAM_W, CAM_H, CAM_D = 42.0, 42.0, 23.0  # 가로(커넥터가 있는 옆면까지) × 세로 × 깊이
CAM_HOLE_X = 10.0  # 뒷면의 M3 구멍 둘: 가로 가운데에서 ±10, 세로 가운데
CAM_HOLE_DEPTH = 4.0  # 나사가 들어갈 수 있는 깊이
CONN_BACK = 6.82  # 옆면 커넥터의 가운데: 뒷면에서
# ---- 플러그 (USB 3.0 규격 그림 5-17, docs/hardware.md의 D405. 가진 기본 선을 재서 확인한다) ----
PLUG_W, PLUG_T = 16.0, 8.5  # 플러그 몸통의 폭(카메라의 세로 방향)과 두께(깊이 방향): 규격의 한계
PLUG_LEN = 18.5  # 곧은 플러그의 몸통이 옆면에서 나온 길이: 규격의 참고 값
PLUG_RELIEF = 12.0  # 그 뒤에 선이 굽지 못하는 길이 (꺾임 보호. 규격에 없는 가정)

# ---- 카메라 자리 ----
TILT = 20.0  # 광축을 도구 축 쪽으로 기울인 각 [°] (0017)
DRUM_GAP = 1.0  # 카메라의 안쪽 앞 모서리와 원통 옆면 사이
# 렌즈 면의 가운데가 플랜지 면보다 뒤에 있는 거리. J6 멈춤 기둥의 나사 끝(이음매에서 8.0 아래, 축에서 26.5 ~ 29.5)이
# 카메라 앞을 2 mm 넘게 띄우고 지나가는 값이다 (8.0이면 0.9 mm)
LENS_BACK = 9.2
_T = math.radians(TILT)
AXIS = (0.0, math.cos(_T), math.sin(_T))  # 광축 (도구 쪽, J6 축 쪽으로 기운다)
INWARD = (0.0, -math.sin(_T), math.cos(_T))  # 카메라의 세로 방향 가운데 J6 축을 보는 쪽
LENS = (
    0.0,
    j.ATOP - LENS_BACK,
    -(j.R_DRUM + DRUM_GAP + CAM_H / 2 * math.cos(_T)),
)  # 렌즈 면의 가운데
# 카메라 좌표: 원점이 렌즈 면 가운데, x가 가로(+x), y가 세로 바깥쪽(J6 축에서 멀어지는 쪽), z가 광축
FRAME = Location(Plane(origin=LENS, x_dir=(1.0, 0.0, 0.0), z_dir=AXIS))

# ---- 받침 ----
# 카메라 뒤의 판. 카메라 무게에 판이 휘어 카메라가 기우는 양이 화소 반 칸(1280 화소에서 0.034°)보다 작은 두께다
# (tilt_under_weight). 3.5면 0.05°다
PLATE_T = 4.5
MOUNT_W = (
    34.0  # 받침의 폭 (카메라 나사 ±10과 발 나사 ±12를 품는다. 카메라 폭 42보다 좁혀 무게를 줄였다)
)
HEAD_REC = 1.5  # 카메라 나사 머리 자리의 깊이: 남은 살 3.0에 M3 × 6이 카메라에 3.0 들어간다
CAM_SCREW = 6.0
FOOT_T = 3.0  # 돋움에 앉는 발
FOOT_Y0 = wrist.MOUNT_Y[0] - 0.5  # 발의 뒤 끝
FOOT_SCREW = 8.0  # M3 × 8: 발 3.0 + 돋움 속 5.0 (너트를 0.6 지나간다)
FILLET = 1.5  # 발과 판이 만나는 안쪽 모서리를 메운 길이 (발 나사 머리와 0.5 뜬다)
PROVISIONAL = (
    "PLUG_* (가진 D405 선의 플러그)",
    "wrist.MOUNT_NUT (흰색 fit1_nut_drops)",
    *j.PROVISIONAL[:1],
)


def _yz(p: tuple[float, float, float]) -> tuple[float, float]:
    return (p[1], p[2])


def _back_edge() -> tuple[float, float]:
    """카메라 뒷면의 안쪽 모서리 (y, z)."""
    return (
        LENS[1] - CAM_D * AXIS[1] + CAM_H / 2 * INWARD[1],
        LENS[2] - CAM_D * AXIS[2] + CAM_H / 2 * INWARD[2],
    )


def camera() -> Compound:
    """D405 몸통 (틈 검사와 그림에 쓴다)."""
    body = box(-CAM_W / 2, CAM_W / 2, -CAM_H / 2, CAM_H / 2, -CAM_D, 0.0)
    return solid(body.moved(FRAME))


def plug(length: float = PLUG_LEN + PLUG_RELIEF) -> Compound:
    """커넥터에 꽂은 플러그가 차지하는 자리 (-x 쪽 옆면에서 length만큼)."""
    z = -CAM_D + CONN_BACK
    body = box(
        -CAM_W / 2 - length, -CAM_W / 2, -PLUG_W / 2, PLUG_W / 2, z - PLUG_T / 2, z + PLUG_T / 2
    )
    return solid(body.moved(FRAME))


def profile() -> list[tuple[float, float]]:
    """받침의 꺾인 단면 (y, z). 발의 안쪽 뒤 모서리에서 시작해 카메라 뒷면을 따라 바깥 끝까지 갔다가 돌아온다:
    발 안쪽 뒤, 판 앞면이 돋움 면과 만나는 곳, 판 앞면의 바깥 끝, 판 뒷면의 바깥 끝, 메운 모서리 둘, 발 바깥 뒤."""
    ay, az = AXIS[1], AXIS[2]
    oy, oz = -INWARD[1], -INWARD[2]  # 판을 따라 바깥으로 가는 방향 (y, z)
    by, bz = _back_edge()
    z_in, z_out = -wrist.MOUNT_Z, -(wrist.MOUNT_Z + FOOT_T)
    s_in = (z_in - bz) / (-oz)  # 카메라 뒷면이 돋움 면과 만나는 곳까지 (안쪽으로)
    front_root = (by - s_in * oy, z_in)
    front_end = (by + CAM_H * oy, bz + CAM_H * oz)
    back_end = (front_end[0] - PLATE_T * ay, front_end[1] - PLATE_T * az)
    qy, qz = by - PLATE_T * ay, bz - PLATE_T * az  # 판 뒷면 위의 한 점
    s_out = (z_out - qz) / (-oz)
    back_root = (qy - s_out * oy, z_out)
    return [
        (FOOT_Y0, z_in),
        front_root,
        front_end,
        back_end,
        (back_root[0] + 2 * FILLET * oy, back_root[1] + 2 * FILLET * oz),
        (back_root[0] - FILLET, z_out),
        (FOOT_Y0, z_out),
    ]


def mount() -> Compound:
    """D405 받침 (검정). 옆으로 눕혀(x가 위) 뽑는다: 꺾인 단면이 바닥에 그려져 각도가 정확하고, 층이 판의 길이
    방향으로 이어져 카메라 무게가 층을 떼는 쪽으로 걸리지 않는다."""
    z_in, z_out = -wrist.MOUNT_Z, -(wrist.MOUNT_Z + FOOT_T)
    m = j.prism_x(profile(), -MOUNT_W / 2, MOUNT_W / 2)
    for x, y in wrist.mount_xy():  # 발 나사
        m = solid(m - cyl_z(j.M3_FREE_H / 2, z_out - 1, z_in + 1, x, y))
    r = j.M3_FREE_H / 2
    for sx in (1, -1):  # 카메라 나사와 머리 자리 (카메라 좌표)
        hole = cyl_z(r, -CAM_D - PLATE_T - 1, -CAM_D + 1, sx * CAM_HOLE_X, 0.0)
        hole += cyl_z(3.1, -CAM_D - PLATE_T - 1, -CAM_D - PLATE_T + HEAD_REC, sx * CAM_HOLE_X, 0.0)
        m = solid(m - solid(hole).moved(FRAME))
    if not m.is_valid:
        raise RuntimeError("D405 받침 형상이 깨졌다")
    return m


def hardware() -> list[j.Part]:
    """카메라 나사 M3 × 6 둘, 발 나사 M3 × 8 둘과 돋움 속 너트."""
    out: list[j.Part] = []
    seat = -CAM_D - PLATE_T + HEAD_REC  # 카메라 좌표에서 머리 밑면
    for k, sx in enumerate((1, -1), start=1):
        s = cyl_z(1.5, seat, seat + CAM_SCREW, sx * CAM_HOLE_X, 0.0)
        s += cyl_z(2.85, seat - 1.65, seat, sx * CAM_HOLE_X, 0.0)
        out.append(
            j.Part(f"카메라 나사 M3x6 {k}", solid(solid(s).moved(FRAME)), j.STEEL, False, "metal")
        )
    z_out = -(wrist.MOUNT_Z + FOOT_T)
    roof = wrist.MOUNT_NUT[2]
    for k, (x, y) in enumerate(wrist.mount_xy(), start=1):
        s = cyl_z(1.5, z_out, z_out + FOOT_SCREW, x, y) + cyl_z(2.85, z_out - 1.65, z_out, x, y)
        out.append(j.Part(f"받침 나사 M3x8 {k}", solid(s), j.STEEL, False, "metal"))
        z0 = -(wrist.MOUNT_Z - roof)
        nut = j.hex_z(x, y, z0, z0 + j.NUT_T, 5.4) - cyl_z(1.5, z0 - 1, z0 + j.NUT_T + 1, x, y)
        out.append(j.Part(f"너트 (돋움) {k}", solid(nut), j.STEEL, False, "metal"))
    return out


def assembly(model: ServoModel = "STS3215") -> list[j.Part]:
    """J6 조립(parts/flange.py)에 D405와 받침을 더한 것. 플러그는 가진 선의 곧은 플러그다 (길이는 가정)."""
    parts = flange.assembly(model)
    parts += [
        j.Part("D405 받침", mount(), color="cap"),
        j.Part("D405", camera(), color="servo"),
        j.Part("D405 플러그", plug(), color="plug"),
    ]
    return parts + hardware()


def screw_checks() -> dict[str, float]:
    """나사가 물린 길이와 여유 [mm]."""
    roof = wrist.MOUNT_NUT[2]
    foot_tip = FOOT_SCREW - FOOT_T  # 돋움 면에서 나사 끝까지
    return {
        "카메라 나사가 카메라에 들어간 길이": CAM_SCREW - (PLATE_T - HEAD_REC),
        "카메라 나사 끝과 구멍 바닥 사이": CAM_HOLE_DEPTH - (CAM_SCREW - (PLATE_T - HEAD_REC)),
        "받침 나사가 너트(2.4)에 물린 길이": min(foot_tip - roof, j.NUT_T),
        "받침 나사 끝과 나사 구멍 바닥 사이": wrist.MOUNT_HOLE - foot_tip,
        "돋움 나사 구멍 바닥과 원통 옆면 사이": math.hypot(
            wrist.MOUNT_X, wrist.MOUNT_Z - wrist.MOUNT_HOLE
        )
        - j.R_DRUM,
    }


CAM_LOAD_N = 0.75  # 카메라 58 ~ 60 g에 플러그와 선의 몫 15 g을 더한 무게 [N]
PIXEL_DEG = 87.0 / 1280  # 가장 촘촘한 영상(가로 1280 화소)의 한 화소


def tilt_under_weight(e_gpa: float = 2.0) -> float:
    """카메라 무게가 판에 직각으로 걸릴 때 카메라가 기우는 각 [°]. 판을 발에서 카메라 나사까지 뻗은 외팔보로 본다.

    탄성 계수는 PLA의 낮은 쪽 값(2.0 GPa)을 기본으로 쓴다 (0029의 허브 판 계산과 같은 범위 2.0 ~ 2.75).
    """
    bz = _back_edge()[1]
    root = (-wrist.MOUNT_Z - bz) / math.cos(_T)  # 카메라 안쪽 모서리에서 발까지 (판을 따라)
    lever = CAM_H / 2 + root
    inertia = MOUNT_W * PLATE_T**3 / 12
    return math.degrees(CAM_LOAD_N * lever**2 / (2 * e_gpa * 1000 * inertia))


def sweep_gaps(parts: list[j.Part]) -> dict[str, float]:
    """J6를 멈춤 각까지 돌릴 때, 도는 부품과 D405·받침·플러그 사이의 가장 좁은 틈 [mm]."""
    fixed = [p for p in parts if p.name in ("D405", "D405 받침", "D405 플러그")]
    moving = [p for p in parts if p.moving]
    out: dict[str, float] = {}
    for q in range(-165, 166, 15):
        for a in moving:
            shape = j.turned(a.shape, float(max(-j.STOP_DEG, min(j.STOP_DEG, q))))
            for b in fixed:
                key = f"{a.name.rstrip('0123456789 ').strip()} ↔ {b.name}"
                if far_apart(shape, b.shape, out.get(key, 6.0)):
                    continue
                d = shape.distance_to(b.shape)
                if d < out.get(key, 6.0):
                    out[key] = d
    return out


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋)
def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {"d405_mount": (mount(), (1.0, 0.0, 0.0), "black", "strength")}


def report(model: ServoModel = "STS3215") -> None:
    parts = assembly(model)
    by, bz = _back_edge()
    print(
        f"D405 받침 ({model}): 렌즈 면 가운데 y {LENS[1]:.1f} (플랜지 면보다 {LENS_BACK} 뒤), 축에서 {-LENS[2]:.1f}, "
        f"기울기 {TILT:.0f}°, 카메라 뒷면 안쪽 모서리 (y {by:.1f}, z {bz:.1f})"
    )
    cam = next(p for p in parts if p.name == "D405")
    bb = cam.shape.bounding_box()
    print(
        f"카메라가 차지하는 범위: y {bb.min.Y:.1f} ~ {bb.max.Y:.1f}, 축에서 {-bb.max.Z:.1f} ~ {-bb.min.Z:.1f} mm"
    )
    print(f"임시 값: {', '.join(PROVISIONAL)}")
    print("\n뽑는 부품")
    for name, (shape, up, color, preset) in printed().items():
        size = shape.bounding_box().size
        over = overhangs(shape, up)
        print(
            f"  {name:11s} {color:5s} {size.X:5.1f} × {size.Y:5.1f} × {size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f})"
        )
    for name, value in screw_checks().items():
        print(f"  {name}: {value:.2f}")
    print(
        f"  카메라 무게에 기우는 각: {tilt_under_weight(2.75):.3f} ~ {tilt_under_weight(2.0):.3f}° "
        f"(화소 반 칸 {PIXEL_DEG / 2:.3f}°)"
    )
    solids = [p for p in parts if "핀" not in p.name and "손잡이" not in p.name]
    hits = j.overlaps(solids)
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    body = next(p for p in parts if p.name == "손목 몸통")
    print(f"카메라와 손목 몸통 사이 {cam.shape.distance_to(body.shape):.2f} mm")
    print("J6를 돌릴 때 도는 부품과의 가장 좁은 틈 [mm] (6 아래만)")
    for key, d in sorted(sweep_gaps(parts).items(), key=lambda kv: kv[1]):
        print(f"  {key}: {d:.2f}")


if __name__ == "__main__":
    report()
