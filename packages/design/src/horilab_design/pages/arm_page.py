"""팔 전체 3D 페이지: 책상 위의 팔, 박스 쌓기 동작, 관절 직접 움직이기.

틀 arm_page.html의 __DATA__ 자리에 데이터를 넣는다. 책상, 박스, 박스 쌓기의 TCP 자리는 바탕 데이터
(arm_page_base.json)에 있고, 팔과 D405, 도구(평행 그리퍼)의 형상과 관절 각은 검토 모델(studies/ur_arm.py)에서
뽑는다. 바탕 데이터의 TCP 자리는 박스를 잡을 때 박스 윗면에서 4.5 mm 아래다 (손가락 어깨가 박스 윗면에 닿는다).

형상(prims)은 링크 좌표 mm다. 모양(s)은 box · cyl · ring · dtube(z = 반지름, 반 길이, 깎은 깊이. 납작한 면이
형상의 -y 쪽) · prism(pts = 단면의 꼭짓점, z = 반 길이. 단면을 형상의 z 방향으로 뽑는다), 색 구실(c)은
body · cap · frame · base · servo · metal · camera · glass · lens · grip이다.
색이 다른 두 면을 같은 평면에 두면 화면에 줄무늬가 생긴다. 덮개는 몸통 면에서 0.5 mm 넘게 띄운다.

    uv run python -m horilab_design.pages.arm_page
"""

from __future__ import annotations

import json
import math

import numpy as np

from horilab_design.pages import HERE, OUT_DIR
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur_arm
from horilab_design.studies.packaging import MM

ROLE = {
    "base": "base",
    "turntable": "base",
    "d405": "camera",
    "d405_plug": "camera",
}
# 관절의 형상 가운데 검정 출력 원판에 붙은 것. 나머지(집, 목, 귀)는 흰 몸통이다
DISC_PARTS = ("cap", "stopear", "post")
ROT = {
    "x": [0, 0, 1, 0, 1, 0, -1, 0, 0],
    "y": [0, 1, 0, 0, 0, 1, 1, 0, 0],
    "z": [0, 1, 0, -1, 0, 0, 0, 0, 1],
}


def _r3(v: object) -> list[float]:
    return [round(float(x), 3) for x in np.asarray(v, float).ravel()]


def _role(p: pk.Prim) -> str | None:
    """색 구실. 서보 속, 베어링, 무게만 든 조각, 벽 나사 머리의 띠는 그리지 않는다 (None)."""
    name = p.name
    part = name.split("/")[-1]
    hidden = "/servo_" in name or "_mass_" in name or part in ("bearing", "wallscrews")
    if hidden or name == "extra":
        return None
    if p.group in ("도구", "교체 어댑터"):
        return "grip"
    if part in ("post_tip",) or name.startswith("d405_screw"):
        return "metal"
    if "/" in name:
        # J6의 영점 핀 귀는 플랜지 원판에 있다. 다른 관절의 것은 자식 몸통의 귀다
        return "cap" if part in DISC_PARTS or name == "J6/pinear" else "body"
    if name in ROLE:
        return ROLE[name]
    return "body" if p.shape == "hull" or name.endswith("_foot") else "frame"


def _prism(p: pk.Prim) -> dict:
    """볼록 기둥(꼭짓점이 두 끝 면에 같은 차례로 든 껍질)을 단면과 뽑는 방향으로 푼다."""
    assert p.verts is not None
    v = p.verts / MM
    n = len(v) // 2
    low, high = v[:n], v[n:]
    ez = high[0] - low[0]
    length = float(np.linalg.norm(ez))
    ez = ez / length
    ex = low[1] - low[0]
    ex = ex / np.linalg.norm(ex)
    ey = np.cross(ez, ex)
    center = v.mean(axis=0)
    pts = [[round(float((q - center) @ ex), 3), round(float((q - center) @ ey), 3)] for q in low]
    rot = np.column_stack([ex, ey, ez])
    return {"s": "prism", "p": _r3(center), "r": _r3(rot), "z": [round(length / 2, 3)], "pts": pts}


def _prim(p: pk.Prim) -> dict | None:
    """검토 형상 하나를 페이지 형식으로."""
    role = _role(p)
    if role is None:
        return None
    name = p.name
    d: dict = {
        "l": int(p.link),
        "s": p.shape,
        "p": _r3(p.pos / MM),
        "r": _r3(p.rot),
        "c": role,
        "n": name,
    }
    if p.shape == "hull" and name in ("upper_foot", "fore_foot"):
        # 링크의 발 상자: 윗모서리를 깎은 껍질을 상자로 그린다
        assert p.verts is not None
        v = p.verts / MM
        lo, hi = v.min(axis=0), v.max(axis=0)
        d.update(s="box", p=_r3((lo + hi) / 2), z=_r3((hi - lo) / 2))
    elif p.shape == "hull" and name.endswith("_tube"):  # 납작한 관
        assert p.verts is not None
        v = p.verts / MM
        z0, z1 = float(v[:, 2].min()), float(v[:, 2].max())
        radius = float(np.abs(v[:, 0]).max())
        yc = float(p.pos[1] / MM)
        y_lo, y_hi = float(v[:, 1].min()), float(v[:, 1].max())
        flat_low = abs(y_lo - (yc - radius)) > 0.5  # -y 쪽이 깎였으면 True
        cut = radius - (yc - y_lo) if flat_low else radius - (y_hi - yc)
        d.update(s="dtube", p=_r3([0.0, yc, (z0 + z1) / 2]), z=_r3([radius, (z1 - z0) / 2, cut]))
        d["r"] = [1, 0, 0, 0, 1, 0, 0, 0, 1] if flat_low else [-1, 0, 0, 0, -1, 0, 0, 0, 1]
    elif p.shape == "hull":
        d.update(_prism(p))
    else:
        d["z"] = _r3(np.array(p.size) / MM)
    if p.mass > 0:
        d["g"] = round(p.mass * 1000, 2)
    return d


def _cyl(
    link: int, name: str, center: list[float], axis: str, radius: float, half: float, role: str
) -> dict:
    return {
        "l": link,
        "s": "cyl",
        "p": _r3(center),
        "r": ROT[axis],
        "z": _r3([radius, half, 0.0]),
        "c": role,
        "n": name,
    }


def _frames(c: ur_arm.Concept, base: dict) -> list[dict]:
    """박스 쌓기의 TCP 자리마다 관절 각을 푼다 (작업 갈래: 앞, 팔꿈치 위, J5 > 0. 앞 프레임과 가장 가까운 해)."""
    tool = ur_arm.GRIPPER
    h = ur_arm.hori(c, tool)
    robot = h.robot
    frames = []
    prev: np.ndarray | None = None
    for fr in base["frames"]:
        x, y, z = (t * MM for t in fr["tcp"])
        azimuth = math.atan2(y, x)
        pose = pk.tool_pose((x, y, z), 0.0, 0.0, math.pi / 4, tool.length)
        best: np.ndarray | None = None
        best_cost = math.inf
        for q in h.ik.solve(pose):
            fit = ur_arm.fit_ranges(q, robot.lower, robot.upper)
            if fit is None or fit[2] <= 0 or fit[4] <= 0:
                continue
            if abs((fit[0] - azimuth + math.pi) % (2 * math.pi) - math.pi) > math.pi / 2:
                continue
            if pk.collisions(pk.hits(h.scene, fit)):
                continue
            cost = (
                float(np.abs(fit - prev).sum())
                if prev is not None
                else float(h.torque(fit)[1:].max())
            )
            if best is None or cost < best_cost:
                best, best_cost = fit, cost
        if best is None:
            raise RuntimeError(f"박스 쌓기 자세를 풀지 못했다: TCP {fr['tcp']}")
        prev = best
        frames.append({**fr, "q": [round(float(a), 5) for a in best]})
    return frames


def page_data(c: ur_arm.Concept | None = None) -> dict:
    c = c or ur_arm.Concept()
    base = json.loads((HERE / "arm_page_base.json").read_text(encoding="utf-8"))
    robot, prims = ur_arm.build(c, ur_arm.GRIPPER)
    st2, st4 = c.st("ST-3120"), c.st("STS3215")
    st3 = c.st("STS3250")

    def mm(x: float) -> float:
        return x / MM

    out = [d for d in (_prim(p) for p in prims) if d]
    # 뒷덮개(검정): 원통 뒤 끝에서 0.6 바깥으로 띄운다
    a2, a3 = mm(c.upper), mm(c.fore)
    r2, r4 = mm(st2.radius) + 1.6, mm(st4.radius) + 1.6
    covers = (
        (1, "J2", [0, mm(c.c2 + st2.back) + 0.4, mm(c.shoulder)], "y", r2),
        (2, "J3", [0, mm(c.c3 - st3.back) - 0.4, a2], "y", r4),
        (3, "J4", [0, mm(c.c4 + c.sign4 * st4.back) + 0.4 * c.sign4, a3], "y", r4),
        (4, "J5", [0, mm(c.d4), mm(c.mid + st4.back) + 0.4], "z", r4),
        (5, "J6", [0, mm(c.mid + st4.back) + 0.4, mm(c.d5)], "y", r4),
    )
    out += [
        _cyl(link, f"{name}/back_cover", center, axis, r, 1.0, "cap")
        for link, name, center, axis, r in covers
    ]
    # D405의 유리와 렌즈 둘 (카메라 좌표: 원점이 렌즈 면 가운데, x가 가로, z가 광축)
    cam_at, cam_rot = ur_arm.camera_frame(c)

    def on_camera(
        shape: str, local: tuple[float, float, float], size: list[float], role: str
    ) -> dict:
        at = cam_at / MM + cam_rot @ np.array(local)
        return {
            "l": 5,
            "s": shape,
            "p": _r3(at),
            "r": _r3(cam_rot),
            "z": size,
            "c": role,
            "n": role,
        }

    out.append(on_camera("box", (0.0, 0.0, 0.4), [20.0, 20.0, 0.4], "glass"))
    out += [on_camera("cyl", (dx, 0.0, 0.9), [4.0, 0.8, 0.0], "lens") for dx in (-9.0, 9.0)]

    chain = [
        {
            "name": jt.name,
            "origin": _r3(np.array(jt.origin) / MM),
            "axis": _r3(jt.axis),
            "lower": round(math.degrees(jt.lower), 1),
            "upper": round(math.degrees(jt.upper), 1),
        }
        for jt in robot.joints
    ]
    flange = np.array(robot.T_last_flange, dtype=float)
    flange[:3, 3] /= MM
    tags = [
        {"l": 0, "p": [34, -34, 30], "t": "J1"},
        {"l": 2, "p": _r3([0.0, mm(c.c2 + st2.back) - 3.0, 0.0]), "t": "J2"},
        {"l": 3, "p": _r3([0.0, mm(c.c3 - st3.back) + 3.0, 0.0]), "t": "J3"},
        {"l": 4, "p": _r3([0.0, mm(c.c4 + c.sign4 * st4.back) - 3.0, 0.0]), "t": "J4"},
        {"l": 5, "p": _r3([0.0, 0.0, mm(c.mid + st4.back) - 3.0]), "t": "J5"},
        {"l": 6, "p": _r3([0.0, mm(c.mid + st4.back) - 3.0, 0.0]), "t": "J6"},
        {"l": 5, "p": _r3(cam_at / MM + cam_rot @ np.array([21.0, 21.0, -11.5])), "t": "D405"},
    ]
    labels = [
        {
            "l": 2,
            "c": _r3([0.0, mm(c.s2), a2 / 2]),
            "r": mm(c.upper_r),
            "half": a2 / 2 - 40.0,
            "text": "BOMI",
            "flip": True,
        },
        {
            "l": 3,
            "c": _r3([0.0, mm(c.s3), a3 / 2]),
            "r": mm(c.fore_r),
            "half": a3 / 2 - 35.0,
            "text": "HORI",
        },
    ]
    offsets = [round(mm(c.d4)), round(mm(c.d5)), round(mm(c.d6))]
    variant = {
        "label": " · ".join(str(o) for o in offsets),
        "offsets": offsets,
        "chain": chain,
        "flange": [_r3(row) for row in flange],
        "tcp_mm": round(ur_arm.GRIPPER.length / MM, 1),
        "prims": out,
        "tags": tags,
        "cam": {
            **base["cam"],
            "l": 5,
            "lens": _r3(cam_at / MM),
            "axis": _r3(cam_rot[:, 2]),
            "wide": _r3(cam_rot[:, 0]),
        },
        "frames": _frames(c, base),
        "labels": labels,
    }
    keep = {k: v for k, v in base.items() if k not in ("cam", "frames")}
    return {**keep, "variants": [variant]}


def render() -> str:
    template = (HERE / "arm_page.html").read_text(encoding="utf-8")
    data = json.dumps(page_data(), ensure_ascii=False, separators=(",", ":"))
    return template.replace("__DATA__", data)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / "arm.html"
    out.write_text(render(), encoding="utf-8", newline="\n")
    print(f"썼다: {out} ({out.stat().st_size / 1000:.0f} kB)")


if __name__ == "__main__":
    main()
