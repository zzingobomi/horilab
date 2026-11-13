"""팔 전체 3D 페이지: 책상 위의 팔, 박스 쌓기 동작, 관절 직접 움직이기.

틀 arm_page.html의 __DATA__ 자리에 데이터를 넣는다. 책상, 박스, 박스 쌓기의 TCP 자리와 도구(평행 그리퍼) 그림은
바탕 데이터(arm_page_base.json)에 있고, 팔의 형상과 관절 각은 검토 모델(studies/ur_arm.py)에서 뽑는다.

형상(prims)은 링크 좌표 mm다. 모양(s)은 box · cyl · ring · dtube(z = 반지름, 반 길이, 깎은 깊이. 납작한 면이
형상의 -y 쪽), 색 구실(c)은 body · cap · frame · base · servo · metal · camera · glass · lens · grip이다.
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
from horilab_design.studies import ur_compare as uc
from horilab_design.studies.packaging import MM, TOOLS

ROLE = {
    "base": "base",
    "turntable": "base",
    "column": "body",
    "d405_plate": "frame",
    "d405": "camera",
}
ROT = {
    "x": [0, 0, 1, 0, 1, 0, -1, 0, 0],
    "y": [0, 1, 0, 0, 0, 1, 1, 0, 0],
    "z": [0, 1, 0, -1, 0, 0, 0, 0, 1],
}


def _r3(v: object) -> list[float]:
    return [round(float(x), 3) for x in np.asarray(v, float).ravel()]


def _prim(p: pk.Prim) -> dict | None:
    """검토 형상 하나를 페이지 형식으로. 서보 속, 베어링, 무게만 든 조각, 벽 나사 머리의 띠, 도구는 뺀다."""
    name = p.name
    hidden = (
        "/servo_" in name
        or name.endswith("/bearing")
        or "_mass_" in name
        or name.endswith("/wallscrews")
        or name in ("adapter", "extra")
    )
    if hidden or p.group == "도구":
        return None
    if name.endswith(("/house", "/far_house", "_foot", "/ears", "/pinear")):
        role = "body"
    elif name.endswith(("/cap", "/stopear")):
        role = "cap"
    else:
        role = ROLE.get(name, "body" if p.shape == "hull" else "frame")
    d: dict = {
        "l": int(p.link),
        "s": p.shape,
        "p": _r3(p.pos / MM),
        "r": _r3(p.rot),
        "c": role,
        "n": name,
    }
    if p.shape == "hull" and name.endswith(
        "_foot"
    ):  # 발 상자: 윗모서리를 깎은 껍질을 상자로 그린다
        assert p.verts is not None
        v = p.verts / MM
        lo, hi = v.min(axis=0), v.max(axis=0)
        d.update(s="box", p=_r3((lo + hi) / 2), z=_r3((hi - lo) / 2))
    elif p.shape == "hull":  # 납작한 관
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
    tool = TOOLS["gripper"]
    robot, prims = ur_arm.build(c, uc.TOOL)
    h = uc.make_hori(robot, prims, (robot.lower, robot.upper), (robot.lower, robot.upper))
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
    robot, prims = ur_arm.build(c, TOOLS["gripper"])
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
    # 도구: 바탕 그림의 평행 그리퍼를 플랜지 면에 놓는다
    for p in base["tool_prims"]:
        out.append({**p, "p": [p["p"][0], round(p["p"][1] + mm(c.d6), 3), p["p"][2]]})
    # D405 유리와 렌즈
    cam = next(p for p in out if p["n"] == "d405")
    cx, cy, cz = cam["p"]
    hy = cam["z"][1]
    one = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    glass = {
        "l": 5,
        "s": "box",
        "p": _r3([cx, cy + hy + 0.4, cz]),
        "r": one,
        "z": [20.0, 0.4, 20.0],
    }
    out.append({**glass, "c": "glass", "n": "glass"})
    for dz in (-9.0, 9.0):
        lens = {
            "l": 5,
            "s": "cyl",
            "p": _r3([cx, cy + hy + 0.9, cz + dz]),
            "r": [1, 0, 0, 0, 0, 1, 0, -1, 0],
        }
        out.append({**lens, "z": [4.0, 0.8, 0.0], "c": "lens", "n": "lens"})

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
        {"l": 5, "p": _r3([cx + 21.0, cy, cz + 21.0]), "t": "D405"},
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
        "tcp_mm": round(TOOLS["gripper"].length / MM, 1),
        "prims": out,
        "tags": tags,
        "cam": {**base["cam"], "l": 5, "lens": _r3([cx, cy + hy, cz])},
        "frames": _frames(c, base),
        "labels": labels,
    }
    keep = {k: v for k, v in base.items() if k not in ("cam", "tool_prims", "frames")}
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
