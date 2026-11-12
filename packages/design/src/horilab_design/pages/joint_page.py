"""첫 관절(손목 몸통 둘) 3D 페이지: 조립 8단계, 펼치기, 관절 돌려 보기, 반으로 잘라 보기.

틀 joint_page.html의 __MESH_JSON__ 자리에 부품 메시를 넣는다. 메시는 부품마다 이름(n), 색 구실(c), 자식과 함께
도는지(m), 무게(g), 꼭짓점(v), 삼각형(i)이고, 좌표는 서보 좌표 mm다 (y가 출력축).

    uv run python -m horilab_design.pages.joint_page
"""

from __future__ import annotations

import json

from horilab_design.pages import HERE, OUT_DIR
from horilab_design.parts import joint as j
from horilab_design.parts import wrist
from horilab_design.printcheck import printed_mass_g
from horilab_design.values import ServoModel

# 조립 그림의 부품 이름 → 뽑는 부품 (무게를 어림할 때 쓴다)
PRINTED = {
    "손목 몸통 (부모)": "wrist_body",
    "손목 몸통 (자식)": "wrist_body",
    "허브 판": "hub_plate",
    "출력 원판": "out_disc",
    "누름 틀": "retainer",
    "서보 받침": "carrier",
    "뒷덮개": "wrist_cover",
}


def mesh_data(model: ServoModel = "STS3215") -> dict:
    printed = wrist.printed()
    masses = {name: printed_mass_g(s, up, preset) for name, (s, up, _c, preset) in printed.items()}
    parts = []
    for p in wrist.first_joint(model):
        verts, tris = p.shape.tessellate(0.06, 0.3)
        grams = masses[PRINTED[p.name]] if p.name in PRINTED else p.mass_g
        parts.append(
            {
                "n": p.name,
                "c": p.color,
                "m": p.moving,
                "g": round(grams, 2),
                "v": [round(c, 2) for v in verts for c in (v.X, v.Y, v.Z)],
                "i": [k for t in tris for k in t],
            }
        )
    meta = {
        "model": model,
        "b0": j.B0,
        "front": j.FRONT,
        "seam": j.SEAM,
        "atop": j.ATOP,
        "back_end": j.BACK_END,
        "cover_out": j.BACK_OUT,
        "r_drum": j.R_DRUM,
        "ret_y": j.RET_SCREW_Y,
        "car_y": j.CAR_SCREW_Y,
        "stop": j.STOP_DEG,
        "r_post": j.R_POST,
        "r_pin": j.R_PIN,
        "web_half": round(j.WEB_HALF, 1),
        "pin_pose": wrist.PIN_POSE_DEG,
    }
    return {"meta": meta, "parts": parts}


def render(model: ServoModel = "STS3215") -> str:
    template = (HERE / "joint_page.html").read_text(encoding="utf-8")
    data = json.dumps(mesh_data(model), ensure_ascii=False, separators=(",", ":"))
    # <script type="application/json"> 안이라 "<"만 바꿔 둔다
    return template.replace("__MESH_JSON__", data.replace("<", "\\u003c"))


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / "joint.html"
    out.write_text(render(), encoding="utf-8", newline="\n")
    print(f"썼다: {out} ({out.stat().st_size / 1000:.0f} kB)")


if __name__ == "__main__":
    main()
