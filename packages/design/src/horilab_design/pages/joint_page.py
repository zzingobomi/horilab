"""관절 3D 페이지: 조립 단계, 펼치기, 관절 돌려 보기, 반으로 잘라 보기. 첫 관절(손목 몸통 둘), 어깨, J1, 손목 끝을
만든다.

틀 joint_page.html의 __MESH_JSON__ 자리에 데이터를 넣는다:
- parts: 부품마다 이름(n), 색 구실(c), 자식과 함께 도는지(m), 무게(g), 꼭짓점(v), 삼각형(i). 좌표는 서보 좌표 mm다 (y가 출력축).
- meta: 멈춤 각(stop).
- page: 제목과 글, 조립 단계(steps), 부품의 구실(roles: 단계와 끼우는 방향), 숫자 표(dims), 화면에 담을 범위(frame).

    uv run python -m horilab_design.pages.joint_page
"""

from __future__ import annotations

import json

from build123d import Compound

from horilab_design.pages import HERE, OUT_DIR
from horilab_design.parts import base as bs
from horilab_design.parts import camera_mount, flange, gripper, links, wrist
from horilab_design.parts import joint as j
from horilab_design.parts import shoulder as sh
from horilab_design.parts.shapes import box, solid
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
SHOULDER_PRINTED = {
    "허브 판": "shoulder_hub_plate",
    "출력 원판": "shoulder_out_disc",
    "누름 틀": "shoulder_retainer",
    "서보 받침": "shoulder_carrier",
}
ARM_STUB = 62.0  # 어깨 페이지에 보이는 위팔의 길이 (관절 축에서 관 쪽으로)
WRIST_END_PRINTED = {
    "손목 몸통": "wrist_body",
    "허브 판": "hub_plate",
    "플랜지 원판": "flange_disc",
    "누름 틀": "retainer",
    "서보 받침": "carrier",
    "뒷덮개": "wrist_cover",
    "도구 어댑터 판": "tool_adapter",
    "D405 받침": "d405_mount",
}
GRIPPER_PRINTED = {
    "그리퍼 몸통": "gripper_frame",
    "바닥판": "gripper_floor",
    "랙 집": "gripper_tub",
    "피니언": "gripper_pinion",
    "손가락 받침 1": "gripper_slider",
    "손가락 받침 2": "gripper_slider",
    "손가락 1": "gripper_finger",
    "손가락 2": "gripper_finger",
}
BASE_PRINTED = {
    "베이스 몸통": "base_core",
    "책상에 무는 판": "base_plate",
    "돌림판": "base_turntable",
    "허브": "base_hub",
    "사이 고리": "base_spacer",
    "누름 틀": "base_retainer",
    "J1 몸통": "j1_body",
}


def _meshes(parts: list[j.Part], masses: dict[str, float]) -> list[dict]:
    out = []
    for p in parts:
        verts, tris = p.shape.tessellate(0.06, 0.3)
        grams = masses.get(p.name, p.mass_g)
        out.append(
            {
                "n": p.name,
                "c": p.color,
                "m": p.moving,
                "g": round(grams, 2),
                "v": [round(c, 2) for v in verts for c in (v.X, v.Y, v.Z)],
                "i": [k for t in tris for k in t],
            }
        )
    return out


def _role(name: str, step: int, dy: float, pre: bool = False, **more: object) -> dict:
    return {"n": name, "pre": pre, "step": step, "dy": dy, **more}


def _step(short: str, title: str, text: str) -> dict:
    return {"t": short, "title": title, "text": text}


def mesh_data(model: ServoModel = "STS3215") -> dict:
    """첫 관절 (STS 관절: 손목 몸통 둘)."""
    printed = wrist.printed()
    by_file = {name: printed_mass_g(s, up, preset) for name, (s, up, _c, preset) in printed.items()}
    masses = {name: by_file[file] for name, file in PRINTED.items()}
    roles = [
        _role("손목 몸통 (부모)", 0, 0, shell=True),
        _role("베어링", 1, -34, pre=True),
        _role("누름 틀", 2, -60),
        _role("너트 (누름 틀)", 2, -60),
        _role("출력 원판", 3, 28),
        _role("너트 (출력 원판)", 3, 28, pre=True),
        _role("멈춤 나사 M3x10", 3, 28),
        _role("너트 (멈춤 기둥)", 3, 28),
        _role("허브 판", 3, -52),
        _role("서보 몸통", 4, -100),
        _role("혼", 4, -100),
        _role("서보 받침", 4, -116),
        _role("너트 (서보 받침)", 4, -116),
        _role("M2x6", 4, -126, pre=True),
        _role("혼 볼트", 5, 50, pre=True),
        _role("플러그", 7, -100, hideUntil=7),
        _role("선 자리", 7, -100, hideUntil=7),
        _role("뒷덮개", 7, -142, shell=True),
        _role("덮개 나사", 7, -156, pre=True),
        _role("손목 몸통 (자식)", 8, 46),
        _role("몸통 나사", 8, 66, pre=True),
        _role("영점 핀", 8, 90, pre=True, hideUntil=8, pinOnly=True),
    ]
    # 벽 나사 다섯: 1·3·5번은 누름 틀(앞줄, 2단계), 2·4번은 서보 받침(뒷줄, 6단계). 반지름 쪽으로 빠진다
    for k, (direction, step) in enumerate(
        (([0, 0, 1], 2), ([0, 0, 1], 6), ([0, 0, -1], 2), ([0, 0, -1], 6), ([1, 0, 0], 2)), start=1
    ):
        roles.append(_role(f"벽 나사 {k}", step, 0, rad=direction))
    steps = [
        _step(
            "다 조립한 모습",
            "다 조립한 모습",
            "펼치기 막대를 밀면 부품이 끼우는 방향대로 흩어집니다. 관절 돌리기 막대로 자식 쪽을 돌려 보면 163°에서 "
            "멈춤 기둥이 살에 닿습니다. 위의 1~8을 누르면 조립 순서대로 볼 수 있습니다.",
        ),
        _step(
            "베어링",
            "1. 베어링",
            "베어링(6806)을 집 뒤에서 넣어 앞 끝 턱에 닿을 때까지 밉니다. 턱은 집과 한 몸입니다.",
        ),
        _step(
            "누름 틀",
            "2. 누름 틀",
            "너트 3개를 다리 안쪽 홈에 끼운 누름 틀을 뒤에서 넣습니다. 앞으로 민 채 벽 나사 3개(앞줄)를 조입니다. "
            "베어링 바깥 링이 턱과 틀 사이에 잡힙니다.",
        ),
        _step(
            "출력 원판",
            "3. 출력 원판과 허브 판",
            "출력 원판 밑면 홈에 너트 4개를 끼우고, 귀의 기둥에 M3 × 10 나사와 너트를 조여 둡니다. 원판을 앞에서 "
            "안쪽 링에 끼우고, 허브 판은 뒤에서 넣어 안쪽 링 위에 얹습니다.",
        ),
        _step(
            "서보",
            "4. 서보",
            "서보 뒷면에 서보 받침을 M2 × 6 나사 4개로 답니다 (날개의 너트 2개도 먼저 끼웁니다). 그대로 뒤에서 넣어 "
            "혼이 허브 판에 닿게 합니다. 아직 잠그지 않습니다.",
        ),
        _step(
            "혼 볼트",
            "5. 혼 볼트",
            "앞에서 혼 볼트 M3 × 20 4개를 조입니다. 출력 원판, 안쪽 링, 허브 판, 혼이 한 덩어리가 되고, 서보가 "
            "제자리를 찾습니다. 볼트는 혼에 2.6 mm 들어갑니다.",
        ),
        _step(
            "서보 잠금",
            "6. 서보 잠금",
            "벽 나사 2개(뒷줄)를 조여 서보 받침을 집에 잠급니다. 벽 구멍이 축 방향으로 길어서, 서보가 찾은 자리 "
            "그대로 잠깁니다.",
        ),
        _step(
            "선과 덮개",
            "7. 선과 덮개",
            "선을 꽂습니다. 서보 받침의 가까운 쪽이 터져 있어 플러그 옆면을 잡을 수 있습니다. 뒷덮개를 M3 × 12 나사 "
            "2개로 서보 받침에 답니다.",
        ),
        _step(
            "자식 몸통",
            "8. 자식 몸통과 영점",
            "자식 몸통(같은 손목 몸통)을 출력 원판 위에 얹고 밖에서 M3 × 10 나사 4개를 조입니다. 영점은 관절을 "
            "-90°로 돌려 자식의 귀와 부모의 살 구멍이 맞는 자리에서 핀을 꽂고 잡습니다 (관절 돌리기 막대를 -90°에 "
            "두면 핀이 보입니다).",
        ),
    ]
    dims = [
        ["원통 지름", f"{2 * j.R_DRUM:.1f} mm"],
        ["길이 (뒷덮개 ~ 원통 앞 끝)", f"{j.FRONT - j.BACK_OUT:.1f} mm"],
        ["자식 몸통이 앉는 면까지", f"{j.ATOP - j.BACK_OUT:.1f} mm"],
        ["원통 앞 끝과 출력 원판 사이", f"{j.SEAM - j.FRONT:.1f} mm"],
        ["출력 원판 테 두께", f"{j.ATOP - j.SEAM:.1f} mm"],
        ["베어링 자리 (서보 좌표)", f"{j.B0:.1f} ~ {j.B0 + j.BRG_B:.1f}"],
        ["기계 멈춤", f"±{j.STOP_DEG:.0f}° (프로그램 한계 ±{j.SOFT_DEG:.0f}°)"],
        ["멈춤 기둥 중심, 영점 핀", f"축에서 {j.R_POST:.0f} mm, {j.R_PIN:.0f} mm"],
        ["허브 판과 서보 앞면 판 사이", "1.7 ~ 1.8 mm"],
        ["허브 판과 누름 틀 사이", "1.3 mm"],
        ["멈춤 기둥과 원통 벽 사이", "1.2 mm (홈 속 0.8)"],
        ["꽂은 선과 뒷덮개 사이", f"{j.COVER_GAP:.1f} mm"],
        ["벽 나사 줄 (서보 좌표)", f"앞줄 {j.RET_SCREW_Y:.0f}, 뒷줄 {j.CAR_SCREW_Y:.0f}"],
    ]
    page = {
        "title": "STS 관절 구조",
        "h1": "STS 관절 구조: 첫 관절",
        "sub": "처음 뽑아 맞춰 볼 관절입니다. 손목 몸통 두 개(같은 부품)를 하나는 집으로, 하나는 자식으로 조립합니다. "
        "손목 1과 손목 2가 이 부품입니다. 공차 시험편 값이 오면 치수를 넣어 뽑을 파일을 냅니다.",
        "steps": steps,
        "roles": roles,
        "dims": dims,
        "mass_note": "무게는 전에 뽑아서 잰 부품에 맞춘 어림입니다. 베어링 24 g, 서보는 따로입니다.",
        "why": [
            "베어링이 축을 정합니다. 서보는 혼 볼트를 조인 뒤 맨 나중에 벽 나사 2개로 잠가서, 서보 축에 억지 힘이 "
            "걸리지 않습니다.",
            "자식 쪽은 검정 출력 원판에 먼저 혼 볼트를 조이고, 그 위에 자식 몸통을 밖에서 보이는 나사 4개(M3 × 10)로 "
            "답니다. 손목처럼 속이 막힌 몸통도 보통 렌치로 조립되고, 관절을 뜯지 않고 몸통만 뗄 수 있습니다.",
            "혼 볼트를 조이면 허브 판의 얇은 테가 0.2 mm 휘면서 베어링 안쪽 링을 80 ~ 110 N으로 누릅니다. 테가 "
            "스프링이라 온도가 바뀌어도 누르는 힘이 남습니다.",
            "기계 멈춤은 출력 원판의 귀에 선 기둥(속에 M3 나사)이 집 목 앞면의 홈을 돌다가 가운데 살에 닿는 것입니다. "
            "양쪽 163°에서 닿고, 프로그램 한계는 160°로 둡니다.",
            "영점 핀(지름 3 mm)은 자식 몸통 옆 귀와 그 살의 구멍에 꽂습니다. 축에서 29 mm이고, 손목은 범위 가운데에서 "
            "90° 돌린 자리에서 맞습니다.",
            "선은 앞벽 가운데 홈에서 살 위로 나와 출력 원판의 테에 감기며 자식으로 넘어갑니다. 테 밑의 테두리가 선이 "
            "틈으로 빠지는 것을 막습니다. 가운데 자세에서 약 7 cm가 느슨한 고리로 남습니다. 손목 몸통 앞벽의 발 쪽 "
            "끝에는 선 홈이 셋 있습니다 (가운데는 나가는 선, 양옆은 들어오는 선).",
            "커넥터 앞쪽은 서보 받침을 터서 플러그를 손톱이나 핀셋으로 잡을 수 있습니다. 뒷덮개(나사 2개)만 떼면 닿습니다.",
            "쓰는 나사는 모두 가진 것입니다 (M3 × 8, × 10, × 12, × 20, M2 × 6, M3 너트). STS3215와 STS3250 둘 다 같은 "
            "부품으로 되고, CAD에서 겹침이 없습니다.",
        ],
        "todo": [
            "서보 혼이 앞면 판보다 얼마나 높은지를 다시 재서, 관절 속 서보 자리를 그 값으로 옮깁니다.",
            "선은 그림에 넣지 않았습니다. 길과 길이는 정했고(결정 기록 0031), D405 선은 가진 선을 잰 뒤에 정합니다.",
            "베어링 자리와 출력 원판 저널의 지름은 시험편 값이 오면 넣습니다 (지금은 "
            f"{j.BEARING_BORE}, {j.JOURNAL_D})",
        ],
        "frame": {"y0": -27, "y1": 102, "w": 96, "cx": -10},
        "pin_pose": wrist.PIN_POSE_DEG,
    }
    return {
        "meta": {"model": model, "stop": j.STOP_DEG},
        "parts": _meshes(wrist.first_joint(model), masses),
        "page": page,
    }


def _arm_stub(link: Compound, length: float) -> Compound:
    """위팔에서 어깨 쪽 발과 관의 첫머리만 남긴다 (링크 좌표: 앞 관절 축이 x = -length)."""
    return solid(link & box(-length - 60, -length + ARM_STUB, -80, 80, -60, 60))


def shoulder_data() -> dict:
    """어깨 관절 (ST-3120, 6808)."""
    length, tube_r, foot = links.LINKS["upper_arm"]
    child = _arm_stub(links.link(length, tube_r, foot), length)
    by_file = {
        name: printed_mass_g(s, up, preset) for name, (s, up, _c, preset) in sh.printed().items()
    }
    masses = {name: by_file[file] for name, file in SHOULDER_PRINTED.items()}
    # 집과 뒷덮개는 이 페이지에 보이는 모양(목이 짧다)의 무게다. 뽑는 것은 J1 관절 페이지의 J1 몸통이다
    masses["어깨 집"] = printed_mass_g(sh.house(), (0.0, -1.0, 0.0))
    masses["뒷덮개"] = printed_mass_g(sh.back_cover(), (0.0, 1.0, 0.0), "standard")
    roles = [
        _role("어깨 집", 0, 0, shell=True),
        _role("베어링", 1, -40, pre=True),
        _role("누름 틀", 2, -74),
        _role("너트 (누름 틀)", 2, -74),
        _role("출력 원판", 3, 30),
        _role("너트 (출력 원판)", 3, 30, pre=True),
        _role("멈춤 나사 M3x10", 3, 30),
        _role("너트 (멈춤 기둥)", 3, 30),
        _role("허브 판", 3, -64),
        _role("서보 몸통", 4, -120),
        _role("혼", 4, -120),
        _role("플러그", 4, -120),
        _role("선 자리", 4, -120),
        _role("서보 받침", 4, -138),
        _role("너트 (서보 받침)", 4, -138),
        _role("M2.5x6", 4, -150, pre=True),
        _role("혼 볼트", 5, 54, pre=True),
        _role("뒷덮개", 7, -166, shell=True),
        _role("덮개 나사", 7, -182, pre=True),
        _role("위팔 나사", 8, 70, pre=True),
        _role("위팔", 8, 48),
        _role("영점 핀", 8, 96, pre=True, hideUntil=8, pinOnly=True),
    ]
    # 벽 나사 여섯: 누름 틀 셋(앞줄, 2단계)과 서보 받침 셋(뒷줄, 6단계). 양옆과 위(가까운 끝 쪽)에서 조인다
    for name, step in (("누름 틀", 2), ("서보 받침", 6)):
        for k, direction in enumerate(([0, 0, 1], [0, 0, -1], [1, 0, 0]), start=1):
            roles.append(_role(f"벽 나사 ({name}) {k}", step, 0, rad=direction))
    steps = [
        _step(
            "다 조립한 모습",
            "다 조립한 모습",
            "어깨 관절입니다. 아래로 뻗은 목은 J1 몸통의 윗부분이고, 위에는 위팔의 발 쪽만 그렸습니다. 펼치기 막대를 "
            "밀면 부품이 끼우는 방향대로 흩어지고, 관절 돌리기 막대로 위팔을 돌리면 163°에서 멈춤 기둥이 살에 닿습니다.",
        ),
        _step(
            "베어링",
            "1. 베어링",
            "베어링(6808)을 집 뒤에서 넣어 앞 끝 턱에 닿을 때까지 밉니다.",
        ),
        _step(
            "누름 틀",
            "2. 누름 틀",
            "너트 3개를 다리 안쪽 홈에 끼운 누름 틀을 뒤에서 넣습니다. 앞으로 민 채 벽 나사 3개(앞줄: 양옆과 위)를 "
            "조입니다.",
        ),
        _step(
            "출력 원판",
            "3. 출력 원판과 허브 판",
            "출력 원판 밑면 홈에 너트 6개를 끼우고, 귀의 기둥에 M3 × 10 나사와 너트를 조여 둡니다. 원판을 앞에서 "
            "안쪽 링에 끼우고, 허브 판은 뒤에서 넣어 안쪽 링 위에 얹습니다.",
        ),
        _step(
            "서보",
            "4. 선을 꽂은 서보",
            "서보 옆면 커넥터에 선을 먼저 꽂습니다 (집 속에서는 옆 틈이 좁아 꽂을 수 없습니다). 서보 뒷면에 서보 받침을 "
            "M2.5 × 6 나사 6개로 달고(날개의 너트 3개도 먼저 끼웁니다), 선을 목 아래로 내리면서 뒤에서 넣어 혼이 "
            "허브 판에 닿게 합니다. 아직 잠그지 않습니다.",
        ),
        _step(
            "혼 볼트",
            "5. 혼 볼트",
            "앞에서 혼 볼트 M3 × 20 8개를 마주 보는 순서로 조금씩 조입니다. 허브 판의 테가 0.2 mm 휘면서 안쪽 링을 "
            "누르고, 서보가 제자리를 찾습니다.",
        ),
        _step(
            "서보 잠금",
            "6. 서보 잠금",
            "벽 나사 3개(뒷줄)를 조여 서보 받침을 집에 잠급니다. 벽 구멍이 축 방향으로 ±1.5 mm 길어서, 서보가 찾은 "
            "자리 그대로 잠깁니다.",
        ),
        _step(
            "뒷덮개",
            "7. 뒷덮개",
            "뒷덮개를 M3 × 12 나사 2개로 서보 받침에 답니다.",
        ),
        _step(
            "위팔",
            "8. 위팔과 영점",
            "위팔의 발을 출력 원판 위에 얹고 밖에서 M3 × 10 나사 6개를 조입니다. 영점은 위팔을 곧게 세운 자리(0°)에서 "
            "발 끝의 귀와 집의 살 구멍에 핀을 꽂고 잡습니다.",
        ),
    ]
    dims = [
        ["원통 지름", f"{2 * sh.R_DRUM:.1f} mm"],
        ["길이 (뒷덮개 ~ 원통 앞 끝)", f"{sh.FRONT - sh.BACK_OUT:.1f} mm"],
        ["위팔이 앉는 면까지", f"{sh.ATOP - sh.BACK_OUT:.1f} mm"],
        ["원통 앞 끝과 출력 원판 사이", f"{sh.SEAM - sh.FRONT:.1f} mm"],
        ["베어링 자리 (서보 좌표)", f"{sh.B0:.1f} ~ {sh.B0 + sh.BRG_B:.1f}"],
        ["목의 폭 (바깥 / 안)", f"{2 * sh.NECK_HW:.1f} / {2 * sh.NECK_IN:.1f} mm"],
        ["기계 멈춤", f"±{j.STOP_DEG:.0f}° (프로그램 한계 ±{j.SOFT_DEG:.0f}°)"],
        ["멈춤 기둥 중심, 영점 핀", f"축에서 {sh.R_POST:.0f} mm, {sh.R_PIN:.0f} mm"],
        ["허브 판이 안쪽 링을 누르는 힘", "140 ~ 190 N"],
        ["허브 판과 누름 틀 사이", "1.3 mm"],
        ["멈춤 기둥과 원통 벽 사이", "1.2 mm (홈 속 0.8)"],
        ["서보 먼 끝과 목 속 턱 사이", f"{sh.PAD_GAP:.2f} mm"],
        ["벽 나사 줄 (서보 좌표)", f"앞줄 {sh.RET_SCREW_Y:.1f}, 뒷줄 {sh.CAR_SCREW_Y:.0f}"],
    ]
    page = {
        "title": "어깨 관절 구조",
        "h1": "어깨 관절 구조 (ST-3120)",
        "sub": "어깨 서보(ST-3120)가 오기 전이라 공식 도면의 치수로 그렸습니다. 서보가 오면 혼 높이와 옆면 커넥터 "
        "자리를 재서 고칩니다. 짜임은 STS 관절과 같고, 베어링이 6808로 크고 혼 볼트가 8개입니다.",
        "steps": steps,
        "roles": roles,
        "dims": dims,
        "mass_note": "무게는 전에 뽑아서 잰 부품에 맞춘 어림입니다. 베어링 31 g, 서보 210 g은 따로입니다. 위팔은 그림에 "
        "보이는 발 쪽만의 무게입니다.",
        "why": [
            "STS 관절과 같은 짜임입니다. 베어링이 축을 정하고, 서보는 혼 볼트를 조인 뒤 맨 나중에 벽 나사 3개로 "
            "잠급니다.",
            "혼(FT110M)은 얇은 테(두께 2.5 mm)에 허브가 달린 모양이라, 베어링을 혼 앞에 두고 허브 판을 혼 윗면에 "
            "앉혔습니다. 베어링을 혼 둘레에 두는 짧은 구조는 누르는 힘을 정할 스프링 자리가 없어서 버렸습니다.",
            "쌓임이 길어진 만큼 서보를 출력 반대쪽으로 9.8 mm 물렸습니다. 위팔의 자리와 팔의 치수는 그대로이고, "
            "어깨 원통이 J1 축을 가운데로 거의 대칭입니다.",
            "커넥터가 서보 옆면에 있어서, 꽂은 플러그와 꺾인 선이 들어가게 목이 원통만큼 넓습니다. 서보 먼 끝은 목 "
            "속의 턱 둘이 옆에서 잡아, 서보가 세게 밀어도 0.5° 안에서 멈춥니다.",
            "위팔은 나사 6개로 답니다. 마찰로 버티는 토크가 서보가 끝까지 미는 토크의 약 2.2배입니다.",
            "위팔 발의 윗모서리는 12 mm 깎아 관의 둥근 윤곽 안에 들게 했습니다. 접은 자세에서 손목과 덜 닿습니다.",
        ],
        "todo": [
            "서보가 오면 잴 것: 혼 윗면 높이(도면으로 9.9 ~ 10.2 mm), 옆면 커넥터 자리, 혼 가운데 나사 머리.",
            f"6808 자리와 저널의 지름은 시험편으로 맞춥니다 (지금은 {sh.BEARING_BORE}, {sh.JOURNAL_D}).",
            "어깨 집은 J1 몸통의 윗부분입니다. 돌림판까지 내려온 목과 바닥판은 J1 관절 페이지에 있습니다.",
            "선은 그림에 넣지 않았습니다 (플러그와 선이 차지하는 자리만 반투명으로 보입니다).",
        ],
        "frame": {"y0": -36, "y1": 96, "w": 132, "cx": 2},
        "pin_pose": 0,
    }
    return {
        "meta": {"model": "ST-3120", "stop": j.STOP_DEG},
        "parts": _meshes(sh.assembly(child, length), masses),
        "page": page,
    }


def base_data(model: ServoModel = "STS3215") -> dict:
    """J1 관절 (베이스 몸통, 6806 둘, 돌림판, J1 몸통)."""
    by_file = {
        name: printed_mass_g(s, up, preset) for name, (s, up, _c, preset) in bs.printed().items()
    }
    masses = {name: by_file[file] for name, file in BASE_PRINTED.items()}
    carrier, up, _c, preset = wrist.printed()["carrier"]
    masses["서보 받침"] = printed_mass_g(carrier, up, preset)
    roles = [
        _role("베이스 몸통", 0, 0, shell=True),
        _role("책상에 무는 판", 7, -40),
        _role("판 나사", 7, -72, pre=True),
        _role("와셔 (판 나사)", 7, -64, pre=True),
        _role("너트 (판)", 7, 0, pre=True),
        _role("책상", 8, -60, hideUntil=8),
        _role("클램프", 8, 0, pre=True, rad=[-3.5, 0, 0], hideUntil=8),
        _role("베어링 6806 (위)", 1, -94),
        _role("사이 고리", 1, -102),
        _role("베어링 6806 (아래)", 1, -110),
        _role("누름 틀", 2, -118),
        _role("너트 (누름 틀)", 2, -118),
        _role("돌림판", 3, 50),
        _role("J1 몸통 나사", 3, 30, pre=True),
        _role("너트 (J1 몸통)", 3, 136, pre=True),
        _role("J1 몸통", 3, 120),
        _role("멈춤 나사 M3x10", 3, 50),
        _role("너트 (멈춤 기둥)", 3, 50),
        _role("허브", 4, -150),
        _role("혼 나사", 4, -136, pre=True),
        _role("너트 (허브)", 4, -150, pre=True),
        _role("서보 몸통", 4, -150),
        _role("혼", 4, -150),
        _role("플러그", 4, -150),
        _role("선 자리", 4, -150),
        _role("서보 받침", 4, -168),
        _role("너트 (서보 받침)", 4, -168),
        _role("M2x6", 4, -180, pre=True),
        _role("조임 볼트", 5, 104, pre=True),
        _role("와셔", 5, 96, pre=True),
        _role("어깨 서보", 9, 120),
        _role("영점 핀", 9, 160, pre=True, hideUntil=9, pinOnly=True),
    ]
    # 벽 나사 다섯: 1·3·5번은 누름 틀(2단계), 2·4번은 서보 받침(6단계). 몸통의 굴 밖으로 빠진다
    for k, (direction, step) in enumerate(
        (
            ([0, 0, 2.2], 2),
            ([0, 0, 2.2], 6),
            ([0, 0, -2.2], 2),
            ([0, 0, -2.2], 6),
            ([2.2, 0, 0], 2),
        ),
        start=1,
    ):
        roles.append(_role(f"벽 나사 {k}", step, 0, rad=direction))
    steps = [
        _step(
            "다 조립한 모습",
            "다 조립한 모습",
            "J1 관절입니다. 아래의 검정 원기둥이 책상에 고정되는 베이스 몸통이고, 그 위의 검정 원판이 돌림판, 흰 부품이 "
            "J1 몸통(위쪽이 어깨 집)입니다. 펼치기 막대를 밀면 부품이 끼우는 방향대로 흩어지고, 관절 돌리기 막대로 "
            "돌리면 163°에서 멈춤 기둥이 살에 닿습니다.",
        ),
        _step(
            "베어링 둘",
            "1. 베어링 둘과 사이 고리",
            "베이스 몸통을 뒤집어 놓고(윗면이 바닥), 베어링(6806) → 사이 고리 → 베어링 순서로 넣습니다. 첫 베어링은 "
            "몸통의 턱에 닿고, 사이 고리는 두 베어링의 안쪽 링 사이에 놓입니다.",
        ),
        _step(
            "누름 틀",
            "2. 누름 틀",
            "너트 3개를 다리 안쪽 홈에 끼운 누름 틀을 넣습니다. 베어링 쪽으로 민 채, 몸통 옆의 굴로 벽 나사 "
            "3개(M3 × 8)를 조입니다. 바깥 링 둘이 턱과 틀 사이에 잡힙니다.",
        ),
        _step(
            "돌림판",
            "3. 돌림판과 J1 몸통",
            "책상 위에서 먼저 묶습니다. 돌림판 윗면의 볼트 자리 4곳은 얇은 막 한 층으로 막혀 나오니, 2.5 mm "
            "육각렌치로 눌러 뚫어 둡니다. 돌림판의 멈춤 기둥에 M3 × 10 나사와 너트를 조여 두고, J1 몸통 바닥판 "
            "윗면의 홈에 너트 4개를 끼웁니다. 돌림판을 J1 몸통 밑에 대고 밑에서 M3 × 12 나사 4개를 조입니다. "
            "베이스 몸통을 바로 세우고, 이 묶음의 저널을 위에서 두 베어링과 사이 고리 속으로 끼웁니다.",
        ),
        _step(
            "서보",
            "4. 허브를 단 서보",
            "책상 위에서 허브를 서보 혼에 M3 × 8 나사 4개로 달고, 허브 옆 홈에 너트 4개를 밀어 넣습니다. 서보 "
            "뒷면에 서보 받침을 M2 × 6 나사 4개로 달고(날개의 너트 2개도 먼저) 선을 꽂습니다. 그대로 아래에서 넣어 "
            "허브가 아래 베어링의 안쪽 링에 닿게 합니다. 아직 잠그지 않습니다.",
        ),
        _step(
            "조임 볼트",
            "5. 조임 볼트",
            "와셔를 낀 M3 × 45 렌치볼트(머리가 원통 모양) 4개를 J1 몸통 속으로 넣어 돌림판의 볼트 자리에 꽂습니다. "
            "2.5 mm ㄱ자 육각렌치의 긴 쪽을 볼트 머리에 세워 꽂고, J1 몸통의 열린 뒷면으로 손을 넣어 짧은 쪽을 "
            "돌립니다. 허브의 너트에 조금씩 번갈아 조입니다. 돌림판, 안쪽 링 둘, 사이 고리, 허브가 한 덩어리가 "
            "되고 서보가 제자리를 찾습니다.",
        ),
        _step(
            "서보 잠금",
            "6. 서보 잠금",
            "몸통 옆의 굴로 벽 나사 2개(M3 × 8)를 조여 서보 받침을 잠급니다. 구멍이 축 방향으로 길어서, 서보가 찾은 "
            "자리 그대로 잠깁니다.",
        ),
        _step(
            "판",
            "7. 책상에 무는 판",
            "몸통 밑면 옆의 홈 4곳에 너트를 밀어 넣습니다. 판 밑면의 나사 자리 4곳은 얇은 막 한 층으로 막혀 나오니 "
            "렌치로 눌러 뚫어 둡니다. 몸통을 판의 둥근 자리에 앉히고(선은 뒤쪽 틈으로 뺍니다), 밑에서 와셔를 낀 "
            "M3 × 10 나사 4개를 조입니다. 나사 머리는 판 밑면 안으로 들어가 책상에 닿지 않습니다.",
        ),
        _step(
            "책상",
            "8. 책상에 물리기",
            "판의 뒤 끝을 책상 뒤 가장자리에 맞추고, 양옆의 날개를 금속 클램프 2개로 뭅니다. 클램프는 가장자리에서 "
            "2 ~ 3 cm 들어온 곳, 몸통 바로 옆을 뭅니다. 그림의 클램프는 가진 금속 클램프를 사진으로 어림한 모양입니다.",
        ),
        _step(
            "영점",
            "9. 영점과 어깨 서보",
            "영점은 팔이 앞을 보는 자리(0°)에서 J1 몸통 뒤쪽 귀의 구멍과 베이스 몸통의 살 구멍에 핀을 꽂고 "
            "잡습니다. 어깨 서보는 그 뒤에 J1 몸통에 넣습니다 (어깨 관절 페이지).",
        ),
    ]
    spool = j.GAP + bs.BRIM_T + j.EAR_T
    dims = [
        ["베이스 몸통 지름", f"{2 * bs.R_CORE:.0f} mm"],
        ["돌림판 지름 (선이 감기는 면)", f"{2 * bs.R_TABLE:.0f} mm"],
        ["몸통 윗면의 높이 (책상에서)", f"{bs.FRONT - bs.Y_DESK:.1f} mm"],
        ["돌림판 윗면의 높이", f"{bs.TOP_H:.1f} mm"],
        ["어깨 축의 높이", f"{bs.SHOULDER_H:.0f} mm"],
        ["두 베어링의 가운데 사이", f"{bs.B1 - bs.B0:.0f} mm"],
        ["기계 멈춤", f"±{j.STOP_DEG:.0f}° (프로그램 한계 ±{j.SOFT_DEG:.0f}°)"],
        ["멈춤 기둥 중심, 영점 핀", f"축에서 {bs.R_POST:.0f} mm, {bs.R_PIN:.0f} mm"],
        ["선이 감기는 면의 높이", f"{spool:.1f} mm"],
        ["허브와 누름 틀 사이", "1.3 mm"],
        ["사이 고리와 몸통 사이", f"{j.R_BORE - bs.SPACER_R:.1f} mm"],
        ["멈춤 기둥과 홈 벽 사이", "1.2 mm"],
        ["J1 몸통 나사", f"축에서 {bs.R_FOOT:.0f} mm, M3 × 12 넷"],
        [
            "책상에 무는 판",
            f"{bs.EDGE + bs.R_CORE + bs.SEAT_GAP + bs.RING_W:.0f} × {2 * bs.WING_Z:.0f} mm, "
            f"날개 두께 {bs.PLATE_T + bs.RING_H:.0f} mm",
        ],
        [
            "클램프가 무는 자리",
            f"가장자리에서 {bs.CLAMP_IN:.0f} mm, 축에서 옆으로 {bs.CLAMP_Z:.0f} mm",
        ],
        ["책상 가장자리에서 J1 축까지", f"{bs.EDGE:.0f} mm"],
        ["조임 볼트", f"M3 × {bs.CLAMP_LEN:.0f} 넷, 머리 자리 깊이 {bs.SINK:.1f} mm"],
        [
            "J1 몸통 바닥판 ~ 어깨 서보 먼 끝",
            f"{bs.SHOULDER_H - sh.FAR - bs.TOP_H - bs.FLOOR_T:.1f} mm",
        ],
    ]
    page = {
        "title": "J1 관절 구조",
        "h1": "J1 관절 구조 (베이스)",
        "sub": "팔 전체를 받치고 돌리는 맨 아래 관절입니다. 밑에 깐 판의 날개 둘을 금속 클램프로 책상 뒤 가장자리에 "
        "뭅니다. 서보 혼의 높이 값이 오면 허브 모양과 돌림판 높이가 1 mm쯤 달라집니다.",
        "steps": steps,
        "roles": roles,
        "dims": dims,
        "mass_note": "무게는 전에 뽑아서 잰 부품에 맞춘 어림입니다. 베어링 24 g 둘, 서보는 따로입니다.",
        "why": [
            "베어링 둘을 30 mm 띄워 받칩니다. 팔 전체가 기울이려는 힘을 두 베어링이 나눠 받아, J1 축이 덜 기웁니다. "
            "돌림판의 저널 하나가 두 안쪽 링을 다 지나므로 두 링이 한 원기둥에 앉습니다.",
            "허브는 혼에 짧은 나사로 먼저 달고, 긴 조임 볼트는 허브 속의 너트(쇠 나사산)에 조입니다. 혼의 얇은 "
            "나사산(2.5 mm)에 긴 볼트를 걸지 않아도 되고, 볼트 길이가 혼 높이에 매이지 않습니다.",
            "조임 볼트는 가진 M3 × 45를 씁니다. 머리와 와셔는 돌림판 윗면에서 5.6 mm 내려간 자리에 앉습니다. 그 "
            "자리가 위 안쪽 링보다 2.4 mm 위라서, 볼트 힘이 그 사이의 살을 누르기만 합니다. 자리의 천장은 뽑을 "
            "때 허공에 걸치므로, 막 한 층이 볼트 구멍까지 덮게 해서 고르게 걸치게 했습니다.",
            "조임 볼트는 돌림판, 위 안쪽 링, 사이 고리, 아래 안쪽 링, 허브를 한 덩어리로 뭅니다. 바깥 링은 위쪽이 "
            "몸통의 턱, 아래쪽이 누름 틀입니다. 누름 틀을 밀어 붙인 채 잠가서 베어링의 헐거움을 줄입니다.",
            "J1 몸통은 돌림판에 축 가까이(21 mm)의 나사 넷으로 답니다. 가장자리에서 잡으면 돌림판의 얇은 테가 "
            "팔 무게에 휘기 때문입니다. 나사는 돌림판 밑에서 올라와 J1 몸통의 두꺼운 바닥판(9.8 mm) 속 너트에 "
            "물리므로, 둘을 책상 위에서 먼저 묶습니다.",
            "서보, 누름 틀, 서보 받침은 STS 관절과 같은 짜임입니다. 서보 자리에는 STS3250도 들어가서, 정밀도가 "
            "모자라면 J1 서보를 바꿀 수 있습니다.",
            "베이스 몸통은 두꺼운 원기둥입니다. 무게가 문제 되지 않는 자리라 베어링 둘레를 두껍게 했고, 벽 나사는 "
            "몸통 옆의 네모 굴 속에 앉습니다.",
            "책상에 무는 판은 몸통 밑에 깔고 밑에서 나사 4개로 묶습니다. 클램프가 무는 자리는 몸통 바로 옆이라, "
            "팔이 베이스를 기울이려는 힘이 판을 거의 휘지 않고 클램프로 갑니다. 두 군데를 띄워 물어서 J1이 돌 때 "
            "베이스가 돌지 않습니다. 날개 바깥 가장자리의 나사 자리 4곳은 선 정리 부품이나 스프링 기둥을 달 "
            "자리입니다.",
            "기계 멈춤 기둥은 돌림판 밑면에 있고, 몸통 윗면의 둥근 홈을 돌다가 뒤쪽에 남긴 살에 닿습니다. 영점 "
            "핀은 J1 몸통의 뒤쪽 귀(두께 6.2 mm)에서 그 살에 꽂습니다.",
            "선은 베이스 몸통 윗부분의 띠(높이 16 mm)에 감깁니다. 반지름이 40.5 mm라 D405 선의 굽힘 기준(40 mm)과 "
            "맞습니다. 몸통 뒤 왼쪽의 누르개와 J1 몸통 뒤쪽 귀 위의 누르개가 선을 잡고, 돌림판 밑의 테두리가 선이 "
            "틈으로 빠지는 것을 막습니다.",
            "돌림판 테의 가는 홈은 J1의 틈을 한쪽으로 미는 스프링(또는 추를 단 줄)의 자리입니다. 다 만든 팔의 반복 "
            "정밀도가 모자랄 때만 답니다.",
        ],
        "todo": [
            "선 누르개(선을 누르는 작은 판)는 쓸 선의 굵기가 정해진 뒤에 그립니다. 나사 자리는 냈습니다.",
            "조임 볼트는 가진 M3 × 45 렌치볼트 4개와 평와셔 4개를 씁니다. 살 것이 없습니다 (M3 × 45가 4개 넘게 "
            "있는지만 셉니다).",
            f"베어링 자리와 저널의 지름은 검정 시험편 값이 오면 넣습니다 (지금은 {j.BEARING_BORE}, {j.JOURNAL_D}).",
            "허브 옆 홈의 너트 자리는 검정 너트 홈 시험편 값으로 바꿉니다.",
        ],
        "frame": {"y0": -95, "y1": 176, "w": 200, "cx": 0},
        "pin_pose": 0,
        "axis": "위가 J1 몸통(어깨 쪽), 아래가 책상",
    }
    return {
        "meta": {"model": model, "stop": j.STOP_DEG},
        "parts": _meshes(bs.assembly(model) + bs.desk_parts(), masses),
        "page": page,
    }


def wrist_end_data(model: ServoModel = "STS3215") -> dict:
    """손목 끝: J6 관절에 플랜지 원판, 도구 어댑터 판, D405와 받침."""
    by_file = {
        name: printed_mass_g(s, up, preset)
        for printed in (wrist.printed(), flange.printed(), camera_mount.printed())
        for name, (s, up, _c, preset) in printed.items()
    }
    masses = {name: by_file[file] for name, file in WRIST_END_PRINTED.items()}
    masses["D405"] = 60.0
    parts = camera_mount.assembly(model)
    roles = [
        _role("손목 몸통", 0, 0, shell=True),
        _role("뒷덮개", 0, 0, shell=True),
        _role("플랜지 원판", 1, 28),
        _role("도구 핀", 1, 52, pre=True),
        _role("너트 (출력 원판)", 1, 28, pre=True),
        _role("멈춤 나사 M3x10", 1, 28),
        _role("너트 (멈춤 기둥)", 1, 28),
        _role("혼 볼트", 1, 62, pre=True),
        _role("D405 받침", 2, 0, rad=[0, 0, -2.2]),
        _role("너트 (돋움)", 2, -24, pre=True),
        _role("받침 나사", 2, 0, pre=True, rad=[0, 0, -4.2]),
        _role("D405 플러그", 4, 0, rad=[-2.6, 0, 0], hideUntil=4),
        _role("D405", 3, 0, rad=[0, 0, -3.2]),
        _role("카메라 나사", 3, -40, pre=True),
        _role("도구 어댑터 판", 5, 40),
        _role("손나사", 5, 62, pre=True),
        _role("손잡이", 5, 62, pre=True),
        _role("영점 핀", 6, 60, pre=True, hideUntil=6, pinOnly=True),
    ]
    # 나머지(관절 속 부품과 나사)는 STS 관절과 같아서 제자리에 둔다
    for p in parts:
        if not any(p.name == r["n"] or (r["pre"] and p.name.startswith(r["n"])) for r in roles):
            roles.append(_role(p.name, 0, 0))
    steps = [
        _step(
            "다 조립한 모습",
            "다 조립한 모습",
            "손목 끝입니다. 흰 부품이 손목 2 몸통(J6의 집), 그 위의 검정 원판이 플랜지, 옆의 어두운 상자가 D405 "
            "카메라입니다. 관절 돌리기 막대로 플랜지를 돌려 보면, 원판 밑으로 내려온 멈춤 기둥이 카메라 앞을 "
            "2.6 mm 틈으로 지나갑니다.",
        ),
        _step(
            "플랜지",
            "1. 플랜지 원판",
            "STS 관절의 출력 원판 자리에 플랜지 원판을 씁니다. 도구 자리를 잡는 강철 핀 2개를 먼저 눌러 박아 둡니다. "
            "나머지 조립은 STS 관절과 같습니다 (STS 관절 구조 페이지).",
        ),
        _step(
            "받침",
            "2. D405 받침",
            "손목 몸통 옆의 돋움 2곳에 너트를 뒤에서 떨어뜨려 넣습니다. 받침의 발을 돋움에 대고 M3 × 8 나사 2개를 "
            "조입니다. 서보를 잠그는 벽 나사는 그 전에 조여 둡니다 (받침을 달면 그쪽 벽 나사 하나에 렌치가 닿지 "
            "않습니다).",
        ),
        _step(
            "카메라",
            "3. 카메라",
            "D405의 뒷면을 받침의 판에 대고, 판 뒤에서 M3 × 6 나사 2개를 조입니다. 커넥터가 있는 옆면이 앞 관절(J5) "
            "쪽을 보게 답니다. 나사는 카메라에 3 mm만 들어가므로 더 긴 나사를 쓰지 않습니다.",
        ),
        _step(
            "플러그",
            "4. 플러그",
            "선을 꽂습니다. 그림의 플러그는 ㄱ자로 꺾인 것(옆으로 12 mm)으로 가정한 크기입니다. 가진 곧은 플러그는 "
            "30 mm쯤 나와서 J5 관절을 넘습니다.",
        ),
        _step(
            "도구",
            "5. 도구",
            "도구의 어댑터 판을 플랜지의 핀 2개에 끼우고 손나사 2개를 조입니다. 도구를 바꿔도 카메라는 그대로입니다.",
        ),
        _step(
            "영점",
            "6. 영점",
            "J6의 영점 핀은 플랜지 원판의 귀에서 손목 몸통의 살 구멍에 꽂습니다 (관절 각 0°).",
        ),
    ]
    dims = [
        ["렌즈 면 가운데 (플랜지 면에서 뒤로)", f"{camera_mount.LENS_BACK:.1f} mm"],
        ["렌즈 면 가운데 (J6 축에서)", f"{-camera_mount.LENS[2]:.1f} mm"],
        ["광축 기울기 (도구 축 쪽으로)", f"{camera_mount.TILT:.0f}°"],
        ["카메라와 원통 옆면 사이", f"{camera_mount.DRUM_GAP:.1f} mm"],
        ["도는 부품과 카메라 사이 (가장 좁은 곳)", "2.6 mm"],
        ["받침의 판", f"두께 {camera_mount.PLATE_T:.1f}, 폭 {camera_mount.MOUNT_W:.0f} mm"],
        [
            "카메라 무게에 기우는 각",
            f"{camera_mount.tilt_under_weight(2.75):.3f} ~ {camera_mount.tilt_under_weight(2.0):.3f}°",
        ],
        ["카메라 나사", "M3 × 6 둘 (카메라에 3.0 mm)"],
        ["받침 나사", "M3 × 8 둘 (돋움 속 너트)"],
        [
            "도구 핀",
            f"축에서 ±{flange.TOOL_PIN_R:.1f} mm (멈춤 귀와 직각 방향), 위로 {flange.TOOL_PIN_UP:.0f} mm",
        ],
        ["도구 어댑터 판", f"지름 {2 * flange.ADAPTER_R:.0f}, 두께 {flange.ADAPTER_T:.1f} mm"],
    ]
    page = {
        "title": "손목 끝 구조",
        "h1": "손목 끝 구조: 플랜지와 D405 받침",
        "sub": "J6 관절 끝입니다. 도구를 갈아 끼우는 플랜지와, J6 집 옆에 다는 D405 카메라 받침입니다. 플러그 크기는 "
        "가정이고, 가진 선을 잰 뒤에 맞춥니다.",
        "steps": steps,
        "roles": roles,
        "dims": dims,
        "mass_note": "무게는 전에 뽑아서 잰 부품에 맞춘 어림입니다. D405는 60 g, 베어링 24 g, 서보는 따로입니다.",
        "why": [
            "카메라는 J6 앞(손목 2 몸통)에 답니다. 도구를 바꿔도 카메라가 그대로라 캘리브레이션을 다시 하지 않고, "
            "뻣뻣한 USB 선이 J6를 지나지 않습니다.",
            "광축을 도구 쪽으로 20° 기울여 도구 끝을 봅니다. 포인터를 달면 잡기 전 높이에서 도구 끝 둘레의 "
            "94 ~ 96%가 보입니다.",
            "받침은 꺾인 판 하나입니다. 판 두께 4.5 mm는 카메라가 자세에 따라 기우는 양을 화소 반 칸(0.034°)보다 "
            "작게 두는 값입니다.",
            "받침만 바꾸면 카메라의 자리와 기울기를 바꿀 수 있습니다. 손목 몸통은 그대로 둡니다.",
            "손목 1에도 같은 돋움이 있습니다 (손목 1과 손목 2는 같은 부품입니다). 거기에는 D405 선을 잡는 클립을 "
            "답니다.",
            "도구는 핀 2개가 자리를 잡고 손나사 2개가 조입니다. 하나는 둥근 구멍, 하나는 긴 구멍이라 두 핀이 서로 "
            "다투지 않습니다.",
        ],
        "todo": [
            "플러그 크기는 가정입니다. 가진 선의 플러그를 재서 확인합니다.",
            "평행 그리퍼를 달면 그리퍼가 화면을 조금 가립니다. 그린 그리퍼로는 손가락을 벌린 채 잡기 전 높이에서 "
            "86%가 보입니다 (평행 그리퍼 구조 페이지).",
            "D405를 켜 두었을 때의 몸통 온도를 보고, 받침을 PLA로 뽑을지 PETG-CF로 뽑을지 정합니다.",
            "선이 지나가는 길과 묶는 자리는 배선을 그릴 때 정합니다.",
        ],
        "frame": {"y0": -27, "y1": 62, "w": 150, "cx": 0},
        "pin_pose": 0,
        "axis": "위가 도구 쪽, 왼쪽이 앞 관절(J5) 쪽",
    }
    return {
        "meta": {"model": model, "stop": j.STOP_DEG},
        "parts": _meshes(parts, masses),
        "page": page,
    }


def gripper_data() -> dict:
    """평행 그리퍼: 손목 끝(플랜지, D405)에 단 모습과 조립 순서."""
    g = gripper
    by_file = {
        name: printed_mass_g(s, up, preset)
        for printed in (wrist.printed(), flange.printed(), camera_mount.printed())
        for name, (s, up, _c, preset) in printed.items()
    }
    grip_file = {
        name: printed_mass_g(s, up, preset) for name, (s, up, _c, preset, _n) in g.printed().items()
    }
    masses = {name: by_file[file] for name, file in WRIST_END_PRINTED.items() if file in by_file}
    masses |= {name: grip_file[file] for name, file in GRIPPER_PRINTED.items()}
    masses |= {"D405": 60.0, "그리퍼 서보": 57.2, "그리퍼 혼": 1.45}
    base = [
        p
        for p in camera_mount.assembly(g.MODEL)
        if p.name != "도구 어댑터 판" and not p.name.startswith("영점 핀")
    ]
    parts = base + g.gripper_parts(0.0)
    low = [5, 46]  # 랙 집 묶음: 5단계에서 몸통 묶음과 합친다 (hold)
    roles = [
        _role("그리퍼 몸통", 1, 0, shell=True),
        _role("그리퍼 서보", 1, 46),
        _role("그리퍼 혼", 1, 46),
        _role("너트 (몸통 돋움)", 2, 0, pre=True, rad=[1.4, 0, 0]),
        _role("바닥판 나사", 2, 44, pre=True),
        _role("바닥판", 2, 26, shell=True),
        _role("피니언 나사", 3, 46, pre=True, gear=True),
        _role("피니언", 3, 30, gear=True),
        _role("랙 집", 4, 0, shell=True, hold=low),
        _role("손가락 받침 1", 4, -24, slide=1, hold=low),
        _role("손가락 받침 2", 4, -24, slide=-1, hold=low),
        _role("손가락 1", 4, 30, slide=1, hold=low),
        _role("손가락 2", 4, 30, slide=-1, hold=low),
        _role("손가락 나사 M3x12 1", 4, -44, slide=1, hold=low),
        _role("손가락 나사 M3x12 2", 4, -44, slide=-1, hold=low),
        _role("너트 (손가락) 1", 4, 30, slide=1, rad=[1.2, 0, 0], hold=low),
        _role("너트 (손가락) 2", 4, 30, slide=-1, rad=[-1.2, 0, 0], hold=low),
        _role("귀 나사", 5, -34, pre=True),
        _role("끝벽 나사", 5, -34, pre=True),
        _role("그리퍼 플러그", 6, -26, hideUntil=6),
        _role("손나사", 6, 30, pre=True),
        _role("손잡이", 6, 30, pre=True),
    ]
    # 손목과 플랜지, 카메라는 맨 나중 단계에서 그리퍼 쪽으로 온다
    for p in parts:
        if not any(p.name == r["n"] or (r["pre"] and p.name.startswith(r["n"])) for r in roles):
            roles.append(_role(p.name, 6, -40))
    length, bite = g.pinion_screw()
    steps = [
        _step(
            "다 조립한 모습",
            "다 조립한 모습",
            "평행 그리퍼를 손목 끝에 단 모습입니다. 그림은 손가락이 아래로 가게 그렸습니다. '손가락 벌리기' 막대를 "
            "움직이면 가운데 피니언이 돌면서 랙 둘이 반대로 가고, 손가락 둘이 똑같이 벌어집니다. 다 벌리면 "
            f"{g.OPEN_MAX:.0f} mm입니다. 뒤에 보이는 어두운 상자가 D405 카메라입니다.",
        ),
        _step(
            "서보",
            "1. 서보를 몸통에 넣기",
            "그리퍼 서보(STS3215, 혼을 단 채)를 몸통의 틀에 밑에서 밀어 넣습니다. 커넥터가 있는 뒷면이 위(플랜지 "
            "쪽)로 가고, 커넥터가 몸통 윗판을 튼 자리로 보입니다. 힘을 주지 않아도 들어갑니다.",
        ),
        _step(
            "바닥판",
            "2. 바닥판",
            "몸통 옆의 돋움 둘에 너트를 옆에서 밀어 넣습니다. 바닥판을 서보 앞면에 대고, 밑에서 M3 × 8 나사 둘을 "
            "조입니다 (2 mm 육각렌치). 서보가 몸통과 바닥판 사이에 물립니다.",
        ),
        _step(
            "피니언",
            "3. 피니언",
            f"피니언의 목을 바닥판 구멍으로 넣어 혼 윗면에 앉히고, M3 × {length:.0f} 나사 넷을 조입니다. "
            "나사 머리는 피니언 속으로 들어가 면 밖으로 나오지 않습니다.",
        ),
        _step(
            "손가락",
            "4. 랙 집에 받침과 손가락",
            "랙 집을 덮개가 아래로 가게 놓고, 받침 둘을 통로에 넣어 가운데에서 서로 닿게 둡니다. 손가락 옆 홈에 "
            "너트를 넣고, 손가락의 목을 덮개의 긴 틈으로 밑에서 올려 받침 판에 댑니다. 받침 위에서 M3 × 12 나사를 "
            "하나씩 조입니다.",
        ),
        _step(
            "합치기",
            "5. 두 묶음 합치기",
            "손가락을 닫은 채로 랙 집을 바닥판 밑에 맞춥니다. 피니언의 이가 랙 둘에 같이 물리게, 안 들어가면 "
            "피니언을 손으로 조금 돌립니다. 위에서 나사 넷을 조입니다: 몸통의 귀 쪽은 M3 × 12 둘, 반대쪽 끝은 "
            "M3 × 8 둘입니다. 끝벽의 플라스틱에 바로 조이므로 세게 조이지 않습니다.",
        ),
        _step(
            "달기",
            "6. 플랜지에 달기",
            "먼저 서보 선을 꽂습니다 (몸통 윗판을 튼 자리로 커넥터가 보입니다). 그다음 플랜지의 핀 둘에 몸통을 "
            "끼우고 손나사 둘을 조입니다. 뗄 때는 손나사를 풀고 그리퍼를 핀에서 뺀 뒤에 선을 뽑습니다.",
        ),
    ]
    t = g.travel()
    dims = [
        ["열림 폭", f"0 ~ {g.OPEN_MAX:.0f} mm"],
        ["박스를 잡을 때의 열림 폭", f"{2 * g.grasp_x():.1f} mm"],
        ["플랜지 면에서 손가락 끝까지", f"{g.H_TIP:.1f} mm"],
        ["플랜지 면에서 도구 끝점(TCP)까지", f"{g.H_TCP:.1f} mm"],
        ["몸통 밑면에서 손가락 끝까지", f"{g.FINGER_LEN:.0f} mm"],
        ["랙 집 (길이 × 폭)", f"{2 * g.HOUSE_HL:.1f} × {2 * g.HOUSE_HW:.1f} mm"],
        ["무게 (서보, 나사, 선까지)", f"{sum(g.mass_g().values()):.0f} g"],
        ["이", f"모듈 {g.MODULE}, 피니언 {g.TEETH}개"],
        ["다 벌릴 때 서보가 도는 각", f"{t['피니언이 도는 각 [°]']:.0f}°"],
        ["서보 한 칸에 바뀌는 열림 폭", f"{t['서보 한 칸에 바뀌는 열림 폭 [mm]']:.3f} mm"],
        ["받침과 통로 사이 위아래 틈", f"{2 * g.SLIDE_GAP:.1f} mm"],
        ["박스 구멍 입구", f"지름 {2 * g.BOX_LIP_R:.0f}, 깊이 {g.BOX_LIP_T:.0f} mm"],
        ["박스 구멍 속", f"지름 {2 * g.BOX_CAVITY_R:.1f}, 깊이 {g.BOX_HOLE_DEPTH:.0f} mm"],
        ["피니언 나사", f"M3 × {length:.0f} 넷 (혼에 {bite:.1f} mm)"],
        ["그 밖의 나사", "M3 × 8 넷, M3 × 12 넷"],
        ["너트", "넷"],
        ["뽑는 파일 (검정, Strength)", "gripper_frame"],
        ["", "gripper_floor"],
        ["", "gripper_tub"],
        ["", "gripper_pinion"],
        ["", "gripper_slider × 2"],
        ["", "gripper_finger × 2"],
    ]
    page = {
        "title": "평행 그리퍼 구조",
        "h1": "평행 그리퍼 구조",
        "sub": "첫 그리퍼입니다. 서보 하나가 손가락 둘을 대칭으로 벌리고 오므립니다. 박스는 윗면 구멍 안에서 벌려 잡고, "
        "가벼운 물체는 바깥에서 오므려 잡습니다. 플랜지의 핀 둘과 손나사 둘로 갈아 끼웁니다.",
        "steps": steps,
        "roles": roles,
        "dims": dims,
        "mass_note": "뽑는 부품의 무게는 전에 뽑아서 잰 부품에 맞춘 어림입니다. 서보와 혼은 잰 값입니다.",
        "why": [
            "서보를 세워 두고 랙을 그 밑에 납작하게 넣었습니다. 손가락 끝까지의 거리가 짧아서 손가락이 받는 힘에 "
            "받침이 덜 기울고, 양쪽 손가락이 똑같이 움직입니다.",
            "받침은 긴 랙 팔과 넓은 판이 한 부품입니다. 바닥판과 덮개 사이에 끼여 옆으로만 미끄러지고, 틈은 "
            "층 높이로 정해져서(0.1 mm씩) 프린터가 달라도 잘 맞습니다.",
            "이는 통로 속에 숨어 있어 선이나 물체가 끼지 않습니다.",
            "박스 구멍은 입구 밑이 45°로 넓어집니다. 손가락 끝의 턱이 그 경사에 걸리면서 박스를 손가락 어깨 쪽으로 "
            "당겨 붙입니다. 박스가 흔들리지 않고, 놓을 때 위에서 눌러 줄 수 있습니다.",
            f"손가락 길이 {g.FINGER_LEN:.0f} mm는 D405가 잡기 전 높이(6 cm 위)에서 도구 끝 둘레를 80% 넘게 보는 "
            "길이입니다. 랙 집의 밑 모서리를 깎은 것도 카메라가 그 너머로 보게 하려는 것입니다.",
            "서보 선은 몸통 윗판을 튼 자리로 꽂습니다. 서보를 그만큼 플랜지 가까이 올려서 그리퍼가 약 15 mm "
            "짧아졌습니다.",
            "몸통의 서보 먼 끝 쪽 밑은 비스듬히 깎았습니다. 손목의 앞 관절(J5) 원판에 달린 멈춤 귀의 끝이 플랜지 "
            "면의 높이까지 올라와 있어서, 그리퍼가 J6를 따라 돌 때 그 위를 2.5 mm 띄우고 지나가게 한 것입니다.",
            "손가락은 나사 하나로 달려 있어 다른 모양으로 바꿀 수 있습니다 (끝벽 나사 넷을 풀고 랙 집을 뗀 뒤에 "
            "바꿉니다).",
        ],
        "todo": [
            "혼 높이를 잰 값이 오면 피니언의 목 길이와 나사 길이를 맞춥니다.",
            "검정 공차 시험편 값(너트 홈, 구멍)을 넣은 뒤에 뽑을 파일을 냅니다.",
            "쥐는 힘의 한도(서보 토크 한도)는 그리퍼 서보의 전류를 잰 뒤에 정합니다. 한도 없이 끝까지 밀면 "
            "이가 상할 수 있습니다.",
            "바깥에서 잡을 때 미끄러우면 손가락 끝에 씌우는 TPU 패드를 더합니다 (필라멘트를 사야 합니다).",
            "서보 선이 플랜지 둘레를 감는 길과 묶는 자리는 배선을 그릴 때 정합니다.",
        ],
        "frame": {"y0": -134, "y1": 27, "w": 170, "cx": 0},
        "pin_pose": 0,
        "axis": "아래가 손가락 끝, 위가 플랜지와 손목",
        "flip": True,
        "grip": {"max": g.OPEN_MAX, "rp": g.RP, "start": 0},
    }
    return {
        "meta": {"model": g.MODEL, "stop": j.STOP_DEG},
        "parts": _meshes(parts, masses),
        "page": page,
    }


def render(kind: str = "first") -> str:
    """kind가 "first"면 첫 관절, "shoulder"면 어깨, "base"면 J1, "wrist_end"면 손목 끝, "gripper"면 그리퍼."""
    makers = {
        "shoulder": shoulder_data,
        "base": base_data,
        "wrist_end": wrist_end_data,
        "gripper": gripper_data,
    }
    data = makers.get(kind, mesh_data)()
    template = (HERE / "joint_page.html").read_text(encoding="utf-8")
    body = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    # <script type="application/json"> 안이라 "<"만 바꿔 둔다
    page = template.replace("__MESH_JSON__", body.replace("<", "\\u003c"))
    return page.replace("__TITLE__", data["page"]["title"])


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for kind, name in (
        ("first", "joint.html"),
        ("shoulder", "shoulder.html"),
        ("base", "base.html"),
        ("wrist_end", "wrist_end.html"),
        ("gripper", "gripper.html"),
    ):
        out = OUT_DIR / name
        out.write_text(render(kind), encoding="utf-8", newline="\n")
        print(f"썼다: {out} ({out.stat().st_size / 1000:.0f} kB)")


if __name__ == "__main__":
    main()
