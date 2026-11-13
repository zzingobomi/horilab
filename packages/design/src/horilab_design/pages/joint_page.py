"""관절 3D 페이지: 조립 단계, 펼치기, 관절 돌려 보기, 반으로 잘라 보기. 첫 관절(손목 몸통 둘)과 어깨를 만든다.

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
from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
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
    "어깨 집": "shoulder_house",
    "허브 판": "shoulder_hub_plate",
    "출력 원판": "shoulder_out_disc",
    "누름 틀": "shoulder_retainer",
    "서보 받침": "shoulder_carrier",
    "뒷덮개": "shoulder_cover",
}
ARM_STUB = 62.0  # 어깨 페이지에 보이는 위팔의 길이 (관절 축에서 관 쪽으로)


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
            "선은 살 옆으로 나와 출력 원판 둘레에 감기며 자식으로 넘어갑니다. 두 몸통 사이에 숨고, 양쪽 160°까지 끼지 "
            "않습니다. 가운데 자세에서 약 7 cm가 느슨한 고리로 남습니다. 손목 몸통은 앞 벽의 발 쪽 끝에 선 홈이 둘 있습니다.",
            "커넥터 앞쪽은 서보 받침을 터서 플러그를 손톱이나 핀셋으로 잡을 수 있습니다. 뒷덮개(나사 2개)만 떼면 닿습니다.",
            "쓰는 나사는 모두 가진 것입니다 (M3 × 8, × 10, × 12, × 20, M2 × 6, M3 너트). STS3215와 STS3250 둘 다 같은 "
            "부품으로 되고, CAD에서 겹침이 없습니다.",
        ],
        "todo": [
            "서보 혼이 앞면 판보다 얼마나 높은지를 다시 재서, 관절 속 서보 자리를 그 값으로 옮깁니다.",
            "선은 그림에 넣지 않았습니다. 묶는 자리와 D405 선은 배선을 그릴 때 정합니다.",
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
            "목의 아래 끝이 J1 몸통(돌림판)과 만나는 자리는 베이스를 그리며 정합니다.",
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


def render(kind: str = "first") -> str:
    """kind가 "first"면 첫 관절, "shoulder"면 어깨."""
    data = shoulder_data() if kind == "shoulder" else mesh_data()
    template = (HERE / "joint_page.html").read_text(encoding="utf-8")
    body = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    # <script type="application/json"> 안이라 "<"만 바꿔 둔다
    page = template.replace("__MESH_JSON__", body.replace("<", "\\u003c"))
    return page.replace("__TITLE__", data["page"]["title"])


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for kind, name in (("first", "joint.html"), ("shoulder", "shoulder.html")):
        out = OUT_DIR / name
        out.write_text(render(kind), encoding="utf-8", newline="\n")
        print(f"썼다: {out} ({out.stat().st_size / 1000:.0f} kB)")


if __name__ == "__main__":
    main()
