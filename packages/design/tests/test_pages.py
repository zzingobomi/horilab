"""3D 페이지 데이터: 틀이 기대하는 모양으로 나오는지, 박스 쌓기 자세가 다 풀리는지."""

from __future__ import annotations

import pytest

from horilab_design.pages import HERE, arm_page, joint_page
from horilab_design.parts import camera_mount


def test_templates_have_one_data_slot() -> None:
    joint = (HERE / "joint_page.html").read_text(encoding="utf-8")
    assert joint.count("__MESH_JSON__") == 1
    assert joint.count("__TITLE__") == 1
    assert (HERE / "arm_page.html").read_text(encoding="utf-8").count("__DATA__") == 1


def _check_joint_page(data: dict, names: tuple[str, ...]) -> None:
    """부품 메시가 온전하고, 조립 단계가 이름으로 모든 부품을 찾는다."""
    found = {p["n"] for p in data["parts"]}
    for name in names:
        assert name in found
    page = data["page"]
    steps = len(page["steps"])
    for p in data["parts"]:
        assert len(p["v"]) % 3 == 0 and len(p["i"]) % 3 == 0, p["n"]
        assert max(p["i"]) < len(p["v"]) // 3, p["n"]
        roles = [
            r for r in page["roles"] if p["n"] == r["n"] or (r["pre"] and p["n"].startswith(r["n"]))
        ]
        assert roles, f"조립 단계가 없는 부품: {p['n']}"
        assert 0 <= roles[0]["step"] < steps, p["n"]
    assert page["frame"]["y1"] > page["frame"]["y0"]


@pytest.mark.slow
def test_joint_page_has_every_part_of_the_first_joint() -> None:
    names = (
        "손목 몸통 (부모)",
        "손목 몸통 (자식)",
        "출력 원판",
        "허브 판",
        "누름 틀",
        "서보 받침",
        "뒷덮개",
    )
    _check_joint_page(joint_page.mesh_data(), names)


@pytest.mark.slow
def test_shoulder_page_has_every_part_of_the_shoulder() -> None:
    names = ("어깨 집", "위팔", "출력 원판", "허브 판", "누름 틀", "서보 받침", "뒷덮개")
    _check_joint_page(joint_page.shoulder_data(), names)


@pytest.mark.slow
def test_base_page_has_every_part_of_j1() -> None:
    names = (
        "베이스 몸통",
        "책상에 무는 판",
        "돌림판",
        "허브",
        "사이 고리",
        "누름 틀",
        "서보 받침",
        "J1 몸통",
        "책상",
    )
    _check_joint_page(joint_page.base_data(), names)


@pytest.mark.slow
def test_wrist_end_page_has_every_part_of_the_flange_and_camera_mount() -> None:
    names = tuple(p.name for p in camera_mount.assembly())
    assert "D405 받침" in names and "플랜지 원판" in names and "도구 어댑터 판" in names
    _check_joint_page(joint_page.wrist_end_data(), names)


@pytest.mark.slow
def test_arm_page_solves_every_box_stacking_pose() -> None:
    data = arm_page.page_data()
    variant = data["variants"][0]
    assert len(variant["chain"]) == 6
    assert all(len(f["q"]) == 6 for f in variant["frames"])
    assert {p["s"] for p in variant["prims"]} <= {"box", "cyl", "ring", "dtube"}
    # 팔꿈치 위, J5 > 0인 작업 갈래로만 푼다
    assert all(f["q"][2] > 0 and f["q"][4] > 0 for f in variant["frames"])
