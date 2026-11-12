"""3D 페이지 데이터: 틀이 기대하는 모양으로 나오는지, 박스 쌓기 자세가 다 풀리는지."""

from __future__ import annotations

import pytest

from horilab_design.pages import HERE, arm_page, joint_page


def test_templates_have_one_data_slot() -> None:
    assert (HERE / "joint_page.html").read_text(encoding="utf-8").count("__MESH_JSON__") == 1
    assert (HERE / "arm_page.html").read_text(encoding="utf-8").count("__DATA__") == 1


@pytest.mark.slow
def test_joint_page_has_every_part_of_the_first_joint() -> None:
    data = joint_page.mesh_data()
    names = {p["n"] for p in data["parts"]}
    for name in (
        "손목 몸통 (부모)",
        "손목 몸통 (자식)",
        "출력 원판",
        "허브 판",
        "누름 틀",
        "서보 받침",
        "뒷덮개",
    ):
        assert name in names
    for p in data["parts"]:
        assert len(p["v"]) % 3 == 0 and len(p["i"]) % 3 == 0, p["n"]
        assert max(p["i"]) < len(p["v"]) // 3, p["n"]
    # 틀의 조립 단계가 이름으로 부품을 찾는다
    template = (HERE / "joint_page.html").read_text(encoding="utf-8")
    for name in (
        "손목 몸통 (부모)",
        "손목 몸통 (자식)",
        "출력 원판",
        "허브 판",
        "누름 틀",
        "서보 받침",
        "뒷덮개",
    ):
        assert name in template, name


@pytest.mark.slow
def test_arm_page_solves_every_box_stacking_pose() -> None:
    data = arm_page.page_data()
    variant = data["variants"][0]
    assert len(variant["chain"]) == 6
    assert all(len(f["q"]) == 6 for f in variant["frames"])
    assert {p["s"] for p in variant["prims"]} <= {"box", "cyl", "ring", "dtube"}
    # 팔꿈치 위, J5 > 0인 작업 갈래로만 푼다
    assert all(f["q"][2] > 0 and f["q"][4] > 0 for f in variant["frames"])
