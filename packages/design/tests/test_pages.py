"""3D 페이지 데이터: 틀이 기대하는 모양으로 나오는지, 박스 쌓기 자세가 다 풀리는지."""

from __future__ import annotations

import pytest

from horilab_design.pages import HERE, arm_page, joint_page
from horilab_design.parts import camera_mount, gripper


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
def test_gripper_page_has_every_part_and_moves_the_fingers() -> None:
    names = tuple(p.name for p in gripper.gripper_parts())
    assert "그리퍼 몸통" in names and "피니언" in names and "손가락 받침 2" in names
    data = joint_page.gripper_data()
    _check_joint_page(data, names)
    page = data["page"]
    assert page["grip"]["max"] == gripper.OPEN_MAX and page["flip"] is True
    # 손가락과 받침, 그 나사는 양쪽이 반대로 가고, 피니언과 그 나사는 돈다
    slide = {r["n"]: r.get("slide") for r in page["roles"]}
    for k, sign in ((1, 1), (2, -1)):
        assert slide[f"손가락 {k}"] == sign and slide[f"손가락 받침 {k}"] == sign
        assert slide[f"손가락 나사 M3x12 {k}"] == sign and slide[f"너트 (손가락) {k}"] == sign
    assert all(r.get("gear") for r in page["roles"] if r["n"].startswith("피니언"))
    # 그리퍼를 단 그림에는 도구 어댑터 판이 없다 (몸통의 윗판이 그 구실이다)
    assert "도구 어댑터 판" not in {p["n"] for p in data["parts"]}


@pytest.mark.slow
def test_arm_page_solves_every_box_stacking_pose() -> None:
    data = arm_page.page_data()
    variant = data["variants"][0]
    assert len(variant["chain"]) == 6
    assert all(len(f["q"]) == 6 for f in variant["frames"])
    assert {p["s"] for p in variant["prims"]} <= {"box", "cyl", "ring", "dtube", "prism"}
    # 팔꿈치 위, J5 > 0인 작업 갈래로만 푼다
    assert all(f["q"][2] > 0 and f["q"][4] > 0 for f in variant["frames"])
    # 도구는 검토 모델의 그린 평행 그리퍼다: 플랜지에 앉는 판, 틀, 랙 집, 손가락
    names = {p["n"] for p in variant["prims"]}
    assert {"adapter", "gripper_frame", "gripper_house", "finger+1_tip", "finger-1_tip"} <= names
    assert not any("_mass_" in n for n in names)
    assert variant["tcp_mm"] == pytest.approx(gripper.H_TCP)
    # 박스를 잡는 자리의 도구 끝점은 박스 윗면에서 손가락 어깨까지의 깊이만큼 아래다
    grips = [f for f in variant["frames"] if f["ev"] == "grip"]
    depth = gripper.H_TCP - gripper.H_SHOULDER
    assert grips and all(f["tcp"][2] == pytest.approx(data["box_mm"] - depth) for f in grips)
    # D405는 기울인 자리다: 광축이 도구 축(링크 5의 +y)에서 J6 축 쪽(-x)으로 20° 누워 있다
    assert variant["cam"]["axis"] == pytest.approx([-0.342, 0.94, 0.0], abs=1e-3)
    for prism in (p for p in variant["prims"] if p["s"] == "prism"):
        assert len(prism["pts"]) >= 3 and prism["z"][0] > 0
