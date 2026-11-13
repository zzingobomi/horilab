"""손목 D405 받침과 손목 몸통의 돋움: 뽑을 수 있는 형상인지, 겹치지 않는지, J6를 돌릴 때 닿지 않는지."""

from __future__ import annotations

import pytest

from horilab_design.parts import camera_mount as cm
from horilab_design.parts import joint as j
from horilab_design.parts import wrist
from horilab_design.printcheck import overhangs, printed_mass_g
from horilab_design.studies import ur_arm


@pytest.fixture(scope="module")
def parts() -> list[j.Part]:
    return cm.assembly("STS3215")


@pytest.mark.slow
def test_mount_is_a_single_valid_solid_printed_on_its_side() -> None:
    shape, up, _color, _preset = cm.printed()["d405_mount"]
    assert shape.is_valid
    assert len(shape.solids()) == 1
    # 옆으로 눕혀 뽑으면 걸치는 면은 가로 구멍 넷의 윗변뿐이다
    over = overhangs(shape, up)
    assert over.total < 60.0, (over.total, over.flat)


@pytest.mark.slow
def test_camera_and_mount_do_not_overlap_the_wrist(parts: list[j.Part]) -> None:
    solids = [p for p in parts if "핀" not in p.name and "손잡이" not in p.name]
    assert j.overlaps(solids) == {}
    cam = next(p for p in parts if p.name == "D405")
    body = next(p for p in parts if p.name == "손목 몸통")
    assert cam.shape.distance_to(body.shape) == pytest.approx(cm.DRUM_GAP, abs=0.05)
    # 받침의 발은 돋움 면에 앉는다 (틈 없이 닿는다)
    mount = next(p for p in parts if p.name == "D405 받침")
    assert mount.shape.distance_to(body.shape) < 0.01


@pytest.mark.slow
def test_turning_parts_clear_the_camera_by_2mm(parts: list[j.Part]) -> None:
    # J6를 멈춤 각까지 돌려도 플랜지 원판, 멈춤 기둥의 나사와 너트, 도구 어댑터가 카메라와 받침에서 2 mm 넘게 뜬다
    gaps = cm.sweep_gaps(parts)
    assert min(gaps.values()) >= 2.0, gaps


def test_screws_engage_and_do_not_bottom() -> None:
    c = cm.screw_checks()
    # 카메라 뒷면 구멍은 나사가 4.0까지만 들어간다
    assert 2.5 <= c["카메라 나사가 카메라에 들어간 길이"] <= cm.CAM_HOLE_DEPTH - 0.5
    assert c["받침 나사가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert c["받침 나사 끝과 나사 구멍 바닥 사이"] >= 0.5
    # 돋움의 나사 구멍은 원통 벽에 닿지 않는다
    assert c["돋움 나사 구멍 바닥과 원통 옆면 사이"] >= 1.0


def test_plate_keeps_the_camera_within_half_a_pixel() -> None:
    # 카메라 무게에 판이 휘어 기우는 각이, 탄성 계수가 낮은 쪽(2.0 GPa)이어도 화소 반 칸보다 작다
    assert cm.tilt_under_weight(2.0) < cm.PIXEL_DEG / 2


def test_bosses_stay_clear_of_the_cover_rim_and_the_lock_screw() -> None:
    # 돋움은 뒷덮개 테(원통 뒤 끝을 2.5 겹친다)보다 앞에 있다
    assert wrist.MOUNT_Y[0] > j.BACK_END + j.COVER_SKIRT + 0.5
    # 서보를 잠그는 벽 나사(x = 0, 머리 반지름 2.85)와 옆으로 1 mm 넘게 떨어져 있다
    assert wrist.MOUNT_X - wrist.MOUNT_HW > 2.85 + 1.0
    # 너트 홈이 돋움 안에 있다: 양옆 벽 1.5 넘게, 바닥 1.0 넘게
    w, _t, _roof = wrist.MOUNT_NUT
    assert wrist.MOUNT_HW - w / 2 >= 1.5
    assert wrist.MOUNT_Y[1] - (wrist.MOUNT_SCREW_Y + w / 2 / 0.866) >= 1.0
    # 받침의 발 나사 머리가 발과 판 사이를 메운 자리에 걸리지 않는다
    shape = cm.mount()
    for x, y in wrist.mount_xy():
        head_edge = (x, y + 2.85 + 0.2, -(wrist.MOUNT_Z + cm.FOOT_T) - 0.5)
        assert not shape.solids()[0].is_inside(head_edge)


@pytest.mark.slow
def test_arm_model_uses_the_mount_masses() -> None:
    shape, up, _color, preset = cm.printed()["d405_mount"]
    total = printed_mass_g(shape, up, preset) + sum(p.mass_g for p in cm.hardware())
    assert total == pytest.approx(ur_arm.D405_PLATE_MASS * 1000, abs=0.5)
