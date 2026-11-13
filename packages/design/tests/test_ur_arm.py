"""UR식 관절 팔의 검토 모델: 쌓임에서 나온 옆 자리, 무게와 손목 끝 형상이 CAD와 맞는지, 세운 자세."""

from __future__ import annotations

import math

import numpy as np
import pytest

from horilab_design.parts import camera_mount as cmt
from horilab_design.parts import flange, links, wrist
from horilab_design.parts import gripper as gp
from horilab_design.parts import joint as j
from horilab_design.printcheck import printed_mass_g
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur_arm
from horilab_design.studies.packaging import DEG, MM

UP = (0.0, -1.0, 0.0)  # 몸통은 원통 앞 끝을 바닥에 두고 뽑는다


def test_links_step_sideways_by_the_joint_stack() -> None:
    c = ur_arm.Concept()
    # 아래팔의 납작한 면은 위팔의 납작한 면에서 틈 2 + 출력 원판 테만큼 안쪽이고, 손목 1은 다시 위팔의 평면이다
    assert c.ya2 - c.yb3 == pytest.approx((j.GAP + j.BRIM_T) * MM)
    assert c.yb4 == pytest.approx(c.ya2)
    # 손목 길이: 출력 원판 윗면 + 틈 + 서보 먼 끝
    assert c.d4 - c.yb4 == pytest.approx((wrist.HEAD + j.FAR) * MM)
    assert c.d5 == pytest.approx((j.ATOP - wrist.AX_Y + wrist.HEAD + j.FAR) * MM)
    # 플랜지 면은 J6 출력 원판(플랜지 원판)의 윗면이다
    assert c.d6 == pytest.approx((j.ATOP - wrist.AX_Y) * MM)


def test_upright_arm_is_clear_and_stops_on_the_webs() -> None:
    info = ur_arm.check(ur_arm.Concept(), verbose=False)
    assert info["upright_hits"] == []
    # J2가 드는 무게 예산 1,050 g (그리퍼 130, D405 60 포함. 선과 형상에 없는 것은 빼고)
    assert info["j2_load"] - info["j2_wires"] < 1.05
    # 선의 무게 (0031): 서보 선과 D405 선을 더해 60 g 안이다
    assert 0.03 < info["j2_wires"] < 0.06
    # 멈춤 기둥과 홈이 형상에 들어 있다: 다른 것에 먼저 닿지 않는 관절은 멈춤 각에서 선다 (J4는 180° 둘레)
    ranges = info["ranges"]
    for joint in (4, 6):
        lo, hi, why_lo, why_hi = ranges[joint - 1]
        assert (lo, hi) == pytest.approx((-j.STOP_DEG, j.STOP_DEG), abs=1.0)
        assert why_lo == why_hi == f"J{joint} 귀와 나사 ↔ J{joint} 집"
    assert ranges[1][1] == pytest.approx(j.STOP_DEG, abs=1.0)
    # 프로그램 한계(160°)까지는 어느 관절도 자기 멈춤에 닿지 않는다
    for lo, hi, why_lo, why_hi in ranges[1:]:
        for end, why in ((lo, why_lo), (hi, why_hi)):
            pair = why.split(" ↔ ")
            own_stop = len(pair) == 2 and pair[0].split(" ")[0] == pair[1].split(" ")[0]
            assert not own_stop or abs(end) > j.SOFT_DEG, (end, why)


def test_model_includes_what_sticks_out_of_the_joints() -> None:
    # 부딪힘 검사에 원통 밖으로 나온 것이 들어 있다: 벽 나사 머리, 멈춤 기둥과 그 귀, 자식 몸통의 귀, 영점 핀 귀
    _robot, prims = ur_arm.build(ur_arm.Concept(), ur_arm.GRIPPER)
    by = {p.name: p for p in prims}
    for joint in range(2, 7):
        for part in ("wallscrews", "stopear", "post", "post_tip", "web", "pinear"):
            assert f"J{joint}/{part}" in by, (joint, part)
    for joint in range(2, 6):
        assert f"J{joint}/ears0" in by and f"J{joint}/ears1" in by
    # 멈춤 기둥과 귀는 자식(도는 쪽) 링크에, 홈의 살과 벽 나사는 부모 링크에 있다
    assert by["J4/stopear"].link == 4 and by["J4/post"].link == 4
    assert by["J4/wallscrews"].link == 3 and by["J4/web"].link == 3
    # 범위의 가운데에서 기둥이 살의 맞은편에 온다: J3는 관절 각 0에서 서보 가까운 끝(+z) 쪽, J4는 180°가
    # 가운데라 관절 각 0에서는 목(-z) 쪽이다
    assert by["J3/post"].pos[2] > 0.02
    assert by["J4/post"].pos[2] < -0.02
    # 손목 몸통 둘의 돋움, D405와 받침과 플러그, 그리퍼
    for name in ("J5/boss0", "J6/boss1", "d405", "d405_plate", "d405_seat", "d405_plug", "adapter"):
        assert name in by, name
    assert by["d405_plate"].mass == pytest.approx(ur_arm.D405_PLATE_MASS)
    assert by["J6/cap"].mass == pytest.approx(ur_arm.FLANGE_MASS)


def test_camera_sits_where_the_mount_puts_it() -> None:
    c = ur_arm.Concept()
    cam = ur_arm.camera(c)
    # 렌즈 면 가운데: J6 축에서 옆으로(링크 5의 +x), 플랜지 면보다 LENS_BACK 뒤, J6의 높이
    assert cam.lens == pytest.approx([-cmt.LENS[2] * MM, c.d6 - cmt.LENS_BACK * MM, c.d5], abs=1e-9)
    t = math.radians(cmt.TILT)
    assert cam.axis == pytest.approx([-math.sin(t), math.cos(t), 0.0], abs=1e-9)
    # 플러그는 J5 쪽(링크 5의 -z)으로 나가고, 곧은 플러그가 ㄱ자보다 길다
    lengths = {}
    for plug in ("angled", "straight", "none"):
        _robot, prims = ur_arm.build(ur_arm.Concept(plug=plug), ur_arm.GRIPPER)
        found = [p for p in prims if p.name == "d405_plug"]
        if plug == "none":
            assert found == []
            continue
        cam_box = next(p for p in prims if p.name == "d405")
        assert found[0].pos[2] < cam_box.pos[2] - 0.021
        lengths[plug] = 2 * found[0].size[0]
    assert lengths["angled"] == pytest.approx(cmt.PLUG_ANGLED * MM)
    assert lengths["straight"] == pytest.approx((cmt.PLUG_LEN + cmt.PLUG_RELIEF) * MM)


def test_gripper_tool_is_the_drawn_gripper() -> None:
    tool = ur_arm.TOOLS["gripper"]
    assert tool is ur_arm.GRIPPER
    assert tool.length == pytest.approx(gp.H_TCP * MM)
    assert tool.box_depth == pytest.approx((gp.H_TCP - gp.H_SHOULDER) * MM)
    assert tool.mass == pytest.approx(sum(m for m, _ in ur_arm.GRIPPER_LUMPS.values()))
    # 팔에 단 형상의 무게와 무게 중심이 덩어리 값 그대로다 (도구 좌표의 H는 링크 6의 y, x는 z)
    c = ur_arm.Concept()
    _robot, prims = ur_arm.build(c, tool)
    lumps = [p for p in prims if p.link == 6 and p.group in ("도구", "교체 어댑터") and p.mass > 0]
    assert sum(p.mass for p in lumps) == pytest.approx(tool.mass)
    total = sum(m for m, _ in ur_arm.GRIPPER_LUMPS.values())
    height = sum(m * at[1] for m, at in ur_arm.GRIPPER_LUMPS.values()) / total
    assert sum(p.mass * (p.pos[1] - c.d6) for p in lumps) / tool.mass == pytest.approx(height * MM)
    # 손가락 끝이 도구 끝점보다 TCP_UP 더 나가 있다
    tips = [p for p in prims if p.name.endswith("_tip") and p.name.startswith("finger")]
    assert len(tips) == 2
    assert max(p.pos[1] + p.size[1] for p in tips) - c.d6 == pytest.approx(gp.H_TIP * MM)


def test_gripper_passes_over_the_j5_stop_ear_when_j6_turns() -> None:
    # J5 출력 원판의 멈춤 귀 끝은 플랜지 면 높이까지 온다. J6를 돌려도 그리퍼가 그 위를 2 mm 넘게 띄우고 지나간다
    import mujoco

    c = ur_arm.Concept()
    robot, prims = ur_arm.build(c, ur_arm.GRIPPER)
    scene = pk.build_scene(robot, prims)
    m, d = scene.model, scene.data
    tool = [g for g, p in scene.geom_prim.items() if p.group in ("도구", "교체 어댑터")]
    ear = [g for g, p in scene.geom_prim.items() if p.name in ("J5/stopear", "J5/post")]
    assert tool and len(ear) == 2
    fromto = np.zeros(6)
    gap = math.inf
    for deg in range(-160, 161, 5):
        d.qpos[:] = np.array([0.0, 0.0, 0.0, 180.0, 0.0, deg]) * DEG
        mujoco.mj_forward(m, d)
        gap = min(
            gap, *(mujoco.mj_geomDistance(m, d, a, b, 0.05, fromto) for a in tool for b in ear)
        )
    assert gap >= pk.MOVING_GAP


def test_outcome_sorts_poses_by_what_stops_them() -> None:
    h = ur_arm.hori(ur_arm.Concept())
    length = ur_arm.GRIPPER.length
    assert ur_arm.outcome(h, pk.tool_pose((0.22, 0.0, 0.10), 0.0, 0.0, 0.0, length)) == ("됨", "")
    assert ur_arm.outcome(h, pk.tool_pose((0.60, 0.0, 0.10), 0.0, 0.0, 0.0, length))[0] == "못 닿음"
    # 손가락 끝이 책상 밑으로 들어가는 자세는 부딪힘이다
    assert ur_arm.outcome(h, pk.tool_pose((0.22, 0.0, -0.02), 0.0, 0.0, 0.0, length))[0] == "부딪힘"


def test_fit_ranges_moves_angles_into_the_joint_ranges() -> None:
    lower, upper = (
        np.radians([-165, -160, -160, 20, -160, -160]),
        np.radians([165, 160, 160, 340, 160, 160]),
    )
    q = np.radians([0.0, 10.0, 20.0, -170.0, 30.0, 170.0])
    fit = ur_arm.fit_ranges(q, lower, upper)
    assert fit is not None
    assert np.degrees(fit[3]) == pytest.approx(190.0)  # J4는 한 바퀴 더해 범위 안으로
    assert np.degrees(fit[5]) == pytest.approx(-10.0)  # 대칭 도구는 J6를 180° 돌려도 같다
    assert ur_arm.fit_ranges(np.radians([0.0, 170.0, 0.0, 180.0, 0.0, 0.0]), lower, upper) is None


def test_camera_sees_past_the_gripper_in_the_arm_model() -> None:
    c = ur_arm.Concept()
    wide, grasp = ur_arm.views(c, ur_arm.gripper(gp.OPEN_MAX)), ur_arm.views(c)
    # 손가락이 벌어지는 방향이 카메라 쪽과 직각일 때(J6 0°) 잡기 전 높이에서 도구 끝점 둘레의 3/4 넘게 본다 (0017).
    # 손가락을 벌리면 더 보인다
    assert wide[0][0] >= 0.80
    assert 0.75 <= grasp[0][0] <= wide[0][0]
    # J6를 90° 돌리면 랙 집의 긴 쪽이 카메라를 가린다
    assert wide[90][0] < 0.2
    # 끝점은 화면 가운데 가까이 있다 (좁은 쪽 시야 절반 29°)
    assert wide[0][2] < 10 * DEG


@pytest.mark.slow
def test_camera_view_agrees_with_the_cad_ray_cast() -> None:
    # 상자와 기둥으로 넣은 그리퍼가 실제 형상으로 센 값(parts/gripper.py)과 점 두 줄(7 × 7점) 안으로 맞는다
    seen = ur_arm.views(ur_arm.Concept(), ur_arm.gripper(gp.OPEN_MAX), turns=(0,))[0]
    assert seen[0] == pytest.approx(gp.view_share(gp.OPEN_MAX), abs=0.06)
    assert seen[1] == pytest.approx(gp.view_share(gp.OPEN_MAX, 2.0), abs=0.06)


@pytest.mark.slow
def test_mass_constants_match_the_cad() -> None:
    assert printed_mass_g(wrist.wrist_body(), UP) == pytest.approx(
        ur_arm.WRIST_BODY_MASS * 1000, abs=1.0
    )
    inner = sum(
        printed_mass_g(s, up, preset)
        for n, (s, up, _c, preset) in wrist.printed().items()
        if n in ("retainer", "carrier", "wrist_cover")
    )
    assert inner + 8.0 == pytest.approx(ur_arm.INNER_MASS * 1000, abs=1.5)  # 나사와 너트 8.0
    out = sum(
        printed_mass_g(s, up)
        for n, (s, up, _c, _p) in wrist.printed().items()
        if n in ("out_disc", "hub_plate")
    )
    assert out + 13.3 == pytest.approx(ur_arm.OUT_MASS * 1000, abs=1.5)  # 혼 2.0, 볼트와 너트 11.3
    # J6: 플랜지 원판과 허브 판에 혼 2.0, 혼 볼트 5.4, 너트 둘 0.7, 멈춤 나사와 너트 1.2, 도구 핀 둘 2.2
    disc = printed_mass_g(flange.flange_disc(), UP) + printed_mass_g(j.hub_plate(), UP)
    assert disc + 11.5 == pytest.approx(ur_arm.FLANGE_MASS * 1000, abs=1.0)
    for key, name in (("upper", "upper_arm"), ("fore", "forearm")):
        length, tube_r, foot = links.LINKS[name]
        k, m_foot, _y = ur_arm.LINK_TUBE[key]
        model = ur_arm.LINK_HOUSE_MASS + k * (length - foot.hx - 40.0) * MM + m_foot
        assert printed_mass_g(links.link(length, tube_r, foot), UP) == pytest.approx(
            model * 1000, abs=2.0
        )


@pytest.mark.slow
def test_gripper_lumps_match_the_cad() -> None:
    cad = gp.mass_centers()
    assert set(cad) == set(ur_arm.GRIPPER_LUMPS)
    for name, (grams, at) in cad.items():
        mass, model_at = ur_arm.GRIPPER_LUMPS[name]
        assert mass * 1000 == pytest.approx(grams, abs=0.3), name
        assert model_at == pytest.approx(at, abs=0.5), name
    assert sum(g for g, _ in cad.values()) == pytest.approx(sum(gp.mass_g().values()), abs=0.01)
