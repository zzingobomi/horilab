"""검토 코드가 깨지지 않았는지 확인한다. 결론(어느 대안이 낫나)은 질량 가정에 달려서 여기서 보지 않는다."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

from horilab_design.ik import AnalyticIK
from horilab_design.robot import flange_pose, link_poses
from horilab_design.statics import PointMass, frame_poses
from horilab_design.studies import (
    cameras,
    covers,
    packaging,
    reach,
    skeleton,
    ur3e,
    ur_compare,
    ur_joints,
)
from horilab_design.studies.common import (
    MASS_CASES,
    RATED_MARGIN,
    TASK_YAWS_DEG,
    pick_solution,
    point_masses,
    spherical_arm,
    task_poses,
    task_solutions,
    tool_down,
    ur_arm,
)
from horilab_design.values import load_arm, load_measured


def test_task_poses_point_the_tool_down() -> None:
    poses = list(task_poses(0.30, steps=3))
    assert len(poses) == 3 * 2 * len(TASK_YAWS_DEG)
    for T in poses:
        np.testing.assert_allclose(T[:3, 2], [0.0, 0.0, -1.0], atol=1e-12)


def test_both_wrist_templates_reach_every_task_pose_and_keep_one_wrist_side() -> None:
    for robot in (spherical_arm(), ur_arm()):
        solutions = task_solutions(robot, point_masses(robot, MASS_CASES[0], 0.05), 0.30)
        assert all(q is not None for q in solutions)
        assert all(q[4] > 0 for q in solutions if q is not None)


def test_skeleton_targets_follow_the_layout() -> None:
    # 0014: 집는 곳 1자리(박스 1층) + 팔레트 4자리 × 4층, 자리마다 박스 윗면과 그 위 3 cm
    poses = skeleton.targets()
    assert len(poses) == (1 + 4 * 4) * 2
    radii = [radius for radius, _, _ in poses]
    heights = [height for _, height, _ in poses]
    assert 0.179 < min(radii) and max(radii) < 0.241
    assert min(heights) == pytest.approx(0.04) and max(heights) == pytest.approx(0.19)


def test_far_target_is_not_reached_by_flipping_the_wrist() -> None:
    robot = ur_arm()
    ik = AnalyticIK(robot)
    T = tool_down(0.40, 0.16)
    assert len(ik.solve_in_limits(T)) > 0  # 손목을 뒤집은 해로는 닿는다
    assert pick_solution(robot, ik, T, point_masses(robot, MASS_CASES[0], 0.05)) is None


def test_packaging_layout_stacks_boxes_without_collisions() -> None:
    # 지금 손목 오프셋(63 · 63 · 50 mm)에서 평행 그리퍼로 박스를 쌓는 자세가 부딪히지 않는다
    tool = packaging.TOOLS["gripper"]
    robot, prims, _ = packaging.build_arm(packaging.Layout(), tool)
    scene = packaging.build_scene(robot, prims)
    poses = packaging.stacking_poses(tool)
    result = packaging.check_set(scene, AnalyticIK(robot), poses, "박스 쌓기", tool.symmetric)
    assert result.reached == result.total == len(poses)
    assert result.clear == result.total, result.failures


def test_sts_shape_follows_the_measured_body() -> None:
    # 실측(data/measured.toml): 혼 윗면 ~ 뒷면, 먼 쪽 두께. 혼은 앉는 자리 바닥과 옆에 겹치지 않는다
    body = load_measured().servo_body
    for model in ("STS3215", "STS3250"):
        s = packaging.shapes()[model]
        mm = packaging.MM
        assert (s.horn_out[1] / mm - packaging.STS_BACK) == pytest.approx(
            body[model].horn_to_back_mm
        )
        assert (s.far_half[1] - s.far_half[0]) / mm == pytest.approx(body[model].far_thickness_mm)
        recess = [z for z in s.zones if z[0] >= packaging.STS_RECESS_X * mm - 1e-9]
        floor = max(z[3] for z in recess if z[4] < 0 < z[5])
        assert s.horn_out[0] > floor
        walls = [z for z in s.zones if z[4] > 0]
        assert walls and all(z[4] >= s.horn_r for z in walls)


def test_packaging_uses_the_weighed_servo_and_horn_mass() -> None:
    # 0006의 "질량": 저울로 잰 값이 있으면 명목값과 어림 대신 그 값을 쓴다
    measured = load_measured()
    for model, grams in measured.servo_horn_mass_g.items():
        assert packaging.shapes()[model].horn_mass == pytest.approx(grams * 1e-3)
    _, prims, mods = packaging.build_arm(packaging.Layout(), packaging.TOOLS["gripper"])
    for m in mods:
        body = sum(p.mass for p in prims if p.name.startswith(f"J{m.joint}/servo"))
        grams = measured.servo_mass_g.get(m.shape.model)
        if grams is not None:
            assert body == pytest.approx(grams * 1e-3), m.joint


def test_rect_overlaps_uses_separating_axes() -> None:
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    assert packaging.rect_overlaps(square, (0.5, 2.0), (0.5, 2.0))
    assert not packaging.rect_overlaps(square, (1.5, 2.0), (0.0, 1.0))
    diamond = np.array([[1.0, 0.0], [2.0, 1.0], [1.0, 2.0], [0.0, 1.0]])
    assert not packaging.rect_overlaps(diamond, (1.8, 2.5), (1.8, 2.5))  # 모서리끼리 비켜 감
    assert packaging.rect_overlaps(diamond, (1.4, 2.5), (1.4, 2.5))


def test_packaging_layout_matches_the_value_file() -> None:
    # 배치 검토의 기본 골격은 params/arm.toml(0017)과 같아야 한다
    arm = load_arm().arm
    layout = packaging.Layout()
    assert layout.shoulder * 1000 == pytest.approx(arm.shoulder_height_mm)
    assert layout.upper * 1000 == pytest.approx(arm.upper_arm_mm)
    assert layout.fore * 1000 == pytest.approx(arm.forearm_mm)
    assert layout.d45 * 1000 == pytest.approx(arm.j4_to_j5_mm)
    assert layout.d56 * 1000 == pytest.approx(arm.j5_to_j6_mm)
    assert layout.d6f * 1000 == pytest.approx(arm.j6_to_flange_mm)


def test_skeleton_reaches_the_corners_of_the_main_work_zone() -> None:
    # 0017: 흡착과 평행 그리퍼가 주 작업 영역의 네 귀퉁이에 도구를 아래로 향해 닿고, 버팀 토크가 목표 안이다
    s = reach.scenes(packaging.Layout())
    for r, z in itertools.product((0.16, 0.30), reach.MAIN_Z):
        cell = reach.cell(s, r, z)
        assert cell.down and cell.grip, (r, z)
        assert max(cell.torque) <= RATED_MARGIN, (r, z)


def test_tool_down_line_across_the_work_zone_is_continuous() -> None:
    # 0017: 칸 사이도 이어진다. 영역을 가로지르는 직선에서 막힌 걸음이 없고 관절이 튀지 않는다
    s = reach.scenes(packaging.Layout())
    tool = packaging.TOOLS["suction"]
    radii = np.linspace(reach.MAIN_R[0], reach.MAIN_R[1], 15)
    t = reach.track(s, "suction", [reach.down_pose(float(r), 0.02, tool) for r in radii])
    assert t.blocked == 0
    assert np.degrees(t.step) < 10


def test_each_servo_port_stays_under_the_connector_rating() -> None:
    # 0016: 서보 줄을 두 단자로 나누면 단자마다 버팀 전류가 커넥터 정격 2 A(26 AWG) 안이다
    worst, at_sum, _ = reach.holding_currents(packaging.Layout())
    assert worst[0] == pytest.approx(0.0, abs=1e-6)  # J1은 중력을 버티지 않는다
    for joints in reach.PORTS:
        assert at_sum[list(joints)].sum() < 2.0, joints


def test_desk_share_counts_the_part_of_the_work_zone_on_the_desk() -> None:
    assert reach.desk_share(0.60, 0.36, 0.06) == pytest.approx(1.0)
    assert 0.9 < reach.desk_share(0.55, 0.34, 0.06) < 1.0


def test_symmetric_tool_flip_stays_inside_the_j6_range() -> None:
    # 대칭 도구는 J6를 180° 돌린 해도 쓰지만, 그 각도 J6 범위 안이어야 한다
    tool = packaging.TOOLS["gripper"]
    robot = packaging.arm(packaging.Layout())
    ik = AnalyticIK(robot)
    T = packaging.tool_pose((0.22, 0.0, 0.06), 0.0, 0.0, 0.0, tool.length)
    assert packaging.solve(ik, T)
    j6 = robot.joints[5]
    assert all(j6.lower <= q[5] <= j6.upper for q in packaging.solve(ik, T, symmetric=True))


def test_packaging_reports_designed_gaps_but_not_as_collisions() -> None:
    tool = packaging.TOOLS["gripper"]
    robot, prims, _ = packaging.build_arm(packaging.Layout(), tool)
    scene = packaging.build_scene(robot, prims)
    q = packaging.solve(
        AnalyticIK(robot), packaging.tool_pose((0.2, 0.0, 0.1), 0.0, 0.0, 0.0, tool.length)
    )[0]
    found = packaging.hits(scene, q)
    assert not packaging.collisions(found)
    assert any(h.designed for h in found)  # 서보와 그 혼·브래킷은 일부러 가깝다


def test_placed_stacking_poses_keep_the_skeleton_targets() -> None:
    # 기둥 검토는 박스 쌓기 자세를 실제 방위에 둔다. 반지름과 높이는 골격 검토의 목표와 같아야 한다
    # (TCP는 그리퍼가 박스 윗면 구멍 안을 잡는 깊이만큼 아래).
    tool = packaging.TOOLS["gripper"]
    placed = cameras.placed_stacking_poses(tool)
    expected = skeleton.targets()
    assert len(placed) == len(expected)
    for pose, (radius, height, _) in zip(placed, expected, strict=True):
        tcp = pose.T[:3, 3] + pose.T[:3, 2] * tool.length
        assert np.hypot(tcp[0], tcp[1]) == pytest.approx(radius)
        assert tcp[2] == pytest.approx(height - tool.box_depth)


def test_overhead_post_camera_sees_the_work_spots_and_stays_out_of_the_way() -> None:
    cam = cameras.RECOMMENDED
    R, eye = cam.frame(), np.array(cam.eye)
    for pts in cameras.work_targets().values():
        assert cameras.in_view(R, eye, pts).all()
    fixed = cameras.post_prims(cam)
    tool = packaging.TOOLS["gripper"]
    robot, prims, _ = cameras.arm_shapes(packaging.Layout(), tool)
    scene = packaging.build_scene(robot, prims, fixed)
    poses = cameras.placed_stacking_poses(tool) + cameras.transit_poses(tool)
    result = packaging.check_set(scene, AnalyticIK(robot), poses, tool.name, tool.symmetric)
    assert result.clear == result.total, result.failures
    robot, prims, _ = cameras.arm_shapes(packaging.Layout(), packaging.TOOLS["gripper"])
    assert cameras.upright_hits(packaging.build_scene(robot, prims, fixed)) == 0


def test_wrist_camera_aim_puts_the_lens_on_the_view_point() -> None:
    layout = packaging.Layout()
    robot, _, cam = cameras.arm_shapes(layout, packaging.TOOLS["suction"])
    board = np.array([*cameras.CAL_BOARD, 0.0])
    eye = board + np.array([0.0, 0.0, 0.25])
    seeds = [np.radians([0.0, 20.0, 80.0, 165.0, 90.0, 10.0])]
    solutions = cameras.aim(robot, cam, eye, board, seeds)
    assert solutions
    lens, axis = cameras.camera_pose(robot, cam, solutions[0])
    np.testing.assert_allclose(lens, eye, atol=1e-4)
    assert axis @ np.array([0.0, 0.0, -1.0]) > np.cos(np.radians(0.2))


def test_rings_become_box_segments_mapped_to_one_shape() -> None:
    # MuJoCo에는 속 빈 원기둥이 없어서 고리는 상자 조각으로 나눈다. 부딪힘 보고는 고리 하나로 나와야 한다.
    robot, prims, _, _ = covers.build_ur_arm(packaging.Layout(), packaging.TOOLS["suction"])
    scene = packaging.build_scene(robot, prims)
    rings = [p for p in prims if p.shape == "ring"]
    assert rings
    for ring in rings:
        pieces = [g for g, p in scene.geom_prim.items() if p is ring]
        assert len(pieces) == packaging.RING_SEGMENTS


def test_ur_covers_fit_whole_at_the_chosen_wrist_offsets() -> None:
    # 손목 오프셋 63 mm(0019)면 뚜껑을 줄이지 않고 평행 그리퍼로 박스를 쌓는 자세가 부딪히지 않는다
    layout = packaging.Layout()
    tool = packaging.TOOLS["gripper"]
    robot, prims, mods, _ = covers.build_ur_arm(layout, tool)
    for drum in covers.drums(layout, mods, covers.Look()).values():
        assert drum.cap_out_r == pytest.approx(drum.r_out)
        assert drum.cap_back_r == pytest.approx(drum.r_out)
    scene = packaging.build_scene(robot, prims)
    poses = packaging.stacking_poses(tool)
    result = packaging.check_set(scene, AnalyticIK(robot), poses, "박스 쌓기", tool.symmetric)
    assert result.clear == result.total, result.failures


def test_groove_stiffness_loss_matches_thin_ring_estimate() -> None:
    # 얇은 고리: 띠 홈이 줄이는 I는 (r⁴ 차이)/4 × ∫cos²θ dθ 이고, 전체 I는 π/4 (r_out⁴ - r_in⁴)이다
    r_out, wall, depth, height = 0.020, 0.0026, 0.0005, 0.008
    a = height / 2 / r_out
    r_in = r_out - wall
    shell = (r_out**4 - (r_out - depth) ** 4) / 4
    whole = math.pi / 4 * (r_out**4 - r_in**4)
    side = shell * (a - math.sin(2 * a) / 2) / whole
    top = shell * (a + math.sin(2 * a) / 2) / whole
    loss = covers.groove_stiffness_loss
    assert loss(r_out, wall, depth, height, math.pi / 2) == pytest.approx(side, rel=0.02)
    assert loss(r_out, wall, depth, height, 0.0) == pytest.approx(top, rel=0.02)


def test_frame_deflection_matches_the_cantilever_formula() -> None:
    # 수평인 보 끝에 무게 P: TCP의 높이 변화 = -P L² (X - L/3) / (2 E I), X는 보 시작점에서 TCP까지 보 방향 거리
    robot = packaging.arm(packaging.Layout())
    q = np.radians([0.0, 90.0, 0.0, 0.0, 0.0, 0.0])
    z0, z1, r, t = 0.05, 0.15, 0.02, 0.0026
    beam = covers.tube_beam("관", 2, z0, z1, r, t)
    mass = PointMass("무게", 2, (0.0, 0.0, z1), 1.0)
    moved, _ = covers.tcp_deflection(robot, q, [beam], [mass], np.zeros(3))
    frames = frame_poses(robot, q)
    start = frames[2][:3, :3] @ np.array([0.0, 0.0, z0]) + frames[2][:3, 3]
    reach = frames[-1][0, 3] - start[0]
    length, ei = z1 - z0, beam.ei1
    expected = -9.81 * length**2 * (reach - length / 3) / (2 * ei)
    assert moved[2] == pytest.approx(expected, rel=1e-3)


def test_outward_wrist_keeps_j5_negative() -> None:
    robot = ur_arm(wrist=-1)
    masses = point_masses(robot, MASS_CASES[0], 0.05)
    solutions = task_solutions(robot, masses, 0.40, wrist=-1, r_min=0.16)
    assert all(q is not None for q in solutions)
    assert all(q[4] < 0 and q[2] > 0 for q in solutions if q is not None)


def test_ur3e_model_follows_the_public_model() -> None:
    # 공개 모형(URDF)의 관절 각 0 = 이 모델의 (180, 90, 0, 90, 0, 0)°, 축 방향은 모두 같다.
    # 아무 관절 각에서도 링크 자리가 공개 모형과 같고, 관절 각 0의 플랜지는 DH 값 그대로다.
    offset, signs = ur3e.correspondence()
    np.testing.assert_allclose(np.degrees(offset), [180, 90, 0, 90, 0, 0], atol=1e-9)
    assert signs == (1, 1, 1, 1, 1, 1)
    robot = ur3e.ur_robot()
    o0 = link_poses(robot, np.array(offset))
    u0 = ur3e.urdf_link_poses(np.zeros(6))
    rng = np.random.default_rng(3)
    for _ in range(20):
        qu = rng.uniform(-math.pi, math.pi, 6)
        ours, urs = link_poses(robot, ur3e.to_ours(qu)), ur3e.urdf_link_poses(qu)
        for i in range(6):
            np.testing.assert_allclose(ours[i] @ np.linalg.inv(o0[i]) @ u0[i], urs[i], atol=1e-12)
    flange = flange_pose(robot, np.array(offset))
    np.testing.assert_allclose(flange[:3, 3], [-0.45675, -0.22315, 0.0665], atol=1e-12)
    assert AnalyticIK(robot).family  # EAIK가 UR형으로 푼다


def test_ur3e_elbow_folds_until_the_wrist_meets_the_upper_arm() -> None:
    # 공개 형상으로는 팔을 세운 자세에서 팔꿈치가 약 155°까지 접히고, 손목 1이 위팔에 닿는다.
    # 나머지 관절은 막힘 없이 돈다. 줄인 모델도 형상이 같은 비율로 줄어서 같다.
    for s in (1.0, 0.8):
        scene = ur3e.ur_scene(s)
        upright = ur3e.to_ours([0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0])
        assert not packaging.collisions(packaging.hits(scene, upright))
        for j, (_lo, hi, why_lo, why_hi) in enumerate(packaging.joint_ranges(scene, upright)):
            if j == 2:
                assert 150 < np.degrees(hi - upright[j]) < 160
                assert why_hi == "UR3e 위팔 ↔ UR3e 손목 1"
            else:
                assert (why_lo, why_hi) == ("막힘 없음", "막힘 없음")


def test_ur3e_tool_is_its_own_body_and_meets_the_desk() -> None:
    # 도구는 손목 3에 붙은 따로 된 몸체라 손목 3하고만 부딪힘을 빼고, 책상과는 부딪힘을 본다.
    tool = packaging.TOOLS["gripper"]
    scene = ur3e.ur_scene(ur_compare.scale(), tool)
    names = [scene.model.body(i).name for i in range(scene.model.nbody)]
    assert "tool" in names
    arm = ur_compare.Arm(scene.robot, AnalyticIK(scene.robot), scene)
    low = ur_compare.flange_target(0.24, 0.0, 0.0, 0, 0.0, tool.length)  # 손가락 끝이 책상 아래
    high = ur_compare.flange_target(0.24, 0.10, 0.0, 0, 0.0, tool.length)
    assert ur_compare.ur_outcome(arm, high)[0] == "됨"
    outcome, pair = ur_compare.ur_outcome(arm, low)
    assert outcome == "부딪힘" and "책상" in pair


def test_flange_target_tilts_the_tool_toward_the_direction() -> None:
    # 기울기 0°는 아래, 180°는 위, 90°는 방향 쪽 수평이다. 방향 0은 바깥(TCP 자리의 방위)이다.
    length = 0.1
    down = ur_compare.flange_target(0.2, 0.1, 0.0, 0, 0.0, length)
    up = ur_compare.flange_target(0.2, 0.1, math.pi, 0, 0.0, length)
    out = ur_compare.flange_target(0.2, 0.1, math.pi / 2, 0, 0.0, length)
    left = ur_compare.flange_target(0.2, 0.1, math.pi / 2, 2, 0.0, length)
    az = ur_compare.AZIMUTH
    radial = np.array([math.cos(az), math.sin(az), 0.0])
    np.testing.assert_allclose(down[:3, 2], [0.0, 0.0, -1.0], atol=1e-12)
    np.testing.assert_allclose(up[:3, 2], [0.0, 0.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(out[:3, 2], radial, atol=1e-12)
    np.testing.assert_allclose(left[:3, 2], np.cross([0.0, 0.0, 1.0], radial), atol=1e-12)
    for T in (down, up, out, left):
        np.testing.assert_allclose(
            T[:3, 3] + length * T[:3, 2], 0.2 * radial + [0, 0, 0.1], atol=1e-12
        )


def test_ur_joint_concept_stacks_and_turns() -> None:
    # UR식 관절: 베어링은 서보 앞면에서 턱만큼 띄우고, 뚜껑은 베어링과 출력 혼 바깥에 둔다.
    # 팔을 세운 자세에서 부딪히지 않고, 손목 1과 손목 3(J4, J6)은 막힘 없이 한 바퀴 돈다.
    shape = packaging.shapes()["STS3215"]
    s = ur_joints.stack(shape)
    mm = packaging.MM
    assert s.bearing[0] == pytest.approx(17.5 * mm + 2.0 * mm)
    assert s.cap[0] >= max(s.bearing[1], shape.horn_out[1])
    assert s.back < packaging.STS_BACK * mm - packaging.CONNECTOR_DEPTH * mm
    c = ur_joints.Concept()
    robot, prims = ur_joints.build(c, packaging.TOOLS["gripper"])
    np.testing.assert_allclose(
        [robot.joints[4].origin[1], robot.joints[5].origin[2], robot.T_last_flange[1, 3]],
        [c.d4, c.d5, c.d6],
    )
    scene = packaging.build_scene(robot, prims)
    q0 = np.radians([0.0, 0.0, 0.0, 180.0, 0.0, 0.0])
    assert not packaging.collisions(packaging.hits(scene, q0))
    ranges = packaging.joint_ranges(scene, q0)
    for j in (0, 3, 5):
        assert ranges[j][2:] == ("막힘 없음", "막힘 없음")
    assert np.degrees(ranges[2][1]) > 150  # 팔꿈치가 UR3e(약 155°)만큼 접힌다
    # 선 칸 예산은 원통 반지름과, 베어링과 뚜껑 사이를 그만큼 키운다 (베어링 자리는 그대로)
    wide = ur_joints.stack(shape, 6 * mm, 6 * mm)
    assert wide.radius == pytest.approx(s.radius + 6 * mm)
    assert wide.bearing == s.bearing
    assert wide.front == pytest.approx(s.bearing[1] + 6 * mm)
    assert wide.cap[0] == pytest.approx(s.cap[0] + 6 * mm)


def test_range_fit_counts_full_turns_and_the_gripper_flip() -> None:
    lower, upper = np.radians([-135.0] * 6), np.radians([135.0] * 6)
    q = np.radians([170.0, -200.0, 10.0, 0.0, 0.0, 170.0])
    ok = ur_compare.fits(q, lower, upper)
    # J1 170°는 밖, J2 -200° = 160°도 밖, J6 170°는 평행 그리퍼를 180° 돌린 -10°로 안
    assert ok.tolist() == [False, False, True, True, True, True]
