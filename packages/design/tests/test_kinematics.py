"""로봇 기술에서 나온 FK, MJCF, URDF, 해석적 IK가 서로 맞는지 확인한다.

docs/decisions/0006-data-and-pipeline.md의 FK 대조 테스트다. 축 방향이 뒤집혀도 영점에서는
같아 보이므로 무작위 자세에서도 비교한다. 테스트용 로봇은 conftest.py에 있다.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yourdfpy

from horilab_design.ik import AnalyticIK, wrap_angle
from horilab_design.mjcf import build_spec
from horilab_design.robot import Joint, Mat, Robot, Vec3, flange_pose
from horilab_design.urdf import build_urdf, matrix_from_rpy, rpy_from_matrix


def _mujoco_flange_pose(model: mujoco.MjModel, robot: Robot, q: Mat) -> Mat:
    data = mujoco.MjData(model)
    for joint, angle in zip(robot.joints, q, strict=True):
        data.qpos[model.joint(joint.name).qposadr[0]] = angle
    mujoco.mj_kinematics(model, data)
    site = data.site("flange")
    T = np.eye(4)
    T[:3, :3] = site.xmat.reshape(3, 3)
    T[:3, 3] = site.xpos
    return T


def test_fk_matches_mujoco(robot: Robot, configs: list[Mat]) -> None:
    model = build_spec(robot).compile()
    for q in configs:
        np.testing.assert_allclose(
            _mujoco_flange_pose(model, robot, q), flange_pose(robot, q), atol=1e-9
        )


def test_mjcf_xml_keeps_kinematics(robot: Robot, configs: list[Mat]) -> None:
    # XML로 저장했다가 다시 읽어도 같아야 한다. 각도 단위(도, 라디안) 실수를 잡는다.
    # MuJoCo는 XML에 유효숫자 6자리만 쓰므로 1 µm 수준의 차이는 허용한다.
    model = mujoco.MjModel.from_xml_string(build_spec(robot).to_xml())
    for q in configs:
        np.testing.assert_allclose(
            _mujoco_flange_pose(model, robot, q), flange_pose(robot, q), atol=1e-5
        )


def test_fk_matches_urdf(robot: Robot, configs: list[Mat], tmp_path: Path) -> None:
    path = tmp_path / "robot.urdf"
    path.write_text(build_urdf(robot), encoding="utf-8")
    urdf = yourdfpy.URDF.load(str(path), load_meshes=False)
    for q in configs:
        urdf.update_cfg({joint.name: float(a) for joint, a in zip(robot.joints, q, strict=True)})
        np.testing.assert_allclose(
            urdf.get_transform("flange", "base"), flange_pose(robot, q), atol=1e-9
        )


def test_urdf_limits_are_plain_numbers_for_numpy_values() -> None:
    # 값 파일의 도(°)를 np.radians로 바꾸면 numpy 수가 된다. URDF에 그대로 적혀도 숫자여야 한다.
    lower, upper = np.radians(-170.0), np.radians(170.0)
    joint = Joint("j1", (0.0, 0.0, 0.1), (0.0, 0.0, 1.0), lower, upper, effort=1.0, velocity=4.0)
    flange = np.eye(4)
    root = ET.fromstring(build_urdf(Robot("numpy_limits", (joint,), flange)))
    limit = root.find("joint/limit")
    assert limit is not None
    assert float(limit.attrib["lower"]) == pytest.approx(np.radians(-170.0))


def test_analytic_ik_finds_the_original_pose(robot: Robot, configs: list[Mat]) -> None:
    ik = AnalyticIK(robot)
    for q in configs[1:]:  # 영점은 특이 자세라 뺀다
        target = flange_pose(robot, q)
        solutions = ik.solve(target)
        assert len(solutions) > 0
        for solution in solutions:
            np.testing.assert_allclose(flange_pose(robot, solution), target, atol=1e-8)
        assert any(np.allclose(wrap_angle(s - q), 0.0, atol=1e-6) for s in solutions)


def test_solutions_beyond_pi_are_kept_when_limits_allow(robot: Robot) -> None:
    # 관절 한계가 ±π를 넘으면, -π~π로 접힌 해와 2π 차이 나는 해도 한계 안에 있어야 한다.
    wide = Robot(
        robot.name,
        tuple(
            Joint(j.name, j.origin, j.axis, -3.5, 3.5, j.effort, j.velocity) for j in robot.joints
        ),
        robot.T_last_flange,
    )
    q = np.array([3.3, 0.4, 0.9, 0.3, 0.7, -0.2])
    solutions = AnalyticIK(wide).solve_in_limits(flange_pose(wide, q))
    assert any(np.allclose(s, q, atol=1e-6) for s in solutions)


@pytest.mark.parametrize(
    "rpy",
    [(0.1, 0.2, 0.3), (-2.0, 1.0, 3.0), (0.3, np.pi / 2, -0.4), (-0.2, -np.pi / 2, 0.5)],
)
def test_rpy_round_trip(rpy: Vec3) -> None:
    R = matrix_from_rpy(rpy)
    np.testing.assert_allclose(matrix_from_rpy(rpy_from_matrix(R)), R, atol=1e-12)


def test_joint_rejects_non_unit_axis() -> None:
    with pytest.raises(ValueError):
        Joint("j1", (0.0, 0.0, 0.1), (0.0, 0.0, 2.0), -1.0, 1.0, effort=1.0, velocity=1.0)


def test_robot_rejects_non_rotation_flange(robot: Robot) -> None:
    bad = np.eye(4)
    bad[0, 0] = 2.0
    with pytest.raises(ValueError):
        Robot("bad", robot.joints, bad)
