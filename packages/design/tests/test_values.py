"""값 파일을 읽어 검사하고 로봇 기술로 바꾸는 과정을 확인한다 (docs/decisions/0006).

레포의 값 파일로 만든 로봇은 conftest.py의 로봇 목록에도 들어가서, FK·MJCF·URDF·IK 대조
테스트를 같이 받는다.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from pydantic import ValidationError

from horilab_design.robot import flange_pose
from horilab_design.studies.common import ur_arm
from horilab_design.values import (
    ArmParams,
    Measured,
    arm_robot,
    load_arm,
    load_measured,
    load_servos,
    servo_mass,
)

# 0011, 0013 검토의 UR형 막대 모델(studies/common.py의 ur_arm)과 같은 값
STUDY_ARM: dict[str, Any] = {
    "arm": {
        "shoulder_height_mm": 120,
        "upper_arm_mm": 200,
        "forearm_mm": 180,
        "j4_to_j5_mm": 50,
        "j5_to_j6_mm": 50,
        "j6_to_flange_mm": 50,
    },
    "joints": {
        "j1": {"servo": "STS3215", "range_deg": [-175, 175]},
        "j2": {"servo": "ST-3120", "range_deg": [-120, 120]},
        "j3": {"servo": "STS3250", "range_deg": [-150, 150]},
        "j4": {"servo": "STS3215", "range_deg": [5, 355]},
        "j5": {"servo": "STS3215", "range_deg": [-175, 175]},
        "j6": {"servo": "STS3215", "range_deg": [-175, 175]},
    },
}


def _with(path: tuple[str, ...], value: Any) -> dict[str, Any]:
    """STUDY_ARM에서 한 값만 바꾼 복사본. value가 None이면 그 키를 지운다."""
    data: dict[str, Any] = {
        "arm": dict(STUDY_ARM["arm"]),
        "joints": {k: dict(v) for k, v in STUDY_ARM["joints"].items()},
    }
    target = data
    for key in path[:-1]:
        target = target[key]
    if value is None:
        del target[path[-1]]
    else:
        target[path[-1]] = value
    return data


def test_repo_value_files_load() -> None:
    params, servos, measured = load_arm(), load_servos(), load_measured()
    robot = arm_robot(params, servos)
    assert [joint.name for joint in robot.joints] == ["j1", "j2", "j3", "j4", "j5", "j6"]
    for name, joint in params.joints.items():
        assert servo_mass(joint.servo, servos, measured) > 0, name


def test_arm_robot_matches_the_study_template() -> None:
    # 값 파일에서 만든 로봇이 0011, 0013을 검토한 막대 모델과 기구학이 같아야 검토 결과가 들어맞는다.
    robot = arm_robot(ArmParams.model_validate(STUDY_ARM), load_servos())
    study = ur_arm()
    np.testing.assert_allclose(robot.lower, study.lower, atol=1e-12)
    np.testing.assert_allclose(robot.upper, study.upper, atol=1e-12)
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = robot.lower + (robot.upper - robot.lower) * rng.random(6)
        np.testing.assert_allclose(flange_pose(robot, q), flange_pose(study, q), atol=1e-12)


def test_servo_limits_come_from_the_servo_spec() -> None:
    robot = arm_robot(ArmParams.model_validate(STUDY_ARM), load_servos())
    j2 = robot.joints[1]  # ST-3120: 정격 40 kg·cm, 60°에 0.454 s
    assert j2.effort == pytest.approx(40 * 0.0980665)
    assert j2.velocity == pytest.approx(np.pi / 3 / 0.454)


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("arm", "upper_arm"), 200),  # 단위 접미사를 빠뜨린 키
        (("arm", "forearm_mm"), -180),
        (("joints", "j1", "range_deg"), [10, 5]),
        (("joints", "j1", "range_deg"), [-200, 200]),  # 서보 한 바퀴를 넘음
        (("joints", "j2", "servo"), "STS3032"),  # 모르는 서보
        (("joints", "j6"), None),  # 관절이 빠짐
    ],
)
def test_bad_values_are_rejected(path: tuple[str, ...], value: Any) -> None:
    with pytest.raises(ValidationError):
        ArmParams.model_validate(_with(path, value))


def test_measured_servo_mass_wins_over_the_nominal_value() -> None:
    servos = load_servos()
    assert servo_mass("STS3215", servos, Measured()) == pytest.approx(0.055)
    measured = Measured.model_validate({"servo_mass_g": {"STS3215": 54.2}})
    assert servo_mass("STS3215", servos, measured) == pytest.approx(0.0542)
