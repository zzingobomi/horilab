"""로봇 설정(data/robot.toml)이 설계(params/arm.toml)와 맞는지 본다 (docs/decisions/0009, 0024).

컨트롤러는 설계 파일을 읽지 않아서, 둘이 맞는지는 PC의 이 테스트가 본다. 파일의 형식과 값 검사는
컨트롤러가 읽을 때 한다 (crates/controller/src/settings.rs, cargo test가 이 파일을 읽어 본다).
"""

import tomllib
from pathlib import Path
from typing import Any

import pytest

from horilab_design.values import load_arm

ROOT = Path(__file__).resolve().parents[1]
JOINTS: dict[str, Any] = tomllib.loads((ROOT / "data" / "robot.toml").read_text(encoding="utf-8"))[
    "joints"
]
ARM = load_arm()


@pytest.mark.parametrize("name", sorted(JOINTS))
def test_joint_matches_design(name: str) -> None:
    assert name in ARM.joints, f"{name}: 설계(params/arm.toml)에 없는 관절이다"
    design = ARM.joints[name]
    joint = JOINTS[name]
    assert joint["model"] == design.servo, f"{name}: 서보 모델이 설계와 다르다"
    lower, upper = joint["soft_limits_deg"]
    design_lower, design_upper = design.range_deg
    assert design_lower <= lower < upper <= design_upper, (
        f"{name}: 소프트 한계 {lower}~{upper}°가 설계 범위 {design_lower}~{design_upper}° 밖이다"
    )
