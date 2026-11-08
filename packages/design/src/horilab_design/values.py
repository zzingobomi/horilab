"""값 파일(TOML)을 읽어 검사하고, 로봇 기술(SI)로 바꾼다 (docs/decisions/0006).

- params/arm.toml: 팔의 설계 값 (명목 치수, 관절 범위, 관절마다 서보)
- params/servos.toml: 서보 모델의 명목 사양
- data/measured.toml: 실측 값. 설계 값을 덮어쓰지 않고 따로 둔다

파일의 단위는 키 이름에 붙은 mm, deg, g, kgcm이고, 로봇 기술로 바꿀 때 SI로 바꾼다.
키 이름이 틀리면(단위를 빠뜨리는 등) 읽을 때 바로 실패한다.
"""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator

from horilab_design.robot import Joint, Robot, Vec3, rotation

REPO_ROOT = Path(__file__).resolve().parents[4]
ARM_FILE = REPO_ROOT / "params" / "arm.toml"
SERVOS_FILE = REPO_ROOT / "params" / "servos.toml"
MEASURED_FILE = REPO_ROOT / "data" / "measured.toml"

KGCM = 0.0980665  # 1 kg·cm [N·m]
JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
X: Vec3 = (1.0, 0.0, 0.0)
Y: Vec3 = (0.0, 1.0, 0.0)
Z: Vec3 = (0.0, 0.0, 1.0)

ServoModel = Literal["STS3215", "STS3250", "ST-3120"]


class _Values(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ServoSpec(_Values):
    rated_torque_kgcm: float = Field(gt=0)
    stall_torque_kgcm: float = Field(gt=0)
    no_load_speed_s_per_60deg: float = Field(gt=0)
    stall_current_a: float = Field(gt=0)
    mass_g: float = Field(gt=0)
    body_length_mm: float = Field(gt=0)  # 출력축에 수직인 단면의 긴 변
    body_width_mm: float = Field(gt=0)
    shaft_to_end_mm: float = Field(gt=0)  # 출력축에서 길이 방향으로 가까운 끝까지
    horn_diameter_mm: float = Field(gt=0)
    horn_bolt_circle_mm: float = Field(gt=0)
    horn_bolts: int = Field(gt=0)  # 혼의 M3 구멍 수

    @property
    def rated_torque(self) -> float:
        """정격 토크 [N·m]. 토크 여유는 최대 토크가 아니라 이 값으로 본다 (docs/hardware.md)."""
        return self.rated_torque_kgcm * KGCM

    @property
    def max_speed(self) -> float:
        """무부하 속도 [rad/s]."""
        return float(np.pi / 3 / self.no_load_speed_s_per_60deg)


class ArmGeometry(_Values):
    shoulder_height_mm: float = Field(gt=0)
    upper_arm_mm: float = Field(gt=0)
    forearm_mm: float = Field(gt=0)
    j4_to_j5_mm: float = Field(ge=0)
    j5_to_j6_mm: float = Field(ge=0)
    j6_to_flange_mm: float = Field(ge=0)


class JointParams(_Values):
    servo: ServoModel
    range_deg: tuple[float, float]

    @model_validator(mode="after")
    def _one_turn(self) -> JointParams:
        lower, upper = self.range_deg
        if not lower < upper:
            raise ValueError(f"관절 범위는 아래 < 위여야 한다: {self.range_deg}")
        if upper - lower > 360:
            raise ValueError(f"서보 위치 모드는 한 바퀴(360°) 안이다: {self.range_deg}")
        return self


class ArmParams(_Values):
    arm: ArmGeometry
    joints: dict[str, JointParams]

    @model_validator(mode="after")
    def _all_joints(self) -> ArmParams:
        if sorted(self.joints) != list(JOINT_NAMES):
            raise ValueError(f"관절은 {JOINT_NAMES}여야 한다: {sorted(self.joints)}")
        return self


Filament = Literal["pla_basic_white", "pla_basic_black"]


class PrintFit(_Values):
    """공차 시험편으로 고른 설계 치수 [mm]. 필라멘트마다 하나다 (docs/manufacturing.md의 "공차")."""

    m3_hole_mm: float | None = None  # 세로 구멍: M3 볼트가 걸리지 않고 지나가는 가장 작은 지름
    m3_hole_horizontal_mm: float | None = None  # 가로 구멍(물방울 모양)의 원 지름
    m3_tap_hole_mm: float | None = None  # M3 볼트를 플라스틱에 바로 조여 넣는 구멍
    m3_nut_pocket_mm: float | None = None  # 위에서 눌러 끼우는 육각 홈의 맞변 거리
    m3_nut_slot_width_mm: float | None = None  # 옆에서 밀어 넣는 홈의 폭
    m3_nut_slot_height_mm: float | None = None  # 옆에서 밀어 넣는 홈의 높이
    # 서보 몸체 자리와 혼 자리: 도면 치수(params/servos.toml)에 더하는 여유. 길이·폭·지름에 그대로 더한다
    seat_clearance_mm: dict[ServoModel, float] = {}
    horn_clearance_mm: dict[ServoModel, float] = {}


class ServoBody(_Values):
    """캘리퍼스로 잰 서보 몸통 치수 [mm] (docs/hardware.md의 "기구 치수")."""

    far_thickness_mm: float = Field(gt=0)  # 라벨 쪽 끝 두께: 앞면 ~ 뒷면의 볼록한 판
    horn_to_back_mm: float = Field(gt=0)  # 출력 혼 윗면 ~ 뒷면 (축 가까운 쪽)


class Measured(_Values):
    servo_mass_g: dict[ServoModel, float] = {}
    servo_body: dict[ServoModel, ServoBody] = {}
    fit: dict[Filament, PrintFit] = {}


def read_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as file:
        return tomllib.load(file)


def load_arm(path: Path = ARM_FILE) -> ArmParams:
    return ArmParams.model_validate(read_toml(path))


def load_servos(path: Path = SERVOS_FILE) -> dict[ServoModel, ServoSpec]:
    return TypeAdapter(dict[ServoModel, ServoSpec]).validate_python(read_toml(path))


def load_measured(path: Path = MEASURED_FILE) -> Measured:
    return Measured.model_validate(read_toml(path))


def servo_mass(model: ServoModel, servos: dict[ServoModel, ServoSpec], measured: Measured) -> float:
    """서보 무게 [kg]. 저울로 잰 값이 있으면 그 값을, 없으면 명목값을 쓴다 (0006의 "질량")."""
    grams = measured.servo_mass_g.get(model, servos[model].mass_g)
    return grams * 1e-3


def arm_robot(params: ArmParams, servos: dict[ServoModel, ServoSpec]) -> Robot:
    """UR형 팔의 로봇 기술 (SI). 관절의 낼 수 있는 토크와 속도는 서보의 정격 토크와 무부하 속도다.

    플랜지 z(도구 방향)는 J6 축(y)을 본다. 도구 축 둘레로는 -90° 더 돌려서, 기본 작업 자세
    (도구가 아래, 박스 돌림 0°)에서 J6가 범위 가운데에 오게 한다 (0013).
    """
    g = params.arm

    def joint(name: str, origin_mm: Vec3, axis: Vec3) -> Joint:
        values = params.joints[name]
        spec = servos[values.servo]
        lower, upper = np.radians(values.range_deg)
        origin = (origin_mm[0] * 1e-3, origin_mm[1] * 1e-3, origin_mm[2] * 1e-3)
        return Joint(
            name, origin, axis, float(lower), float(upper), spec.rated_torque, spec.max_speed
        )

    joints = (
        joint("j1", (0.0, 0.0, 0.0), Z),
        joint("j2", (0.0, 0.0, g.shoulder_height_mm), Y),
        joint("j3", (0.0, 0.0, g.upper_arm_mm), Y),
        joint("j4", (0.0, 0.0, g.forearm_mm), Y),
        joint("j5", (0.0, g.j4_to_j5_mm, 0.0), Z),
        joint("j6", (0.0, 0.0, g.j5_to_j6_mm), Y),
    )
    flange = np.eye(4)
    flange[:3, :3] = rotation(X, -np.pi / 2) @ rotation(Z, -np.pi / 2)
    flange[:3, 3] = (0.0, g.j6_to_flange_mm * 1e-3, 0.0)
    return Robot("horilab", joints, flange)


def load_robot() -> Robot:
    """레포의 값 파일로 만든 지금 설계의 로봇 기술."""
    return arm_robot(load_arm(), load_servos())
