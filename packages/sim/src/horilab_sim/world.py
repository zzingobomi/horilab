"""시뮬레이션 세계: 설계의 로봇 모델에 위치 서보를 달고, 서보 레지스터를 실물과 같은 단위로 흉내 낸다 (0033).

- 서보는 관절 각 0이 서보 값 2048로 읽히게 달려 있다고 본다 (방향 +1). 읽는 위치 = 엔코더 값 - 위치 오프셋.
- 토크가 켜진 서보는 목표(각도 한계 안으로 자른 값)로 가는 위치 액추에이터이고, 힘은 토크 한도만큼이다.
  토크가 꺼지면 힘이 없고 기어의 감쇠와 마찰만 남는다.
- 강성, 감쇠, 기어 마찰은 어림이다. 강성은 STS3250이 20 kg·cm에 1.23° 처진다는 다른 사람의 값(0012)에서 잡아
  멈춤 토크에 비례시킨다. 다 만든 팔에서 잰 처짐으로 맞춘다.
- 로봇 모델은 설계가 쓴 MJCF 파일이다 (`uv run python -m horilab_design.mjcf`, 막대 모델). 시뮬레이션은 설계를
  불러오지 않고 그 파일과 서보 사양(params/servos.toml)만 읽는다 (0007). 무게는 막대의 부피에서 나온 값이라
  실물보다 가볍다.
"""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

import mujoco

STEPS_PER_TURN = 4096
RAD_PER_STEP = 2 * math.pi / STEPS_PER_TURN
CENTER = 2048  # 관절 각 0의 엔코더 값
KGCM = 0.0980665  # [N·m / (kg·cm)]

# 모델 번호 → 서보 모델 (horilab_servo의 MODELS와 같다. ST-3120은 오면 더한다)
MODEL_NAMES = {777: "STS3215", 2825: "STS3250"}

# 위치 서보의 어림
STIFFNESS_STS3250 = 20 * KGCM / math.radians(1.23)  # [N·m/rad] 20 kg·cm에 1.23° 처짐
DAMPING_TIME = 0.02  # [s] 서보의 감쇠 = 강성 × 이 시간
JOINT_DAMPING = 0.1  # [N·m·s/rad] 기어의 감쇠
JOINT_FRICTION = 0.05  # [N·m] 기어의 마찰
MOVING_SPEED = 0.02  # [rad/s] 이보다 빠르면 움직이는 중으로 낸다
UNDRIVEN_DAMPING = 50.0  # [N·m·s/rad] 서보를 달지 않은 관절 (거의 잠근다)
UNDRIVEN_FRICTION = 5.0  # [N·m]

# 전원을 넣은 직후의 RAM (맨 서보 확인: 목표 0, 속도 0, 가속 0, 토크 한도 최대, 토크 꺼짐)
POWER_ON = {
    "torque_enable": 0,
    "acceleration": 0,
    "goal_position": 0,
    "goal_speed": 0,
    "torque_limit": 1000,
    "lock": 1,
    "present_voltage": 120,
    "present_temperature": 25,
    "status": 0,
    "present_current": 0,
}
# 전원을 꺼도 남는 EEPROM 가운데 흉내 내는 칸
EEPROM = {"position_offset": 0, "min_angle_limit": 0, "max_angle_limit": 4095, "max_torque": 1000}


def _stall_torques(path: Path) -> dict[str, float]:
    """서보 사양 파일(params/servos.toml)의 모델마다 멈춤 토크 [N·m]."""
    table = tomllib.loads(path.read_text(encoding="utf-8"))
    return {name: float(spec["stall_torque_kgcm"]) * KGCM for name, spec in table.items()}


@dataclass
class Servo:
    """시뮬레이션 서보 하나: 레지스터와 MuJoCo 안의 자리."""

    id: int
    joint: str
    model: int
    stall: float  # 멈춤 토크 [N·m]
    stiffness: float  # [N·m/rad]
    actuator: int
    qpos: int
    dof: int
    regs: dict[str, int] = field(default_factory=dict)
    flash: dict[str, int] = field(default_factory=dict)  # 전원을 꺼도 남는 값


class World:
    """설계의 로봇에 서보를 단 MuJoCo 세계. `step`이 한 주기만큼 진행한다."""

    def __init__(
        self, model_file: Path, servo_specs: Path, period: float, servos: list[tuple[int, str, int]]
    ) -> None:
        spec = mujoco.MjSpec.from_file(str(model_file))
        joints = {joint.name for joint in spec.joints}
        stalls = _stall_torques(servo_specs)
        planned: list[tuple[int, str, int, float]] = []
        for servo_id, joint, model in servos:
            if joint not in joints:
                raise ValueError(f"모르는 관절이다: {joint}")
            if model not in MODEL_NAMES:
                raise ValueError(f"모르는 모델 번호다: {model}")
            stall = stalls[MODEL_NAMES[model]]
            actuator = spec.add_actuator()
            actuator.name = f"servo{servo_id}"
            actuator.target = joint
            actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
            actuator.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            actuator.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            actuator.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
            actuator.forcerange = [-stall, stall]
            planned.append((servo_id, joint, model, stall))
        self.model = spec.compile()
        # 막대 모델은 관절 둘레에서 막대끼리 겹친다. 그대로 두면 부딪힘 힘이 서보를 밀어서 끈다
        # (자기 충돌 검사는 실행 전 궤적 검사가 따로 한다, 0009)
        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0
        self.model.dof_damping[:] = JOINT_DAMPING
        self.model.dof_frictionloss[:] = JOINT_FRICTION
        # 서보를 달지 않은 관절은 거의 잠근다. 힘없이 흔들리면 서보를 단 관절까지 흔든다
        driven = {self.model.jnt_dofadr[self.model.joint(joint).id] for _, joint, _, _ in planned}
        for dof in range(self.model.nv):
            if dof not in driven:
                self.model.dof_damping[dof] = UNDRIVEN_DAMPING
                self.model.dof_frictionloss[dof] = UNDRIVEN_FRICTION
        # 서보의 감쇠와 기어 감쇠를 안정하게 푼다
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.data = mujoco.MjData(self.model)
        self.period = period
        self.substeps = max(1, round(period / self.model.opt.timestep))
        self.time = 0.0
        stall_sts3250 = stalls["STS3250"]
        self.servos: dict[int, Servo] = {}
        for servo_id, joint, model, stall in planned:
            joint_id = self.model.joint(joint).id
            servo = Servo(
                id=servo_id,
                joint=joint,
                model=model,
                stall=stall,
                stiffness=STIFFNESS_STS3250 * stall / stall_sts3250,
                actuator=self.model.actuator(f"servo{servo_id}").id,
                qpos=int(self.model.jnt_qposadr[joint_id]),
                dof=int(self.model.jnt_dofadr[joint_id]),
            )
            servo.flash = dict(EEPROM)
            servo.regs = {**EEPROM, **POWER_ON, "model": model, "id": servo_id}
            self.servos[servo_id] = servo
        mujoco.mj_forward(self.model, self.data)
        self._read_back()

    def step(self) -> None:
        """서보 레지스터로 액추에이터를 정하고, 한 주기만큼 물리를 진행한 뒤 레지스터를 고친다."""
        for servo in self.servos.values():
            self._drive(servo)
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self.time += self.substeps * self.model.opt.timestep
        self._read_back()

    def write(self, servo_id: int, name: str, value: int) -> None:
        self.servos[servo_id].regs[name] = value

    def power_cycle(self, servo_id: int) -> None:
        """서보 전원을 껐다 켠다: RAM은 처음 값이 되고, 잠근 채 쓴 EEPROM 값은 사라진다."""
        servo = self.servos[servo_id]
        servo.regs.update(servo.flash)
        servo.regs.update(POWER_ON)

    def angle(self, servo_id: int) -> float:
        return float(self.data.qpos[self.servos[servo_id].qpos])

    def _drive(self, servo: Servo) -> None:
        model, index = self.model, servo.actuator
        on = servo.regs["torque_enable"] == 1
        kp = servo.stiffness if on else 0.0
        model.actuator_gainprm[index, 0] = kp
        model.actuator_biasprm[index, 1] = -kp
        model.actuator_biasprm[index, 2] = -kp * DAMPING_TIME
        force = servo.stall * servo.regs["torque_limit"] / 1000
        model.actuator_forcerange[index] = (-force, force)
        self.data.ctrl[index] = self._goal_angle(servo)

    def _goal_angle(self, servo: Servo) -> float:
        """목표를 각도 한계 안으로 자른 자리의 관절 각 (한계가 0과 0이면 자르지 않는다)."""
        regs = servo.regs
        goal = regs["goal_position"]
        low, high = regs["min_angle_limit"], regs["max_angle_limit"]
        if not (low == 0 and high == 0):
            goal = min(max(goal, low), high)
        return (goal + regs["position_offset"] - CENTER) * RAD_PER_STEP

    def _read_back(self) -> None:
        for servo in self.servos.values():
            angle = float(self.data.qpos[servo.qpos])
            speed = float(self.data.qvel[servo.dof])
            encoder = round(CENTER + angle / RAD_PER_STEP)
            regs = servo.regs
            regs["present_position"] = (encoder - regs["position_offset"]) % STEPS_PER_TURN
            regs["present_speed"] = round(speed / RAD_PER_STEP)
            force = float(self.data.actuator_force[servo.actuator])
            regs["present_load"] = round(1000 * force / servo.stall)
            regs["moving"] = int(abs(speed) > MOVING_SPEED)
