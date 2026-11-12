"""Feetech STS 계열 서보(STS3215, STS3250, ST-3120)의 메모리 표.

출처는 Waveshare의 "Magnetic Encoder ST Servo - Memory Table Explanation"이다
(https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation, 2026-11-10 확인).
SDK(scservo_sdk)는 이 표의 일부만 상수로 둬서, 표는 여기 한 곳에 둔다.

- 2바이트 값은 작은 주소가 아래 바이트다.
- 표에 적힌 초기값은 TABLE_INITIAL에 둔다. 표가 비워 둔 칸은 모델마다 달라서 없고, 첫 덤프를
  같은 모델끼리 견주어 본다.
- ST-3120은 같은 표를 쓴다고 보고, 받으면 덤프로 확인한다 (docs/hardware.md).
- 현재 전류(69)에 방향 비트가 있는지는 표에 없다. 부호 없이 읽고 실물로 확인한다.
"""

from dataclasses import dataclass
from enum import Enum, IntFlag


class Area(Enum):
    """값이 사는 곳."""

    EEPROM = (
        "eeprom"  # 잠금(55)을 풀고 쓴 값만 전원을 꺼도 남는다. 잠근 채 쓴 값은 꺼질 때까지만 간다
    )
    RAM = "ram"


class Encoding(Enum):
    """레지스터의 부호 없는 값을 정수로 읽는 방법."""

    UNSIGNED = "unsigned"
    SIGN15 = "sign15"  # 부호-크기: 비트 15가 방향 (위치, 속도)
    SIGN10 = "sign10"  # 부호-크기: 비트 10이 방향 (부하)
    OFFSET = "offset"  # 위치 오프셋만의 표현: 비트 11이 방향, 비트 12가 크기 2048


@dataclass(frozen=True)
class Register:
    name: str
    address: int
    size: int  # 바이트 수 (1 또는 2)
    area: Area
    writable: bool
    unit: str  # 한 칸의 뜻
    encoding: Encoding = Encoding.UNSIGNED

    def decode(self, raw: int) -> int:
        """서보가 준 부호 없는 값을 부호 있는 정수로 바꾼다."""
        match self.encoding:
            case Encoding.UNSIGNED:
                return raw
            case Encoding.SIGN15:
                return -(raw & 0x7FFF) if raw & 0x8000 else raw
            case Encoding.SIGN10:
                return -(raw & 0x3FF) if raw & 0x400 else raw
            case Encoding.OFFSET:
                magnitude = (raw & 0x7FF) | (0x800 if raw & 0x1000 else 0)
                return -magnitude if raw & 0x800 else magnitude

    def encode(self, value: int) -> int:
        """정수를 서보에 쓸 부호 없는 값으로 바꾼다. 담을 수 없으면 ValueError."""
        match self.encoding:
            case Encoding.UNSIGNED:
                if not 0 <= value < 1 << (8 * self.size):
                    raise ValueError(f"{self.name}: {value}는 {self.size}바이트에 담을 수 없다")
                return value
            case Encoding.SIGN15:
                return _sign_magnitude(self.name, value, 15)
            case Encoding.SIGN10:
                return _sign_magnitude(self.name, value, 10)
            case Encoding.OFFSET:
                magnitude = abs(value)
                if magnitude > 4095:
                    raise ValueError(f"{self.name}: {value}는 ±4095 밖이다")
                raw = (magnitude & 0x7FF) | (0x1000 if magnitude & 0x800 else 0)
                return raw | 0x800 if value < 0 else raw


def _sign_magnitude(name: str, value: int, bit: int) -> int:
    if abs(value) >= 1 << bit:
        raise ValueError(f"{name}: {value}는 크기 비트 {bit}개에 담을 수 없다")
    return (-value) | (1 << bit) if value < 0 else value


def _eeprom(name: str, address: int, size: int, unit: str, **kwargs: Encoding) -> Register:
    return Register(name, address, size, Area.EEPROM, True, unit, **kwargs)


def _ram(name: str, address: int, size: int, unit: str, **kwargs: Encoding) -> Register:
    return Register(name, address, size, Area.RAM, True, unit, **kwargs)


def _feedback(name: str, address: int, size: int, unit: str, **kwargs: Encoding) -> Register:
    return Register(name, address, size, Area.RAM, False, unit, **kwargs)


# 버전 정보 (읽기 전용)
FIRMWARE_MAJOR = Register("firmware_major", 0, 1, Area.EEPROM, False, "-")
FIRMWARE_MINOR = Register("firmware_minor", 1, 1, Area.EEPROM, False, "-")
ENDIAN = Register("endian", 2, 1, Area.EEPROM, False, "0이면 작은 바이트가 앞")
# 표는 3과 4를 서보 버전 둘로 적지만, SDK와 다른 자료는 3~4를 2바이트 모델 번호로 읽는다
MODEL = Register("model", 3, 2, Area.EEPROM, False, "모델 번호")

# EEPROM 설정
ID = _eeprom("id", 5, 1, "0~253")
BAUD_RATE = _eeprom("baud_rate", 6, 1, "코드 (BAUD_RATES)")
RESPONSE_DELAY = _eeprom(
    "response_delay",
    7,
    1,
    "2 µs (메모리 표에는 예약. LeRobot과 rustypot은 응답 지연으로 쓴다, 실물로 확인)",
)
RESPONSE_LEVEL = _eeprom("response_level", 8, 1, "0: 읽기·핑에만 응답, 1: 모든 명령에 응답")
MIN_ANGLE_LIMIT = _eeprom("min_angle_limit", 9, 2, "step (0.087°)")
MAX_ANGLE_LIMIT = _eeprom("max_angle_limit", 11, 2, "step (0.087°)")
MAX_TEMPERATURE = _eeprom("max_temperature", 13, 1, "°C")
MAX_VOLTAGE = _eeprom("max_voltage", 14, 1, "0.1 V")
MIN_VOLTAGE = _eeprom("min_voltage", 15, 1, "0.1 V")
MAX_TORQUE = _eeprom("max_torque", 16, 2, "0.1 % (켤 때 토크 제한(48)으로 옮겨진다)")
PHASE = _eeprom("phase", 18, 1, "비트 (특수 기능, 바꾸지 않는다)")
UNLOAD_CONDITION = _eeprom("unload_condition", 19, 1, "비트 (Fault): 켜진 보호")
LED_ALARM_CONDITION = _eeprom("led_alarm_condition", 20, 1, "비트 (Fault): LED로 알릴 것")
P_GAIN = _eeprom("p_gain", 21, 1, "-")
D_GAIN = _eeprom("d_gain", 22, 1, "-")
I_GAIN = _eeprom("i_gain", 23, 1, "-")
MIN_STARTING_TORQUE = _eeprom("min_starting_torque", 24, 1, "0.1 %")
INTEGRAL_LIMIT = _eeprom("integral_limit", 25, 1, "×4, 0이면 끔")
DEAD_ZONE_POSITIVE = _eeprom("dead_zone_positive", 26, 1, "step (0.087°)")
DEAD_ZONE_NEGATIVE = _eeprom("dead_zone_negative", 27, 1, "step (0.087°)")
PROTECTION_CURRENT = _eeprom("protection_current", 28, 2, "6.5 mA")
ANGLE_RESOLUTION = _eeprom("angle_resolution", 30, 1, "배")
POSITION_OFFSET = _eeprom("position_offset", 31, 2, "step", encoding=Encoding.OFFSET)
MODE = _eeprom("mode", 33, 1, "0: 위치, 1: 일정 속도, 2: PWM, 3: 스텝")
HOLDING_TORQUE = _eeprom("holding_torque", 34, 1, "1 % (과부하 보호에 들어간 뒤 내는 토크)")
PROTECTION_TIME = _eeprom("protection_time", 35, 1, "10 ms (과부하 토크를 넘어 버틸 시간)")
OVERLOAD_TORQUE = _eeprom("overload_torque", 36, 1, "1 % (과부하 보호 시간을 재기 시작하는 토크)")
SPEED_P_GAIN = _eeprom("speed_p_gain", 37, 1, "- (일정 속도 모드)")
OVERCURRENT_PROTECTION_TIME = _eeprom("overcurrent_protection_time", 38, 1, "10 ms")
SPEED_I_GAIN = _eeprom("speed_i_gain", 39, 1, "- (일정 속도 모드)")

# RAM 제어
TORQUE_ENABLE = _ram("torque_enable", 40, 1, "0: 끔, 1: 켬, 2: 댐핑")
ACCELERATION = _ram("acceleration", 41, 1, "100 step/s² (0이면 최대)")
GOAL_POSITION = _ram("goal_position", 42, 2, "step", encoding=Encoding.SIGN15)
GOAL_TIME = _ram("goal_time", 44, 2, "PWM 모드의 출력 (0.1 %)", encoding=Encoding.SIGN10)
GOAL_SPEED = _ram(
    "goal_speed", 46, 2, "phase 비트 2에 따라 50 step/s 또는 1 step/s", encoding=Encoding.SIGN15
)
TORQUE_LIMIT = _ram("torque_limit", 48, 2, "0.1 %")
LOCK = _ram("lock", 55, 1, "0: EEPROM 쓰기를 남김, 1: 남기지 않음")

# RAM 상태 (읽기 전용)
PRESENT_POSITION = _feedback("present_position", 56, 2, "step", encoding=Encoding.SIGN15)
PRESENT_SPEED = _feedback(
    "present_speed", 58, 2, "goal_speed와 같은 단위", encoding=Encoding.SIGN15
)
PRESENT_LOAD = _feedback("present_load", 60, 2, "0.1 % (모터 구동 듀티)", encoding=Encoding.SIGN10)
PRESENT_VOLTAGE = _feedback("present_voltage", 62, 1, "0.1 V")
PRESENT_TEMPERATURE = _feedback("present_temperature", 63, 1, "°C")
ASYNC_WRITE_FLAG = _feedback("async_write_flag", 64, 1, "-")
STATUS = _feedback("status", 65, 1, "비트 (Fault)")
MOVING = _feedback("moving", 66, 1, "1이면 목표로 가는 중")
PRESENT_GOAL = _feedback("present_goal", 67, 2, "step", encoding=Encoding.SIGN15)
PRESENT_CURRENT = _feedback("present_current", 69, 2, "6.5 mA")

ALL: tuple[Register, ...] = (
    FIRMWARE_MAJOR,
    FIRMWARE_MINOR,
    ENDIAN,
    MODEL,
    ID,
    BAUD_RATE,
    RESPONSE_DELAY,
    RESPONSE_LEVEL,
    MIN_ANGLE_LIMIT,
    MAX_ANGLE_LIMIT,
    MAX_TEMPERATURE,
    MAX_VOLTAGE,
    MIN_VOLTAGE,
    MAX_TORQUE,
    PHASE,
    UNLOAD_CONDITION,
    LED_ALARM_CONDITION,
    P_GAIN,
    D_GAIN,
    I_GAIN,
    MIN_STARTING_TORQUE,
    INTEGRAL_LIMIT,
    DEAD_ZONE_POSITIVE,
    DEAD_ZONE_NEGATIVE,
    PROTECTION_CURRENT,
    ANGLE_RESOLUTION,
    POSITION_OFFSET,
    MODE,
    HOLDING_TORQUE,
    PROTECTION_TIME,
    OVERLOAD_TORQUE,
    SPEED_P_GAIN,
    OVERCURRENT_PROTECTION_TIME,
    SPEED_I_GAIN,
    TORQUE_ENABLE,
    ACCELERATION,
    GOAL_POSITION,
    GOAL_TIME,
    GOAL_SPEED,
    TORQUE_LIMIT,
    LOCK,
    PRESENT_POSITION,
    PRESENT_SPEED,
    PRESENT_LOAD,
    PRESENT_VOLTAGE,
    PRESENT_TEMPERATURE,
    ASYNC_WRITE_FLAG,
    STATUS,
    MOVING,
    PRESENT_GOAL,
    PRESENT_CURRENT,
)
BY_NAME: dict[str, Register] = {r.name: r for r in ALL}

# 메모리 표에 적힌 초기값 (2026-11-11 확인). 최대 입력 전압(14), Phase(18), 보호·알림 조건(19, 20),
# P·D(21, 22), 최소 시작 토크(24), 속도 고리 게인(37, 39), 7번은 표가 비워 둬서 없다. RAM은 켰을 때의
# 값이다. 덤프가 이 값과 다른 칸을 보여 주고, 값 적용(0024 만드는 순서 3번)이 robot.toml에 정하지
# 않은 칸을 이 값으로 맞춘다. Rust 표(registers.rs의 TABLE_INITIAL)와 같은지는 test_rust_table.py가 본다.
TABLE_INITIAL: dict[Register, int] = {
    ENDIAN: 0,
    ID: 1,
    BAUD_RATE: 0,
    RESPONSE_LEVEL: 1,
    MIN_ANGLE_LIMIT: 0,
    MAX_ANGLE_LIMIT: 4095,
    MAX_TEMPERATURE: 70,
    MIN_VOLTAGE: 40,
    MAX_TORQUE: 1000,
    I_GAIN: 0,
    INTEGRAL_LIMIT: 0,
    DEAD_ZONE_POSITIVE: 1,
    DEAD_ZONE_NEGATIVE: 1,
    PROTECTION_CURRENT: 511,
    ANGLE_RESOLUTION: 1,
    POSITION_OFFSET: 0,
    MODE: 0,
    HOLDING_TORQUE: 20,
    PROTECTION_TIME: 200,
    OVERLOAD_TORQUE: 80,
    OVERCURRENT_PROTECTION_TIME: 200,
    TORQUE_ENABLE: 0,
    ACCELERATION: 0,
    LOCK: 1,
}

# 덤프는 주소 0부터 마지막 레지스터 끝까지 한 번에 읽는다 (응답 최대 길이 안).
MEMORY_SIZE = max(r.address + r.size for r in ALL)

# 운전 중에 한 번의 동기 읽기로 읽는 상태 묶음 (56~70)
STATE: tuple[Register, ...] = (
    PRESENT_POSITION,
    PRESENT_SPEED,
    PRESENT_LOAD,
    PRESENT_VOLTAGE,
    PRESENT_TEMPERATURE,
    STATUS,
    MOVING,
    PRESENT_GOAL,
    PRESENT_CURRENT,
)


class Fault(IntFlag):
    """상태 바이트(응답의 오류 바이트, 레지스터 65)와 보호·알림 조건(19, 20)의 비트."""

    NONE = 0
    VOLTAGE = 0x01  # 전압이 범위 밖
    SENSOR = 0x02  # 자석 엔코더 이상
    TEMPERATURE = 0x04  # 과열
    CURRENT = 0x08  # 과전류
    OVERLOAD = 0x20  # 과부하


# 토크 스위치에 써도 되는 값. 128은 현재 위치를 2048로 다시 잡으며 위치 오프셋(EEPROM)을 바꿔서 막는다.
TORQUE_OFF = 0
TORQUE_ON = 1
TORQUE_DAMPING = 2
TORQUE_VALUES = (TORQUE_OFF, TORQUE_ON, TORQUE_DAMPING)

# 통신 속도 코드 (레지스터 6)
BAUD_RATES: dict[int, int] = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 76_800,
    6: 57_600,
    7: 38_400,
}

# 모델 번호 (레지스터 3~4). 가진 서보에서 읽은 값이다 (docs/hardware.md). ST-3120은 받으면 더한다.
MODELS: dict[int, str] = {777: "STS3215", 2825: "STS3250"}

STEPS_PER_TURN = 4096
MAX_ID = 252  # SDK의 최대 ID. 254(0xFE)는 방송용이다
