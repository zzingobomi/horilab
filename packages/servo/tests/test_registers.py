"""메모리 표 검사: 겹침, SDK 상수와 맞는지, 부호 표현."""

import importlib
import itertools

import pytest

from horilab_servo.feetech import registers as reg
from horilab_servo.feetech.registers import Encoding, Fault, Register

# SDK는 형식 정보가 없어서 상수는 모듈에서 이름으로 꺼낸다.
SDK_STS = importlib.import_module("scservo_sdk.sms_sts")
SDK_PACKET = importlib.import_module("scservo_sdk.protocol_packet_handler")


def test_names_are_unique() -> None:
    assert len(reg.BY_NAME) == len(reg.ALL)


def test_registers_do_not_overlap() -> None:
    ordered = sorted(reg.ALL, key=lambda r: r.address)
    for before, after in itertools.pairwise(ordered):
        assert before.address + before.size <= after.address, (before.name, after.name)
    assert all(r.size in (1, 2) for r in reg.ALL)
    assert reg.MEMORY_SIZE == 71


@pytest.mark.parametrize(
    ("sdk_name", "register"),
    [
        ("SMS_STS_MODEL_L", reg.MODEL),
        ("SMS_STS_ID", reg.ID),
        ("SMS_STS_BAUD_RATE", reg.BAUD_RATE),
        ("SMS_STS_MIN_ANGLE_LIMIT_L", reg.MIN_ANGLE_LIMIT),
        ("SMS_STS_MAX_ANGLE_LIMIT_L", reg.MAX_ANGLE_LIMIT),
        ("SMS_STS_CW_DEAD", reg.DEAD_ZONE_POSITIVE),
        ("SMS_STS_CCW_DEAD", reg.DEAD_ZONE_NEGATIVE),
        ("SMS_STS_OFS_L", reg.POSITION_OFFSET),
        ("SMS_STS_MODE", reg.MODE),
        ("SMS_STS_TORQUE_ENABLE", reg.TORQUE_ENABLE),
        ("SMS_STS_ACC", reg.ACCELERATION),
        ("SMS_STS_GOAL_POSITION_L", reg.GOAL_POSITION),
        ("SMS_STS_GOAL_TIME_L", reg.GOAL_TIME),
        ("SMS_STS_GOAL_SPEED_L", reg.GOAL_SPEED),
        ("SMS_STS_LOCK", reg.LOCK),
        ("SMS_STS_PRESENT_POSITION_L", reg.PRESENT_POSITION),
        ("SMS_STS_PRESENT_SPEED_L", reg.PRESENT_SPEED),
        ("SMS_STS_PRESENT_LOAD_L", reg.PRESENT_LOAD),
        ("SMS_STS_PRESENT_VOLTAGE", reg.PRESENT_VOLTAGE),
        ("SMS_STS_PRESENT_TEMPERATURE", reg.PRESENT_TEMPERATURE),
        ("SMS_STS_MOVING", reg.MOVING),
        ("SMS_STS_PRESENT_CURRENT_L", reg.PRESENT_CURRENT),
    ],
)
def test_addresses_match_sdk(sdk_name: str, register: Register) -> None:
    assert getattr(SDK_STS, sdk_name) == register.address


@pytest.mark.parametrize(
    ("sdk_name", "fault"),
    [
        ("ERRBIT_VOLTAGE", Fault.VOLTAGE),
        ("ERRBIT_ANGLE", Fault.SENSOR),
        ("ERRBIT_OVERHEAT", Fault.TEMPERATURE),
        ("ERRBIT_OVERELE", Fault.CURRENT),
        ("ERRBIT_OVERLOAD", Fault.OVERLOAD),
    ],
)
def test_fault_bits_match_sdk(sdk_name: str, fault: Fault) -> None:
    assert getattr(SDK_PACKET, sdk_name) == fault


def test_state_block_is_one_read() -> None:
    first = min(r.address for r in reg.STATE)
    end = max(r.address + r.size for r in reg.STATE)
    assert (first, end) == (56, 71)


@pytest.mark.parametrize(
    ("register", "value", "raw"),
    [
        (reg.PRESENT_POSITION, 1000, 1000),
        (reg.GOAL_POSITION, -100, 0x8064),
        (reg.PRESENT_SPEED, -1, 0x8001),
        (reg.PRESENT_LOAD, -500, 0x400 | 500),
        (reg.PRESENT_LOAD, 1000, 1000),
    ],
)
def test_sign_magnitude(register: Register, value: int, raw: int) -> None:
    assert register.encode(value) == raw
    assert register.decode(raw) == value


@pytest.mark.parametrize(
    ("raw", "value"),
    # 메모리 표의 예: 0~2047은 0~2047, 2048~4095는 0~-2047, 4096~6143은 2048~4095, 6144~8191은 -2048~-4095
    [
        (0, 0),
        (2047, 2047),
        (2048, 0),
        (2049, -1),
        (4096, 2048),
        (6143, 4095),
        (6144, -2048),
        (8191, -4095),
    ],
)
def test_offset_encoding(raw: int, value: int) -> None:
    assert reg.POSITION_OFFSET.decode(raw) == value


def test_offset_round_trip() -> None:
    for value in range(-4095, 4096):
        assert reg.POSITION_OFFSET.decode(reg.POSITION_OFFSET.encode(value)) == value


@pytest.mark.parametrize(
    ("register", "value"),
    [
        (reg.P_GAIN, 256),
        (reg.P_GAIN, -1),
        (reg.MAX_TORQUE, 65536),
        (reg.GOAL_POSITION, 32768),
        (reg.PRESENT_LOAD, 1024),
        (reg.POSITION_OFFSET, 4096),
    ],
)
def test_encode_rejects_values_that_do_not_fit(register: Register, value: int) -> None:
    with pytest.raises(ValueError):
        register.encode(value)


def test_every_encoding_is_used() -> None:
    assert {r.encoding for r in reg.ALL} == set(Encoding)
