"""서보 버스 검사. 가짜 서보를 시리얼 포트 자리에 끼워 SDK의 패킷 처리까지 지나간다.

docs/decisions/0009-control-architecture.md의 "실물 백엔드의 오류 처리를 가짜 서보 버스로 시험한다"의
맨 아래층 몫이다 (응답 없음, 깨진 응답, 오류 비트, 포트 예외).
"""

import pytest

from horilab_motor.feetech import registers as reg
from horilab_motor.feetech.bus import (
    BadReply,
    NoReply,
    PortError,
    Refused,
    Reply,
    VerifyError,
)
from horilab_motor.feetech.fake import FakeServo, open_fake_bus
from horilab_motor.feetech.registers import Fault, Register


def test_ping_returns_model() -> None:
    bus, _ = open_fake_bus([FakeServo(3, model=2825)])
    assert bus.ping(3) == 2825


def test_ping_missing_servo_raises_no_reply() -> None:
    bus, _ = open_fake_bus([FakeServo(3)])
    with pytest.raises(NoReply):
        bus.ping(4)


@pytest.mark.parametrize("servo_id", [-1, 253, 254])
def test_rejects_ids_outside_range(servo_id: int) -> None:
    bus, _ = open_fake_bus([FakeServo(3)])
    with pytest.raises(ValueError):
        bus.ping(servo_id)


def test_scan_finds_servos_and_flags_shared_id() -> None:
    servos = [FakeServo(1, 777), FakeServo(3, 2825), FakeServo(5), FakeServo(5)]
    bus, _ = open_fake_bus(servos)
    result = bus.scan(range(8))
    assert result.models == {1: 777, 3: 2825}
    assert result.garbled == [5]
    assert set(bus.stats) == {1, 3, 5}  # 없는 ID의 성적은 남기지 않는다


def test_read_decodes_signed_values() -> None:
    servo = FakeServo(3)
    servo.set(reg.PRESENT_POSITION, 1000)
    servo.set(reg.PRESENT_LOAD, -250)
    servo.set(reg.PRESENT_TEMPERATURE, 31)
    bus, _ = open_fake_bus([servo])
    reply = bus.read(3, reg.PRESENT_POSITION, reg.PRESENT_LOAD, reg.PRESENT_TEMPERATURE)
    assert reply.values == {
        "present_position": 1000,
        "present_load": -250,
        "present_temperature": 31,
    }
    assert reply.fault == Fault.NONE


def test_reply_carries_status_bits() -> None:
    servo = FakeServo(3)
    servo.faults.status = Fault.TEMPERATURE | Fault.OVERLOAD
    bus, _ = open_fake_bus([servo])
    assert bus.read(3, reg.PRESENT_TEMPERATURE).fault == Fault.TEMPERATURE | Fault.OVERLOAD


def test_dump_reads_whole_memory_table() -> None:
    servo = FakeServo(3)
    bus, _ = open_fake_bus([servo])
    assert bus.dump(3) == bytes(servo.memory)


def test_write_ram_register() -> None:
    servo = FakeServo(3)
    bus, _ = open_fake_bus([servo])
    bus.write(3, reg.TORQUE_LIMIT, 300)
    assert servo.get(reg.TORQUE_LIMIT) == 300


@pytest.mark.parametrize(
    ("register", "value"),
    [
        (reg.P_GAIN, 32),  # EEPROM은 write_eeprom으로
        (reg.PRESENT_POSITION, 0),  # 읽기 전용
        (reg.LOCK, 0),  # 잠금은 EEPROM 절차가 다룬다
        (reg.TORQUE_ENABLE, 128),  # 위치 오프셋을 바꾸는 값
    ],
)
def test_write_refuses(register: Register, value: int) -> None:
    bus, _ = open_fake_bus([FakeServo(3)])
    with pytest.raises(ValueError):
        bus.write(3, register, value)


def test_eeprom_written_while_locked_is_lost_at_power_off() -> None:
    # 메모리 표의 잠금 뜻을 가짜 서보가 따르는지. 아래 시험들이 이것에 기댄다.
    servo = FakeServo(3)
    bus, _ = open_fake_bus([servo])
    bus.write(3, reg.TORQUE_LIMIT, 300)  # RAM 쓰기 경로가 되는지만 본다
    servo.write(reg.P_GAIN.address, bytes([16]))
    assert servo.get(reg.P_GAIN) == 16
    servo.power_cycle()
    assert servo.get(reg.P_GAIN) == 32


def test_write_eeprom_persists_and_relocks() -> None:
    servo = FakeServo(3)
    servo.set(reg.P_GAIN, 16)
    bus, _ = open_fake_bus([servo])
    values = {reg.P_GAIN: 32, reg.DEAD_ZONE_POSITIVE: 0, reg.DEAD_ZONE_NEGATIVE: 0}
    bus.write_eeprom(3, values)
    assert servo.get(reg.LOCK) == 1
    servo.power_cycle()
    assert {r: servo.get(r) for r in values} == values


def test_write_eeprom_refuses_while_torque_on() -> None:
    servo = FakeServo(3)
    bus, _ = open_fake_bus([servo])
    bus.write(3, reg.TORQUE_ENABLE, reg.TORQUE_ON)
    with pytest.raises(Refused):
        bus.write_eeprom(3, {reg.P_GAIN: 32})
    assert servo.eeprom_saves == 0


@pytest.mark.parametrize("register", [reg.ID, reg.BAUD_RATE, reg.MODEL, reg.TORQUE_LIMIT])
def test_write_eeprom_rejects_registers(register: Register) -> None:
    bus, _ = open_fake_bus([FakeServo(3)])
    with pytest.raises(ValueError):
        bus.write_eeprom(3, {register: 1})


def test_write_eeprom_relocks_when_a_write_fails() -> None:
    servo = FakeServo(3)
    servo.faults.silent_writes_to = {reg.D_GAIN.address}
    bus, _ = open_fake_bus([servo])
    with pytest.raises(NoReply):
        bus.write_eeprom(3, {reg.P_GAIN: 20, reg.D_GAIN: 20})
    assert servo.get(reg.LOCK) == 1


def test_write_eeprom_detects_value_that_did_not_stick() -> None:
    servo = FakeServo(3)
    servo.faults.ignored_writes_to = {reg.MAX_TEMPERATURE.address}
    bus, _ = open_fake_bus([servo])
    with pytest.raises(VerifyError):
        bus.write_eeprom(3, {reg.MAX_TEMPERATURE: 60})


def test_change_id_persists() -> None:
    servo = FakeServo(1)
    bus, _ = open_fake_bus([servo])
    bus.change_id(1, 3)
    assert servo.id == 3
    assert servo.get(reg.LOCK) == 1
    servo.power_cycle()
    assert servo.id == 3
    with pytest.raises(NoReply):
        bus.ping(1)


def test_change_id_refuses_id_in_use() -> None:
    first, second = FakeServo(1), FakeServo(3)
    bus, _ = open_fake_bus([first, second])
    with pytest.raises(Refused):
        bus.change_id(1, 3)
    assert first.id == 1


def test_sync_read_returns_each_servo() -> None:
    bus, _ = open_fake_bus([FakeServo(1, position=100), FakeServo(3, position=200)])
    results = bus.sync_read([1, 3], *reg.STATE)
    positions: dict[int, int] = {}
    for servo_id, result in results.items():
        assert isinstance(result, Reply)
        positions[servo_id] = result.values["present_position"]
    assert positions == {1: 100, 3: 200}


def test_sync_read_never_returns_stale_values() -> None:
    # SDK의 GroupSyncRead는 응답이 한 패킷보다 짧으면 지난 주기 값을 그대로 돌려준다.
    servo = FakeServo(3, position=500)
    bus, _ = open_fake_bus([servo])
    assert isinstance(bus.sync_read([3], reg.PRESENT_POSITION)[3], Reply)
    servo.faults.silent = True
    assert isinstance(bus.sync_read([3], reg.PRESENT_POSITION)[3], NoReply)
    assert bus.stats[3].consecutive_failures == 1


def test_sync_read_keeps_others_when_one_is_silent() -> None:
    silent, alive = FakeServo(1), FakeServo(3)
    silent.faults.silent = True
    bus, _ = open_fake_bus([silent, alive])
    results = bus.sync_read([1, 3], reg.PRESENT_POSITION)
    assert isinstance(results[1], NoReply)
    assert isinstance(results[3], Reply)


@pytest.mark.parametrize("broken_first", [True, False])
@pytest.mark.parametrize("fault", ["corrupt", "truncate"])
def test_sync_read_reports_broken_reply(fault: str, broken_first: bool) -> None:
    broken, good = FakeServo(1), FakeServo(3)
    setattr(broken.faults, fault, True)
    bus, _ = open_fake_bus([broken, good])
    order = [1, 3] if broken_first else [3, 1]
    results = bus.sync_read(order, reg.PRESENT_POSITION)
    assert isinstance(results[1], BadReply)
    assert isinstance(results[3], Reply)


def test_stats_count_failures_and_reset_on_success() -> None:
    servo = FakeServo(3)
    servo.faults.silent = True
    bus, _ = open_fake_bus([servo])
    for _ in range(3):
        with pytest.raises(NoReply):
            bus.read(3, reg.PRESENT_POSITION)
    assert (bus.stats[3].no_reply, bus.stats[3].consecutive_failures) == (3, 3)
    servo.faults.silent = False
    bus.read(3, reg.PRESENT_POSITION)
    assert (bus.stats[3].ok, bus.stats[3].consecutive_failures) == (1, 0)


def test_port_recovers_after_exception_mid_transaction() -> None:
    # SDK만 쓰면 이때 '사용 중' 표시가 남아 그 뒤 모든 통신이 PORT_BUSY가 된다.
    bus, wire = open_fake_bus([FakeServo(3)])
    wire.raise_on_write = OSError("USB가 빠졌다")
    with pytest.raises(PortError):
        bus.read(3, reg.PRESENT_POSITION)
    assert bus.read(3, reg.PRESENT_POSITION).values["present_position"] == 2048


def test_sync_write_sets_each_servo() -> None:
    first, second = FakeServo(1, position=2048), FakeServo(3, position=2048)
    bus, _ = open_fake_bus([first, second])
    bus.sync_write(reg.TORQUE_ENABLE, {1: reg.TORQUE_ON, 3: reg.TORQUE_ON})
    bus.sync_write(reg.GOAL_POSITION, {1: 1000, 3: 3000})
    assert (first.get(reg.PRESENT_POSITION), second.get(reg.PRESENT_POSITION)) == (1000, 3000)


@pytest.mark.parametrize(("register", "value"), [(reg.P_GAIN, 32), (reg.TORQUE_ENABLE, 128)])
def test_sync_write_refuses(register: Register, value: int) -> None:
    bus, _ = open_fake_bus([FakeServo(1)])
    with pytest.raises(ValueError):
        bus.sync_write(register, {1: value})
