"""가짜 Feetech 서보 버스. 실물 없이 bus 층과 그 위를 시험한다.

[0009](docs/decisions/0009-control-architecture.md)의 "실물 백엔드의 오류 처리를 가짜 서보 버스로
시험한다"에 쓴다. 시리얼 포트 자리에 끼워서 SDK의 패킷 처리까지 그대로 지나가게 한다.

- 서보 메모리는 메모리 표(registers.py)를 따른다. 기본값은 시험용이고 실물의 기본값이 아니다.
- EEPROM은 잠금(55)을 푼 채 쓴 값만 전원을 다시 켜도 남는다 (power_cycle).
- 토크를 켜거나 토크가 켜진 채 목표를 쓰면 현재 위치가 곧바로 목표가 된다.
- 응답 없음, 깨진 응답, 잘린 응답, 상태 바이트의 오류 비트, 같은 ID 둘, 포트 예외를 일부러 낸다.
"""

from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field

from horilab_motor.feetech import registers as reg
from horilab_motor.feetech.bus import Bus, SerialPort
from horilab_motor.feetech.registers import Area, Fault, Register

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

_EEPROM_END = max(r.address + r.size for r in reg.ALL if r.area is Area.EEPROM)
_READ_ONLY = {
    address for r in reg.ALL if not r.writable for address in range(r.address, r.address + r.size)
}


@dataclass
class Faults:
    """일부러 내는 고장."""

    silent: bool = False  # 응답하지 않는다
    corrupt: bool = False  # 체크섬이 틀린 응답을 보낸다
    truncate: bool = False  # 응답의 앞 절반만 보낸다
    status: Fault = Fault.NONE  # 응답의 상태 바이트
    # 이 주소에 쓰면 응답하지 않는다
    silent_writes_to: set[int] = field(default_factory=set[int])
    # 이 주소에 쓰면 응답은 하지만 값을 바꾸지 않는다
    ignored_writes_to: set[int] = field(default_factory=set[int])


class FakeServo:
    """서보 하나의 메모리와 동작."""

    def __init__(self, servo_id: int, model: int = 2825, *, position: int = 2048) -> None:
        self.memory = bytearray(reg.MEMORY_SIZE)
        defaults = {
            reg.FIRMWARE_MAJOR: 3,
            reg.FIRMWARE_MINOR: 10,
            reg.MODEL: model,
            reg.ID: servo_id,
            reg.BAUD_RATE: 0,
            reg.RESPONSE_LEVEL: 1,
            reg.MIN_ANGLE_LIMIT: 0,
            reg.MAX_ANGLE_LIMIT: 4095,
            reg.MAX_TEMPERATURE: 70,
            reg.MAX_VOLTAGE: 140,
            reg.MIN_VOLTAGE: 40,
            reg.MAX_TORQUE: 1000,
            reg.UNLOAD_CONDITION: 44,
            reg.LED_ALARM_CONDITION: 47,
            reg.P_GAIN: 32,
            reg.D_GAIN: 32,
            reg.I_GAIN: 0,
            reg.DEAD_ZONE_POSITIVE: 1,
            reg.DEAD_ZONE_NEGATIVE: 1,
            reg.PROTECTION_CURRENT: 511,
            reg.ANGLE_RESOLUTION: 1,
            reg.HOLDING_TORQUE: 20,
            reg.PROTECTION_TIME: 200,
            reg.OVERLOAD_TORQUE: 80,
            reg.OVERCURRENT_PROTECTION_TIME: 200,
        }
        for register, value in defaults.items():
            self.set(register, value)
        self.flash = bytes(self.memory[:_EEPROM_END])  # 전원을 꺼도 남는 EEPROM
        self.eeprom_saves = 0  # flash에 쓴 횟수 (EEPROM 수명)
        self.faults = Faults()
        self._power_on(position)

    @property
    def id(self) -> int:
        return self.memory[reg.ID.address]

    def get(self, register: Register) -> int:
        raw = self.memory[register.address]
        if register.size == 2:
            raw |= self.memory[register.address + 1] << 8
        return register.decode(raw)

    def set(self, register: Register, value: int) -> None:
        """시험에서 상태를 만들 때 쓴다. 읽기 전용 레지스터도 바꾼다."""
        raw = register.encode(value)
        self.memory[register.address] = raw & 0xFF
        if register.size == 2:
            self.memory[register.address + 1] = (raw >> 8) & 0xFF

    def power_cycle(self) -> None:
        """전원을 껐다 켠다. EEPROM은 flash에서 되살리고 RAM은 처음 값이 된다."""
        position = self.get(reg.PRESENT_POSITION)
        self.memory[:_EEPROM_END] = self.flash
        self.memory[_EEPROM_END:] = bytes(len(self.memory) - _EEPROM_END)
        self._power_on(position)

    def _power_on(self, position: int) -> None:
        self.set(reg.TORQUE_LIMIT, self.get(reg.MAX_TORQUE))
        self.set(reg.LOCK, 1)
        self.set(reg.PRESENT_POSITION, position)
        self.set(reg.GOAL_POSITION, position)
        self.set(reg.PRESENT_GOAL, position)
        self.set(reg.PRESENT_VOLTAGE, 120)
        self.set(reg.PRESENT_TEMPERATURE, 25)

    def read(self, address: int, length: int) -> bytes:
        return bytes(self.memory[address : address + length])

    def write(self, address: int, data: bytes) -> None:
        for offset, value in enumerate(data):
            target = address + offset
            ignored = _READ_ONLY | self.faults.ignored_writes_to
            if target < len(self.memory) and target not in ignored:
                self.memory[target] = value
        if address < _EEPROM_END and self.memory[reg.LOCK.address] == 0:
            self.flash = bytes(self.memory[:_EEPROM_END])
            self.eeprom_saves += 1
        touched = range(address, address + len(data))
        moves = reg.TORQUE_ENABLE.address in touched or reg.GOAL_POSITION.address in touched
        if moves and self.memory[reg.TORQUE_ENABLE.address] == reg.TORQUE_ON:
            goal = self.get(reg.GOAL_POSITION)
            self.set(reg.PRESENT_POSITION, goal)
            self.set(reg.PRESENT_GOAL, goal)


class FakeWire:
    """서보들이 물린 버스 선. 받은 명령 패킷에 서보들이 낼 응답을 만든다."""

    def __init__(self, servos: Iterable[FakeServo]) -> None:
        self.servos = list(servos)
        self.raise_on_write: OSError | None = (
            None  # 다음 쓰기에서 포트 예외를 낸다 (USB가 빠진 것처럼)
        )

    def find(self, servo_id: int) -> list[FakeServo]:
        return [s for s in self.servos if s.id == servo_id]

    def handle(self, packet: bytes) -> bytes:
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
            return b""
        servo_id, length, instruction = packet[2], packet[3], packet[4]
        if len(packet) != length + 4 or (~sum(packet[2:-1])) & 0xFF != packet[-1]:
            return b""  # 실물도 체크섬이 틀린 명령은 무시한다
        params = packet[5:-1]
        if instruction == INST_SYNC_READ:
            address, size = params[0], params[1]
            return b"".join(self._reply(i, lambda s: s.read(address, size)) for i in params[2:])
        if instruction == INST_SYNC_WRITE:
            address, size = params[0], params[1]
            for start in range(2, len(params), size + 1):
                for servo in self.find(params[start]):
                    servo.write(address, params[start + 1 : start + 1 + size])
            return b""
        if servo_id == BROADCAST_ID:
            return b""
        if instruction == INST_PING:
            return self._reply(servo_id, lambda s: b"")
        if instruction == INST_READ:
            address, size = params[0], params[1]
            return self._reply(servo_id, lambda s: s.read(address, size))
        if instruction == INST_WRITE:
            address, data = params[0], params[1:]
            targets = self.find(servo_id)
            for servo in targets:
                servo.write(address, data)
            if any(address in s.faults.silent_writes_to for s in targets):
                return b""
            # 모든 명령에 응답하는 설정(8번이 1)일 때만 쓰기에 응답한다. ID를 바꿨으면 새 ID로 답한다.
            replying = [s for s in targets if s.memory[reg.RESPONSE_LEVEL.address] == 1]
            return b"".join(self._status(s, b"") for s in replying)
        return b""

    def _reply(self, servo_id: int, payload: Callable[[FakeServo], bytes]) -> bytes:
        servos = self.find(servo_id)
        if len(servos) > 1:
            # 같은 ID의 서보 둘이 동시에 답하면 선 위에서 섞인다
            replies = [self._status(s, payload(s)) for s in servos]
            mixed = bytes(a | b for a, b in zip(replies[0], replies[1], strict=False))
            return mixed[:3] + bytes([mixed[3] ^ 0x5A]) + mixed[4:]
        return b"".join(self._status(s, payload(s)) for s in servos)

    @staticmethod
    def _status(servo: FakeServo, data: bytes) -> bytes:
        faults = servo.faults
        if faults.silent:
            return b""
        body = bytes([servo.id, len(data) + 2, int(faults.status), *data])
        checksum = (~sum(body)) & 0xFF
        if faults.corrupt:
            checksum ^= 0xFF
        reply = b"\xff\xff" + body + bytes([checksum])
        return reply[: len(reply) // 2] if faults.truncate else reply


class FakeSerial:
    """pyserial의 Serial 자리에 끼우는 가짜. SDK가 부르는 것만 있다."""

    def __init__(self, wire: FakeWire) -> None:
        self._wire = wire
        self._received = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self._received)

    def write(self, data: Sequence[int] | bytes) -> int:
        if self._wire.raise_on_write is not None:
            error, self._wire.raise_on_write = self._wire.raise_on_write, None
            raise error
        packet = bytes(data)
        self._received += self._wire.handle(packet)
        return len(packet)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._received[:size])
        del self._received[:size]
        return chunk

    def reset_input_buffer(self) -> None:
        self._received.clear()

    def close(self) -> None:
        pass


class FakePort(SerialPort):
    """가짜 선에 붙는 SDK 포트."""

    def __init__(self, wire: FakeWire, latency_ms: float) -> None:
        super().__init__("fake", latency_ms)
        self._wire = wire

    def setupPort(self, cflag_baud: int) -> bool:
        self.ser = FakeSerial(self._wire)
        self.is_open = True
        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0
        return True


def open_fake_bus(servos: Iterable[FakeServo], *, latency_ms: float = 1.0) -> tuple[Bus, FakeWire]:
    """가짜 서보들을 물린 버스를 연다. 응답 여유를 짧게 둬서 응답 없음 시험이 빨리 끝난다."""
    wire = FakeWire(servos)
    port = FakePort(wire, latency_ms)
    port.openPort()
    return Bus(port), wire
