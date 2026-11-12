"""Feetech 서보 버스: 공식 SDK(scservo_sdk)로 패킷을 주고받고, SDK가 하지 않는 일을 여기서 한다.

- 레지스터는 이름으로 읽고 쓰고, 부호도 여기서 푼다 (registers.py). 값은 서보의 raw 단위 그대로다.
  관절 이름과 rad는 이 위의 층이 다룬다.
- 동기 읽기는 SDK의 GroupSyncRead를 쓰지 않는다. 응답이 한 패킷보다 짧으면 지난 주기 값을
  그대로 돌려주기 때문이다. 받은 바이트를 서보마다 나눠 확인하고, 응답이 없는 서보는 없다고 알린다.
- EEPROM은 세 길로만 쓴다: write_eeprom(잠금 풀기 → 쓰기 → 잠그기 → 다시 읽어 확인), change_id,
  write_trial(잠근 채 바꿔 보기: 바로 듣고 전원을 끄면 사라진다).
- SDK는 스레드 안전이 없고, 통신 도중 예외가 나면 포트가 계속 '사용 중'으로 남는다.
  그래서 모든 통신을 잠금 하나로 묶고, 끝날 때마다 사용 중 표시를 되돌린다.
"""

import threading
import time
from collections.abc import Generator, Iterable, Mapping, Sequence
from contextlib import contextmanager, suppress
from dataclasses import dataclass
from typing import NoReturn, Self

import serial
from scservo_sdk import (
    COMM_RX_CORRUPT,
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    PortHandler,
    protocol_packet_handler,
)

from horilab_servo.feetech import registers as reg
from horilab_servo.feetech.registers import Area, Fault, Register

# 잠근 채 바꿔 볼 수 있는 레지스터: P·I·D, 응답 지연(7번), 그리고 맨 서보 확인에서 서보의 동작을 보려고
# 잠깐 바꾸는 각도 한계, 전압 하한, 힘을 빼는 조건, 위치 오프셋, 과부하 보호 설정
# (docs/decisions/0024-servo-bus.md)
TRIAL_REGISTERS = (
    reg.P_GAIN,
    reg.I_GAIN,
    reg.D_GAIN,
    reg.RESPONSE_DELAY,
    reg.MIN_ANGLE_LIMIT,
    reg.MAX_ANGLE_LIMIT,
    reg.MIN_VOLTAGE,
    reg.UNLOAD_CONDITION,
    reg.POSITION_OFFSET,
    reg.HOLDING_TORQUE,
    reg.PROTECTION_TIME,
    reg.OVERLOAD_TORQUE,
)

# 한 번에 읽는 최대 바이트. SDK는 응답의 길이 칸(바이트 수 + 2)이 250을 넘으면 버린다.
MAX_READ = 248


class BusError(Exception):
    """서보와 통신하지 못했다."""

    def __init__(self, message: str, servo_id: int | None = None) -> None:
        super().__init__(message if servo_id is None else f"서보 {servo_id}: {message}")
        self.servo_id = servo_id


class NoReply(BusError):
    """응답이 오지 않았다 (전원이 없거나, 선이 빠졌거나, 그 ID가 없다)."""


class BadReply(BusError):
    """응답이 깨졌다 (체크섬, 길이). 같은 ID의 서보가 둘이어도 이렇게 된다."""


class PortError(BusError):
    """포트를 열거나 쓰지 못했다."""


class VerifyError(BusError):
    """쓴 값을 다시 읽었더니 다르다."""


class Refused(Exception):
    """안전 조건 때문에 하지 않았다 (토크가 켜져 있음, 새 ID가 이미 있음 등)."""


@dataclass(frozen=True)
class Reply:
    values: dict[str, int]  # 레지스터 이름 → 부호를 푼 값
    fault: Fault  # 응답의 상태 바이트


@dataclass
class LinkStats:
    """서보 하나와의 통신 성적. 안전 검사는 연속 실패를 본다 (docs/decisions/0009)."""

    ok: int = 0
    no_reply: int = 0
    bad_reply: int = 0
    consecutive_failures: int = 0

    def record(self, error: NoReply | BadReply | None) -> None:
        if error is None:
            self.ok += 1
            self.consecutive_failures = 0
            return
        if isinstance(error, NoReply):
            self.no_reply += 1
        else:
            self.bad_reply += 1
        self.consecutive_failures += 1


@dataclass(frozen=True)
class ScanResult:
    models: dict[int, int]  # ID → 모델 번호
    garbled: list[int]  # 응답이 깨진 ID. 같은 ID의 서보가 둘일 수 있다


class SerialPort(PortHandler):
    """SDK 포트. 응답을 기다리는 여유를 정할 수 있게 하고, 시간은 perf_counter로 잰다.

    SDK는 여유를 모듈 상수(50 ms)로 두고 time.time()으로 잰다. Windows의 Python 3.12에서
    time.time()은 약 15.6 ms 단위로 움직여서 기다리는 시간이 들쭉날쭉하다.
    """

    def __init__(self, port_name: str, latency_ms: float) -> None:
        super().__init__(port_name)
        self.latency_ms = latency_ms

    def setPacketTimeout(self, packet_length: int) -> None:
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = self.tx_time_per_byte * (packet_length + 3) + self.latency_ms

    def getCurrentTime(self) -> float:
        return time.perf_counter() * 1000.0


class Bus:
    """서보 버스 하나. 한 포트를 쓰는 코드는 모두 이 객체 하나를 거친다."""

    def __init__(self, port: PortHandler) -> None:
        self._port = port
        self._packets = protocol_packet_handler(port, 0)  # STS는 작은 바이트가 앞이다 (end=0)
        self._lock = threading.Lock()
        self.stats: dict[int, LinkStats] = {}

    @classmethod
    def open(cls, port_name: str, *, baud: int = 1_000_000, latency_ms: float = 50.0) -> Self:
        """실물 포트를 연다. latency_ms는 응답 하나를 더 기다리는 여유다 (SDK 기본값 50 ms)."""
        port = SerialPort(port_name, latency_ms)
        port.baudrate = baud
        try:
            opened = port.openPort()
        except serial.SerialException as error:
            raise PortError(f"{port_name}을 열지 못했다: {error}") from error
        if not opened:
            raise PortError(f"SDK가 지원하지 않는 통신 속도다: {baud}")
        return cls(port)

    def close(self) -> None:
        with self._lock:
            if self._port.is_open:
                self._port.closePort()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # 찾기와 읽기

    def ping(self, servo_id: int) -> int:
        """서보가 응답하면 모델 번호를 돌려준다."""
        _check_id(servo_id)
        with self._transaction(servo_id):
            model, result, _ = self._packets.ping(servo_id)
        self._check(servo_id, result, "핑")
        self._succeed(servo_id)
        return model

    def scan(self, ids: Iterable[int] = range(reg.MAX_ID + 1)) -> ScanResult:
        """ID마다 핑을 보내 응답하는 서보를 찾는다. 없는 ID마다 응답 여유만큼 기다린다."""
        models: dict[int, int] = {}
        garbled: list[int] = []
        for servo_id in ids:
            try:
                models[servo_id] = self.ping(servo_id)
            except NoReply:
                self.stats.pop(servo_id, None)  # 없는 ID의 성적은 남기지 않는다
            except BadReply:
                garbled.append(servo_id)
        return ScanResult(models, garbled)

    def read(self, servo_id: int, *registers: Register) -> Reply:
        """레지스터들을 한 번의 읽기로 읽는다. 떨어진 주소면 그 사이까지 한 덩어리로 읽는다."""
        first, length = _span(registers)
        data, fault = self._read_bytes(servo_id, first, length)
        return Reply(_decode(registers, data, first), fault)

    def dump(self, servo_id: int) -> bytes:
        """주소 0부터 메모리 표 끝까지의 바이트. 서보 설정을 기록할 때 쓴다."""
        data, _ = self._read_bytes(servo_id, 0, reg.MEMORY_SIZE)
        return bytes(data)

    def read_raw(self, servo_id: int, address: int, length: int) -> bytes:
        """주소와 길이로 바이트를 읽는다. 메모리 표 밖을 덤프할 때 쓴다. 읽기는 서보를 바꾸지 않는다."""
        if address < 0 or length < 1 or address + length > 256 or length > MAX_READ:
            raise ValueError(f"{address}번부터 {length}바이트는 한 번에 읽지 않는다")
        data, _ = self._read_bytes(servo_id, address, length)
        return bytes(data)

    def sync_read(self, ids: Sequence[int], *registers: Register) -> dict[int, Reply | BusError]:
        """여러 서보에서 같은 레지스터들을 한 번에 읽는다.

        서보마다 결과를 돌려준다. 응답이 없거나 깨진 서보는 Reply 대신 그 오류가 들어간다.
        """
        for servo_id in ids:
            _check_id(servo_id)
        if len(set(ids)) != len(ids):
            raise ValueError(f"ID가 겹친다: {list(ids)}")
        first, length = _span(registers)
        with self._transaction(None):
            result = self._packets.syncReadTx(first, length, list(ids), len(ids))
            if result != COMM_SUCCESS:
                raise PortError(f"동기 읽기 보내기: {self._packets.getTxRxResult(result)}")
            _, raw = self._packets.syncReadRx(length, len(ids))
        good, broken = _split_replies(raw, length)
        results: dict[int, Reply | BusError] = {}
        for servo_id in ids:
            if servo_id in good:
                fault, data = good[servo_id]
                results[servo_id] = Reply(_decode(registers, data, first), fault)
                self._succeed(servo_id)
                continue
            error = (
                BadReply("동기 읽기 응답이 깨졌다", servo_id)
                if servo_id in broken
                else NoReply("동기 읽기에 응답하지 않았다", servo_id)
            )
            self._stats(servo_id).record(error)
            results[servo_id] = error
        return results

    # 쓰기

    def write(self, servo_id: int, register: Register, value: int) -> Fault:
        """RAM 레지스터 하나를 쓰고, 응답의 상태 바이트를 돌려준다. EEPROM은 write_eeprom으로 쓴다."""
        _check_ram_write(register, value)
        return self._write_bytes(servo_id, register.address, _to_bytes(register, value))

    def sync_write(self, register: Register, values: Mapping[int, int]) -> None:
        """여러 서보의 같은 RAM 레지스터를 한 번에 쓴다. 응답이 없어서 결과는 다음 읽기로 본다."""
        param: list[int] = []
        for servo_id, value in values.items():
            _check_id(servo_id)
            _check_ram_write(register, value)
            param += [servo_id, *_to_bytes(register, value)]
        if not param:
            return
        with self._transaction(None):
            result = self._packets.syncWriteTxOnly(
                register.address, register.size, param, len(param)
            )
        if result != COMM_SUCCESS:
            raise PortError(f"동기 쓰기: {self._packets.getTxRxResult(result)}")

    def write_eeprom(self, servo_id: int, values: Mapping[Register, int]) -> None:
        """EEPROM 값을 전원을 꺼도 남게 쓰고, 다시 읽어 확인한다.

        토크가 켜져 있으면 쓰지 않는다. ID는 change_id로 바꾸고, 통신 속도는 바꾸지 않는다.
        EEPROM은 쓰는 횟수에 수명이 있어서, 값이 같은지는 부르는 쪽이 먼저 본다.
        """
        if not values:
            return
        for register, value in values.items():
            if register.area is not Area.EEPROM or not register.writable:
                raise ValueError(f"{register.name}은 쓸 수 있는 EEPROM 레지스터가 아니다")
            if register in (reg.ID, reg.BAUD_RATE):
                raise ValueError(f"{register.name}은 write_eeprom으로 바꾸지 않는다")
            register.encode(value)  # 쓰기 전에 범위를 모두 확인한다
        self._require_torque_off(servo_id)
        with self._unlocked(servo_id, relock_ids=(servo_id,)):
            for register, value in values.items():
                self._write_bytes(servo_id, register.address, _to_bytes(register, value))
        self._verify(servo_id, values)

    def write_trial(self, servo_id: int, values: Mapping[Register, int]) -> None:
        """잠근 채 바꿔 보기: 바로 듣고, 전원을 끄면 사라진다. 토크가 켜져 있어도 된다.

        TRIAL_REGISTERS만 받는다. 잠금이 풀린 채 쓰면 값이 남으므로 먼저 잠근다. 바꾼 값을 기록에
        남기는 것은 부르는 쪽이 한다.
        """
        if not values:
            return
        for register, value in values.items():
            if register not in TRIAL_REGISTERS:
                raise ValueError(f"{register.name}은 잠근 채 바꿔 보지 않는다 (TRIAL_REGISTERS만)")
            register.encode(value)
        self._write_bytes(servo_id, reg.LOCK.address, [1])
        for register, value in values.items():
            self._write_bytes(servo_id, register.address, _to_bytes(register, value))
        self._verify(servo_id, values)

    def change_id(self, old_id: int, new_id: int) -> None:
        """서보 ID를 바꾼다. 버스에 서보가 하나만 물렸을 때만 쓴다 (부르는 쪽이 확인한다).

        ID를 쓰면 서보가 바로 새 ID로 바뀌어서 그 쓰기는 응답을 기다리지 않는다.
        잠금은 새 ID로 다시 걸고, 새 ID는 응답하고 옛 ID는 응답하지 않는지 핑으로 확인한다.
        """
        _check_id(old_id)
        _check_id(new_id)
        if old_id == new_id:
            return
        if self._responds(new_id):
            raise Refused(f"ID {new_id}가 이미 버스에 있다")
        self._require_torque_off(old_id)
        with self._unlocked(old_id, relock_ids=(new_id, old_id)):
            with self._transaction(old_id):
                result = self._packets.writeTxOnly(old_id, reg.ID.address, 1, [new_id])
            if result != COMM_SUCCESS:
                raise PortError(f"ID 쓰기: {self._packets.getTxRxResult(result)}", old_id)
        self.ping(new_id)
        if self._responds(old_id):
            raise VerifyError(f"ID {new_id}로 바꿨는데 옛 ID가 아직 응답한다", old_id)
        self.stats.pop(old_id, None)

    # 안쪽

    @contextmanager
    def _transaction(self, servo_id: int | None) -> Generator[None]:
        """SDK 호출 하나를 잠금 안에서 한다. 시리얼 예외는 PortError로 바꾸고 사용 중 표시를 되돌린다."""
        with self._lock:
            try:
                yield
            except (serial.SerialException, OSError) as error:
                raise PortError(f"포트 오류: {error}", servo_id) from error
            finally:
                self._port.is_using = False

    @contextmanager
    def _unlocked(self, servo_id: int, *, relock_ids: Sequence[int]) -> Generator[None]:
        """EEPROM 잠금을 풀고, 끝나면 다시 건다. 도중에 실패해도 다시 걸어 보고 원래 오류를 올린다."""
        self._write_bytes(servo_id, reg.LOCK.address, [0])
        try:
            yield
        except BaseException:
            for relock_id in relock_ids:
                with suppress(BusError):
                    self._write_bytes(relock_id, reg.LOCK.address, [1])
                    break
            raise
        self._write_bytes(relock_ids[0], reg.LOCK.address, [1])

    def _verify(self, servo_id: int, values: Mapping[Register, int]) -> None:
        readback = self.read(servo_id, *values).values
        wrong = {r.name: (v, readback[r.name]) for r, v in values.items() if readback[r.name] != v}
        if wrong:
            raise VerifyError(f"다시 읽은 값이 다르다 (쓴 값, 읽은 값): {wrong}", servo_id)

    def _require_torque_off(self, servo_id: int) -> None:
        torque = self.read(servo_id, reg.TORQUE_ENABLE).values[reg.TORQUE_ENABLE.name]
        if torque != reg.TORQUE_OFF:
            raise Refused(f"서보 {servo_id}의 토크가 켜져 있어서 EEPROM을 쓰지 않는다")

    def _responds(self, servo_id: int) -> bool:
        try:
            self.ping(servo_id)
        except NoReply:
            self.stats.pop(servo_id, None)
            return False
        return True

    def _read_bytes(self, servo_id: int, address: int, length: int) -> tuple[list[int], Fault]:
        _check_id(servo_id)
        with self._transaction(servo_id):
            data, result, error = self._packets.readTxRx(servo_id, address, length)
        self._check(servo_id, result, f"{address}번부터 {length}바이트 읽기")
        if len(data) != length:
            self._fail(BadReply(f"{length}바이트를 읽었는데 {len(data)}바이트가 왔다", servo_id))
        self._succeed(servo_id)
        return data, Fault(error)

    def _write_bytes(self, servo_id: int, address: int, data: list[int]) -> Fault:
        _check_id(servo_id)
        with self._transaction(servo_id):
            result, error = self._packets.writeTxRx(servo_id, address, len(data), data)
        self._check(servo_id, result, f"{address}번에 쓰기")
        self._succeed(servo_id)
        return Fault(error)

    def _check(self, servo_id: int, result: int, what: str) -> None:
        if result == COMM_SUCCESS:
            return
        message = f"{what}: {self._packets.getTxRxResult(result)}"
        if result == COMM_RX_TIMEOUT:
            self._fail(NoReply(message, servo_id))
        if result == COMM_RX_CORRUPT:
            self._fail(BadReply(message, servo_id))
        raise PortError(message, servo_id)

    def _fail(self, error: NoReply | BadReply) -> NoReturn:
        if error.servo_id is not None:
            self._stats(error.servo_id).record(error)
        raise error

    def _succeed(self, servo_id: int) -> None:
        self._stats(servo_id).record(None)

    def _stats(self, servo_id: int) -> LinkStats:
        return self.stats.setdefault(servo_id, LinkStats())


def _check_id(servo_id: int) -> None:
    if not 0 <= servo_id <= reg.MAX_ID:
        raise ValueError(f"서보 ID는 0~{reg.MAX_ID}다: {servo_id}")


def _check_ram_write(register: Register, value: int) -> None:
    if not register.writable:
        raise ValueError(f"{register.name}은 읽기 전용이다")
    if register.area is not Area.RAM:
        raise ValueError(f"{register.name}은 EEPROM이라 write_eeprom으로 쓴다")
    if register == reg.LOCK:
        raise ValueError("잠금은 write_eeprom과 change_id가 다룬다")
    if register == reg.TORQUE_ENABLE and value not in reg.TORQUE_VALUES:
        raise ValueError(f"토크 스위치에 {value}를 쓰지 않는다 (128은 위치 오프셋을 바꾼다)")
    register.encode(value)


def _span(registers: Sequence[Register]) -> tuple[int, int]:
    """레지스터들을 덮는 첫 주소와 길이."""
    if not registers:
        raise ValueError("읽을 레지스터가 없다")
    first = min(r.address for r in registers)
    end = max(r.address + r.size for r in registers)
    return first, end - first


def _decode(registers: Sequence[Register], data: Sequence[int], first: int) -> dict[str, int]:
    values: dict[str, int] = {}
    for register in registers:
        offset = register.address - first
        raw = data[offset]
        if register.size == 2:
            raw |= data[offset + 1] << 8
        values[register.name] = register.decode(raw)
    return values


def _to_bytes(register: Register, value: int) -> list[int]:
    raw = register.encode(value)
    return [raw & 0xFF] if register.size == 1 else [raw & 0xFF, (raw >> 8) & 0xFF]


def _split_replies(
    raw: Sequence[int], length: int
) -> tuple[dict[int, tuple[Fault, list[int]]], set[int]]:
    """동기 읽기로 받은 바이트를 서보마다의 응답 패킷으로 나눈다.

    패킷은 FF FF ID LEN ERR DATA… CHK이고 LEN은 length + 2다. 길이와 체크섬이 맞는 패킷만 받는다.
    머리(FF FF ID)는 있는데 깨진 패킷의 ID는 따로 돌려준다.
    """
    good: dict[int, tuple[Fault, list[int]]] = {}
    broken: set[int] = set()
    size = length + 6
    i = 0
    while i + 2 < len(raw):
        if raw[i] != 0xFF or raw[i + 1] != 0xFF or raw[i + 2] == 0xFF:
            i += 1
            continue
        servo_id = raw[i + 2]
        packet = list(raw[i : i + size])
        if (
            len(packet) < size
            or packet[3] != length + 2
            or (~sum(packet[2:-1])) & 0xFF != packet[-1]
        ):
            broken.add(servo_id)
            i += 3
            continue
        good[servo_id] = (Fault(packet[4]), packet[5:-1])
        i += size
    return good, broken - good.keys()
