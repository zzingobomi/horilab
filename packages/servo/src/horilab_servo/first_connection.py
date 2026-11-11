"""첫 연결: 어댑터와 서보를 찾고, 설정을 덤프하고, 통신 시간을 재고, ID를 바꾼다 (docs/servo-setup.md).

명령줄은 __main__.py에 있다 (uv run python -m horilab_servo …). 여기에는 그 일과 기록이 있다.

- 서보를 움직이지 않는다. 목표 위치도 쓰지 않는다: 각도 한계 밖에 놓인 서보에 목표를 쓰면
  한계까지 움직일 수 있다.
- 통신 시간을 잴 때는 동기 쓰기로 토크 제한(RAM)에 읽은 값을 그대로 쓰고, 응답 지연(7번)은 잠근 채
  0으로 바꿔 한 번 더 잰다. 서보 전원을 껐다 켜면 처음 값으로 돌아간다.
- 전원을 꺼도 남는 것은 ID 바꾸기뿐이다. 버스에 서보가 하나만 있을 때 한다 (0024의 EEPROM 절차).
- 기록은 records/servo/<날짜시각>_<명령>-<컴퓨터>/에 남긴다 (docs/decisions/0022, 0024).
"""

import csv
import json
import math
import platform
import re
import statistics
import subprocess
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from serial.tools import list_ports
from serial.tools.list_ports_common import ListPortInfo

from horilab_servo.feetech import registers as reg
from horilab_servo.feetech.bus import Bus, BusError, Refused, Reply
from horilab_servo.feetech.registers import Area, Register

# 어댑터의 USB 칩(CH343P)을 만든 WCH의 USB 제조사 번호
WCH_VID = 0x1A86

RECORD_KIND = "servo"  # records/<종류>/

# 메모리 표 뒤(71~127번)도 읽히면 덤프에 같이 남긴다. 표에 없는 칸을 다른 도구가 바꿨을 수 있는데
# 값 적용은 표 안의 칸만 맞춘다. 새 서보의 덤프와 견주면 바뀐 칸이 보인다.
EXTRA_FIRST = reg.MEMORY_SIZE
EXTRA_END = 128

# 같은 모델끼리 견주는 설정 칸: 쓸 수 있는 EEPROM (ID는 서보마다 다르게 두므로 뺀다)
SETTINGS: tuple[Register, ...] = tuple(
    r for r in reg.ALL if r.area is Area.EEPROM and r.writable and r is not reg.ID
)
EEPROM: tuple[Register, ...] = tuple(r for r in reg.ALL if r.area is Area.EEPROM)

# 통신 시간. 동작마다 실행 루프처럼 주기에 맞춰 되풀이한다.
OPS = ("short_read", "state_read", "sync_read", "tick")
OP_NAMES = {
    "short_read": "짧은 읽기",  # 모델 번호 2바이트. 핑 대신이다: SDK의 핑은 모델을 한 번 더 읽는다
    "state_read": "상태 읽기",  # 상태 묶음(56~70) 서보 하나
    "sync_read": "동기 읽기",  # 상태 묶음을 서보 전체에서
    "tick": "한 박자",  # 동기 쓰기 + 동기 읽기. 실행 루프가 주기마다 하는 일
}
SINGLE_OPS = ("short_read", "state_read")
PHASE_AS_IS = "as_is"  # 서보에 들어 있는 응답 지연 그대로
PHASE_ZERO = "delay_0"  # 응답 지연 0 (잠근 채 썼다)
PHASE_NAMES = {PHASE_AS_IS: "응답 지연 그대로", PHASE_ZERO: "응답 지연 0"}

# 7번 한 칸의 시간. LeRobot과 rustypot의 해석이고, 두 단계의 차이로 실물에서 확인한다.
RESPONSE_DELAY_STEP_S = 2e-6
# 한 박자의 동기 쓰기. 실행 루프는 목표를 쓰지만 여기서는 움직이지 않는 칸에 읽은 값을 그대로 쓴다.
TICK_WRITE = reg.TORQUE_LIMIT
STATE_FIRST = min(r.address for r in reg.STATE)
STATE_SIZE = max(r.address + r.size for r in reg.STATE) - STATE_FIRST


# 포트와 ID


def serial_ports() -> list[ListPortInfo]:
    """이 컴퓨터의 시리얼 포트."""
    return sorted(list_ports.comports(), key=lambda port: port.device)


def adapter_ports(ports: Iterable[ListPortInfo]) -> list[ListPortInfo]:
    """서보 어댑터로 보이는 포트 (USB 칩이 WCH 것)."""
    return [port for port in ports if port.vid == WCH_VID]


def parse_ids(text: str) -> list[int]:
    """'1-7', '1,3,5', '1-3,7' 같은 ID 목록을 읽는다."""
    ids: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        first, dash, last = part.partition("-")
        try:
            start = int(first)
            end = int(last) if dash else start
        except ValueError:
            raise ValueError(f"ID 목록을 읽지 못했다: {text}") from None
        if start > end:
            raise ValueError(f"ID 범위가 거꾸로다: {part}")
        ids.extend(range(start, end + 1))
    if not ids:
        raise ValueError("ID가 없다")
    outside = [i for i in ids if not 0 <= i <= reg.MAX_ID]
    if outside:
        raise ValueError(f"서보 ID는 0~{reg.MAX_ID}다: {outside}")
    if len(set(ids)) != len(ids):
        raise ValueError(f"ID가 겹친다: {text}")
    return ids


def model_name(model: int) -> str:
    return reg.MODELS.get(model, str(model))


# 덤프


@dataclass(frozen=True)
class ServoDump:
    """서보 하나의 메모리. memory는 주소 0부터 메모리 표 끝까지, extra는 그 뒤(읽혔을 때만)."""

    servo_id: int
    memory: bytes
    extra: bytes | None = None

    @property
    def values(self) -> dict[str, int]:
        return decode(self.memory)

    @property
    def model(self) -> int:
        return self.values[reg.MODEL.name]


def decode(memory: bytes) -> dict[str, int]:
    """덤프 바이트를 레지스터 이름 → 값으로 푼다 (부호도 푼다)."""
    if len(memory) < reg.MEMORY_SIZE:
        raise ValueError(f"덤프가 {len(memory)}바이트다 ({reg.MEMORY_SIZE}바이트가 필요하다)")
    values: dict[str, int] = {}
    for register in reg.ALL:
        raw = memory[register.address]
        if register.size == 2:
            raw |= memory[register.address + 1] << 8
        values[register.name] = register.decode(raw)
    return values


def dump_servo(bus: Bus, servo_id: int) -> ServoDump:
    """메모리 표를 읽고, 표 밖도 읽어 본다. 표 밖이 읽히지 않으면 extra는 None이다."""
    memory = bus.dump(servo_id)
    try:
        extra = bus.read_raw(servo_id, EXTRA_FIRST, EXTRA_END - EXTRA_FIRST)
    except BusError:
        extra = None
    return ServoDump(servo_id, memory, extra)


def table_differences(dump: ServoDump) -> dict[str, tuple[int, int]]:
    """메모리 표의 초기값과 다른 칸: 이름 → (표의 값, 읽은 값). ID는 뺀다."""
    values = dump.values
    return {
        r.name: (initial, values[r.name])
        for r, initial in reg.TABLE_INITIAL.items()
        if r is not reg.ID and values[r.name] != initial
    }


def model_disagreements(dumps: Iterable[ServoDump]) -> dict[int, list[str]]:
    """같은 모델끼리 값이 다른 설정 칸: 모델 번호 → 칸 이름. 표 밖은 address_<주소>다."""
    groups: dict[int, list[ServoDump]] = {}
    for dump in dumps:
        groups.setdefault(dump.model, []).append(dump)
    result: dict[int, list[str]] = {}
    for model, group in groups.items():
        if len(group) < 2:
            continue
        values = [d.values for d in group]
        names = [r.name for r in SETTINGS if len({v[r.name] for v in values}) > 1]
        extras = [d.extra for d in group if d.extra is not None]
        if len(extras) == len(group):
            size = min(len(e) for e in extras)
            names += [
                f"address_{EXTRA_FIRST + i}" for i in range(size) if len({e[i] for e in extras}) > 1
            ]
        if names:
            result[model] = names
    return result


def changes(before: ServoDump, after: ServoDump) -> dict[str, tuple[int, int]]:
    """두 덤프에서 달라진 EEPROM 칸과 표 밖 바이트: 이름 → (앞, 뒤)."""
    old, new = before.values, after.values
    result = {r.name: (old[r.name], new[r.name]) for r in EEPROM if old[r.name] != new[r.name]}
    if before.extra is not None and after.extra is not None:
        for i, (a, b) in enumerate(zip(before.extra, after.extra, strict=False)):
            if a != b:
                result[f"address_{EXTRA_FIRST + i}"] = (a, b)
    return result


def dump_meta(dumps: Sequence[ServoDump]) -> dict[str, Any]:
    """덤프 기록의 서보 부분. load_dumps가 다시 읽는다."""
    return {
        str(d.servo_id): {
            "model": d.model,
            "model_name": model_name(d.model),
            "memory": d.memory.hex(" "),
            "extra_first": EXTRA_FIRST,
            "extra": None if d.extra is None else d.extra.hex(" "),
            "values": d.values,
            "table_differences": table_differences(d),
        }
        for d in dumps
    }


def load_dumps(record: Path) -> dict[int, ServoDump]:
    """덤프 기록에서 서보마다의 덤프를 읽는다."""
    meta = json.loads((record / "meta.json").read_text(encoding="utf-8"))
    servos: dict[str, dict[str, Any]] = meta["servos"]
    dumps: dict[int, ServoDump] = {}
    for key, servo in servos.items():
        extra: str | None = servo.get("extra")
        memory = bytes.fromhex(servo["memory"])
        dumps[int(key)] = ServoDump(
            int(key), memory, None if extra is None else bytes.fromhex(extra)
        )
    return dumps


def id_changes_between(root: Path, earlier: Path, later: Path) -> list[Path]:
    """두 기록 사이에 남은 ID 바꾸기 기록. ID를 바꾸면 같은 ID라도 다른 서보일 수 있다."""
    folder = root / RECORD_KIND
    if not folder.is_dir():
        return []
    return sorted(
        p
        for p in folder.glob("*_set-id-*")
        if earlier.name < p.name < later.name and (p / "meta.json").is_file()
    )


def previous_dump(root: Path, current: Path) -> Path | None:
    """records/servo/에서 current 앞의 마지막 덤프 기록. 폴더 이름이 시각으로 시작해 이름 순이 시간 순이다."""
    folder = root / RECORD_KIND
    if not folder.is_dir():
        return None
    found = sorted(
        p for p in folder.glob("*_dump-*") if p.name < current.name and (p / "meta.json").is_file()
    )
    return found[-1] if found else None


# 통신 시간


@dataclass(frozen=True)
class Sample:
    phase: str
    op: str
    servo_id: int | None  # 짧은 읽기와 상태 읽기는 그 서보, 동기는 None (전체)
    t_s: float  # 잰 것 전체의 시작부터
    duration_s: float
    error: str  # 비었으면 성공. 동기는 실패한 서보마다 "ID:오류"


@dataclass(frozen=True)
class OpSummary:
    phase: str
    op: str
    count: int
    failures: int
    over_period: int  # 성공한 것 중 주기를 넘긴 수
    min_ms: float | None
    median_ms: float | None
    p99_ms: float | None
    max_ms: float | None
    wire_ms: float  # 선로 위 시간만의 계산 (wire_time_s)


@dataclass
class TimingRun:
    ids: list[int]
    models: dict[int, int]
    delays_before: dict[int, int]  # 서보마다 처음 응답 지연 (7번 값)
    count: int
    period_s: float
    samples: list[Sample] = field(default_factory=list[Sample])
    summaries: list[OpSummary] = field(default_factory=list[OpSummary])
    trial_error: str | None = None  # 응답 지연을 0으로 바꾸지 못했으면 그 까닭


def wire_time_s(op: str, delays: Sequence[int], *, baud: int = 1_000_000) -> float:
    """선로 위 시간: 주고받는 바이트(한 바이트 10비트)와 서보의 응답 지연만 더한 값.

    짧은 읽기와 상태 읽기는 서보 하나(응답 지연은 평균), 동기 읽기와 한 박자는 서보 전체다.
    USB와 운영체제가 더하는 시간은 들어 있지 않아서, 잰 값과의 차이가 그 몫이다.
    """
    if not delays:
        raise ValueError("서보가 없다")
    byte_s = 10 / baud
    n = len(delays)
    one_delay_s = statistics.fmean(delays) * RESPONSE_DELAY_STEP_S
    all_delays_s = sum(delays) * RESPONSE_DELAY_STEP_S
    # 읽기 요청 FF FF ID LEN INST 주소 길이 CHK, 응답 FF FF ID LEN ERR 값… CHK
    sync_read = (8 + n) + n * (6 + STATE_SIZE)
    match op:
        case "short_read":
            return (8 + 6 + reg.MODEL.size) * byte_s + one_delay_s
        case "state_read":
            return (8 + 6 + STATE_SIZE) * byte_s + one_delay_s
        case "sync_read":
            return sync_read * byte_s + all_delays_s
        case "tick":
            sync_write = 8 + n * (1 + TICK_WRITE.size)
            return (sync_write + sync_read) * byte_s + all_delays_s
        case _:
            raise ValueError(f"모르는 동작이다: {op}")


def summarize(
    samples: Iterable[Sample], *, phase: str, op: str, period_s: float, wire_s: float
) -> OpSummary:
    mine = [s for s in samples if s.phase == phase and s.op == op]
    good = sorted(s.duration_s for s in mine if not s.error)

    def ms(value: float | None) -> float | None:
        return None if value is None else round(value * 1000, 3)

    return OpSummary(
        phase=phase,
        op=op,
        count=len(mine),
        failures=len(mine) - len(good),
        over_period=sum(1 for d in good if d > period_s),
        min_ms=ms(good[0] if good else None),
        median_ms=ms(statistics.median(good) if good else None),
        p99_ms=ms(_percentile(good, 0.99) if good else None),
        max_ms=ms(good[-1] if good else None),
        wire_ms=round(wire_s * 1000, 3),
    )


def _percentile(ordered: Sequence[float], q: float) -> float:
    """가까운 순위 방식: 값의 q 비율이 이 값 이하다."""
    return ordered[max(0, math.ceil(q * len(ordered)) - 1)]


def measure_timing(
    bus: Bus,
    ids: Sequence[int],
    *,
    count: int = 300,
    period_s: float = 0.01,
    clock: Callable[[], float] = time.perf_counter,
    sleep: Callable[[float], None] = time.sleep,
) -> TimingRun:
    """통신 시간을 잰다. 토크가 켜진 서보가 있으면 재지 않는다 (Refused).

    응답 지연을 그대로 둔 채 재고, 0이 아닌 서보가 있으면 잠근 채 0으로 바꿔 한 번 더 잰다.
    바꾼 응답 지연은 되돌리지 않는다. 서보 전원을 껐다 켜서 돌아오는지가 잠금 동작의 확인이다.
    """
    if not ids:
        raise ValueError("잴 서보가 없다")
    if count < 1:
        raise ValueError("횟수는 1 이상이다")
    models: dict[int, int] = {}
    limits: dict[int, int] = {}
    delays: dict[int, int] = {}
    for servo_id in ids:
        models[servo_id] = bus.read(servo_id, reg.MODEL).values[reg.MODEL.name]
        state = bus.read(servo_id, reg.TORQUE_ENABLE, reg.TORQUE_LIMIT).values
        if state[reg.TORQUE_ENABLE.name] != reg.TORQUE_OFF:
            raise Refused(f"서보 {servo_id}의 토크가 켜져 있어서 재지 않는다")
        limits[servo_id] = state[reg.TORQUE_LIMIT.name]
        delays[servo_id] = bus.read(servo_id, reg.RESPONSE_DELAY).values[reg.RESPONSE_DELAY.name]
    run = TimingRun(list(ids), models, delays, count, period_s)
    start = clock()

    def phase(name: str, phase_delays: Sequence[int]) -> None:
        for op in OPS:
            _repeat(bus, run, name, op, limits, start=start, clock=clock, sleep=sleep)
            wire_s = wire_time_s(op, phase_delays)
            run.summaries.append(
                summarize(run.samples, phase=name, op=op, period_s=period_s, wire_s=wire_s)
            )

    phase(PHASE_AS_IS, [delays[i] for i in ids])
    if any(delays.values()):
        try:
            for servo_id in ids:
                bus.write_trial(servo_id, {reg.RESPONSE_DELAY: 0})
        except BusError as error:
            run.trial_error = str(error)
            return run
        phase(PHASE_ZERO, [0] * len(ids))
    return run


def _repeat(
    bus: Bus,
    run: TimingRun,
    phase: str,
    op: str,
    limits: Mapping[int, int],
    *,
    start: float,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    """동작 하나를 주기마다 되풀이한다. 늦어지면 기다리지 않고 바로 다음을 한다."""
    due = clock()
    for i in range(run.count):
        wait = due - clock()
        if wait > 0:
            sleep(wait)
        servo_id = run.ids[i % len(run.ids)]
        begin = clock()
        error = _attempt(bus, op, servo_id, run.ids, limits)
        end = clock()
        single = servo_id if op in SINGLE_OPS else None
        run.samples.append(Sample(phase, op, single, begin - start, end - begin, error))
        due = max(due + run.period_s, end)


def _attempt(
    bus: Bus, op: str, servo_id: int, ids: Sequence[int], limits: Mapping[int, int]
) -> str:
    try:
        match op:
            case "short_read":
                bus.read(servo_id, reg.MODEL)
            case "state_read":
                bus.read(servo_id, *reg.STATE)
            case "sync_read":
                return _sync_errors(bus.sync_read(ids, *reg.STATE))
            case _:
                bus.sync_write(TICK_WRITE, limits)
                return _sync_errors(bus.sync_read(ids, *reg.STATE))
    except BusError as error:
        return type(error).__name__
    return ""


def _sync_errors(results: Mapping[int, Reply | BusError]) -> str:
    return " ".join(
        f"{servo_id}:{type(r).__name__}"
        for servo_id, r in results.items()
        if isinstance(r, BusError)
    )


def timing_meta(run: TimingRun) -> dict[str, Any]:
    """통신 시간 기록의 결과 부분."""
    return {
        "ids": run.ids,
        "models": {str(i): m for i, m in run.models.items()},
        "count": run.count,
        "period_s": run.period_s,
        "response_delay_before": {str(i): d for i, d in run.delays_before.items()},
        "response_delay_step_s": RESPONSE_DELAY_STEP_S,
        "tick_write": TICK_WRITE.name,
        "trial_error": run.trial_error,
        "summaries": [asdict(s) for s in run.summaries],
    }


def write_samples(path: Path, samples: Iterable[Sample]) -> None:
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["phase", "op", "servo_id", "t_s", "duration_ms", "error"])
        for s in samples:
            servo = "" if s.servo_id is None else s.servo_id
            writer.writerow(
                [s.phase, s.op, servo, f"{s.t_s:.6f}", f"{s.duration_s * 1000:.4f}", s.error]
            )


# 기록


def records_root() -> Path:
    """레포의 records/ (0022의 표준 경로). 레포 밖에 깔렸으면 지금 폴더 아래 records/다."""
    for parent in Path(__file__).resolve().parents:
        if (parent / ".git").exists():
            return parent / "records"
    return Path.cwd() / "records"


def new_record_dir(root: Path, command: str, started: datetime) -> Path:
    """records/servo/<날짜시각>_<명령>-<컴퓨터>/를 만든다."""
    host = re.sub(r"[^A-Za-z0-9_.-]", "_", platform.node()) or "host"
    base = root / RECORD_KIND / f"{started:%Y-%m-%d_%H%M%S}_{command}-{host}"
    path, n = base, 1
    while path.exists():
        n += 1
        path = base.with_name(f"{base.name}-{n}")
    path.mkdir(parents=True)
    return path


def run_info(port: str) -> dict[str, Any]:
    """어디서 어떤 코드로 돌렸나. 기록만 보고 다른 PC에서 분석할 수 있게 남긴다 (0022)."""
    status = _git("status", "--porcelain")
    return {
        "host": platform.node(),
        "system": platform.platform(),
        "python": platform.python_version(),
        "commit": _git("rev-parse", "HEAD"),
        "uncommitted_changes": None if status is None else bool(status),
        "port": port,
        "baud": reg.BAUD_RATES[0],
    }


def _git(*args: str) -> str | None:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=Path(__file__).resolve().parent,
            capture_output=True,
            text=True,
            encoding="utf-8",
            check=False,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
