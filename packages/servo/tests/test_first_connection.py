"""첫 연결 명령 검사. 가짜 서보를 포트 자리에 끼워 명령줄부터 기록까지 지나간다.

실물에서 확인할 것(응답 지연 7번의 뜻, 잠금 동작, USB 지연)은 docs/servo-setup.md에 있다.
"""

import csv
import json
import time
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

import pytest
from serial.tools.list_ports_common import ListPortInfo

from horilab_servo import first_connection as fc
from horilab_servo.__main__ import main
from horilab_servo.feetech import registers as reg
from horilab_servo.feetech.bus import Bus, Refused
from horilab_servo.feetech.fake import FakeServo, open_fake_bus

STS3215, STS3250 = 777, 2825


def _servos() -> list[FakeServo]:
    servos = [FakeServo(1, STS3215), FakeServo(2, STS3215), FakeServo(3, STS3250)]
    for servo in servos:
        servo.set(reg.RESPONSE_DELAY, 250)  # 공장값 500 µs (LeRobot의 해석)
        servo.flash = bytes(servo.memory[: len(servo.flash)])
    return servos


def _opener(servos: list[FakeServo]) -> Callable[[str], Bus]:
    """명령이 포트를 열 때마다 같은 가짜 서보들에 새 버스를 잇는다."""

    def open_bus(port: str) -> Bus:
        assert port == "fake"
        bus, _ = open_fake_bus(servos)
        return bus

    return open_bus


def _port(device: str, vid: int | None) -> ListPortInfo:
    port = ListPortInfo(device, skip_link_detection=True)
    port.vid, port.pid = vid, None if vid is None else 0x55D3
    return port


# ID와 포트


@pytest.mark.parametrize(
    ("text", "ids"),
    [("1-7", [1, 2, 3, 4, 5, 6, 7]), ("1,3,5", [1, 3, 5]), ("1-3, 7", [1, 2, 3, 7]), ("0", [0])],
)
def test_parse_ids(text: str, ids: list[int]) -> None:
    assert fc.parse_ids(text) == ids


@pytest.mark.parametrize("text", ["3-1", "1,1", "1-3,2", "253", "a", "", "1-"])
def test_parse_ids_rejects(text: str) -> None:
    with pytest.raises(ValueError):
        fc.parse_ids(text)


def test_adapter_is_the_wch_port() -> None:
    ports = [_port("COM3", None), _port("COM5", fc.WCH_VID), _port("COM7", 0x0403)]
    assert [p.device for p in fc.adapter_ports(ports)] == ["COM5"]


def test_command_picks_the_only_adapter() -> None:
    opened: list[str] = []

    def open_bus(port: str) -> Bus:
        opened.append(port)
        return open_fake_bus(_servos())[0]

    ports = [_port("COM3", None), _port("COM5", fc.WCH_VID)]
    assert main(["scan", "--ids", "1-3"], open_bus=open_bus, ports=lambda: ports) == 0
    assert opened == ["COM5"]


def test_command_stops_without_adapter(capsys: pytest.CaptureFixture[str]) -> None:
    def open_bus(port: str) -> Bus:
        raise AssertionError("포트를 열면 안 된다")

    assert main(["scan"], open_bus=open_bus, ports=lambda: [_port("COM3", None)]) == 1
    assert "보이지 않는다" in capsys.readouterr().out


# 찾기


def test_scan_lists_servos(capsys: pytest.CaptureFixture[str]) -> None:
    servos = _servos()
    assert main(["scan", "--port", "fake", "--ids", "0-5"], open_bus=_opener(servos)) == 0
    out = capsys.readouterr().out
    assert "서보 3개를 찾았다: 1, 2, 3" in out
    assert "STS3250" in out
    assert "12.0 V" in out


def test_scan_without_servos_fails(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["scan", "--port", "fake", "--ids", "4-6"], open_bus=_opener(_servos())) == 1
    assert "서보 0개" in capsys.readouterr().out


# 덤프


def test_decode_matches_servo_memory() -> None:
    servo = FakeServo(4, STS3250)
    servo.set(reg.POSITION_OFFSET, -300)
    values = fc.decode(bytes(servo.memory))
    assert values == {r.name: servo.get(r) for r in reg.ALL}
    assert values["position_offset"] == -300


def test_dump_reads_beyond_table_when_it_can() -> None:
    plain, extended = FakeServo(1), FakeServo(2)
    extended.memory += bytes(fc.EXTRA_END - fc.EXTRA_FIRST)  # 표 밖도 읽히는 서보
    extended.memory[85] = 254
    bus, _ = open_fake_bus([plain, extended])
    assert fc.dump_servo(bus, 1).extra is None
    dump = fc.dump_servo(bus, 2)
    assert dump.memory == bytes(extended.memory[: reg.MEMORY_SIZE])
    assert dump.extra is not None and dump.extra[85 - fc.EXTRA_FIRST] == 254


def test_table_differences_skip_id() -> None:
    servo = FakeServo(5)
    assert fc.table_differences(fc.ServoDump(5, bytes(servo.memory))) == {}
    servo.set(reg.POSITION_OFFSET, -100)
    servo.set(reg.MAX_TORQUE, 500)
    got = fc.table_differences(fc.ServoDump(5, bytes(servo.memory)))
    assert got == {"max_torque": (1000, 500), "position_offset": (0, -100)}


def test_model_disagreements_compare_same_model_only() -> None:
    a, b, c = FakeServo(1, STS3215), FakeServo(2, STS3215), FakeServo(3, STS3250)
    b.set(reg.P_GAIN, 16)
    c.set(reg.D_GAIN, 0)
    for servo in (a, b):
        servo.memory += bytes(fc.EXTRA_END - fc.EXTRA_FIRST)
    a.memory[85] = 254
    dumps = [
        fc.ServoDump(s.id, bytes(s.memory[: reg.MEMORY_SIZE]), bytes(s.memory[fc.EXTRA_FIRST :]))
        for s in (a, b)
    ]
    dumps.append(fc.ServoDump(3, bytes(c.memory)))
    assert fc.model_disagreements(dumps) == {STS3215: ["p_gain", "address_85"]}


def test_dump_command_records_and_compares_with_last_dump(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    servos = _servos()
    args = ["dump", "--port", "fake", "--ids", "1-3", "--records", str(tmp_path)]
    assert main(args, open_bus=_opener(servos)) == 0
    [first] = (tmp_path / "servo").iterdir()
    meta = json.loads((first / "meta.json").read_text(encoding="utf-8"))
    assert meta["kind"] == "servo-dump"
    assert set(meta["servos"]) == {"1", "2", "3"}
    assert meta["servos"]["3"]["model_name"] == "STS3250"
    assert fc.load_dumps(first)[2].memory == bytes(servos[1].memory)
    assert "previous" not in meta

    servos[0].set(reg.RESPONSE_DELAY, 0)  # 응답 지연을 바꾼 채 다시 덤프
    capsys.readouterr()
    assert main(args, open_bus=_opener(servos)) == 0
    second = max((tmp_path / "servo").iterdir())
    meta = json.loads((second / "meta.json").read_text(encoding="utf-8"))
    assert meta["previous"] == {
        "record": f"servo/{first.name}",
        "changes": {"1": {"response_delay": [250, 0]}},
    }
    out = capsys.readouterr().out
    assert "ID 1: response_delay 250 → 0" in out
    assert "ID 2: 같다" in out


def test_previous_dump_skips_timing_and_later_records(tmp_path: Path) -> None:
    folder = tmp_path / "servo"
    for name in (
        "2026-11-14_100000_dump-pc",
        "2026-11-14_110000_timing-pc",
        "2026-11-14_120000_dump-pc",
        "2026-11-14_130000_dump-pc",
    ):
        (folder / name).mkdir(parents=True)
        (folder / name / "meta.json").write_text("{}", encoding="utf-8")
    current = folder / "2026-11-14_120000_dump-pc"
    assert fc.previous_dump(tmp_path, current) == folder / "2026-11-14_100000_dump-pc"


def test_record_dirs_do_not_collide(tmp_path: Path) -> None:
    started = datetime(2026, 11, 14, 15, 30, 12)
    first = fc.new_record_dir(tmp_path, "dump", started)
    second = fc.new_record_dir(tmp_path, "dump", started)
    assert first.name.startswith("2026-11-14_153012_dump-")
    assert second.name == first.name + "-2"
    assert second.name > first.name  # 이름 순이 시간 순이다


# 통신 시간


def test_wire_time_counts_bytes_and_response_delays() -> None:
    seven = [0] * 7
    assert fc.wire_time_s("short_read", seven) == pytest.approx(16e-5)
    assert fc.wire_time_s("state_read", seven) == pytest.approx(29e-5)
    assert fc.wire_time_s("sync_read", seven) == pytest.approx(162e-5)
    assert fc.wire_time_s("tick", seven) == pytest.approx(191e-5)
    # 응답 지연 500 µs는 서보 하나에 한 번, 동기 읽기에는 서보마다 한 번씩 더해진다
    assert fc.wire_time_s("state_read", [250] * 7) == pytest.approx(29e-5 + 500e-6)
    assert fc.wire_time_s("sync_read", [250] * 7) == pytest.approx(162e-5 + 3.5e-3)


def test_summary_uses_successes_and_counts_failures() -> None:
    samples = [fc.Sample("as_is", "tick", None, 0.0, i / 1000, "") for i in range(1, 101)]
    samples.append(fc.Sample("as_is", "tick", None, 0.0, 0.05, "3:NoReply"))
    samples.append(fc.Sample("delay_0", "tick", None, 0.0, 0.5, ""))  # 다른 단계
    s = fc.summarize(samples, phase="as_is", op="tick", period_s=0.01, wire_s=0.002)
    assert (s.count, s.failures, s.over_period) == (101, 1, 90)
    assert (s.min_ms, s.median_ms, s.p99_ms, s.max_ms, s.wire_ms) == (1, 50.5, 99, 100, 2)


def test_timing_measures_as_is_then_with_zero_delay() -> None:
    servos = _servos()
    limits = [s.get(reg.TORQUE_LIMIT) for s in servos]
    bus, _ = open_fake_bus(servos)
    run = fc.measure_timing(bus, [1, 2, 3], count=4, period_s=0)
    assert [(s.phase, s.op) for s in run.summaries] == [
        (phase, op) for phase in ("as_is", "delay_0") for op in fc.OPS
    ]
    assert len(run.samples) == 2 * len(fc.OPS) * 4
    assert all(not s.error for s in run.samples)
    assert run.delays_before == {1: 250, 2: 250, 3: 250}
    assert run.models == {1: STS3215, 2: STS3215, 3: STS3250}
    # 서보를 바꾼 것은 잠근 채 쓴 응답 지연뿐이다: 전원을 껐다 켜면 돌아간다
    assert [s.get(reg.RESPONSE_DELAY) for s in servos] == [0, 0, 0]
    assert [s.get(reg.TORQUE_LIMIT) for s in servos] == limits
    assert [s.eeprom_saves for s in servos] == [0, 0, 0]
    for servo in servos:
        servo.power_cycle()
    assert [s.get(reg.RESPONSE_DELAY) for s in servos] == [250, 250, 250]


def test_timing_measures_once_when_delays_are_zero() -> None:
    servos = _servos()
    for servo in servos:
        servo.set(reg.RESPONSE_DELAY, 0)
    bus, _ = open_fake_bus(servos)
    run = fc.measure_timing(bus, [1, 2, 3], count=2, period_s=0)
    assert {s.phase for s in run.summaries} == {"as_is"}


def test_timing_refuses_when_torque_is_on() -> None:
    servos = _servos()
    servos[1].set(reg.TORQUE_ENABLE, reg.TORQUE_ON)
    bus, _ = open_fake_bus(servos)
    with pytest.raises(Refused):
        fc.measure_timing(bus, [1, 2, 3], count=2, period_s=0)


def test_timing_counts_failures_and_skips_zero_delay_when_trial_fails() -> None:
    servos = _servos()
    calls = 0

    def clock() -> float:
        # 재는 도중에 서보 2가 응답을 멈춘다 (선이 빠진 것처럼)
        nonlocal calls
        calls += 1
        if calls == 20:
            servos[1].faults.silent = True
        return time.perf_counter()

    bus, _ = open_fake_bus(servos)
    run = fc.measure_timing(bus, [1, 2, 3], count=6, period_s=0, clock=clock)
    assert run.trial_error is not None and "서보 2" in run.trial_error
    assert {s.phase for s in run.summaries} == {"as_is"}
    failed = [s for s in run.samples if s.error]
    assert failed and all(s.servo_id in (2, None) for s in failed)
    assert any(s.error == "2:NoReply" for s in failed)
    assert sum(s.failures for s in run.summaries) == len(failed)


def test_timing_command_needs_yes(capsys: pytest.CaptureFixture[str], tmp_path: Path) -> None:
    def open_bus(port: str) -> Bus:
        raise AssertionError("--yes 없이 포트를 열면 안 된다")

    args = ["timing", "--port", "fake", "--ids", "1-3", "--records", str(tmp_path)]
    assert main(args, open_bus=open_bus) == 0
    assert "--yes" in capsys.readouterr().out
    assert not any(tmp_path.iterdir())


def test_timing_command_records_samples_and_summaries(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    args = ["timing", "--port", "fake", "--ids", "1-3", "--records", str(tmp_path)]
    args += ["--count", "3", "--period-ms", "0", "--yes"]
    assert main(args, open_bus=_opener(_servos())) == 0
    [folder] = (tmp_path / "servo").iterdir()
    assert "_timing-" in folder.name
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    assert meta["kind"] == "servo-timing"
    assert meta["tick_write"] == "torque_limit"
    assert len(meta["summaries"]) == 2 * len(fc.OPS)
    assert set(meta["link_stats"]) == {"1", "2", "3"}
    with (folder / "samples.csv").open(encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 2 * len(fc.OPS) * 3
    assert {r["op"] for r in rows} == set(fc.OPS)
    out = capsys.readouterr().out
    assert "한 박자" in out
    assert "7번이 처음 값으로 돌아왔는지" in out
