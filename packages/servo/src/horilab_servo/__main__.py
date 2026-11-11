"""첫 연결 명령 (docs/servo-setup.md). 서보를 움직이지 않는다.

    uv run python -m horilab_servo ports                       시리얼 포트와 서보 어댑터
    uv run python -m horilab_servo scan [--ids 1-7]            서보 찾기
    uv run python -m horilab_servo dump [--ids 1-7]            설정 덤프를 뜨고 기록한다
    uv run python -m horilab_servo timing [--ids 1-7] [--yes]  통신 시간을 재고 기록한다

포트는 서보 어댑터(USB 칩이 WCH 것)가 하나만 보이면 저절로 고른다. 아니면 --port로 준다.
--ids를 주지 않으면 ID 0~252를 모두 찾는다 (없는 ID마다 응답을 기다려 15초쯤 걸린다).
"""

import argparse
import sys
import unicodedata
from collections.abc import Callable, Sequence
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

from serial.tools.list_ports_common import ListPortInfo

from horilab_servo import first_connection as fc
from horilab_servo.feetech import registers as reg
from horilab_servo.feetech.bus import Bus, BusError, Refused

ALL_IDS = range(reg.MAX_ID + 1)


def main(
    argv: Sequence[str] | None = None,
    *,
    open_bus: Callable[[str], Bus] = Bus.open,
    ports: Callable[[], list[ListPortInfo]] = fc.serial_ports,
) -> int:
    args = _parser().parse_args(argv)
    # 진행 안내가 바로 보이게 하고, 콘솔이 못 쓰는 글자(µ 등) 때문에 멈추지 않게 한다.
    reconfigure: Callable[..., None] | None = getattr(sys.stdout, "reconfigure", None)
    if reconfigure is not None:
        reconfigure(line_buffering=True, errors="replace")
    if args.command == "ports":
        return _ports(ports())
    if args.command == "timing" and not args.yes:
        _print_timing_plan(args)
        return 0
    port: str | None = args.port or _find_adapter(ports())
    if port is None:
        return 1
    ids: list[int] | None = args.ids
    try:
        with open_bus(port) as bus:
            if args.command == "scan" or ids is None:
                if ids is None:
                    print(f"{port}에서 서보를 찾는다 (ID 0~{reg.MAX_ID}, 15초쯤 걸린다)")
                ids = _scan(bus, ALL_IDS if ids is None else ids)
                if args.command == "scan" or not ids:
                    return 0 if ids else 1
                print()
            if args.command == "dump":
                return _dump(bus, port, ids, args.records)
            return _timing(bus, port, ids, args)
    except (BusError, Refused) as error:
        print(f"멈췄다: {error}")
        return 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uv run python -m horilab_servo",
        description="첫 연결: 서보 찾기, 덤프, 통신 시간 (docs/servo-setup.md). 서보를 움직이지 않는다.",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("ports", help="시리얼 포트와 서보 어댑터를 보여 준다")
    scan = commands.add_parser("scan", help="서보를 찾는다 (읽기만 한다)")
    dump = commands.add_parser("dump", help="설정 덤프를 뜨고 기록한다 (읽기만 한다)")
    timing = commands.add_parser(
        "timing",
        help="통신 시간을 재고 기록한다. 토크 제한에 읽은 값을 그대로 쓰고 응답 지연을 잠근 채 바꾼다",
    )
    for command in (scan, dump, timing):
        command.add_argument("--port", help="시리얼 포트 (예: COM5, /dev/ttyACM0)")
        command.add_argument(
            "--ids", type=_ids, metavar="ID", help="이 ID만 본다 (예: 1-7, 1,3,5). 없으면 다 찾는다"
        )
    for command in (dump, timing):
        command.add_argument(
            "--records",
            type=Path,
            default=fc.records_root(),
            metavar="폴더",
            help="기록을 둘 records 폴더 (기본: 레포의 records/)",
        )
    timing.add_argument("--count", type=_positive, default=300, help="동작마다 되풀이할 횟수")
    timing.add_argument("--period-ms", type=float, default=10.0, help="되풀이 주기 [ms]")
    timing.add_argument("--yes", action="store_true", help="실제로 잰다")
    return parser


def _ids(text: str) -> list[int]:
    try:
        return fc.parse_ids(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from None


def _positive(text: str) -> int:
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError("1 이상이다")
    return value


# 화면에 줄 맞춰 쓰기. 한글은 두 칸을 차지한다.


def _width(text: str) -> int:
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)


def _left(text: str, width: int) -> str:
    return text + " " * max(0, width - _width(text))


def _right(text: object, width: int) -> str:
    text = str(text)
    return " " * max(0, width - _width(text)) + text


# 포트


def _ports(found: Sequence[ListPortInfo]) -> int:
    if not found:
        print("시리얼 포트가 없다. 어댑터의 USB가 꽂혀 있는지 본다.")
        return 1
    adapters = fc.adapter_ports(found)
    for port in found:
        usb = "" if port.vid is None else f" [{port.vid:04X}:{port.pid or 0:04X}]"
        mark = "  ← 서보 어댑터" if port in adapters else ""
        print(f"{port.device}  {port.description}{usb}{mark}")
    return 0


def _find_adapter(found: Sequence[ListPortInfo]) -> str | None:
    adapters = fc.adapter_ports(found)
    if len(adapters) == 1:
        return adapters[0].device
    if adapters:
        names = ", ".join(p.device for p in adapters)
        print(f"서보 어댑터로 보이는 포트가 여럿이다 ({names}). --port로 고른다.")
    else:
        print("서보 어댑터(USB 칩이 WCH 것)가 보이지 않는다. 포트 목록:")
        _ports(found)
        print("어댑터의 USB가 꽂혀 있는지 보고, 다른 이름으로 보이면 --port로 준다.")
    return None


# 찾기


def _scan(bus: Bus, ids: Sequence[int]) -> list[int]:
    result = bus.scan(ids)
    found = sorted(result.models)
    if found:
        print(
            f"{_right('ID', 3)}  {_left('모델', 8)} {_left('펌웨어', 6)} {_right('전압', 7)} "
            f"{_right('온도', 5)}  토크"
        )
        for servo_id in found:
            try:
                values = fc.decode(bus.dump(servo_id))
            except BusError as error:
                print(f"{servo_id:>3}  읽지 못했다: {error}")
                continue
            firmware = f"{values['firmware_major']}.{values['firmware_minor']}"
            torque = {0: "꺼짐", 1: "켜짐", 2: "댐핑"}.get(values["torque_enable"], "?")
            voltage = f"{values['present_voltage'] / 10:.1f} V"
            print(
                f"{servo_id:>3}  {_left(fc.model_name(values['model']), 8)} {firmware:<6} "
                f"{voltage:>7} {values['present_temperature']:>3}°C  {torque}"
            )
    print(f"서보 {len(found)}개를 찾았다" + (f": {', '.join(map(str, found))}" if found else ""))
    if result.garbled:
        print(f"응답이 깨진 ID: {result.garbled}. 같은 ID의 서보가 둘일 수 있다.")
    if not found:
        print("서보 전원(가변 전원 화면의 12.0 V), 서보 선, 어댑터 점퍼(B)를 본다.")
    else:
        print("꽂은 서보 수와 다르면 선이 빠졌거나 같은 ID의 서보가 있는 것이다.")
    return found


# 덤프


def _dump(bus: Bus, port: str, ids: Sequence[int], records: Path) -> int:
    started = datetime.now().astimezone()
    dumps = [fc.dump_servo(bus, servo_id) for servo_id in ids]
    folder = fc.new_record_dir(records, "dump", started)
    meta: dict[str, object] = {
        "kind": "servo-dump",
        "started": started.isoformat(timespec="seconds"),
        "run": fc.run_info(port),
        "servos": fc.dump_meta(dumps),
        "model_disagreements": {
            str(m): names for m, names in fc.model_disagreements(dumps).items()
        },
    }
    previous = fc.previous_dump(records, folder)
    changed: dict[int, dict[str, tuple[int, int]]] = {}
    if previous is not None:
        before = fc.load_dumps(previous)
        changed = {
            d.servo_id: fc.changes(before[d.servo_id], d) for d in dumps if d.servo_id in before
        }
        meta["previous"] = {
            "record": previous.relative_to(records).as_posix(),
            "changes": {str(i): c for i, c in changed.items() if c},
        }
    fc.write_json(folder / "meta.json", meta)
    _print_dump_table(dumps)
    if previous is not None:
        _print_changes(previous.name, changed)
    print(f"기록: {folder}")
    return 0


def _print_dump_table(dumps: Sequence[fc.ServoDump]) -> None:
    disagree = {name for names in fc.model_disagreements(dumps).values() for name in names}
    rows = [(d.values, fc.table_differences(d)) for d in dumps]
    head = f"  {_left('칸', 28)}{_right('주소', 4)} {_right('표', 6)}"
    print(head + "".join(_right(f"ID {d.servo_id}", 9) for d in dumps))
    print(" " * _width(head) + "".join(_right(fc.model_name(d.model), 9) for d in dumps))
    for register in reg.ALL:
        initial = reg.TABLE_INITIAL.get(register)
        flag = "!" if register.name in disagree else " "
        cells = "".join(
            f"{values[register.name]:>8}{'*' if register.name in diffs else ' '}"
            for values, diffs in rows
        )
        table = "" if initial is None else str(initial)
        print(f"{flag} {register.name:<28}{register.address:>4} {table:>6}{cells}")
    extras = [d.extra for d in dumps]
    for i in range(fc.EXTRA_END - fc.EXTRA_FIRST):
        column = [None if e is None or i >= len(e) else e[i] for e in extras]
        if not any(column):
            continue  # 모두 0이거나 읽히지 않은 칸은 줄인다
        address = fc.EXTRA_FIRST + i
        flag = "!" if f"address_{address}" in disagree else " "
        cells = "".join(f"{'-' if v is None else v:>8} " for v in column)
        print(f"{flag} {_left('(표 밖)', 28)}{address:>4} {'':>6}{cells}")
    print()
    print("* 메모리 표의 초기값과 다르다.  ! 같은 모델끼리 다르다.")
    unread = [d.servo_id for d in dumps if d.extra is None]
    print(
        f"(표 밖)은 {fc.EXTRA_FIRST}~{fc.EXTRA_END - 1}번에서 0이 아닌 칸만 보인다."
        + (f" 표 밖이 읽히지 않은 서보: {unread}" if unread else "")
    )


def _print_changes(previous: str, changed: dict[int, dict[str, tuple[int, int]]]) -> None:
    print(f"지난 덤프({previous})와 견줌:")
    if not changed:
        print("  같은 ID의 서보가 없다.")
    for servo_id, diffs in changed.items():
        text = ", ".join(f"{name} {a} → {b}" for name, (a, b) in diffs.items()) or "같다"
        print(f"  ID {servo_id}: {text}")


# 통신 시간


def _print_timing_plan(args: argparse.Namespace) -> None:
    ids: list[int] | None = args.ids
    target = "찾은 서보 전체" if ids is None else f"ID {', '.join(map(str, ids))}"
    print(f"통신 시간을 잰다 ({target}). 서보를 움직이지 않는다.")
    print("- 토크가 켜진 서보가 있으면 재지 않는다.")
    print(
        f"- 짧은 읽기, 상태 읽기, 동기 읽기, 한 박자(동기 쓰기 + 동기 읽기)를 "
        f"{args.period_ms:g} ms마다 {args.count}번씩 잰다."
    )
    print("- 동기 쓰기는 토크 제한에 읽은 값을 그대로 쓴다.")
    print(
        "- 응답 지연(7번)이 0이 아닌 서보가 있으면 잠근 채 0으로 바꿔 한 번 더 잰다. "
        "서보 전원을 껐다 켜면 돌아간다."
    )
    print("재려면 --yes를 붙인다.")


def _timing(bus: Bus, port: str, ids: Sequence[int], args: argparse.Namespace) -> int:
    started = datetime.now().astimezone()
    seconds = 2 * len(fc.OPS) * args.count * args.period_ms / 1000
    print(f"통신 시간을 잰다: 서보 {len(ids)}개, 동작마다 {args.count}번 ({seconds:.0f}초쯤)")
    run = fc.measure_timing(bus, ids, count=args.count, period_s=args.period_ms / 1000)
    folder = fc.new_record_dir(args.records, "timing", started)
    fc.write_samples(folder / "samples.csv", run.samples)
    meta: dict[str, object] = {
        "kind": "servo-timing",
        "started": started.isoformat(timespec="seconds"),
        "ended": datetime.now().astimezone().isoformat(timespec="seconds"),
        "run": fc.run_info(port),
        **fc.timing_meta(run),
        "link_stats": {str(i): asdict(s) for i, s in bus.stats.items()},
    }
    fc.write_json(folder / "meta.json", meta)
    _print_timing(run)
    print(f"기록: {folder}")
    return 0


def _print_timing(run: fc.TimingRun) -> None:
    period_ms = run.period_s * 1000
    for phase in (fc.PHASE_AS_IS, fc.PHASE_ZERO):
        summaries = [s for s in run.summaries if s.phase == phase]
        if not summaries:
            continue
        print()
        if phase == fc.PHASE_ZERO:
            print(f"{fc.PHASE_NAMES[phase]}: 잠근 채 0으로 바꿨다")
        else:
            delays = sorted(set(run.delays_before.values()))
            if len(delays) == 1:
                us = delays[0] * fc.RESPONSE_DELAY_STEP_S * 1e6
                print(f"{fc.PHASE_NAMES[phase]}: 서보마다 7번 = {delays[0]} ({us:g} µs)")
            else:
                text = ", ".join(f"ID {i} = {d}" for i, d in run.delays_before.items())
                print(f"{fc.PHASE_NAMES[phase]}: 7번 {text}")
        names = ("횟수", "실패", "최소", "중간", "p99", "최대", "선로")
        widths = (5, 5, 8, 8, 8, 8, 8)
        print(
            "  "
            + _left("[ms]", 10)
            + "".join(_right(n, w) for n, w in zip(names, widths, strict=True))
            + f"  {period_ms:g} ms 넘음"
        )
        for s in summaries:
            numbers = "".join(
                _right("-" if v is None else f"{v:.2f}", 8)
                for v in (s.min_ms, s.median_ms, s.p99_ms, s.max_ms, s.wire_ms)
            )
            print(
                f"  {_left(fc.OP_NAMES[s.op], 10)}{s.count:>5}{s.failures:>5}{numbers}"
                f"  {s.over_period:>10}"
            )
    print()
    print("선로는 바이트와 응답 지연만 더한 계산이다. 잰 값과의 차이가 USB와 운영체제의 몫이다.")
    if run.trial_error is not None:
        print(f"응답 지연을 0으로 바꾸지 못해 한 번만 쟀다: {run.trial_error}")
        print("서보 전원(+ 플러그)을 뺐다 꽂아 처음 상태로 돌린다.")
    elif any(run.delays_before.values()):
        print("응답 지연을 0으로 바꿔 두었다 (잠근 채라 전원을 끄면 사라진다).")
        print(
            "서보 전원(+ 플러그)을 뺐다 꽂고 dump를 다시 떠서, 7번이 처음 값으로 돌아왔는지 본다."
        )
    else:
        print("모든 서보의 응답 지연이 이미 0이라 한 번만 쟀다.")


if __name__ == "__main__":
    sys.exit(main())
