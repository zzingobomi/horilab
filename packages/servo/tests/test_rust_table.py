"""Rust 메모리 표(crates/servo)가 이 기준 구현의 표와 같은지 본다.

두 구현이 같은 표를 써야 실물에서 결과(덤프, 동기 읽기)를 맞춰 볼 수 있다
(docs/decisions/0024-servo-bus.md). 이 표는 공식 SDK 상수와도 맞춰 봤다 (test_registers.py).
Python 기준 구현을 지울 때 이 검사도 같이 지운다.
"""

import re
from pathlib import Path

from horilab_servo.feetech import registers as reg
from horilab_servo.feetech.registers import Area, Encoding

ROOT = Path(__file__).resolve().parents[3]
RUST_TABLE = ROOT / "crates" / "servo" / "src" / "feetech" / "registers.rs"

# pub const NAME: Register = kind("name", address, size, "unit")[.encoded(Encoding::X)];
# rustfmt가 줄을 나눠도 읽히게 줄바꿈을 건너뛴다. 단위 글에 괄호가 있어서 끝은 ;로 찾는다.
DEFINITION = re.compile(
    r'pub const \w+: Register =\s*(version|eeprom|ram|feedback)\(\s*"(\w+)",\s*(\d+),\s*(\d+),'
    r".*?\)\s*(?:\.encoded\(Encoding::(\w+)\))?\s*;",
    re.DOTALL,
)
KINDS = {
    "version": (Area.EEPROM, False),
    "eeprom": (Area.EEPROM, True),
    "ram": (Area.RAM, True),
    "feedback": (Area.RAM, False),
}
ENCODINGS = {
    "": Encoding.UNSIGNED,
    "Sign15": Encoding.SIGN15,
    "Sign10": Encoding.SIGN10,
    "Offset": Encoding.OFFSET,
}

Row = tuple[int, int, Area, bool, Encoding]


def _rust_table() -> dict[str, Row]:
    text = RUST_TABLE.read_text(encoding="utf-8")
    table: dict[str, Row] = {}
    for kind, name, address, size, encoding in DEFINITION.findall(text):
        area, writable = KINDS[kind]
        table[name] = (int(address), int(size), area, writable, ENCODINGS[encoding])
    return table


def test_rust_table_matches() -> None:
    expected = {r.name: (r.address, r.size, r.area, r.writable, r.encoding) for r in reg.ALL}
    assert _rust_table() == expected
