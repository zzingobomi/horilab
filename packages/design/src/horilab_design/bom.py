"""구매 데이터(data/purchases.toml)로 사람이 보는 부품표 docs/bom.md를 만든다 (docs/decisions/0002, 0004, 0006).

    uv run python -m horilab_design.bom

설계 부품표(볼트·너트·베어링 개수)는 부품 코드가 생기면 여기에 합친다.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from pydantic import Field

from horilab_design.values import REPO_ROOT, _Values, read_toml

PURCHASES_FILE = REPO_ROOT / "data" / "purchases.toml"
BOM_FILE = REPO_ROOT / "docs" / "bom.md"
BUDGET_KRW = 300_000  # 서보를 뺀 추가 부품 예산 (0004)

Status = Literal["buy_now", "at_stage", "decide_first", "if_needed", "ordered"]

SECTIONS: tuple[tuple[Status, str], ...] = (
    ("buy_now", "지금 산다"),
    ("at_stage", "단계에 맞춰 산다"),
    ("decide_first", "규격을 정한 뒤 산다"),
    ("if_needed", "조건이 생기면 산다"),
    ("ordered", "주문함"),
)
STAGES = (
    "0 레포 준비",
    "1 골격 설계",
    "2 서보와 연결 준비",
    "3 관절 하나 시험",
    "4 손목과 그리퍼 교체부",
    "5 팔 전체",
    "6 실물과 시뮬레이션 연동",
)


class Item(_Values):
    name: str
    qty: int = Field(gt=0)
    stage: int = Field(ge=0, le=len(STAGES) - 1)  # 처음 필요한 제작 로드맵 단계
    status: Status
    why: str
    vendor: str = ""
    lead_days: tuple[int, int] | None = None  # 배송 기간 [일]
    unit_price_krw: int | None = None
    unit_price_usd: float | None = None
    priced: str = ""  # 값을 확인한 때
    servo: bool = False  # 서보는 예산과 따로 센다 (0004)

    def price(self) -> str:
        if self.unit_price_krw is not None:
            return f"{self.unit_price_krw * self.qty:,}원"
        if self.unit_price_usd is not None:
            return f"${self.unit_price_usd * self.qty:g}"
        return "미확인"


class Purchases(_Values):
    item: list[Item]


def load_purchases(path: Path = PURCHASES_FILE) -> Purchases:
    return Purchases.model_validate(read_toml(path))


def _cell(text: str) -> str:
    return text.replace("|", "\\|")


def render(purchases: Purchases) -> str:
    items = purchases.item
    budget = [i for i in items if not i.servo and i.status != "if_needed"]
    krw = sum(i.unit_price_krw * i.qty for i in budget if i.unit_price_krw is not None)
    usd = sum(i.unit_price_usd * i.qty for i in budget if i.unit_price_usd is not None)
    unknown = sum(i.unit_price_krw is None and i.unit_price_usd is None for i in budget)
    now = [i for i in items if i.status == "buy_now"]
    now_krw = sum(i.unit_price_krw * i.qty for i in now if i.unit_price_krw is not None)
    now_usd = sum(i.unit_price_usd * i.qty for i in now if i.unit_price_usd is not None)
    lines = [
        "# 부품표",
        "",
        "이 파일은 `uv run python -m horilab_design.bom`이 [data/purchases.toml](../data/purchases.toml)에서 만든다. "
        "고칠 때는 데이터 파일을 고친다. 구매 방식은 [0004](decisions/0004-purchasing.md), "
        "가진 것은 [hardware.md](hardware.md#가진-것)에 있다.",
        "",
        f"- 지금 살 것: {len(now)}개, 확인한 값 {now_krw:,}원"
        + (f" + ${now_usd:g}" if now_usd else ""),
        f"- 예산 {BUDGET_KRW:,}원(서보 제외, 조건부 품목 제외) 중 확인한 값: {krw:,}원"
        + (f" + ${usd:g}" if usd else "")
        + f", 값 미확인 {unknown}개",
    ]
    for status, title in SECTIONS:
        rows = sorted((i for i in items if i.status == status), key=lambda i: i.stage)
        if not rows:
            continue
        lines += [
            "",
            f"## {title}",
            "",
            "| 품목 | 수량 | 처음 필요한 단계 | 구매처 | 배송 | 값 | 왜 |",
            "|---|---|---|---|---|---|---|",
        ]
        for i in rows:
            lead = f"{i.lead_days[0]}~{i.lead_days[1]}일" if i.lead_days else ""
            price = i.price() + (f" ({i.priced})" if i.priced and i.price() != "미확인" else "")
            lines.append(
                f"| {_cell(i.name)} | {i.qty} | {STAGES[i.stage]} | {_cell(i.vendor)} | {lead} | "
                f"{price} | {_cell(i.why)} |"
            )
    return "\n".join(lines) + "\n"


def main() -> None:
    BOM_FILE.write_text(render(load_purchases()), encoding="utf-8", newline="\n")
    print(f"썼다: {BOM_FILE.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
