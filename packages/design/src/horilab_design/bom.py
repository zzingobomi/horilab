"""구매 데이터(data/purchases.toml)로 사람이 보는 부품표 docs/bom.md를 만든다 (docs/decisions/0002, 0004, 0006).

    uv run python -m horilab_design.bom

부품표에는 아직 손에 없는 것만 둔다. 맨 위에서 지금 주문할 것과 배송 중인 것을 바로 보게 한다.
받은 것은 데이터에서 지우고 docs/hardware.md의 "가진 것"에 넣는다.
설계 부품표(볼트·너트·베어링 개수)는 부품 코드가 생기면 여기에 합친다.
"""

from __future__ import annotations

import datetime as dt
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from horilab_design.values import REPO_ROOT, _Values, read_toml

PURCHASES_FILE = REPO_ROOT / "data" / "purchases.toml"
BOM_FILE = REPO_ROOT / "docs" / "bom.md"
BUDGET_KRW = 300_000  # 서보를 뺀 추가 부품 예산 (0004)

# 언제 사나. 주문한 것(ordered)은 이것과 상관없이 "배송 중"이다.
When = Literal["now", "stage", "pick", "if_needed"]
LATER: tuple[tuple[When, str, str], ...] = (
    (
        "stage",
        "단계 전에 산다",
        "그 앞 단계를 하는 동안 주문한다. 배송이 길면 더 일찍 지금 주문할 것으로 옮긴다.",
    ),
    (
        "pick",
        "고르는 중",
        "살 것은 정했고 규격이나 제품을 고르는 중이다. 고르면 지금 주문할 것으로 옮긴다.",
    ),
    ("if_needed", "필요해지면", "아직 살지 모른다. 왜 칸의 조건이 생기면 산다."),
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
    why: str
    when: When | None = None
    since: dt.date | None = None  # 지금 사기로 한 날 (when = "now")
    ordered: dt.date | None = None  # 주문한 날. 있으면 배송 중이다
    arrive: tuple[dt.date, dt.date] | None = None  # 도착 예정 (첫날, 끝날)
    received: dt.date | None = None  # 받은 날. 있으면 예산과 받은 것에만 나온다
    vendor: str = ""
    url: str = ""  # 상품이나 사는 곳 링크
    search: str = ""  # 찾을 때 쓸 검색어 (모델 번호나 규격)
    lead_days: tuple[int, int] | None = None  # 배송 기간 [일]
    unit_price_krw: int | None = None
    unit_price_usd: float | None = None
    priced: str = ""  # 값을 확인한 때

    @model_validator(mode="after")
    def _complete(self) -> Item:
        if self.ordered is None and self.received is None:
            if self.when is None:
                raise ValueError(f"{self.name}: 주문하지 않은 품목은 when(언제 사나)이 있어야 한다")
            if self.when == "now" and self.since is None:
                raise ValueError(
                    f"{self.name}: 지금 살 것은 since(지금 사기로 한 날)가 있어야 한다"
                )
            if not self.url and not self.search:
                raise ValueError(f"{self.name}: 링크가 없으면 검색어라도 있어야 한다")
        return self

    def price(self) -> str:
        if self.unit_price_krw is not None:
            return f"{self.unit_price_krw * self.qty:,}원"
        if self.unit_price_usd is not None:
            return f"${self.unit_price_usd * self.qty:g}"
        return "모름"

    def known(self) -> bool:
        return self.unit_price_krw is not None or self.unit_price_usd is not None


class Purchases(_Values):
    item: list[Item]


def load_purchases(path: Path = PURCHASES_FILE) -> Purchases:
    return Purchases.model_validate(read_toml(path))


def _cell(text: str) -> str:
    return text.replace("|", "\\|")


def _where(i: Item) -> str:
    if i.url:
        place = f"[{_cell(i.vendor or '링크')}]({i.url})"
    else:
        place = _cell(i.vendor) if i.vendor else "찾는 중"
    if i.search:
        place += f" · 검색어 `{_cell(i.search)}`"
    return place


def _lead(i: Item) -> str:
    return f"{i.lead_days[0]}~{i.lead_days[1]}일" if i.lead_days else ""


def _total(items: list[Item]) -> str:
    krw = sum(i.unit_price_krw * i.qty for i in items if i.unit_price_krw is not None)
    usd = sum(i.unit_price_usd * i.qty for i in items if i.unit_price_usd is not None)
    unknown = sum(not i.known() for i in items)
    parts = ([f"{krw:,}원"] if krw else []) + ([f"${usd:g}"] if usd else [])
    text = " + ".join(parts)
    if unknown:
        text += (", " if text else "") + f"값 모름 {unknown}개"
    return text or "0원"


def _budget(rows: list[tuple[str, list[Item]]]) -> list[str]:
    """묶음마다 합을 보이고, 모두 더한 값을 예산 기준과 견준다."""
    out = ["| | 값 |", "|---|---|"]
    for title, group in rows:
        out.append(f"| {title} | {_total(group) if group else '-'} |")
    every = [i for _, group in rows for i in group]
    out.append(f"| **합** | **{_total(every)}** |")
    krw = sum(i.unit_price_krw * i.qty for i in every if i.unit_price_krw is not None)
    gap = BUDGET_KRW - krw
    side = f"{gap:,}원 남는다" if gap >= 0 else f"{-gap:,}원 넘는다"
    out += [
        "",
        f"- 예산은 {BUDGET_KRW:,}원 안팎이고 서보도 들어간다. 조금 넘는 것은 괜찮고, 수십만 원 차이가 나면 "
        '다시 생각한다 ([0004](decisions/0004-purchasing.md)). "필요해지면" 품목은 아직 살지 몰라서 넣지 않는다.',
        f"- 원화로 값을 아는 것만 빼면 {side}. 달러 값과 값 모름은 아직 들어가지 않았다.",
        "- 값은 확인한 때의 값이다. 지금 주문할 것의 값은 주문할 때 다시 본다.",
    ]
    return out


def render(purchases: Purchases) -> str:
    items = [i for i in purchases.item if i.received is None]
    received = sorted(
        (i for i in purchases.item if i.received is not None),
        key=lambda i: i.received or dt.date.min,
    )
    now = sorted((i for i in items if i.ordered is None and i.when == "now"), key=lambda i: i.stage)
    shipping = [i for i in items if i.ordered is not None]
    later = [i for i in items if i.ordered is None and i.when != "now"]
    lines = [
        "# 부품표",
        "",
        "이 파일은 `uv run python -m horilab_design.bom`이 [data/purchases.toml](../data/purchases.toml)에서 만든다. "
        "고칠 때는 데이터 파일을 고친다. 구매 방식은 [0004](decisions/0004-purchasing.md), "
        "가진 것은 [hardware.md](hardware.md#가진-것)에 있다.",
        "",
        "위에는 아직 손에 없는 것만 있다. 주문하면 배송 중으로 옮기고, 받으면 맨 아래 받은 것으로 옮겨 "
        "예산에 남긴다 (가진 것에도 넣는다).",
        "",
        "| 지금 주문할 것 | 배송 중 | 나중에 살 것 |",
        "|---|---|---|",
        f"| **{len(now)}개** ({_total(now)}) | {len(shipping)}개 | {len(later)}개 |",
        "",
        "## 지금 주문할 것",
        "",
    ]
    if now:
        lines += [
            "아직 주문하지 않았다.",
            "",
            "| 품목 | 수량 | 값 | 사는 곳 | 배송 | 왜 지금 | 정한 날 |",
            "|---|---|---|---|---|---|---|",
        ]
        for i in now:
            assert i.since is not None
            lines.append(
                f"| {_cell(i.name)} | {i.qty} | {i.price()} | {_where(i)} | {_lead(i)} | {_cell(i.why)} | "
                f"{i.since.isoformat()} |"
            )
        shops: dict[str, list[Item]] = {}
        for i in now:
            shops.setdefault(i.vendor or "사는 곳 찾는 중", []).append(i)
        lines += [
            "",
            "사는 곳별: "
            + " · ".join(f"{_cell(v)} {len(rows)}개 {_total(rows)}" for v, rows in shops.items()),
        ]
    else:
        lines.append("지금 주문할 것이 없다.")
    if shipping:
        lines += [
            "",
            "## 배송 중",
            "",
            "| 품목 | 수량 | 주문한 날 | 도착 예정 | 사는 곳 | 값 | 왜 |",
            "|---|---|---|---|---|---|---|",
        ]
        for i in sorted(shipping, key=lambda i: i.ordered or dt.date.min):
            assert i.ordered is not None
            arrive = (
                f"{i.arrive[0].isoformat()} ~ {i.arrive[1].isoformat()}" if i.arrive else "모름"
            )
            lines.append(
                f"| {_cell(i.name)} | {i.qty} | {i.ordered.isoformat()} | {arrive} | {_where(i)} | "
                f"{i.price()} | {_cell(i.why)} |"
            )
    lines += ["", "## 나중에 살 것"]
    for when, title, note in LATER:
        rows = sorted((i for i in later if i.when == when), key=lambda i: i.stage)
        if not rows:
            continue
        lines += [
            "",
            f"### {title}",
            "",
            note,
            "",
            "| 품목 | 수량 | 필요한 단계 | 값 | 사는 곳 | 배송 | 왜 |",
            "|---|---|---|---|---|---|---|",
        ]
        for i in rows:
            price = i.price() + (f" ({i.priced})" if i.priced and i.known() else "")
            lines.append(
                f"| {_cell(i.name)} | {i.qty} | {STAGES[i.stage]} | {price} | {_where(i)} | {_lead(i)} | "
                f"{_cell(i.why)} |"
            )
    lines += ["", "## 예산", ""]
    lines += _budget(
        [
            ("받은 것", received),
            ("배송 중", shipping),
            ("지금 주문할 것", now),
            ("나중에 살 것", [i for i in later if i.when != "if_needed"]),
        ]
    )
    if received:
        lines += [
            "",
            "## 받은 것",
            "",
            "| 품목 | 수량 | 받은 날 | 사는 곳 | 값 |",
            "|---|---|---|---|---|",
        ]
        for i in received:
            assert i.received is not None
            lines.append(
                f"| {_cell(i.name)} | {i.qty} | {i.received.isoformat()} | {_where(i)} | {i.price()} |"
            )
    return "\n".join(lines) + "\n"


def main() -> None:
    BOM_FILE.write_text(render(load_purchases()), encoding="utf-8", newline="\n")
    print(f"썼다: {BOM_FILE.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
