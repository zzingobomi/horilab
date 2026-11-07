"""부품표 문서가 구매 데이터와 맞는지 본다 (docs/decisions/0006)."""

from __future__ import annotations

import datetime as dt

import pytest
from pydantic import ValidationError

from horilab_design import bom


def test_bom_document_matches_the_purchase_data() -> None:
    # 데이터를 고치고 문서를 다시 만들지 않으면 실패한다: uv run python -m horilab_design.bom
    expected = bom.render(bom.load_purchases())
    assert bom.BOM_FILE.read_text(encoding="utf-8") == expected


def test_every_item_says_why_it_is_needed() -> None:
    for item in bom.load_purchases().item:
        assert item.why.strip(), item.name


def test_an_item_to_buy_has_a_link_or_search_words_and_a_time() -> None:
    with pytest.raises(ValidationError, match="검색어"):
        bom.Item(name="나사", qty=1, stage=3, why="체결", when="stage")
    with pytest.raises(ValidationError, match="since"):
        bom.Item(name="저울", qty=1, stage=0, why="무게", when="now", search="저울")
    with pytest.raises(ValidationError, match="when"):
        bom.Item(name="저울", qty=1, stage=0, why="무게", search="저울")
    # 주문한 것은 언제 살지와 링크가 없어도 된다
    bom.Item(name="서보", qty=1, stage=2, why="어깨", ordered=dt.date(2026, 11, 7))


def test_received_items_stay_in_the_budget() -> None:
    got = bom.Item(
        name="서보",
        qty=1,
        stage=2,
        why="어깨",
        ordered=dt.date(2026, 11, 7),
        received=dt.date(2026, 11, 14),
        unit_price_krw=155960,
    )
    text = bom.render(bom.Purchases(item=[got]))
    assert "## 배송 중" not in text
    assert "| 받은 것 | 155,960원 |" in text
    assert "| **합** | **155,960원** |" in text
    assert "144,040원 남는다" in text
    assert "| 서보 | 1 | 2026-11-14 |" in text


def test_ordered_items_leave_the_buy_now_list() -> None:
    now = bom.Item(
        name="저울",
        qty=1,
        stage=0,
        why="무게",
        when="now",
        since=dt.date(2026, 11, 8),
        search="저울",
    )
    sent = now.model_copy(update={"ordered": dt.date(2026, 11, 9)})
    text = bom.render(bom.Purchases(item=[sent]))
    head, _, rest = text.partition("## 배송 중")
    assert "| 저울 |" not in head
    assert "| 저울 | 1 | 2026-11-09 |" in rest
