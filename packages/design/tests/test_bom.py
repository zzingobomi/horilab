"""부품표 문서가 구매 데이터와 맞는지 본다 (docs/decisions/0006)."""

from __future__ import annotations

from horilab_design import bom


def test_bom_document_matches_the_purchase_data() -> None:
    # 데이터를 고치고 문서를 다시 만들지 않으면 실패한다: uv run python -m horilab_design.bom
    expected = bom.render(bom.load_purchases())
    assert bom.BOM_FILE.read_text(encoding="utf-8") == expected


def test_every_item_says_why_it_is_needed() -> None:
    for item in bom.load_purchases().item:
        assert item.why.strip(), item.name
