"""공차 시험편이 서포트 없이 뽑힐 수 있는 올바른 형상인지 확인한다."""

from __future__ import annotations

import pytest

from horilab_design import coupons


@pytest.mark.slow
def test_coupons_are_valid_solids() -> None:
    for name, build in coupons.COUPONS.items():
        part = build()
        assert part.is_valid, name
        assert part.volume > 0, name


def test_heights_follow_the_layer_height() -> None:
    # 높이 방향 치수가 층 높이의 배수가 아니면 슬라이서가 반올림해서 칸끼리 차이가 사라진다.
    heights = [
        coupons.POCKET_DEPTH,
        coupons.LABEL_DEPTH,
        coupons.FRAME_HEIGHT,
        coupons.HORN_RECESS_DEPTH,
        *(h for _, h in coupons.NUT_SLOTS),
        coupons.PIN_DEPTH,
        coupons.SEAT_DEPTH,
        coupons.LIP_HEIGHT,
        coupons.SHOULDER_H,
        coupons.STUB_BASE,
        coupons.STUB_HEIGHT,
        coupons.GAUGE_PLATE,
    ]
    for height in heights:
        layers = height / coupons.LAYER
        assert layers == pytest.approx(round(layers), abs=1e-9), height
