"""공차 시험편이 서포트 없이 뽑힐 수 있는 올바른 형상인지 확인한다."""

from __future__ import annotations

import pytest
from build123d import Box, Pos

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
        # 서는 너트의 옆 홈은 폭이 세로다
        *(w for w, _ in coupons.UPRIGHT_SLOTS),
        coupons.UPRIGHT_FLOOR,
        coupons.UPRIGHT_BLOCK[2],
        *coupons.DROP_BLOCK[1:],
        coupons.PIN_DEPTH,
        coupons.SEAT_DEPTH,
        coupons.LIP_HEIGHT,
        coupons.SHOULDER_H,
        coupons.STUB_BASE,
        coupons.STUB_HEIGHT,
    ]
    for height in heights:
        layers = height / coupons.LAYER
        assert layers == pytest.approx(round(layers), abs=1e-9), height


def test_upright_nut_slots_keep_the_nut_in_the_same_pose_as_the_parts() -> None:
    # 옆 홈: 맞변 거리가 세로(z)이고 두께가 볼트 방향(y)이다. 받침 기둥 귀와 같은 모양이다
    slot = coupons.nut_slots_upright()
    bb = slot.bounding_box()
    bx, by, bh = coupons.UPRIGHT_BLOCK
    assert bb.size.Y == pytest.approx(by)
    assert bb.size.Z == pytest.approx(bh)
    # 홈이 칸 덩어리의 +x 면으로 열려서 너트를 옆에서 밀어 넣을 수 있다
    width, thickness = coupons.UPRIGHT_SLOTS[0]
    x_face = coupons.MARGIN + bx
    probe = Pos(x_face - 0.5, by / 2, coupons.UPRIGHT_FLOOR + width / 2) * Box(
        0.4, thickness - 0.4, width - 0.4
    )
    assert (slot & probe).volume == pytest.approx(0, abs=1e-6)
    # 떨어뜨리는 홈은 윗면으로 열린다
    drop = coupons.nut_drops()
    _, height = coupons.DROP_BLOCK
    ny, _ = coupons.DROP_NUT
    w, t = coupons.DROP_SLOTS[0]
    top = Pos(coupons.MARGIN + coupons.PITCH / 2, ny, height - 0.5) * Box(w - 0.4, t - 0.4, 0.4)
    assert (drop & top).volume == pytest.approx(0, abs=1e-6)
