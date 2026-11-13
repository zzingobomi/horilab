"""배선 검토 (0031): 관절을 넘는 선 길이, 구간마다 고른 선, 살 위 통로, 부품에 낸 선 자리."""

from __future__ import annotations

import math

import pytest
from build123d import Pos

from horilab_design.parts import base as bp
from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
from horilab_design.parts import shoulder as sp
from horilab_design.parts.shapes import box, solid
from horilab_design.studies import wiring as wr


def test_wire_is_longest_at_a_stop_and_slack_in_the_middle() -> None:
    for name, c in wr.crossings().items():
        longest = c.longest()
        assert longest == pytest.approx(max(c.length(q) for q in range(-163, 164)), abs=0.5), name
        # 범위 가운데에서 남는 길이는 테에 160° 안팎 감기는 길이다
        wrapped = math.radians(j.STOP_DEG) * (c.drum_r + j.WIRE_T / 2)
        assert c.slack(0.0) == pytest.approx(wrapped, rel=0.2), name
        assert c.slack(0.0) > c.slack(90.0) > c.slack(c.stop_deg) > -1e-9, name
    # J1은 두 누르개가 범위 가운데에서 같은 방위각이라 양쪽이 같다
    j1 = wr.crossings()["J1"]
    assert j1.length(160.0) == pytest.approx(j1.length(-160.0))


def test_each_wire_reaches_with_a_cable_on_hand() -> None:
    chosen = {w.name: w for w in wr.wires()}
    assert [w.cable for w in chosen.values()] == [900.0, 900.0, 900.0, 300.0, 300.0, 200.0]
    for w in chosen.values():
        assert w.left >= wr.SPARE, (w.name, w.need, w.left)
    # 산 300 mm 선으로는 아래팔을 지나는 J3 → J4가 닿지 않는다 (0016의 어림과 다르다)
    assert chosen["J3 → J4"].need > wr.usable(300.0)
    # 띠 밑 누르개에서 어댑터까지 20 cm 넘게 남는다
    assert chosen["띠 밑 누르개 → J3"].left > 200.0


def test_wire_crosses_the_web_where_the_stop_never_sweeps() -> None:
    for drum_r, post_r, ear in (
        (j.R_DRUM, j.R_POST, j.EAR_HW),
        (sp.R_DRUM, sp.R_POST, sp.EAR_HW),
    ):
        post_soft, ear_soft = wr.corridor(drum_r, post_r, j.SOFT_DEG, ear)
        post_hard, ear_hard = wr.corridor(drum_r, post_r, j.STOP_DEG, ear)
        # 프로그램 한계에서는 선 옆으로 1.5 mm 넘게 남고, 멈춤 각에서도 선에 닿지 않는다
        assert min(post_soft, ear_soft) - j.WIRE_W / 2 > 1.5
        assert min(post_hard, ear_hard) - j.WIRE_W / 2 > 0.4
    # 기둥이 살에 닿는 각에서 기둥 가장자리가 살의 옆면 가까이 있다 (둥근 기둥이 비스듬한 옆면에 닿는다)
    web = j.R_POST * math.sin(math.radians(j.WEB_HALF))
    assert wr.corridor(j.R_DRUM, j.R_POST, j.STOP_DEG)[0] == pytest.approx(web, abs=0.15)
    # 손목 몸통의 옆 홈에서 나온 선은 멈춤 귀 끝 가까이를 지나고, 가운데 홈에서 나온 선은 멀리 지난다
    side = sum(wrist.NOTCH) / 2
    assert wr.exit_to_ear(side, j.STOP_DEG) - j.WIRE_W / 2 < 1.5
    assert wr.exit_to_ear(0.0, j.STOP_DEG) - j.WIRE_W / 2 > 5.0
    # 손목 몸통의 가운데 홈과 링크의 선 창은 선 폭보다 넓고, 플러그는 창을 지나간다
    assert 2 * wrist.EXIT_HW > j.WIRE_W + 1.0
    # 손목 몸통의 옆 홈과 링크 발의 선 틈도 선 폭(잰 값 4.75)보다 넓다. 긴 틈은 선을 세워 밀어 넣을 만큼 구멍과 겹친다
    assert wrist.NOTCH[1] - wrist.NOTCH[0] > j.WIRE_W + 0.9
    assert links.FOOT_SLOT_Z[1] - links.FOOT_SLOT_Z[0] > j.WIRE_W + 0.9
    assert links.FOOT_SLOT_OPEN > j.WIRE_T + 0.9 and links.FOOT_SLOT_H > j.WIRE_T + 1.0
    for foot in (links.STS_FOOT, links.SHOULDER_FOOT):
        assert (
            links.FOOT_SLOT_Z[1] <= foot.hw - links.WALL + 1e-9
        )  # 발 상자의 안쪽 벽을 넘지 않는다
    # 서보 먼 끝 밑의 길은 혼 볼트 머리와 목 안쪽 벽 사이이고, 선 폭이 겨우 든다
    head_edge = max(abs(z) for _x, z in j.bolt_xz()) + 2.85
    assert j.NECK_IN - head_edge > j.WIRE_W
    assert wrist.SIDE_LANE == pytest.approx((head_edge + j.NECK_IN) / 2, abs=0.05)
    assert wrist.HEAD > j.WIRE_T + 1.0
    assert 2 * j.WIN_Z >= wr.PLUG[2] + 1.0 and j.WIN_X[1] - j.WIN_X[0] >= wr.PLUG[1] + 0.5
    assert 2 * sp.WIN_Z >= wr.PLUG[2] + 1.0 and sp.WIN_X[1] - sp.WIN_X[0] >= wr.PLUG[1] + 0.5


def test_gap_under_the_brim_is_too_narrow_for_a_wire() -> None:
    assert j.LIP_GAP < j.WIRE_T - 0.4
    assert j.LIP_GAP >= 0.6  # 도는 원판과 서 있는 원통 사이의 축 방향 틈


def test_d405_cable_cannot_wrap_the_brims() -> None:
    # 두 몸통 사이 층(5.8 mm)에 감기려면 선이 3.8 mm보다 가늘어야 한다. 가진 기본 선은 5.12 mm다
    assert wr.layer_height() == pytest.approx(j.GAP + j.BRIM_T)
    assert wr.d405_max_diameter() == pytest.approx(3.8)
    assert wr.D405_D == pytest.approx(5.12)
    assert wr.D405_D > wr.d405_max_diameter()
    # 테에 감으면 범위 가운데에서 남는 선이 반지름 30 mm 아래의 고리가 된다. 움직이는 고리의 기준은 지름의 10배다
    x = wr.crossings()
    for name in ("J1", "J2", "J3", "J4", "J5"):
        assert wr.d405_mid_loop(x[name], wr.D405_D) < 30.0 < wr.BEND_RATIO * wr.D405_D, name
    # 고리 식: 반원(현 2R, 호 πR)이면 R이 나오고, 선이 두 끝 사이보다 짧으면 고리가 없다
    assert wr.loop_radius(40.0, math.pi * 20.0) == pytest.approx(20.0, rel=1e-6)
    assert wr.loop_radius(50.0, 40.0) == math.inf
    # 기본 선 1 m는 가장 짧은 길(테에 감는 길)로도 베이스의 띠 밑 누르개까지 닿지 않는다
    assert sum(v for _, v in wr.d405_path(wr.D405_D)) > wr.D405_STOCK_MM


def test_wire_masses_for_the_arm_model() -> None:
    masses = wr.wire_masses()
    assert set(masses) == {"upper_mid", "upper_end", "fore_mid", "fore_end", "wrist1", "wrist2"}
    assert sum(masses.values()) == pytest.approx(0.052, abs=0.006)
    # 20 cm 선의 무게(실측 3.5 g)가 식에서 다시 나온다
    again = wr.usable(200.0) * 1e-3 * wr.SERVO_WIRE_KG_M + 2 * wr.PLUG_KG
    assert again == pytest.approx(3.5e-3)


def test_base_clamps_line_up_in_the_middle_of_the_range() -> None:
    x, z = bp.EAR_CLAMP
    assert math.degrees(math.atan2(z, x)) % 360.0 == pytest.approx(bp.WIRE_AZ, abs=1.0)
    # 귀 위 누르개 나사는 귀 안에 있고(가장자리에서 3 mm 넘게), 영점 핀 구멍에서 멀다
    assert bp.R_TABLE - math.hypot(x, z) > 3.0
    assert math.dist((x, z), (-bp.R_PIN, 0.0)) > 10.0
    # 돌림판 밑의 테두리는 몸통 윗면의 바깥 벽 위에 있다 (멈춤 홈 밖)
    assert bp.CH_R[1] < bp.TABLE_LIP[0] and bp.TABLE_LIP[1] <= bp.R_TABLE + bp.FLARE


@pytest.mark.slow
def test_wire_openings_are_open_in_the_printed_bodies() -> None:
    # 링크 발: 끝벽 밑의 구멍과 발 바닥의 긴 틈이 발 속까지 뚫려 있다
    for name, (length, tube_r, foot) in links.LINKS.items():
        part = links.link(length, tube_r, foot)
        xa = -length
        z0, z1 = links.FOOT_SLOT_Z
        mouse = box(
            xa - foot.hx - 0.5,
            xa - foot.hx + links.WALL + 0.5,
            j.FRONT - 2.0,
            j.FRONT - 0.2,
            z0 + 0.5,
            z1 - 0.5,
        )
        floor = box(
            xa - foot.hx + links.WALL + 1.0,
            xa - foot.head_r,
            j.FRONT - links.WALL - 0.8,
            j.FRONT - 0.2,
            z0 + 0.5,
            z1 - 0.5,
        )
        for probe in (mouse, floor):
            assert solid(part & probe).volume < 0.01, name
    # 손목 몸통: 앞벽 발 쪽 끝의 가운데 홈과 옆 홈
    body = wrist.wrist_body()
    for zc in (0.0, sum(wrist.NOTCH) / 2, -sum(wrist.NOTCH) / 2):
        probe = box(
            wrist.X_FOOT + 0.2, wrist.X_FOOT + 2.0, j.NECK_FRONT_IN, j.FRONT, zc - 1.5, zc + 1.5
        )
        assert solid(body & probe).volume < 0.01, zc
    # 출력 원판의 테두리가 원통 앞 끝과 LIP_GAP만 띄운다
    disc = j.out_disc()
    lip = box(
        -j.R_DRUM, -j.R_DRUM + j.LIP_W - 0.2, j.FRONT + j.LIP_GAP + 0.05, j.SEAM - 0.05, -1.0, 1.0
    )
    assert solid(disc & lip).volume == pytest.approx(lip.volume, rel=0.1)


@pytest.mark.slow
def test_base_band_and_clamp_holes() -> None:
    core = bp.core()
    assert core.is_valid and len(core.solids()) == 1
    # 띠의 위 턱이 몸통보다 넓고, 누르개 나사 구멍이 띠 밑에 뚫려 있다
    assert core.bounding_box().size.X == pytest.approx(2 * (bp.R_CORE + bp.BAND_LIP), abs=0.3)
    az = math.radians(bp.WIRE_AZ)
    y = bp.FRONT - bp.BAND_H - bp.CLAMP_DROP
    r = bp.R_CORE - 3.0
    hole = solid(
        Pos(r * math.cos(az), 0, r * math.sin(az)) * box(-0.6, 0.6, y - 0.6, y + 0.6, -0.6, 0.6)
    )
    assert solid(core & hole).volume < 0.01
    # J1 몸통 뒤쪽 귀의 누르개 나사 구멍
    body = bp.as_child(bp.j1_body())
    x, z = bp.EAR_CLAMP
    probe = box(x - 0.6, x + 0.6, bp.ATOP + 1.0, bp.ATOP + j.EAR_T - 1.0, z - 0.6, z + 0.6)
    assert solid(body & probe).volume < 0.01
