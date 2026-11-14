"""시뮬레이션 서버: 서보 거래를 받고, 동기 읽기마다 한 주기만큼 물리를 진행한다 (0033)."""

from __future__ import annotations

import json
import math
import socket
import threading
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import pytest

from horilab_sim.server import Server
from horilab_sim.world import RAD_PER_STEP

Ask = Callable[..., dict[str, Any]]
STATE = ["torque_enable", "goal_position", "torque_limit", "present_position", "present_load"]
TILT = 2048 + round(0.6 / RAD_PER_STEP)  # J2를 0.6 rad 기울인 자리

# 시험용 팔: 위로 선 막대 셋, 관절 j2·j3·j4가 모두 y축으로 돈다 (j4에는 서보를 달지 않는다)
MODEL = """<mujoco model="test">
  <compiler angle="radian"/>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <joint name="j2" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.22" size="0.012"/>
      <body name="link2" pos="0 0 0.22">
        <joint name="j3" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.18" size="0.012"/>
        <body name="link3" pos="0 0 0.18">
          <joint name="j4" type="hinge" axis="0 1 0" range="-3 3" limited="true"/>
          <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.012"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
SERVO_SPECS = "[STS3250]\nstall_torque_kgcm = 50\n"


@pytest.fixture
def ask(tmp_path: Path) -> Iterator[Ask]:
    model, specs = tmp_path / "arm.xml", tmp_path / "servos.toml"
    model.write_text(MODEL, encoding="utf-8")
    specs.write_text(SERVO_SPECS, encoding="utf-8")
    server = Server(("127.0.0.1", 0), model, specs)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = socket.create_connection(("127.0.0.1", server.server_address[1]), timeout=30)
    stream = connection.makefile("rw", encoding="utf-8")

    def request(**body: Any) -> dict[str, Any]:
        stream.write(json.dumps(body) + "\n")
        stream.flush()
        return json.loads(stream.readline())

    hello = request(
        op="hello",
        period=0.01,
        servos=[{"id": 2, "joint": "j2", "model": 2825}, {"id": 3, "joint": "j3", "model": 2825}],
    )
    assert hello["ok"], hello
    yield request
    # 스트림까지 닫아야 연결이 끊겨, 서버가 다음 줄을 기다리지 않고 끝난다
    stream.close()
    connection.close()
    server.shutdown()
    server.server_close()


def run(ask: Ask, beats: int) -> dict[str, Any]:
    """박자만큼 동기 읽기를 보내고 마지막 답을 돌려준다."""
    reply: dict[str, Any] = {}
    for _ in range(beats):
        reply = ask(op="sync_read", ids=[2, 3], registers=STATE)
    return reply


def enable(ask: Ask, servo_id: int, goal: int) -> None:
    """시작 절차: 목표를 쓰고 토크를 켠다."""
    assert ask(op="write", id=servo_id, register="goal_position", value=goal)["ok"]
    assert ask(op="write", id=servo_id, register="torque_enable", value=1)["ok"]


def test_servos_answer_like_after_power_on(ask: Ask) -> None:
    assert ask(op="ping", id=2) == {"ok": True, "model": 2825}
    assert ask(op="ping", id=9)["error"] == "no_reply"
    values = ask(op="read", id=2, registers=STATE)["values"]
    # 전원 직후: 토크 꺼짐, 목표 0, 토크 한도 최대. 영점 자세(팔이 곧게 섬)는 2048로 읽힌다
    assert values == {
        "torque_enable": 0,
        "goal_position": 0,
        "torque_limit": 1000,
        "present_position": 2048,
        "present_load": 0,
    }


def test_each_sync_read_advances_one_period(ask: Ask) -> None:
    reply: dict[str, Any] = {}
    for beat in range(1, 11):
        reply = ask(op="sync_read", ids=[2, 3, 9], registers=STATE)
        assert math.isclose(reply["time"], beat * 0.01, abs_tol=1e-9)
    assert reply["replies"]["9"] == {"error": "no_reply"}


def test_position_servo_holds_a_tilted_arm_with_load(ask: Ask) -> None:
    enable(ask, 3, 2048)
    enable(ask, 2, TILT)
    replies = run(ask, 300)["replies"]
    j2 = replies["2"]["values"]
    assert abs(j2["present_position"] - TILT) <= 10, j2  # 짐에 조금 처진 채 버틴다
    assert abs(j2["present_load"]) > 10, j2  # 팔의 무게를 버티는 힘


def test_arm_falls_when_torque_is_off_or_too_weak(ask: Ask) -> None:
    enable(ask, 3, 2048)
    enable(ask, 2, TILT)
    run(ask, 300)
    ask(op="write", id=2, register="torque_limit", value=5)  # 0.5 %로는 못 버틴다
    weak = run(ask, 100)["replies"]["2"]["values"]["present_position"]
    assert abs(weak - TILT) > 50
    ask(op="write", id=2, register="torque_enable", value=0)
    before = ask(op="state")["angles"]["j2"]
    after = (run(ask, 100), ask(op="state")["angles"]["j2"])[1]
    assert abs(after - before) > 0.05 or abs(after) > 1.0  # 힘이 없으면 계속 떨어진다


def test_power_cycle_resets_ram(ask: Ask) -> None:
    enable(ask, 2, TILT)
    ask(op="write_trial", id=2, values=[["position_offset", 100]])
    ask(op="power_cycle", id=2)
    values = ask(op="read", id=2, registers=["torque_enable", "goal_position", "position_offset"])
    assert values["values"] == {"torque_enable": 0, "goal_position": 0, "position_offset": 0}
