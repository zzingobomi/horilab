"""시뮬레이션 서버의 TCP 쪽: JSON 한 줄 요청에 JSON 한 줄로 답한다 (0033). 한 번에 한 클라이언트를 받는다.

요청 (op):
- hello {period, servos: [{id, joint, model}]}: 세계를 새로 만든다. 맨 먼저 보낸다.
- ping {id} → {model}. 없는 서보는 {"ok": false, "error": "no_reply"}.
- read {id, registers} → {values, fault}
- sync_read {ids, registers} → {replies: {id: {values, fault} | {error}}, time}. 답하기 전에 한 주기만큼 진행한다.
- write {id, register, value} → {fault}. sync_write {register, values: [[id, value]]}.
- write_trial {id, values: [[register, value]]}. dump {id} → {bytes} (메모리 표는 흉내 내지 않아 비어 있다).
- 시험용: power_cycle {id}, state → {time, angles}.
"""

from __future__ import annotations

import argparse
import json
import socketserver
import time
from pathlib import Path
from typing import Any

from horilab_sim.world import World

NO_REPLY = {"ok": False, "error": "no_reply"}
# 레포 뿌리에서 띄운다고 본 기본 경로. 모델은 `uv run python -m horilab_design.mjcf`가 쓴다
SIM_MODEL = Path("build/sim/horibot.xml")
SERVO_SPECS = Path("params/servos.toml")


class Session:
    """클라이언트 하나와의 대화. 세계는 hello로 만든다."""

    def __init__(self, model: Path, servo_specs: Path, realtime: bool) -> None:
        self.world: World | None = None
        self.model = model
        self.servo_specs = servo_specs
        self.realtime = realtime
        self.started = 0.0

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        op = request.get("op")
        if op == "hello":
            servos = [(int(s["id"]), str(s["joint"]), int(s["model"])) for s in request["servos"]]
            self.world = World(self.model, self.servo_specs, float(request["period"]), servos)
            self.started = time.monotonic()
            return {"ok": True, "time": 0.0}
        world = self.world
        if world is None:
            return {"ok": False, "error": "hello를 먼저 보낸다"}
        servos = world.servos
        if op == "ping":
            servo = servos.get(int(request["id"]))
            return NO_REPLY if servo is None else {"ok": True, "model": servo.model}
        if op == "read":
            servo = servos.get(int(request["id"]))
            if servo is None:
                return NO_REPLY
            values = {name: servo.regs.get(name, 0) for name in request["registers"]}
            return {"ok": True, "values": values, "fault": servo.regs["status"]}
        if op == "sync_read":
            world.step()
            self._pace(world)
            replies: dict[str, Any] = {}
            for servo_id in request["ids"]:
                servo = servos.get(int(servo_id))
                if servo is None:
                    replies[str(servo_id)] = {"error": "no_reply"}
                else:
                    values = {name: servo.regs.get(name, 0) for name in request["registers"]}
                    replies[str(servo_id)] = {"values": values, "fault": servo.regs["status"]}
            return {"ok": True, "replies": replies, "time": world.time}
        if op == "write":
            servo_id = int(request["id"])
            if servo_id not in servos:
                return NO_REPLY
            world.write(servo_id, str(request["register"]), int(request["value"]))
            return {"ok": True, "fault": servos[servo_id].regs["status"]}
        if op == "sync_write":
            for servo_id, value in request["values"]:
                if int(servo_id) in servos:
                    world.write(int(servo_id), str(request["register"]), int(value))
            return {"ok": True}
        if op == "write_trial":
            servo_id = int(request["id"])
            if servo_id not in servos:
                return NO_REPLY
            for name, value in request["values"]:
                world.write(servo_id, str(name), int(value))
            return {"ok": True}
        if op == "dump":
            return NO_REPLY if int(request["id"]) not in servos else {"ok": True, "bytes": []}
        if op == "power_cycle":
            world.power_cycle(int(request["id"]))
            return {"ok": True}
        if op == "state":
            angles = {servo.joint: world.angle(servo.id) for servo in servos.values()}
            return {"ok": True, "time": world.time, "angles": angles}
        return {"ok": False, "error": f"모르는 요청이다: {op}"}

    def _pace(self, world: World) -> None:
        """실시간 모드면 시뮬레이션 시간이 벽시계를 앞서지 않게 기다린다."""
        if self.realtime:
            ahead = self.started + world.time - time.monotonic()
            if ahead > 0:
                time.sleep(ahead)


class Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        server = self.server
        session = Session(
            Path(getattr(server, "model", SIM_MODEL)),
            Path(getattr(server, "servo_specs", SERVO_SPECS)),
            bool(getattr(server, "realtime", False)),
        )
        for line in self.rfile:
            try:
                reply = session.handle(json.loads(line))
            except (ValueError, KeyError, TypeError) as error:
                reply = {"ok": False, "error": f"{type(error).__name__}: {error}"}
            self.wfile.write((json.dumps(reply) + "\n").encode())
            self.wfile.flush()


class Server(socketserver.TCPServer):
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        model: Path = SIM_MODEL,
        servo_specs: Path = SERVO_SPECS,
        realtime: bool = False,
    ) -> None:
        super().__init__(address, Handler)
        self.model = model
        self.servo_specs = servo_specs
        self.realtime = realtime


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="horilab 시뮬레이션 서버 (0033)")
    parser.add_argument("--host", default="127.0.0.1", help="파이의 컨트롤러를 붙이려면 0.0.0.0")
    parser.add_argument("--port", type=int, default=5600, help="0이면 빈 포트를 고른다")
    parser.add_argument(
        "--realtime", action="store_true", help="벽시계에 맞춰 진행한다 (뷰어·펜던트)"
    )
    parser.add_argument("--model", type=Path, default=SIM_MODEL, help="설계가 쓴 MJCF")
    parser.add_argument("--servos", type=Path, default=SERVO_SPECS, help="서보 사양 (멈춤 토크)")
    args = parser.parse_args(argv)
    with Server((args.host, args.port), args.model, args.servos, args.realtime) as server:
        print(f"listening {server.server_address[1]}", flush=True)
        server.serve_forever()
