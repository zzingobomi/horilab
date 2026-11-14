"""시뮬레이션 서버 (docs/decisions/0033-planning-and-simulation.md).

컨트롤러가 실물 서보 대신 TCP로 붙는다. 서보 거래(핑, 읽기, 동기 읽기, 쓰기, 동기 쓰기)를 JSON 한 줄씩 받고,
동기 읽기마다 MuJoCo 물리를 한 주기만큼 진행한 뒤 상태로 답한다 (한 걸음씩).

실행: uv run python -m horilab_sim [--port 5600] [--realtime]
"""
