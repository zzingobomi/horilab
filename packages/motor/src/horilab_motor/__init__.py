"""모터 프로세스와 이야기하는 Python 쪽.

모터 프로세스(실행 루프, 안전 검사, 서보 버스)는 Rust(crates/motor)이고 컨트롤러 파이에서 돈다
(docs/decisions/0025-controller-on-pi.md). 메시지 코드는 전송 방식을 정한 뒤에 들어온다 (0023).
지금은 서보 버스의 Python 기준 구현(feetech)만 있다 (docs/decisions/0024-servo-bus.md).
가벼운 의존성만 쓴다.
"""
