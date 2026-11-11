"""서보 버스의 Python 기준 구현 (docs/decisions/0024-servo-bus.md).

컨트롤러와 그 서보 버스는 Rust(crates/controller, crates/servo)이고 컨트롤러 파이에서 돈다
(docs/decisions/0025-controller-on-pi.md). 이 패키지는 서보에 직접 명령하므로 응용은 불러오지 않는다.
응용은 컨트롤러 API의 클라이언트를 쓴다 (docs/decisions/0007-repo-structure.md). 가벼운 의존성만 쓴다.
"""
