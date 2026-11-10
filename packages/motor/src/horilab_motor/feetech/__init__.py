"""Feetech STS 계열 서보의 Python 기준 구현: 메모리 표, 서보 버스, 시험용 가짜 버스.

모터 프로세스는 Rust(crates/motor)다 (docs/decisions/0025-controller-on-pi.md). 이 구현은 같은 설계
(docs/decisions/0024-servo-bus.md)를 공식 SDK 위에 먼저 짠 것이다. 실물에서 첫 덤프를 뜨고 Rust 버스와
결과를 맞춰 보는 데 쓰고, Rust 버스가 실물에서 확인되면 지운다.
"""
