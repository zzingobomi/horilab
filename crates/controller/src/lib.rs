//! 로봇 컨트롤러 (docs/decisions/0023-robot-controller.md, 0024-servo-bus.md).
//!
//! 지금은 로봇 설정(settings), 서보 준비의 값 적용(prepare), 기록(record)이 있다. 0024의 만드는 순서대로
//! joints(관절 층과 안전 검사)와 관절 시험 명령을 더한다. 서보를 움직이는 명령은 joints만 낸다 (0009의
//! 한곳 안전 검사). prepare는 EEPROM만 쓰고 서보를 움직이지 않는다.

pub mod prepare;
pub mod record;
pub mod settings;
