//! 로봇 컨트롤러 (docs/decisions/0023-robot-controller.md, 0024-servo-bus.md).
//!
//! 로봇 설정(settings), 서보 준비의 값 적용(prepare), 기록(record), 관절 층과 안전 검사(joints), 맨 서보
//! 확인(check)이 있다. 서보를 움직이는 명령은 joints만 낸다 (0009의 한곳 안전 검사). prepare는 EEPROM만
//! 쓰고 서보를 움직이지 않는다.

pub mod check;
pub mod joints;
pub mod kinematics;
pub mod plan;
pub mod prepare;
pub mod record;
pub mod runloop;
pub mod settings;
