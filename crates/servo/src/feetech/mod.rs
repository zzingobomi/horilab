//! Feetech STS 계열 서보(STS3215, STS3250, ST-3120): 메모리 표, 서보 버스, 시험용 가짜 버스.
//!
//! 설계는 docs/decisions/0024-servo-bus.md이고, Python 기준 구현(packages/servo)과 같은 시험 항목을 통과한다.

pub mod bus;
pub mod fake;
mod packet;
pub mod registers;

pub use bus::{Bus, Error, LinkStats, Port, Reply, ScanResult, SerialLink, SyncRead};
pub use registers::{Area, Encoding, Fault, Register};
