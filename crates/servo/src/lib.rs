//! 서보 버스: 서보와 이야기하는 맨 아래층 (docs/decisions/0024-servo-bus.md).
//!
//! 서보에 직접 닿는 코드는 이 크레이트에만 둔다 (docs/decisions/0007-repo-structure.md).
//! Python 기준 구현(packages/servo)과 같은 설계이고, 실물에서 두 구현의 결과를 맞춰 본다.

pub mod feetech;

/// 이 컴퓨터에서 보이는 시리얼 포트 이름. 서보 어댑터를 찾을 때 쓴다.
pub fn port_names() -> Result<Vec<String>, serialport::Error> {
    Ok(serialport::available_ports()?
        .into_iter()
        .map(|port| port.port_name)
        .collect())
}

#[cfg(test)]
mod tests {
    #[test]
    fn port_names_can_be_listed() {
        // 어댑터가 없는 PC에서도 목록을 읽는 것 자체는 실패하지 않아야 한다.
        assert!(super::port_names().is_ok());
    }
}
