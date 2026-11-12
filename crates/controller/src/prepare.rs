//! 서보 준비: 값 적용 (docs/decisions/0024-servo-bus.md의 만드는 순서 3).
//!
//! 서보에 남은 값에 기대지 않고, 로봇 설정에서 계산한 EEPROM 값([`Joint::eeprom`])을 모두 맞춘다. 다른 칸만
//! 쓴다 (EEPROM은 쓰는 횟수에 수명이 있다). 서보를 움직이지 않는다: 쓰는 것은 EEPROM뿐이고, 토크가 켜져
//! 있으면 버스가 쓰지 않는다. 쓰기 전후의 덤프를 기록에 남기는 것은 부르는 쪽이 한다.

use horilab_servo::feetech::registers::{MODELS, Register};
use horilab_servo::feetech::{Bus, Port};

use crate::settings::Joint;

/// 바꿀 칸 하나.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Change {
    pub register: Register,
    pub current: i32,
    pub target: i32,
}

impl Change {
    /// 사람이 볼 한 줄: `p_gain 32 → 40 (-)`
    pub fn describe(&self) -> String {
        format!(
            "{} {} → {} ({})",
            self.register.name, self.current, self.target, self.register.unit
        )
    }
}

/// 덤프(주소 0부터의 메모리)에서 레지스터 값을 푼다 (부호도 푼다).
pub fn value_in(memory: &[u8], register: Register) -> Option<i32> {
    let address = usize::from(register.address);
    let low = *memory.get(address)?;
    let high = if register.size == 2 {
        *memory.get(address + 1)?
    } else {
        0
    };
    Some(register.decode(u16::from_le_bytes([low, high])))
}

/// 목표와 다른 칸.
pub fn plan(target: &[(Register, i32)], memory: &[u8]) -> Result<Vec<Change>, String> {
    let mut changes = Vec::new();
    for &(register, target) in target {
        let current = value_in(memory, register)
            .ok_or_else(|| format!("덤프가 짧아 {}을 읽지 못했다", register.name))?;
        if current != target {
            changes.push(Change {
                register,
                current,
                target,
            });
        }
    }
    Ok(changes)
}

/// 모델 번호의 이름. 모르는 번호는 번호 그대로.
pub fn model_name(model: u16) -> String {
    MODELS
        .iter()
        .find(|(number, _)| *number == model)
        .map_or_else(|| model.to_string(), |(_, name)| (*name).to_owned())
}

/// 버스에서 관절의 서보를 찾아 덤프를 읽는다. 서보가 응답하지 않거나 모델이 다르면 오류다.
pub fn inspect<P: Port>(bus: &mut Bus<P>, joint: &Joint) -> Result<Vec<u8>, String> {
    let model = bus.ping(joint.id).map_err(|e| {
        format!(
            "{}: ID {} 서보가 응답하지 않는다: {e}",
            joint.name, joint.id
        )
    })?;
    if model != joint.model_number {
        return Err(format!(
            "{}: ID {}는 {}가 아니라 {}다",
            joint.name,
            joint.id,
            joint.model,
            model_name(model)
        ));
    }
    bus.dump(joint.id)
        .map_err(|e| format!("{}: 덤프를 읽지 못했다: {e}", joint.name))
}

/// 바꿀 칸을 쓰고, 다시 읽어 목표와 모두 같은지 본다. 쓴 뒤의 덤프를 돌려준다.
pub fn write<P: Port>(
    bus: &mut Bus<P>,
    joint: &Joint,
    changes: &[Change],
) -> Result<Vec<u8>, String> {
    let values: Vec<(Register, i32)> = changes
        .iter()
        .map(|change| (change.register, change.target))
        .collect();
    bus.write_eeprom(joint.id, &values)
        .map_err(|e| format!("{}: 쓰지 못했다: {e}", joint.name))?;
    let after = bus
        .dump(joint.id)
        .map_err(|e| format!("{}: 쓴 뒤 덤프를 읽지 못했다: {e}", joint.name))?;
    let left = plan(&joint.eeprom, &after)?;
    if !left.is_empty() {
        let names: Vec<&str> = left.iter().map(|change| change.register.name).collect();
        return Err(format!(
            "{}: 쓴 뒤에도 목표와 다른 칸이 있다: {}",
            joint.name,
            names.join(", ")
        ));
    }
    Ok(after)
}

#[cfg(test)]
mod tests {
    use horilab_servo::feetech::fake::{FakeServo, open_fake_bus};
    use horilab_servo::feetech::registers as reg;

    use super::*;
    use crate::settings::{Settings, TEST_J3};

    fn j3() -> Joint {
        Settings::parse(TEST_J3)
            .unwrap()
            .joint("j3")
            .unwrap()
            .clone()
    }

    #[test]
    fn apply_writes_only_differences_and_they_stay() {
        let joint = j3();
        let mut bus = open_fake_bus([FakeServo::new(3)]);
        let before = inspect(&mut bus, &joint).unwrap();
        let changes = plan(&joint.eeprom, &before).unwrap();
        let names: Vec<&str> = changes.iter().map(|change| change.register.name).collect();
        // 가짜 서보의 기본값(시험용)과 STS3250 첫 덤프 값이 다른 칸만 쓴다
        for expected in [
            "max_voltage",
            "unload_condition",
            "phase",
            "protection_current",
        ] {
            assert!(names.contains(&expected), "{names:?}");
        }
        assert!(!names.contains(&"p_gain"), "{names:?}"); // 둘 다 32
        let after = write(&mut bus, &joint, &changes).unwrap();
        assert!(plan(&joint.eeprom, &after).unwrap().is_empty());
        // 전원을 다시 켜도 남는다 (잠금을 풀고 썼다)
        bus.wire().servos[0].power_cycle();
        let again = inspect(&mut bus, &joint).unwrap();
        assert!(plan(&joint.eeprom, &again).unwrap().is_empty());
    }

    #[test]
    fn wrong_model_or_missing_servo_is_refused() {
        let joint = j3();
        let mut bus = open_fake_bus([FakeServo::with(3, 777, 2048)]);
        let error = inspect(&mut bus, &joint).unwrap_err();
        assert!(error.contains("STS3250가 아니라 STS3215"), "{error}");
        let mut bus = open_fake_bus([FakeServo::new(4)]);
        let error = inspect(&mut bus, &joint).unwrap_err();
        assert!(error.contains("응답하지 않는다"), "{error}");
    }

    #[test]
    fn nothing_is_written_while_torque_is_on() {
        let joint = j3();
        let mut servo = FakeServo::new(3);
        servo.set(reg::TORQUE_ENABLE, reg::TORQUE_ON);
        let mut bus = open_fake_bus([servo]);
        let before = inspect(&mut bus, &joint).unwrap();
        let changes = plan(&joint.eeprom, &before).unwrap();
        assert!(write(&mut bus, &joint, &changes).is_err());
        assert_eq!(bus.wire().servos[0].eeprom_saves, 0);
    }

    #[test]
    fn values_are_read_from_a_dump() {
        let servo = FakeServo::with(3, 2825, 3093);
        let memory = servo.read(0, reg::MEMORY_SIZE as u8);
        assert_eq!(value_in(&memory, reg::PRESENT_POSITION), Some(3093));
        assert_eq!(value_in(&memory, reg::MODEL), Some(2825));
        assert_eq!(value_in(&memory[..10], reg::MODEL), Some(2825));
        assert_eq!(value_in(&memory[..10], reg::PRESENT_POSITION), None);
        assert!(plan(&[(reg::PRESENT_POSITION, 1)], &memory[..10]).is_err());
    }
}
