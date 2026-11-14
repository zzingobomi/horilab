//! 영점 재기 (docs/decisions/0024-servo-bus.md의 영점, 0034).
//!
//! 영점 핀을 꽂고 핀 구멍의 틈을 + 쪽(관절 각이 느는 쪽)으로 밀어 붙인 채, 토크를 끈 서보의 값을 읽는다. 범위
//! 가운데가 서보 값 2048 둘레에 오도록(소프트 한계에 경계 여유를 더한 범위가 0~4095 안에 들도록) 위치 오프셋을
//! 계산해 EEPROM 쓰기로 굽고, 다시 읽은 핀 자리의 서보 값을 영점으로 쓴다. 방향은 핀을 꽂기 전에 관절을 손으로
//! + 쪽으로 돌려 서보 값이 느는지 주는지로 정한다. 서보는 움직이지 않는다 (토크가 꺼져 있어야 굽는다).

use horilab_servo::feetech::registers::{self as reg, STEPS_PER_TURN};
use horilab_servo::feetech::{Bus, Port};

use crate::prepare::value_in;
use crate::settings::{Direction, Joint};

const TURN: i32 = STEPS_PER_TURN as i32;
/// 방향을 정하려고 관절을 돌릴 때 이만큼 [칸] 넘게 움직여야 한다 (약 9°). 손이 흔들린 것과 가린다
pub const MIN_TURN_STEPS: i32 = 100;
/// 영점을 읽을 때 몇 번 읽어 가운데 값을 쓰나
pub const READS: usize = 5;
/// 구운 뒤 다시 읽은 영점이 목표에서 이만큼 [칸]까지 벗어나도 된다 (읽기의 흔들림)
pub const ZERO_TOLERANCE: i32 = 1;

/// 두 서보 값 사이의 가장 짧은 차이 [칸] (-2048 ~ 2047). 0/4095 경계를 넘어 돌아도 맞다.
pub fn turn_between(before: i32, after: i32) -> i32 {
    let d = (after - before).rem_euclid(TURN);
    if d >= TURN / 2 { d - TURN } else { d }
}

/// 관절을 + 쪽으로 돌리기 전과 뒤의 서보 값으로 방향을 정한다.
pub fn direction_from(before: i32, after: i32) -> Result<Direction, String> {
    let d = turn_between(before, after);
    if d.abs() < MIN_TURN_STEPS {
        return Err(format!(
            "관절이 {d}칸만 움직였다. + 쪽으로 더 크게({MIN_TURN_STEPS}칸, 약 9° 넘게) 돌린다"
        ));
    }
    Ok(if d > 0 {
        Direction::Same
    } else {
        Direction::Reversed
    })
}

/// 굽기 전에 계산한 것.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZeroPlan {
    /// 핀 자리에서 읽은 서보 값 (지금 오프셋으로)
    pub reading: i32,
    /// 지금 서보에 구워진 위치 오프셋
    pub offset_now: i32,
    /// 엔코더 값 (읽는 위치 = 엔코더 값 - 오프셋)
    pub encoder: i32,
    /// 핀 자리가 와야 할 서보 값 (영점이 들어야 할 범위의 가운데)
    pub target: i32,
    /// 구울 위치 오프셋
    pub offset_new: i32,
}

/// 핀 자리의 서보 값을 영점이 들어야 할 범위의 가운데에 맞추는 위치 오프셋을 계산한다.
pub fn plan(
    joint: &Joint,
    clearance: f64,
    direction: Direction,
    reading: i32,
    offset_now: i32,
) -> Result<ZeroPlan, String> {
    let (low, high) = joint.zero_window(clearance, direction).ok_or_else(|| {
        format!(
            "{}: 소프트 한계에 경계 여유를 더한 범위가 한 바퀴보다 넓다",
            joint.name
        )
    })?;
    let target = ((low + high) / 2.0).round() as i32;
    let encoder = (reading + offset_now).rem_euclid(TURN);
    let offset_new = turn_between(target, encoder);
    reg::POSITION_OFFSET.encode(offset_new).map_err(|e| {
        format!(
            "{}: 위치 오프셋 {offset_new}을 담을 수 없다: {e}",
            joint.name
        )
    })?;
    Ok(ZeroPlan {
        reading,
        offset_now,
        encoder,
        target,
        offset_new,
    })
}

/// 서보 값을 몇 번 읽어 가운데 값을 낸다 (핀에 붙여 둔 관절이 손 떨림에 한두 칸 흔들린다).
pub fn read_position<P: Port>(bus: &mut Bus<P>, id: u8, reads: usize) -> Result<i32, String> {
    let mut values = Vec::with_capacity(reads);
    for _ in 0..reads.max(1) {
        let reply = bus
            .read(id, &[reg::PRESENT_POSITION])
            .map_err(|e| format!("서보 {id}의 위치를 읽지 못했다: {e}"))?;
        let value = reply
            .get(reg::PRESENT_POSITION)
            .ok_or_else(|| format!("서보 {id}의 위치가 답에 없다"))?;
        values.push(value);
    }
    values.sort_unstable();
    Ok(values[values.len() / 2])
}

/// 덤프에서 토크가 꺼져 있는지와 지금 위치 오프셋을 읽는다.
pub fn state_in(memory: &[u8]) -> Result<(bool, i32), String> {
    let torque = value_in(memory, reg::TORQUE_ENABLE).ok_or("덤프에 토크 스위치가 없다")?;
    let offset = value_in(memory, reg::POSITION_OFFSET).ok_or("덤프에 위치 오프셋이 없다")?;
    Ok((torque == reg::TORQUE_OFF, offset))
}

/// 위치 오프셋을 굽고(토크가 꺼져 있어야 한다), 쓴 뒤의 덤프로 확인한다. 쓴 뒤의 덤프를 돌려준다.
pub fn burn_offset<P: Port>(
    bus: &mut Bus<P>,
    joint: &Joint,
    offset: i32,
) -> Result<Vec<u8>, String> {
    bus.write_eeprom(joint.id, &[(reg::POSITION_OFFSET, offset)])
        .map_err(|e| format!("{}: 위치 오프셋을 굽지 못했다: {e}", joint.name))?;
    let after = bus
        .dump(joint.id)
        .map_err(|e| format!("{}: 구운 뒤 덤프를 읽지 못했다: {e}", joint.name))?;
    match value_in(&after, reg::POSITION_OFFSET) {
        Some(value) if value == offset => Ok(after),
        other => Err(format!(
            "{}: 구운 뒤 읽은 위치 오프셋이 {other:?}다 ({offset}을 구웠다)",
            joint.name
        )),
    }
}

/// 구운 뒤 다시 읽은 영점이 목표에 들었는지 본다.
pub fn check_zero(plan: &ZeroPlan, zero_raw: i32) -> Result<(), String> {
    let off = turn_between(plan.target, zero_raw);
    if off.abs() > ZERO_TOLERANCE {
        return Err(format!(
            "구운 뒤 읽은 핀 자리가 {zero_raw}로 목표 {}에서 {off}칸 벗어났다. 핀이 빠지지 않았는지, 틈을 + 쪽으로 붙였는지 \
             보고 다시 잰다",
            plan.target
        ));
    }
    Ok(())
}

/// 로봇 설정에 적을 세 줄.
pub fn settings_lines(zero_raw: i32, direction: Direction, offset: i32) -> String {
    format!(
        "position_offset = {offset}\nzero_raw = {zero_raw}\ndirection = {}\n",
        direction.number()
    )
}

#[cfg(test)]
mod tests {
    use horilab_servo::feetech::fake::{FakeServo, open_fake_bus};

    use super::*;
    use crate::settings::{Settings, TEST_J3};

    fn j3() -> (Joint, f64) {
        let settings = Settings::parse(TEST_J3).unwrap();
        (
            settings.joint("j3").unwrap().clone(),
            settings.boundary_clearance,
        )
    }

    #[test]
    fn shortest_turn_crosses_the_boundary() {
        assert_eq!(turn_between(4000, 100), 196);
        assert_eq!(turn_between(100, 4000), -196);
        assert_eq!(turn_between(1000, 1500), 500);
        assert_eq!(turn_between(1500, 1000), -500);
    }

    #[test]
    fn direction_follows_the_sign_of_the_turn() {
        assert_eq!(direction_from(1000, 1300).unwrap(), Direction::Same);
        assert_eq!(direction_from(1000, 700).unwrap(), Direction::Reversed);
        assert_eq!(direction_from(4050, 200).unwrap(), Direction::Same); // 경계를 넘어 돌았다
        assert!(direction_from(1000, 1050).unwrap_err().contains("50칸만"));
    }

    #[test]
    fn offset_puts_the_pin_in_the_middle_of_the_window() {
        let (joint, clearance) = j3();
        for direction in [Direction::Same, Direction::Reversed] {
            let (low, high) = joint.zero_window(clearance, direction).unwrap();
            let middle = ((low + high) / 2.0).round() as i32;
            for (reading, offset_now) in [(3500, 0), (100, 85), (2048, -300), (4095, 2000)] {
                let p = plan(&joint, clearance, direction, reading, offset_now).unwrap();
                assert_eq!(p.target, middle);
                assert!((-2048..2048).contains(&p.offset_new), "{p:?}");
                // 구운 뒤 읽는 위치 = 엔코더 값 - 새 오프셋 (한 바퀴로 돌아온다)
                assert_eq!((p.encoder - p.offset_new).rem_euclid(TURN), middle, "{p:?}");
            }
        }
    }

    #[test]
    fn burning_moves_the_reading_to_the_target() {
        let (joint, clearance) = j3();
        // 핀 자리에서 3600으로 읽히는 서보 (오프셋 85가 구워져 있다)
        let mut servo = FakeServo::with(3, 2825, 3600);
        servo.set(reg::POSITION_OFFSET, 85);
        servo.flash[usize::from(reg::POSITION_OFFSET.address)..][..2]
            .copy_from_slice(&servo.memory[usize::from(reg::POSITION_OFFSET.address)..][..2]);
        let mut bus = open_fake_bus([servo]);
        let before = bus.dump(3).unwrap();
        let (torque_off, offset_now) = state_in(&before).unwrap();
        assert!(torque_off);
        assert_eq!(offset_now, 85);
        let reading = read_position(&mut bus, 3, READS).unwrap();
        let p = plan(&joint, clearance, Direction::Same, reading, offset_now).unwrap();
        burn_offset(&mut bus, &joint, p.offset_new).unwrap();
        let zero_raw = read_position(&mut bus, 3, READS).unwrap();
        assert_eq!(zero_raw, p.target);
        check_zero(&p, zero_raw).unwrap();
        // 전원을 다시 켜도 남는다
        bus.wire().servos[0].power_cycle();
        assert_eq!(state_in(&bus.dump(3).unwrap()).unwrap().1, p.offset_new);
        assert!(check_zero(&p, p.target + 3).is_err());
    }

    #[test]
    fn nothing_is_burned_while_torque_is_on() {
        let (joint, _) = j3();
        let mut servo = FakeServo::new(3);
        servo.set(reg::TORQUE_ENABLE, reg::TORQUE_ON);
        let mut bus = open_fake_bus([servo]);
        assert!(burn_offset(&mut bus, &joint, 100).is_err());
        assert_eq!(bus.wire().servos[0].eeprom_saves, 0);
    }

    #[test]
    fn settings_lines_are_what_robot_toml_reads() {
        assert_eq!(
            settings_lines(2048, Direction::Reversed, -312),
            "position_offset = -312\nzero_raw = 2048\ndirection = -1\n"
        );
    }
}
