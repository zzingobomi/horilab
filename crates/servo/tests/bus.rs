//! 서보 버스 검사. 가짜 서보를 포트 자리에 끼워 패킷 처리까지 지나간다.
//!
//! Python 기준 구현의 시험(packages/servo/tests/test_bus.py)과 같은 항목이다. 0009의 "실물 백엔드의
//! 오류 처리를 가짜 서보 버스로 시험한다"의 맨 아래층 몫이다 (응답 없음, 깨진 응답, 오류 비트, 포트 오류).

use std::collections::BTreeMap;

use horilab_servo::feetech::fake::{FakeServo, open_fake_bus};
use horilab_servo::feetech::registers as reg;
use horilab_servo::feetech::{Error, Fault};

#[test]
fn ping_returns_model() {
    let mut bus = open_fake_bus([FakeServo::with(3, 2825, 2048)]);
    assert_eq!(bus.ping(3).unwrap(), 2825);
}

#[test]
fn ping_missing_servo_is_no_reply() {
    let mut bus = open_fake_bus([FakeServo::new(3)]);
    assert!(matches!(bus.ping(4), Err(Error::NoReply { id: 4, .. })));
}

#[test]
fn rejects_ids_outside_range() {
    let mut bus = open_fake_bus([FakeServo::new(3)]);
    for id in [253, 254, 255] {
        assert!(matches!(bus.ping(id), Err(Error::Invalid(_))), "{id}");
    }
}

#[test]
fn scan_finds_servos_and_flags_shared_id() {
    let servos = [
        FakeServo::with(1, 777, 2048),
        FakeServo::with(3, 2825, 2048),
        FakeServo::new(5),
        FakeServo::new(5),
    ];
    let mut bus = open_fake_bus(servos);
    let result = bus.scan(0..8).unwrap();
    assert_eq!(result.models, BTreeMap::from([(1, 777), (3, 2825)]));
    assert_eq!(result.garbled, vec![5]);
    // 없는 ID의 성적은 남기지 않는다
    assert_eq!(
        bus.stats().keys().copied().collect::<Vec<_>>(),
        vec![1, 3, 5]
    );
}

#[test]
fn read_decodes_signed_values() {
    let mut servo = FakeServo::new(3);
    servo.set(reg::PRESENT_POSITION, 1000);
    servo.set(reg::PRESENT_LOAD, -250);
    servo.set(reg::PRESENT_TEMPERATURE, 31);
    let mut bus = open_fake_bus([servo]);
    let registers = [
        reg::PRESENT_POSITION,
        reg::PRESENT_LOAD,
        reg::PRESENT_TEMPERATURE,
    ];
    let reply = bus.read(3, &registers).unwrap();
    assert_eq!(
        reply.values,
        BTreeMap::from([
            ("present_position", 1000),
            ("present_load", -250),
            ("present_temperature", 31),
        ])
    );
    assert_eq!(reply.fault, Fault::NONE);
}

#[test]
fn reply_carries_status_bits() {
    let mut servo = FakeServo::new(3);
    servo.faults.status = Fault::TEMPERATURE | Fault::OVERLOAD;
    let mut bus = open_fake_bus([servo]);
    let reply = bus.read(3, &[reg::PRESENT_TEMPERATURE]).unwrap();
    assert_eq!(reply.fault, Fault::TEMPERATURE | Fault::OVERLOAD);
}

#[test]
fn dump_reads_whole_memory_table() {
    let mut bus = open_fake_bus([FakeServo::new(3)]);
    let memory = bus.wire().servos[0].memory.to_vec();
    assert_eq!(bus.dump(3).unwrap(), memory);
}

#[test]
fn write_ram_register() {
    let mut bus = open_fake_bus([FakeServo::new(3)]);
    bus.write(3, reg::TORQUE_LIMIT, 300).unwrap();
    assert_eq!(bus.wire().servos[0].get(reg::TORQUE_LIMIT), 300);
}

#[test]
fn write_refuses() {
    let cases = [
        (reg::P_GAIN, 32),          // EEPROM은 write_eeprom이나 write_trial로
        (reg::PRESENT_POSITION, 0), // 읽기 전용
        (reg::LOCK, 0),             // 잠금은 EEPROM 절차가 다룬다
        (reg::TORQUE_ENABLE, 128),  // 위치 오프셋을 바꾸는 값
    ];
    for (register, value) in cases {
        let mut bus = open_fake_bus([FakeServo::new(3)]);
        let result = bus.write(3, register, value);
        assert!(
            matches!(result, Err(Error::Invalid(_))),
            "{}",
            register.name
        );
    }
}

#[test]
fn eeprom_written_while_locked_is_lost_at_power_off() {
    // 메모리 표의 잠금 뜻을 가짜 서보가 따르는지. 아래 시험들이 이것에 기댄다.
    let mut servo = FakeServo::new(3);
    servo.write(reg::P_GAIN.address, &[16]);
    assert_eq!(servo.get(reg::P_GAIN), 16);
    servo.power_cycle();
    assert_eq!(servo.get(reg::P_GAIN), 32);
}

#[test]
fn write_eeprom_persists_and_relocks() {
    let mut servo = FakeServo::new(3);
    servo.set(reg::P_GAIN, 16);
    let mut bus = open_fake_bus([servo]);
    let values = [
        (reg::P_GAIN, 32),
        (reg::DEAD_ZONE_POSITIVE, 0),
        (reg::DEAD_ZONE_NEGATIVE, 0),
    ];
    bus.write_eeprom(3, &values).unwrap();
    let servo = &mut bus.wire().servos[0];
    assert_eq!(servo.get(reg::LOCK), 1);
    servo.power_cycle();
    for (register, value) in values {
        assert_eq!(servo.get(register), value, "{}", register.name);
    }
}

#[test]
fn write_eeprom_refuses_while_torque_on() {
    let mut bus = open_fake_bus([FakeServo::new(3)]);
    bus.write(3, reg::TORQUE_ENABLE, reg::TORQUE_ON).unwrap();
    let result = bus.write_eeprom(3, &[(reg::P_GAIN, 32)]);
    assert!(matches!(result, Err(Error::Refused(_))));
    assert_eq!(bus.wire().servos[0].eeprom_saves, 0);
}

#[test]
fn write_eeprom_rejects_registers() {
    for register in [reg::ID, reg::BAUD_RATE, reg::MODEL, reg::TORQUE_LIMIT] {
        let mut bus = open_fake_bus([FakeServo::new(3)]);
        let result = bus.write_eeprom(3, &[(register, 1)]);
        assert!(
            matches!(result, Err(Error::Invalid(_))),
            "{}",
            register.name
        );
    }
}

#[test]
fn write_eeprom_relocks_when_a_write_fails() {
    let mut servo = FakeServo::new(3);
    servo.faults.silent_writes_to.insert(reg::D_GAIN.address);
    let mut bus = open_fake_bus([servo]);
    let result = bus.write_eeprom(3, &[(reg::P_GAIN, 20), (reg::D_GAIN, 20)]);
    assert!(matches!(result, Err(Error::NoReply { .. })));
    assert_eq!(bus.wire().servos[0].get(reg::LOCK), 1);
}

#[test]
fn write_eeprom_detects_value_that_did_not_stick() {
    let mut servo = FakeServo::new(3);
    servo
        .faults
        .ignored_writes_to
        .insert(reg::MAX_TEMPERATURE.address);
    let mut bus = open_fake_bus([servo]);
    let result = bus.write_eeprom(3, &[(reg::MAX_TEMPERATURE, 60)]);
    assert!(matches!(result, Err(Error::Verify { .. })));
}

#[test]
fn write_trial_applies_now_and_is_lost_at_power_off() {
    let mut bus = open_fake_bus([FakeServo::new(3)]);
    bus.write(3, reg::TORQUE_ENABLE, reg::TORQUE_ON).unwrap(); // 토크가 켜져 있어도 된다
    bus.write_trial(3, &[(reg::P_GAIN, 16), (reg::RESPONSE_DELAY, 5)])
        .unwrap();
    let servo = &mut bus.wire().servos[0];
    assert_eq!(
        (servo.get(reg::P_GAIN), servo.get(reg::RESPONSE_DELAY)),
        (16, 5)
    );
    assert_eq!(servo.eeprom_saves, 0);
    servo.power_cycle();
    assert_eq!(
        (servo.get(reg::P_GAIN), servo.get(reg::RESPONSE_DELAY)),
        (32, 0)
    );
}

#[test]
fn write_trial_locks_first_so_values_never_persist() {
    let mut servo = FakeServo::new(3);
    servo.set(reg::LOCK, 0); // 앞선 작업이 잠금을 풀어 둔 채 끝난 경우
    let mut bus = open_fake_bus([servo]);
    bus.write_trial(3, &[(reg::P_GAIN, 16)]).unwrap();
    let servo = &bus.wire().servos[0];
    assert_eq!(servo.get(reg::LOCK), 1);
    assert_eq!(servo.eeprom_saves, 0);
}

#[test]
fn write_trial_rejects_other_registers() {
    for register in [reg::MAX_TEMPERATURE, reg::ID, reg::TORQUE_LIMIT, reg::LOCK] {
        let mut bus = open_fake_bus([FakeServo::new(3)]);
        let result = bus.write_trial(3, &[(register, 1)]);
        assert!(
            matches!(result, Err(Error::Invalid(_))),
            "{}",
            register.name
        );
    }
}

#[test]
fn change_id_persists() {
    let mut bus = open_fake_bus([FakeServo::new(1)]);
    bus.change_id(1, 3).unwrap();
    let servo = &mut bus.wire().servos[0];
    assert_eq!(servo.id(), 3);
    assert_eq!(servo.get(reg::LOCK), 1);
    servo.power_cycle();
    assert_eq!(servo.id(), 3);
    assert!(matches!(bus.ping(1), Err(Error::NoReply { .. })));
}

#[test]
fn change_id_refuses_id_in_use() {
    let mut bus = open_fake_bus([FakeServo::new(1), FakeServo::new(3)]);
    assert!(matches!(bus.change_id(1, 3), Err(Error::Refused(_))));
    assert_eq!(bus.wire().servos[0].id(), 1);
}

#[test]
fn sync_read_returns_each_servo() {
    let mut bus = open_fake_bus([FakeServo::with(1, 2825, 100), FakeServo::with(3, 2825, 200)]);
    let results = bus.sync_read(&[1, 3], reg::STATE).unwrap();
    let positions: BTreeMap<u8, Option<i32>> = results
        .into_iter()
        .map(|(id, result)| (id, result.unwrap().get(reg::PRESENT_POSITION)))
        .collect();
    assert_eq!(positions, BTreeMap::from([(1, Some(100)), (3, Some(200))]));
}

#[test]
fn sync_read_never_returns_stale_values() {
    // 공식 SDK의 GroupSyncRead는 응답이 한 패킷보다 짧으면 지난 주기 값을 그대로 돌려준다.
    let mut bus = open_fake_bus([FakeServo::with(3, 2825, 500)]);
    assert!(bus.sync_read(&[3], &[reg::PRESENT_POSITION]).unwrap()[&3].is_ok());
    bus.wire().servos[0].faults.silent = true;
    let results = bus.sync_read(&[3], &[reg::PRESENT_POSITION]).unwrap();
    assert!(matches!(results[&3], Err(Error::NoReply { .. })));
    assert_eq!(bus.stats()[&3].consecutive_failures, 1);
}

#[test]
fn sync_read_keeps_others_when_one_is_silent() {
    let mut silent = FakeServo::new(1);
    silent.faults.silent = true;
    let mut bus = open_fake_bus([silent, FakeServo::new(3)]);
    let results = bus.sync_read(&[1, 3], &[reg::PRESENT_POSITION]).unwrap();
    assert!(matches!(results[&1], Err(Error::NoReply { .. })));
    assert!(results[&3].is_ok());
}

#[test]
fn sync_read_reports_broken_reply() {
    for corrupt in [true, false] {
        for broken_first in [true, false] {
            let mut broken = FakeServo::new(1);
            if corrupt {
                broken.faults.corrupt = true;
            } else {
                broken.faults.truncate = true;
            }
            let mut bus = open_fake_bus([broken, FakeServo::new(3)]);
            let order: &[u8] = if broken_first { &[1, 3] } else { &[3, 1] };
            let results = bus.sync_read(order, &[reg::PRESENT_POSITION]).unwrap();
            let case = format!("corrupt={corrupt} broken_first={broken_first}");
            assert!(matches!(results[&1], Err(Error::BadReply { .. })), "{case}");
            assert!(results[&3].is_ok(), "{case}");
        }
    }
}

#[test]
fn sync_read_rejects_repeated_ids() {
    let mut bus = open_fake_bus([FakeServo::new(1)]);
    let result = bus.sync_read(&[1, 1], &[reg::PRESENT_POSITION]);
    assert!(matches!(result, Err(Error::Invalid(_))));
}

#[test]
fn stats_count_failures_and_reset_on_success() {
    let mut servo = FakeServo::new(3);
    servo.faults.silent = true;
    let mut bus = open_fake_bus([servo]);
    for _ in 0..3 {
        let result = bus.read(3, &[reg::PRESENT_POSITION]);
        assert!(matches!(result, Err(Error::NoReply { .. })));
    }
    let stats = bus.stats()[&3];
    assert_eq!((stats.no_reply, stats.consecutive_failures), (3, 3));
    bus.wire().servos[0].faults.silent = false;
    bus.read(3, &[reg::PRESENT_POSITION]).unwrap();
    let stats = bus.stats()[&3];
    assert_eq!((stats.ok, stats.consecutive_failures), (1, 0));
}

#[test]
fn port_recovers_after_error_mid_transaction() {
    // 공식 SDK만 쓰면 이때 '사용 중' 표시가 남아 그 뒤 모든 통신이 막힌다.
    let mut bus = open_fake_bus([FakeServo::new(3)]);
    bus.wire().fail_next_send = true;
    let result = bus.read(3, &[reg::PRESENT_POSITION]);
    assert!(matches!(result, Err(Error::Port { .. })));
    let reply = bus.read(3, &[reg::PRESENT_POSITION]).unwrap();
    assert_eq!(reply.get(reg::PRESENT_POSITION), Some(2048));
}

#[test]
fn sync_write_sets_each_servo() {
    let mut bus = open_fake_bus([FakeServo::new(1), FakeServo::new(3)]);
    let on = [(1, reg::TORQUE_ON), (3, reg::TORQUE_ON)];
    bus.sync_write(reg::TORQUE_ENABLE, &on).unwrap();
    bus.sync_write(reg::GOAL_POSITION, &[(1, 1000), (3, 3000)])
        .unwrap();
    let servos = &bus.wire().servos;
    let positions = (
        servos[0].get(reg::PRESENT_POSITION),
        servos[1].get(reg::PRESENT_POSITION),
    );
    assert_eq!(positions, (1000, 3000));
}

#[test]
fn sync_write_refuses() {
    for (register, value) in [(reg::P_GAIN, 32), (reg::TORQUE_ENABLE, 128)] {
        let mut bus = open_fake_bus([FakeServo::new(1)]);
        let result = bus.sync_write(register, &[(1, value)]);
        assert!(
            matches!(result, Err(Error::Invalid(_))),
            "{}",
            register.name
        );
    }
}
