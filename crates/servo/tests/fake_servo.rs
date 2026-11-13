//! 가짜 서보가 실물처럼 움직이는지 본다 (맨 서보 확인, 2026-11-13, docs/decisions/0024-servo-bus.md).
//! 관절 층과 실행 루프의 실물 전 시험이 가짜 서보에 기대므로, 실물과 다르면 그 시험을 믿을 수 없다.

use horilab_servo::feetech::Fault;
use horilab_servo::feetech::fake::FakeServo;
use horilab_servo::feetech::registers::{self as reg, Register};

fn write(servo: &mut FakeServo, register: Register, value: i32) {
    let bytes = register.encode(value).unwrap().to_le_bytes();
    servo.write(register.address, &bytes[..usize::from(register.size)]);
}

/// 시작 절차(목표를 지금 위치로 쓰고 토크 켜기)를 거친 서보
fn enabled(position: i32) -> FakeServo {
    let mut servo = FakeServo::with(20, 777, position);
    write(&mut servo, reg::GOAL_POSITION, position);
    write(&mut servo, reg::TORQUE_ENABLE, reg::TORQUE_ON);
    servo
}

#[test]
fn goals_stop_at_the_angle_limits_and_never_cross_the_boundary() {
    let mut servo = enabled(3900);
    // 한계 0~4095: 경계 밖 목표는 끝에서 선다 (실물은 4094와 2)
    write(&mut servo, reg::GOAL_POSITION, 4300);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 4095);
    write(&mut servo, reg::GOAL_POSITION, -200);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 0);
    // 한계 1500~2600 밖 목표는 한계에서 선다 (실물은 2597)
    write(&mut servo, reg::MIN_ANGLE_LIMIT, 1500);
    write(&mut servo, reg::MAX_ANGLE_LIMIT, 2600);
    write(&mut servo, reg::GOAL_POSITION, 3000);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 2600);
}

#[test]
fn angle_limits_of_zero_and_zero_cross_the_boundary() {
    let mut servo = enabled(2046);
    write(&mut servo, reg::MIN_ANGLE_LIMIT, 0);
    write(&mut servo, reg::MAX_ANGLE_LIMIT, 0);
    write(&mut servo, reg::GOAL_POSITION, 4300); // 실물은 203
    assert_eq!(servo.get(reg::PRESENT_POSITION), 204);
    write(&mut servo, reg::GOAL_POSITION, -200); // 실물은 3897
    assert_eq!(servo.get(reg::PRESENT_POSITION), 3896);
}

#[test]
fn torque_on_outside_the_limits_moves_to_the_limit() {
    let mut servo = FakeServo::with(20, 777, 2050);
    write(&mut servo, reg::MIN_ANGLE_LIMIT, 2300);
    write(&mut servo, reg::MAX_ANGLE_LIMIT, 2600);
    write(&mut servo, reg::GOAL_POSITION, 2050);
    write(&mut servo, reg::TORQUE_ENABLE, reg::TORQUE_ON);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 2300); // 실물은 2298
}

#[test]
fn goal_is_zero_after_power_on() {
    // 목표를 쓰지 않고 토크를 켜면 0으로 간다: 시작 절차가 있어야 하는 까닭
    let mut servo = FakeServo::with(20, 777, 3000);
    assert_eq!(servo.get(reg::GOAL_POSITION), 0);
    write(&mut servo, reg::TORQUE_ENABLE, reg::TORQUE_ON);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 0);
}

#[test]
fn overload_with_unload_weakens_the_servo_until_torque_is_toggled() {
    let mut servo = enabled(2050);
    servo.held = true;
    write(&mut servo, reg::GOAL_POSITION, 2650);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 2050);
    servo.overload();
    assert_eq!(servo.get(reg::STATUS), i32::from(Fault::OVERLOAD.0));
    assert_eq!(servo.get(reg::TORQUE_ENABLE), reg::TORQUE_ON); // 토크 스위치는 켜진 채
    assert_eq!(servo.get(reg::TORQUE_LIMIT), 1000); // 토크 한도 값도 그대로
    assert_eq!(servo.get(reg::PRESENT_LOAD), 200); // 힘만 보호 뒤 토크(20 %)로
    servo.release();
    assert_eq!(servo.get(reg::PRESENT_POSITION), 2050); // 놓아도 묶여 있다
    assert_eq!(servo.get(reg::STATUS), i32::from(Fault::OVERLOAD.0));
    write(&mut servo, reg::TORQUE_ENABLE, reg::TORQUE_OFF);
    assert_eq!(servo.get(reg::STATUS), 0);
    write(&mut servo, reg::TORQUE_ENABLE, reg::TORQUE_ON);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 2650);
}

#[test]
fn overload_without_unload_clears_when_released() {
    let mut servo = enabled(2050);
    servo.set(reg::UNLOAD_CONDITION, 44 & !i32::from(Fault::OVERLOAD.0));
    servo.held = true;
    write(&mut servo, reg::GOAL_POSITION, 2650);
    servo.overload();
    assert_eq!(servo.get(reg::STATUS), i32::from(Fault::OVERLOAD.0));
    assert_eq!(servo.get(reg::PRESENT_LOAD), 1000); // 힘은 토크 한도 그대로
    servo.release();
    assert_eq!(servo.get(reg::STATUS), 0);
    assert_eq!(servo.get(reg::PRESENT_POSITION), 2650);
}

#[test]
fn voltage_outside_the_range_sets_the_bit_and_keeps_torque() {
    let mut servo = enabled(2048);
    servo.set(reg::MIN_VOLTAGE, 100);
    servo.set_voltage(96);
    assert_eq!(servo.get(reg::STATUS), i32::from(Fault::VOLTAGE.0));
    assert_eq!(servo.get(reg::TORQUE_ENABLE), reg::TORQUE_ON);
    servo.set_voltage(100);
    assert_eq!(servo.get(reg::STATUS), 0);
}
