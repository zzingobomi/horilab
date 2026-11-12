//! 가짜 Feetech 서보 버스. 실물 없이 버스 층과 그 위를 시험한다.
//!
//! 0009의 "실물 백엔드의 오류 처리를 가짜 서보 버스로 시험한다"에 쓴다. 포트 자리에 끼워서
//! 패킷 처리까지 그대로 지나가게 한다. Python 기준 구현의 fake.py와 같은 동작이다.
//!
//! - 서보 메모리는 메모리 표를 따른다. 기본값은 시험용이고 실물의 기본값이 아니다.
//! - EEPROM은 잠금(55)을 푼 채 쓴 값만 전원을 다시 켜도 남는다 (`power_cycle`).
//! - 토크를 켜거나 토크가 켜진 채 목표를 쓰면 현재 위치가 곧바로 목표가 된다.
//! - 응답 없음, 깨진 응답, 잘린 응답, 상태 바이트의 오류 비트, 같은 ID 둘, 포트 오류를 일부러 낸다.
//! - 가짜 포트는 기다리지 않아서, 응답 없음 시험도 바로 끝난다.
//! - `offset_moves_reading`을 켜면 위치 오프셋을 쓸 때 읽는 위치가 그만큼 옮겨진다 (Rust 쪽에만 있다).
//!   실물로 확인하기 전의 가정이고, 맨 서보 확인의 오프셋 단계를 시험할 때 쓴다.
//!
//! 각도 한계, 보호, 0/4095 경계는 흉내 내지 않는다. 맨 서보 확인 결과가 나오면 실물의 동작으로 맞춘다.

use std::collections::BTreeSet;
use std::io;
use std::time::{Duration, Instant};

use crate::feetech::bus::{Bus, Port};
use crate::feetech::packet;
use crate::feetech::registers::{self as reg, EEPROM_END, Fault, Register};

/// 일부러 내는 고장.
#[derive(Debug, Clone, Default)]
pub struct Faults {
    /// 응답하지 않는다
    pub silent: bool,
    /// 체크섬이 틀린 응답을 보낸다
    pub corrupt: bool,
    /// 응답의 앞 절반만 보낸다
    pub truncate: bool,
    /// 응답의 상태 바이트
    pub status: Fault,
    /// 이 주소에 쓰면 응답하지 않는다
    pub silent_writes_to: BTreeSet<u8>,
    /// 이 주소에 쓰면 응답은 하지만 값을 바꾸지 않는다
    pub ignored_writes_to: BTreeSet<u8>,
}

/// 서보 하나의 메모리와 동작.
#[derive(Debug, Clone)]
pub struct FakeServo {
    pub memory: [u8; reg::MEMORY_SIZE],
    /// 전원을 꺼도 남는 EEPROM
    pub flash: [u8; EEPROM_END],
    /// flash에 쓴 횟수 (EEPROM 수명)
    pub eeprom_saves: u32,
    pub faults: Faults,
    /// 위치 오프셋을 쓰면 읽는 위치가 그만큼 옮겨진다: 읽는 위치 = 엔코더 값 - 오프셋 (확인 전의 가정)
    pub offset_moves_reading: bool,
}

impl FakeServo {
    /// STS3250(모델 2825), 위치 2048인 서보.
    pub fn new(id: u8) -> Self {
        Self::with(id, 2825, 2048)
    }

    pub fn with(id: u8, model: u16, position: i32) -> Self {
        let mut servo = Self {
            memory: [0; reg::MEMORY_SIZE],
            flash: [0; EEPROM_END],
            eeprom_saves: 0,
            faults: Faults::default(),
            offset_moves_reading: false,
        };
        let defaults = [
            (reg::FIRMWARE_MAJOR, 3),
            (reg::FIRMWARE_MINOR, 10),
            (reg::MODEL, i32::from(model)),
            (reg::ID, i32::from(id)),
            (reg::BAUD_RATE, 0),
            (reg::RESPONSE_LEVEL, 1),
            (reg::MIN_ANGLE_LIMIT, 0),
            (reg::MAX_ANGLE_LIMIT, 4095),
            (reg::MAX_TEMPERATURE, 70),
            (reg::MAX_VOLTAGE, 140),
            (reg::MIN_VOLTAGE, 40),
            (reg::MAX_TORQUE, 1000),
            (reg::UNLOAD_CONDITION, 44),
            (reg::LED_ALARM_CONDITION, 47),
            (reg::P_GAIN, 32),
            (reg::D_GAIN, 32),
            (reg::I_GAIN, 0),
            (reg::DEAD_ZONE_POSITIVE, 1),
            (reg::DEAD_ZONE_NEGATIVE, 1),
            (reg::PROTECTION_CURRENT, 511),
            (reg::ANGLE_RESOLUTION, 1),
            (reg::HOLDING_TORQUE, 20),
            (reg::PROTECTION_TIME, 200),
            (reg::OVERLOAD_TORQUE, 80),
            (reg::OVERCURRENT_PROTECTION_TIME, 200),
        ];
        for (register, value) in defaults {
            servo.set(register, value);
        }
        servo.flash.copy_from_slice(&servo.memory[..EEPROM_END]);
        servo.power_on(position);
        servo
    }

    pub fn id(&self) -> u8 {
        self.memory[usize::from(reg::ID.address)]
    }

    pub fn get(&self, register: Register) -> i32 {
        let address = usize::from(register.address);
        let low = self.memory[address];
        let high = if register.size == 2 {
            self.memory[address + 1]
        } else {
            0
        };
        register.decode(u16::from_le_bytes([low, high]))
    }

    /// 시험에서 상태를 만들 때 쓴다. 읽기 전용 레지스터도 바꾼다.
    pub fn set(&mut self, register: Register, value: i32) {
        let [low, high] = register
            .encode(value)
            .expect("시험 값은 레지스터에 담을 수 있어야 한다")
            .to_le_bytes();
        let address = usize::from(register.address);
        self.memory[address] = low;
        if register.size == 2 {
            self.memory[address + 1] = high;
        }
    }

    /// 전원을 껐다 켠다. EEPROM은 flash에서 되살리고 RAM은 처음 값이 된다.
    pub fn power_cycle(&mut self) {
        let position = self.get(reg::PRESENT_POSITION);
        self.memory[..EEPROM_END].copy_from_slice(&self.flash);
        self.memory[EEPROM_END..].fill(0);
        self.power_on(position);
    }

    fn power_on(&mut self, position: i32) {
        self.set(reg::TORQUE_LIMIT, self.get(reg::MAX_TORQUE));
        self.set(reg::LOCK, 1);
        self.set(reg::PRESENT_POSITION, position);
        self.set(reg::GOAL_POSITION, position);
        self.set(reg::PRESENT_GOAL, position);
        self.set(reg::PRESENT_VOLTAGE, 120);
        self.set(reg::PRESENT_TEMPERATURE, 25);
    }

    pub fn read(&self, address: u8, length: u8) -> Vec<u8> {
        let start = usize::from(address).min(self.memory.len());
        let end = (start + usize::from(length)).min(self.memory.len());
        self.memory[start..end].to_vec()
    }

    pub fn write(&mut self, address: u8, data: &[u8]) {
        let start = usize::from(address);
        let offset_before = self.get(reg::POSITION_OFFSET);
        for (offset, &value) in data.iter().enumerate() {
            let target = start + offset;
            let ignored = is_read_only(target)
                || u8::try_from(target).is_ok_and(|t| self.faults.ignored_writes_to.contains(&t));
            if target < self.memory.len() && !ignored {
                self.memory[target] = value;
            }
        }
        let offset_change = self.get(reg::POSITION_OFFSET) - offset_before;
        if self.offset_moves_reading && offset_change != 0 {
            let turn = reg::STEPS_PER_TURN as i32;
            let position = (self.get(reg::PRESENT_POSITION) - offset_change).rem_euclid(turn);
            self.set(reg::PRESENT_POSITION, position);
        }
        if start < EEPROM_END && self.memory[usize::from(reg::LOCK.address)] == 0 {
            self.flash.copy_from_slice(&self.memory[..EEPROM_END]);
            self.eeprom_saves += 1;
        }
        let touched = |register: Register| {
            (start..start + data.len()).contains(&usize::from(register.address))
        };
        let torque_on =
            i32::from(self.memory[usize::from(reg::TORQUE_ENABLE.address)]) == reg::TORQUE_ON;
        if (touched(reg::TORQUE_ENABLE) || touched(reg::GOAL_POSITION)) && torque_on {
            let goal = self.get(reg::GOAL_POSITION);
            self.set(reg::PRESENT_POSITION, goal);
            self.set(reg::PRESENT_GOAL, goal);
        }
    }
}

fn is_read_only(address: usize) -> bool {
    reg::ALL.iter().any(|register| {
        let start = usize::from(register.address);
        !register.writable && (start..start + usize::from(register.size)).contains(&address)
    })
}

/// 서보들이 물린 버스 선. 받은 명령 패킷에 서보들이 낼 응답을 만든다.
#[derive(Debug, Clone, Default)]
pub struct FakeWire {
    pub servos: Vec<FakeServo>,
    /// 다음 보내기에서 포트 오류를 낸다 (USB가 빠진 것처럼)
    pub fail_next_send: bool,
}

impl FakeWire {
    fn handle(&mut self, request: &[u8]) -> Vec<u8> {
        let &[0xFF, 0xFF, id, length, instruction, ..] = request else {
            return Vec::new();
        };
        let well_formed = request.len() == usize::from(length) + 4
            && packet::checksum(&request[2..request.len() - 1]) == request[request.len() - 1];
        if !well_formed {
            return Vec::new(); // 실물도 체크섬이 틀린 명령은 무시한다
        }
        let params = &request[5..request.len() - 1];
        match (instruction, params) {
            (packet::SYNC_READ, [address, size, ids @ ..]) => ids
                .iter()
                .flat_map(|&servo_id| self.reply(servo_id, |servo| servo.read(*address, *size)))
                .collect(),
            (packet::SYNC_WRITE, [address, size, rest @ ..]) => {
                for chunk in rest.chunks(usize::from(*size) + 1) {
                    if let [servo_id, data @ ..] = chunk {
                        for servo in self.servos.iter_mut().filter(|s| s.id() == *servo_id) {
                            servo.write(*address, data);
                        }
                    }
                }
                Vec::new()
            }
            _ if id == packet::BROADCAST_ID => Vec::new(),
            (packet::PING, _) => self.reply(id, |_| Vec::new()),
            (packet::READ, [address, size]) => self.reply(id, |servo| servo.read(*address, *size)),
            (packet::WRITE, [address, data @ ..]) => {
                let mut silent = false;
                let mut replies = Vec::new();
                for servo in self.servos.iter_mut().filter(|s| s.id() == id) {
                    servo.write(*address, data);
                    silent |= servo.faults.silent_writes_to.contains(address);
                    // 모든 명령에 응답하는 설정(8번이 1)일 때만 쓰기에 응답한다. ID를 바꿨으면 새 ID로 답한다.
                    if servo.memory[usize::from(reg::RESPONSE_LEVEL.address)] == 1 {
                        replies.extend(status(servo, &[]));
                    }
                }
                if silent { Vec::new() } else { replies }
            }
            _ => Vec::new(),
        }
    }

    fn reply(&self, id: u8, payload: impl Fn(&FakeServo) -> Vec<u8>) -> Vec<u8> {
        let servos: Vec<&FakeServo> = self.servos.iter().filter(|s| s.id() == id).collect();
        match servos[..] {
            [] => Vec::new(),
            [servo] => status(servo, &payload(servo)),
            [first, second, ..] => {
                // 같은 ID의 서보 둘이 동시에 답하면 선 위에서 섞인다
                let a = status(first, &payload(first));
                let b = status(second, &payload(second));
                let mut mixed: Vec<u8> = a.iter().zip(&b).map(|(x, y)| x | y).collect();
                if let Some(length) = mixed.get_mut(3) {
                    *length ^= 0x5A;
                }
                mixed
            }
        }
    }
}

fn status(servo: &FakeServo, data: &[u8]) -> Vec<u8> {
    let faults = &servo.faults;
    if faults.silent {
        return Vec::new();
    }
    let mut reply = packet::status(servo.id(), faults.status.0, data);
    if faults.corrupt
        && let Some(checksum) = reply.last_mut()
    {
        *checksum ^= 0xFF;
    }
    if faults.truncate {
        reply.truncate(reply.len() / 2);
    }
    reply
}

/// 가짜 선에 붙는 포트.
#[derive(Debug, Clone, Default)]
pub struct FakePort {
    pub wire: FakeWire,
    received: Vec<u8>,
}

impl Port for FakePort {
    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        if std::mem::take(&mut self.wire.fail_next_send) {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "가짜 포트 오류 (USB가 빠진 것처럼)",
            ));
        }
        let reply = self.wire.handle(bytes);
        self.received.extend(reply);
        Ok(())
    }

    fn receive(&mut self, buffer: &mut Vec<u8>, want: usize, _deadline: Instant) -> io::Result<()> {
        let take = want.saturating_sub(buffer.len()).min(self.received.len());
        buffer.extend(self.received.drain(..take));
        Ok(())
    }

    fn clear_input(&mut self) -> io::Result<()> {
        self.received.clear();
        Ok(())
    }
}

/// 가짜 서보들을 물린 버스를 연다.
pub fn open_fake_bus(servos: impl IntoIterator<Item = FakeServo>) -> Bus<FakePort> {
    let port = FakePort {
        wire: FakeWire {
            servos: servos.into_iter().collect(),
            fail_next_send: false,
        },
        received: Vec::new(),
    };
    Bus::new(port, 1_000_000, Duration::from_millis(1))
}

impl Bus<FakePort> {
    /// 버스에 물린 가짜 선. 시험에서 서보 상태를 보고 고장을 넣는다.
    pub fn wire(&mut self) -> &mut FakeWire {
        &mut self.port.wire
    }
}
