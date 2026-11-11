//! 메모리 표: Feetech STS 계열 서보(STS3215, STS3250, ST-3120).
//!
//! 출처는 Waveshare의 "Magnetic Encoder ST Servo - Memory Table Explanation"이다 (2026-11-10 확인).
//! Python 기준 구현(packages/servo의 registers.py)과 같은 표이고, 같은지는 그쪽 시험이 본다.
//!
//! - 2바이트 값은 작은 주소가 아래 바이트다.
//! - 기본값은 모델마다 달라서 두지 않는다. 실물의 값은 처음 읽은 덤프가 기준이다.
//! - ST-3120은 같은 표를 쓴다고 보고, 받으면 덤프로 확인한다.
//! - 현재 전류(69)에 방향 비트가 있는지는 표에 없다. 부호 없이 읽고 실물로 확인한다.

use std::fmt;
use std::ops::{BitOr, BitOrAssign};

/// 값이 사는 곳.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Area {
    /// 잠금(55)을 풀고 쓴 값만 전원을 꺼도 남는다. 잠근 채 쓴 값은 꺼질 때까지만 간다.
    Eeprom,
    Ram,
}

/// 레지스터의 부호 없는 값을 정수로 읽는 방법.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Encoding {
    Unsigned,
    /// 부호-크기: 비트 15가 방향 (위치, 속도)
    Sign15,
    /// 부호-크기: 비트 10이 방향 (부하)
    Sign10,
    /// 위치 오프셋만의 표현: 비트 11이 방향, 비트 12가 크기 2048
    Offset,
}

/// 레지스터 하나.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Register {
    pub name: &'static str,
    pub address: u8,
    /// 바이트 수 (1 또는 2)
    pub size: u8,
    pub area: Area,
    pub writable: bool,
    /// 한 칸의 뜻
    pub unit: &'static str,
    pub encoding: Encoding,
}

/// 레지스터에 담을 수 없는 값.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutOfRange {
    pub register: &'static str,
    pub value: i32,
}

impl fmt::Display for OutOfRange {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}는 담을 수 없다", self.register, self.value)
    }
}

impl std::error::Error for OutOfRange {}

impl Register {
    const fn new(
        name: &'static str,
        address: u8,
        size: u8,
        area: Area,
        writable: bool,
        unit: &'static str,
    ) -> Self {
        Self {
            name,
            address,
            size,
            area,
            writable,
            unit,
            encoding: Encoding::Unsigned,
        }
    }

    const fn encoded(self, encoding: Encoding) -> Self {
        Self { encoding, ..self }
    }

    /// 서보가 준 부호 없는 값을 부호 있는 정수로 바꾼다.
    pub fn decode(&self, raw: u16) -> i32 {
        let raw = i32::from(raw);
        match self.encoding {
            Encoding::Unsigned => raw,
            Encoding::Sign15 if raw & 0x8000 != 0 => -(raw & 0x7FFF),
            Encoding::Sign10 if raw & 0x400 != 0 => -(raw & 0x3FF),
            Encoding::Sign15 | Encoding::Sign10 => raw,
            Encoding::Offset => {
                let magnitude = (raw & 0x7FF) | if raw & 0x1000 != 0 { 0x800 } else { 0 };
                if raw & 0x800 != 0 {
                    -magnitude
                } else {
                    magnitude
                }
            }
        }
    }

    /// 정수를 서보에 쓸 부호 없는 값으로 바꾼다.
    pub fn encode(&self, value: i32) -> Result<u16, OutOfRange> {
        let out_of_range = || OutOfRange {
            register: self.name,
            value,
        };
        let magnitude = value.unsigned_abs();
        let raw = match self.encoding {
            Encoding::Unsigned => {
                let max = if self.size == 1 { 0xFF } else { 0xFFFF };
                (0..=max).contains(&value).then_some(magnitude)
            }
            Encoding::Sign15 => sign_magnitude(value, 15),
            Encoding::Sign10 => sign_magnitude(value, 10),
            Encoding::Offset => (magnitude <= 4095).then(|| {
                let raw = (magnitude & 0x7FF) | if magnitude & 0x800 != 0 { 0x1000 } else { 0 };
                if value < 0 { raw | 0x800 } else { raw }
            }),
        };
        raw.and_then(|raw| u16::try_from(raw).ok())
            .ok_or_else(out_of_range)
    }
}

fn sign_magnitude(value: i32, bit: u32) -> Option<u32> {
    let magnitude = value.unsigned_abs();
    (magnitude < 1 << bit).then_some(if value < 0 {
        magnitude | 1 << bit
    } else {
        magnitude
    })
}

const fn version(name: &'static str, address: u8, size: u8, unit: &'static str) -> Register {
    Register::new(name, address, size, Area::Eeprom, false, unit)
}

const fn eeprom(name: &'static str, address: u8, size: u8, unit: &'static str) -> Register {
    Register::new(name, address, size, Area::Eeprom, true, unit)
}

const fn ram(name: &'static str, address: u8, size: u8, unit: &'static str) -> Register {
    Register::new(name, address, size, Area::Ram, true, unit)
}

const fn feedback(name: &'static str, address: u8, size: u8, unit: &'static str) -> Register {
    Register::new(name, address, size, Area::Ram, false, unit)
}

// 버전 정보 (읽기 전용)
pub const FIRMWARE_MAJOR: Register = version("firmware_major", 0, 1, "-");
pub const FIRMWARE_MINOR: Register = version("firmware_minor", 1, 1, "-");
pub const ENDIAN: Register = version("endian", 2, 1, "0이면 작은 바이트가 앞");
// 표는 3과 4를 서보 버전 둘로 적지만, SDK와 다른 자료는 3~4를 2바이트 모델 번호로 읽는다
pub const MODEL: Register = version("model", 3, 2, "모델 번호");

// EEPROM 설정
pub const ID: Register = eeprom("id", 5, 1, "0~253");
pub const BAUD_RATE: Register = eeprom("baud_rate", 6, 1, "코드 (BAUD_RATES)");
pub const RESPONSE_DELAY: Register = eeprom(
    "response_delay",
    7,
    1,
    "2 µs (메모리 표에는 예약. LeRobot과 rustypot은 응답 지연으로 쓴다, 실물로 확인)",
);
pub const RESPONSE_LEVEL: Register = eeprom(
    "response_level",
    8,
    1,
    "0: 읽기·핑에만 응답, 1: 모든 명령에 응답",
);
pub const MIN_ANGLE_LIMIT: Register = eeprom("min_angle_limit", 9, 2, "step (0.087°)");
pub const MAX_ANGLE_LIMIT: Register = eeprom("max_angle_limit", 11, 2, "step (0.087°)");
pub const MAX_TEMPERATURE: Register = eeprom("max_temperature", 13, 1, "°C");
pub const MAX_VOLTAGE: Register = eeprom("max_voltage", 14, 1, "0.1 V");
pub const MIN_VOLTAGE: Register = eeprom("min_voltage", 15, 1, "0.1 V");
pub const MAX_TORQUE: Register = eeprom(
    "max_torque",
    16,
    2,
    "0.1 % (켤 때 토크 제한(48)으로 옮겨진다)",
);
pub const PHASE: Register = eeprom("phase", 18, 1, "비트 (특수 기능, 바꾸지 않는다)");
pub const UNLOAD_CONDITION: Register = eeprom("unload_condition", 19, 1, "비트 (Fault): 켜진 보호");
pub const LED_ALARM_CONDITION: Register =
    eeprom("led_alarm_condition", 20, 1, "비트 (Fault): LED로 알릴 것");
pub const P_GAIN: Register = eeprom("p_gain", 21, 1, "-");
pub const D_GAIN: Register = eeprom("d_gain", 22, 1, "-");
pub const I_GAIN: Register = eeprom("i_gain", 23, 1, "-");
pub const MIN_STARTING_TORQUE: Register = eeprom("min_starting_torque", 24, 1, "0.1 %");
pub const INTEGRAL_LIMIT: Register = eeprom("integral_limit", 25, 1, "×4, 0이면 끔");
pub const DEAD_ZONE_POSITIVE: Register = eeprom("dead_zone_positive", 26, 1, "step (0.087°)");
pub const DEAD_ZONE_NEGATIVE: Register = eeprom("dead_zone_negative", 27, 1, "step (0.087°)");
pub const PROTECTION_CURRENT: Register = eeprom("protection_current", 28, 2, "6.5 mA");
pub const ANGLE_RESOLUTION: Register = eeprom("angle_resolution", 30, 1, "배");
pub const POSITION_OFFSET: Register =
    eeprom("position_offset", 31, 2, "step").encoded(Encoding::Offset);
pub const MODE: Register = eeprom("mode", 33, 1, "0: 위치, 1: 일정 속도, 2: PWM, 3: 스텝");
pub const HOLDING_TORQUE: Register = eeprom(
    "holding_torque",
    34,
    1,
    "1 % (과부하 보호에 들어간 뒤 내는 토크)",
);
pub const PROTECTION_TIME: Register = eeprom(
    "protection_time",
    35,
    1,
    "10 ms (과부하 토크를 넘어 버틸 시간)",
);
pub const OVERLOAD_TORQUE: Register = eeprom(
    "overload_torque",
    36,
    1,
    "1 % (과부하 보호 시간을 재기 시작하는 토크)",
);
pub const SPEED_P_GAIN: Register = eeprom("speed_p_gain", 37, 1, "- (일정 속도 모드)");
pub const OVERCURRENT_PROTECTION_TIME: Register =
    eeprom("overcurrent_protection_time", 38, 1, "10 ms");
pub const SPEED_I_GAIN: Register = eeprom("speed_i_gain", 39, 1, "- (일정 속도 모드)");

// RAM 제어
pub const TORQUE_ENABLE: Register = ram("torque_enable", 40, 1, "0: 끔, 1: 켬, 2: 댐핑");
pub const ACCELERATION: Register = ram("acceleration", 41, 1, "100 step/s² (0이면 최대)");
pub const GOAL_POSITION: Register = ram("goal_position", 42, 2, "step").encoded(Encoding::Sign15);
pub const GOAL_TIME: Register =
    ram("goal_time", 44, 2, "PWM 모드의 출력 (0.1 %)").encoded(Encoding::Sign10);
pub const GOAL_SPEED: Register = ram(
    "goal_speed",
    46,
    2,
    "phase 비트 2에 따라 50 step/s 또는 1 step/s",
)
.encoded(Encoding::Sign15);
pub const TORQUE_LIMIT: Register = ram("torque_limit", 48, 2, "0.1 %");
pub const LOCK: Register = ram("lock", 55, 1, "0: EEPROM 쓰기를 남김, 1: 남기지 않음");

// RAM 상태 (읽기 전용)
pub const PRESENT_POSITION: Register =
    feedback("present_position", 56, 2, "step").encoded(Encoding::Sign15);
pub const PRESENT_SPEED: Register =
    feedback("present_speed", 58, 2, "goal_speed와 같은 단위").encoded(Encoding::Sign15);
pub const PRESENT_LOAD: Register =
    feedback("present_load", 60, 2, "0.1 % (모터 구동 듀티)").encoded(Encoding::Sign10);
pub const PRESENT_VOLTAGE: Register = feedback("present_voltage", 62, 1, "0.1 V");
pub const PRESENT_TEMPERATURE: Register = feedback("present_temperature", 63, 1, "°C");
pub const ASYNC_WRITE_FLAG: Register = feedback("async_write_flag", 64, 1, "-");
pub const STATUS: Register = feedback("status", 65, 1, "비트 (Fault)");
pub const MOVING: Register = feedback("moving", 66, 1, "1이면 목표로 가는 중");
pub const PRESENT_GOAL: Register =
    feedback("present_goal", 67, 2, "step").encoded(Encoding::Sign15);
pub const PRESENT_CURRENT: Register = feedback("present_current", 69, 2, "6.5 mA");

/// 표 전체 (주소 순).
pub const ALL: &[Register] = &[
    FIRMWARE_MAJOR,
    FIRMWARE_MINOR,
    ENDIAN,
    MODEL,
    ID,
    BAUD_RATE,
    RESPONSE_DELAY,
    RESPONSE_LEVEL,
    MIN_ANGLE_LIMIT,
    MAX_ANGLE_LIMIT,
    MAX_TEMPERATURE,
    MAX_VOLTAGE,
    MIN_VOLTAGE,
    MAX_TORQUE,
    PHASE,
    UNLOAD_CONDITION,
    LED_ALARM_CONDITION,
    P_GAIN,
    D_GAIN,
    I_GAIN,
    MIN_STARTING_TORQUE,
    INTEGRAL_LIMIT,
    DEAD_ZONE_POSITIVE,
    DEAD_ZONE_NEGATIVE,
    PROTECTION_CURRENT,
    ANGLE_RESOLUTION,
    POSITION_OFFSET,
    MODE,
    HOLDING_TORQUE,
    PROTECTION_TIME,
    OVERLOAD_TORQUE,
    SPEED_P_GAIN,
    OVERCURRENT_PROTECTION_TIME,
    SPEED_I_GAIN,
    TORQUE_ENABLE,
    ACCELERATION,
    GOAL_POSITION,
    GOAL_TIME,
    GOAL_SPEED,
    TORQUE_LIMIT,
    LOCK,
    PRESENT_POSITION,
    PRESENT_SPEED,
    PRESENT_LOAD,
    PRESENT_VOLTAGE,
    PRESENT_TEMPERATURE,
    ASYNC_WRITE_FLAG,
    STATUS,
    MOVING,
    PRESENT_GOAL,
    PRESENT_CURRENT,
];

/// 덤프는 주소 0부터 마지막 레지스터 끝까지 한 번에 읽는다 (응답 최대 길이 안).
pub const MEMORY_SIZE: usize = end_of(ALL, None);

/// EEPROM의 끝. 가짜 서보가 전원을 꺼도 남는 부분이다.
pub(crate) const EEPROM_END: usize = end_of(ALL, Some(Area::Eeprom));

const fn end_of(registers: &[Register], area: Option<Area>) -> usize {
    let mut end = 0;
    let mut i = 0;
    while i < registers.len() {
        let register = registers[i];
        let counted = match area {
            None => true,
            Some(Area::Eeprom) => matches!(register.area, Area::Eeprom),
            Some(Area::Ram) => matches!(register.area, Area::Ram),
        };
        let register_end = register.address as usize + register.size as usize;
        if counted && register_end > end {
            end = register_end;
        }
        i += 1;
    }
    end
}

/// 운전 중에 한 번의 동기 읽기로 읽는 상태 묶음 (56~70).
pub const STATE: &[Register] = &[
    PRESENT_POSITION,
    PRESENT_SPEED,
    PRESENT_LOAD,
    PRESENT_VOLTAGE,
    PRESENT_TEMPERATURE,
    STATUS,
    MOVING,
    PRESENT_GOAL,
    PRESENT_CURRENT,
];

/// 이름으로 레지스터를 찾는다.
pub fn by_name(name: &str) -> Option<Register> {
    ALL.iter().copied().find(|register| register.name == name)
}

/// 상태 바이트(응답의 오류 바이트, 레지스터 65)와 보호·알림 조건(19, 20)의 비트.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct Fault(pub u8);

impl Fault {
    pub const NONE: Self = Self(0);
    /// 전압이 범위 밖
    pub const VOLTAGE: Self = Self(0x01);
    /// 자석 엔코더 이상
    pub const SENSOR: Self = Self(0x02);
    /// 과열
    pub const TEMPERATURE: Self = Self(0x04);
    /// 과전류
    pub const CURRENT: Self = Self(0x08);
    /// 과부하
    pub const OVERLOAD: Self = Self(0x20);

    const NAMED: [(Self, &'static str); 5] = [
        (Self::VOLTAGE, "전압"),
        (Self::SENSOR, "엔코더"),
        (Self::TEMPERATURE, "과열"),
        (Self::CURRENT, "과전류"),
        (Self::OVERLOAD, "과부하"),
    ];

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    pub fn is_none(self) -> bool {
        self.0 == 0
    }
}

impl BitOr for Fault {
    type Output = Self;

    fn bitor(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

impl BitOrAssign for Fault {
    fn bitor_assign(&mut self, other: Self) {
        self.0 |= other.0;
    }
}

impl fmt::Display for Fault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_none() {
            return write!(f, "없음");
        }
        let mut names: Vec<String> = Self::NAMED
            .iter()
            .filter(|(bit, _)| self.contains(*bit))
            .map(|(_, name)| (*name).to_owned())
            .collect();
        let known = Self::NAMED.iter().fold(0, |bits, (bit, _)| bits | bit.0);
        if self.0 & !known != 0 {
            names.push(format!("알 수 없는 비트 {:#04x}", self.0 & !known));
        }
        write!(f, "{}", names.join(", "))
    }
}

// 토크 스위치에 써도 되는 값. 128은 현재 위치를 2048로 다시 잡으며 위치 오프셋(EEPROM)을 바꿔서 막는다.
pub const TORQUE_OFF: i32 = 0;
pub const TORQUE_ON: i32 = 1;
pub const TORQUE_DAMPING: i32 = 2;
pub const TORQUE_VALUES: [i32; 3] = [TORQUE_OFF, TORQUE_ON, TORQUE_DAMPING];

/// 통신 속도 코드 (레지스터 6).
pub const BAUD_RATES: [(u8, u32); 8] = [
    (0, 1_000_000),
    (1, 500_000),
    (2, 250_000),
    (3, 128_000),
    (4, 115_200),
    (5, 76_800),
    (6, 57_600),
    (7, 38_400),
];

/// 모델 번호 (레지스터 3~4). 가진 서보에서 읽은 값이다 (docs/hardware.md). ST-3120은 받으면 더한다.
pub const MODELS: [(u16, &str); 2] = [(777, "STS3215"), (2825, "STS3250")];

pub const STEPS_PER_TURN: u32 = 4096;
/// SDK의 최대 ID. 254(0xFE)는 방송용이다.
pub const MAX_ID: u8 = 252;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_are_unique() {
        for (i, register) in ALL.iter().enumerate() {
            assert!(
                ALL[i + 1..].iter().all(|other| other.name != register.name),
                "{}",
                register.name
            );
        }
        assert_eq!(by_name("lock"), Some(LOCK));
    }

    #[test]
    fn registers_do_not_overlap() {
        for pair in ALL.windows(2) {
            assert!(
                pair[0].address + pair[0].size <= pair[1].address,
                "{} {}",
                pair[0].name,
                pair[1].name
            );
        }
        assert!(ALL.iter().all(|register| matches!(register.size, 1 | 2)));
        assert_eq!(MEMORY_SIZE, 71);
        assert_eq!(EEPROM_END, 40);
    }

    #[test]
    fn response_delay_is_register_7() {
        assert_eq!((RESPONSE_DELAY.address, RESPONSE_DELAY.size), (7, 1));
    }

    #[test]
    fn state_block_is_one_read() {
        let first = STATE.iter().map(|r| r.address).min();
        let end = STATE.iter().map(|r| r.address + r.size).max();
        assert_eq!((first, end), (Some(56), Some(71)));
    }

    #[test]
    fn fault_bits() {
        let all = Fault::VOLTAGE | Fault::SENSOR | Fault::TEMPERATURE | Fault::CURRENT;
        assert_eq!((all | Fault::OVERLOAD).0, 0x2F);
        assert_eq!(
            (Fault::TEMPERATURE | Fault::OVERLOAD).to_string(),
            "과열, 과부하"
        );
    }

    #[test]
    fn sign_magnitude() {
        let cases = [
            (PRESENT_POSITION, 1000, 1000),
            (GOAL_POSITION, -100, 0x8064),
            (PRESENT_SPEED, -1, 0x8001),
            (PRESENT_LOAD, -500, 0x400 | 500),
            (PRESENT_LOAD, 1000, 1000),
        ];
        for (register, value, raw) in cases {
            assert_eq!(register.encode(value), Ok(raw), "{}", register.name);
            assert_eq!(register.decode(raw), value, "{}", register.name);
        }
    }

    #[test]
    fn offset_encoding() {
        // 메모리 표의 예: 0~2047은 0~2047, 2048~4095는 0~-2047, 4096~6143은 2048~4095, 6144~8191은 -2048~-4095
        let cases = [
            (0, 0),
            (2047, 2047),
            (2048, 0),
            (2049, -1),
            (4096, 2048),
            (6143, 4095),
            (6144, -2048),
            (8191, -4095),
        ];
        for (raw, value) in cases {
            assert_eq!(POSITION_OFFSET.decode(raw), value, "{raw}");
        }
        for value in -4095..=4095 {
            let raw = POSITION_OFFSET.encode(value);
            assert_eq!(raw.map(|raw| POSITION_OFFSET.decode(raw)), Ok(value));
        }
    }

    #[test]
    fn encode_rejects_values_that_do_not_fit() {
        let cases = [
            (P_GAIN, 256),
            (P_GAIN, -1),
            (MAX_TORQUE, 65536),
            (GOAL_POSITION, 32768),
            (PRESENT_LOAD, 1024),
            (POSITION_OFFSET, 4096),
        ];
        for (register, value) in cases {
            assert!(register.encode(value).is_err(), "{} {value}", register.name);
        }
    }

    #[test]
    fn every_encoding_is_used() {
        for encoding in [
            Encoding::Unsigned,
            Encoding::Sign15,
            Encoding::Sign10,
            Encoding::Offset,
        ] {
            assert!(ALL.iter().any(|register| register.encoding == encoding));
        }
    }
}
