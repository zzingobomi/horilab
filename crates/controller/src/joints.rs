//! 관절 층과 안전 검사 (docs/decisions/0024-servo-bus.md의 만드는 순서 4, 0009의 한곳 안전 검사).
//!
//! 서보를 움직이는 쓰기(토크 켜기·끄기, 목표, 토크 한도, 속도, 가속)는 이 모듈만 한다. 움직이는 명령은
//! [`Joints`]를 거쳐야 버스에 닿는다. 값 적용의 EEPROM 쓰기는 서보를 움직이지 않아 `prepare`가 따로 한다.
//!
//! - 로봇 관절은 영점과 방향을 잰 뒤에만 움직인다. 관절 각(rad)을 서보 값으로 바꾸고 소프트 한계 안인지 본다.
//! - 맨 서보(로봇에 달지 않고 책상에 둔 서보 하나, 맨 서보 확인)는 서보 값 그대로 다루고, 한계는 부르는
//!   쪽이 정한다.
//! - 토크를 켤 때는 목표를 현재 위치로 맞추고 토크 한도·속도·가속을 쓴 뒤 켠다. 이미 켜져 있으면 서보에
//!   남은 목표와 설정을 이어받고 다시 쓰지 않는다 (0009의 "컨트롤러만 다시 켜짐").
//! - 서보 보호 비트가 켜져 있거나 로봇 관절의 위치 오프셋이 로봇 설정과 다르면 토크를 켜지도 이어받지도
//!   않는다 (0024의 서보 보호와 영점).
//! - 한 서보와의 통신이 정한 횟수만큼 잇달아 실패하면 멈춤이 되고, 사람이 풀기 전에는 목표를 쓰지 않는다.

use std::time::Duration;

use horilab_servo::feetech::ServoBus;
use horilab_servo::feetech::bus::Reply;
use horilab_servo::feetech::registers::{self as reg, Fault, Register};

use crate::prepare::{model_name, plan};
use crate::settings::{Calibration, Joint, LoopSettings, RAD_PER_STEP, angle_limits};

/// 토크를 켤 때 쓰는 상한. 토크 한도는 0.1 %, 속도는 서보의 속도 단위(phase 비트에 따라 다르다),
/// 가속은 100 step/s² 단위다 (레지스터 표).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Caps {
    pub torque_limit: i32,
    pub speed: i32,
    pub acceleration: i32,
}

/// 맨 서보 확인의 상한: 토크 20 %, 속도 300, 가속 2,000 step/s².
pub const BENCH_CAPS: Caps = Caps {
    torque_limit: 200,
    speed: 300,
    acceleration: 20,
};

/// 실행 루프의 서보 상한을 궤적 한계보다 넉넉히 두는 배수
const LOOP_CAPS_MARGIN: f64 = 1.5;

impl Caps {
    /// 실행 루프의 상한: 서보 안의 속도·가속 상한은 궤적 한계의 1.5배로 두어, 박자마다 쓰는 목표를 서보가 늦게
    /// 따라가지 않게 한다. 토크는 로봇 설정의 상한이고, 관절마다 더 낮은 토크 한도를 주면 그것을 쓴다.
    pub fn for_loop(settings: &LoopSettings) -> Self {
        let steps = |radians: f64| radians / RAD_PER_STEP * LOOP_CAPS_MARGIN;
        Self {
            torque_limit: settings.torque_cap,
            speed: steps(settings.max_speed).ceil() as i32,
            acceleration: (steps(settings.max_acceleration) / 100.0)
                .ceil()
                .clamp(1.0, 254.0) as i32,
        }
    }

    /// 이 상한으로 서보가 실행 루프의 속도·가속 한계를 따라갈 수 있는가.
    pub fn keeps_up_with(&self, settings: &LoopSettings) -> bool {
        f64::from(self.speed) * RAD_PER_STEP >= settings.max_speed
            && f64::from(self.acceleration) * 100.0 * RAD_PER_STEP >= settings.max_acceleration
    }
}

/// 이만큼 잇달아 실패하면 멈춘다 (0009).
pub const MAX_FAILURES: u64 = 3;
/// 상한(속도, 가속)을 쓴 뒤 토크를 켜기 전에 기다리는 시간. 서보는 쓴 값을 몇 ms 뒤에 반영한다
/// (2026-11-13 실물: 오프셋을 쓰고 2 ms 뒤에는 옛 값, 4 ms 뒤에는 새 값).
pub const CAPS_SETTLE: Duration = Duration::from_millis(20);

/// 로봇 관절에서 잠근 채 바꿔 볼 수 있는 레지스터 (게인을 고를 때, 통신 시간을 잴 때). 한계와 보호 설정은
/// 맨 서보에서만 바꿔 본다.
const JOINT_TRIALS: [Register; 4] = [reg::P_GAIN, reg::I_GAIN, reg::D_GAIN, reg::RESPONSE_DELAY];

/// 움직일 서보 하나.
#[derive(Debug, Clone, PartialEq)]
pub struct Servo {
    /// 사람이 볼 이름 (관절 이름, 맨 서보는 "서보 <ID>")
    pub name: String,
    pub id: u8,
    pub model_number: u16,
    /// 이 범위 밖의 목표는 쓰지 않는다 (서보 값)
    pub min_raw: i32,
    pub max_raw: i32,
    /// 토크 한도 [0.1 %]. 없으면 관절 층의 상한을 쓰고, 상한보다 크면 상한을 쓴다
    pub torque_limit: Option<i32>,
    /// 서보에 구워져 있어야 할 EEPROM 값 (로봇 설정의 값 적용 목표. 각도 한계와 위치 오프셋은 이 서보의 영점과
    /// 소프트 한계로 다시 계산한다). 비어 있으면 토크를 켤 때 위치 오프셋만 견준다
    pub eeprom: Vec<(Register, i32)>,
    calibration: Option<Calibration>,
}

impl Servo {
    /// 맨 서보: 서보 값 그대로 다룬다.
    pub fn bench(id: u8, model_number: u16, min_raw: i32, max_raw: i32) -> Self {
        Self {
            name: format!("서보 {id}"),
            id,
            model_number,
            min_raw,
            max_raw,
            torque_limit: None,
            eeprom: Vec::new(),
            calibration: None,
        }
    }

    /// 로봇 관절. 영점과 방향을 재기 전에는 움직이지 않는다. 한계는 소프트 한계를 안쪽으로 반올림한 서보 값이다.
    pub fn from_joint(joint: &Joint) -> Result<Self, String> {
        let calibration = joint
            .calibration
            .ok_or_else(|| format!("{}: 영점과 방향을 재기 전이라 움직이지 않는다", joint.name))?;
        let (a, b) = (
            calibration.raw_at(joint.lower),
            calibration.raw_at(joint.upper),
        );
        let (min_angle, max_angle) = angle_limits(joint.lower, joint.upper, Some(calibration));
        let eeprom = joint
            .eeprom
            .iter()
            .map(|&(register, value)| {
                let value = if register == reg::MIN_ANGLE_LIMIT {
                    min_angle
                } else if register == reg::MAX_ANGLE_LIMIT {
                    max_angle
                } else if register == reg::POSITION_OFFSET {
                    calibration.position_offset
                } else {
                    value
                };
                (register, value)
            })
            .collect();
        Ok(Self {
            name: joint.name.to_owned(),
            id: joint.id,
            model_number: joint.model_number,
            min_raw: a.min(b).ceil() as i32,
            max_raw: a.max(b).floor() as i32,
            torque_limit: joint.torque_limit,
            eeprom,
            calibration: Some(calibration),
        })
    }

    /// 로봇에 달지 않은 맨 서보인가.
    pub fn is_bench(&self) -> bool {
        self.calibration.is_none()
    }

    /// 관절 각 [rad]의 서보 값. 맨 서보이거나 각이 수가 아니면 없다.
    pub fn raw_at(&self, angle: f64) -> Option<i32> {
        if !angle.is_finite() {
            return None;
        }
        self.calibration.map(|c| c.raw_at(angle).round() as i32)
    }

    /// 서보 값의 관절 각 [rad]. 맨 서보는 없다.
    pub fn angle_at(&self, raw: i32) -> Option<f64> {
        self.calibration.map(|c| c.angle_at(f64::from(raw)))
    }
}

/// 토크를 켠 결과.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Enabled {
    /// 목표를 현재 위치로 맞추고 켰다
    TurnedOn,
    /// 이미 켜져 있어서 남은 목표와 설정을 이어받았다
    AlreadyOn,
}

/// 한 번의 읽기로 본 서보 상태 (레지스터 40~70).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    pub torque_enable: i32,
    pub acceleration: i32,
    pub goal_position: i32,
    pub goal_speed: i32,
    pub torque_limit: i32,
    pub lock: i32,
    pub position: i32,
    pub speed: i32,
    pub load: i32,
    /// 0.1 V
    pub voltage: i32,
    /// °C
    pub temperature: i32,
    pub status: Fault,
    pub moving: bool,
    /// 6.5 mA
    pub current: i32,
}

/// 기록(samples.csv)의 값 칸 이름. 레지스터 이름과 같다.
pub const SNAPSHOT_COLUMNS: [&str; 14] = [
    "torque_enable",
    "acceleration",
    "goal_position",
    "goal_speed",
    "torque_limit",
    "lock",
    "present_position",
    "present_speed",
    "present_load",
    "present_voltage",
    "present_temperature",
    "status",
    "moving",
    "present_current",
];

impl Snapshot {
    pub fn torque_on(&self) -> bool {
        self.torque_enable == reg::TORQUE_ON
    }

    /// 기록 칸 이름 ([`SNAPSHOT_COLUMNS`]).
    pub fn columns() -> Vec<&'static str> {
        SNAPSHOT_COLUMNS.to_vec()
    }

    /// 기록과 화면에 쓰는 칸 이름과 값 ([`SNAPSHOT_COLUMNS`] 순).
    pub fn fields(&self) -> [(&'static str, i32); 14] {
        let values = [
            self.torque_enable,
            self.acceleration,
            self.goal_position,
            self.goal_speed,
            self.torque_limit,
            self.lock,
            self.position,
            self.speed,
            self.load,
            self.voltage,
            self.temperature,
            i32::from(self.status.0),
            i32::from(self.moving),
            self.current,
        ];
        let mut fields = [("", 0); 14];
        for (field, (name, value)) in fields.iter_mut().zip(SNAPSHOT_COLUMNS.iter().zip(values)) {
            *field = (*name, value);
        }
        fields
    }
}

/// 상태를 읽는 레지스터 (40~70을 한 번에 읽는다).
const SNAPSHOT: [Register; 14] = [
    reg::TORQUE_ENABLE,
    reg::ACCELERATION,
    reg::GOAL_POSITION,
    reg::GOAL_SPEED,
    reg::TORQUE_LIMIT,
    reg::LOCK,
    reg::PRESENT_POSITION,
    reg::PRESENT_SPEED,
    reg::PRESENT_LOAD,
    reg::PRESENT_VOLTAGE,
    reg::PRESENT_TEMPERATURE,
    reg::STATUS,
    reg::MOVING,
    reg::PRESENT_CURRENT,
];

/// 관절 층: 버스 하나와 그 위의 서보들.
pub struct Joints<B: ServoBus> {
    bus: B,
    servos: Vec<Servo>,
    caps: Caps,
    stopped: Option<String>,
}

impl<B: ServoBus> Joints<B> {
    /// 서보마다 응답과 모델을 확인하고 연다.
    pub fn open(mut bus: B, servos: Vec<Servo>, caps: Caps) -> Result<Self, String> {
        for servo in &servos {
            let model = bus
                .ping(servo.id)
                .map_err(|e| format!("{}가 응답하지 않는다: {e}", servo.name))?;
            if model != servo.model_number {
                return Err(format!(
                    "{}는 {}가 아니라 {}다",
                    servo.name,
                    model_name(servo.model_number),
                    model_name(model)
                ));
            }
        }
        Ok(Self {
            bus,
            servos,
            caps,
            stopped: None,
        })
    }

    pub fn caps(&self) -> Caps {
        self.caps
    }

    pub fn servo(&self, id: u8) -> Result<&Servo, String> {
        self.servos
            .iter()
            .find(|servo| servo.id == id)
            .ok_or_else(|| format!("서보 {id}는 이 관절 층에 없다"))
    }

    /// 멈춘 까닭. 멈췄으면 사람이 [`Joints::resume`]으로 풀기 전에는 목표를 쓰지 않는다.
    pub fn stopped(&self) -> Option<&str> {
        self.stopped.as_deref()
    }

    /// 사람이 확인하고 멈춤을 푼다.
    pub fn resume(&mut self) {
        self.stopped = None;
    }

    /// 상태를 읽는다. 멈춰 있어도 읽는다.
    pub fn snapshot(&mut self, id: u8) -> Result<Snapshot, String> {
        let name = self.servo(id)?.name.clone();
        let result = self.bus.read(id, &SNAPSHOT);
        self.watch(id);
        let reply = result.map_err(|e| format!("{name}: 상태를 읽지 못했다: {e}"))?;
        Ok(snapshot_of(&reply))
    }

    /// 관절 층의 서보 ID (연 순서).
    pub fn ids(&self) -> Vec<u8> {
        self.servos.iter().map(|servo| servo.id).collect()
    }

    /// 모든 서보의 상태를 한 번에 읽는다 (동기 읽기, 실행 루프의 박자). 멈춰 있어도 읽는다. 한 서보라도
    /// 읽지 못하면 오류이고, 잇단 실패는 멈춤이 된다. 결과는 연 순서다.
    pub fn read_all(&mut self) -> Result<Vec<(u8, Snapshot)>, String> {
        let ids = self.ids();
        let result = self.bus.sync_read(&ids, &SNAPSHOT);
        for &id in &ids {
            self.watch(id);
        }
        let replies = result.map_err(|e| format!("상태를 한 번에 읽지 못했다: {e}"))?;
        let mut states = Vec::new();
        let mut failed = Vec::new();
        for servo in &self.servos {
            match replies.get(&servo.id) {
                Some(Ok(reply)) => states.push((servo.id, snapshot_of(reply))),
                Some(Err(e)) => failed.push(format!("{}: {e}", servo.name)),
                None => failed.push(format!("{}: 응답이 없다", servo.name)),
            }
        }
        if failed.is_empty() {
            Ok(states)
        } else {
            Err(format!("상태를 읽지 못했다: {}", failed.join(", ")))
        }
    }

    /// 여러 서보의 목표를 한 번에 쓴다 (동기 쓰기, 실행 루프의 박자). 멈춰 있거나 하나라도 한계 밖이면
    /// 아무것도 쓰지 않는다. 토크가 켜져 있는지는 같은 박자에 읽은 상태로 부르는 쪽이 본다.
    pub fn write_goals(&mut self, goals: &[(u8, i32)]) -> Result<(), String> {
        self.require_running()?;
        for &(id, raw) in goals {
            let servo = self.servo(id)?;
            if raw < servo.min_raw || raw > servo.max_raw {
                return Err(format!(
                    "{}: 목표 {raw}가 한계({}~{}) 밖이라 아무것도 쓰지 않는다",
                    servo.name, servo.min_raw, servo.max_raw
                ));
            }
        }
        self.bus
            .sync_write(reg::GOAL_POSITION, goals)
            .map_err(|e| format!("목표를 한 번에 쓰지 못했다: {e}"))
    }
}

/// 상태 읽기의 응답을 [`Snapshot`]으로 푼다.
fn snapshot_of(reply: &Reply) -> Snapshot {
    let get = |register: Register| reply.get(register).unwrap_or_default();
    Snapshot {
        torque_enable: get(reg::TORQUE_ENABLE),
        acceleration: get(reg::ACCELERATION),
        goal_position: get(reg::GOAL_POSITION),
        goal_speed: get(reg::GOAL_SPEED),
        torque_limit: get(reg::TORQUE_LIMIT),
        lock: get(reg::LOCK),
        position: get(reg::PRESENT_POSITION),
        speed: get(reg::PRESENT_SPEED),
        load: get(reg::PRESENT_LOAD),
        voltage: get(reg::PRESENT_VOLTAGE),
        temperature: get(reg::PRESENT_TEMPERATURE),
        status: Fault(u8::try_from(get(reg::STATUS)).unwrap_or(u8::MAX)),
        moving: get(reg::MOVING) != 0,
        current: get(reg::PRESENT_CURRENT),
    }
}

impl<B: ServoBus> Joints<B> {
    /// EEPROM이나 RAM 레지스터를 읽는다 (서보를 움직이지 않는다).
    pub fn read(&mut self, id: u8, registers: &[Register]) -> Result<Vec<(Register, i32)>, String> {
        let name = self.servo(id)?.name.clone();
        let result = self.bus.read(id, registers);
        self.watch(id);
        let reply = result.map_err(|e| format!("{name}: 읽지 못했다: {e}"))?;
        registers
            .iter()
            .map(|&register| {
                reply
                    .get(register)
                    .map(|value| (register, value))
                    .ok_or_else(|| format!("{name}: {}을 읽지 못했다", register.name))
            })
            .collect()
    }

    /// 토크를 켠다 (0009). 꺼져 있으면 목표를 현재 위치로 맞추고 상한을 쓴 뒤 켠다. 이미 켜져 있으면 아무것도
    /// 쓰지 않고 이어받는다. 서보 보호 비트가 켜져 있거나(보호가 걸려도 토크 스위치는 켜진 채다), 로봇 관절의
    /// 위치 오프셋이 로봇 설정과 다르면(읽는 위치가 그만큼 틀어진다) 켜지도 이어받지도 않는다 (0024). 로봇 설정이
    /// EEPROM 목표를 주면 덤프로 모두 견주고, 하나라도 다르면 켜지 않는다 (예비 서보를 갈아 끼우고 값 적용을
    /// 빠뜨리면 각도 한계와 보호 설정이 다르다).
    pub fn enable(&mut self, id: u8) -> Result<Enabled, String> {
        self.require_running()?;
        let state = self.snapshot(id)?;
        let servo = self.servo(id)?.clone();
        if !state.status.is_none() {
            return Err(format!(
                "{}: 서보 보호가 걸려 있다 ({}). 토크를 켜지도 이어받지도 않는다. 원인을 없앤 뒤 사람이 \
                 토크를 껐다 다시 켠다",
                servo.name, state.status
            ));
        }
        if let Some(calibration) = servo.calibration {
            let offset = self.read(id, &[reg::POSITION_OFFSET])?[0].1;
            if offset != calibration.position_offset {
                return Err(format!(
                    "{}: 서보의 위치 오프셋 {offset}이 로봇 설정의 {}와 달라 읽는 위치가 틀어진다. \
                     값 적용으로 맞추거나 영점을 다시 잰다",
                    servo.name, calibration.position_offset
                ));
            }
            if !servo.eeprom.is_empty() {
                let memory = self.dump(id)?;
                let changes = plan(&servo.eeprom, &memory)?;
                if !changes.is_empty() {
                    let listed: Vec<String> = changes.iter().map(|c| c.describe()).collect();
                    return Err(format!(
                        "{}: 서보의 EEPROM 값이 로봇 설정과 다르다 ({}). 값 적용으로 맞춘 뒤 켠다",
                        servo.name,
                        listed.join(", ")
                    ));
                }
            }
        }
        if state.torque_on() {
            return Ok(Enabled::AlreadyOn);
        }
        let caps = self.caps;
        self.write(id, reg::GOAL_POSITION, state.position)?;
        let torque_limit = servo
            .torque_limit
            .map_or(caps.torque_limit, |limit| limit.min(caps.torque_limit));
        self.write(id, reg::TORQUE_LIMIT, torque_limit)?;
        self.write(id, reg::ACCELERATION, caps.acceleration)?;
        self.write(id, reg::GOAL_SPEED, caps.speed)?;
        // 서보는 쓴 값을 몇 ms 뒤에 반영한다. 전원을 넣고 처음 켤 때 속도와 가속을 쓰고 바로 토크를 켜자,
        // 다음에 토크를 껐다 켤 때까지 속도 상한 없이 토크 한도껏 돌았다 (2026-11-13 맨 서보 확인)
        self.bus.pause(CAPS_SETTLE);
        self.write(id, reg::TORQUE_ENABLE, reg::TORQUE_ON)?;
        Ok(Enabled::TurnedOn)
    }

    /// 토크를 끈다. 로봇 관절은 끄면 떨어지므로 부르는 쪽이 받쳐 둔 뒤에 끈다.
    pub fn disable(&mut self, id: u8) -> Result<(), String> {
        self.servo(id)?;
        self.write(id, reg::TORQUE_ENABLE, reg::TORQUE_OFF)
    }

    /// 서보 값으로 목표를 쓴다. 한계 밖이거나, 토크가 꺼져 있거나, 멈춰 있으면 쓰지 않는다.
    pub fn move_raw(&mut self, id: u8, raw: i32) -> Result<(), String> {
        self.require_running()?;
        let servo = self.servo(id)?;
        if raw < servo.min_raw || raw > servo.max_raw {
            return Err(format!(
                "{}: 목표 {raw}가 한계({}~{}) 밖이라 쓰지 않는다",
                servo.name, servo.min_raw, servo.max_raw
            ));
        }
        let name = servo.name.clone();
        if !self.snapshot(id)?.torque_on() {
            return Err(format!("{name}: 토크가 꺼져 있어 목표를 쓰지 않는다"));
        }
        self.write(id, reg::GOAL_POSITION, raw)
    }

    /// 관절 각 [rad]으로 목표를 쓴다. 맨 서보는 받지 않는다.
    pub fn move_angle(&mut self, id: u8, angle: f64) -> Result<(), String> {
        let servo = self.servo(id)?;
        let raw = servo
            .raw_at(angle)
            .ok_or_else(|| format!("{}는 맨 서보라 관절 각을 받지 않는다", servo.name))?;
        self.move_raw(id, raw)
    }

    /// 토크 한도를 상한보다 낮게 바꾼다 (보호 동작을 볼 때). 상한보다 높이지는 않는다.
    pub fn limit_torque(&mut self, id: u8, torque_limit: i32) -> Result<(), String> {
        self.require_running()?;
        let caps = self.caps;
        if !(0..=caps.torque_limit).contains(&torque_limit) {
            return Err(format!(
                "토크 한도 {torque_limit}는 0~{}(상한) 밖이다",
                caps.torque_limit
            ));
        }
        self.servo(id)?;
        self.write(id, reg::TORQUE_LIMIT, torque_limit)
    }

    /// 메모리 전체를 읽는다 (서보를 움직이지 않는다).
    pub fn dump(&mut self, id: u8) -> Result<Vec<u8>, String> {
        let name = self.servo(id)?.name.clone();
        let result = self.bus.dump(id);
        self.watch(id);
        result.map_err(|e| format!("{name}: 덤프를 뜨지 못했다: {e}"))
    }

    /// 잠근 채 바꿔 보기 (EEPROM, 전원을 끄면 사라진다). 서보를 움직이지 않는다. 로봇 관절은 게인과 응답
    /// 지연만 받는다.
    pub fn trial(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), String> {
        let servo = self.servo(id)?;
        let name = servo.name.clone();
        if !servo.is_bench()
            && let Some((register, _)) = values.iter().find(|(r, _)| !JOINT_TRIALS.contains(r))
        {
            return Err(format!(
                "{name}: {}은 로봇 관절에서 바꿔 보지 않는다 (맨 서보에서만)",
                register.name
            ));
        }
        let result = self.bus.write_trial(id, values);
        self.watch(id);
        result.map_err(|e| format!("{name}: 잠근 채 쓰지 못했다: {e}"))
    }

    fn write(&mut self, id: u8, register: Register, value: i32) -> Result<(), String> {
        let name = self.servo(id)?.name.clone();
        let result = self.bus.write(id, register, value);
        self.watch(id);
        result
            .map(|_| ())
            .map_err(|e| format!("{name}: {}을 쓰지 못했다: {e}", register.name))
    }

    fn require_running(&self) -> Result<(), String> {
        match &self.stopped {
            Some(reason) => Err(format!("멈춰 있어 움직이지 않는다: {reason}")),
            None => Ok(()),
        }
    }

    /// 통신 실패가 잇달아 정한 횟수에 닿으면 멈춘다.
    fn watch(&mut self, id: u8) {
        let failures = self
            .bus
            .stats()
            .get(&id)
            .map_or(0, |stats| stats.consecutive_failures);
        if failures >= MAX_FAILURES && self.stopped.is_none() {
            self.stopped = Some(format!("서보 {id}와의 통신이 {failures}번 잇달아 실패했다"));
        }
    }
}

#[cfg(test)]
impl Joints<horilab_servo::feetech::Bus<horilab_servo::feetech::fake::FakePort>> {
    /// 시험에서 가짜 서보 상태를 보고 고장을 넣는다.
    pub(crate) fn wire(&mut self) -> &mut horilab_servo::feetech::fake::FakeWire {
        self.bus.wire()
    }
}

/// 시험용: 서보의 EEPROM 목표를 구운 가짜 서보 (값 적용을 한 것처럼, 전원을 껐다 켜도 남는다).
#[cfg(test)]
pub(crate) fn programmed(
    mut fake: horilab_servo::feetech::fake::FakeServo,
    servo: &Servo,
) -> horilab_servo::feetech::fake::FakeServo {
    for &(register, value) in &servo.eeprom {
        fake.set(register, value);
    }
    let burned = fake.flash.len();
    fake.flash.copy_from_slice(&fake.memory[..burned]);
    fake
}

#[cfg(test)]
mod tests {
    use horilab_servo::feetech::Bus;
    use horilab_servo::feetech::fake::{FakePort, FakeServo, open_fake_bus};

    use super::*;
    use crate::settings::{Direction, Settings, TEST_J3};

    fn bench(servo: FakeServo) -> Joints<Bus<FakePort>> {
        let id = servo.id();
        Joints::open(
            open_fake_bus([servo]),
            vec![Servo::bench(id, 2825, 0, 4095)],
            BENCH_CAPS,
        )
        .unwrap()
    }

    #[test]
    fn enable_sets_goal_to_present_then_caps_then_torque() {
        let mut joints = bench(FakeServo::with(20, 2825, 3000));
        assert_eq!(joints.enable(20).unwrap(), Enabled::TurnedOn);
        let state = joints.snapshot(20).unwrap();
        assert!(state.torque_on());
        assert_eq!(state.goal_position, 3000);
        assert_eq!(state.position, 3000);
        assert_eq!(state.torque_limit, BENCH_CAPS.torque_limit);
        assert_eq!(state.goal_speed, BENCH_CAPS.speed);
        assert_eq!(state.acceleration, BENCH_CAPS.acceleration);
    }

    #[test]
    fn torque_already_on_is_inherited_without_writing() {
        let mut servo = FakeServo::with(20, 2825, 1000);
        servo.set(reg::TORQUE_ENABLE, reg::TORQUE_ON);
        servo.set(reg::GOAL_POSITION, 1200); // 짐에 눌려 목표보다 아래에서 버티는 중
        servo.set(reg::TORQUE_LIMIT, 700);
        let mut joints = bench(servo);
        assert_eq!(joints.enable(20).unwrap(), Enabled::AlreadyOn);
        let state = joints.snapshot(20).unwrap();
        assert_eq!((state.goal_position, state.torque_limit), (1200, 700));
    }

    #[test]
    fn a_servo_weakened_by_overload_is_not_taken_over() {
        let mut joints = bench(FakeServo::with(20, 2825, 2048));
        joints.enable(20).unwrap();
        joints.wire().servos[0].held = true;
        joints.move_raw(20, 2600).unwrap();
        joints.wire().servos[0].overload(); // 붙잡힌 채 보호 시간이 지났다
        let state = joints.snapshot(20).unwrap();
        assert!(state.torque_on() && state.status.contains(Fault::OVERLOAD));
        let message = joints.enable(20).unwrap_err();
        assert!(
            message.contains("서보 보호가 걸려 있다 (과부하)"),
            "{message}"
        );
        // 놓아도 묶인 채 목표로 가지 않는다. 사람이 토크를 껐다 켜면 풀린다
        joints.wire().servos[0].release();
        assert_eq!(joints.snapshot(20).unwrap().position, 2048);
        joints.disable(20).unwrap();
        assert_eq!(joints.enable(20).unwrap(), Enabled::TurnedOn);
        let state = joints.snapshot(20).unwrap();
        assert!(state.status.is_none() && state.position == 2048);
    }

    #[test]
    fn low_voltage_blocks_enable() {
        let mut servo = FakeServo::with(20, 2825, 2048);
        servo.set(reg::MIN_VOLTAGE, 100);
        servo.set_voltage(96);
        let mut joints = bench(servo);
        assert!(joints.enable(20).unwrap_err().contains("(전압)"));
        joints.wire().servos[0].set_voltage(120);
        assert_eq!(joints.enable(20).unwrap(), Enabled::TurnedOn);
    }

    #[test]
    fn robot_joint_offset_must_match_the_settings() {
        let mut joint = Settings::parse(TEST_J3)
            .unwrap()
            .joint("j3")
            .unwrap()
            .clone();
        joint.calibration = Some(Calibration {
            pin: 0.0,
            zero_raw: 1500,
            direction: Direction::Same,
            position_offset: 300,
        });
        let servo = Servo::from_joint(&joint).unwrap();
        let mut fake = programmed(FakeServo::with(3, 2825, 1500), &servo);
        fake.set(reg::POSITION_OFFSET, 0);
        let mut joints = Joints::open(open_fake_bus([fake]), vec![servo], BENCH_CAPS).unwrap();
        let message = joints.enable(3).unwrap_err();
        assert!(
            message.contains("위치 오프셋 0이 로봇 설정의 300와"),
            "{message}"
        );
        joints.wire().servos[0].set(reg::POSITION_OFFSET, 300);
        assert_eq!(joints.enable(3).unwrap(), Enabled::TurnedOn);
    }

    #[test]
    fn robot_joint_eeprom_must_match_the_settings() {
        let mut joint = Settings::parse(TEST_J3)
            .unwrap()
            .joint("j3")
            .unwrap()
            .clone();
        joint.calibration = Some(Calibration {
            pin: 0.0,
            zero_raw: 2048,
            direction: Direction::Same,
            position_offset: 0,
        });
        let servo = Servo::from_joint(&joint).unwrap();
        // 각도 한계는 이 서보의 영점과 소프트 한계로 계산한 값이다
        let limit = |register| servo.eeprom.iter().find(|(r, _)| *r == register).unwrap().1;
        assert_eq!(limit(reg::MIN_ANGLE_LIMIT), servo.min_raw - 1);
        assert_eq!(limit(reg::MAX_ANGLE_LIMIT), servo.max_raw + 1);
        // 예비 서보를 갈아 끼우고 값 적용을 빠뜨렸다: 각도 한계와 과부하 보호가 다르다
        let mut fake = programmed(FakeServo::with(3, 2825, 2048), &servo);
        fake.set(reg::MAX_ANGLE_LIMIT, 4095);
        fake.set(reg::OVERLOAD_TORQUE, 100);
        let mut joints = Joints::open(open_fake_bus([fake]), vec![servo], BENCH_CAPS).unwrap();
        let message = joints.enable(3).unwrap_err();
        assert!(
            message.contains("max_angle_limit") && message.contains("overload_torque"),
            "{message}"
        );
        assert!(!joints.snapshot(3).unwrap().torque_on());
    }

    #[test]
    fn loop_caps_keep_up_with_the_run_loop_limits() {
        // 로봇 설정의 시험값: 30°/s, 60°/s², 토크 상한 40 %
        let settings = Settings::parse(TEST_J3).unwrap().run_loop;
        let caps = Caps::for_loop(&settings);
        // 30°/s = 341.3 step/s의 1.5배, 60°/s² = 682.7 step/s²의 1.5배를 100 step/s² 단위로
        assert_eq!(
            caps,
            Caps {
                torque_limit: 400,
                speed: 512,
                acceleration: 11,
            }
        );
        assert!(caps.keeps_up_with(&settings));
        assert!(!BENCH_CAPS.keeps_up_with(&LoopSettings {
            max_speed: 1.0,
            ..settings
        }));
    }

    #[test]
    fn joint_torque_limit_is_used_up_to_the_cap() {
        for (limit, expected) in [
            (BENCH_CAPS.torque_limit - 50, BENCH_CAPS.torque_limit - 50),
            (BENCH_CAPS.torque_limit + 50, BENCH_CAPS.torque_limit),
        ] {
            let mut servo = Servo::bench(20, 2825, 0, 4095);
            servo.torque_limit = Some(limit);
            let mut joints =
                Joints::open(open_fake_bus([FakeServo::new(20)]), vec![servo], BENCH_CAPS).unwrap();
            joints.enable(20).unwrap();
            assert_eq!(joints.snapshot(20).unwrap().torque_limit, expected);
        }
    }

    #[test]
    fn goals_outside_limits_or_without_torque_are_refused() {
        let mut joints = Joints::open(
            open_fake_bus([FakeServo::new(20)]),
            vec![Servo::bench(20, 2825, 1000, 3000)],
            BENCH_CAPS,
        )
        .unwrap();
        assert!(
            joints
                .move_raw(20, 2500)
                .unwrap_err()
                .contains("토크가 꺼져")
        );
        joints.enable(20).unwrap();
        assert!(joints.move_raw(20, 3001).unwrap_err().contains("한계"));
        joints.move_raw(20, 2500).unwrap();
        assert_eq!(joints.snapshot(20).unwrap().position, 2500);
        assert!(joints.limit_torque(20, 201).is_err());
        joints.limit_torque(20, 100).unwrap();
        assert_eq!(joints.snapshot(20).unwrap().torque_limit, 100);
    }

    #[test]
    fn repeated_link_failures_stop_motion_until_resumed() {
        let mut joints = bench(FakeServo::new(20));
        joints.enable(20).unwrap();
        joints.bus.wire().servos[0].faults.silent = true;
        for _ in 0..MAX_FAILURES {
            assert!(joints.snapshot(20).is_err());
        }
        assert!(joints.stopped().is_some());
        joints.bus.wire().servos[0].faults.silent = false;
        assert!(joints.move_raw(20, 2100).unwrap_err().contains("멈춰"));
        joints.resume();
        joints.move_raw(20, 2100).unwrap();
    }

    #[test]
    fn wrong_model_is_refused_at_open() {
        let result = Joints::open(
            open_fake_bus([FakeServo::with(20, 777, 2048)]),
            vec![Servo::bench(20, 2825, 0, 4095)],
            BENCH_CAPS,
        );
        assert!(result.is_err_and(|e| e.contains("STS3250가 아니라 STS3215")));
    }

    #[test]
    fn robot_joints_need_calibration_and_convert_angles() {
        let mut joint = Settings::parse(TEST_J3)
            .unwrap()
            .joint("j3")
            .unwrap()
            .clone();
        assert!(Servo::from_joint(&joint).is_err());
        joint.calibration = Some(Calibration {
            pin: 0.0,
            zero_raw: 1500,
            direction: Direction::Same,
            position_offset: 0,
        });
        let servo = Servo::from_joint(&joint).unwrap();
        assert!(servo.min_raw < 1500 && 1500 < servo.max_raw);
        assert_eq!(servo.raw_at(0.0), Some(1500));
        assert_eq!(servo.raw_at(f64::NAN), None);
        let fake = programmed(FakeServo::with(3, 2825, 1500), &servo);
        let mut joints = Joints::open(open_fake_bus([fake]), vec![servo], BENCH_CAPS).unwrap();
        joints.enable(3).unwrap();
        joints.move_angle(3, 0.5).unwrap();
        let raw = joints.snapshot(3).unwrap().position;
        let angle = joints.servo(3).unwrap().angle_at(raw).unwrap();
        assert!((angle - 0.5).abs() < 0.002, "{angle}");
        assert!(joints.move_angle(3, 3.0).unwrap_err().contains("한계"));
        // 로봇 관절은 게인만 바꿔 보고, 한계와 보호 설정은 맨 서보에서만 바꿔 본다
        joints.trial(3, &[(reg::P_GAIN, 40)]).unwrap();
        let refused = joints.trial(3, &[(reg::P_GAIN, 40), (reg::OVERLOAD_TORQUE, 5)]);
        assert!(refused.unwrap_err().contains("맨 서보에서만"));
        let mut bench = bench(FakeServo::new(20));
        bench.trial(20, &[(reg::OVERLOAD_TORQUE, 5)]).unwrap();
    }
}
