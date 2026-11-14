//! 로봇 설정 (`data/robot.toml`): 관절마다 서보 ID와 모델, 소프트 한계, 영점과 방향, 서보에 구울 EEPROM 값
//! (docs/decisions/0024-servo-bus.md).
//!
//! 컨트롤러가 읽는 설정은 이 파일뿐이다 (docs/decisions/0009-control-architecture.md). 파일의 각도는 도이고
//! 읽으면서 rad로 바꾼다. 모르는 키가 있으면(단위를 빠뜨린 키 이름 등) 읽을 때 실패하고, 문제가 여럿이면
//! 한 번에 모두 알린다. 컨트롤러는 설계 파일을 읽지 않으므로, 설계(`params/arm.toml`)와 맞는지는 PC의
//! 테스트(`tests/test_robot_settings.py`)가 본다.

use std::collections::BTreeMap;
use std::f64::consts::TAU;
use std::fs;
use std::ops::RangeInclusive;
use std::path::Path;

use horilab_servo::feetech::registers::{self as reg, Area, MODELS, Register, STEPS_PER_TURN};
use serde::Deserialize;

use crate::kinematics::Kinematics;

/// 관절 이름. MJCF, URDF, 메시지와 같은 이름이고(docs/conventions.md), 서보 ID는 관절 번호다 (0024).
pub const JOINT_NAMES: [&str; 6] = ["j1", "j2", "j3", "j4", "j5", "j6"];

/// 그리퍼 서보의 ID: 11부터 그리퍼마다 하나씩 다르다 (0024)
const GRIPPER_IDS: RangeInclusive<u8> = 11..=19;

/// 서보 값 한 칸 [rad]
pub const RAD_PER_STEP: f64 = TAU / STEPS_PER_TURN as f64;

/// 서보 값의 끝. 위치 모드는 0~4095 안에서 움직인다.
const MAX_RAW: f64 = (STEPS_PER_TURN - 1) as f64;

/// `[... .registers]` 표에 적지 않는 EEPROM 칸. ID와 통신 속도는 값 적용이 바꾸지 않고(0024), 위치 오프셋은
/// 영점과 같이 관절에 적고(`position_offset`), 각도 한계는 소프트 한계에서 계산한다.
const NOT_IN_FILE: [Register; 5] = [
    reg::ID,
    reg::BAUD_RATE,
    reg::POSITION_OFFSET,
    reg::MIN_ANGLE_LIMIT,
    reg::MAX_ANGLE_LIMIT,
];

/// 읽고 검사한 로봇 설정.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// 서보 값의 0/4095 경계를 소프트 한계에서 떨어뜨릴 각도 [rad]
    pub boundary_clearance: f64,
    /// 실행 루프의 기준값 (0032)
    pub run_loop: LoopSettings,
    /// 팔의 기구학 (0009, 0033)
    pub kinematics: Kinematics,
    /// 관절 번호 순
    pub joints: Vec<Joint>,
    /// 달린 그리퍼 (서보가 있는 도구)
    pub gripper: Option<GripperSettings>,
}

/// 그리퍼 (0030, 0032). 서보는 관절처럼 다룬다: 관절 각은 손가락이 맞닿은 자리(영점)에서 돈 각이고, 소프트
/// 한계는 0 ~ 다 벌림, 토크 한도는 쥐는 힘이다. 열림 폭은 피니언으로 바꾼다.
#[derive(Debug, Clone, PartialEq)]
pub struct GripperSettings {
    /// 그리퍼 서보 (이름은 gripper)
    pub joint: Joint,
    /// 피니언의 피치 반지름 [mm]. 손가락 둘이 대칭으로 움직여 열림 폭 = 2 × 반지름 × 각
    pub pinion_radius: f64,
    /// 다 벌린 열림 폭 [mm]
    pub max_width: f64,
    /// 열고 닫는 빠르기 [mm/s]
    pub speed: f64,
    /// 목표 폭에서 이만큼 안이면 닿은 것이고, 그 전에 막혀 부하가 오르면 잡은 것이다 [mm]
    pub grasp_tolerance: f64,
}

/// 실행 루프의 기준값 (docs/decisions/0032-run-loop.md). 재기 전의 값은 로봇 설정에 시험값이라고 적는다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopSettings {
    /// 관절 속도 한계 [rad/s]
    pub max_speed: f64,
    /// 궤적의 첫 점이 지금 목표에서 이만큼까지 떨어져도 받는다 [rad]
    pub start_tolerance: f64,
    /// 서보 온도가 이 이상이면 알린다 [°C]
    pub temperature_warn: f64,
    /// 서보 온도가 이 이상이면 멈추고 쉰다 [°C]. 쉬는 자세가 정해지기 전에는 그 자리에서 버틴다
    pub temperature_rest: f64,
    /// 서보 전압이 이보다 낮으면 알린다 [V]
    pub voltage_warn: f64,
    /// 처짐 보정의 이득 [1/s]. 0이면 끈다 (0012)
    pub sag_gain: f64,
    /// 처짐 보정의 최대 크기 [rad]
    pub sag_max: f64,
    /// 관절의 가속 한계 [rad/s²] (관절 이동, 조그, 직선 이동, 정지)
    pub max_acceleration: f64,
    /// 관절의 저크 한계 [rad/s³] (관절 이동, 조그, 직선 이동, 정지)
    pub max_jerk: f64,
    /// 직교 조그와 직선 이동의 손끝 속도 한계 [m/s]
    pub max_tool_speed: f64,
    /// 직교 조그와 직선 이동의 손끝 회전 한계 [rad/s]
    pub max_tool_rotation: f64,
}

/// 관절 하나.
#[derive(Debug, Clone, PartialEq)]
pub struct Joint {
    pub name: &'static str,
    /// 서보 ID (관절 번호)
    pub id: u8,
    /// 서보 모델 이름
    pub model: &'static str,
    /// 서보가 알려 주는 모델 번호 (레지스터 3~4). 버스에서 찾은 서보와 맞춰 본다
    pub model_number: u16,
    /// 소프트 한계의 아래 [rad]. 컨트롤러는 소프트 한계 밖으로 움직이지 않는다 (0009)
    pub lower: f64,
    /// 소프트 한계의 위 [rad]
    pub upper: f64,
    /// 영점 핀을 꽂는 자리의 관절 각 [rad]. 설계 값이다 (docs/decisions/0029-joint-parts.md)
    pub pin: f64,
    /// 영점과 방향. 영점 핀으로 재기 전에는 없고, 없으면 이 관절은 움직이지 않는다
    pub calibration: Option<Calibration>,
    /// 토크 한도 [0.1 %] (RAM 48). 없으면 관절 층의 상한을 쓴다 (0027, 0032)
    pub torque_limit: Option<i32>,
    /// 서보에 구울 EEPROM 값 (값 적용의 목표, 주소 순). ID와 통신 속도는 빠진다
    pub eeprom: Vec<(Register, i32)>,
}

/// 서보 값과 관절 각 사이의 변환. 실물 백엔드 안에서만 쓴다 (0009).
///
/// 영점은 영점 핀을 꽂은 자리에서 잰다. 그때 위치 오프셋을 구워 범위 가운데를 서보 값 2048 둘레에 두면,
/// 위치 모드가 넘지 않는 0/4095 경계가 범위의 반대편에 온다 (0024의 영점).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Calibration {
    /// 영점 핀을 꽂는 자리의 관절 각 [rad] (관절의 `pin`)
    pub pin: f64,
    /// 영점 핀을 꽂은 자리의 서보 값 (아래 위치 오프셋을 구운 뒤 읽은 값)
    pub zero_raw: u16,
    pub direction: Direction,
    /// 영점을 잴 때 서보에 구운 위치 오프셋 (EEPROM 31). 값 적용이 이 값을 굽는다
    pub position_offset: i32,
}

/// 관절 각이 늘 때 서보 값이 움직이는 쪽.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// 서보 값도 는다 (파일에는 1)
    Same,
    /// 서보 값이 준다 (파일에는 -1)
    Reversed,
}

impl Direction {
    /// 파일에 적는 값: 1 또는 -1
    pub fn number(self) -> i8 {
        match self {
            Self::Same => 1,
            Self::Reversed => -1,
        }
    }

    fn sign(self) -> f64 {
        f64::from(self.number())
    }
}

impl Calibration {
    /// 관절 각 [rad]에서의 서보 값. 반올림하지 않는다.
    pub fn raw_at(&self, angle: f64) -> f64 {
        f64::from(self.zero_raw) + self.direction.sign() * (angle - self.pin) / RAD_PER_STEP
    }

    /// 서보 값에서의 관절 각 [rad].
    pub fn angle_at(&self, raw: f64) -> f64 {
        self.pin + self.direction.sign() * (raw - f64::from(self.zero_raw)) * RAD_PER_STEP
    }
}

impl Joint {
    /// 소프트 한계에 양쪽 경계 여유를 더한 범위가 서보 값 0~4095 안에 드는, 영점 핀 자리의 서보 값의 범위
    /// (양 끝 포함). 그 범위가 한 바퀴보다 넓어 어떤 값도 안 되면 없다. 영점을 잴 때는 위치 오프셋으로 핀
    /// 자리를 이 범위의 가운데에 맞춘다.
    pub fn zero_window(&self, clearance: f64, direction: Direction) -> Option<(f64, f64)> {
        zero_window(
            self.lower - self.pin,
            self.upper - self.pin,
            clearance,
            direction,
        )
    }
}

fn zero_window(lower: f64, upper: f64, clearance: f64, direction: Direction) -> Option<(f64, f64)> {
    // 영점에서 범위의 양 끝까지 [칸]. 방향이 같으면 서보 값 = 영점 + 칸이다.
    let below = (lower - clearance) / RAD_PER_STEP;
    let above = (upper + clearance) / RAD_PER_STEP;
    let (low, high) = match direction {
        Direction::Same => (-below, MAX_RAW - above),
        Direction::Reversed => (above, MAX_RAW + below),
    };
    (low <= high).then_some((low, high))
}

/// 서보 각도 한계(EEPROM 9, 11): 영점을 쟀으면 소프트 한계를 바깥쪽으로 반올림한 서보 값이고,
/// 재기 전에는 한 바퀴 전체다. 서보는 이 밖의 목표로 가지 않아서 컨트롤러 다음의 막이 된다.
fn angle_limits(lower: f64, upper: f64, calibration: Option<Calibration>) -> (i32, i32) {
    match calibration {
        Some(c) => {
            let (a, b) = (c.raw_at(lower), c.raw_at(upper));
            (a.min(b).floor() as i32, a.max(b).ceil() as i32)
        }
        None => (0, MAX_RAW as i32),
    }
}

/// 서보에 구울 EEPROM 값: 관절의 값, 모델의 값, 메모리 표의 초기값 순으로 찾는다. 위치 오프셋은 영점을 잴 때
/// 정한 값(재기 전에는 0)이고 각도 한계는 계산한 값이다. 어디에도 값이 없는 칸이 있으면 그 이름들을 돌려준다.
fn eeprom_target(
    model: &[(Register, i32)],
    joint: &[(Register, i32)],
    (min_raw, max_raw): (i32, i32),
    position_offset: i32,
) -> Result<Vec<(Register, i32)>, Vec<&'static str>> {
    let find = |values: &[(Register, i32)], register: Register| {
        values
            .iter()
            .find(|(known, _)| *known == register)
            .map(|&(_, value)| value)
    };
    let mut target = Vec::new();
    let mut missing = Vec::new();
    for &register in reg::ALL {
        let settable = register.area == Area::Eeprom && register.writable;
        if !settable || register == reg::ID || register == reg::BAUD_RATE {
            continue;
        }
        let value = if register == reg::POSITION_OFFSET {
            Some(position_offset)
        } else if register == reg::MIN_ANGLE_LIMIT {
            Some(min_raw)
        } else if register == reg::MAX_ANGLE_LIMIT {
            Some(max_raw)
        } else {
            find(joint, register)
                .or_else(|| find(model, register))
                .or_else(|| reg::table_initial(register))
        };
        match value {
            Some(value) => target.push((register, value)),
            None => missing.push(register.name),
        }
    }
    if missing.is_empty() {
        Ok(target)
    } else {
        Err(missing)
    }
}

/// `[... .registers]` 표: 레지스터 이름과 값이 맞는지 본다.
fn check_registers(
    context: &str,
    values: &BTreeMap<String, i64>,
) -> Result<Vec<(Register, i32)>, Vec<String>> {
    let mut checked = Vec::new();
    let mut problems = Vec::new();
    for (name, &value) in values {
        let Some(register) = reg::by_name(name) else {
            problems.push(format!("{context}: 모르는 레지스터다: {name}"));
            continue;
        };
        if register.area != Area::Eeprom || !register.writable {
            problems.push(format!("{context}: {name}은 쓸 수 있는 EEPROM 칸이 아니다"));
        } else if NOT_IN_FILE.contains(&register) {
            problems.push(format!(
                "{context}: {name}은 적지 않는다 (ID와 통신 속도는 바꾸지 않고, 위치 오프셋은 관절의 \
                 position_offset에, 각도 한계는 소프트 한계로 계산한다)"
            ));
        } else {
            match i32::try_from(value)
                .ok()
                .filter(|&value| register.encode(value).is_ok())
            {
                Some(value) => checked.push((register, value)),
                None => problems.push(format!("{context}: {name}에 담을 수 없는 값이다: {value}")),
            }
        }
    }
    if problems.is_empty() {
        Ok(checked)
    } else {
        Err(problems)
    }
}

fn model_names() -> String {
    let names: Vec<&str> = MODELS.iter().map(|(_, name)| *name).collect();
    names.join(", ")
}

impl Settings {
    /// 파일을 읽어 검사한다.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = fs::read_to_string(path)
            .map_err(|e| format!("{}을 읽지 못했다: {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    /// 글을 읽어 검사한다.
    pub fn parse(text: &str) -> Result<Self, String> {
        let file: SettingsFile = toml::from_str(text).map_err(|e| e.to_string())?;
        let clearance_deg = file.boundary_clearance_deg;
        if !(clearance_deg.is_finite() && clearance_deg >= 0.0) {
            return Err(format!(
                "boundary_clearance_deg는 0 이상이다: {clearance_deg}"
            ));
        }
        let clearance = clearance_deg.to_radians();
        let mut problems = Vec::new();
        let run_loop = match file.run_loop.check() {
            Ok(values) => Some(values),
            Err(found) => {
                problems.extend(found);
                None
            }
        };
        let kinematics = match file.kinematics.check() {
            Ok(kinematics) => Some(kinematics),
            Err(problem) => {
                problems.push(problem);
                None
            }
        };

        let mut models: BTreeMap<&str, Vec<(Register, i32)>> = BTreeMap::new();
        for (name, model) in &file.models {
            if !MODELS.iter().any(|(_, known)| *known == name.as_str()) {
                problems.push(format!(
                    "models.{name}: 모르는 서보 모델이다 ({})",
                    model_names()
                ));
                continue;
            }
            match check_registers(&format!("models.{name}.registers"), &model.registers) {
                Ok(values) => {
                    models.insert(name.as_str(), values);
                }
                Err(found) => problems.extend(found),
            }
        }

        if file.joints.is_empty() {
            problems.push("관절이 하나도 없다".to_owned());
        }
        let mut joints = Vec::new();
        for (name, joint) in &file.joints {
            match joint.check(name, clearance, &models) {
                Ok(joint) => joints.push(joint),
                Err(found) => problems.extend(found),
            }
        }
        let gripper = match &file.gripper {
            None => None,
            Some(gripper) => match gripper.check(clearance, &models) {
                Ok(gripper) => Some(gripper),
                Err(found) => {
                    problems.extend(found);
                    None
                }
            },
        };
        if !problems.is_empty() {
            return Err(format!(
                "고칠 것 {}가지\n- {}",
                problems.len(),
                problems.join("\n- ")
            ));
        }
        joints.sort_by_key(|joint| joint.id);
        Ok(Self {
            boundary_clearance: clearance,
            run_loop: run_loop.expect("문제가 없으면 읽었다"),
            kinematics: kinematics.expect("문제가 없으면 읽었다"),
            joints,
            gripper,
        })
    }

    pub fn joint(&self, name: &str) -> Option<&Joint> {
        self.joints.iter().find(|joint| joint.name == name)
    }

    /// 사람이 볼 요약: 관절마다 한 줄.
    pub fn summary(&self) -> String {
        let mut lines = vec![format!(
            "0/4095 경계 여유: 소프트 한계 밖 {}",
            deg(self.boundary_clearance)
        )];
        let r = &self.run_loop;
        let sag = if r.sag_gain > 0.0 {
            format!("이득 {}/s, 최대 {}", r.sag_gain, deg(r.sag_max))
        } else {
            "꺼짐".to_owned()
        };
        lines.push(format!(
            "실행 루프: 속도 한계 {}/s, 가속 한계 {}/s², 저크 한계 {}/s³, 온도 알림 {}°C와 쉬기 {}°C, 전압 알림 {} V, 처짐 보정 {sag}",
            deg(r.max_speed),
            deg(r.max_acceleration),
            deg(r.max_jerk),
            r.temperature_warn,
            r.temperature_rest,
            r.voltage_warn
        ));
        for joint in &self.joints {
            let calibration = match joint.calibration {
                None => "영점과 방향을 아직 재지 않았다 (재기 전에는 움직이지 않는다)".to_owned(),
                Some(c) => {
                    let (a, b) = (c.raw_at(joint.lower), c.raw_at(joint.upper));
                    format!(
                        "핀 자리의 서보 값 {} (위치 오프셋 {}), 방향 {:+}, 소프트 한계의 서보 값 {}~{}",
                        c.zero_raw,
                        c.position_offset,
                        c.direction.number(),
                        a.min(b).round(),
                        a.max(b).round()
                    )
                }
            };
            let torque = joint.torque_limit.map_or_else(String::new, |limit| {
                format!("  토크 한도 {}%", f64::from(limit) / 10.0)
            });
            lines.push(format!(
                "{}  ID {}  {}  소프트 한계 {}~{}  영점 핀 {}  {calibration}{torque}",
                joint.name,
                joint.id,
                joint.model,
                deg(joint.lower),
                deg(joint.upper),
                deg(joint.pin)
            ));
        }
        if let Some(gripper) = &self.gripper {
            let zero = match gripper.joint.calibration {
                None => "닫은 자리를 아직 재지 않았다 (재기 전에는 움직이지 않는다)".to_owned(),
                Some(c) => format!(
                    "닫은 자리의 서보 값 {} (위치 오프셋 {}), 방향 {:+}",
                    c.zero_raw,
                    c.position_offset,
                    c.direction.number()
                ),
            };
            lines.push(format!(
                "그리퍼  ID {}  {}  열림 폭 0~{} mm  쥐는 힘 {}%  {zero}",
                gripper.joint.id,
                gripper.joint.model,
                gripper.max_width,
                f64::from(gripper.joint.torque_limit.unwrap_or_default()) / 10.0
            ));
        }
        lines.join("\n") + "\n"
    }
}

/// 사람이 볼 각도: 소수 한 자리까지 (예: -30°, 12.5°).
fn deg(rad: f64) -> String {
    let degrees = (rad.to_degrees() * 10.0).round() / 10.0 + 0.0; // + 0.0: -0을 0으로
    format!("{degrees}°")
}

/// 파일 그대로의 모양. 읽은 뒤 검사해 [`Settings`]로 바꾼다.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SettingsFile {
    boundary_clearance_deg: f64,
    run_loop: LoopFile,
    #[serde(default)]
    models: BTreeMap<String, ModelFile>,
    joints: BTreeMap<String, JointFile>,
    gripper: Option<GripperFile>,
    kinematics: KinematicsFile,
}

/// 기구학: 영점 자세에서 관절마다 부모 좌표계의 원점 [mm]과 축, 플랜지의 위치 [mm]와 회전.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct KinematicsFile {
    origins_mm: [[f64; 3]; 6],
    axes: [[f64; 3]; 6],
    flange_position_mm: [f64; 3],
    flange_rotation: [[f64; 3]; 3],
}

impl KinematicsFile {
    fn check(&self) -> Result<Kinematics, String> {
        let meters = |v: [f64; 3]| v.map(|x| x * 1e-3);
        Kinematics::new(
            self.origins_mm.map(meters),
            self.axes,
            self.flange_rotation,
            meters(self.flange_position_mm),
        )
        .map_err(|e| format!("kinematics: {e}"))
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LoopFile {
    max_speed_deg_s: f64,
    start_tolerance_deg: f64,
    temperature_warn_c: f64,
    temperature_rest_c: f64,
    voltage_warn_v: f64,
    sag_gain_per_s: f64,
    sag_max_deg: f64,
    max_acceleration_deg_s2: f64,
    max_jerk_deg_s3: f64,
    max_tool_speed_mm_s: f64,
    max_tool_rotation_deg_s: f64,
}

impl LoopFile {
    fn check(&self) -> Result<LoopSettings, Vec<String>> {
        let mut problems = Vec::new();
        let positive = [
            ("max_speed_deg_s", self.max_speed_deg_s),
            ("start_tolerance_deg", self.start_tolerance_deg),
            ("temperature_warn_c", self.temperature_warn_c),
            ("temperature_rest_c", self.temperature_rest_c),
            ("voltage_warn_v", self.voltage_warn_v),
            ("max_acceleration_deg_s2", self.max_acceleration_deg_s2),
            ("max_jerk_deg_s3", self.max_jerk_deg_s3),
            ("max_tool_speed_mm_s", self.max_tool_speed_mm_s),
            ("max_tool_rotation_deg_s", self.max_tool_rotation_deg_s),
        ];
        for (name, value) in positive {
            if !(value.is_finite() && value > 0.0) {
                problems.push(format!("run_loop.{name}는 0보다 크다: {value}"));
            }
        }
        for (name, value) in [
            ("sag_gain_per_s", self.sag_gain_per_s),
            ("sag_max_deg", self.sag_max_deg),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                problems.push(format!("run_loop.{name}는 0 이상이다: {value}"));
            }
        }
        if self.temperature_rest_c <= self.temperature_warn_c {
            problems.push(format!(
                "run_loop.temperature_rest_c({})는 temperature_warn_c({})보다 높다",
                self.temperature_rest_c, self.temperature_warn_c
            ));
        }
        if !problems.is_empty() {
            return Err(problems);
        }
        Ok(LoopSettings {
            max_speed: self.max_speed_deg_s.to_radians(),
            start_tolerance: self.start_tolerance_deg.to_radians(),
            temperature_warn: self.temperature_warn_c,
            temperature_rest: self.temperature_rest_c,
            voltage_warn: self.voltage_warn_v,
            sag_gain: self.sag_gain_per_s,
            sag_max: self.sag_max_deg.to_radians(),
            max_acceleration: self.max_acceleration_deg_s2.to_radians(),
            max_jerk: self.max_jerk_deg_s3.to_radians(),
            max_tool_speed: self.max_tool_speed_mm_s * 1e-3,
            max_tool_rotation: self.max_tool_rotation_deg_s.to_radians(),
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GripperFile {
    id: u8,
    model: String,
    pinion_radius_mm: f64,
    max_width_mm: f64,
    force_percent: f64,
    speed_mm_s: f64,
    grasp_tolerance_mm: f64,
    zero_raw: Option<u16>,
    direction: Option<i8>,
    position_offset: Option<i32>,
    #[serde(default)]
    registers: BTreeMap<String, i64>,
}

impl GripperFile {
    fn check(
        &self,
        clearance: f64,
        models: &BTreeMap<&str, Vec<(Register, i32)>>,
    ) -> Result<GripperSettings, Vec<String>> {
        let mut problems = Vec::new();
        for (key, value) in [
            ("pinion_radius_mm", self.pinion_radius_mm),
            ("max_width_mm", self.max_width_mm),
            ("speed_mm_s", self.speed_mm_s),
            ("grasp_tolerance_mm", self.grasp_tolerance_mm),
        ] {
            if !(value.is_finite() && value > 0.0) {
                problems.push(format!("gripper.{key}는 0보다 크다: {value}"));
            }
        }
        if !problems.is_empty() {
            return Err(problems);
        }
        // 서보는 관절처럼 검사한다: 각 0(손가락이 맞닿은 자리) ~ 다 벌린 각, 토크 한도는 쥐는 힘
        let open = self.max_width_mm / (2.0 * self.pinion_radius_mm);
        let servo = JointFile {
            id: self.id,
            model: self.model.clone(),
            soft_limits_deg: [0.0, open.to_degrees()],
            pin_deg: 0.0,
            zero_raw: self.zero_raw,
            direction: self.direction,
            position_offset: self.position_offset,
            torque_limit_percent: Some(self.force_percent),
            registers: self.registers.clone(),
        };
        let rule = format!("{}~{} 가운데 하나", GRIPPER_IDS.start(), GRIPPER_IDS.end());
        let joint = servo.check_servo("gripper", (GRIPPER_IDS, rule), clearance, models)?;
        // 쥔 채 버티는 힘이 과부하 기준에 닿으면 서보 보호가 걸려 힘이 빠진다 (0024)
        let overload = joint
            .eeprom
            .iter()
            .find(|(register, _)| *register == reg::OVERLOAD_TORQUE)
            .map(|&(_, value)| value);
        if let Some(overload) = overload
            && self.force_percent >= f64::from(overload)
        {
            return Err(vec![format!(
                "gripper: force_percent {}는 과부하 기준 {overload} %보다 낮아야 한다 \
                 (쥔 채 버티면 서보 보호가 걸린다)",
                self.force_percent
            )]);
        }
        Ok(GripperSettings {
            joint,
            pinion_radius: self.pinion_radius_mm,
            max_width: self.max_width_mm,
            speed: self.speed_mm_s,
            grasp_tolerance: self.grasp_tolerance_mm,
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ModelFile {
    #[serde(default)]
    registers: BTreeMap<String, i64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct JointFile {
    id: u8,
    model: String,
    soft_limits_deg: [f64; 2],
    pin_deg: f64,
    zero_raw: Option<u16>,
    direction: Option<i8>,
    position_offset: Option<i32>,
    torque_limit_percent: Option<f64>,
    #[serde(default)]
    registers: BTreeMap<String, i64>,
}

impl JointFile {
    fn check(
        &self,
        name: &str,
        clearance: f64,
        models: &BTreeMap<&str, Vec<(Register, i32)>>,
    ) -> Result<Joint, Vec<String>> {
        let Some(index) = JOINT_NAMES.iter().position(|known| *known == name) else {
            return Err(vec![format!(
                "{name}: 모르는 관절이다 ({})",
                JOINT_NAMES.join(", ")
            )]);
        };
        let number = u8::try_from(index + 1).unwrap_or(u8::MAX);
        let rule = format!("관절 번호와 같은 {number}");
        self.check_servo(
            JOINT_NAMES[index],
            (number..=number, rule),
            clearance,
            models,
        )
    }

    /// 서보 하나의 설정을 검사한다 (관절과 그리퍼). `ids`는 받는 ID의 범위와 그 설명이다.
    fn check_servo(
        &self,
        name: &'static str,
        (ids, id_rule): (RangeInclusive<u8>, String),
        clearance: f64,
        models: &BTreeMap<&str, Vec<(Register, i32)>>,
    ) -> Result<Joint, Vec<String>> {
        let mut problems = Vec::new();

        if !ids.contains(&self.id) {
            problems.push(format!("{name}: id는 {id_rule}이다: {}", self.id));
        }

        let model = MODELS.iter().find(|(_, model)| *model == self.model);
        if model.is_none() {
            problems.push(format!(
                "{name}: 모르는 서보 모델이다: {} ({})",
                self.model,
                model_names()
            ));
        }

        let [lower_deg, upper_deg] = self.soft_limits_deg;
        let (lower, upper) = (lower_deg.to_radians(), upper_deg.to_radians());
        let ordered = lower_deg.is_finite() && upper_deg.is_finite() && lower_deg < upper_deg;
        let fits = ordered && zero_window(lower, upper, clearance, Direction::Same).is_some();
        if !ordered {
            problems.push(format!(
                "{name}: soft_limits_deg는 [아래, 위]이고 아래 < 위다: \
                 [{lower_deg}, {upper_deg}]"
            ));
        } else if !fits {
            problems.push(format!(
                "{name}: 소프트 한계 {}~{}에 양쪽 경계 여유 {}를 더하면 \
                 한 바퀴(360°)를 넘는다",
                deg(lower),
                deg(upper),
                deg(clearance)
            ));
        }
        let limits_ok = fits;
        let pin = self.pin_deg.to_radians();
        if ordered && !(lower_deg..=upper_deg).contains(&self.pin_deg) {
            problems.push(format!(
                "{name}: pin_deg(영점 핀을 꽂는 자리의 관절 각)는 소프트 한계 안이다: {}",
                self.pin_deg
            ));
        }

        let calibration = match (self.zero_raw, self.direction, self.position_offset) {
            (None, None, None) => None,
            (Some(zero_raw), Some(sign), Some(position_offset)) => {
                let direction = match sign {
                    1 => Some(Direction::Same),
                    -1 => Some(Direction::Reversed),
                    _ => {
                        problems.push(format!("{name}: direction은 1이나 -1이다: {sign}"));
                        None
                    }
                };
                let zero_ok = u32::from(zero_raw) < STEPS_PER_TURN;
                if !zero_ok {
                    problems.push(format!("{name}: zero_raw는 0~4095다: {zero_raw}"));
                }
                if reg::POSITION_OFFSET.encode(position_offset).is_err() {
                    problems.push(format!(
                        "{name}: position_offset에 담을 수 없는 값이다: {position_offset}"
                    ));
                }
                direction.map(|direction| {
                    (
                        Calibration {
                            pin,
                            zero_raw,
                            direction,
                            position_offset,
                        },
                        zero_ok,
                    )
                })
            }
            _ => {
                problems.push(format!(
                    "{name}: zero_raw, direction, position_offset는 같이 적는다 (셋 다 영점 핀으로 잴 때 정한다)"
                ));
                None
            }
        };
        // 영점과 경계: 한계와 영점 값이 제대로일 때만 본다 (아니면 위에서 이미 알렸다)
        if let Some((c, true)) = calibration
            && limits_ok
            && let Some((low, high)) = zero_window(lower - pin, upper - pin, clearance, c.direction)
            && !(low..=high).contains(&f64::from(c.zero_raw))
        {
            problems.push(format!(
                "{name}: 영점 {}, 방향 {:+}이면 서보 값의 0/4095 경계가 소프트 한계에서 {} 안에 든다. \
                 영점이 {}~{}여야 한다 (가운데 {}). 영점 핀을 꽂고 위치 오프셋으로 핀 자리를 가운데에 맞춰 \
                 영점을 다시 잰다 (0024의 영점)",
                c.zero_raw,
                c.direction.number(),
                deg(clearance),
                low.ceil(),
                high.floor(),
                ((low + high) / 2.0).round()
            ));
        }

        let torque_limit = match self.torque_limit_percent {
            None => None,
            Some(percent) if percent.is_finite() && percent > 0.0 && percent <= 100.0 => {
                Some((percent * 10.0).round() as i32)
            }
            Some(percent) => {
                problems.push(format!(
                    "{name}: torque_limit_percent는 0보다 크고 100 이하다: {percent}"
                ));
                None
            }
        };

        let joint_values = check_registers(&format!("joints.{name}.registers"), &self.registers)
            .unwrap_or_else(|found| {
                problems.extend(found);
                Vec::new()
            });

        let Some(&(model_number, model)) = model else {
            return Err(problems);
        };
        if !problems.is_empty() {
            return Err(problems);
        }
        let calibration = calibration.map(|(c, _)| c);
        let model_values = models.get(model).map_or(&[][..], Vec::as_slice);
        let eeprom = eeprom_target(
            model_values,
            &joint_values,
            angle_limits(lower, upper, calibration),
            calibration.map_or(0, |c| c.position_offset),
        )
        .map_err(|missing| {
            vec![format!(
                "{name}: 메모리 표가 초기값을 비워 둔 칸은 [models.{model}.registers]에 적는다: {}",
                missing.join(", ")
            )]
        })?;
        Ok(Joint {
            name,
            id: self.id,
            model,
            model_number,
            lower,
            upper,
            pin,
            calibration,
            torque_limit,
            eeprom,
        })
    }
}

/// 시험용 로봇 설정: 관절 하나 시험의 J3 (data/robot.toml과 같은 값). 끝에 키를 붙이면 j3에 들어간다.
#[cfg(test)]
pub(crate) const TEST_J3: &str = r#"
boundary_clearance_deg = 15

[run_loop]
max_speed_deg_s = 30
start_tolerance_deg = 0.5
temperature_warn_c = 50
temperature_rest_c = 55
voltage_warn_v = 11.0
sag_gain_per_s = 0
sag_max_deg = 3
max_acceleration_deg_s2 = 60
max_jerk_deg_s3 = 300
max_tool_speed_mm_s = 30
max_tool_rotation_deg_s = 20

[kinematics]
origins_mm = [[0, 0, 0], [0, 0, 160], [0, 0, 220], [0, 0, 180], [0, 63, 0], [0, 0, 63]]
axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 1, 0]]
flange_position_mm = [0, 50, 0]
flange_rotation = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]

[models.STS3250.registers]
response_delay = 0
max_voltage = 160
phase = 12
unload_condition = 45
led_alarm_condition = 45
p_gain = 32
d_gain = 32
min_starting_torque = 0
speed_p_gain = 10
speed_i_gain = 200
max_temperature = 80
min_voltage = 60
dead_zone_positive = 0
dead_zone_negative = 0
protection_current = 310
overcurrent_protection_time = 250

[joints.j3]
id = 3
model = "STS3250"
soft_limits_deg = [-30, 118]
pin_deg = 0
"#;

/// 시험용 그리퍼 (가짜 서보가 STS3250이라 같은 모델로 둔다)
#[cfg(test)]
pub(crate) const TEST_GRIPPER: &str = r#"
[gripper]
id = 11
model = "STS3250"
pinion_radius_mm = 12.5
max_width_mm = 60
force_percent = 15
speed_mm_s = 20
grasp_tolerance_mm = 1.0
zero_raw = 2048
direction = 1
position_offset = 0
"#;

#[cfg(test)]
mod tests {
    use super::*;

    const J3: &str = TEST_J3;

    fn j3_with(lines: &str) -> String {
        format!("{J3}{lines}\n")
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn error(text: &str) -> String {
        Settings::parse(text).expect_err("읽히면 안 된다")
    }

    #[test]
    fn reads_a_joint_before_calibration() {
        let settings = Settings::parse(J3).unwrap();
        assert!(close(settings.boundary_clearance, 15f64.to_radians()));
        assert_eq!(settings.joints.len(), 1);
        let j3 = settings.joint("j3").unwrap();
        assert_eq!(
            (j3.name, j3.id, j3.model, j3.model_number),
            ("j3", 3, "STS3250", 2825)
        );
        assert!(close(j3.lower, (-30f64).to_radians()));
        assert!(close(j3.upper, 118f64.to_radians()));
        assert!(close(j3.pin, 0.0));
        assert_eq!(j3.torque_limit, None);
        let r = settings.run_loop;
        assert!(close(r.max_speed, 30f64.to_radians()) && close(r.sag_max, 3f64.to_radians()));
        assert_eq!(
            (r.temperature_warn, r.temperature_rest, r.sag_gain),
            (50.0, 55.0, 0.0)
        );
        assert_eq!(j3.calibration, None);
        assert!(settings.joint("j2").is_none());
    }

    #[test]
    fn calibration_converts_both_ways() {
        let settings = Settings::parse(&j3_with(
            "zero_raw = 1547\ndirection = 1\nposition_offset = 0",
        ))
        .unwrap();
        let c = settings.joint("j3").unwrap().calibration.unwrap();
        assert_eq!(c.direction, Direction::Same);
        assert!(close(c.raw_at(0.0), 1547.0));
        assert!(close(c.raw_at(90f64.to_radians()), 1547.0 + 1024.0));
        assert!(close(c.angle_at(1547.0 - 2048.0), -std::f64::consts::PI));

        let settings = Settings::parse(&j3_with(
            "zero_raw = 2548\ndirection = -1\nposition_offset = 0",
        ))
        .unwrap();
        let c = settings.joint("j3").unwrap().calibration.unwrap();
        assert_eq!(c.direction, Direction::Reversed);
        assert!(close(c.raw_at(90f64.to_radians()), 2548.0 - 1024.0));
        for raw in [0.0, 1000.0, 2548.0, 4095.0] {
            assert!(close(c.raw_at(c.angle_at(raw)), raw));
        }
    }

    #[test]
    fn zero_window_keeps_the_boundary_away() {
        // J3 -30°~118°, 여유 15° → -45°~133°가 0~4095 안에 들어야 한다 (한 칸 = 360/4096°)
        let settings = Settings::parse(J3).unwrap();
        let j3 = settings.joint("j3").unwrap();
        let steps = |degrees: f64| degrees * 4096.0 / 360.0;
        let (low, high) = j3
            .zero_window(settings.boundary_clearance, Direction::Same)
            .unwrap();
        assert!(close(low, steps(45.0)) && close(high, 4095.0 - steps(133.0)));
        let (low, high) = j3
            .zero_window(settings.boundary_clearance, Direction::Reversed)
            .unwrap();
        assert!(close(low, steps(133.0)) && close(high, 4095.0 - steps(45.0)));
        // 창의 양 끝(약 512~2581)은 되고, 한 칸 밖은 안 된다
        let (low, high) = j3
            .zero_window(settings.boundary_clearance, Direction::Same)
            .unwrap();
        let (first, last) = (low.ceil() as i32, high.floor() as i32);
        assert!(
            (511..=513).contains(&first) && last == 2581,
            "{first} {last}"
        );
        for zero in [first, last] {
            let text = j3_with(&format!(
                "zero_raw = {zero}\ndirection = 1\nposition_offset = 0"
            ));
            assert!(Settings::parse(&text).is_ok(), "{zero}");
        }
        for zero in [first - 1, last + 1] {
            let text = j3_with(&format!(
                "zero_raw = {zero}\ndirection = 1\nposition_offset = 0"
            ));
            assert!(Settings::parse(&text).is_err(), "{zero}");
        }
    }

    #[test]
    fn zero_near_the_boundary_is_refused_with_the_window() {
        let message = error(&j3_with(
            "zero_raw = 3000\ndirection = 1\nposition_offset = 0",
        ));
        assert!(message.contains("여야 한다 (가운데 1547)"), "{message}");
        let message = error(&j3_with(
            "zero_raw = 1000\ndirection = -1\nposition_offset = 0",
        ));
        assert!(message.contains("여야 한다 (가운데 2548)"), "{message}");
    }

    #[test]
    fn unknown_keys_are_refused() {
        // 단위를 빠뜨린 키, 틀린 이름, 모르는 위층 키
        for text in [
            J3.replace("soft_limits_deg", "soft_limits"),
            j3_with("zero = 1500"),
            format!("speed = 1\n{J3}"),
        ] {
            let message = error(&text);
            assert!(message.contains("unknown field"), "{message}");
        }
    }

    #[test]
    fn id_must_be_the_joint_number() {
        let message = error(&J3.replace("id = 3", "id = 4"));
        assert!(
            message.contains("id는 관절 번호와 같은 3이다: 4"),
            "{message}"
        );
    }

    #[test]
    fn model_and_joint_must_be_known() {
        let message = error(&J3.replace(r#"model = "STS3250""#, r#"model = "XL430""#));
        assert!(message.contains("모르는 서보 모델이다: XL430"), "{message}");
        let message = error(&J3.replace("joints.j3", "joints.j7"));
        assert!(message.contains("j7: 모르는 관절이다"), "{message}");
        let message = error(&J3.replace("[models.STS3250", "[models.XL430"));
        assert!(
            message.contains("models.XL430: 모르는 서보 모델이다"),
            "{message}"
        );
    }

    #[test]
    fn limits_must_be_ordered_and_leave_room_for_the_boundary() {
        for limits in ["[118, -30]", "[0, 0]", "[nan, 118]"] {
            let message = error(&J3.replace("[-30, 118]", limits));
            assert!(message.contains("아래 < 위"), "{limits}: {message}");
        }
        // 폭 331°에 여유 15° 둘을 더하면 361°
        let message = error(&J3.replace("[-30, 118]", "[-165, 166]"));
        assert!(message.contains("한 바퀴(360°)를 넘는다"), "{message}");
        assert!(Settings::parse(&J3.replace("[-30, 118]", "[-165, 164]")).is_ok());
    }

    #[test]
    fn zero_and_direction_come_together_and_in_range() {
        let message = error(&j3_with("zero_raw = 1500"));
        assert!(message.contains("같이 적는다"), "{message}");
        let message = error(&j3_with("direction = 1"));
        assert!(message.contains("같이 적는다"), "{message}");
        let message = error(&j3_with(
            "zero_raw = 1500\ndirection = 2\nposition_offset = 0",
        ));
        assert!(message.contains("direction은 1이나 -1이다: 2"), "{message}");
        let message = error(&j3_with(
            "zero_raw = 4096\ndirection = 1\nposition_offset = 0",
        ));
        assert!(message.contains("zero_raw는 0~4095다: 4096"), "{message}");
    }

    #[test]
    fn clearance_must_not_be_negative() {
        let message =
            error(&J3.replace("boundary_clearance_deg = 15", "boundary_clearance_deg = -1"));
        assert!(message.contains("0 이상"), "{message}");
    }

    #[test]
    fn every_problem_is_reported_at_once() {
        let text = J3
            .replace("id = 3", "id = 4")
            .replace(r#"model = "STS3250""#, r#"model = "XL430""#);
        let message = error(&(text + "zero_raw = 3000\ndirection = 1\nposition_offset = 0\n"));
        assert!(message.starts_with("고칠 것 3가지"), "{message}");
        assert!(message.contains("여야 한다 (가운데 1547)"), "{message}");
        let head = J3.split("[models").next().unwrap();
        let message = error(&format!("{head}[joints]\n"));
        assert!(message.contains("관절이 하나도 없다"), "{message}");
    }

    #[test]
    fn eeprom_target_takes_joint_then_model_then_table() {
        let settings = Settings::parse(&j3_with("\n[joints.j3.registers]\np_gain = 40")).unwrap();
        let eeprom = &settings.joint("j3").unwrap().eeprom;
        let value = |register: Register| {
            eeprom
                .iter()
                .find(|(known, _)| *known == register)
                .map(|&(_, value)| value)
        };
        assert_eq!(value(reg::P_GAIN), Some(40)); // 관절
        assert_eq!(value(reg::MAX_TEMPERATURE), Some(80)); // 모델
        assert_eq!(value(reg::MAX_TORQUE), Some(1000)); // 메모리 표
        assert_eq!(value(reg::POSITION_OFFSET), Some(0));
        assert_eq!(value(reg::MIN_ANGLE_LIMIT), Some(0)); // 영점을 재기 전에는 한 바퀴
        assert_eq!(value(reg::MAX_ANGLE_LIMIT), Some(4095));
        assert_eq!((value(reg::ID), value(reg::BAUD_RATE)), (None, None));
        let settable = reg::ALL
            .iter()
            .filter(|register| register.area == Area::Eeprom && register.writable)
            .count();
        assert_eq!(eeprom.len(), settable - 2);
    }

    #[test]
    fn angle_limits_follow_the_soft_limits_after_calibration() {
        let settings = Settings::parse(&j3_with(
            "zero_raw = 1547\ndirection = 1\nposition_offset = 0",
        ))
        .unwrap();
        let eeprom = &settings.joint("j3").unwrap().eeprom;
        // -30° → 1205.7, 118° → 2889.6을 바깥쪽으로
        assert!(eeprom.contains(&(reg::MIN_ANGLE_LIMIT, 1205)));
        assert!(eeprom.contains(&(reg::MAX_ANGLE_LIMIT, 2890)));
        let settings = Settings::parse(&j3_with(
            "zero_raw = 2548\ndirection = -1\nposition_offset = 0",
        ))
        .unwrap();
        let eeprom = &settings.joint("j3").unwrap().eeprom;
        // 118° → 2548 - 1342.6 = 1205.4, -30° → 2548 + 341.3 = 2889.3
        assert!(eeprom.contains(&(reg::MIN_ANGLE_LIMIT, 1205)));
        assert!(eeprom.contains(&(reg::MAX_ANGLE_LIMIT, 2890)));
    }

    #[test]
    fn position_offset_comes_with_the_zero() {
        let text = j3_with("zero_raw = 1547\ndirection = 1\nposition_offset = -300");
        let settings = Settings::parse(&text).unwrap();
        let eeprom = &settings.joint("j3").unwrap().eeprom;
        assert!(eeprom.contains(&(reg::POSITION_OFFSET, -300)));
        let message = error(&j3_with(
            "zero_raw = 1547\ndirection = 1\nposition_offset = 5000",
        ));
        assert!(
            message.contains("position_offset에 담을 수 없는 값이다: 5000"),
            "{message}"
        );
        let message = error(&j3_with("zero_raw = 1547\ndirection = 1"));
        assert!(message.contains("같이 적는다"), "{message}");
    }

    #[test]
    fn pin_angle_moves_the_zero() {
        // 핀을 관절 각 90°에서 꽂으면 핀 자리의 서보 값이 90°의 값이다
        let text = J3.replace("pin_deg = 0", "pin_deg = 90")
            + "zero_raw = 2571\ndirection = 1\nposition_offset = 0\n";
        let settings = Settings::parse(&text).unwrap();
        let j3 = settings.joint("j3").unwrap();
        let c = j3.calibration.unwrap();
        assert!(close(c.raw_at(90f64.to_radians()), 2571.0));
        assert!(close(c.raw_at(0.0), 2571.0 - 1024.0));
        assert!(close(c.angle_at(2571.0), 90f64.to_radians()));
        // 창의 가운데는 범위 가운데(44°)가 2047.5에 오는 핀 자리의 값이다: 2047.5 + 46° = 2570.9
        let (low, high) = j3
            .zero_window(settings.boundary_clearance, Direction::Same)
            .unwrap();
        assert_eq!(((low + high) / 2.0).round(), 2571.0);
        let message = error(&J3.replace("pin_deg = 0", "pin_deg = 120"));
        assert!(message.contains("pin_deg"), "{message}");
    }

    #[test]
    fn run_loop_values_are_checked() {
        let message = error(&J3.replace("temperature_rest_c = 55", "temperature_rest_c = 45"));
        assert!(
            message.contains("temperature_warn_c(50)보다 높다"),
            "{message}"
        );
        let message = error(&J3.replace("max_speed_deg_s = 30", "max_speed_deg_s = -1"));
        assert!(
            message.contains("max_speed_deg_s는 0보다 크다"),
            "{message}"
        );
        let message = error(&J3.replace("sag_gain_per_s = 0", "sag_gain_per_s = -1"));
        assert!(message.contains("sag_gain_per_s는 0 이상이다"), "{message}");
    }

    #[test]
    fn kinematics_are_read_and_checked() {
        let settings = Settings::parse(J3).unwrap();
        let pose = settings.kinematics.forward(&[0.0; 6]);
        assert!(close(pose.position[1], 0.113) && close(pose.position[2], 0.623));
        assert!(close(settings.run_loop.max_jerk, 300f64.to_radians()));
        let message = error(&J3.replace("axes = [[0, 0, 1],", "axes = [[1, 0, 0],"));
        assert!(message.contains("kinematics: J1의 축"), "{message}");
        let message = error(&J3.replace("max_jerk_deg_s3 = 300", "max_jerk_deg_s3 = 0"));
        assert!(
            message.contains("max_jerk_deg_s3는 0보다 크다"),
            "{message}"
        );
    }

    #[test]
    fn torque_limit_is_a_percent() {
        let settings = Settings::parse(&j3_with("torque_limit_percent = 45")).unwrap();
        assert_eq!(settings.joint("j3").unwrap().torque_limit, Some(450));
        assert!(settings.summary().contains("토크 한도 45%"));
        let message = error(&j3_with("torque_limit_percent = 120"));
        assert!(message.contains("100 이하다: 120"), "{message}");
    }

    #[test]
    fn gripper_is_read_as_a_servo_with_width() {
        let settings = Settings::parse(&format!("{J3}{TEST_GRIPPER}")).unwrap();
        let gripper = settings.gripper.unwrap();
        assert_eq!((gripper.joint.name, gripper.joint.id), ("gripper", 11));
        assert!(close(gripper.joint.upper, 60.0 / 25.0)); // 2.4 rad = 137.5°
        assert_eq!(gripper.joint.torque_limit, Some(150));
        assert!(Settings::parse(J3).unwrap().gripper.is_none());
        let message = error(&format!(
            "{J3}{}",
            TEST_GRIPPER.replace("id = 11", "id = 5")
        ));
        assert!(
            message.contains("gripper: id는 11~19 가운데 하나이다: 5"),
            "{message}"
        );
        // 쥐는 힘이 과부하 기준(메모리 표 80 %)에 닿으면 쥔 채 버티지 못한다
        let text = TEST_GRIPPER.replace("force_percent = 15", "force_percent = 80");
        let message = error(&format!("{J3}{text}"));
        assert!(message.contains("과부하 기준 80 %보다 낮아야"), "{message}");
    }

    #[test]
    fn cells_without_a_value_are_named() {
        let message = error(&J3.replace("p_gain = 32\n", ""));
        assert!(
            message.contains("[models.STS3250.registers]에 적는다: p_gain"),
            "{message}"
        );
    }

    #[test]
    fn register_values_are_checked() {
        for (line, expected) in [
            ("torque_limit = 500", "쓸 수 있는 EEPROM 칸이 아니다"),
            ("position_offset = 0", "적지 않는다"),
            ("goal = 1", "모르는 레지스터다: goal"),
            ("p_gain = 300", "담을 수 없는 값이다: 300"),
        ] {
            let message = error(&j3_with(&format!("\n[joints.j3.registers]\n{line}")));
            assert!(message.contains(expected), "{line}: {message}");
        }
    }

    #[test]
    fn summary_shows_each_joint() {
        let summary = Settings::parse(J3).unwrap().summary();
        assert!(summary.contains(
            "j3  ID 3  STS3250  소프트 한계 -30°~118°  영점 핀 0°  영점과 방향을 아직 재지 않았다"
        ));
        let settings = Settings::parse(&j3_with(
            "zero_raw = 1547\ndirection = 1\nposition_offset = 0",
        ))
        .unwrap();
        // -30° → 1547 - 341.3, 118° → 1547 + 1342.6
        assert!(settings.summary().contains(
            "핀 자리의 서보 값 1547 (위치 오프셋 0), 방향 +1, 소프트 한계의 서보 값 1206~2890"
        ));
    }
}
