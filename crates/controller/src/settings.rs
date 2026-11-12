//! 로봇 설정 (`data/robot.toml`): 관절마다 서보 ID와 모델, 소프트 한계, 영점과 방향 (docs/decisions/0024-servo-bus.md).
//!
//! 컨트롤러가 읽는 설정은 이 파일뿐이다 (docs/decisions/0009-control-architecture.md). 파일의 각도는 도이고
//! 읽으면서 rad로 바꾼다. 모르는 키가 있으면(단위를 빠뜨린 키 이름 등) 읽을 때 실패하고, 문제가 여럿이면
//! 한 번에 모두 알린다. 컨트롤러는 설계 파일을 읽지 않으므로, 설계(`params/arm.toml`)와 맞는지는 PC의
//! 테스트(`tests/test_robot_settings.py`)가 본다.

use std::collections::BTreeMap;
use std::f64::consts::TAU;
use std::fs;
use std::path::Path;

use horilab_servo::feetech::registers::{MODELS, STEPS_PER_TURN};
use serde::Deserialize;

/// 관절 이름. MJCF, URDF, 메시지와 같은 이름이고(docs/conventions.md), 서보 ID는 관절 번호다 (0024).
pub const JOINT_NAMES: [&str; 6] = ["j1", "j2", "j3", "j4", "j5", "j6"];

/// 서보 값 한 칸 [rad]
pub const RAD_PER_STEP: f64 = TAU / STEPS_PER_TURN as f64;

/// 서보 값의 끝. 위치 모드는 0~4095 안에서 움직인다.
const MAX_RAW: f64 = (STEPS_PER_TURN - 1) as f64;

/// 읽고 검사한 로봇 설정.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// 서보 값의 0/4095 경계를 소프트 한계에서 떨어뜨릴 각도 [rad]
    pub boundary_clearance: f64,
    /// 관절 번호 순
    pub joints: Vec<Joint>,
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
    /// 영점과 방향. 관절 시험의 영점 단계에서 재기 전에는 없고, 없으면 이 관절은 움직이지 않는다
    pub calibration: Option<Calibration>,
}

/// 서보 값과 관절 각 사이의 변환. 실물 백엔드 안에서만 쓴다 (0009).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// 영점 핀을 꽂은 자리의 서보 값 (위치 오프셋 0에서 읽은 값)
    pub zero_raw: u16,
    pub direction: Direction,
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
        f64::from(self.zero_raw) + self.direction.sign() * angle / RAD_PER_STEP
    }

    /// 서보 값에서의 관절 각 [rad].
    pub fn angle_at(&self, raw: f64) -> f64 {
        self.direction.sign() * (raw - f64::from(self.zero_raw)) * RAD_PER_STEP
    }
}

impl Joint {
    /// 소프트 한계에 양쪽 경계 여유를 더한 범위가 서보 값 0~4095 안에 드는 영점의 범위 (양 끝 포함).
    /// 그 범위가 한 바퀴보다 넓어 어떤 영점도 안 되면 없다.
    pub fn zero_window(&self, clearance: f64, direction: Direction) -> Option<(f64, f64)> {
        zero_window(self.lower, self.upper, clearance, direction)
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
        if file.joints.is_empty() {
            problems.push("관절이 하나도 없다".to_owned());
        }
        let mut joints = Vec::new();
        for (name, joint) in &file.joints {
            match joint.check(name, clearance) {
                Ok(joint) => joints.push(joint),
                Err(found) => problems.extend(found),
            }
        }
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
            joints,
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
        for joint in &self.joints {
            let calibration = match joint.calibration {
                None => "영점과 방향을 아직 재지 않았다 (재기 전에는 움직이지 않는다)".to_owned(),
                Some(c) => {
                    let (a, b) = (c.raw_at(joint.lower), c.raw_at(joint.upper));
                    format!(
                        "영점 {}, 방향 {:+}, 소프트 한계의 서보 값 {}~{}",
                        c.zero_raw,
                        c.direction.number(),
                        a.min(b).round(),
                        a.max(b).round()
                    )
                }
            };
            lines.push(format!(
                "{}  ID {}  {}  소프트 한계 {}~{}  {calibration}",
                joint.name,
                joint.id,
                joint.model,
                deg(joint.lower),
                deg(joint.upper)
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
    joints: BTreeMap<String, JointFile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct JointFile {
    id: u8,
    model: String,
    soft_limits_deg: [f64; 2],
    zero_raw: Option<u16>,
    direction: Option<i8>,
}

impl JointFile {
    fn check(&self, name: &str, clearance: f64) -> Result<Joint, Vec<String>> {
        let Some(index) = JOINT_NAMES.iter().position(|known| *known == name) else {
            return Err(vec![format!(
                "{name}: 모르는 관절이다 ({})",
                JOINT_NAMES.join(", ")
            )]);
        };
        let name = JOINT_NAMES[index];
        let mut problems = Vec::new();

        let number = index + 1;
        if usize::from(self.id) != number {
            problems.push(format!(
                "{name}: id는 관절 번호와 같은 {number}이다: {}",
                self.id
            ));
        }

        let model = MODELS.iter().find(|(_, model)| *model == self.model);
        if model.is_none() {
            let known: Vec<&str> = MODELS.iter().map(|(_, model)| *model).collect();
            problems.push(format!(
                "{name}: 모르는 서보 모델이다: {} ({})",
                self.model,
                known.join(", ")
            ));
        }

        let [lower_deg, upper_deg] = self.soft_limits_deg;
        let (lower, upper) = (lower_deg.to_radians(), upper_deg.to_radians());
        let limits_ok =
            if !(lower_deg.is_finite() && upper_deg.is_finite() && lower_deg < upper_deg) {
                problems.push(format!(
                "{name}: soft_limits_deg는 [아래, 위]이고 아래 < 위다: [{lower_deg}, {upper_deg}]"
            ));
                false
            } else if zero_window(lower, upper, clearance, Direction::Same).is_none() {
                problems.push(format!(
                    "{name}: 소프트 한계 {}~{}에 양쪽 경계 여유 {}를 더하면 한 바퀴(360°)를 넘는다",
                    deg(lower),
                    deg(upper),
                    deg(clearance)
                ));
                false
            } else {
                true
            };

        let calibration = match (self.zero_raw, self.direction) {
            (None, None) => None,
            (Some(zero_raw), Some(sign)) => {
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
                direction.map(|direction| {
                    (
                        Calibration {
                            zero_raw,
                            direction,
                        },
                        zero_ok,
                    )
                })
            }
            _ => {
                problems.push(format!(
                    "{name}: zero_raw와 direction은 같이 적는다 (둘 다 관절 시험의 영점 단계에서 잰다)"
                ));
                None
            }
        };
        // 영점과 경계: 한계와 영점 값이 제대로일 때만 본다 (아니면 위에서 이미 알렸다)
        if let Some((c, true)) = calibration
            && limits_ok
            && let Some((low, high)) = zero_window(lower, upper, clearance, c.direction)
            && !(low..=high).contains(&f64::from(c.zero_raw))
        {
            problems.push(format!(
                "{name}: 영점 {}, 방향 {:+}이면 서보 값의 0/4095 경계가 소프트 한계에서 {} 안에 든다. \
                 영점이 {}~{}여야 한다 (가운데 {}). 출력 쪽 혼을 다시 끼워 서보 자리를 맞추고 영점을 다시 잰다 \
                 (0024의 영점)",
                c.zero_raw,
                c.direction.number(),
                deg(clearance),
                low.ceil(),
                high.floor(),
                ((low + high) / 2.0).round()
            ));
        }

        match model {
            Some(&(model_number, model)) if problems.is_empty() => Ok(Joint {
                name,
                id: self.id,
                model,
                model_number,
                lower,
                upper,
                calibration: calibration.map(|(c, _)| c),
            }),
            _ => Err(problems),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 관절 하나 시험의 J3 (data/robot.toml과 같은 값). 끝에 키를 붙이면 j3에 들어간다.
    const J3: &str = r#"
boundary_clearance_deg = 45

[joints.j3]
id = 3
model = "STS3250"
soft_limits_deg = [-30, 118]
"#;

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
        assert!(close(settings.boundary_clearance, 45f64.to_radians()));
        assert_eq!(settings.joints.len(), 1);
        let j3 = settings.joint("j3").unwrap();
        assert_eq!(
            (j3.name, j3.id, j3.model, j3.model_number),
            ("j3", 3, "STS3250", 2825)
        );
        assert!(close(j3.lower, (-30f64).to_radians()));
        assert!(close(j3.upper, 118f64.to_radians()));
        assert_eq!(j3.calibration, None);
        assert!(settings.joint("j2").is_none());
    }

    #[test]
    fn calibration_converts_both_ways() {
        let settings = Settings::parse(&j3_with("zero_raw = 1547\ndirection = 1")).unwrap();
        let c = settings.joint("j3").unwrap().calibration.unwrap();
        assert_eq!(c.direction, Direction::Same);
        assert!(close(c.raw_at(0.0), 1547.0));
        assert!(close(c.raw_at(90f64.to_radians()), 1547.0 + 1024.0));
        assert!(close(c.angle_at(1547.0 - 2048.0), -std::f64::consts::PI));

        let settings = Settings::parse(&j3_with("zero_raw = 2548\ndirection = -1")).unwrap();
        let c = settings.joint("j3").unwrap().calibration.unwrap();
        assert_eq!(c.direction, Direction::Reversed);
        assert!(close(c.raw_at(90f64.to_radians()), 2548.0 - 1024.0));
        for raw in [0.0, 1000.0, 2548.0, 4095.0] {
            assert!(close(c.raw_at(c.angle_at(raw)), raw));
        }
    }

    #[test]
    fn zero_window_keeps_the_boundary_away() {
        // J3 -30°~118°, 여유 45° → -75°~163°가 0~4095 안에 들어야 한다 (한 칸 = 360/4096°)
        let settings = Settings::parse(J3).unwrap();
        let j3 = settings.joint("j3").unwrap();
        let steps = |degrees: f64| degrees * 4096.0 / 360.0;
        let (low, high) = j3
            .zero_window(settings.boundary_clearance, Direction::Same)
            .unwrap();
        assert!(close(low, steps(75.0)) && close(high, 4095.0 - steps(163.0)));
        let (low, high) = j3
            .zero_window(settings.boundary_clearance, Direction::Reversed)
            .unwrap();
        assert!(close(low, steps(163.0)) && close(high, 4095.0 - steps(75.0)));
        // 창의 양 끝 (854~2240)은 되고, 한 칸 밖은 안 된다
        for zero in [854, 2240] {
            let text = j3_with(&format!("zero_raw = {zero}\ndirection = 1"));
            assert!(Settings::parse(&text).is_ok(), "{zero}");
        }
        for zero in [853, 2241] {
            let text = j3_with(&format!("zero_raw = {zero}\ndirection = 1"));
            assert!(Settings::parse(&text).is_err(), "{zero}");
        }
    }

    #[test]
    fn zero_near_the_boundary_is_refused_with_the_window() {
        let message = error(&j3_with("zero_raw = 3000\ndirection = 1"));
        assert!(
            message.contains("영점이 854~2240여야 한다 (가운데 1547)"),
            "{message}"
        );
        let message = error(&j3_with("zero_raw = 1000\ndirection = -1"));
        assert!(
            message.contains("영점이 1855~3241여야 한다 (가운데 2548)"),
            "{message}"
        );
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
        let message = error(&J3.replace("STS3250", "XL430"));
        assert!(message.contains("모르는 서보 모델이다: XL430"), "{message}");
        let message = error(&J3.replace("joints.j3", "joints.j7"));
        assert!(message.contains("j7: 모르는 관절이다"), "{message}");
    }

    #[test]
    fn limits_must_be_ordered_and_leave_room_for_the_boundary() {
        for limits in ["[118, -30]", "[0, 0]", "[nan, 118]"] {
            let message = error(&J3.replace("[-30, 118]", limits));
            assert!(message.contains("아래 < 위"), "{limits}: {message}");
        }
        // 폭 271°에 여유 45° 둘을 더하면 361°
        let message = error(&J3.replace("[-30, 118]", "[-135, 136]"));
        assert!(message.contains("한 바퀴(360°)를 넘는다"), "{message}");
        assert!(Settings::parse(&J3.replace("[-30, 118]", "[-135, 134]")).is_ok());
    }

    #[test]
    fn zero_and_direction_come_together_and_in_range() {
        let message = error(&j3_with("zero_raw = 1500"));
        assert!(message.contains("같이 적는다"), "{message}");
        let message = error(&j3_with("direction = 1"));
        assert!(message.contains("같이 적는다"), "{message}");
        let message = error(&j3_with("zero_raw = 1500\ndirection = 2"));
        assert!(message.contains("direction은 1이나 -1이다: 2"), "{message}");
        let message = error(&j3_with("zero_raw = 4096\ndirection = 1"));
        assert!(message.contains("zero_raw는 0~4095다: 4096"), "{message}");
    }

    #[test]
    fn clearance_must_not_be_negative() {
        let message = error(&J3.replace("= 45", "= -1"));
        assert!(message.contains("0 이상"), "{message}");
    }

    #[test]
    fn every_problem_is_reported_at_once() {
        let text = J3.replace("id = 3", "id = 4").replace("STS3250", "XL430");
        let message = error(&(text + "zero_raw = 3000\ndirection = 1\n"));
        assert!(message.starts_with("고칠 것 3가지"), "{message}");
        assert!(message.contains("영점이 854~2240여야 한다"), "{message}");
        let message = error("boundary_clearance_deg = 45\n[joints]\n");
        assert!(message.contains("관절이 하나도 없다"), "{message}");
    }

    #[test]
    fn summary_shows_each_joint() {
        let summary = Settings::parse(J3).unwrap().summary();
        assert!(
            summary.contains(
                "j3  ID 3  STS3250  소프트 한계 -30°~118°  영점과 방향을 아직 재지 않았다"
            )
        );
        let settings = Settings::parse(&j3_with("zero_raw = 1547\ndirection = 1")).unwrap();
        // -30° → 1547 - 341.3, 118° → 1547 + 1342.6
        assert!(
            settings
                .summary()
                .contains("영점 1547, 방향 +1, 소프트 한계의 서보 값 1206~2890")
        );
    }
}
