//! 실행 명령의 시험 순서 (docs/decisions/0034): 첫 동력 시험처럼 사람이 지켜보는 짧은 시험을 실제 실행 루프로
//! 돌린다. 순서는 TOML 파일 하나다:
//!
//! ```toml
//! name = "어깨 버티기"
//! joints = ["j1", "j2"]     # 켤 관절 (로봇 설정에 영점이 있어야 한다)
//! speed_percent = 50        # 실행 루프의 속도·가속·저크 한계에 곱한다 (기본 100)
//!
//! [[steps]]
//! pause = "위팔 끝에 추를 걸었으면"   # Enter를 기다리며 버틴다
//! [[steps]]
//! move = { j2 = 90.0 }      # 관절 각 [°]. 적지 않은 관절은 그 자리
//! [[steps]]
//! hold_s = 600              # 버틴다. 온도가 알림 기준에 닿으면 일찍 끝낸다
//! [[steps]]
//! move = { j2 = 0.0 }
//! [[steps]]
//! pause = "팔을 받쳤으면 (다음에 토크를 끈다)"
//! [[steps]]
//! release = true            # 모든 관절의 토크를 끈다 (마지막 단계만)
//! ```
//!
//! 움직이거나 버티는 동안 Enter를 누르면 감속해 서고 순서를 끝낸다. 고장이 나면 순서를 끝낸다 (서보는 마지막
//! 목표를 잡고 있다).

use std::collections::BTreeMap;

use horilab_servo::feetech::ServoBus;
use serde::Deserialize;

use crate::check::Console;
use crate::plan::{MotionLimits, move_joints};
use crate::runloop::{Mode, RunLoop};
use crate::settings::{LoopSettings, Settings};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScriptFile {
    name: String,
    joints: Vec<String>,
    speed_percent: Option<f64>,
    #[serde(default)]
    steps: Vec<StepFile>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StepFile {
    pause: Option<String>,
    #[serde(rename = "move")]
    move_to: Option<BTreeMap<String, f64>>,
    hold_s: Option<f64>,
    release: Option<bool>,
}

/// 단계 하나.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// 안내를 보이고 Enter를 기다린다 (그동안 버틴다)
    Pause(String),
    /// 관절 이동: (순서의 관절 번호, 목표 [rad])
    Move(Vec<(usize, f64)>),
    /// 버틴다 [s]
    Hold(f64),
    /// 모든 관절의 토크를 끈다
    Release,
}

/// 읽고 검사한 시험 순서.
#[derive(Debug, Clone, PartialEq)]
pub struct Script {
    pub name: String,
    /// 켤 관절 (관절 번호 순, 실행 루프의 관절 순서와 같다)
    pub joints: Vec<&'static str>,
    /// 실행 루프의 이동 한계에 곱하는 몫 (0~1)
    pub speed: f64,
    pub steps: Vec<Step>,
}

impl Script {
    /// 시험 순서를 읽고 로봇 설정과 맞춰 본다. 관절은 영점이 있어야 하고, 목표는 소프트 한계 안이어야 하고,
    /// 토크 끄기는 마지막 단계여야 한다. 잘못된 곳은 모두 모아 알린다.
    pub fn parse(text: &str, settings: &Settings) -> Result<Self, String> {
        let file: ScriptFile =
            toml::from_str(text).map_err(|e| format!("시험 순서를 읽지 못했다: {e}"))?;
        let mut problems = Vec::new();
        let mut joints: Vec<&crate::settings::Joint> = Vec::new();
        for name in &file.joints {
            match settings.joint(name) {
                None => problems.push(format!("로봇 설정에 없는 관절이다: {name}")),
                Some(joint) if joint.calibration.is_none() => problems.push(format!(
                    "{name}: 영점을 재지 않아 움직이지 않는다 (zero로 잰다)"
                )),
                Some(joint) if joints.iter().any(|known| known.name == joint.name) => {
                    problems.push(format!("관절을 두 번 적었다: {name}"))
                }
                Some(joint) => joints.push(joint),
            }
        }
        if file.joints.is_empty() {
            problems.push("joints에 켤 관절이 없다".to_owned());
        }
        joints.sort_by_key(|joint| joint.id);
        let speed = file.speed_percent.unwrap_or(100.0);
        if !(speed.is_finite() && speed > 0.0 && speed <= 100.0) {
            problems.push(format!("speed_percent는 0보다 크고 100 이하다: {speed}"));
        }
        let mut steps = Vec::new();
        let last = file.steps.len().saturating_sub(1);
        for (n, step) in file.steps.iter().enumerate() {
            let at = format!("{}번째 단계", n + 1);
            let kinds = usize::from(step.pause.is_some())
                + usize::from(step.move_to.is_some())
                + usize::from(step.hold_s.is_some())
                + usize::from(step.release.is_some());
            if kinds != 1 {
                problems.push(format!(
                    "{at}: pause, move, hold_s, release 가운데 하나만 적는다"
                ));
                continue;
            }
            if let Some(text) = &step.pause {
                steps.push(Step::Pause(text.clone()));
            } else if let Some(targets) = &step.move_to {
                let mut found = Vec::new();
                for (name, &degrees) in targets {
                    match joints.iter().position(|joint| joint.name == name) {
                        None => {
                            problems.push(format!("{at}: {name}은 joints에 없어 움직일 수 없다"))
                        }
                        Some(index) => {
                            let joint = joints[index];
                            let angle = degrees.to_radians();
                            if !(degrees.is_finite()
                                && (joint.lower..=joint.upper).contains(&angle))
                            {
                                problems.push(format!(
                                    "{at}: {name} {degrees}°는 소프트 한계({:.1}°~{:.1}°) 밖이다",
                                    joint.lower.to_degrees(),
                                    joint.upper.to_degrees()
                                ));
                            } else {
                                found.push((index, angle));
                            }
                        }
                    }
                }
                if targets.is_empty() {
                    problems.push(format!("{at}: move에 관절이 없다"));
                }
                steps.push(Step::Move(found));
            } else if let Some(seconds) = step.hold_s {
                if !(seconds.is_finite() && seconds > 0.0) {
                    problems.push(format!("{at}: hold_s는 0보다 크다: {seconds}"));
                }
                steps.push(Step::Hold(seconds));
            } else if step.release == Some(true) {
                if n != last {
                    problems.push(format!(
                        "{at}: release는 마지막 단계여야 한다 (토크를 끈 뒤에는 움직이지 않는다)"
                    ));
                }
                steps.push(Step::Release);
            } else {
                problems.push(format!("{at}: release는 true만 적는다"));
            }
        }
        if !problems.is_empty() {
            return Err(problems.join("\n"));
        }
        Ok(Self {
            name: file.name,
            joints: joints.iter().map(|joint| joint.name).collect(),
            speed: speed / 100.0,
            steps,
        })
    }

    /// 사람이 볼 순서: 단계마다 한 줄.
    pub fn describe(&self) -> String {
        let mut lines = vec![format!(
            "시험 순서 \"{}\": 관절 {}, 속도 {:.0}%",
            self.name,
            self.joints.join(", "),
            self.speed * 100.0
        )];
        for (n, step) in self.steps.iter().enumerate() {
            let text = match step {
                Step::Pause(text) => format!("멈춤: {text} (Enter)"),
                Step::Move(targets) => {
                    let parts: Vec<String> = targets
                        .iter()
                        .map(|&(index, angle)| {
                            format!("{} {:.1}°", self.joints[index], angle.to_degrees())
                        })
                        .collect();
                    format!("움직이기: {}", parts.join(", "))
                }
                Step::Hold(seconds) => {
                    format!("버티기: {seconds} s (온도가 알림 기준에 닿으면 일찍 끝낸다)")
                }
                Step::Release => "토크 끄기: 모든 관절".to_owned(),
            };
            lines.push(format!("  {}. {text}", n + 1));
        }
        lines.join("\n") + "\n"
    }
}

/// 순서의 진행.
#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    Running,
    /// 모든 단계를 마쳤다
    Done,
    /// 사람이 멈췄거나 고장이 나 순서를 끝냈다
    Stopped(String),
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Begin,
    Waiting,
    Moving,
    Holding { until: f64 },
}

/// 시험 순서를 실행 루프에 건다. 박자마다 [`RunLoop::tick`] 뒤에 [`Runner::tick`]을 부른다.
pub struct Runner {
    script: Script,
    motion: MotionLimits,
    period: f64,
    temperature_warn: f64,
    index: usize,
    phase: Phase,
    events: Vec<String>,
    released: bool,
}

impl Runner {
    pub fn new(script: Script, settings: &LoopSettings, period: f64) -> Self {
        let n = script.joints.len();
        let s = script.speed;
        Self {
            motion: MotionLimits::same(
                n,
                settings.max_speed * s,
                settings.max_acceleration * s,
                settings.max_jerk * s,
            ),
            script,
            period,
            temperature_warn: settings.temperature_warn,
            index: 0,
            phase: Phase::Begin,
            events: Vec::new(),
            released: false,
        }
    }

    /// 순서가 모든 관절의 토크를 껐다 (끈 뒤에는 박자를 돌리지 않아 읽은 상태로는 알 수 없다).
    pub fn released(&self) -> bool {
        self.released
    }

    /// 지금 단계 (기록의 step 칸): `3-hold`처럼 번호와 종류.
    pub fn label(&self) -> String {
        match self.script.steps.get(self.index) {
            None => "end".to_owned(),
            Some(step) => {
                let kind = match step {
                    Step::Pause(_) => "pause",
                    Step::Move(_) => "move",
                    Step::Hold(_) => "hold",
                    Step::Release => "release",
                };
                format!("{}-{kind}", self.index + 1)
            }
        }
    }

    /// 지난 부름 뒤에 생긴 알림 (기록의 events.csv에 남긴다).
    pub fn take_events(&mut self) -> Vec<String> {
        std::mem::take(&mut self.events)
    }

    fn note(&mut self, console: &mut dyn Console, text: String) {
        console.say(&text);
        self.events.push(text);
    }

    fn next(&mut self) {
        self.index += 1;
        self.phase = Phase::Begin;
    }

    /// 박자 하나만큼 순서를 진행한다. `now`는 시작부터의 시간 [s]이다.
    pub fn tick<B: ServoBus>(
        &mut self,
        run: &mut RunLoop<B>,
        console: &mut dyn Console,
        now: f64,
    ) -> Status {
        if let Mode::Fault(reason) = run.mode() {
            let text = format!("고장으로 순서를 끝낸다: {reason}");
            self.note(console, text.clone());
            return Status::Stopped(text);
        }
        loop {
            let Some(step) = self.script.steps.get(self.index).cloned() else {
                return Status::Done;
            };
            match (self.phase, step) {
                (Phase::Begin, Step::Pause(text)) => {
                    console.begin_wait(&format!("[{}] {text} (Enter)", self.label()));
                    self.phase = Phase::Waiting;
                    return Status::Running;
                }
                (Phase::Waiting, Step::Pause(_)) => {
                    if !console.enter_pressed() {
                        return Status::Running;
                    }
                    self.next();
                }
                (Phase::Begin, Step::Move(targets)) => {
                    let mut goal = run.targets().to_vec();
                    for &(index, angle) in &targets {
                        goal[index] = angle;
                    }
                    let submitted = move_joints(run.targets(), &goal, &self.motion, self.period)
                        .and_then(|trajectory| {
                            let seconds = trajectory.points.last().map_or(0.0, |point| point.0);
                            run.submit(trajectory).map(|()| seconds)
                        });
                    match submitted {
                        Ok(seconds) => {
                            let text = format!("[{}] 움직인다 ({seconds:.1} s)", self.label());
                            self.note(console, text);
                            self.phase = Phase::Moving;
                            return Status::Running;
                        }
                        Err(e) => {
                            let text = format!("[{}] 움직이지 못했다: {e}", self.label());
                            self.note(console, text.clone());
                            return Status::Stopped(text);
                        }
                    }
                }
                (Phase::Moving, Step::Move(_)) => {
                    if console.enter_pressed() {
                        run.stop();
                        let text = format!("[{}] 사람이 멈췄다", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    if *run.mode() != Mode::Holding {
                        return Status::Running;
                    }
                    self.next();
                }
                (Phase::Begin, Step::Hold(seconds)) => {
                    let text = format!("[{}] {seconds} s 버틴다", self.label());
                    self.note(console, text);
                    self.phase = Phase::Holding {
                        until: now + seconds,
                    };
                    return Status::Running;
                }
                (Phase::Holding { until }, Step::Hold(_)) => {
                    if console.enter_pressed() {
                        let text = format!("[{}] 사람이 멈췄다", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    let hottest = run
                        .states()
                        .iter()
                        .map(|(_, state)| state.temperature)
                        .max()
                        .unwrap_or(0);
                    if f64::from(hottest) >= self.temperature_warn {
                        let text = format!(
                            "[{}] 온도 {hottest}°C가 알림 기준 {}°C에 닿아 버티기를 끝낸다",
                            self.label(),
                            self.temperature_warn
                        );
                        self.note(console, text);
                        self.next();
                        continue;
                    }
                    if now < until {
                        return Status::Running;
                    }
                    self.next();
                }
                (Phase::Begin, Step::Release) => {
                    return match run.release_all() {
                        Ok(()) => {
                            let text = format!("[{}] 모든 관절의 토크를 껐다", self.label());
                            self.note(console, text);
                            self.index += 1;
                            self.released = true;
                            Status::Done
                        }
                        Err(e) => {
                            let text = format!("[{}] 토크를 끄지 못했다: {e}", self.label());
                            self.note(console, text.clone());
                            Status::Stopped(text)
                        }
                    };
                }
                // 단계마다 시작할 때만 Begin이고, 진행 상태는 그 단계의 종류와 짝이 맞는다
                (_, _) => {
                    self.phase = Phase::Begin;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use horilab_servo::feetech::Bus;
    use horilab_servo::feetech::fake::{FakePort, FakeServo, open_fake_bus};
    use horilab_servo::feetech::registers as reg;

    use super::*;
    use crate::joints::{Caps, Joints, Servo, programmed};
    use crate::settings::TEST_J3;

    /// 시험용 로봇 설정: TEST_J3에 영점을 준다.
    fn settings() -> Settings {
        let text = TEST_J3.replace(
            "[joints.j3]",
            "[joints.j3]\nzero_raw = 2048\ndirection = 1\nposition_offset = 0",
        );
        Settings::parse(&text).unwrap()
    }

    /// 누를 줄을 미리 정해 두는 콘솔: 박자마다 Enter를 눌렀는지를 순서대로 준다.
    #[derive(Default)]
    struct Keys {
        presses: Vec<bool>,
        said: Vec<String>,
    }

    impl Console for Keys {
        fn say(&mut self, text: &str) {
            self.said.push(text.to_owned());
        }
        fn ask(&mut self, _question: &str) -> String {
            String::new()
        }
        fn begin_wait(&mut self, prompt: &str) {
            self.said.push(prompt.to_owned());
        }
        fn enter_pressed(&mut self) -> bool {
            if self.presses.is_empty() {
                false
            } else {
                self.presses.remove(0)
            }
        }
        fn status(&mut self, _text: &str) {}
    }

    fn run_loop(settings: &Settings) -> RunLoop<Bus<FakePort>> {
        let joint = settings.joint("j3").unwrap();
        let servo = Servo::from_joint(joint).unwrap();
        let fake = programmed(FakeServo::with(3, joint.model_number, 2048), &servo);
        let joints = Joints::open(
            open_fake_bus([fake]),
            vec![servo],
            Caps::for_loop(&settings.run_loop),
        )
        .unwrap();
        RunLoop::start(joints, settings.run_loop, None).unwrap()
    }

    const DT: f64 = 0.01;

    /// 박자를 돌리며 순서를 끝까지 진행한다. `hook`는 박자마다 가짜 서보를 바꿀 자리다.
    fn drive(
        run: &mut RunLoop<Bus<FakePort>>,
        runner: &mut Runner,
        keys: &mut Keys,
        limit: usize,
        mut hook: impl FnMut(&mut RunLoop<Bus<FakePort>>, f64),
    ) -> Status {
        for n in 0..limit {
            let now = n as f64 * DT;
            hook(run, now);
            run.tick(DT);
            let status = runner.tick(run, keys, now);
            if status != Status::Running {
                return status;
            }
        }
        Status::Running
    }

    #[test]
    fn a_script_is_checked_against_the_robot_settings() {
        let settings = settings();
        let good = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nmove = { j3 = 30.0 }\n[[steps]]\nrelease = true\n";
        let script = Script::parse(good, &settings).unwrap();
        assert_eq!(script.joints, ["j3"]);
        assert_eq!(script.steps.len(), 2);
        assert!(script.describe().contains("j3 30.0°"));
        let bad = "name = \"t\"\njoints = [\"j3\", \"j9\"]\nspeed_percent = 150\n\
                   [[steps]]\nmove = { j3 = 170.0 }\n[[steps]]\nrelease = true\n[[steps]]\nhold_s = 1\n\
                   [[steps]]\npause = \"a\"\nhold_s = 2\n";
        let error = Script::parse(bad, &settings).unwrap_err();
        for expected in [
            "j9",
            "speed_percent",
            "소프트 한계",
            "release는 마지막",
            "하나만",
        ] {
            assert!(error.contains(expected), "{expected}: {error}");
        }
        let uncalibrated = Settings::parse(TEST_J3).unwrap();
        let error = Script::parse(good, &uncalibrated).unwrap_err();
        assert!(error.contains("영점"), "{error}");
        assert!(Script::parse("name = \"t\"\njoints = [\"j3\"]\nwhat = 1\n", &settings).is_err());
    }

    #[test]
    fn a_script_moves_holds_and_releases() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\npause = \"추를 걸었으면\"\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nhold_s = 1.0\n\
                    [[steps]]\nmove = { j3 = 0.0 }\n\
                    [[steps]]\nrelease = true\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        // 멈춤에서 다섯 박자 뒤에 Enter를 누른다
        let mut keys = Keys {
            presses: vec![false, false, false, false, true],
            ..Default::default()
        };
        let mut peak = 0.0_f64;
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |run, _| {
            peak = peak.max(run.targets()[0]);
        });
        assert_eq!(status, Status::Done);
        assert!((peak - 20f64.to_radians()).abs() < 1e-9, "{peak}");
        assert!(run.targets()[0].abs() < 1e-9);
        // 토크를 껐다
        let torque = run.joints_for_test().wire().servos[0].get(reg::TORQUE_ENABLE);
        assert_eq!(torque, reg::TORQUE_OFF);
        assert!(runner.released());
        assert!(
            keys.said.iter().any(|line| line.contains("토크를 껐다")),
            "{:?}",
            keys.said
        );
    }

    #[test]
    fn enter_stops_a_move_and_ends_the_script() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nmove = { j3 = 60.0 }\n[[steps]]\nhold_s = 5\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut presses = vec![false; 50];
        presses.push(true);
        let mut keys = Keys {
            presses,
            ..Default::default()
        };
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |_, _| {});
        assert!(
            matches!(&status, Status::Stopped(text) if text.contains("사람이 멈췄다")),
            "{status:?}"
        );
        // 감속해 선다: 정지 중이고, 목표 60°에 닿기 전이다
        assert_eq!(run.mode(), &Mode::Stopping);
        for _ in 0..500 {
            run.tick(DT);
        }
        assert_eq!(run.mode(), &Mode::Holding);
        assert!(
            run.targets()[0] < 60f64.to_radians() - 0.1,
            "{}",
            run.targets()[0]
        );
    }

    #[test]
    fn holding_ends_early_when_the_servo_gets_warm() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nhold_s = 600\n[[steps]]\nmove = { j3 = 10.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let warm = settings.run_loop.temperature_warn as i32;
        let status = drive(&mut run, &mut runner, &mut keys, 20_000, |run, now| {
            if now > 2.0 {
                run.joints_for_test().wire().servos[0].set(reg::PRESENT_TEMPERATURE, warm);
            }
        });
        assert_eq!(status, Status::Done);
        assert!(
            keys.said
                .iter()
                .any(|line| line.contains("알림 기준") && line.contains("버티기를 끝낸다"))
        );
        // 버티기를 끝내고 다음 단계(움직이기)를 했다
        assert!((run.targets()[0] - 10f64.to_radians()).abs() < 1e-9);
    }

    #[test]
    fn a_fault_ends_the_script() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nhold_s = 600\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let status = drive(&mut run, &mut runner, &mut keys, 1000, |run, now| {
            if now > 0.5 {
                run.joints_for_test().wire().servos[0].overload();
            }
        });
        assert!(
            matches!(&status, Status::Stopped(text) if text.contains("고장")),
            "{status:?}"
        );
    }
}
