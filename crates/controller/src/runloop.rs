//! 실행 루프 (docs/decisions/0032-run-loop.md의 만드는 순서 1): 박자마다 서보 상태를 모두 읽고, 검사하고,
//! 궤적의 그 시각 목표를 쓴다. 서보를 움직이는 쓰기는 관절 층([`Joints`])을 지난다 (0009의 한곳 안전 검사).
//!
//! - 궤적은 점마다 시각 [s]과 관절 각 [rad]이다. 지금 붙잡고 있는 목표에서 시작하고 소프트 한계와 속도
//!   한계 안이어야 받고, 아니면 실행 전에 통째로 거부한다 (0009). 점 사이는 곧게 잇는다.
//! - 정지는 같은 길을 따라가며 재생 빠르기를 정지 시간 동안 0으로 줄인 뒤 그 자리에서 버틴다.
//! - 서보 보호 비트, 꺼진 토크(서보 전원이 꺼졌다 켜짐), 잇단 통신 실패는 고장이다: 새 목표를 쓰지 않고
//!   (서보는 마지막 목표를 잡고 있다) 까닭을 남긴다. 사람이 [`RunLoop::resume`]으로 푼다.
//! - 시간은 부르는 쪽이 준다. 실물에서는 주기마다, 시험에서는 가짜 시계로 [`RunLoop::tick`]을 부른다.

use horilab_servo::feetech::Port;

use crate::joints::{Joints, MAX_FAILURES, Snapshot};

/// 루프의 한계값. 만드는 순서 2에서 로봇 설정으로 옮긴다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopLimits {
    /// 관절 속도 한계 [rad/s]
    pub max_speed: f64,
    /// 궤적의 첫 점이 지금 목표에서 이만큼까지 떨어져도 받는다 [rad]
    pub start_tolerance: f64,
    /// 정지 명령에서 재생 빠르기를 0으로 줄이는 시간 [s]
    pub stop_time: f64,
}

/// 시간이 붙은 관절 궤적: 점마다 시작에서의 시각 [s]과 관절 각 [rad] (루프의 관절 순서).
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    pub points: Vec<(f64, Vec<f64>)>,
}

/// 루프의 상태.
#[derive(Debug, Clone, PartialEq)]
pub enum Mode {
    /// 목표를 붙잡고 있다
    Holding,
    /// 궤적을 따라간다
    Moving,
    /// 정지 명령으로 같은 길에서 감속하는 중
    Stopping,
    /// 고장: 새 목표를 쓰지 않는다. 사람이 풀기 전에는 궤적을 받지 않는다
    Fault(String),
}

struct Playing {
    trajectory: Trajectory,
    /// 궤적 안의 시각 [s]
    time: f64,
    /// 재생 빠르기 (1이면 궤적의 시간대로)
    rate: f64,
}

/// 실행 루프.
pub struct RunLoop<P: Port> {
    joints: Joints<P>,
    ids: Vec<u8>,
    limits: LoopLimits,
    /// 지금 쓰는 목표 [rad] (관절 순서)
    targets: Vec<f64>,
    /// 마지막 박자에 읽은 상태
    states: Vec<(u8, Snapshot)>,
    playing: Option<Playing>,
    mode: Mode,
    /// 상태를 읽지 못한 박자가 잇달아 몇 번인지
    missed: u64,
}

impl<P: Port> RunLoop<P> {
    /// 관절 층의 서보를 모두 켜거나 이어받고(0009의 시작 절차), 지금 목표를 붙잡는다. 영점을 잰 로봇 관절만
    /// 받는다.
    pub fn start(mut joints: Joints<P>, limits: LoopLimits) -> Result<Self, String> {
        let ids = joints.ids();
        for &id in &ids {
            if joints.servo(id)?.is_bench() {
                return Err(format!(
                    "서보 {id}: 실행 루프는 영점을 잰 로봇 관절만 다룬다"
                ));
            }
            joints.enable(id)?;
        }
        let mut run = Self {
            joints,
            ids,
            limits,
            targets: Vec::new(),
            states: Vec::new(),
            playing: None,
            mode: Mode::Holding,
            missed: 0,
        };
        run.hold_present_goals()?;
        Ok(run)
    }

    pub fn mode(&self) -> &Mode {
        &self.mode
    }

    /// 지금 쓰는 목표 [rad] (관절 순서).
    pub fn targets(&self) -> &[f64] {
        &self.targets
    }

    /// 마지막 박자에 읽은 상태.
    pub fn states(&self) -> &[(u8, Snapshot)] {
        &self.states
    }

    /// 한 박자: 상태를 읽고, 검사하고, 그 시각의 목표를 쓴다. `dt`는 지난 박자에서 흐른 시간 [s]이다.
    pub fn tick(&mut self, dt: f64) -> &Mode {
        match self.joints.read_all() {
            Ok(states) => {
                self.missed = 0;
                self.states = states;
            }
            Err(e) => {
                // 이번 박자는 쓰지 않는다: 서보는 마지막 목표를 잡고 있다
                self.missed += 1;
                if self.missed >= MAX_FAILURES {
                    self.fault(format!("{e} (박자 {}번 잇달아)", self.missed));
                }
                return &self.mode;
            }
        }
        if matches!(self.mode, Mode::Fault(_)) {
            return &self.mode;
        }
        if let Some(reason) = self.check() {
            self.fault(reason);
            return &self.mode;
        }
        self.advance(dt);
        let goals: Result<Vec<(u8, i32)>, String> = self
            .ids
            .iter()
            .zip(&self.targets)
            .map(|(&id, &angle)| {
                let servo = self.joints.servo(id)?;
                servo
                    .raw_at(angle)
                    .map(|raw| (id, raw))
                    .ok_or_else(|| format!("{}: 영점이 없다", servo.name))
            })
            .collect();
        if let Err(e) = goals.and_then(|goals| self.joints.write_goals(&goals)) {
            self.fault(e);
        }
        &self.mode
    }

    /// 궤적을 받는다. 붙잡고 있을 때만 받고, 첫 점이 지금 목표이고 모든 점이 한계 안이어야 한다. 아니면
    /// 실행 전에 통째로 거부한다 (0009).
    pub fn submit(&mut self, trajectory: Trajectory) -> Result<(), String> {
        match &self.mode {
            Mode::Holding => {}
            Mode::Fault(reason) => return Err(format!("고장이라 궤적을 받지 않는다: {reason}")),
            Mode::Moving | Mode::Stopping => {
                return Err("움직이는 중에는 새 궤적을 받지 않는다 (정지한 뒤 보낸다)".to_owned());
            }
        }
        self.check_trajectory(&trajectory)?;
        self.playing = Some(Playing {
            trajectory,
            time: 0.0,
            rate: 1.0,
        });
        self.mode = Mode::Moving;
        Ok(())
    }

    /// 정지: 움직이는 중이면 같은 길에서 감속해 그 자리에서 버틴다. 붙잡고 있거나 고장이면 그대로다.
    pub fn stop(&mut self) {
        if self.mode == Mode::Moving {
            self.mode = Mode::Stopping;
        }
    }

    /// 사람이 고장을 푼다: 관절 층의 멈춤을 풀고, 서보마다 토크를 켜거나 이어받은 뒤 지금 목표를 붙잡는다.
    /// 서보 보호가 남아 있으면 풀리지 않는다 (과부하로 묶인 서보는 원인을 없앤 뒤 토크를 껐다 켠다, 0024).
    pub fn resume(&mut self) -> Result<(), String> {
        self.joints.resume();
        for id in self.ids.clone() {
            self.joints.enable(id)?;
        }
        self.hold_present_goals()
    }

    /// 서보마다 남은 목표를 읽어 붙잡는다.
    fn hold_present_goals(&mut self) -> Result<(), String> {
        let states = self.joints.read_all()?;
        let mut targets = Vec::new();
        for (id, state) in &states {
            let servo = self.joints.servo(*id)?;
            let angle = servo
                .angle_at(state.goal_position)
                .ok_or_else(|| format!("{}: 영점이 없다", servo.name))?;
            targets.push(angle);
        }
        self.targets = targets;
        self.states = states;
        self.playing = None;
        self.mode = Mode::Holding;
        self.missed = 0;
        Ok(())
    }

    /// 읽은 상태에서 고장을 찾는다: 서보 보호 비트, 꺼진 토크.
    fn check(&self) -> Option<String> {
        self.states.iter().find_map(|(id, state)| {
            let name = self
                .joints
                .servo(*id)
                .map_or_else(|_| format!("서보 {id}"), |servo| servo.name.clone());
            if !state.status.is_none() {
                Some(format!("{name}: 서보 보호가 걸렸다 ({})", state.status))
            } else if !state.torque_on() {
                Some(format!(
                    "{name}: 토크가 꺼졌다 (서보 전원이 꺼졌다 켜졌을 수 있다)"
                ))
            } else {
                None
            }
        })
    }

    /// 궤적을 `dt`만큼 재생한다. 정지 중이면 재생 빠르기를 줄이고, 끝나거나 멈추면 붙잡는다.
    fn advance(&mut self, dt: f64) {
        let stopping = self.mode == Mode::Stopping;
        let Some(playing) = &mut self.playing else {
            return;
        };
        if stopping {
            playing.rate = (playing.rate - dt / self.limits.stop_time).max(0.0);
        }
        playing.time += playing.rate * dt;
        let (targets, end) = sample(&playing.trajectory, playing.time);
        let done = end || playing.rate == 0.0;
        self.targets = targets;
        if done {
            self.playing = None;
            self.mode = Mode::Holding;
        }
    }

    fn fault(&mut self, reason: String) {
        self.playing = None;
        if !matches!(self.mode, Mode::Fault(_)) {
            self.mode = Mode::Fault(reason);
        }
    }

    fn check_trajectory(&self, trajectory: &Trajectory) -> Result<(), String> {
        let points = &trajectory.points;
        let count = self.ids.len();
        let Some((first_time, first)) = points.first() else {
            return Err("궤적에 점이 없다".to_owned());
        };
        if *first_time != 0.0 {
            return Err(format!("첫 점의 시각은 0이다: {first_time}"));
        }
        for (time, angles) in points {
            if angles.len() != count {
                return Err(format!(
                    "점마다 관절 {count}개의 각이 있어야 한다: {}개",
                    angles.len()
                ));
            }
            if !time.is_finite() || angles.iter().any(|angle| !angle.is_finite()) {
                return Err("궤적에 수가 아닌 값이 있다".to_owned());
            }
        }
        let name = |index: usize| {
            self.joints
                .servo(self.ids[index])
                .map_or_else(|_| format!("서보 {}", self.ids[index]), |s| s.name.clone())
        };
        for pair in points.windows(2) {
            let ((t0, q0), (t1, q1)) = (&pair[0], &pair[1]);
            if t1 <= t0 {
                return Err(format!("시각은 늘어야 한다: {t0} → {t1} s"));
            }
            for (index, (a, b)) in q0.iter().zip(q1).enumerate() {
                let speed = (b - a).abs() / (t1 - t0);
                if speed > self.limits.max_speed {
                    return Err(format!(
                        "{}: {t0}~{t1} s의 속도 {speed:.3} rad/s가 한계 {} rad/s를 넘는다",
                        name(index),
                        self.limits.max_speed
                    ));
                }
            }
        }
        for (index, (&start, &now)) in first.iter().zip(&self.targets).enumerate() {
            if (start - now).abs() > self.limits.start_tolerance {
                return Err(format!(
                    "{}: 첫 점 {start:.4} rad가 지금 목표 {now:.4} rad에서 시작하지 않는다",
                    name(index)
                ));
            }
        }
        for (time, angles) in points {
            for (index, &angle) in angles.iter().enumerate() {
                let servo = self.joints.servo(self.ids[index])?;
                let inside = servo
                    .raw_at(angle)
                    .is_some_and(|raw| (servo.min_raw..=servo.max_raw).contains(&raw));
                if !inside {
                    return Err(format!(
                        "{}: {time} s의 {angle:.4} rad가 소프트 한계 밖이다",
                        servo.name
                    ));
                }
            }
        }
        Ok(())
    }
}

/// 궤적의 `time` 시각 관절 각 (점 사이는 곧게). 끝을 지났으면 마지막 점과 `true`.
fn sample(trajectory: &Trajectory, time: f64) -> (Vec<f64>, bool) {
    let points = &trajectory.points;
    let Some((last_time, last)) = points.last() else {
        return (Vec::new(), true);
    };
    if time >= *last_time {
        return (last.clone(), true);
    }
    let next = points
        .iter()
        .position(|(t, _)| *t > time)
        .unwrap_or(points.len() - 1)
        .max(1);
    let ((t0, q0), (t1, q1)) = (&points[next - 1], &points[next]);
    let s = (time - t0) / (t1 - t0);
    let angles = q0.iter().zip(q1).map(|(a, b)| a + (b - a) * s).collect();
    (angles, false)
}

#[cfg(test)]
mod tests {
    use horilab_servo::feetech::fake::{FakePort, FakeServo, open_fake_bus};
    use horilab_servo::feetech::registers as reg;

    use super::*;
    use crate::joints::{BENCH_CAPS, Servo};
    use crate::settings::{Calibration, Direction, RAD_PER_STEP, Settings, TEST_J3};

    const LIMITS: LoopLimits = LoopLimits {
        max_speed: 2.0,
        start_tolerance: 0.01,
        stop_time: 0.2,
    };
    const DT: f64 = 0.01;

    /// J3과 J4 (둘 다 옛 시험의 J3 한계 -30°~118°, 영점 2048에서 0°)
    fn two_joints() -> RunLoop<FakePort> {
        let j3 = Settings::parse(TEST_J3)
            .unwrap()
            .joint("j3")
            .unwrap()
            .clone();
        let servos = [("j3", 3), ("j4", 4)].map(|(name, id)| {
            let mut joint = j3.clone();
            joint.name = name;
            joint.id = id;
            joint.calibration = Some(Calibration {
                pin: 0.0,
                zero_raw: 2048,
                direction: Direction::Same,
                position_offset: 0,
            });
            Servo::from_joint(&joint).unwrap()
        });
        let bus = open_fake_bus([
            FakeServo::with(3, 2825, 2048),
            FakeServo::with(4, 2825, 2048),
        ]);
        RunLoop::start(
            Joints::open(bus, servos.to_vec(), BENCH_CAPS).unwrap(),
            LIMITS,
        )
        .unwrap()
    }

    fn line(to: [f64; 2], seconds: f64) -> Trajectory {
        Trajectory {
            points: vec![(0.0, vec![0.0, 0.0]), (seconds, to.to_vec())],
        }
    }

    fn position(run: &mut RunLoop<FakePort>, index: usize) -> i32 {
        run.joints.wire().servos[index].get(reg::PRESENT_POSITION)
    }

    /// 붙잡을 때까지 박자를 돌린다 (넉넉히 1000박자까지).
    fn until_holding(run: &mut RunLoop<FakePort>) {
        for _ in 0..1000 {
            if run.tick(DT) == &Mode::Holding {
                return;
            }
        }
        panic!("붙잡지 않았다: {:?}", run.mode());
    }

    fn steps(angle: f64) -> i32 {
        2048 + (angle / RAD_PER_STEP).round() as i32
    }

    #[test]
    fn starts_by_holding_the_present_goals() {
        let run = two_joints();
        assert_eq!(run.mode(), &Mode::Holding);
        assert_eq!(run.targets(), [0.0, 0.0]);
        assert!(run.states().iter().all(|(_, state)| state.torque_on()));
    }

    #[test]
    fn follows_a_trajectory_and_holds_at_the_end() {
        let mut run = two_joints();
        run.submit(line([0.5, -0.2], 1.0)).unwrap();
        for _ in 0..50 {
            run.tick(DT);
        }
        assert_eq!(run.mode(), &Mode::Moving);
        assert!(
            (run.targets()[0] - 0.25).abs() < 1e-6,
            "{:?}",
            run.targets()
        );
        assert_eq!(position(&mut run, 0), steps(run.targets()[0]));
        until_holding(&mut run);
        assert_eq!(run.targets(), [0.5, -0.2]);
        assert_eq!(position(&mut run, 0), steps(0.5));
        assert_eq!(position(&mut run, 1), steps(-0.2));
    }

    #[test]
    fn bad_trajectories_are_refused_whole() {
        let mut run = two_joints();
        let cases = [
            (Trajectory { points: vec![] }, "점이 없다"),
            (
                Trajectory {
                    points: vec![(0.5, vec![0.0, 0.0])],
                },
                "첫 점의 시각",
            ),
            (
                Trajectory {
                    points: vec![(0.0, vec![0.0])],
                },
                "관절 2개",
            ),
            (
                Trajectory {
                    points: vec![(0.0, vec![0.0, 0.0]), (0.0, vec![0.0, 0.0])],
                },
                "시각은 늘어야",
            ),
            (
                Trajectory {
                    points: vec![(0.0, vec![0.1, 0.0]), (1.0, vec![0.2, 0.0])],
                },
                "시작하지 않는다",
            ),
            (line([0.5, 0.0], 0.1), "속도"),
            (line([2.5, 0.0], 2.0), "소프트 한계 밖"), // 143° > 118°
        ];
        for (trajectory, expected) in cases {
            let message = run.submit(trajectory).unwrap_err();
            assert!(message.contains(expected), "{expected}: {message}");
        }
        assert_eq!(run.mode(), &Mode::Holding);
        run.submit(line([0.5, 0.0], 1.0)).unwrap();
        let message = run.submit(line([0.5, 0.0], 1.0)).unwrap_err();
        assert!(message.contains("움직이는 중"), "{message}");
    }

    #[test]
    fn stop_slows_down_on_the_same_path_and_holds() {
        let mut run = two_joints();
        run.submit(line([1.0, 0.5], 1.0)).unwrap();
        for _ in 0..30 {
            run.tick(DT);
        }
        run.stop();
        assert_eq!(run.mode(), &Mode::Stopping);
        until_holding(&mut run);
        // 정지 시간 0.2 s 동안 빠르기를 0으로 줄이면 약 0.1 s만큼 더 간다
        let [a, b] = [run.targets()[0], run.targets()[1]];
        assert!((0.35..0.45).contains(&a), "{a}");
        assert!((b - a / 2.0).abs() < 1e-9, "같은 길이 아니다: {a} {b}");
        let held = position(&mut run, 0);
        for _ in 0..20 {
            run.tick(DT);
        }
        assert_eq!(position(&mut run, 0), held);
    }

    #[test]
    fn a_protection_bit_is_a_fault_until_a_person_clears_it() {
        let mut run = two_joints();
        run.submit(line([1.0, 0.5], 1.0)).unwrap();
        for _ in 0..10 {
            run.tick(DT);
        }
        run.joints.wire().servos[1].held = true;
        let held_at = position(&mut run, 1);
        run.tick(DT);
        run.joints.wire().servos[1].overload(); // 붙잡힌 채 보호 시간이 지났다
        let mode = run.tick(DT).clone();
        assert!(
            matches!(&mode, Mode::Fault(reason) if reason.contains("j4: 서보 보호가 걸렸다 (과부하)")),
            "{mode:?}"
        );
        // 고장이면 새 목표를 쓰지 않는다
        let stopped_at = position(&mut run, 0);
        for _ in 0..10 {
            run.tick(DT);
        }
        assert_eq!(position(&mut run, 0), stopped_at);
        assert!(
            run.submit(line([0.0, 0.0], 1.0))
                .unwrap_err()
                .contains("고장")
        );
        // 과부하로 묶인 채로는 풀리지 않는다. 원인을 없애고(놓고) 토크를 껐다 켜면 풀린다
        assert!(run.resume().unwrap_err().contains("서보 보호"));
        let servo = &mut run.joints.wire().servos[1];
        servo.release();
        servo.write(reg::TORQUE_ENABLE.address, &[0]);
        run.resume().unwrap();
        assert_eq!(run.mode(), &Mode::Holding);
        assert_eq!(steps(run.targets()[1]), held_at); // j4는 붙잡혔던 자리에서 버틴다
    }

    #[test]
    fn torque_turned_off_is_a_fault() {
        let mut run = two_joints();
        run.joints.wire().servos[0].power_cycle(); // 서보 전원이 꺼졌다 켜짐
        let mode = run.tick(DT).clone();
        assert!(
            matches!(&mode, Mode::Fault(reason) if reason.contains("j3: 토크가 꺼졌다")),
            "{mode:?}"
        );
        run.resume().unwrap(); // 시작 절차로 다시 켠다
        assert_eq!(run.mode(), &Mode::Holding);
        assert!(run.states().iter().all(|(_, state)| state.torque_on()));
    }

    #[test]
    fn repeated_read_failures_are_a_fault() {
        let mut run = two_joints();
        run.joints.wire().servos[0].faults.silent = true;
        for _ in 1..MAX_FAILURES {
            assert_eq!(run.tick(DT), &Mode::Holding);
        }
        let mode = run.tick(DT).clone();
        assert!(
            matches!(&mode, Mode::Fault(reason) if reason.contains("j3")),
            "{mode:?}"
        );
        run.joints.wire().servos[0].faults.silent = false;
        run.resume().unwrap();
        assert_eq!(run.tick(DT), &Mode::Holding);
    }
}
