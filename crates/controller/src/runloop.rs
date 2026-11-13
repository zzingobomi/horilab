//! 실행 루프 (docs/decisions/0032-run-loop.md의 만드는 순서 1, 2): 박자마다 서보 상태를 모두 읽고,
//! 검사하고, 궤적의 그 시각 목표를 쓴다. 서보를 움직이는 쓰기는 관절 층([`Joints`])을 지난다 (0009의 한곳
//! 안전 검사). 기준값은 로봇 설정의 [`LoopSettings`]다.
//!
//! - 궤적은 점마다 시각 [s]과 관절 각 [rad]이다. 지금 붙잡고 있는 목표에서 시작하고 소프트 한계와 속도
//!   한계 안이어야 받고, 아니면 실행 전에 통째로 거부한다 (0009). 점 사이는 곧게 잇는다.
//! - 정지는 같은 길을 따라가며 재생 빠르기를 정지 시간 동안 0으로 줄인 뒤 그 자리에서 버틴다.
//! - 서보 보호 비트, 꺼진 토크(서보 전원이 꺼졌다 켜짐), 잇단 통신 실패는 고장이다: 새 목표를 쓰지 않고
//!   (서보는 마지막 목표를 잡고 있다) 까닭을 남긴다. 사람이 [`RunLoop::resume`]으로 푼다.
//! - 서보 온도가 쉬는 기준에 닿으면 정지해 버티고, 알림 기준 아래로 식을 때까지 새 궤적을 받지 않는다 (쉬는
//!   자세가 정해지면 그리로 내린다, 0028). 알림 기준을 넘은 온도와 기준보다 낮은 전압은 알린다.
//! - 처짐 보정: 붙잡고 있는 동안 서보가 읽은 위치와 목표의 차이를 쌓아 쓰는 목표를 고친다 (0012). 움직이는
//!   동안에는 쌓은 값을 그대로 쓰고, 이득이 0이면 끈다.
//! - 그리퍼 서보는 관절과 같은 박자로 읽고 쓴다. 명령은 열림 폭이고, 정한 빠르기로 그 폭까지 간다. 목표 폭에
//!   닿으면 닿음, 그 전에 막혀 부하가 오르면 잡음이다 (0023). 정지나 고장에도 쥔 채 둔다.
//! - 시간은 부르는 쪽이 준다. 실물에서는 주기마다, 시험에서는 가짜 시계로 [`RunLoop::tick`]을 부른다.

use horilab_servo::feetech::Port;

use crate::joints::{Joints, MAX_FAILURES, Snapshot};
use crate::settings::{GripperSettings, LoopSettings};

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

/// 그리퍼의 상태.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Grip {
    /// 명령을 기다린다
    Idle,
    /// 목표 폭으로 가는 중
    Moving,
    /// 목표 폭에 닿았다 (가는 길에 막힌 것이 없었다). 열림 폭 [mm]
    Reached(f64),
    /// 목표 폭 전에 막혀 부하가 올랐다: 잡았다. 열림 폭 [mm]
    Grasped(f64),
}

struct Gripper {
    id: u8,
    /// 피니언 반지름 [mm]: 열림 폭 = 2 × 반지름 × 각
    radius: f64,
    max_width: f64,
    /// 열고 닫는 빠르기 [mm/s]
    speed: f64,
    tolerance: f64,
    /// 지금 쓰는 목표 각 [rad] (닫은 자리에서)
    goal: f64,
    /// 가려는 열림 폭 [mm]
    target: f64,
    state: Grip,
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
    /// 팔 관절의 서보 ID (궤적의 관절 순서)
    ids: Vec<u8>,
    settings: LoopSettings,
    gripper: Option<Gripper>,
    /// 지금 따르는 목표 [rad] (관절 순서)
    targets: Vec<f64>,
    /// 처짐 보정 [rad]: 서보에 쓰는 목표는 targets + corrections다
    corrections: Vec<f64>,
    /// 마지막 박자에 읽은 상태
    states: Vec<(u8, Snapshot)>,
    playing: Option<Playing>,
    mode: Mode,
    /// 서보가 뜨거워 쉬는 까닭. 있으면 새 궤적을 받지 않는다
    hot: Option<String>,
    /// 상태를 읽지 못한 박자가 잇달아 몇 번인지
    missed: u64,
}

impl<P: Port> RunLoop<P> {
    /// 관절 층의 서보를 모두 켜거나 이어받고(0009의 시작 절차), 지금 목표를 붙잡는다. 영점을 잰 로봇 관절만
    /// 받는다. 그리퍼가 있으면 그 서보도 관절 층에 있어야 한다.
    pub fn start(
        mut joints: Joints<P>,
        settings: LoopSettings,
        gripper: Option<&GripperSettings>,
    ) -> Result<Self, String> {
        let all = joints.ids();
        for &id in &all {
            if joints.servo(id)?.is_bench() {
                return Err(format!(
                    "서보 {id}: 실행 루프는 영점을 잰 로봇 관절만 다룬다"
                ));
            }
            joints.enable(id)?;
        }
        let gripper_id = gripper.map(|g| g.joint.id);
        if let Some(id) = gripper_id
            && !all.contains(&id)
        {
            return Err(format!("그리퍼 서보 {id}가 관절 층에 없다"));
        }
        let ids = all
            .into_iter()
            .filter(|&id| Some(id) != gripper_id)
            .collect();
        let gripper = gripper.map(|g| Gripper {
            id: g.joint.id,
            radius: g.pinion_radius,
            max_width: g.max_width,
            speed: g.speed,
            tolerance: g.grasp_tolerance,
            goal: 0.0,
            target: 0.0,
            state: Grip::Idle,
        });
        let mut run = Self {
            joints,
            ids,
            settings,
            gripper,
            targets: Vec::new(),
            corrections: Vec::new(),
            states: Vec::new(),
            playing: None,
            mode: Mode::Holding,
            hot: None,
            missed: 0,
        };
        run.hold_present_goals()?;
        Ok(run)
    }

    pub fn mode(&self) -> &Mode {
        &self.mode
    }

    /// 지금 따르는 목표 [rad] (관절 순서). 처짐 보정은 빠져 있다.
    pub fn targets(&self) -> &[f64] {
        &self.targets
    }

    /// 마지막 박자에 읽은 상태.
    pub fn states(&self) -> &[(u8, Snapshot)] {
        &self.states
    }

    /// 그리퍼의 열림 폭 [mm](마지막에 읽은 위치)과 상태. 그리퍼가 없으면 없다.
    pub fn gripper(&self) -> Option<(f64, Grip)> {
        let gripper = self.gripper.as_ref()?;
        let width = self.width_now(gripper.id, gripper.radius)?;
        Some((width, gripper.state))
    }

    /// 그리퍼를 열림 폭 `width` [mm]로 움직인다. 가는 길에 막혀 부하가 오르면 잡은 것이다 (박스를 안에서
    /// 벌려 잡을 때도 같다, 0030). 고장이면 받지 않는다.
    pub fn grip(&mut self, width: f64) -> Result<(), String> {
        if let Mode::Fault(reason) = &self.mode {
            return Err(format!("고장이라 그리퍼를 움직이지 않는다: {reason}"));
        }
        let Some(gripper) = &mut self.gripper else {
            return Err("그리퍼가 없다".to_owned());
        };
        if !(width.is_finite() && (0.0..=gripper.max_width).contains(&width)) {
            return Err(format!("열림 폭은 0~{} mm다: {width}", gripper.max_width));
        }
        gripper.target = width;
        gripper.state = Grip::Moving;
        Ok(())
    }

    /// 지금 알릴 것: 쉬는 까닭, 알림 기준에 닿은 온도, 기준보다 낮은 전압.
    pub fn warnings(&self) -> Vec<String> {
        let mut found: Vec<String> = self.hot.iter().cloned().collect();
        for (id, state) in &self.states {
            let temperature = f64::from(state.temperature);
            if temperature >= self.settings.temperature_warn {
                found.push(format!(
                    "{}: 온도 {temperature}°C가 알림 기준 {}°C에 닿았다",
                    self.name(*id),
                    self.settings.temperature_warn
                ));
            }
            let voltage = f64::from(state.voltage) / 10.0;
            if voltage < self.settings.voltage_warn {
                found.push(format!(
                    "{}: 전압 {voltage} V가 알림 기준 {} V보다 낮다",
                    self.name(*id),
                    self.settings.voltage_warn
                ));
            }
        }
        found
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
        self.watch_heat();
        self.advance(dt);
        self.correct_sag(dt);
        self.advance_gripper(dt);
        if let Err(e) = self
            .goals()
            .and_then(|goals| self.joints.write_goals(&goals))
        {
            self.fault(e);
        }
        &self.mode
    }

    /// 궤적을 받는다. 붙잡고 있을 때만 받고, 첫 점이 지금 목표이고 모든 점이 한계 안이어야 한다. 아니면
    /// 실행 전에 통째로 거부한다 (0009). 서보가 뜨거워 쉬는 동안에는 받지 않는다.
    pub fn submit(&mut self, trajectory: Trajectory) -> Result<(), String> {
        match &self.mode {
            Mode::Holding => {}
            Mode::Fault(reason) => return Err(format!("고장이라 궤적을 받지 않는다: {reason}")),
            Mode::Moving | Mode::Stopping => {
                return Err("움직이는 중에는 새 궤적을 받지 않는다 (정지한 뒤 보낸다)".to_owned());
            }
        }
        if let Some(reason) = &self.hot {
            return Err(format!(
                "서보가 뜨거워 쉬는 중이라 궤적을 받지 않는다: {reason}"
            ));
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

    /// 서보마다 남은 목표를 읽어 붙잡는다. 남은 목표에 처짐 보정이 들어 있으므로 보정은 0에서 다시 쌓는다.
    fn hold_present_goals(&mut self) -> Result<(), String> {
        let states = self.joints.read_all()?;
        let goal_of = |id: u8| -> Result<f64, String> {
            let state = states
                .iter()
                .find(|(known, _)| *known == id)
                .map(|(_, state)| state)
                .ok_or_else(|| format!("서보 {id}의 상태가 없다"))?;
            let servo = self.joints.servo(id)?;
            servo
                .angle_at(state.goal_position)
                .ok_or_else(|| format!("{}: 영점이 없다", servo.name))
        };
        let targets = self
            .ids
            .iter()
            .map(|&id| goal_of(id))
            .collect::<Result<Vec<f64>, String>>()?;
        let gripper_goal = match &self.gripper {
            Some(gripper) => Some(goal_of(gripper.id)?),
            None => None,
        };
        if let (Some(gripper), Some(goal)) = (&mut self.gripper, gripper_goal) {
            gripper.goal = goal;
            gripper.target = 2.0 * gripper.radius * goal;
            gripper.state = Grip::Idle;
        }
        self.corrections = vec![0.0; targets.len()];
        self.targets = targets;
        self.states = states;
        self.playing = None;
        self.mode = Mode::Holding;
        self.missed = 0;
        Ok(())
    }

    fn name(&self, id: u8) -> String {
        self.joints
            .servo(id)
            .map_or_else(|_| format!("서보 {id}"), |servo| servo.name.clone())
    }

    /// 읽은 상태에서 고장을 찾는다: 서보 보호 비트, 꺼진 토크.
    fn check(&self) -> Option<String> {
        self.states.iter().find_map(|(id, state)| {
            if !state.status.is_none() {
                Some(format!(
                    "{}: 서보 보호가 걸렸다 ({})",
                    self.name(*id),
                    state.status
                ))
            } else if !state.torque_on() {
                Some(format!(
                    "{}: 토크가 꺼졌다 (서보 전원이 꺼졌다 켜졌을 수 있다)",
                    self.name(*id)
                ))
            } else {
                None
            }
        })
    }

    /// 가장 뜨거운 서보가 쉬는 기준에 닿으면 정지해 쉬고, 알림 기준 아래로 식으면 쉬기를 푼다.
    fn watch_heat(&mut self) {
        let Some((id, temperature)) = self
            .states
            .iter()
            .map(|(id, state)| (*id, state.temperature))
            .max_by_key(|&(_, temperature)| temperature)
        else {
            return;
        };
        let temperature = f64::from(temperature);
        if temperature >= self.settings.temperature_rest {
            if self.hot.is_none() {
                self.hot = Some(format!(
                    "{}: 온도 {temperature}°C가 쉬는 기준 {}°C에 닿았다",
                    self.name(id),
                    self.settings.temperature_rest
                ));
            }
            self.stop();
        } else if temperature < self.settings.temperature_warn {
            self.hot = None;
        }
    }

    /// 궤적을 `dt`만큼 재생한다. 정지 중이면 재생 빠르기를 줄이고, 끝나거나 멈추면 붙잡는다.
    fn advance(&mut self, dt: f64) {
        let stopping = self.mode == Mode::Stopping;
        let Some(playing) = &mut self.playing else {
            return;
        };
        if stopping {
            playing.rate = (playing.rate - dt / self.settings.stop_time).max(0.0);
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

    /// 붙잡고 있는 동안 서보가 읽은 위치와 목표의 차이를 쌓아 처짐을 보정한다 (0012).
    fn correct_sag(&mut self, dt: f64) {
        let (gain, max) = (self.settings.sag_gain, self.settings.sag_max);
        if gain <= 0.0 || self.mode != Mode::Holding {
            return;
        }
        for (index, &id) in self.ids.iter().enumerate() {
            let Some(measured) = self.angle_now(id) else {
                continue;
            };
            let error = self.targets[index] - measured;
            self.corrections[index] =
                (self.corrections[index] + gain * error * dt).clamp(-max, max);
        }
    }

    /// 마지막에 읽은 서보 위치의 각 [rad].
    fn angle_now(&self, id: u8) -> Option<f64> {
        let (_, state) = self.states.iter().find(|(known, _)| *known == id)?;
        self.joints.servo(id).ok()?.angle_at(state.position)
    }

    /// 마지막에 읽은 그리퍼의 열림 폭 [mm].
    fn width_now(&self, id: u8, radius: f64) -> Option<f64> {
        self.angle_now(id).map(|angle| 2.0 * radius * angle)
    }

    /// 그리퍼 목표를 빠르기만큼 옮긴다. 목표 폭의 목표를 쓴 뒤의 박자부터, 새로 읽은 위치와 부하로 닿았는지
    /// 잡았는지 본다 (목표를 쓰기 전에 읽은 상태로 보면 한 박자 늦은 값이다).
    fn advance_gripper(&mut self, dt: f64) {
        let Some(gripper) = &self.gripper else {
            return;
        };
        if gripper.state != Grip::Moving {
            return;
        }
        let (id, radius, tolerance) = (gripper.id, gripper.radius, gripper.tolerance);
        let end = gripper.target / (2.0 * radius);
        let step = gripper.speed / (2.0 * radius) * dt;
        // 목표 폭에 이르면 목표를 끝 값 그대로 두므로, 다음 박자부터는 차이가 정확히 0이다
        let written = gripper.goal == end;
        let goal = if (end - gripper.goal).abs() <= step {
            end
        } else {
            gripper.goal + step.copysign(end - gripper.goal)
        };
        let target = gripper.target;
        let judged = if written {
            self.judge_grip(id, radius, target, tolerance)
        } else {
            None
        };
        if let Some(gripper) = &mut self.gripper {
            gripper.goal = goal;
            if let Some(state) = judged {
                gripper.state = state;
            }
        }
    }

    /// 목표 폭에 이른 뒤의 판정: 폭이 목표 안이면 닿음, 멈춘 채 토크 한도 가까이 밀고 있으면 잡음.
    fn judge_grip(&self, id: u8, radius: f64, target: f64, tolerance: f64) -> Option<Grip> {
        let width = self.width_now(id, radius)?;
        if (width - target).abs() <= tolerance {
            return Some(Grip::Reached(width));
        }
        let (_, state) = self.states.iter().find(|(known, _)| *known == id)?;
        let pushing = state.torque_limit > 0 && state.load.abs() * 10 >= state.torque_limit * 8;
        (pushing && !state.moving).then_some(Grip::Grasped(width))
    }

    /// 이번 박자에 쓸 목표: 팔 관절(목표 + 처짐 보정)과 그리퍼. 한계 안으로 자른다.
    fn goals(&self) -> Result<Vec<(u8, i32)>, String> {
        let arm = self
            .ids
            .iter()
            .zip(self.targets.iter().zip(&self.corrections))
            .map(|(&id, (&angle, &correction))| (id, angle + correction));
        let gripper = self
            .gripper
            .as_ref()
            .map(|gripper| (gripper.id, gripper.goal));
        arm.chain(gripper)
            .map(|(id, angle)| {
                let servo = self.joints.servo(id)?;
                servo
                    .raw_at(angle)
                    .map(|raw| (id, raw.clamp(servo.min_raw, servo.max_raw)))
                    .ok_or_else(|| format!("{}: 영점이 없다", servo.name))
            })
            .collect()
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
        for pair in points.windows(2) {
            let ((t0, q0), (t1, q1)) = (&pair[0], &pair[1]);
            if t1 <= t0 {
                return Err(format!("시각은 늘어야 한다: {t0} → {t1} s"));
            }
            for (index, (a, b)) in q0.iter().zip(q1).enumerate() {
                let speed = (b - a).abs() / (t1 - t0);
                if speed > self.settings.max_speed {
                    return Err(format!(
                        "{}: {t0}~{t1} s의 속도 {speed:.3} rad/s가 한계 {:.3} rad/s를 넘는다",
                        self.name(self.ids[index]),
                        self.settings.max_speed
                    ));
                }
            }
        }
        for (index, (&start, &now)) in first.iter().zip(&self.targets).enumerate() {
            if (start - now).abs() > self.settings.start_tolerance {
                return Err(format!(
                    "{}: 첫 점 {start:.4} rad가 지금 목표 {now:.4} rad에서 시작하지 않는다",
                    self.name(self.ids[index])
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
    use crate::settings::{Calibration, Direction, RAD_PER_STEP, Settings, TEST_GRIPPER, TEST_J3};

    /// 시험용 기준값 (로봇 설정의 시험값보다 빠르게 움직여 시험을 짧게 한다)
    const SETTINGS: LoopSettings = LoopSettings {
        max_speed: 2.0,
        start_tolerance: 0.01,
        stop_time: 0.2,
        temperature_warn: 50.0,
        temperature_rest: 55.0,
        voltage_warn: 11.0,
        sag_gain: 0.0,
        sag_max: 0.05,
    };
    const DT: f64 = 0.01;

    /// J3과 J4 (둘 다 옛 시험의 J3 한계 -30°~118°, 영점 2048에서 0°)
    fn two_joints_with(settings: LoopSettings) -> RunLoop<FakePort> {
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
            settings,
            None,
        )
        .unwrap()
    }

    fn two_joints() -> RunLoop<FakePort> {
        two_joints_with(SETTINGS)
    }

    fn line(to: [f64; 2], seconds: f64) -> Trajectory {
        Trajectory {
            points: vec![(0.0, vec![0.0, 0.0]), (seconds, to.to_vec())],
        }
    }

    fn fake(run: &mut RunLoop<FakePort>, index: usize) -> &mut FakeServo {
        &mut run.joints.wire().servos[index]
    }

    fn position(run: &mut RunLoop<FakePort>, index: usize) -> i32 {
        fake(run, index).get(reg::PRESENT_POSITION)
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
        assert!(run.warnings().is_empty());
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
        fake(&mut run, 1).held = true;
        let held_at = position(&mut run, 1);
        run.tick(DT);
        fake(&mut run, 1).overload(); // 붙잡힌 채 보호 시간이 지났다
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
        let message = run.submit(line([0.0, 0.0], 1.0)).unwrap_err();
        assert!(message.contains("고장"), "{message}");
        // 과부하로 묶인 채로는 풀리지 않는다. 원인을 없애고(놓고) 토크를 껐다 켜면 풀린다
        assert!(run.resume().unwrap_err().contains("서보 보호"));
        let servo = fake(&mut run, 1);
        servo.release();
        servo.write(reg::TORQUE_ENABLE.address, &[0]);
        run.resume().unwrap();
        assert_eq!(run.mode(), &Mode::Holding);
        assert_eq!(steps(run.targets()[1]), held_at); // j4는 붙잡혔던 자리에서 버틴다
    }

    #[test]
    fn torque_turned_off_is_a_fault() {
        let mut run = two_joints();
        fake(&mut run, 0).power_cycle(); // 서보 전원이 꺼졌다 켜짐
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
        fake(&mut run, 0).faults.silent = true;
        for _ in 1..MAX_FAILURES {
            assert_eq!(run.tick(DT), &Mode::Holding);
        }
        let mode = run.tick(DT).clone();
        assert!(
            matches!(&mode, Mode::Fault(reason) if reason.contains("j3")),
            "{mode:?}"
        );
        fake(&mut run, 0).faults.silent = false;
        run.resume().unwrap();
        assert_eq!(run.tick(DT), &Mode::Holding);
    }

    #[test]
    fn a_hot_servo_stops_the_arm_until_it_cools() {
        let mut run = two_joints();
        run.submit(line([1.0, 0.5], 1.0)).unwrap();
        for _ in 0..10 {
            run.tick(DT);
        }
        fake(&mut run, 1).set(reg::PRESENT_TEMPERATURE, 52);
        run.tick(DT);
        assert_eq!(run.mode(), &Mode::Moving); // 알림 기준만 넘으면 알리기만 한다
        let warnings = run.warnings();
        assert!(
            warnings[0].contains("j4: 온도 52°C가 알림 기준 50°C"),
            "{warnings:?}"
        );
        fake(&mut run, 1).set(reg::PRESENT_TEMPERATURE, 56);
        run.tick(DT);
        assert_eq!(run.mode(), &Mode::Stopping);
        until_holding(&mut run);
        let now = run.targets().to_vec();
        let stay = Trajectory {
            points: vec![(0.0, now.clone()), (1.0, now)],
        };
        let message = run.submit(stay.clone()).unwrap_err();
        assert!(message.contains("뜨거워"), "{message}");
        // 알림 기준 아래로 식어야 다시 받는다
        fake(&mut run, 1).set(reg::PRESENT_TEMPERATURE, 52);
        run.tick(DT);
        assert!(run.submit(stay.clone()).is_err());
        fake(&mut run, 1).set(reg::PRESENT_TEMPERATURE, 45);
        run.tick(DT);
        assert!(run.warnings().is_empty());
        run.submit(stay).unwrap();
    }

    #[test]
    fn low_voltage_is_warned_but_does_not_stop() {
        let mut run = two_joints();
        fake(&mut run, 0).set(reg::PRESENT_VOLTAGE, 105);
        run.tick(DT);
        let warnings = run.warnings();
        assert!(
            warnings
                .iter()
                .any(|w| w.contains("j3: 전압 10.5 V가 알림 기준 11 V")),
            "{warnings:?}"
        );
        assert_eq!(run.mode(), &Mode::Holding);
    }

    #[test]
    fn sag_correction_raises_the_goal_until_the_servo_reads_the_target() {
        let mut settings = SETTINGS;
        settings.sag_gain = 20.0;
        let mut run = two_joints_with(settings);
        fake(&mut run, 0).sag = 10; // 짐에 눌려 10칸 처진다
        for _ in 0..300 {
            run.tick(DT);
        }
        assert_eq!(position(&mut run, 0), 2048); // 서보가 읽는 위치가 목표에 닿았다
        assert_eq!(fake(&mut run, 0).get(reg::GOAL_POSITION), 2058);
        assert_eq!(run.targets()[0], 0.0); // 따르는 목표는 그대로다
        // 처짐 보정이 꺼져 있으면 처진 채 둔다
        let mut run = two_joints();
        fake(&mut run, 0).sag = 10;
        for _ in 0..300 {
            run.tick(DT);
        }
        assert_eq!(position(&mut run, 0), 2038);
    }

    /// J3과 그리퍼(ID 11, 닫은 자리 2048)
    fn with_gripper() -> RunLoop<FakePort> {
        let settings = Settings::parse(&format!("{TEST_J3}{TEST_GRIPPER}")).unwrap();
        let gripper = settings.gripper.clone().unwrap();
        let mut j3 = settings.joint("j3").unwrap().clone();
        j3.calibration = Some(Calibration {
            pin: 0.0,
            zero_raw: 2048,
            direction: Direction::Same,
            position_offset: 0,
        });
        let servos = vec![
            Servo::from_joint(&j3).unwrap(),
            Servo::from_joint(&gripper.joint).unwrap(),
        ];
        let bus = open_fake_bus([
            FakeServo::with(3, 2825, 2048),
            FakeServo::with(11, 2825, 2048),
        ]);
        let joints = Joints::open(bus, servos, BENCH_CAPS).unwrap();
        RunLoop::start(joints, SETTINGS, Some(&gripper)).unwrap()
    }

    fn until_grip_settles(run: &mut RunLoop<FakePort>) -> Grip {
        for _ in 0..1000 {
            run.tick(DT);
            let (_, state) = run.gripper().unwrap();
            if state != Grip::Moving {
                return state;
            }
        }
        panic!("그리퍼가 멈추지 않았다");
    }

    #[test]
    fn gripper_opens_to_a_width_at_its_speed() {
        let mut run = with_gripper();
        assert_eq!(run.gripper(), Some((0.0, Grip::Idle)));
        assert_eq!(run.targets().len(), 1); // 궤적은 팔 관절만
        run.grip(30.0).unwrap();
        for _ in 0..50 {
            run.tick(DT);
        }
        let (width, state) = run.gripper().unwrap();
        assert_eq!(state, Grip::Moving);
        assert!((width - 10.0).abs() < 0.5, "{width}"); // 20 mm/s로 0.5 s
        let Grip::Reached(width) = until_grip_settles(&mut run) else {
            panic!("닿지 않았다");
        };
        assert!((width - 30.0).abs() < 0.1, "{width}");
        assert!(run.grip(61.0).unwrap_err().contains("0~60 mm"));
    }

    #[test]
    fn gripper_grasps_when_blocked_and_keeps_holding_through_a_stop() {
        let mut run = with_gripper();
        run.grip(40.0).unwrap();
        for _ in 0..60 {
            run.tick(DT);
        }
        fake(&mut run, 1).held = true; // 손가락이 물체에 닿았다
        let Grip::Grasped(width) = until_grip_settles(&mut run) else {
            panic!("잡지 않았다");
        };
        assert!((10.0..14.0).contains(&width), "{width}");
        // 팔이 멈추거나 고장 나도 쥔 채 둔다 (목표와 토크 한도가 그대로다)
        let goal = fake(&mut run, 1).get(reg::GOAL_POSITION);
        run.submit(Trajectory {
            points: vec![(0.0, vec![0.0]), (1.0, vec![0.5])],
        })
        .unwrap();
        run.tick(DT);
        run.stop();
        until_holding(&mut run);
        assert_eq!(fake(&mut run, 1).get(reg::GOAL_POSITION), goal);
        fake(&mut run, 0).power_cycle();
        assert!(matches!(run.tick(DT), Mode::Fault(_)));
        let servo = fake(&mut run, 1);
        assert_eq!(servo.get(reg::GOAL_POSITION), goal);
        assert_eq!(servo.get(reg::TORQUE_LIMIT), 150);
    }
}
