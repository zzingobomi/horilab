//! 경로 계획 (docs/decisions/0033-planning-and-simulation.md의 만드는 순서 3): 관절 이동, 자세로 가기, 직선
//! 이동. 궤적은 모두 저크를 제한하고(관절 이동은 rsruckig, 직선 이동은 [`crate::timing`]), 실행 루프의 박자마다
//! 점 하나다. 실행 루프가 실행 전에 한계를 다시 본다 (0009).

use std::f64::consts::{PI, TAU};

use rsruckig::prelude::*;

use crate::kinematics::{Kinematics, Pose, PoseLine, wrap};
use crate::runloop::Trajectory;
use crate::settings::LoopSettings;
use crate::timing::{FINE, time_path};

/// 궤적이 이보다 길면 만들지 않는다 (박자 수). 100 Hz로 10분이다.
const MAX_POINTS: usize = 60_000;
/// 직선 이동에서 길 전체를 미리 볼 때 IK를 푸는 간격: 손끝이 이만큼 [m] 가거나 [rad] 돌 때마다 (0.1 mm, 0.1°)
const LINE_STEP: f64 = 1e-4;
const LINE_STEP_ANGLE: f64 = 0.1 * PI / 180.0;
/// 직선 이동에서 IK를 이어 풀 때 한 걸음에 관절이 이보다 많이 바뀌면 해가 끊긴 것으로 본다 [rad] (5°)
const LINE_JUMP: f64 = 5.0 * PI / 180.0;

/// 관절 이동의 한계 (관절마다, rad와 s).
#[derive(Debug, Clone, PartialEq)]
pub struct MotionLimits {
    /// 속도 [rad/s]
    pub velocity: Vec<f64>,
    /// 가속 [rad/s²]
    pub acceleration: Vec<f64>,
    /// 저크 [rad/s³]
    pub jerk: Vec<f64>,
}

impl MotionLimits {
    /// 로봇 설정의 실행 루프 기준값 (모든 관절이 같은 한계).
    pub fn from_settings(settings: &LoopSettings, joints: usize) -> Self {
        Self::same(
            joints,
            settings.max_speed,
            settings.max_acceleration,
            settings.max_jerk,
        )
    }

    /// 모든 관절이 같은 한계다.
    pub fn same(joints: usize, velocity: f64, acceleration: f64, jerk: f64) -> Self {
        Self {
            velocity: vec![velocity; joints],
            acceleration: vec![acceleration; joints],
            jerk: vec![jerk; joints],
        }
    }
}

/// 관절 이동: `start`에서 `goal`까지 멈춘 채 시작해 멈춘 채 끝나고, 모든 관절이 같이 끝난다. 점은 `period`마다
/// 하나이고 마지막 점은 `goal` 그대로다.
pub fn move_joints(
    start: &[f64],
    goal: &[f64],
    limits: &MotionLimits,
    period: f64,
) -> Result<Trajectory, String> {
    let n = start.len();
    let sizes = [
        goal.len(),
        limits.velocity.len(),
        limits.acceleration.len(),
        limits.jerk.len(),
    ];
    if n == 0 || sizes.iter().any(|&size| size != n) {
        return Err(format!(
            "관절 수가 맞지 않는다: 시작 {n}, 목표와 한계 {sizes:?}"
        ));
    }
    let positive = |values: &[f64]| values.iter().all(|v| v.is_finite() && *v > 0.0);
    if !(period.is_finite() && period > 0.0)
        || !positive(&limits.velocity)
        || !positive(&limits.acceleration)
        || !positive(&limits.jerk)
    {
        return Err("박자 길이와 한계는 0보다 크다".to_owned());
    }
    if start.iter().chain(goal).any(|v| !v.is_finite()) {
        return Err("관절 각에 수가 아닌 값이 있다".to_owned());
    }
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(n), period);
    let mut input = InputParameter::new(Some(n));
    let mut output = OutputParameter::new(Some(n));
    for i in 0..n {
        input.current_position[i] = start[i];
        input.target_position[i] = goal[i];
        input.max_velocity[i] = limits.velocity[i];
        input.max_acceleration[i] = limits.acceleration[i];
        input.max_jerk[i] = limits.jerk[i];
    }
    let mut points = vec![(0.0, start.to_vec())];
    loop {
        let result = otg
            .update(&input, &mut output)
            .map_err(|e| format!("궤적을 만들지 못했다: {e:?}"))?;
        let time = points.len() as f64 * period;
        points.push((time, (0..n).map(|i| output.new_position[i]).collect()));
        match result {
            RuckigResult::Working => output.pass_to_input(&mut input),
            RuckigResult::Finished => break,
            other => return Err(format!("궤적을 만들지 못했다: {other:?}")),
        }
        if points.len() > MAX_POINTS {
            return Err(format!("궤적이 박자 {MAX_POINTS}개보다 길다"));
        }
    }
    if let Some((_, last)) = points.last_mut() {
        last.copy_from_slice(goal);
    }
    Ok(Trajectory { points })
}

/// IK의 해 가운데 관절 한계 안이고 지금 관절 각에 가장 가까운 해. 한계가 ±π를 넘는 관절은 2π 차이 나는 각도
/// 본다 (예: J4의 80°~235°에서 -160°는 200°다).
pub fn closest_solution(
    kinematics: &Kinematics,
    target: &Pose,
    current: &[f64; 6],
    limits: &[(f64, f64); 6],
) -> Option<[f64; 6]> {
    let distance = |q: &[f64; 6]| {
        q.iter()
            .zip(current)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
    };
    kinematics
        .inverse(target)
        .into_iter()
        .filter_map(|solution| {
            let mut chosen = [0.0; 6];
            for i in 0..6 {
                let (low, high) = limits[i];
                chosen[i] = [-TAU, 0.0, TAU]
                    .map(|turn| solution[i] + turn)
                    .into_iter()
                    .filter(|angle| (low..=high).contains(angle))
                    .min_by(|a, b| (a - current[i]).abs().total_cmp(&(b - current[i]).abs()))?;
            }
            Some(chosen)
        })
        .min_by(|a, b| distance(a).total_cmp(&distance(b)))
}

/// 자세로 가기: 지금 관절 각에서 IK의 가장 가까운 해(관절 한계 안)까지 관절 이동한다.
pub fn move_to_pose(
    kinematics: &Kinematics,
    target: &Pose,
    current: &[f64; 6],
    limits: &[(f64, f64); 6],
    motion: &MotionLimits,
    period: f64,
) -> Result<Trajectory, String> {
    let goal = closest_solution(kinematics, target, current, limits)
        .ok_or_else(|| "자세에 닿는 해가 관절 한계 안에 없다".to_owned())?;
    move_joints(current, &goal, motion, period)
}

/// 직선 이동의 손끝 한계.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToolLimits {
    /// 선속도 [m/s]
    pub speed: f64,
    /// 각속도 [rad/s]
    pub rotation: f64,
}

impl ToolLimits {
    /// 로봇 설정의 실행 루프 기준값 (직교 조그와 같은 한계).
    pub fn from_settings(settings: &LoopSettings) -> Self {
        Self {
            speed: settings.max_tool_speed,
            rotation: settings.max_tool_rotation,
        }
    }
}

/// 직선 이동: 플랜지가 지금 자세에서 `target`까지 곧은 선을 따라가고, 방향은 가장 짧은 회전으로 고르게 바뀐다.
///
/// - 지금 해의 갈래(어깨, 팔꿈치, 손목)를 지키며 IK를 이어 푼다. 먼저 길 전체를 촘촘히 풀어, 닿지 않는 곳, 관절
///   한계 밖, 해가 끊기는 곳(특이 자세를 지남)이 있으면 거절한다.
/// - 언제 어디까지 갈지는 [`crate::timing`]이 정한다: 손끝 한계와 관절 한계 안에서, 관절 한계 때문에 늦춰야 하는
///   곳(특이 자세 가까이 등)만 늦추고 저크를 제한한다. 멈춘 채 시작해 멈춘 채 끝난다.
/// - 박자마다의 관절 각에서 속도·가속·저크를 다시 재어, 관절 한계를 넘으면 전체를 같은 비율로 늦춘다 (안전망).
pub fn move_line(
    kinematics: &Kinematics,
    target: &Pose,
    current: &[f64; 6],
    limits: &[(f64, f64); 6],
    motion: &MotionLimits,
    tool: &ToolLimits,
    period: f64,
) -> Result<Trajectory, String> {
    let sizes = [
        motion.velocity.len(),
        motion.acceleration.len(),
        motion.jerk.len(),
    ];
    if sizes.iter().any(|&size| size != 6) {
        return Err(format!("관절 한계는 여섯 관절의 수다: {sizes:?}"));
    }
    let positive = |values: &[f64]| values.iter().all(|v| v.is_finite() && *v > 0.0);
    if !(period.is_finite() && period > 0.0)
        || !positive(&motion.velocity)
        || !positive(&motion.acceleration)
        || !positive(&motion.jerk)
        || !positive(&[tool.speed, tool.rotation])
    {
        return Err("박자 길이와 한계는 0보다 크다".to_owned());
    }
    let finite = current
        .iter()
        .chain(&target.position)
        .chain(target.rotation.iter().flatten())
        .all(|v| v.is_finite());
    if !finite {
        return Err("관절 각이나 목표 자세에 수가 아닌 값이 있다".to_owned());
    }
    let line = PoseLine::new(&kinematics.forward(current), target);
    let (length, angle) = (line.length(), line.angle());
    if length < 1e-9 && angle < 1e-9 {
        return Ok(Trajectory {
            points: vec![(0.0, current.to_vec())],
        });
    }
    if kinematics.inverse(target).is_empty() {
        return Err("목표 자세에 팔이 닿지 않는다".to_owned());
    }
    // 길 전체를 촘촘히 풀어 본다 (도함수를 차분으로 구하려고 걸음은 둘 이상)
    let steps = ((length / LINE_STEP).max(angle / LINE_STEP_ANGLE).ceil() as usize).max(2);
    let mut path = Vec::with_capacity(steps + 1);
    path.push(*current);
    for k in 1..=steps {
        let u = k as f64 / steps as f64;
        let next = follow(kinematics, &line.at(u), &path[k - 1], u)?;
        path.push(next);
    }
    for (k, q) in path.iter().enumerate() {
        if let Some(i) = (0..6).find(|&i| !(limits[i].0..=limits[i].1).contains(&q[i])) {
            return Err(format!(
                "직선의 {:.0}% 자리에서 J{}가 관절 한계 밖이다 ({:.1}°)",
                100.0 * k as f64 / steps as f64,
                i + 1,
                q[i].to_degrees()
            ));
        }
    }
    // u에 대한 관절 각의 도함수 (안쪽은 가운데 차분, 양 끝은 옆 역의 값)
    let du = 1.0 / steps as f64;
    let mut dq = vec![[0.0; 6]; steps + 1];
    let mut ddq = vec![[0.0; 6]; steps + 1];
    for k in 1..steps {
        let (back, here, ahead) = (&path[k - 1], &path[k], &path[k + 1]);
        dq[k] = std::array::from_fn(|i| (ahead[i] - back[i]) / (2.0 * du));
        ddq[k] = std::array::from_fn(|i| (ahead[i] - 2.0 * here[i] + back[i]) / (du * du));
    }
    (dq[0], ddq[0]) = (dq[1], ddq[1]);
    (dq[steps], ddq[steps]) = (dq[steps - 1], ddq[steps - 1]);
    // 손끝 한계가 허락하는 u의 빠르기 [1/s]
    let mut tool_rate = f64::INFINITY;
    if length > 0.0 {
        tool_rate = tool_rate.min(tool.speed / length);
    }
    if angle > 0.0 {
        tool_rate = tool_rate.min(tool.rotation / angle);
    }
    let samples = time_path(&dq, &ddq, du, tool_rate, motion, period)?;
    // 박자마다 뽑아 IK를 이어 풀고, 관절 한계를 넘으면 전체를 같은 비율로 늦춰 다시 뽑는다
    let last = (samples.len() - 1) as f64;
    let mut scale = 1.0_f64;
    for _ in 0..4 {
        let ticks = (last * FINE * scale / period).ceil() as usize;
        if ticks > MAX_POINTS {
            return Err(format!("궤적이 박자 {MAX_POINTS}개보다 길다"));
        }
        let mut points = vec![*current];
        for tick in 1..=ticks {
            let at = (tick as f64 * period / scale / FINE).min(last);
            let (low, high) = (
                at.floor() as usize,
                (at.ceil() as usize).min(samples.len() - 1),
            );
            let u = if tick == ticks {
                1.0
            } else {
                samples[low] + (samples[high] - samples[low]) * (at - low as f64)
            };
            let next = follow(kinematics, &line.at(u), &points[points.len() - 1], u)?;
            points.push(next);
        }
        let over = excess(&points, motion, period);
        if over <= 1.0 {
            let points = points
                .iter()
                .enumerate()
                .map(|(n, q)| (n as f64 * period, q.to_vec()))
                .collect();
            return Ok(Trajectory { points });
        }
        scale *= over * 1.01;
    }
    Err("직선 이동을 관절 한계 안으로 늦추지 못했다".to_owned())
}

/// 직선 위 자세의 IK 해 가운데 `previous`에서 이어지는 해: 관절마다 2π 차이 나는 각 가운데 가장 가까운 것으로
/// 맞춘 해 가운데 가장 가까운 것. 해가 없거나 관절이 [`LINE_JUMP`]보다 많이 바뀌면 거절한다. `u`는 알림에 쓴다.
fn follow(
    kinematics: &Kinematics,
    pose: &Pose,
    previous: &[f64; 6],
    u: f64,
) -> Result<[f64; 6], String> {
    let distance = |q: &[f64; 6]| {
        q.iter()
            .zip(previous)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
    };
    let next = kinematics
        .inverse(pose)
        .into_iter()
        .map(|solution| {
            let unwrapped: [f64; 6] =
                std::array::from_fn(|i| previous[i] + wrap(solution[i] - previous[i]));
            unwrapped
        })
        .min_by(|a, b| distance(a).total_cmp(&distance(b)))
        .ok_or_else(|| format!("직선의 {:.0}% 자리에 팔이 닿지 않는다", 100.0 * u))?;
    let jump = next
        .iter()
        .zip(previous)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    if jump > LINE_JUMP {
        return Err(format!(
            "직선의 {:.0}% 자리에서 IK 해가 끊긴다 (특이 자세를 지난다). 관절 이동을 쓴다",
            100.0 * u
        ));
    }
    Ok(next)
}

/// 박자마다의 관절 각을 차분해 본 속도, 가속, 저크가 관절 한계를 넘는 정도: 전체를 같은 비율로 늦출 때 필요한
/// 배수 (속도 비, 가속 비의 제곱근, 저크 비의 세제곱근 가운데 가장 큰 것). 앞뒤로는 멈춰 있다고 본다.
fn excess(points: &[[f64; 6]], motion: &MotionLimits, period: f64) -> f64 {
    let (first, last) = (points[0], points[points.len() - 1]);
    let padded: Vec<[f64; 6]> = std::iter::repeat_n(first, 3)
        .chain(points.iter().copied())
        .chain(std::iter::repeat_n(last, 3))
        .collect();
    let mut worst = 0.0_f64;
    for i in 0..6 {
        let mut values: Vec<f64> = padded.iter().map(|q| q[i]).collect();
        let limits = [motion.velocity[i], motion.acceleration[i], motion.jerk[i]];
        for (order, limit) in (1..).zip(limits) {
            values = values
                .windows(2)
                .map(|pair| (pair[1] - pair[0]) / period)
                .collect();
            let peak = values.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            worst = worst.max((peak / limit).powf(1.0 / f64::from(order)));
        }
    }
    worst
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::{Settings, TEST_J3};

    const PERIOD: f64 = 0.01;

    fn speeds(trajectory: &Trajectory) -> Vec<Vec<f64>> {
        trajectory
            .points
            .windows(2)
            .map(|pair| {
                let ((t0, q0), (t1, q1)) = (&pair[0], &pair[1]);
                q0.iter()
                    .zip(q1)
                    .map(|(a, b)| (b - a) / (t1 - t0))
                    .collect()
            })
            .collect()
    }

    #[test]
    fn move_starts_and_ends_at_rest_within_the_limits() {
        let limits = MotionLimits::same(2, 1.0, 2.0, 10.0);
        let trajectory = move_joints(&[0.0, 0.5], &[1.0, 0.0], &limits, PERIOD).unwrap();
        let points = &trajectory.points;
        assert_eq!(points[0], (0.0, vec![0.0, 0.5]));
        assert_eq!(points.last().unwrap().1, vec![1.0, 0.0]);
        for (index, (time, _)) in points.iter().enumerate() {
            assert!((time - index as f64 * PERIOD).abs() < 1e-9);
        }
        let speeds = speeds(&trajectory);
        let fastest = speeds.iter().flatten().fold(0.0_f64, |m, v| m.max(v.abs()));
        assert!(fastest <= 1.0 + 1e-9 && fastest > 0.9, "{fastest}");
        // 멈춘 채 시작하고 멈춘 채 끝난다 (첫 박자와 마지막 박자의 움직임이 작다)
        for step in [speeds.first().unwrap(), speeds.last().unwrap()] {
            assert!(step.iter().all(|v| v.abs() < 0.05), "{step:?}");
        }
        // 1 rad를 1 rad/s로: 가속 구간이 있어 1초보다 조금 길다
        let duration = points.last().unwrap().0;
        assert!((1.0..2.0).contains(&duration), "{duration}");
    }

    #[test]
    fn all_joints_finish_together() {
        let limits = MotionLimits::same(2, 1.0, 2.0, 10.0);
        let trajectory = move_joints(&[0.0, 0.0], &[1.0, 0.1], &limits, PERIOD).unwrap();
        // 짧게 가는 관절도 끝까지 움직인다 (먼저 서지 않는다)
        let points = &trajectory.points;
        let before_end = &points[points.len() - 5].1;
        assert!(before_end[1] < 0.1 - 1e-6, "{before_end:?}");
    }

    /// 설계의 지금 관절 범위 (params/arm.toml)
    fn design_limits() -> [(f64, f64); 6] {
        [
            (-175.0, 175.0),
            (-110.0, 110.0),
            (-30.0, 118.0),
            (80.0, 235.0),
            (-15.0, 140.0),
            (-95.0, 115.0),
        ]
        .map(|(low, high): (f64, f64)| (low.to_radians(), high.to_radians()))
    }

    #[test]
    fn move_to_pose_takes_the_nearest_solution_in_limits() {
        let kinematics = Settings::parse(TEST_J3).unwrap().kinematics;
        let limits = design_limits();
        // J4 200°는 IK가 -160°로 내지만 한계(80°~235°) 안의 200°를 고른다
        let q = [0.3, -0.4, 0.9, 200f64.to_radians(), 0.8, -0.5];
        let target = kinematics.forward(&q);
        let current = [0.2, -0.3, 0.8, 3.4, 0.7, -0.4];
        let chosen = closest_solution(&kinematics, &target, &current, &limits).unwrap();
        assert!(
            chosen.iter().zip(&q).all(|(a, b)| (a - b).abs() < 1e-6),
            "{chosen:?}"
        );
        let motion = MotionLimits::same(6, 1.0, 2.0, 10.0);
        let trajectory =
            move_to_pose(&kinematics, &target, &current, &limits, &motion, PERIOD).unwrap();
        assert_eq!(trajectory.points.last().unwrap().1, chosen.to_vec());
        assert_eq!(trajectory.points[0].1, current.to_vec());
        // 닿는 해가 한계 밖이면 거절한다
        let tight = [(-0.1, 0.1); 6];
        let message =
            move_to_pose(&kinematics, &target, &current, &tight, &motion, PERIOD).unwrap_err();
        assert!(message.contains("관절 한계 안에 없다"), "{message}");
    }

    /// 로봇 설정의 시험값 (관절 30°/s, 60°/s², 300°/s³, 손끝 30 mm/s, 20°/s)
    fn robot_limits() -> (MotionLimits, ToolLimits) {
        let motion = MotionLimits::same(
            6,
            30f64.to_radians(),
            60f64.to_radians(),
            300f64.to_radians(),
        );
        let tool = ToolLimits {
            speed: 0.03,
            rotation: 20f64.to_radians(),
        };
        (motion, tool)
    }

    /// 굽힌 작업 자세 (J4 200°)
    const WORKING: [f64; 6] = [0.3, -0.4, 0.9, 3.49, 0.8, -0.5];

    fn joint_path(trajectory: &Trajectory) -> Vec<[f64; 6]> {
        trajectory
            .points
            .iter()
            .map(|(_, q)| std::array::from_fn(|i| q[i]))
            .collect()
    }

    fn distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
        a.iter()
            .zip(b)
            .map(|(x, y)| (x - y).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    fn rotation_gap(a: &Pose, b: &Pose) -> f64 {
        a.rotation
            .iter()
            .flatten()
            .zip(b.rotation.iter().flatten())
            .fold(0.0_f64, |m, (x, y)| m.max((x - y).abs()))
    }

    #[test]
    fn line_moves_the_flange_straight_within_the_limits() {
        let kinematics = Settings::parse(TEST_J3).unwrap().kinematics;
        let (motion, tool) = robot_limits();
        let start = kinematics.forward(&WORKING);
        let target = kinematics.forward(&[0.2, -0.3, 0.8, 3.3, 0.9, -0.3]);
        let trajectory = move_line(
            &kinematics,
            &target,
            &WORKING,
            &design_limits(),
            &motion,
            &tool,
            PERIOD,
        )
        .unwrap();
        let path = joint_path(&trajectory);
        assert_eq!(path[0], WORKING);
        for (index, (time, _)) in trajectory.points.iter().enumerate() {
            assert!((time - index as f64 * PERIOD).abs() < 1e-9);
        }
        // 점마다 플랜지가 곧은 선 위에 있고, 방향은 그 자리의 고른 회전이며, 앞으로만 간다
        let line = PoseLine::new(&start, &target);
        let shift: Vec<f64> = (0..3)
            .map(|i| target.position[i] - start.position[i])
            .collect();
        let squared: f64 = shift.iter().map(|v| v * v).sum();
        let mut last_u = 0.0;
        for q in &path {
            let pose = kinematics.forward(q);
            let u = (0..3)
                .map(|i| (pose.position[i] - start.position[i]) * shift[i])
                .sum::<f64>()
                / squared;
            let on_line = line.at(u);
            assert!(distance(&pose.position, &on_line.position) < 1e-9, "{u}");
            assert!(rotation_gap(&pose, &on_line) < 1e-9, "{u}");
            assert!(u >= last_u - 1e-12, "{u} < {last_u}");
            last_u = u;
        }
        let end = kinematics.forward(path.last().unwrap());
        assert!(distance(&end.position, &target.position) < 1e-9);
        assert!(rotation_gap(&end, &target) < 1e-9);
        // 관절은 속도·가속·저크 한계 안이고, 손끝은 손끝 한계 안이다
        assert!(excess(&path, &motion, PERIOD) <= 1.0);
        for pair in path.windows(2) {
            let (a, b) = (kinematics.forward(&pair[0]), kinematics.forward(&pair[1]));
            let speed = distance(&a.position, &b.position) / PERIOD;
            assert!(speed <= tool.speed + 1e-9, "{speed}");
            let turn = PoseLine::new(&a, &b).angle() / PERIOD;
            assert!(turn <= tool.rotation + 1e-9, "{turn}");
        }
        // 빠르기는 손끝 속도 한계가 정한다: 46 mm를 30 mm/s로 1.5초 (16° 돌기는 20°/s로 0.8초)에 가속과 감속을
        // 더해 2.1초이고, 가운데에서는 손끝 한계 가까이 간다
        let duration = trajectory.points.last().unwrap().0;
        let shortest = line.length() / tool.speed;
        assert!(
            (shortest..shortest + 0.8).contains(&duration),
            "{duration} s, {shortest} s"
        );
        assert!(tool_peak(&kinematics, &path) > 0.95 * tool.speed);
    }

    /// 박자마다 본 손끝 속도의 최대 [m/s]
    fn tool_peak(kinematics: &Kinematics, path: &[[f64; 6]]) -> f64 {
        path.windows(2)
            .map(|pair| {
                let (a, b) = (kinematics.forward(&pair[0]), kinematics.forward(&pair[1]));
                distance(&a.position, &b.position) / PERIOD
            })
            .fold(0.0_f64, f64::max)
    }

    #[test]
    fn line_near_a_singularity_slows_only_there() {
        let kinematics = Settings::parse(TEST_J3).unwrap().kinematics;
        let (motion, tool) = robot_limits();
        // 팔꿈치가 거의 펴진 자세(J3 1.1°)에서 안으로 50 mm, 아래로 50 mm (71 mm): 시작 자리에서는 J3가 손끝보다
        // 훨씬 빨리 돌아야 해서 천천히 떠나고, 팔꿈치가 굽은 뒤에는 손끝 한계 가까이 간다
        let stretched = [0.3, -0.4, 0.02, 3.49, 0.8, -0.5];
        let mut target = kinematics.forward(&stretched);
        let [x, y, z] = target.position;
        let r = x.hypot(y);
        target.position = [x - 0.05 * x / r, y - 0.05 * y / r, z - 0.05];
        let trajectory = move_line(
            &kinematics,
            &target,
            &stretched,
            &design_limits(),
            &motion,
            &tool,
            PERIOD,
        )
        .unwrap();
        let path = joint_path(&trajectory);
        assert!(excess(&path, &motion, PERIOD) <= 1.0);
        assert!(tool_peak(&kinematics, &path) > 0.9 * tool.speed);
        // 첫 0.5초 동안은 손끝이 10 mm/s보다 느리다
        assert!(tool_peak(&kinematics, &path[..50]) < 0.01);
        // 손끝 한계로만 가면 2.4초. 시작 자리에서 늦춘 만큼만 길다 (길 전체를 늦추면 20초를 넘었다)
        let line = PoseLine::new(&kinematics.forward(&stretched), &target);
        let duration = trajectory.points.last().unwrap().0;
        let shortest = line.length() / tool.speed;
        assert!(duration < 2.5 * shortest, "{duration} s, {shortest} s");
    }

    #[test]
    fn fast_tool_limits_are_slowed_to_the_joint_limits() {
        let kinematics = Settings::parse(TEST_J3).unwrap().kinematics;
        let (motion, _) = robot_limits();
        let fast = ToolLimits {
            speed: 0.5,
            rotation: 5.0,
        };
        let target = kinematics.forward(&[0.2, -0.3, 0.8, 3.3, 0.9, -0.3]);
        let trajectory = move_line(
            &kinematics,
            &target,
            &WORKING,
            &design_limits(),
            &motion,
            &fast,
            PERIOD,
        )
        .unwrap();
        let path = joint_path(&trajectory);
        let over = excess(&path, &motion, PERIOD);
        assert!((0.9..=1.0).contains(&over), "{over}");
        let duration = trajectory.points.last().unwrap().0;
        // 손끝 한계로만 가면 0.1초 남짓인데, 관절 속도 한계 때문에 몇 배 늦다 (거절하지는 않는다)
        let line = PoseLine::new(&kinematics.forward(&WORKING), &target);
        assert!(duration > 5.0 * line.length() / fast.speed, "{duration}");
    }

    #[test]
    fn turning_in_place_keeps_the_flange_still() {
        let kinematics = Settings::parse(TEST_J3).unwrap().kinematics;
        let (motion, tool) = robot_limits();
        let start = kinematics.forward(&WORKING);
        let turned = kinematics.forward(&[0.3, -0.4, 0.9, 3.6, 0.9, -0.2]);
        let target = Pose {
            rotation: turned.rotation,
            position: start.position,
        };
        let trajectory = move_line(
            &kinematics,
            &target,
            &WORKING,
            &design_limits(),
            &motion,
            &tool,
            PERIOD,
        )
        .unwrap();
        for q in joint_path(&trajectory) {
            let pose = kinematics.forward(&q);
            assert!(distance(&pose.position, &start.position) < 1e-9);
        }
        let end = kinematics.forward(&joint_path(&trajectory).pop().unwrap());
        assert!(rotation_gap(&end, &target) < 1e-9);
    }

    #[test]
    fn lines_that_cannot_be_followed_are_refused() {
        let kinematics = Settings::parse(TEST_J3).unwrap().kinematics;
        let (motion, tool) = robot_limits();
        let limits = design_limits();
        let refuse = |target: &Pose, current: &[f64; 6], limits: &[(f64, f64); 6]| {
            move_line(&kinematics, target, current, limits, &motion, &tool, PERIOD).unwrap_err()
        };
        // 손목 특이 자세(J5 = 0)를 지난다: 가운데에서 해가 끊긴다
        let wrist_up = [0.3, -0.4, 0.9, 3.49, 0.15, -0.5];
        let wrist_down = kinematics.forward(&[0.3, -0.4, 0.9, 3.49, -0.15, -0.5]);
        let message = refuse(&wrist_down, &wrist_up, &limits);
        assert!(message.contains("해가 끊긴다"), "{message}");
        // 팔이 닿지 않는 곳으로 간다
        let mut far = kinematics.forward(&WORKING);
        far.position = [0.0, 0.0, 2.0];
        let message = refuse(&far, &WORKING, &limits);
        assert!(message.contains("닿지 않는다"), "{message}");
        // 가는 길에 J1이 한계 밖으로 나간다
        let target = kinematics.forward(&[0.2, -0.3, 0.8, 3.3, 0.9, -0.3]);
        let mut tight = limits;
        tight[0] = (0.25, 0.35);
        let message = refuse(&target, &WORKING, &tight);
        assert!(message.contains("J1가 관절 한계 밖"), "{message}");
    }

    #[test]
    fn line_to_where_the_flange_is_stays_put() {
        let kinematics = Settings::parse(TEST_J3).unwrap().kinematics;
        let (motion, tool) = robot_limits();
        let here = kinematics.forward(&WORKING);
        let trajectory = move_line(
            &kinematics,
            &here,
            &WORKING,
            &design_limits(),
            &motion,
            &tool,
            PERIOD,
        )
        .unwrap();
        assert_eq!(trajectory.points, vec![(0.0, WORKING.to_vec())]);
        // 박자 길이와 한계가 0이거나 목표가 수가 아니면 받지 않는다
        let line = |target: &Pose, tool: &ToolLimits, period: f64| {
            move_line(
                &kinematics,
                target,
                &WORKING,
                &design_limits(),
                &motion,
                tool,
                period,
            )
        };
        assert!(line(&here, &tool, 0.0).is_err());
        let still = ToolLimits {
            speed: 0.0,
            rotation: 1.0,
        };
        assert!(line(&here, &still, PERIOD).is_err());
        let mut broken = here;
        broken.position[0] = f64::NAN;
        assert!(line(&broken, &tool, PERIOD).is_err());
    }

    #[test]
    fn bad_inputs_are_refused() {
        let limits = MotionLimits::same(2, 1.0, 2.0, 10.0);
        assert!(move_joints(&[0.0], &[1.0, 0.0], &limits, PERIOD).is_err());
        assert!(move_joints(&[0.0, 0.0], &[1.0, 0.0], &limits, 0.0).is_err());
        let zero = MotionLimits::same(2, 0.0, 2.0, 10.0);
        assert!(move_joints(&[0.0, 0.0], &[1.0, 0.0], &zero, PERIOD).is_err());
        assert!(move_joints(&[0.0, f64::NAN], &[1.0, 0.0], &limits, PERIOD).is_err());
    }
}
