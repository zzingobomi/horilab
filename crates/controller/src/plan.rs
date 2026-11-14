//! 경로 계획의 관절 이동 (docs/decisions/0033-planning-and-simulation.md의 만드는 순서 3): 저크를 제한하고 모든
//! 관절이 같이 끝나는 궤적을 rsruckig로 만든다. 궤적은 실행 루프의 박자마다 점 하나이고, 실행 루프가 실행 전에
//! 한계를 다시 본다 (0009).

use std::f64::consts::TAU;

use rsruckig::prelude::*;

use crate::kinematics::{Kinematics, Pose};
use crate::runloop::Trajectory;
use crate::settings::LoopSettings;

/// 궤적이 이보다 길면 만들지 않는다 (박자 수). 100 Hz로 10분이다.
const MAX_POINTS: usize = 60_000;

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
