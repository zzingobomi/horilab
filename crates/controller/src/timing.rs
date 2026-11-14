//! 경로 시간 매기기 (docs/decisions/0033-planning-and-simulation.md의 만드는 순서 3): 정해진 길(경로 위 위치 u가
//! 0에서 1까지) 위를 언제 어디까지 갈지 정한다. 길의 곳마다 빠르기를 달리해, 관절 한계 때문에 늦춰야 하는 곳만
//! 늦춘다.
//!
//! 1. 역(길 위의 점)마다 빠르기 상한(손끝 한계, 관절 속도)과 가속 제약 |q' ü + q'' u̇²| ≤ a를 둔다. q'와 q''는 u에
//!    대한 관절 각의 도함수이고, 제약은 x = u̇²와 ü에 대해 선형이다.
//! 2. 2차 풀이 (TOPP-RA와 같은 도달 가능 집합): 뒤에서 앞으로 끝에서 멈출 수 있는 x의 상한을 훑고, 앞에서 뒤로
//!    그 안에서 가장 빨리 간다.
//! 3. 저크는 u(t)를 이동 평균(FIR)으로 걸러 제한한다. 폭은 2차 궤적의 관절 가속이 그 폭 동안 바뀌는 양이 저크
//!    한계 × 폭 안에 드는 가장 작은 값이다.
//! 4. 거르면 빠르기가 그 폭 동안 평균되므로, 빠르기 상한을 미리 "그 빠르기로 폭만큼 가는 거리 안의 가장 낮은
//!    상한"으로 깎는다. 빠르기가 낮은 곳은 조금만 깎인다.
//!
//! 거른 뒤의 한계는 부르는 쪽(직선 이동)이 박자마다의 관절 각으로 다시 본다.

use crate::plan::MotionLimits;

/// 거르기 전 u(t)를 뽑는 간격 [s]
pub(crate) const FINE: f64 = 0.001;
/// 2차 풀이에 쓰는 가속 한계의 몫 (거른 뒤 위치가 조금 어긋나 생기는 차이의 여유)
const ACCEL_SHARE: f64 = 0.95;
/// 거르개 폭을 고를 때 쓰는 저크 한계의 몫 (q''와 q'''가 만드는 저크의 여유)
const JERK_SHARE: f64 = 0.85;

/// 길 위 u의 시각 표본: `FINE`마다 하나, 0에서 시작해 1에서 끝나고 앞뒤로 멈춰 있다.
///
/// - `dq`, `ddq`: 역마다 u에 대한 관절 각의 1·2차 도함수 (역 사이 간격은 `du`)
/// - `tool_rate`: 손끝 한계가 허락하는 u의 빠르기 [1/s]
pub(crate) fn time_path(
    dq: &[[f64; 6]],
    ddq: &[[f64; 6]],
    du: f64,
    tool_rate: f64,
    motion: &MotionLimits,
    period: f64,
) -> Result<Vec<f64>, String> {
    let a: [f64; 6] = std::array::from_fn(|i| ACCEL_SHARE * motion.acceleration[i]);
    let speed_cap: Vec<f64> = dq
        .iter()
        .zip(ddq)
        .map(|(dq, ddq)| speed_limit(dq, ddq, tool_rate, &motion.velocity, &a))
        .collect();
    // 거르개 폭과 깎은 상한을 맞춰 간다: 폭으로 상한을 깎아 2차 풀이를 한 뒤, 필요한 폭이 더 넓으면 다시 한다
    let widest = (0..6)
        .map(|i| 2.0 * motion.acceleration[i] / motion.jerk[i])
        .fold(0.0, f64::max);
    let mut window = widest / 2.0;
    let mut samples = Vec::new();
    for _ in 0..4 {
        let cap: Vec<f64> = erode(&speed_cap, window, du)
            .iter()
            .map(|v| v * v)
            .collect();
        let x = second_order(dq, ddq, &cap, &a, du);
        let (us, accelerations) = sample(&x, dq, ddq, du)?;
        let needed = needed_window(&accelerations, &motion.jerk, 2.0 * period, widest);
        samples = us;
        let enough = needed <= window * (1.0 + 1e-9);
        window = needed;
        if enough {
            break;
        }
    }
    Ok(moving_average(&samples, window))
}

/// 역 하나의 빠르기 상한 [1/s]: 손끝 한계, 관절 속도 한계, 가속 제약이 비지 않는 가장 큰 빠르기.
fn speed_limit(
    dq: &[f64; 6],
    ddq: &[f64; 6],
    tool_rate: f64,
    velocity: &[f64],
    a: &[f64; 6],
) -> f64 {
    let mut x = tool_rate * tool_rate;
    for i in 0..6 {
        if dq[i].abs() > 1e-12 {
            x = x.min((velocity[i] / dq[i]).powi(2));
        }
    }
    let feasible = |x: f64| {
        let (low, high) = accel_range(dq, ddq, x, a);
        low <= high
    };
    largest(x, feasible).sqrt()
}

/// `ok`가 참인 [0, top]의 가장 큰 값 (`ok`는 작은 값에서 참이고 커지면 거짓이 된다).
fn largest(top: f64, ok: impl Fn(f64) -> bool) -> f64 {
    if ok(top) {
        return top;
    }
    let (mut good, mut bad) = (0.0, top);
    for _ in 0..50 {
        let middle = 0.5 * (good + bad);
        if ok(middle) {
            good = middle;
        } else {
            bad = middle;
        }
    }
    good
}

/// 가속 제약 |q' ü + q'' x| ≤ a에서 x = u̇²일 때 ü가 될 수 있는 범위 [low, high]. 비면 low > high다.
fn accel_range(dq: &[f64; 6], ddq: &[f64; 6], x: f64, a: &[f64; 6]) -> (f64, f64) {
    let (mut low, mut high) = (f64::NEG_INFINITY, f64::INFINITY);
    for i in 0..6 {
        let centripetal = ddq[i] * x;
        if dq[i].abs() < 1e-12 {
            if centripetal.abs() > a[i] {
                return (1.0, -1.0);
            }
            continue;
        }
        let (up, down) = ((a[i] - centripetal) / dq[i], (-a[i] - centripetal) / dq[i]);
        low = low.max(up.min(down));
        high = high.min(up.max(down));
    }
    (low, high)
}

/// 구간 최솟값 표 (희소 표): 만들 때 n log n, 물을 때 한 번.
struct MinTable(Vec<Vec<f64>>);

impl MinTable {
    fn new(values: &[f64]) -> Self {
        let mut levels = vec![values.to_vec()];
        let mut span = 1;
        while 2 * span <= values.len() {
            let previous = &levels[levels.len() - 1];
            let next = (0..=values.len() - 2 * span)
                .map(|i| previous[i].min(previous[i + span]))
                .collect();
            levels.push(next);
            span *= 2;
        }
        Self(levels)
    }

    /// [first, last]의 최솟값 (양 끝 포함)
    fn min(&self, first: usize, last: usize) -> f64 {
        let level = (usize::BITS - 1 - (last - first + 1).leading_zeros()) as usize;
        let values = &self.0[level];
        values[first].min(values[last + 1 - (1 << level)])
    }
}

/// 빠르기 상한을 깎는다: 역 j의 상한 v는 v로 `window` 동안 가는 거리 안의 가장 낮은 상한 이하여야 한다.
fn erode(speed: &[f64], window: f64, du: f64) -> Vec<f64> {
    let table = MinTable::new(speed);
    let last = speed.len() - 1;
    (0..speed.len())
        .map(|j| {
            largest(speed[j], |v| {
                let reach = (v * window / du).ceil() as usize;
                v <= table.min(j.saturating_sub(reach), (j + reach).min(last))
            })
        })
        .collect()
}

/// 2차 풀이: 역마다 x = u̇². 뒤에서 앞으로 끝에서 멈출 수 있는 상한을 훑고, 앞에서 뒤로 가장 빨리 간다.
fn second_order(dq: &[[f64; 6]], ddq: &[[f64; 6]], cap: &[f64], a: &[f64; 6], du: f64) -> Vec<f64> {
    let n = cap.len() - 1;
    let mut reach = vec![0.0; n + 1];
    for j in (0..n).rev() {
        let next = reach[j + 1];
        reach[j] = largest(cap[j], |x| {
            let (low, high) = accel_range(&dq[j], &ddq[j], x, a);
            low <= high && x + 2.0 * du * low <= next
        });
    }
    let mut x = vec![0.0; n + 1];
    for j in 0..n {
        let (_, high) = accel_range(&dq[j], &ddq[j], x[j], a);
        x[j + 1] = reach[j + 1].min(x[j] + 2.0 * du * high).max(0.0);
    }
    x
}

/// 2차 궤적을 `FINE`마다 뽑은 u와 관절 가속 (역 사이에서는 ü가 일정하다).
fn sample(
    x: &[f64],
    dq: &[[f64; 6]],
    ddq: &[[f64; 6]],
    du: f64,
) -> Result<(Vec<f64>, Vec<[f64; 6]>), String> {
    let n = x.len() - 1;
    let speed: Vec<f64> = x.iter().map(|v| v.sqrt()).collect();
    let mut times = vec![0.0; n + 1];
    for j in 0..n {
        let sum = speed[j] + speed[j + 1];
        if sum <= 0.0 {
            return Err(format!(
                "직선의 {:.0}% 자리에서 멈춰 버린다",
                100.0 * j as f64 * du
            ));
        }
        times[j + 1] = times[j] + 2.0 * du / sum;
    }
    let total = times[n];
    let count = (total / FINE).ceil() as usize + 1;
    let (mut us, mut accelerations) = (Vec::with_capacity(count), Vec::with_capacity(count));
    let mut segment = 0;
    for m in 0..count {
        let t = (m as f64 * FINE).min(total);
        while segment < n - 1 && times[segment + 1] < t {
            segment += 1;
        }
        let tau = t - times[segment];
        let ud = (x[segment + 1] - x[segment]) / (2.0 * du);
        let v = speed[segment] + ud * tau;
        let u = segment as f64 * du + speed[segment] * tau + 0.5 * ud * tau * tau;
        us.push(u.clamp(0.0, 1.0));
        accelerations.push(std::array::from_fn(|i| {
            dq[segment][i] * ud + ddq[segment][i] * v * v
        }));
    }
    Ok((us, accelerations))
}

/// 관절 가속이 폭 동안 바뀌는 양이 저크 한계의 몫 × 폭 안에 드는 가장 작은 폭 [s] (앞뒤로는 멈춰 있다).
fn needed_window(accelerations: &[[f64; 6]], jerk: &[f64], narrowest: f64, widest: f64) -> f64 {
    let at = |k: isize| -> [f64; 6] {
        usize::try_from(k)
            .ok()
            .and_then(|k| accelerations.get(k).copied())
            .unwrap_or([0.0; 6])
    };
    let fits = |m: usize| {
        let width = m as f64 * FINE;
        let shift = m as isize;
        (0..accelerations.len() as isize + shift).all(|k| {
            let (now, before) = (at(k), at(k - shift));
            (0..6).all(|i| (now[i] - before[i]).abs() <= JERK_SHARE * jerk[i] * width)
        })
    };
    let mut m = (narrowest / FINE).round().max(1.0) as usize;
    let top = (widest / FINE).round() as usize;
    while m < top && !fits(m) {
        m = ((m as f64 * 1.1).ceil() as usize).max(m + 1);
    }
    m.min(top) as f64 * FINE
}

/// 폭 `window` [s]의 이동 평균. 앞은 0, 뒤는 1로 이어 붙여 멈춘 채 시작하고 멈춘 채 끝난다.
fn moving_average(samples: &[f64], window: f64) -> Vec<f64> {
    let width = ((window / FINE).round() as usize).max(1);
    let length = samples.len() + width - 1;
    let value = |m: usize| samples.get(m).copied().unwrap_or(1.0);
    let mut sum = 0.0;
    (0..length)
        .map(|m| {
            sum += value(m);
            if m >= width {
                sum -= value(m - width);
            }
            sum / width as f64
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 차분으로 본 최대 (앞뒤로는 멈춰 있다)
    fn peaks(samples: &[f64], step: f64) -> [f64; 3] {
        let mut values: Vec<f64> = std::iter::repeat_n(0.0, 3)
            .chain(samples.iter().copied())
            .chain(std::iter::repeat_n(1.0, 3))
            .collect();
        let mut out = [0.0; 3];
        for peak in &mut out {
            values = values.windows(2).map(|p| (p[1] - p[0]) / step).collect();
            *peak = values.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        }
        out
    }

    #[test]
    fn straight_joint_path_is_a_smoothed_trapezoid() {
        // 관절 하나가 u를 따라 1 rad 곧게 간다 (q' = 1, q'' = 0)
        let n = 1000;
        let mut dq = vec![[0.0; 6]; n + 1];
        for row in &mut dq {
            row[0] = 1.0;
        }
        let ddq = vec![[0.0; 6]; n + 1];
        let motion = MotionLimits::same(6, 1.0, 2.0, 10.0);
        let us = time_path(&dq, &ddq, 1.0 / n as f64, 100.0, &motion, 0.01).unwrap();
        assert_eq!(us[0], 0.0);
        assert!((us[us.len() - 1] - 1.0).abs() < 1e-12);
        assert!(us.windows(2).all(|p| p[1] >= p[0] - 1e-12));
        // 사다리꼴(1 rad를 1 rad/s, 가속 1.9 rad/s²로 1.53초)에 거르개 폭(가속 1.9를 저크 8.5 안에서 바꾸는 0.22초쯤)을 더한다
        let duration = (us.len() - 1) as f64 * FINE;
        assert!((1.70..1.80).contains(&duration), "{duration}");
        let [velocity, acceleration, jerk] = peaks(&us, FINE);
        assert!(velocity <= 1.0 + 1e-6, "{velocity}");
        assert!(acceleration <= 2.0, "{acceleration}");
        assert!(jerk <= 10.0, "{jerk}");
    }

    #[test]
    fn erosion_spreads_a_dip_only_as_far_as_the_window_carries() {
        // 빠르기 1인 길에 빠르기 0.1인 점 하나 (역 사이 0.01, 폭 0.1초면 빠르기 v로 10v 역을 간다)
        let mut speed = vec![1.0; 101];
        speed[50] = 0.1;
        let eroded = erode(&speed, 0.1, 0.01);
        assert_eq!(eroded[50], 0.1);
        // 다섯 역 떨어진 곳은 폭 동안 그 점까지 가지 않는 빠르기다 (10v가 5보다 작게, v는 0.4 바로 아래)
        assert!((0.39..=0.4).contains(&eroded[55]), "{}", eroded[55]);
        assert!((0.39..=0.4).contains(&eroded[45]), "{}", eroded[45]);
        // 열한 역 넘게 떨어진 곳은 그대로다
        assert_eq!(eroded[62], 1.0);
        assert_eq!(eroded[38], 1.0);
    }

    #[test]
    fn window_grows_with_the_acceleration_change() {
        // 관절 하나의 가속이 0에서 2로 뛴다: 저크 10의 85% 안에 들려면 폭이 2 / 8.5 = 0.235초 넘게 필요하다
        let mut accelerations = vec![[0.0; 6]; 500];
        for row in accelerations.iter_mut().skip(100) {
            row[0] = 2.0;
        }
        let window = needed_window(&accelerations, &[10.0; 6], 0.02, 0.4);
        assert!((0.235..0.26).contains(&window), "{window}");
    }
}
