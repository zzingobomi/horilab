//! 실행 루프 부하 시험 (docs/decisions/0026-pi-runtime-deploy-records.md의 2, 미리 재기 2).
//!
//! 실행 파일은 `src/bin/runloop.rs`이고, 기록 파일과 스레드를 맡는다. 여기에는 실행 루프 자체와 박자를
//! 세는 법, 도수분포, 첫 연결 때 잰 버스 시간을 두어 실물 없는 PC에서 시험한다.
//!
//! 실행 루프는 가짜 서보 7개(그리퍼 포함)를 물린 진짜 서보 버스 코드로 박자마다 동기 쓰기(목표 위치)와
//! 동기 읽기(상태 묶음)를 보낸다. 선을 기다리는 시간은 첫 연결 때 잰 한 박자 시간 가운데 하나를 골라,
//! 실물처럼 세 번(동기 쓰기, 읽기 요청, 응답)으로 나눠 잔다. 이어서 계산을 흉내 내어 정한 시간만큼
//! CPU를 쓴다.

use std::time::Duration;

use horilab_servo::feetech::fake::{FakeServo, open_fake_bus};
use horilab_servo::feetech::registers as reg;
use serde_json::{Value, json};

use crate::rt;

/// 통과 기준 (0026의 2): 부하를 건 채 깨어나는 지연이 최악 1 ms 안이고, 놓친 박자가 없다.
pub const WAKE_LIMIT_US: u64 = 1_000;

/// 첫 연결 때 hori3에서 잰 한 박자(동기 쓰기 + 동기 읽기, 서보 7개)의 중간값 [µs] (0024의 첫 연결 결과).
/// 통신 시간 기록 파일을 주지 않을 때 쓴다.
pub const DEFAULT_BUS_US: u32 = 2_750;

/// 도수분포의 상한 [µs]. 넘는 값은 넘침으로 세고 최댓값은 따로 남는다.
const HISTOGRAM_LIMIT_US: usize = 20_000;

/// "90", "90s", "1.5m", "2h"를 읽는다.
pub fn parse_duration(text: &str) -> Result<Duration, String> {
    let text = text.trim();
    let (number, unit) = match text.chars().last() {
        Some(unit) if unit.is_ascii_alphabetic() => (&text[..text.len() - unit.len_utf8()], unit),
        _ => (text, 's'),
    };
    let value: f64 = number
        .parse()
        .map_err(|_| format!("시간을 읽지 못했다: {text} (예: 90s, 30m, 2h)"))?;
    let seconds = match unit {
        's' => value,
        'm' => value * 60.0,
        'h' => value * 3600.0,
        _ => return Err(format!("시간 단위는 s, m, h다: {text}")),
    };
    if !seconds.is_finite() || seconds <= 0.0 {
        return Err(format!("시간은 0보다 커야 한다: {text}"));
    }
    Ok(Duration::from_secs_f64(seconds))
}

/// 박자 `k`(시작에서 k × 주기에 깨어나는 박자)의 일이 시작에서 `end_ns`에 끝났을 때, 다음에 돌 박자와
/// 건너뛴 박자 수. 깨어날 시각이 이미 지난 박자는 몰아서 돌지 않고 건너뛴다.
pub fn next_beat(k: u64, end_ns: u64, period_ns: u64) -> (u64, u64) {
    let next = end_ns.div_ceil(period_ns).max(k + 1);
    (next, next - (k + 1))
}

/// 박자 하나를 잰 값 [ns]. 실행 루프가 기록 줄에 넣고, 낮은 우선순위의 스레드가 모은다.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Beat {
    /// 박자 번호: 시작에서 index × 주기에 깨어나는 박자
    pub index: u64,
    /// 깨어날 시각보다 늦게 깨어난 시간
    pub wake_ns: u64,
    /// 동기 쓰기 + 동기 읽기 (버스를 기다린 시간 포함)
    pub bus_ns: u64,
    /// 계산을 흉내 낸 시간
    pub compute_ns: u64,
    /// 깨어날 시각부터 이 박자의 일이 끝날 때까지
    pub work_ns: u64,
    /// 이 박자 뒤에 건너뛴 박자 수
    pub missed: u64,
    /// 동기 읽기에 응답하지 않았거나 응답이 깨진 서보 수
    pub bus_errors: u64,
}

fn micros(ns: u64) -> u64 {
    ns / 1_000
}

/// 마이크로초 단위 도수분포.
#[derive(Debug, Clone)]
pub struct Histogram {
    counts: Vec<u64>,
    overflow: u64,
    count: u64,
    sum_us: u128,
    max_us: u64,
}

impl Default for Histogram {
    fn default() -> Self {
        Self::new(HISTOGRAM_LIMIT_US)
    }
}

impl Histogram {
    pub fn new(limit_us: usize) -> Self {
        Self {
            counts: vec![0; limit_us + 1],
            overflow: 0,
            count: 0,
            sum_us: 0,
            max_us: 0,
        }
    }

    pub fn record(&mut self, us: u64) {
        self.count += 1;
        self.sum_us += u128::from(us);
        self.max_us = self.max_us.max(us);
        match usize::try_from(us)
            .ok()
            .and_then(|i| self.counts.get_mut(i))
        {
            Some(slot) => *slot += 1,
            None => self.overflow += 1,
        }
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    pub fn max_us(&self) -> u64 {
        self.max_us
    }

    /// 칸을 세는 상한 [µs].
    pub fn limit_us(&self) -> usize {
        self.counts.len() - 1
    }

    /// `us` 칸에 든 수.
    pub fn count_at(&self, us: usize) -> u64 {
        self.counts.get(us).copied().unwrap_or(0)
    }

    /// 상한을 넘은 수.
    pub fn over_limit(&self) -> u64 {
        self.overflow
    }

    pub fn mean_us(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum_us as f64 / self.count as f64
        }
    }

    /// `p`(0~1) 분위수. 상한을 넘는 자리면 최댓값을 돌려준다.
    pub fn percentile_us(&self, p: f64) -> u64 {
        if self.count == 0 {
            return 0;
        }
        let rank = ((p * self.count as f64).ceil() as u64).clamp(1, self.count);
        let mut seen = 0;
        for (us, &n) in self.counts.iter().enumerate() {
            seen += n;
            if seen >= rank {
                return us as u64;
            }
        }
        self.max_us
    }

    pub fn summary(&self) -> Value {
        json!({
            "count": self.count,
            "mean_us": round1(self.mean_us()),
            "p50_us": self.percentile_us(0.5),
            "p99_us": self.percentile_us(0.99),
            "p999_us": self.percentile_us(0.999),
            "p9999_us": self.percentile_us(0.9999),
            "max_us": self.max_us,
            "over_limit": self.overflow,
        })
    }
}

fn round1(x: f64) -> f64 {
    (x * 10.0).round() / 10.0
}

/// 1초 동안의 박자를 모은 것. 기록의 samples.csv 한 줄이다.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Second {
    pub beats: u64,
    pub missed: u64,
    pub wake_sum_us: u64,
    pub wake_max_us: u64,
    pub work_max_us: u64,
    pub bus_max_us: u64,
    pub compute_max_us: u64,
    pub bus_errors: u64,
}

impl Second {
    pub fn add(&mut self, beat: &Beat) {
        let wake = micros(beat.wake_ns);
        self.beats += 1;
        self.missed += beat.missed;
        self.wake_sum_us += wake;
        self.wake_max_us = self.wake_max_us.max(wake);
        self.work_max_us = self.work_max_us.max(micros(beat.work_ns));
        self.bus_max_us = self.bus_max_us.max(micros(beat.bus_ns));
        self.compute_max_us = self.compute_max_us.max(micros(beat.compute_ns));
        self.bus_errors += beat.bus_errors;
    }

    pub fn wake_mean_us(&self) -> f64 {
        if self.beats == 0 {
            0.0
        } else {
            self.wake_sum_us as f64 / self.beats as f64
        }
    }
}

/// 시험 전체를 모은 것.
#[derive(Debug, Clone, Default)]
pub struct Totals {
    pub wake: Histogram,
    pub work: Histogram,
    pub bus: Histogram,
    pub compute: Histogram,
    pub beats: u64,
    pub missed: u64,
    pub bus_errors: u64,
}

impl Totals {
    pub fn add(&mut self, beat: &Beat) {
        self.beats += 1;
        self.missed += beat.missed;
        self.bus_errors += beat.bus_errors;
        self.wake.record(micros(beat.wake_ns));
        self.work.record(micros(beat.work_ns));
        self.bus.record(micros(beat.bus_ns));
        self.compute.record(micros(beat.compute_ns));
    }

    /// 통과 기준을 지켰나: 깨어나는 지연이 최악 1 ms 안이고, 놓친 박자와 버린 기록이 없다.
    pub fn passed(&self, dropped: u64) -> bool {
        self.beats > 0 && self.wake.max_us() <= WAKE_LIMIT_US && self.missed == 0 && dropped == 0
    }

    pub fn summary(&self, dropped: u64) -> Value {
        json!({
            "beats": self.beats,
            "missed": self.missed,
            "dropped_records": dropped,
            "bus_errors": self.bus_errors,
            "wake": self.wake.summary(),
            "work": self.work.summary(),
            "bus": self.bus.summary(),
            "compute": self.compute.summary(),
            "pass": {
                "wake_limit_us": WAKE_LIMIT_US,
                "wake_max_within_limit": self.wake.max_us() <= WAKE_LIMIT_US,
                "no_missed_beats": self.missed == 0,
                "no_dropped_records": dropped == 0,
                "all": self.passed(dropped),
            },
        })
    }
}

/// 버스 시간을 고르는 작은 난수. 같은 씨앗이면 같은 순서라 같은 시험을 다시 돌릴 수 있다.
#[derive(Debug, Clone)]
pub struct XorShift(u64);

impl XorShift {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

/// 한 박자(동기 쓰기 + 동기 읽기)에 버스가 쓰는 시간. 첫 연결 때 잰 값 가운데 하나를 박자마다 고른다.
#[derive(Debug, Clone, PartialEq)]
pub struct BusModel {
    samples_us: Vec<u32>,
    source: String,
}

impl BusModel {
    pub fn fixed(us: u32) -> Self {
        Self {
            samples_us: vec![us.max(1)],
            source: format!("고정 {us} µs"),
        }
    }

    /// 첫 연결 통신 시간 기록(`records/servo/…_timing-…/samples.csv`)에서 한 박자(op가 tick)인 줄을 쓴다.
    /// 오류가 난 줄은 뺀다.
    pub fn from_timing_csv(text: &str, source: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let header: Vec<&str> = lines.next().ok_or("빈 파일이다")?.split(',').collect();
        let column = |name: &str| {
            header
                .iter()
                .position(|h| h.trim() == name)
                .ok_or(format!("{name} 열이 없다"))
        };
        let (op, duration, error) = (column("op")?, column("duration_ms")?, column("error")?);
        let mut samples_us = Vec::new();
        for (row, line) in lines.enumerate() {
            let fields: Vec<&str> = line.split(',').collect();
            if fields.get(op).map(|f| f.trim()) != Some("tick") {
                continue;
            }
            if fields.get(error).is_some_and(|e| !e.trim().is_empty()) {
                continue;
            }
            let ms: f64 = fields
                .get(duration)
                .and_then(|f| f.trim().parse().ok())
                .ok_or(format!("{}번째 줄의 duration_ms를 읽지 못했다", row + 2))?;
            samples_us.push((ms * 1000.0).round().max(1.0) as u32);
        }
        if samples_us.is_empty() {
            return Err("한 박자(op가 tick)인 줄이 없다".to_owned());
        }
        Ok(Self {
            samples_us,
            source: source.to_owned(),
        })
    }

    pub fn pick_us(&self, rng: &mut XorShift) -> u32 {
        let i = rng.next_u64() % self.samples_us.len() as u64;
        self.samples_us[i as usize]
    }

    pub fn describe(&self) -> Value {
        let mut sorted = self.samples_us.clone();
        sorted.sort_unstable();
        let at =
            |p: f64| sorted[((p * sorted.len() as f64).ceil() as usize).clamp(1, sorted.len()) - 1];
        json!({
            "source": self.source,
            "count": sorted.len(),
            "median_us": at(0.5),
            "p99_us": at(0.99),
            "max_us": sorted[sorted.len() - 1],
        })
    }
}

/// 한 박자의 버스 시간을 세 번의 기다림으로 나눈 자리 (0~1, 누적): 동기 쓰기를 보내는 동안, 동기 읽기
/// 요청을 보내는 동안, 응답을 받는 동안. 선 위의 바이트 수에 비례해 나눈다.
pub fn bus_phases(servos: usize, write_size: usize, read_size: usize) -> [f64; 3] {
    let write = 8 + servos * (1 + write_size); // FF FF FE 길이 명령 주소 크기 (ID 값)… 체크섬
    let request = 8 + servos; // FF FF FE 길이 명령 주소 크기 ID… 체크섬
    let replies = servos * (6 + read_size); // 서보마다 FF FF ID 길이 오류 값… 체크섬
    let total = (write + request + replies) as f64;
    [write as f64 / total, (write + request) as f64 / total, 1.0]
}

/// 값이 가장 큰 박자 몇 개 (큰 순). 늦은 박자를 시각과 맞춰 무엇 때문이었는지 보려고 남긴다.
#[derive(Debug, Clone)]
pub struct Worst {
    keep: usize,
    beats: Vec<(u64, Beat)>,
}

impl Worst {
    pub fn new(keep: usize) -> Self {
        Self {
            keep,
            beats: Vec::with_capacity(keep + 1),
        }
    }

    pub fn offer(&mut self, value: u64, beat: &Beat) {
        if self.beats.len() == self.keep && self.beats.last().is_none_or(|(v, _)| value <= *v) {
            return;
        }
        let at = self.beats.partition_point(|(v, _)| *v >= value);
        self.beats.insert(at, (value, *beat));
        self.beats.truncate(self.keep);
    }

    pub fn values(&self) -> Vec<u64> {
        self.beats.iter().map(|(v, _)| *v).collect()
    }

    pub fn to_json(&self, period_ns: u64) -> Value {
        self.beats
            .iter()
            .map(|(_, beat)| {
                json!({
                    "index": beat.index,
                    // 0.01초 단위. 정수를 100으로 나눠야 42.59가 42.589999…로 적히지 않는다
                    "at_s": (beat.index as f64 * period_ns as f64 / 1e7).round() / 100.0,
                    "wake_us": micros(beat.wake_ns),
                    "work_us": micros(beat.work_ns),
                    "bus_us": micros(beat.bus_ns),
                    "compute_us": micros(beat.compute_ns),
                    "missed": beat.missed,
                })
            })
            .collect()
    }
}

/// 팔의 서보: 관절 1~6과 그리퍼(11). 모델 번호는 응답 길이에 영향이 없어 STS3215(777)로 두고, 팔꿈치만
/// STS3250(2825)이다.
pub const ARM_SERVOS: [(u8, u16); 7] = [
    (1, 777),
    (2, 777),
    (3, 2825),
    (4, 777),
    (5, 777),
    (6, 777),
    (11, 777),
];

/// 실행 루프의 설정.
#[derive(Debug, Clone)]
pub struct LoopSettings {
    pub period_ns: u64,
    /// 돌릴 박자 수 (건너뛴 박자도 센다)
    pub beats: u64,
    /// 박자마다 계산을 흉내 내는 시간
    pub compute_ns: u64,
    pub bus: BusModel,
    pub seed: u64,
    /// SCHED_FIFO 우선순위. 없으면 보통 스레드다
    pub fifo: Option<i32>,
    /// 실행 루프를 묶을 코어
    pub cpu: Option<usize>,
}

/// 실행 루프 스레드에 실제로 걸린 설정 (기록에 남긴다).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoopThread {
    pub policy: String,
    pub priority: i32,
    pub affinity: Vec<usize>,
}

/// 실행 루프가 끝난 모양.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoopEnd {
    /// 돈 박자 수
    pub ran: u64,
    /// 멈춤 신호(Ctrl+C, SIGTERM)로 일찍 끝났다
    pub stopped: bool,
}

/// 실행 루프를 부른 스레드에서 돌린다. 스레드를 준비한 뒤 `ready`를 한 번 부르고, 박자마다 `on_beat`를
/// 부른다. `on_beat`는 기다리지 않아야 한다 (기록 줄에 넣기만 한다).
pub fn run_loop(
    settings: &LoopSettings,
    ready: impl FnOnce(&LoopThread),
    mut on_beat: impl FnMut(Beat),
) -> Result<LoopEnd, String> {
    if let Some(cpu) = settings.cpu {
        rt::pin_to_cpus(&[cpu]).map_err(|e| format!("코어 {cpu}에 묶지 못했다: {e}"))?;
    }
    if let Some(priority) = settings.fifo {
        rt::set_fifo(priority)
            .map_err(|e| format!("SCHED_FIFO {priority}를 걸지 못했다 (root로 돌린다): {e}"))?;
    }
    prefault_stack();

    let ids = ARM_SERVOS.map(|(id, _)| id);
    let mut bus = open_fake_bus(
        ARM_SERVOS
            .iter()
            .map(|&(id, model)| FakeServo::with(id, model, 2048)),
    );
    let torque_on = ids.map(|id| (id, reg::TORQUE_ON));
    bus.sync_write(reg::TORQUE_ENABLE, &torque_on)
        .map_err(|e| format!("가짜 서보 토크를 켜지 못했다: {e}"))?;
    let state_size = reg::STATE
        .iter()
        .map(|r| usize::from(r.address) + usize::from(r.size))
        .max()
        .unwrap_or(0)
        - reg::STATE
            .iter()
            .map(|r| usize::from(r.address))
            .min()
            .unwrap_or(0);
    let phases = bus_phases(ids.len(), usize::from(reg::GOAL_POSITION.size), state_size);
    // 처음 몇 번은 할당과 캐시 때문에 느리니 박자 전에 돌려 둔다.
    for _ in 0..100 {
        let _ = bus.sync_write(reg::GOAL_POSITION, &ids.map(|id| (id, 2048)));
        let _ = bus.sync_read(&ids, reg::STATE);
    }
    let (policy, priority) = rt::current_sched();
    ready(&LoopThread {
        policy,
        priority,
        affinity: rt::affinity(),
    });

    let period = settings.period_ns;
    let start = rt::now_ns() + 50_000_000;
    let mut rng = XorShift::new(settings.seed);
    let mut x = 0.5;
    let (mut k, mut ran) = (0, 0);
    while k < settings.beats {
        if rt::stop_requested() {
            return Ok(LoopEnd { ran, stopped: true });
        }
        let target = start + k * period;
        rt::sleep_until_ns(target);
        let woke = rt::now_ns();

        let wire_ns = u64::from(settings.bus.pick_us(&mut rng)) * 1_000;
        let wait_until = |share: f64| woke + (wire_ns as f64 * share) as u64;
        let goal = 2048 + (k % 200) as i32 - 100; // 조금씩 움직이는 목표
        let mut bus_errors = 0;
        if bus
            .sync_write(reg::GOAL_POSITION, &ids.map(|id| (id, goal)))
            .is_err()
        {
            bus_errors += ids.len() as u64;
        }
        rt::sleep_until_ns(wait_until(phases[0]));
        match bus.sync_read(&ids, reg::STATE) {
            Ok(replies) => bus_errors += replies.values().filter(|r| r.is_err()).count() as u64,
            Err(_) => bus_errors += ids.len() as u64,
        }
        rt::sleep_until_ns(wait_until(phases[1]));
        rt::sleep_until_ns(wait_until(phases[2]));
        let bus_end = rt::now_ns();

        x = compute(bus_end + settings.compute_ns, x);
        let end = rt::now_ns();
        let (next, missed) = next_beat(k, end - start, period);
        on_beat(Beat {
            index: k,
            wake_ns: woke.saturating_sub(target),
            bus_ns: bus_end - woke,
            compute_ns: end - bus_end,
            work_ns: end - target,
            missed,
            bus_errors,
        });
        ran += 1;
        k = next;
    }
    std::hint::black_box(x);
    Ok(LoopEnd {
        ran,
        stopped: false,
    })
}

/// 계산을 흉내 낸다: `until_ns`까지 부동소수 계산으로 CPU를 쓴다.
fn compute(until_ns: u64, mut x: f64) -> f64 {
    loop {
        for _ in 0..32 {
            x = (x * 1.000_001 + 0.25).sin();
        }
        if rt::now_ns() >= until_ns {
            return x;
        }
    }
}

/// 실행 루프 스레드의 스택을 미리 한 번 써서, 박자 도중에 처음 쓰는 페이지 때문에 늦지 않게 한다.
/// 스레드 스택은 이보다 커야 한다.
pub const PREFAULT_STACK: usize = 256 * 1024;

fn prefault_stack() {
    let mut pages = [0u8; PREFAULT_STACK];
    for i in (0..pages.len()).step_by(4096) {
        pages[i] = 1;
    }
    std::hint::black_box(&pages);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn durations_read_seconds_minutes_and_hours() {
        assert_eq!(parse_duration("90").unwrap(), Duration::from_secs(90));
        assert_eq!(parse_duration("90s").unwrap(), Duration::from_secs(90));
        assert_eq!(parse_duration("1.5m").unwrap(), Duration::from_secs(90));
        assert_eq!(parse_duration("2h").unwrap(), Duration::from_secs(7200));
        assert!(parse_duration("0s").is_err());
        assert!(parse_duration("5x").is_err());
        assert!(parse_duration("m").is_err());
    }

    #[test]
    fn late_beats_are_skipped_not_bunched() {
        let p = 10_000_000; // 100 Hz
        // 박자 3의 일이 그 주기 안에 끝났다
        assert_eq!(next_beat(3, 3 * p + 4_000_000, p), (4, 0));
        // 다음 박자가 깨어날 바로 그 시각에 끝났으면 늦지 않았다
        assert_eq!(next_beat(3, 4 * p, p), (4, 0));
        // 다음 박자를 넘겨서 끝났다: 박자 4를 건너뛴다
        assert_eq!(next_beat(3, 4 * p + 1, p), (5, 1));
        assert_eq!(next_beat(3, 7 * p + 500, p), (8, 4));
    }

    #[test]
    fn histogram_percentiles_and_overflow() {
        let mut h = Histogram::new(1_000);
        for us in 1..=100 {
            h.record(us);
        }
        assert_eq!(h.percentile_us(0.5), 50);
        assert_eq!(h.percentile_us(0.99), 99);
        assert_eq!(h.max_us(), 100);
        assert!((h.mean_us() - 50.5).abs() < 1e-9);
        h.record(5_000); // 상한을 넘으면 넘침으로 세고 최댓값은 남는다
        assert_eq!(h.max_us(), 5_000);
        assert_eq!(h.percentile_us(1.0), 5_000);
        assert_eq!(h.summary()["over_limit"], 1);
    }

    #[test]
    fn bus_model_reads_ticks_from_the_first_connection_record() {
        let csv = "phase,op,servo_id,t_s,duration_ms,error\n\
                   as_is,short_read,1,0.0,0.4161,\n\
                   as_is,tick,,0.01,2.752,\n\
                   as_is,tick,,0.02,2.850,\n\
                   as_is,tick,,0.03,9.900,응답 없음\n\
                   as_is,tick,,0.04,2.683,\n";
        let model = BusModel::from_timing_csv(csv, "samples.csv").unwrap();
        let described = model.describe();
        assert_eq!(described["count"], 3); // 오류 난 줄은 뺀다
        assert_eq!(described["median_us"], 2752);
        assert_eq!(described["max_us"], 2850);
        let mut rng = XorShift::new(7);
        for _ in 0..100 {
            assert!([2683, 2752, 2850].contains(&model.pick_us(&mut rng)));
        }
        assert!(BusModel::from_timing_csv("op,duration_ms\ntick,1.0\n", "x").is_err()); // error 열이 없다
        assert!(BusModel::from_timing_csv("op,duration_ms,error\nsync_read,1.0,\n", "x").is_err());
    }

    #[test]
    fn random_order_repeats_with_the_same_seed() {
        let (mut a, mut b) = (XorShift::new(42), XorShift::new(42));
        let first: Vec<u64> = (0..5).map(|_| a.next_u64()).collect();
        let second: Vec<u64> = (0..5).map(|_| b.next_u64()).collect();
        assert_eq!(first, second);
        assert_ne!(XorShift::new(0).next_u64(), 0); // 씨앗 0도 멈추지 않는다
    }

    #[test]
    fn totals_pass_only_when_every_criterion_holds() {
        let beat = Beat {
            wake_ns: 80_000,
            work_ns: 3_000_000,
            bus_ns: 2_800_000,
            ..Beat::default()
        };
        let mut totals = Totals::default();
        totals.add(&beat);
        assert!(totals.passed(0));
        assert!(!totals.passed(1)); // 기록을 버렸다
        let mut late = totals.clone();
        late.add(&Beat {
            wake_ns: 1_200_000,
            ..beat
        });
        assert!(!late.passed(0)); // 깨어나기가 1 ms를 넘었다
        let mut missed = totals.clone();
        missed.add(&Beat { missed: 1, ..beat });
        assert!(!missed.passed(0));
        assert_eq!(missed.summary(0)["pass"]["no_missed_beats"], false);
        assert!(!Totals::default().passed(0)); // 박자가 없으면 통과가 아니다
    }

    #[test]
    fn bus_time_splits_like_the_bytes_on_the_wire() {
        // 서보 7개, 목표 위치 2바이트, 상태 묶음 15바이트: 쓰기 29, 읽기 요청 15, 응답 147바이트
        let [write, request, all] = bus_phases(7, 2, 15);
        assert!((write - 29.0 / 191.0).abs() < 1e-12);
        assert!((request - 44.0 / 191.0).abs() < 1e-12);
        assert_eq!(all, 1.0);
    }

    #[test]
    fn worst_keeps_the_largest_values_in_order() {
        let mut worst = Worst::new(3);
        for (i, value) in [5, 1, 9, 3, 7, 9].into_iter().enumerate() {
            worst.offer(
                value,
                &Beat {
                    index: i as u64,
                    ..Beat::default()
                },
            );
        }
        assert_eq!(worst.values(), vec![9, 9, 7]);
        let listed = worst.to_json(10_000_000);
        assert_eq!(listed[0]["index"], 2); // 같은 값이면 먼저 온 박자가 앞이다
        assert_eq!(listed[0]["at_s"], 0.02);
    }

    #[test]
    fn the_loop_runs_every_beat_on_the_fake_bus() {
        let settings = LoopSettings {
            period_ns: 20_000_000,
            beats: 15,
            compute_ns: 100_000,
            bus: BusModel::fixed(1_000),
            seed: 1,
            fifo: None,
            cpu: None,
        };
        let mut thread = None;
        let mut beats = Vec::new();
        let end = run_loop(
            &settings,
            |t| thread = Some(t.clone()),
            |beat| beats.push(beat),
        )
        .unwrap();
        assert!(!end.stopped);
        assert_eq!(end.ran, beats.len() as u64);
        assert!(!thread.unwrap().affinity.is_empty());
        assert_eq!(beats[0].index, 0);
        assert!(beats.windows(2).all(|w| w[1].index > w[0].index));
        // 박자 번호와 건너뛴 수를 더하면 모든 박자를 덮는다
        let last = beats.last().unwrap();
        assert!(last.index + 1 + last.missed >= settings.beats);
        for beat in &beats {
            assert_eq!(beat.bus_errors, 0);
            assert!(beat.bus_ns >= 1_000_000); // 고른 버스 시간만큼은 기다린다
            assert!(beat.compute_ns >= 100_000);
            assert!(beat.work_ns >= beat.bus_ns + beat.compute_ns);
        }
    }

    #[test]
    fn a_second_keeps_maximums_and_the_mean_wake() {
        let mut second = Second::default();
        second.add(&Beat {
            wake_ns: 100_000,
            bus_ns: 2_700_000,
            ..Beat::default()
        });
        second.add(&Beat {
            wake_ns: 300_000,
            bus_ns: 2_900_000,
            missed: 2,
            ..Beat::default()
        });
        assert_eq!(second.beats, 2);
        assert_eq!(second.missed, 2);
        assert_eq!(second.wake_max_us, 300);
        assert_eq!(second.bus_max_us, 2_900);
        assert!((second.wake_mean_us() - 200.0).abs() < 1e-9);
    }
}
