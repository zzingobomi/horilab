//! 시뮬레이션 버스의 한 박자 시간 재기 (docs/decisions/0033-planning-and-simulation.md의 실시간 모드).
//!
//! 실행 루프를 시뮬레이션 서버에 붙여, 박자마다 걸린 실제 시간과 그 안의 거래 시간(동기 읽기와 물리 한 걸음,
//! 동기 쓰기)을 잰다. 서버의 실시간 모드(`--realtime`)는 한 박자가 주기(10 ms) 안에 끝나야 벽시계를 따라간다.
//! 서버는 빠른 모드로 띄운다 (실시간 모드는 기다리는 시간이 섞인다).
//!
//! 서보는 다섯이다 (실물은 그리퍼까지 일곱). 시뮬레이션에는 그리퍼가 없고, 시뮬레이션은 모든 관절을 0°에서
//! 시작하는데 지금 MJCF(옛 골격)의 J4 범위는 80°~235°라 0°를 품지 않아서 J4에는 서보를 달지 않는다.
//!
//! ```text
//! uv run python -m horilab_design.mjcf
//! uv run python -m horilab_sim
//! cargo run --release -p horilab-controller --example sim_timing [-- <서버 주소, 기본 127.0.0.1:5600>]
//! ```

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;
use std::time::{Duration, Instant};

use horilab_controller::joints::{Caps, Joints, Servo};
use horilab_controller::plan::{MotionLimits, move_joints};
use horilab_controller::runloop::{Mode, RunLoop};
use horilab_controller::settings::{Calibration, Direction, Joint, Settings};
use horilab_servo::feetech::{Error, Fault, LinkStats, Register, Reply, ServoBus, SyncRead};
use horilab_servo::sim::{SimBus, SimServo};

const PERIOD: Duration = Duration::from_millis(10);
/// 관절의 소프트 한계 [rad]: 팔 설계 범위인 양쪽 160°
const LIMIT: f64 = 160.0 * std::f64::consts::PI / 180.0;
/// 서보를 단 관절의 굽힌 작업 자세 [rad] (옛 골격 MJCF의 관절 범위 안)
const WORKING: [f64; 5] = [0.3, 0.4, 0.9, 0.7, -0.5];
/// 관절, 서보 ID, 모델. 시뮬레이션이 아는 모델만 쓴다 (J2의 ST-3120 자리에 STS3250)
const SERVOS: [(&str, u8, &str, u16); 5] = [
    ("j1", 1, "STS3215", 777),
    ("j2", 2, "STS3250", 2825),
    ("j3", 3, "STS3250", 2825),
    ("j5", 5, "STS3215", 777),
    ("j6", 6, "STS3215", 777),
];

/// 거래마다 걸린 시간 [µs]
#[derive(Default)]
struct Timings {
    sync_read: Vec<f64>,
    sync_write: Vec<f64>,
    other: Vec<f64>,
}

#[derive(Clone, Copy)]
enum Kind {
    Read,
    Write,
    Other,
}

/// 거래마다 걸린 시간을 적는 서보 버스
struct Timed<B> {
    inner: B,
    timings: Rc<RefCell<Timings>>,
}

impl<B> Timed<B> {
    fn timed<T>(&mut self, kind: Kind, call: impl FnOnce(&mut B) -> T) -> T {
        let start = Instant::now();
        let out = call(&mut self.inner);
        let spent = start.elapsed().as_secs_f64() * 1e6;
        let mut timings = self.timings.borrow_mut();
        match kind {
            Kind::Read => timings.sync_read.push(spent),
            Kind::Write => timings.sync_write.push(spent),
            Kind::Other => timings.other.push(spent),
        }
        out
    }
}

impl<B: ServoBus> ServoBus for Timed<B> {
    fn ping(&mut self, id: u8) -> Result<u16, Error> {
        self.timed(Kind::Other, |bus| bus.ping(id))
    }

    fn read(&mut self, id: u8, registers: &[Register]) -> Result<Reply, Error> {
        self.timed(Kind::Other, |bus| bus.read(id, registers))
    }

    fn sync_read(&mut self, ids: &[u8], registers: &[Register]) -> Result<SyncRead, Error> {
        self.timed(Kind::Read, |bus| bus.sync_read(ids, registers))
    }

    fn write(&mut self, id: u8, register: Register, value: i32) -> Result<Fault, Error> {
        self.timed(Kind::Other, |bus| bus.write(id, register, value))
    }

    fn sync_write(&mut self, register: Register, values: &[(u8, i32)]) -> Result<(), Error> {
        self.timed(Kind::Write, |bus| bus.sync_write(register, values))
    }

    fn write_trial(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), Error> {
        self.timed(Kind::Other, |bus| bus.write_trial(id, values))
    }

    fn dump(&mut self, id: u8) -> Result<Vec<u8>, Error> {
        self.timed(Kind::Other, |bus| bus.dump(id))
    }

    fn stats(&self) -> &BTreeMap<u8, LinkStats> {
        self.inner.stats()
    }

    fn pause(&mut self, duration: Duration) {
        self.inner.pause(duration);
    }
}

/// 시간 표본의 중간, 99번째 백분위, 최대 [ms]
fn summary(samples: &mut [f64]) -> String {
    if samples.is_empty() {
        return "없음".to_owned();
    }
    samples.sort_by(f64::total_cmp);
    let at = |q: f64| samples[((samples.len() - 1) as f64 * q).round() as usize] / 1e3;
    format!(
        "중간 {:.2} ms, 99% {:.2} ms, 최대 {:.2} ms ({}번)",
        at(0.5),
        at(0.99),
        samples[samples.len() - 1] / 1e3,
        samples.len()
    )
}

/// 시뮬레이션의 서보는 관절 각 0이 2048로 읽히게 달려 있다 (방향 +1, 오프셋 0).
fn joint(name: &'static str, id: u8, model: &'static str, model_number: u16) -> Joint {
    Joint {
        name,
        id,
        model,
        model_number,
        lower: -LIMIT,
        upper: LIMIT,
        pin: 0.0,
        calibration: Some(Calibration {
            pin: 0.0,
            zero_raw: 2048,
            direction: Direction::Same,
            position_offset: 0,
        }),
        torque_limit: None,
        eeprom: Vec::new(),
    }
}

fn main() {
    let address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "127.0.0.1:5600".to_owned());
    let settings = Settings::parse(include_str!("../../../data/robot.toml"))
        .expect("레포의 로봇 설정을 읽는다");
    let motion = MotionLimits::from_settings(&settings.run_loop, SERVOS.len());
    let dt = PERIOD.as_secs_f64();

    let sim: Vec<SimServo> = SERVOS
        .iter()
        .map(|&(joint, id, _, model)| SimServo {
            id,
            joint: joint.to_owned(),
            model,
        })
        .collect();
    let clock = Instant::now();
    let bus = SimBus::connect(&address, PERIOD, &sim)
        .unwrap_or_else(|e| panic!("{address}의 시뮬레이션 서버에 붙지 못했다: {e}"));
    let hello = clock.elapsed();
    let timings = Rc::new(RefCell::new(Timings::default()));
    let bus = Timed {
        inner: bus,
        timings: Rc::clone(&timings),
    };
    let robot: Vec<Servo> = SERVOS
        .iter()
        .map(|&(name, id, model, number)| {
            Servo::from_joint(&joint(name, id, model, number)).expect("영점을 줬다")
        })
        .collect();
    let joints = Joints::open(bus, robot, Caps::for_loop(&settings.run_loop))
        .expect("시뮬레이션 서보가 열린다");
    let mut run = RunLoop::start(joints, settings.run_loop, None).expect("실행 루프가 선다");
    // 열고 켜는 동안의 거래는 빼고 잰다
    *timings.borrow_mut() = Timings::default();

    let mut ticks: Vec<f64> = Vec::new();
    let mut tick = |run: &mut RunLoop<Timed<SimBus>>| -> Mode {
        let start = Instant::now();
        let mode = run.tick(dt).clone();
        ticks.push(start.elapsed().as_secs_f64() * 1e6);
        mode
    };
    // 붙잡기 → 작업 자세로 관절 이동 → 관절 조그(3초마다 방향을 바꾼다) → 영점으로 관절 이동
    for _ in 0..500 {
        tick(&mut run);
    }
    let mut refused = 0;
    for goal in [WORKING, [0.0; 5]] {
        let trajectory = move_joints(run.targets(), &goal, &motion, dt).expect("관절 이동");
        if run.submit(trajectory).is_err() {
            refused += 1;
        }
        for _ in 0..3000 {
            if tick(&mut run) != Mode::Moving {
                break;
            }
        }
        if goal == WORKING {
            for n in 0..1500 {
                let way = if (n / 300) % 2 == 0 { 0.1 } else { -0.1 };
                if run.jog(&[way; 5]).is_err() {
                    refused += 1;
                }
                tick(&mut run);
            }
            for _ in 0..100 {
                tick(&mut run);
            }
        }
    }

    let late = ticks
        .iter()
        .filter(|&&t| t > PERIOD.as_secs_f64() * 1e6)
        .count();
    let mut timings = timings.borrow_mut();
    println!(
        "## 시뮬레이션 버스 ({address}): 서보 다섯, 세계 만들기 {:.0} ms",
        hello.as_secs_f64() * 1e3
    );
    println!("- 한 박자 (실행 루프 전체): {}", summary(&mut ticks));
    println!(
        "  - 동기 읽기 (물리 한 걸음 포함): {}",
        summary(&mut timings.sync_read)
    );
    println!("  - 동기 쓰기: {}", summary(&mut timings.sync_write));
    println!("  - 그 밖의 거래: {}번", timings.other.len());
    println!("- 주기(10 ms)를 넘은 박자: {late}개");
    println!(
        "- 실행 루프: {:?}, 받지 않은 명령 {refused}개, 알림 {:?}",
        run.mode(),
        run.warnings()
    );
}
