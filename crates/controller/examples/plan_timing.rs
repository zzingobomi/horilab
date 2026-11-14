//! 계산 시간 재기 (docs/decisions/0026-pi-runtime-deploy-records.md의 미리 재기 3): 실행 루프 한 박자(붙잡기, 관절
//! 조그, 직교 조그)와 이동 계획(관절 이동, 직선 이동)에 드는 시간을 잰다.
//!
//! 서보는 가짜 서보라 박자 시간에 가짜 버스의 몫이 들어 있다. 조그의 계산 몫은 붙잡는 박자와의 차이로 본다.
//! 자기 충돌 검사는 아직 없어서 재지 않는다. 기구학과 기준값은 레포의 로봇 설정이다.
//!
//! ```text
//! cargo run --release -p horilab-controller --example plan_timing
//! uv run tools/pi_build.py -p horilab-controller --example plan_timing
//!     (파이용: target/aarch64-unknown-linux-musl/release/examples/plan_timing)
//! ```

use std::time::Instant;

use horilab_controller::joints::{Caps, Joints, Servo};
use horilab_controller::kinematics::{Pose, PoseLine};
use horilab_controller::plan::{MotionLimits, ToolLimits, move_joints, move_line};
use horilab_controller::runloop::{Mode, RunLoop};
use horilab_controller::settings::{Calibration, Direction, Settings};
use horilab_servo::feetech::Bus;
use horilab_servo::feetech::fake::{FakePort, FakeServo, open_fake_bus};

const PERIOD: f64 = 0.01;
/// 굽힌 작업 자세 [rad] (곧게 선 영점은 특이 자세다)
const WORKING: [f64; 6] = [0.3, 0.4, 0.9, 0.6, 0.7, -0.5];
/// 관절의 소프트 한계 [rad]: 팔 설계 범위인 양쪽 160° (로봇 설정에는 아직 옛 관절 시험의 J3만 있다)
const LIMIT: f64 = 160.0 * std::f64::consts::PI / 180.0;

/// 시간 표본의 중간, 99번째 백분위, 최대 [µs]
fn summary(samples: &mut [f64]) -> String {
    samples.sort_by(f64::total_cmp);
    let at = |q: f64| samples[((samples.len() - 1) as f64 * q).round() as usize];
    format!(
        "중간 {:.0} µs, 99% {:.0} µs, 최대 {:.0} µs ({}번)",
        at(0.5),
        at(0.99),
        samples[samples.len() - 1],
        samples.len()
    )
}

/// 여섯 관절의 실행 루프 (가짜 서보, 값 적용을 한 것처럼 EEPROM을 구워 둔다).
fn run_loop(settings: &Settings) -> RunLoop<Bus<FakePort>> {
    let template = settings
        .joints
        .iter()
        .find(|joint| joint.name == "j3")
        .expect("로봇 설정에 j3가 있다")
        .clone();
    let names = ["j1", "j2", "j3", "j4", "j5", "j6"];
    let servos: Vec<Servo> = names
        .iter()
        .zip(1u8..)
        .map(|(&name, id)| {
            let mut joint = template.clone();
            joint.name = name;
            joint.id = id;
            joint.lower = -LIMIT;
            joint.upper = LIMIT;
            joint.calibration = Some(Calibration {
                pin: 0.0,
                zero_raw: 2048,
                direction: Direction::Same,
                position_offset: 0,
            });
            Servo::from_joint(&joint).expect("영점을 줬다")
        })
        .collect();
    let fakes = servos.iter().map(|servo| {
        let mut fake = FakeServo::with(servo.id, template.model_number, 2048);
        for &(register, value) in &servo.eeprom {
            fake.set(register, value);
        }
        let burned = fake.flash.len();
        fake.flash.copy_from_slice(&fake.memory[..burned]);
        fake
    });
    let joints = Joints::open(
        open_fake_bus(fakes),
        servos,
        Caps::for_loop(&settings.run_loop),
    )
    .expect("가짜 서보가 열린다");
    let mut run = RunLoop::start(joints, settings.run_loop, None).expect("실행 루프가 선다");
    run.set_kinematics(settings.kinematics.clone())
        .expect("관절 여섯");
    run
}

/// 박자를 `ticks`번 돌며 한 박자에 걸린 시간 [µs]. 박자마다 `command`를 먼저 부른다.
fn time_ticks(
    run: &mut RunLoop<Bus<FakePort>>,
    ticks: usize,
    mut command: impl FnMut(&mut RunLoop<Bus<FakePort>>, usize),
) -> Vec<f64> {
    (0..ticks)
        .map(|n| {
            command(run, n);
            let start = Instant::now();
            run.tick(PERIOD);
            start.elapsed().as_secs_f64() * 1e6
        })
        .collect()
}

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.next()
    }
}

fn main() {
    let settings = Settings::parse(include_str!("../../../data/robot.toml"))
        .expect("레포의 로봇 설정을 읽는다");
    let kinematics = &settings.kinematics;
    let motion = MotionLimits::from_settings(&settings.run_loop, 6);
    let tool = ToolLimits::from_settings(&settings.run_loop);

    // 실행 루프 한 박자
    let mut run = run_loop(&settings);
    let trajectory = move_joints(run.targets(), &WORKING, &motion, PERIOD).expect("작업 자세로");
    run.submit(trajectory).expect("궤적을 받는다");
    while run.tick(PERIOD) != &Mode::Holding {}
    let mut holding = time_ticks(&mut run, 3000, |_, _| {});
    // 3초마다 방향을 바꾼다 (관절마다 0.1 rad/s, 손끝 10 mm/s)
    let mut joint_jog = time_ticks(&mut run, 3000, |run, n| {
        let way = if (n / 300) % 2 == 0 { 0.1 } else { -0.1 };
        run.jog(&[way; 6]).expect("관절 조그를 받는다");
    });
    while run.tick(PERIOD) != &Mode::Holding {}
    let mut cartesian = time_ticks(&mut run, 3000, |run, n| {
        let way = if (n / 300) % 2 == 0 { 0.01 } else { -0.01 };
        run.jog_cartesian(&[way, 0.0, way, 0.0, 0.0, 0.0])
            .expect("직교 조그를 받는다");
    });
    println!("## 실행 루프 한 박자 (가짜 서보 여섯)");
    println!("- 붙잡기: {}", summary(&mut holding));
    println!("- 관절 조그: {}", summary(&mut joint_jog));
    println!("- 직교 조그: {}", summary(&mut cartesian));

    // 관절 이동 계획
    let mut rng = Lcg(7);
    let limits = [(-LIMIT, LIMIT); 6];
    let mut joint_moves = Vec::new();
    for _ in 0..200 {
        let goal: [f64; 6] = std::array::from_fn(|i| WORKING[i] + rng.range(-0.5, 0.5));
        let start = Instant::now();
        let planned = move_joints(&WORKING, &goal, &motion, PERIOD);
        joint_moves.push(start.elapsed().as_secs_f64() * 1e6);
        assert!(planned.is_ok());
    }
    println!("\n## 이동 계획");
    println!("- 관절 이동: {}", summary(&mut joint_moves));

    // 직선 이동 계획: 작업 자세 둘레의 무작위 직선 (30~150 mm, 손목을 조금 돌린 방향)
    let mut rng = Lcg(7);
    let (mut lines, mut durations, mut refused) = (Vec::new(), Vec::new(), 0);
    while lines.len() < 30 {
        let start: [f64; 6] = std::array::from_fn(|i| WORKING[i] + rng.range(-0.3, 0.3));
        let from = kinematics.forward(&start);
        let direction = [
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ];
        let size = direction.iter().map(|v| v * v).sum::<f64>().sqrt();
        let length = rng.range(0.03, 0.15);
        let axis = [
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ];
        let turn = rng.range(0.0, 30f64.to_radians());
        let mut turned = start;
        turned[3] += turn * axis[0].signum();
        turned[4] += turn * axis[1] * 0.5;
        turned[5] += turn * axis[2];
        let target = Pose {
            rotation: kinematics.forward(&turned).rotation,
            position: std::array::from_fn(|i| from.position[i] + direction[i] / size * length),
        };
        if (0..6).any(|i| !(limits[i].0..=limits[i].1).contains(&start[i])) {
            continue;
        }
        let clock = Instant::now();
        let planned = move_line(kinematics, &target, &start, &limits, &motion, &tool, PERIOD);
        let spent = clock.elapsed().as_secs_f64() * 1e6;
        match planned {
            Ok(trajectory) => {
                lines.push(spent);
                let line = PoseLine::new(&from, &target);
                let duration = trajectory.points.last().map_or(0.0, |(t, _)| *t);
                durations.push((line.length() * 1e3, duration));
            }
            Err(_) => refused += 1,
        }
    }
    println!(
        "- 직선 이동: {} (닿지 않거나 한계 밖이라 거절한 {refused}개는 뺐다)",
        summary(&mut lines)
    );
    let (total_mm, total_s) = durations
        .iter()
        .fold((0.0, 0.0), |(m, s), (mm, sec)| (m + mm, s + sec));
    println!("  - 직선 30개: 길이 합 {total_mm:.0} mm, 시간 합 {total_s:.1} s");
}
