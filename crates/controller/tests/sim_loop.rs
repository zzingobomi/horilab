//! 실행 루프를 시뮬레이션 서버에 붙여 돌린다 (docs/decisions/0033의 만드는 순서 1).
//!
//! Python 시뮬레이션 서버(`uv run python -m horilab_sim --port 0`)를 띄워 쓴다. 관절 층과 실행 루프는
//! 실물과 같은 코드이고, 서보 버스만 시뮬레이션이다.

use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

use horilab_controller::joints::{BENCH_CAPS, Joints, Servo};
use horilab_controller::plan::{MotionLimits, move_joints};
use horilab_controller::runloop::{Mode, RunLoop, Trajectory};
use horilab_controller::settings::{Calibration, Direction, Joint, LoopSettings, RAD_PER_STEP};
use horilab_servo::sim::{SimBus, SimServo};

const PERIOD: Duration = Duration::from_millis(10);

/// 띄운 시뮬레이션 서버. 시험이 끝나면 끈다.
struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// 가상환경의 Python으로 서버를 바로 띄운다. `uv run`을 거치면 시험이 끝나 uv를 꺼도 Python 서버가 남아
/// (Windows는 손자 프로세스를 같이 끄지 않는다) 출력 통로를 붙잡는다. 가상환경은 `uv sync`로 만든다.
fn start_server() -> (Server, String) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let python = if cfg!(windows) {
        root.join(".venv/Scripts/python.exe")
    } else {
        root.join(".venv/bin/python")
    };
    // 시뮬레이션은 설계를 불러오지 않고 설계가 쓴 MJCF만 읽는다 (0007)
    let model = std::env::temp_dir().join(format!("horilab-sim-{}.xml", std::process::id()));
    let written = Command::new(&python)
        .args(["-m", "horilab_design.mjcf", "--out"])
        .arg(&model)
        .current_dir(&root)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .expect("설계 MJCF를 쓰는 명령을 띄우지 못했다");
    assert!(written.success(), "설계 MJCF를 쓰지 못했다");
    let mut child = Command::new(&python)
        .args(["-m", "horilab_sim", "--port", "0", "--model"])
        .arg(&model)
        .arg("--servos")
        .arg(root.join("params/servos.toml"))
        .current_dir(&root)
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap_or_else(|e| {
            panic!(
                "{}로 시뮬레이션 서버를 띄우지 못했다: {e}",
                python.display()
            )
        });
    let stdout = child.stdout.take().expect("서버의 출력");
    let mut line = String::new();
    BufReader::new(stdout)
        .read_line(&mut line)
        .expect("서버가 준비됐다는 줄");
    let port = line
        .trim()
        .strip_prefix("listening ")
        .unwrap_or_else(|| panic!("서버의 첫 줄이 다르다: {line}"));
    (Server(child), format!("127.0.0.1:{port}"))
}

/// 시뮬레이션의 서보는 관절 각 0이 2048로 읽히게 달려 있다 (방향 +1, 오프셋 0).
fn joint(name: &'static str, id: u8) -> Joint {
    Joint {
        name,
        id,
        model: "STS3250",
        model_number: 2825,
        lower: -1.0,
        upper: 1.5,
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

fn settings(sag_gain: f64) -> LoopSettings {
    LoopSettings {
        max_speed: 1.0,
        start_tolerance: 0.01,
        stop_time: 0.2,
        temperature_warn: 50.0,
        temperature_rest: 55.0,
        voltage_warn: 11.0,
        sag_gain,
        sag_max: 0.05,
    }
}

/// J2를 0.6 rad 기울이고 3초 버틴 뒤, J2가 읽은 관절 각과 부하.
fn tilt_and_hold(address: &str, sag_gain: f64) -> (f64, i32) {
    let servos = [("j2", 2), ("j3", 3)];
    let sim: Vec<SimServo> = servos
        .iter()
        .map(|&(joint, id)| SimServo {
            id,
            joint: joint.to_owned(),
            model: 2825,
        })
        .collect();
    let bus = SimBus::connect(address, PERIOD, &sim).unwrap();
    let robot: Vec<Servo> = servos
        .iter()
        .map(|&(name, id)| Servo::from_joint(&joint(name, id)).unwrap())
        .collect();
    let joints = Joints::open(bus, robot, BENCH_CAPS).unwrap();
    let mut run = RunLoop::start(joints, settings(sag_gain), None).unwrap();
    run.submit(Trajectory {
        points: vec![(0.0, vec![0.0, 0.0]), (1.0, vec![0.6, 0.0])],
    })
    .unwrap();
    for _ in 0..400 {
        run.tick(PERIOD.as_secs_f64());
    }
    assert_eq!(run.mode(), &Mode::Holding);
    let (_, j2) = run.states()[0];
    (f64::from(j2.position - 2048) * RAD_PER_STEP, j2.load)
}

#[test]
fn run_loop_moves_the_simulated_arm_and_sag_correction_closes_the_gap() {
    let (_server, address) = start_server();
    // 위치 서보는 팔의 무게만큼 처진 채 버틴다
    let (angle, load) = tilt_and_hold(&address, 0.0);
    let sag = (0.6 - angle).abs() / RAD_PER_STEP;
    assert!((1.0..20.0).contains(&sag), "처짐 {sag}칸");
    assert!(load.abs() > 10, "부하 {load}");
    // 처짐 보정을 켜면 읽는 위치가 목표에 닿는다
    let (angle, _) = tilt_and_hold(&address, 20.0);
    let sag = (0.6 - angle).abs() / RAD_PER_STEP;
    assert!(sag <= 1.0, "보정한 뒤 처짐 {sag}칸");
}

#[test]
fn planned_joint_move_runs_on_the_simulated_arm() {
    let (_server, address) = start_server();
    let servos = [("j2", 2), ("j3", 3)];
    let sim: Vec<SimServo> = servos
        .iter()
        .map(|&(joint, id)| SimServo {
            id,
            joint: joint.to_owned(),
            model: 2825,
        })
        .collect();
    let bus = SimBus::connect(&address, PERIOD, &sim).unwrap();
    let robot: Vec<Servo> = servos
        .iter()
        .map(|&(name, id)| Servo::from_joint(&joint(name, id)).unwrap())
        .collect();
    let joints = Joints::open(bus, robot, BENCH_CAPS).unwrap();
    let mut run = RunLoop::start(joints, settings(0.0), None).unwrap();
    let dt = PERIOD.as_secs_f64();
    // 저크를 제한한 관절 이동을 만들어 실행 루프에 넣는다
    let goal = [0.5, -0.3];
    let limits = MotionLimits::same(2, 0.8, 2.0, 10.0);
    let trajectory = move_joints(run.targets(), &goal, &limits, dt).unwrap();
    let duration = trajectory.points.last().unwrap().0;
    run.submit(trajectory).unwrap();
    let mut ticks = 1;
    while run.tick(dt) != &Mode::Holding {
        ticks += 1;
        assert!(ticks < 2000, "궤적이 끝나지 않았다");
    }
    assert!(
        (f64::from(ticks) * dt - duration).abs() < 0.05,
        "{ticks}박자, 궤적 {duration} s"
    );
    // 자리를 잡은 뒤 시뮬레이션 팔이 목표에 있다 (팔의 무게로 몇 칸 처질 수 있다)
    for _ in 0..100 {
        run.tick(dt);
    }
    for (index, (_, state)) in run.states().iter().enumerate() {
        let angle = f64::from(state.position - 2048) * RAD_PER_STEP;
        assert!((angle - goal[index]).abs() < 0.02, "{index}: {angle}");
    }
}
