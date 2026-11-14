//! 로봇 컨트롤러 (docs/decisions/0023-robot-controller.md, 0024-servo-bus.md).
//!
//! 로봇 설정 확인, 포트 목록, 값 적용(0024 만드는 순서 2~3), 맨 서보 확인(만드는 순서 5), 영점 재기와 시험
//! 순서 실행(0034)이 있다. 레포 루트에서 돌린다:
//!
//!     cargo run -p horilab-controller -- settings
//!     cargo run -p horilab-controller -- apply --port COM5            (바꿀 칸만 보여 준다)
//!     cargo run -p horilab-controller -- apply --port COM5 --yes      (쓴다)
//!     cargo run -p horilab-controller -- check --port COM5 --id 20    (할 일을 보여 주고 상태를 읽는다)
//!     cargo run -p horilab-controller -- check --port COM5 --id 20 --yes
//!     cargo run -p horilab-controller -- check-state --port COM5 --id 20
//!     cargo run -p horilab-controller -- zero --port COM5 --joint j2 [--yes]
//!     cargo run -p horilab-controller -- run --script <파일> --port COM5 [--yes]
//!     cargo run -p horilab-controller -- run --script <파일> --sim 127.0.0.1:5600 --yes

use std::collections::BTreeSet;
use std::fs;
use std::io::IsTerminal;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Duration;

use chrono::Local;
use horilab_controller::check::{
    self, BENCH_RANGE, Check, Clock, Console, SPARE_IDS, STEPS, Terminal, WallClock,
};
use horilab_controller::joints::{BENCH_CAPS, Caps, Joints, Servo, Snapshot};
use horilab_controller::prepare::{self, Change};
use horilab_controller::record::{self, Log};
use horilab_controller::runloop::{Mode, RunLoop};
use horilab_controller::runner::{Runner, Script, Status, Step};
use horilab_controller::settings::{Direction, Joint, Settings};
use horilab_controller::zero;
use horilab_servo::feetech::registers::BAUD_RATES;
use horilab_servo::feetech::{Bus, Error, SerialLink, ServoBus};
use horilab_servo::sim::{SimBus, SimServo};
use serde_json::{Value, json};

const USAGE: &str = "\
쓰는 법: horilab-controller <명령> [옵션]

명령:
  settings            로봇 설정을 읽어 검사하고 보여 준다
  ports               이 컴퓨터의 시리얼 포트를 보여 준다
  apply               값 적용: 서보의 EEPROM을 로봇 설정과 견주어 다른 칸을 보여 주고,
                      --yes가 있으면 쓴다. 토크가 꺼져 있어야 하고, 서보는 움직이지 않는다
  check               맨 서보 확인: 로봇에 달지 않은 서보 하나의 동작을 단계마다 본다.
                      --yes가 없으면 할 일을 보여 주고 서보 상태를 한 번 읽는다 (움직이지 않는다).
                      --yes가 있으면 서보가 움직인다. --only로 단계를 고르면 그 단계만 한다
  check-state         맨 서보의 상태만 읽는다 (끊김 확인). 서보는 움직이지 않는다.
                      --situation이 없으면 상황을 묻는다
  zero                영점 재기: 관절 하나(--joint)의 영점 핀 자리를 읽어 위치 오프셋을 굽는다.
                      --yes가 없으면 지금 상태와 할 일만 보여 준다. --yes가 있으면 방향을 재고(로봇
                      설정에 방향이 없으면) 핀 자리를 읽어 굽는다. 토크가 꺼져 있어야 하고 움직이지 않는다
  run                 시험 순서(--script)를 실행 루프로 돌린다. --yes가 없으면 순서를 검사해 보여 주고
                      서보 상태만 읽는다. --yes가 있으면 서보가 움직인다. 움직이는 동안 Enter로 멈춘다

옵션:
  --settings <파일>   로봇 설정 파일. 기본 data/robot.toml (레포 루트에서 돌릴 때)
  --port <포트>       서보 어댑터 포트 (예: COM5, /dev/ttyACM0)
  --joint <이름>      이 관절만 (여러 번 줄 수 있다). 기본은 로봇 설정의 모든 관절
  --id <ID>           맨 서보의 ID (check, check-state)
  --yes               보여 준 대로 서보에 쓰거나 움직인다
  --only <단계,단계>  맨 서보 확인에서 이 단계만 한다 (예: state,overload_hold,voltage)
  --situation <1~4>   끊김 확인의 상황 (check-state): 1 프로그램을 끈 뒤, 2 USB를 뺐다 꽂은 뒤,
                      3 + 플러그를 뺀 채, 4 + 플러그를 다시 꽂은 뒤
  --records <폴더>    기록 폴더. 기본 records (그 아래 servo/, run/에 쓴다)
  --direction <1|-1>  영점을 잴 때 방향을 재지 않고 이 값을 쓴다 (zero)
  --script <파일>     시험 순서 (run). 쓰는 법은 crates/controller/src/runner.rs
  --sim <주소>        서보 대신 시뮬레이션 서버에 붙는다 (run, 예: 127.0.0.1:5600)
";

/// 서보 응답 하나를 더 기다리는 여유. 서보 준비는 박자가 없어 넉넉히 둔다.
const LATENCY: Duration = Duration::from_millis(20);
/// 실행 루프가 박자마다 응답을 기다리는 여유
const LOOP_LATENCY: Duration = Duration::from_millis(5);
/// 실행 루프의 주기 (100 Hz, 0032의 후보)
const LOOP_PERIOD: Duration = Duration::from_millis(10);

#[derive(Debug, PartialEq)]
enum Command {
    Settings,
    Ports,
    Apply,
    Check,
    CheckState,
    Zero,
    Run,
}

#[derive(Debug)]
struct Options {
    command: Command,
    settings: PathBuf,
    port: Option<String>,
    joints: Vec<String>,
    id: Option<u8>,
    yes: bool,
    only: Vec<String>,
    situation: Option<usize>,
    records: PathBuf,
    direction: Option<Direction>,
    script: Option<PathBuf>,
    sim: Option<String>,
}

fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut command = None;
    let mut options = Options {
        command: Command::Settings,
        settings: PathBuf::from("data/robot.toml"),
        port: None,
        joints: Vec::new(),
        id: None,
        yes: false,
        only: Vec::new(),
        situation: None,
        records: PathBuf::from("records"),
        direction: None,
        script: None,
        sim: None,
    };
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--settings" => {
                options.settings = PathBuf::from(args.next().ok_or("--settings 뒤에 파일이 없다")?)
            }
            "--port" => options.port = Some(args.next().ok_or("--port 뒤에 포트가 없다")?),
            "--joint" => options
                .joints
                .push(args.next().ok_or("--joint 뒤에 이름이 없다")?),
            "--id" => {
                let text = args.next().ok_or("--id 뒤에 ID가 없다")?;
                options.id = Some(
                    text.parse()
                        .map_err(|_| format!("ID는 0~252의 수다: {text}"))?,
                );
            }
            "--yes" => options.yes = true,
            "--only" => {
                let text = args.next().ok_or("--only 뒤에 단계 이름이 없다")?;
                for name in text.split(',').filter(|name| !name.is_empty()) {
                    if !STEPS.iter().any(|(step, _)| *step == name) {
                        let names: Vec<&str> = STEPS.iter().map(|(step, _)| *step).collect();
                        return Err(format!(
                            "모르는 단계다: {name} (단계: {})",
                            names.join(", ")
                        ));
                    }
                    options.only.push(name.to_owned());
                }
            }
            "--situation" => {
                let text = args.next().ok_or("--situation 뒤에 번호가 없다")?;
                let number = text
                    .parse()
                    .ok()
                    .filter(|n| check::situation_name(*n).is_some());
                options.situation =
                    Some(number.ok_or_else(|| format!("상황은 1~4의 번호다: {text}"))?);
            }
            "--records" => {
                options.records = PathBuf::from(args.next().ok_or("--records 뒤에 폴더가 없다")?)
            }
            "--direction" => {
                let text = args.next().ok_or("--direction 뒤에 1이나 -1이 없다")?;
                options.direction = Some(match text.as_str() {
                    "1" => Direction::Same,
                    "-1" => Direction::Reversed,
                    _ => return Err(format!("방향은 1이나 -1이다: {text}")),
                });
            }
            "--script" => {
                options.script = Some(PathBuf::from(
                    args.next().ok_or("--script 뒤에 파일이 없다")?,
                ))
            }
            "--sim" => options.sim = Some(args.next().ok_or("--sim 뒤에 주소가 없다")?),
            "-h" | "--help" => return Err(String::new()),
            name if command.is_none() && !name.starts_with('-') => {
                command = Some(match name {
                    "settings" => Command::Settings,
                    "ports" => Command::Ports,
                    "apply" => Command::Apply,
                    "check" => Command::Check,
                    "check-state" => Command::CheckState,
                    "zero" => Command::Zero,
                    "run" => Command::Run,
                    other => return Err(format!("모르는 명령이다: {other}")),
                })
            }
            other => return Err(format!("모르는 옵션이다: {other}")),
        }
    }
    options.command = command.ok_or("명령이 없다")?;
    let needs_port = matches!(
        options.command,
        Command::Apply | Command::Check | Command::CheckState | Command::Zero
    );
    if needs_port && options.port.is_none() {
        return Err(
            "이 명령에는 --port가 있어야 한다 (포트 목록: horilab-controller ports)".to_owned(),
        );
    }
    let needs_id = matches!(options.command, Command::Check | Command::CheckState);
    if needs_id && options.id.is_none() {
        return Err("이 명령에는 맨 서보의 --id가 있어야 한다 (예비 서보는 20)".to_owned());
    }
    if options.command == Command::Zero && options.joints.len() != 1 {
        return Err("영점은 관절 하나씩 잰다: --joint를 하나 준다".to_owned());
    }
    if options.command == Command::Run {
        if options.script.is_none() {
            return Err("run에는 --script가 있어야 한다".to_owned());
        }
        if options.port.is_some() == options.sim.is_some() {
            return Err("run은 --port(서보)나 --sim(시뮬레이션) 가운데 하나에 붙는다".to_owned());
        }
    }
    Ok(options)
}

fn main() -> ExitCode {
    let options = match parse_args(std::env::args().skip(1)) {
        Ok(options) => options,
        Err(error) => {
            if !error.is_empty() {
                eprintln!("{error}\n");
            }
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    let result = match options.command {
        Command::Settings => show_settings(&options),
        Command::Ports => show_ports(),
        Command::Apply => apply(&options),
        Command::Check => run_check(&options),
        Command::CheckState => check_state(&options),
        Command::Zero => measure_zero(&options),
        Command::Run => run_script(&options),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("오류: {error}");
            ExitCode::FAILURE
        }
    }
}

fn show_settings(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    println!("로봇 설정: {}", options.settings.display());
    print!("{}", settings.summary());
    Ok(())
}

fn show_ports() -> Result<(), String> {
    let names = horilab_servo::port_names().map_err(|e| format!("포트를 읽지 못했다: {e}"))?;
    if names.is_empty() {
        println!("시리얼 포트가 없다. 서보 어댑터의 USB를 꽂았는지 본다");
    }
    for name in names {
        println!("{name}");
    }
    Ok(())
}

/// 고른 관절. 고르지 않았으면 로봇 설정의 모든 관절.
fn selected<'a>(settings: &'a Settings, names: &[String]) -> Result<Vec<&'a Joint>, String> {
    if names.is_empty() {
        return Ok(settings.joints.iter().collect());
    }
    names
        .iter()
        .map(|name| {
            settings
                .joint(name)
                .ok_or_else(|| format!("로봇 설정에 없는 관절이다: {name}"))
        })
        .collect()
}

fn changes_json(changes: &[Change]) -> Value {
    let changes: serde_json::Map<String, Value> = changes
        .iter()
        .map(|change| {
            (
                change.register.name.to_owned(),
                json!([change.current, change.target]),
            )
        })
        .collect();
    Value::Object(changes)
}

/// 값 적용 (0024): 서보마다 다른 칸을 보여 주고, --yes면 쓴 뒤 다시 읽어 확인한다.
/// 쓰기 전 덤프를 먼저 기록에 남기고 쓴다. 중간에 실패해도 되돌릴 자리가 남는다.
fn apply(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    let joints = selected(&settings, &options.joints)?;
    let port = options.port.as_deref().unwrap_or_default();
    let baud = BAUD_RATES[0].1;
    let mut bus = Bus::open(port, baud, LATENCY).map_err(|e| e.to_string())?;

    let mut found = Vec::new();
    for joint in joints {
        let before = prepare::inspect(&mut bus, joint)?;
        let changes = prepare::plan(&joint.eeprom, &before)?;
        if changes.is_empty() {
            println!(
                "{} (ID {}, {}): 로봇 설정과 같다",
                joint.name, joint.id, joint.model
            );
        } else {
            println!(
                "{} (ID {}, {}): 바꿀 칸 {}개",
                joint.name,
                joint.id,
                joint.model,
                changes.len()
            );
            for change in &changes {
                println!("  {}", change.describe());
            }
        }
        found.push((joint, before, changes));
    }
    let total: usize = found.iter().map(|(_, _, changes)| changes.len()).sum();
    if total == 0 {
        println!("쓸 것이 없다");
        return Ok(());
    }
    if !options.yes {
        println!("쓰려면 토크를 끈 채로 --yes를 붙여 다시 돌린다 (서보는 움직이지 않는다)");
        return Ok(());
    }

    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "servo", "apply", &started)?;
    let meta_path = folder.join("meta.json");
    let robot_toml = fs::read_to_string(&options.settings)
        .map_err(|e| format!("{}을 읽지 못했다: {e}", options.settings.display()))?;
    let mut meta = json!({
        "kind": "servo-apply",
        "started": record::time_text(&started),
        "run": record::run_info(port, baud),
        "robot_toml": robot_toml,
        "joints": {},
    });
    for (joint, before, changes) in &found {
        meta["joints"][joint.name] = json!({
            "id": joint.id,
            "model": joint.model,
            "before": record::dump_json(before),
            "changes": changes_json(changes),
        });
    }
    record::write_json(&meta_path, &meta)?;

    let mut result = Ok(());
    for (joint, _, changes) in &found {
        if changes.is_empty() {
            continue;
        }
        match prepare::write(&mut bus, joint, changes) {
            Ok(after) => {
                meta["joints"][joint.name]["after"] = record::dump_json(&after);
                println!(
                    "{}: {}칸을 쓰고 다시 읽어 확인했다",
                    joint.name,
                    changes.len()
                );
            }
            Err(error) => {
                meta["joints"][joint.name]["error"] = json!(error);
                result = Err(error);
                break;
            }
        }
    }
    meta["ended"] = json!(record::time_text(&Local::now()));
    meta["end"] = json!(if result.is_ok() { "normal" } else { "error" });
    record::write_json(&meta_path, &meta)?;
    println!("기록: {}", folder.display());
    result
}

/// 맨 서보 하나를 연다: 응답하는 모델을 읽어 맨 서보로 관절 층을 만든다. 서보가 응답하지 않으면 없다.
/// 시리얼 어댑터에 물린 서보의 관절 층.
type SerialJoints = Joints<Bus<SerialLink>>;

fn open_bench(options: &Options, id: u8) -> Result<Option<(SerialJoints, u16)>, String> {
    let port = options.port.as_deref().unwrap_or_default();
    let mut bus = Bus::open(port, BAUD_RATES[0].1, LATENCY).map_err(|e| e.to_string())?;
    let model = match bus.ping(id) {
        Ok(model) => model,
        Err(Error::NoReply { .. }) => return Ok(None),
        Err(error) => return Err(error.to_string()),
    };
    let servo = Servo::bench(id, model, BENCH_RANGE.0, BENCH_RANGE.1);
    Ok(Some((Joints::open(bus, vec![servo], BENCH_CAPS)?, model)))
}

/// 맨 서보가 응답하지 않을 때 볼 것. 같은 서보를 Python 첫 연결 명령이 찾으면 선과 전원은 맞다.
fn no_reply(id: u8) -> String {
    format!(
        "서보 {id}가 응답하지 않는다. + 플러그를 꽂았는지, 서보 선과 USB, 포트 이름(ports)을 본다. 다 맞으면 \
         `uv run python -m horilab_servo scan`으로 같은 서보가 찾아지는지 본다 (찾아지면 선과 전원은 맞고 \
         컨트롤러 쪽 문제다)"
    )
}

fn servo_json(id: u8, model: u16) -> Value {
    json!({ "id": id, "model": model, "model_name": prepare::model_name(model) })
}

/// 맨 서보 확인 ([`check`]). --yes가 없으면 할 일을 보여 주고 서보 상태를 한 번 읽는다 (움직이지 않는다).
fn run_check(options: &Options) -> Result<(), String> {
    let id = options.id.ok_or("--id가 없다")?;
    if !SPARE_IDS.contains(&id) {
        return Err(format!(
            "맨 서보 확인은 예비 서보 ID({}~{})로만 한다. 관절에 단 서보를 크게 돌리지 않으려는 것이다",
            SPARE_IDS.start(),
            SPARE_IDS.end()
        ));
    }
    println!(
        "맨 서보 확인: 서보 {id} 하나만 어댑터에 꽂아 책상에 두고, 혼에는 아무것도 달지 않는다."
    );
    for (index, (name, text)) in STEPS.iter().enumerate() {
        let chosen = options.only.is_empty() || options.only.iter().any(|only| only == name);
        let mark = if chosen {
            ""
        } else {
            " (이번에는 하지 않는다)"
        };
        println!("  {}. {text}{mark}", index + 1);
    }
    println!(
        "토크 한도 {} (0.1 %), 속도 {}, 가속 {} (100 step/s²)로 움직인다. 서보 설정은 잠근 채 잠깐 바꾸고 \
         단계마다 되돌린다. 중간에 끊으면 서보 전원을 껐다 켠다 (잠근 채 바꾼 값이 사라진다).",
        BENCH_CAPS.torque_limit, BENCH_CAPS.speed, BENCH_CAPS.acceleration
    );
    if !options.yes {
        // 움직이기 전에 서보를 한 번 읽어 본다. 컨트롤러가 이 서보와 처음 말하는 자리다
        let (mut joints, model) = open_bench(options, id)?.ok_or_else(|| no_reply(id))?;
        let state = joints.snapshot(id)?;
        println!(
            "서보 {id} ({})가 응답한다: {}",
            prepare::model_name(model),
            check::describe(&state)
        );
        println!("--yes를 붙이면 서보가 움직인다. 준비되면 --yes를 붙여 다시 돌린다");
        return Ok(());
    }
    let (joints, model) = open_bench(options, id)?.ok_or_else(|| no_reply(id))?;
    let port = options.port.as_deref().unwrap_or_default();
    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "servo", "check", &started)?;
    let meta_path = folder.join("meta.json");
    let mut meta = json!({
        "kind": "servo-check",
        "started": record::time_text(&started),
        "run": record::run_info(port, BAUD_RATES[0].1),
        "servo": servo_json(id, model),
        "caps": {
            "torque_limit": BENCH_CAPS.torque_limit,
            "speed": BENCH_CAPS.speed,
            "acceleration": BENCH_CAPS.acceleration,
        },
        "range": [BENCH_RANGE.0, BENCH_RANGE.1],
        "only": options.only,
        "results": {},
    });
    record::write_json(&meta_path, &meta)?;
    let log = Log::create(&folder, &Snapshot::columns())?;
    let mut run = Check::new(joints, id, Terminal::default(), WallClock::default(), log);
    run.only = options.only.clone();
    let result = run.run(&mut |results| {
        meta["results"] = Value::Object(results.clone());
        let _ = record::write_json(&meta_path, &meta);
    });
    meta["results"] = Value::Object(run.results.clone());
    meta["ended"] = json!(record::time_text(&Local::now()));
    meta["end"] = json!(if result.is_ok() { "normal" } else { "stopped" });
    record::write_json(&meta_path, &meta)?;
    println!("기록: {}", folder.display());
    result
}

/// 맨 서보의 상태만 읽는다 (끊김 확인). 상황은 --situation으로 받거나 묻고, 앞선 맨 서보 확인과 견준다.
fn check_state(options: &Options) -> Result<(), String> {
    let id = options.id.ok_or("--id가 없다")?;
    let mut console = Terminal::default();
    let situation = match options.situation {
        Some(number) => check::situation_name(number),
        None => check::ask_situation(&mut console),
    };
    let port = options.port.as_deref().unwrap_or_default();
    let started = Local::now();
    let mut meta = json!({
        "kind": "servo-check-state",
        "started": record::time_text(&started),
        "run": record::run_info(port, BAUD_RATES[0].1),
        "situation": situation,
    });
    match open_bench(options, id)? {
        Some((mut joints, model)) => {
            if situation == Some("power_out") {
                println!("+ 플러그를 뺐는데 서보가 응답한다: USB 전기로 서보가 살아 있다");
            }
            let previous = last_check_results(&options.records, id);
            let report =
                check::state_report(&mut joints, id, &mut console, previous.as_ref(), situation)?;
            meta["servo"] = servo_json(id, model);
            meta["responds"] = json!(true);
            meta["report"] = report;
        }
        None => {
            println!("서보 {id}가 응답하지 않는다");
            if situation == Some("power_out") {
                println!("+ 플러그를 뺐으니 맞다: USB 전기만으로는 서보가 살아 있지 않다");
            }
            meta["servo"] = json!({ "id": id });
            meta["responds"] = json!(false);
        }
    }
    let folder = record::new_record_dir(&options.records, "servo", "check-state", &started)?;
    record::write_json(&folder.join("meta.json"), &meta)?;
    println!("기록: {}", folder.display());
    Ok(())
}

/// 이 서보의 가장 최근 맨 서보 확인 기록에서, 잠금 단계까지 마친 것의 결과를 찾는다.
fn last_check_results(records: &Path, id: u8) -> Option<Value> {
    let mut folders: Vec<PathBuf> = fs::read_dir(records.join("servo"))
        .ok()?
        .filter_map(|entry| entry.ok().map(|entry| entry.path()))
        .collect();
    folders.sort(); // 폴더 이름이 시각으로 시작한다
    folders.iter().rev().find_map(|folder| {
        let text = fs::read_to_string(folder.join("meta.json")).ok()?;
        let meta: Value = serde_json::from_str(&text).ok()?;
        let same = meta["kind"] == "servo-check" && meta["servo"]["id"] == id;
        let results = &meta["results"];
        (same && results["lock"].get("p_before").is_some()).then(|| results.clone())
    })
}

/// 영점 재기 ([`zero`], 0024의 영점, 0034). --yes가 없으면 지금 상태와 할 일만 보여 준다.
fn measure_zero(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    let joint = selected(&settings, &options.joints)?[0];
    let port = options.port.as_deref().unwrap_or_default();
    let baud = BAUD_RATES[0].1;
    let mut bus = Bus::open(port, baud, LATENCY).map_err(|e| e.to_string())?;
    let before = prepare::inspect(&mut bus, joint)?;
    let (torque_off, offset_now) = zero::state_in(&before)?;
    let reading = zero::read_position(&mut bus, joint.id, zero::READS)?;
    let known = options
        .direction
        .or(joint.calibration.map(|calibration| calibration.direction));
    println!(
        "{} (ID {}, {}): 지금 서보 값 {reading}, 위치 오프셋 {offset_now}, 토크 {}",
        joint.name,
        joint.id,
        joint.model,
        if torque_off { "꺼짐" } else { "켜짐" }
    );
    println!(
        "영점 핀 자리의 관절 각은 {:.1}°다 (설계 값). 핀 구멍의 틈은 + 쪽(관절 각이 느는 쪽)으로 붙인다.",
        joint.pin.to_degrees()
    );
    match known {
        Some(direction) => println!("방향 {:+} (재지 않는다)", direction.number()),
        None => println!("방향은 핀을 꽂기 전에 관절을 + 쪽으로 돌려 잰다"),
    }
    if !torque_off {
        return Err(
            "토크가 켜져 있다. 팔을 받친 뒤 서보 전원을 껐다 켜면 토크가 꺼진다 (영점은 토크를 끈 채 잰다)"
                .to_owned(),
        );
    }
    if !options.yes {
        println!("--yes를 붙이면 순서대로 안내하고 위치 오프셋을 굽는다 (서보는 움직이지 않는다)");
        return Ok(());
    }
    if !std::io::stdin().is_terminal() {
        return Err(
            "영점은 사람이 핀을 꽂으며 Enter를 누르는 일이라 터미널 창에서 돌린다".to_owned(),
        );
    }

    let mut console = Terminal::default();
    let direction = match known {
        Some(direction) => direction,
        None => {
            console.ask("영점 핀을 빼고, 관절을 손으로 돌릴 수 있게 되면 Enter: ");
            let start = zero::read_position(&mut bus, joint.id, zero::READS)?;
            console.ask("관절을 + 쪽(관절 각이 느는 쪽)으로 20°쯤 돌린 채 Enter: ");
            let end = zero::read_position(&mut bus, joint.id, zero::READS)?;
            let direction = zero::direction_from(start, end)?;
            println!("서보 값 {start} → {end}: 방향 {:+}", direction.number());
            direction
        }
    };
    console.ask("영점 핀을 꽂고, 핀 구멍의 틈을 + 쪽으로 밀어 붙인 채 Enter: ");
    let reading = zero::read_position(&mut bus, joint.id, zero::READS)?;
    let plan = zero::plan(
        joint,
        settings.boundary_clearance,
        direction,
        reading,
        offset_now,
    )?;
    println!(
        "핀 자리 {reading} (오프셋 {offset_now}): 오프셋 {}을 구우면 {}로 읽힌다",
        plan.offset_new, plan.target
    );

    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "servo", "zero", &started)?;
    let meta_path = folder.join("meta.json");
    let mut meta = json!({
        "kind": "servo-zero",
        "started": record::time_text(&started),
        "run": record::run_info(port, baud),
        "joint": {
            "name": joint.name,
            "id": joint.id,
            "model": joint.model,
            "pin_deg": joint.pin.to_degrees(),
        },
        "direction": direction.number(),
        "direction_measured": known.is_none(),
        "plan": {
            "reading": plan.reading,
            "offset_now": plan.offset_now,
            "encoder": plan.encoder,
            "target": plan.target,
            "offset_new": plan.offset_new,
        },
        "before": record::dump_json(&before),
    });
    record::write_json(&meta_path, &meta)?;
    let burned = zero::burn_offset(&mut bus, joint, plan.offset_new).and_then(|after| {
        meta["after"] = record::dump_json(&after);
        bus.pause(Duration::from_millis(20));
        let zero_raw = zero::read_position(&mut bus, joint.id, zero::READS)?;
        meta["zero_raw"] = json!(zero_raw);
        zero::check_zero(&plan, zero_raw).map(|()| zero_raw)
    });
    meta["ended"] = json!(record::time_text(&Local::now()));
    match &burned {
        Ok(zero_raw) => {
            let lines = zero::settings_lines(*zero_raw, direction, plan.offset_new);
            meta["settings_lines"] = json!(lines);
            meta["end"] = json!("normal");
            println!(
                "로봇 설정(data/robot.toml)의 [joints.{}]에 적는다:\n{lines}그다음 apply로 각도 한계를 굽는다 (영점에서 계산한다)",
                joint.name
            );
        }
        Err(error) => {
            meta["error"] = json!(error);
            meta["end"] = json!("error");
        }
    }
    record::write_json(&meta_path, &meta)?;
    println!("기록: {}", folder.display());
    burned.map(|_| ())
}

/// 시험 순서를 실행 루프로 돌린다 ([`horilab_controller::runner`], 0034).
fn run_script(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    let path = options.script.as_ref().ok_or("--script가 없다")?;
    let text =
        fs::read_to_string(path).map_err(|e| format!("{}을 읽지 못했다: {e}", path.display()))?;
    let script = Script::parse(&text, &settings)?;
    print!("{}", script.describe());
    let joints: Vec<&Joint> = script
        .joints
        .iter()
        .map(|name| {
            settings
                .joint(name)
                .ok_or_else(|| format!("로봇 설정에 없는 관절이다: {name}"))
        })
        .collect::<Result<_, _>>()?;
    let servos: Vec<Servo> = joints
        .iter()
        .map(|joint| Servo::from_joint(joint))
        .collect::<Result<_, _>>()?;
    let caps = Caps::for_loop(&settings.run_loop);
    let robot_toml = fs::read_to_string(&options.settings)
        .map_err(|e| format!("{}을 읽지 못했다: {e}", options.settings.display()))?;
    let source = Source {
        settings: &settings,
        script_text: &text,
        robot_toml: &robot_toml,
    };
    if let Some(address) = &options.sim {
        let sim: Vec<SimServo> = joints
            .iter()
            .map(|joint| SimServo {
                id: joint.id,
                joint: joint.name.to_owned(),
                model: joint.model_number,
            })
            .collect();
        let bus = SimBus::connect(address, LOOP_PERIOD, &sim).map_err(|e| e.to_string())?;
        let link = json!({ "sim": address });
        // 시뮬레이션은 메모리 표(EEPROM)를 흉내 내지 않아서, 토크를 켤 때 값 적용의 목표와 견주지 않는다
        let servos = servos
            .into_iter()
            .map(|mut servo| {
                servo.eeprom.clear();
                servo
            })
            .collect();
        return start_script(
            options,
            &source,
            script,
            Joints::open(bus, servos, caps)?,
            link,
        );
    }
    let port = options.port.as_deref().unwrap_or_default();
    let bus = Bus::open(port, BAUD_RATES[0].1, LOOP_LATENCY).map_err(|e| e.to_string())?;
    let link = record::run_info(port, BAUD_RATES[0].1);
    start_script(
        options,
        &source,
        script,
        Joints::open(bus, servos, caps)?,
        link,
    )
}

/// 시험 순서를 돌릴 때 기록에 함께 남길 것.
struct Source<'a> {
    settings: &'a Settings,
    script_text: &'a str,
    robot_toml: &'a str,
}

/// --yes가 없으면 서보 상태만 읽어 보이고, 있으면 실행 루프를 세워 순서를 돌린다.
fn start_script<B: ServoBus>(
    options: &Options,
    source: &Source,
    script: Script,
    mut joints: Joints<B>,
    link: Value,
) -> Result<(), String> {
    if !options.yes {
        for id in joints.ids() {
            let state = joints.snapshot(id)?;
            println!("서보 {id}: {}", check::describe(&state));
        }
        println!(
            "--yes를 붙이면 토크를 켜고 순서대로 움직인다. 움직이거나 버티는 동안 Enter를 누르면 감속해 선다"
        );
        return Ok(());
    }
    let pauses = script
        .steps
        .iter()
        .any(|step| matches!(step, Step::Pause(_)));
    if pauses && !std::io::stdin().is_terminal() {
        return Err("멈춤 단계가 있는 순서는 Enter를 눌러야 해서 터미널 창에서 돌린다".to_owned());
    }
    let run = RunLoop::start(joints, source.settings.run_loop, None)?;
    drive_script(options, source, script, run, link)
}

/// 실행 명령의 터미널: 같은 줄을 고쳐 쓰는 상태 줄 뒤에 알림을 낼 때 먼저 줄을 바꾼다.
struct RunConsole {
    terminal: Terminal,
    status_shown: bool,
}

impl RunConsole {
    fn end_status(&mut self) {
        if self.status_shown {
            println!();
            self.status_shown = false;
        }
    }
}

impl Console for RunConsole {
    fn say(&mut self, text: &str) {
        self.end_status();
        self.terminal.say(text);
    }

    fn ask(&mut self, question: &str) -> String {
        self.end_status();
        self.terminal.ask(question)
    }

    fn begin_wait(&mut self, prompt: &str) {
        self.end_status();
        self.terminal.begin_wait(prompt);
    }

    fn enter_pressed(&mut self) -> bool {
        self.terminal.enter_pressed()
    }

    fn status(&mut self, text: &str) {
        self.terminal.status(text);
        self.status_shown = true;
    }
}

/// 기록 칸: 모드, 그리고 관절마다 목표, 위치, 속도, 부하, 온도, 전압, 전류, 상태, 토크 스위치.
const RUN_FIELDS: [&str; 9] = [
    "goal",
    "position",
    "speed",
    "load",
    "temperature",
    "voltage",
    "current",
    "status",
    "torque",
];

fn mode_code(mode: &Mode) -> i32 {
    match mode {
        Mode::Holding => 0,
        Mode::Moving => 1,
        Mode::Stopping => 2,
        Mode::Jogging => 3,
        Mode::Fault(_) => 9,
    }
}

fn run_row<B: ServoBus>(run: &RunLoop<B>) -> Vec<i32> {
    let mut row = vec![mode_code(run.mode())];
    for (_, state) in run.states() {
        row.extend([
            state.goal_position,
            state.position,
            state.speed,
            state.load,
            state.temperature,
            state.voltage,
            state.current,
            i32::from(state.status.0),
            state.torque_enable,
        ]);
    }
    row
}

/// 사람이 볼 한 줄: 관절마다 각, 부하, 온도.
fn run_status<B: ServoBus>(run: &RunLoop<B>, settings: &Settings, names: &[&str]) -> String {
    let parts: Vec<String> = run
        .states()
        .iter()
        .zip(names)
        .map(|((_, state), name)| {
            let angle = settings
                .joint(name)
                .and_then(|joint| joint.calibration)
                .map_or(f64::NAN, |c| {
                    c.angle_at(f64::from(state.position)).to_degrees()
                });
            format!(
                "{name} {angle:.1}° 부하 {:.1}% {}°C",
                f64::from(state.load) / 10.0,
                state.temperature
            )
        })
        .collect();
    parts.join(" | ")
}

fn drive_script<B: ServoBus>(
    options: &Options,
    source: &Source,
    script: Script,
    mut run: RunLoop<B>,
    link: Value,
) -> Result<(), String> {
    let names = script.joints.clone();
    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "run", "run", &started)?;
    let meta_path = folder.join("meta.json");
    let mut meta = json!({
        "kind": "run",
        "started": record::time_text(&started),
        "link": link,
        "script_name": script.name,
        "script": source.script_text,
        "robot_toml": source.robot_toml,
        "period_s": LOOP_PERIOD.as_secs_f64(),
        "joints": names,
    });
    record::write_json(&meta_path, &meta)?;
    let mut columns = vec!["mode".to_owned()];
    for name in &names {
        for field in RUN_FIELDS {
            columns.push(format!("{name}_{field}"));
        }
    }
    let column_refs: Vec<&str> = columns.iter().map(String::as_str).collect();
    let mut log = Log::create(&folder, &column_refs)?;
    let mut console = RunConsole {
        terminal: Terminal::closed_is_silent(),
        status_shown: false,
    };
    let mut clock = WallClock::default();
    let mut runner = Runner::new(script, &source.settings.run_loop, LOOP_PERIOD.as_secs_f64());
    let period = LOOP_PERIOD.as_secs_f64();
    let mut seen = BTreeSet::new();
    let mut last = clock.now();
    let mut next = last;
    let mut ticks: u64 = 0;
    let mut tick = |run: &mut RunLoop<B>,
                    clock: &mut WallClock,
                    log: &mut Log,
                    label: &str|
     -> Result<f64, String> {
        next += period;
        let now = clock.now();
        if next > now {
            clock.sleep(next - now);
        } else if now - next > 0.2 {
            next = now; // 많이 밀렸으면 따라잡으려 서두르지 않는다
        }
        let now = clock.now();
        let dt = (now - last).max(1e-4);
        last = now;
        run.tick(dt);
        log.sample(now, label, &run_row(run))?;
        Ok(now)
    };
    let status = loop {
        let label = runner.label();
        let now = tick(&mut run, &mut clock, &mut log, &label)?;
        let status = runner.tick(&mut run, &mut console, now);
        for event in runner.take_events() {
            log.event(now, &label, &event)?;
        }
        for warning in run.warnings() {
            if seen.insert(warning.clone()) {
                console.say(&format!("알림: {warning}"));
                log.event(now, &label, &warning)?;
            }
        }
        ticks += 1;
        if ticks.is_multiple_of(50) {
            console.status(&run_status(&run, source.settings, &names));
        }
        if status != Status::Running {
            break status;
        }
    };
    // 멈췄으면 감속이 끝나 붙잡을 때까지 박자를 더 돌린다 (그 전에 끝내면 마지막 목표에서 갑자기 선다)
    if matches!(status, Status::Stopped(_)) {
        for _ in 0..1000 {
            if matches!(run.mode(), Mode::Holding | Mode::Fault(_)) {
                break;
            }
            tick(&mut run, &mut clock, &mut log, "settle")?;
        }
    }
    console.end_status();
    meta["end"] = json!(match &status {
        Status::Stopped(_) => "stopped",
        _ => "done",
    });
    meta["result"] = json!(match &status {
        Status::Stopped(reason) => reason.clone(),
        _ => "순서를 마쳤다".to_owned(),
    });
    meta["released"] = json!(runner.released());
    meta["mode_at_end"] = json!(format!("{:?}", run.mode()));
    meta["ended"] = json!(record::time_text(&Local::now()));
    record::write_json(&meta_path, &meta)?;
    if !runner.released() {
        println!(
            "서보는 마지막 자리에서 버틴다 (토크 켜짐). 끝내려면 팔을 받친 뒤 + 플러그를 뺀다 (0016)"
        );
    }
    println!("기록: {}", folder.display());
    match status {
        Status::Stopped(reason) => Err(reason),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(text: &str) -> impl Iterator<Item = String> {
        text.split_whitespace()
            .map(str::to_owned)
            .collect::<Vec<_>>()
            .into_iter()
    }

    #[test]
    fn settings_command_reads_the_file_option() {
        let options = parse_args(args("settings")).unwrap();
        assert_eq!(options.command, Command::Settings);
        assert_eq!(options.settings, PathBuf::from("data/robot.toml"));
        let options = parse_args(args("--settings other.toml settings")).unwrap();
        assert_eq!(options.settings, PathBuf::from("other.toml"));
    }

    #[test]
    fn apply_reads_port_joints_and_yes() {
        let options = parse_args(args("apply --port COM5")).unwrap();
        assert_eq!(options.command, Command::Apply);
        assert_eq!(options.port.as_deref(), Some("COM5"));
        assert!(options.joints.is_empty() && !options.yes);
        let options = parse_args(args(
            "apply --port COM5 --joint j3 --joint j2 --yes --records r",
        ))
        .unwrap();
        assert_eq!(options.joints, ["j3", "j2"]);
        assert!(options.yes);
        assert_eq!(options.records, PathBuf::from("r"));
    }

    #[test]
    fn bad_arguments_are_refused() {
        assert!(parse_args(args("")).is_err());
        assert!(parse_args(args("move")).is_err());
        assert!(parse_args(args("settings settings")).is_err());
        assert!(parse_args(args("settings --settings")).is_err());
        assert!(parse_args(args("apply")).is_err()); // --port가 없다
        assert!(parse_args(args("settings --what")).is_err());
        assert_eq!(parse_args(args("--help")).unwrap_err(), "");
    }

    #[test]
    fn check_needs_port_and_id() {
        let options = parse_args(args("check --port COM5 --id 20")).unwrap();
        assert_eq!(options.command, Command::Check);
        assert_eq!(options.id, Some(20));
        assert!(!options.yes);
        let options = parse_args(args("check-state --port COM5 --id 20")).unwrap();
        assert_eq!(options.command, Command::CheckState);
        assert_eq!(options.situation, None);
        let options = parse_args(args("check-state --port COM5 --id 20 --situation 3")).unwrap();
        assert_eq!(options.situation, Some(3));
        assert!(parse_args(args("check-state --port COM5 --id 20 --situation 5")).is_err());
        assert!(parse_args(args("check --port COM5")).is_err());
        assert!(parse_args(args("check --id 20")).is_err());
        assert!(parse_args(args("check --port COM5 --id 300")).is_err());
    }

    #[test]
    fn zero_and_run_take_their_options() {
        let options = parse_args(args("zero --port COM5 --joint j2 --direction -1")).unwrap();
        assert_eq!(options.command, Command::Zero);
        assert_eq!(options.direction, Some(Direction::Reversed));
        assert!(parse_args(args("zero --port COM5")).is_err()); // 관절이 없다
        assert!(parse_args(args("zero --port COM5 --joint j2 --joint j3")).is_err());
        assert!(parse_args(args("zero --port COM5 --joint j2 --direction 2")).is_err());
        let options = parse_args(args("run --script a.toml --sim 127.0.0.1:5600 --yes")).unwrap();
        assert_eq!(options.command, Command::Run);
        assert_eq!(options.script, Some(PathBuf::from("a.toml")));
        assert_eq!(options.sim.as_deref(), Some("127.0.0.1:5600"));
        assert!(parse_args(args("run --script a.toml")).is_err()); // 붙을 곳이 없다
        assert!(parse_args(args("run --script a.toml --port COM5 --sim x:1")).is_err());
        assert!(parse_args(args("run --port COM5")).is_err()); // 순서가 없다
    }

    #[test]
    fn check_takes_step_names() {
        let options =
            parse_args(args("check --port COM5 --id 20 --only state,voltage --yes")).unwrap();
        assert_eq!(options.only, ["state", "voltage"]);
        let error = parse_args(args("check --port COM5 --id 20 --only volt")).unwrap_err();
        assert!(error.contains("모르는 단계"), "{error}");
    }

    #[test]
    fn check_runs_only_on_spare_ids() {
        let options = parse_args(args("check --port COM99 --id 3 --yes")).unwrap();
        assert!(run_check(&options).unwrap_err().contains("예비 서보"));
    }

    #[test]
    fn last_check_results_finds_the_newest_check_of_that_servo() {
        let root = std::env::temp_dir().join(format!("horilab-lock-test-{}", std::process::id()));
        let servo = root.join("servo");
        for (name, id, p) in [
            ("2026-11-12_100000_check-a", 20, Some(32)),
            ("2026-11-12_110000_check-a", 20, Some(30)),
            ("2026-11-12_113000_check-a", 20, None), // 잠금 단계 전에 끊겼다
            ("2026-11-12_120000_check-a", 21, Some(28)),
        ] {
            fs::create_dir_all(servo.join(name)).unwrap();
            let lock = p.map_or(json!({}), |p| json!({ "p_before": p, "p_trial": p + 8 }));
            let meta = json!({
                "kind": "servo-check",
                "servo": { "id": id },
                "results": { "lock": lock },
            });
            record::write_json(&servo.join(name).join("meta.json"), &meta).unwrap();
        }
        let results = last_check_results(&root, 20).unwrap();
        assert_eq!(results["lock"]["p_before"], 30);
        assert!(last_check_results(&root, 5).is_none());
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn joints_can_be_selected() {
        let settings = Settings::load(&PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/robot.toml"
        )))
        .unwrap();
        assert_eq!(
            selected(&settings, &[]).unwrap().len(),
            settings.joints.len()
        );
        assert_eq!(
            selected(&settings, &["j3".to_owned()]).unwrap()[0].name,
            "j3"
        );
        assert!(selected(&settings, &["j7".to_owned()]).is_err());
    }
}
