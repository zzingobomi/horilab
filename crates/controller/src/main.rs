//! 로봇 컨트롤러 (docs/decisions/0023-robot-controller.md, 0024-servo-bus.md).
//!
//! 로봇 설정 확인, 포트 목록, 값 적용(0024 만드는 순서 2~3), 맨 서보 확인(만드는 순서 5)이 있다.
//! 레포 루트에서 돌린다:
//!
//!     cargo run -p horilab-controller -- settings
//!     cargo run -p horilab-controller -- apply --port COM5            (바꿀 칸만 보여 준다)
//!     cargo run -p horilab-controller -- apply --port COM5 --yes      (쓴다)
//!     cargo run -p horilab-controller -- check --port COM5 --id 20    (할 일을 보여 주고 상태를 읽는다)
//!     cargo run -p horilab-controller -- check --port COM5 --id 20 --yes
//!     cargo run -p horilab-controller -- check-state --port COM5 --id 20

use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Duration;

use chrono::Local;
use horilab_controller::check::{self, BENCH_RANGE, Check, SPARE_IDS, STEPS, Terminal, WallClock};
use horilab_controller::joints::{BENCH_CAPS, Joints, Servo, Snapshot};
use horilab_controller::prepare::{self, Change};
use horilab_controller::record::{self, Log};
use horilab_controller::settings::{Joint, Settings};
use horilab_servo::feetech::registers::BAUD_RATES;
use horilab_servo::feetech::{Bus, Error, SerialLink};
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

옵션:
  --settings <파일>   로봇 설정 파일. 기본 data/robot.toml (레포 루트에서 돌릴 때)
  --port <포트>       서보 어댑터 포트 (예: COM5, /dev/ttyACM0)
  --joint <이름>      이 관절만 (여러 번 줄 수 있다). 기본은 로봇 설정의 모든 관절
  --id <ID>           맨 서보의 ID (check, check-state)
  --yes               보여 준 대로 서보에 쓰거나 움직인다
  --only <단계,단계>  맨 서보 확인에서 이 단계만 한다 (예: state,overload_hold,voltage)
  --situation <1~4>   끊김 확인의 상황 (check-state): 1 프로그램을 끈 뒤, 2 USB를 뺐다 꽂은 뒤,
                      3 + 플러그를 뺀 채, 4 + 플러그를 다시 꽂은 뒤
  --records <폴더>    기록 폴더. 기본 records (그 아래 servo/에 쓴다)
";

/// 서보 응답 하나를 더 기다리는 여유. 서보 준비는 박자가 없어 넉넉히 둔다.
const LATENCY: Duration = Duration::from_millis(20);

#[derive(Debug, PartialEq)]
enum Command {
    Settings,
    Ports,
    Apply,
    Check,
    CheckState,
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
            "-h" | "--help" => return Err(String::new()),
            name if command.is_none() && !name.starts_with('-') => {
                command = Some(match name {
                    "settings" => Command::Settings,
                    "ports" => Command::Ports,
                    "apply" => Command::Apply,
                    "check" => Command::Check,
                    "check-state" => Command::CheckState,
                    other => return Err(format!("모르는 명령이다: {other}")),
                })
            }
            other => return Err(format!("모르는 옵션이다: {other}")),
        }
    }
    options.command = command.ok_or("명령이 없다")?;
    let needs_port = matches!(
        options.command,
        Command::Apply | Command::Check | Command::CheckState
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
fn open_bench(options: &Options, id: u8) -> Result<Option<(Joints<SerialLink>, u16)>, String> {
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
