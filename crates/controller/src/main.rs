//! 로봇 컨트롤러 (docs/decisions/0023-robot-controller.md, 0024-servo-bus.md).
//!
//! 지금은 로봇 설정 확인, 포트 목록, 값 적용 명령이 있다 (0024 만드는 순서 2~3). 관절 시험 명령은
//! 만드는 순서 4~5에서 더한다. 레포 루트에서 돌린다:
//!
//!     cargo run -p horilab-controller -- settings
//!     cargo run -p horilab-controller -- apply --port COM5            (바꿀 칸만 보여 준다)
//!     cargo run -p horilab-controller -- apply --port COM5 --yes      (쓴다)

use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use chrono::Local;
use horilab_controller::prepare::{self, Change};
use horilab_controller::record;
use horilab_controller::settings::{Joint, Settings};
use horilab_servo::feetech::Bus;
use horilab_servo::feetech::registers::BAUD_RATES;
use serde_json::{Value, json};

const USAGE: &str = "\
쓰는 법: horilab-controller <명령> [옵션]

명령:
  settings            로봇 설정을 읽어 검사하고 보여 준다
  ports               이 컴퓨터의 시리얼 포트를 보여 준다
  apply               값 적용: 서보의 EEPROM을 로봇 설정과 견주어 다른 칸을 보여 주고,
                      --yes가 있으면 쓴다. 토크가 꺼져 있어야 하고, 서보는 움직이지 않는다

옵션:
  --settings <파일>   로봇 설정 파일. 기본 data/robot.toml (레포 루트에서 돌릴 때)
  --port <포트>       서보 어댑터 포트 (예: COM5, /dev/ttyACM0)
  --joint <이름>      이 관절만 (여러 번 줄 수 있다). 기본은 로봇 설정의 모든 관절
  --yes               보여 준 대로 서보에 쓴다
  --records <폴더>    기록 폴더. 기본 records (그 아래 servo/에 쓴다)
";

/// 서보 응답 하나를 더 기다리는 여유. 서보 준비는 박자가 없어 넉넉히 둔다.
const LATENCY: Duration = Duration::from_millis(20);

#[derive(Debug, PartialEq)]
enum Command {
    Settings,
    Ports,
    Apply,
}

#[derive(Debug)]
struct Options {
    command: Command,
    settings: PathBuf,
    port: Option<String>,
    joints: Vec<String>,
    yes: bool,
    records: PathBuf,
}

fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut command = None;
    let mut options = Options {
        command: Command::Settings,
        settings: PathBuf::from("data/robot.toml"),
        port: None,
        joints: Vec::new(),
        yes: false,
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
            "--yes" => options.yes = true,
            "--records" => {
                options.records = PathBuf::from(args.next().ok_or("--records 뒤에 폴더가 없다")?)
            }
            "-h" | "--help" => return Err(String::new()),
            name if command.is_none() && !name.starts_with('-') => {
                command = Some(match name {
                    "settings" => Command::Settings,
                    "ports" => Command::Ports,
                    "apply" => Command::Apply,
                    other => return Err(format!("모르는 명령이다: {other}")),
                })
            }
            other => return Err(format!("모르는 옵션이다: {other}")),
        }
    }
    options.command = command.ok_or("명령이 없다")?;
    if options.command == Command::Apply && options.port.is_none() {
        return Err(
            "apply에는 --port가 있어야 한다 (포트 목록: horilab-controller ports)".to_owned(),
        );
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
