//! 로봇 컨트롤러 (docs/decisions/0023-robot-controller.md, 0024-servo-bus.md).
//!
//! 지금은 로봇 설정을 읽어 보여 주는 명령만 있다 (0024 만드는 순서 2). 서보 준비와 관절 시험 명령은
//! 만드는 순서 3~5에서 더한다. 레포 루트에서 돌린다:
//!
//!     cargo run -p horilab-controller -- settings

use std::path::PathBuf;
use std::process::ExitCode;

use horilab_controller::settings::Settings;

const USAGE: &str = "\
쓰는 법: horilab-controller <명령> [옵션]

명령:
  settings            로봇 설정을 읽어 검사하고 보여 준다

옵션:
  --settings <파일>   로봇 설정 파일. 기본 data/robot.toml (레포 루트에서 돌릴 때)
";

#[derive(Debug, PartialEq)]
enum Command {
    Settings,
}

#[derive(Debug)]
struct Options {
    command: Command,
    settings: PathBuf,
}

fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut command = None;
    let mut settings = PathBuf::from("data/robot.toml");
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--settings" => {
                settings = PathBuf::from(args.next().ok_or("--settings 뒤에 파일이 없다")?)
            }
            "-h" | "--help" => return Err(String::new()),
            "settings" if command.is_none() => command = Some(Command::Settings),
            other => return Err(format!("모르는 명령이나 옵션이다: {other}")),
        }
    }
    let command = command.ok_or("명령이 없다")?;
    Ok(Options { command, settings })
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
    match run(&options) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("오류: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(options: &Options) -> Result<(), String> {
    match options.command {
        Command::Settings => {
            let settings = Settings::load(&options.settings)?;
            println!("로봇 설정: {}", options.settings.display());
            print!("{}", settings.summary());
            Ok(())
        }
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
    fn bad_arguments_are_refused() {
        assert!(parse_args(args("")).is_err());
        assert!(parse_args(args("move")).is_err());
        assert!(parse_args(args("settings settings")).is_err());
        assert!(parse_args(args("settings --settings")).is_err());
        assert_eq!(parse_args(args("--help")).unwrap_err(), "");
    }
}
