//! 기록: `records/<종류>/<날짜시각>_<명령>-<컴퓨터>/meta.json` (docs/decisions/0022-records-and-sync.md,
//! 0024의 기록 형식). 서보 기록은 Python 기준 구현의 기록과 같은 꼴로 남겨 두 구현의 결과를 견줄 수 있게 한다.

use std::fs;
use std::path::{Path, PathBuf};

use chrono::{DateTime, Local, SecondsFormat};
use horilab_servo::feetech::registers as reg;
use serde_json::{Map, Value, json};

use crate::prepare::{model_name, value_in};

/// 이 컴퓨터의 이름 (소문자, 폴더 이름에 쓸 수 있는 글자만).
pub fn hostname() -> String {
    let name = fs::read_to_string("/proc/sys/kernel/hostname")
        .ok()
        .map(|text| text.trim().to_owned())
        .or_else(|| std::env::var("COMPUTERNAME").ok())
        .or_else(|| std::env::var("HOSTNAME").ok())
        .unwrap_or_else(|| "host".to_owned());
    name.to_lowercase()
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || "_.-".contains(c) {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// 기록 폴더를 새로 만든다. 같은 이름이 있으면 뒤에 -2, -3을 붙인다.
pub fn new_record_dir(
    root: &Path,
    kind: &str,
    command: &str,
    started: &DateTime<Local>,
) -> Result<PathBuf, String> {
    let base = format!(
        "{}_{command}-{}",
        started.format("%Y-%m-%d_%H%M%S"),
        hostname()
    );
    let parent = root.join(kind);
    let mut path = parent.join(&base);
    let mut n = 1;
    while path.exists() {
        n += 1;
        path = parent.join(format!("{base}-{n}"));
    }
    fs::create_dir_all(&path).map_err(|e| format!("{}를 만들지 못했다: {e}", path.display()))?;
    Ok(path)
}

/// 기록에 남길 시각 (세계 시각, 사진·영상 시각과 맞춘다).
pub fn time_text(time: &DateTime<Local>) -> String {
    time.to_rfc3339_opts(SecondsFormat::Secs, false)
}

/// 어디서 어떤 코드로 돌렸나. 기록만 보고 다른 PC에서 분석할 수 있게 남긴다 (0022).
pub fn run_info(port: &str, baud: u32) -> Value {
    let uncommitted = match env!("HORILAB_UNCOMMITTED") {
        "true" => json!(true),
        "false" => json!(false),
        _ => Value::Null,
    };
    json!({
        "host": hostname(),
        "system": format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH),
        "program": format!("horilab-controller {}", env!("CARGO_PKG_VERSION")),
        "commit": env!("HORILAB_COMMIT"),
        "uncommitted_changes": uncommitted,
        "port": port,
        "baud": baud,
    })
}

/// 서보 하나의 덤프: memory는 16진수, values는 레지스터 이름 → 값 (Python 덤프 기록과 같은 꼴).
pub fn dump_json(memory: &[u8]) -> Value {
    let hex: Vec<String> = memory.iter().map(|byte| format!("{byte:02x}")).collect();
    let values: Map<String, Value> = reg::ALL
        .iter()
        .filter_map(|&register| {
            value_in(memory, register).map(|v| (register.name.to_owned(), json!(v)))
        })
        .collect();
    let mut dump = json!({ "memory": hex.join(" "), "values": values });
    if let Some(model) = value_in(memory, reg::MODEL).and_then(|model| u16::try_from(model).ok()) {
        dump["model"] = json!(model);
        dump["model_name"] = json!(model_name(model));
    }
    dump
}

/// JSON을 임시 파일에 쓰고 이름을 바꾼다. 중간에 끊겨도 반쯤 쓴 파일이 남지 않는다.
pub fn write_json(path: &Path, value: &Value) -> Result<(), String> {
    let text = serde_json::to_string_pretty(value).map_err(|e| e.to_string())? + "\n";
    let temporary = path.with_extension("json.tmp");
    fs::write(&temporary, text)
        .and_then(|()| fs::rename(&temporary, path))
        .map_err(|e| format!("{}를 쓰지 못했다: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use horilab_servo::feetech::fake::FakeServo;

    use super::*;

    #[test]
    fn dump_json_matches_the_python_record_shape() {
        let memory = FakeServo::with(3, 2825, 3093).read(0, reg::MEMORY_SIZE as u8);
        let dump = dump_json(&memory);
        assert_eq!(dump["values"]["present_position"], json!(3093));
        assert_eq!(dump["model"], json!(2825));
        assert_eq!(dump["model_name"], json!("STS3250"));
        let hex = dump["memory"].as_str().unwrap();
        assert_eq!(hex.split(' ').count(), reg::MEMORY_SIZE);
        assert!(hex.starts_with("03 0a 00 09 0b 03"), "{hex}"); // 펌웨어 3.10, 모델 2825, ID 3
    }

    #[test]
    fn record_dirs_do_not_collide() {
        let root = std::env::temp_dir().join(format!("horilab-record-test-{}", std::process::id()));
        let started = Local::now();
        let first = new_record_dir(&root, "servo", "apply", &started).unwrap();
        let second = new_record_dir(&root, "servo", "apply", &started).unwrap();
        assert_ne!(first, second);
        assert!(
            second
                .file_name()
                .unwrap()
                .to_string_lossy()
                .ends_with("-2")
        );
        let path = first.join("meta.json");
        write_json(&path, &json!({ "a": 1 })).unwrap();
        assert_eq!(fs::read_to_string(&path).unwrap(), "{\n  \"a\": 1\n}\n");
        fs::remove_dir_all(&root).unwrap();
    }
}
