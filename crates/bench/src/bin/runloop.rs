//! 실행 루프 부하 시험 (docs/decisions/0026-pi-runtime-deploy-records.md의 2, 미리 재기 2).
//!
//! 가짜 서보 7개(그리퍼 포함)를 물린 서보 버스로 100 Hz 실행 루프를 흉내 낸다 (`horilab_bench::runloop`).
//! 박자 기록은 메모리 줄에 넣기만 하고, 낮은 우선순위 스레드가 모아 1초마다 samples.csv에 쓴다. 줄이 차면
//! 실행 루프는 기다리지 않고 기록을 버리며 버린 수를 센다 (0026의 5). 끝나면 meta.json에 요약과 통과
//! 여부를 쓰고, 도수분포를 histogram.csv에 쓴다. meta.json에 끝 표시(`end`)가 없으면 중간에 끊긴 기록이다.
//!
//! 파이에서 돌린다. 실시간 우선순위와 메모리 잠금은 root가 필요하다:
//!
//!     sudo ./runloop --duration 3h --fifo 80 --mlock --bus-csv timing-hori3.csv --note "부하: ..."

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;
use std::time::Duration;

use chrono::{Local, SecondsFormat};
use horilab_bench::rt;
use horilab_bench::runloop::{
    ARM_SERVOS, Beat, BusModel, DEFAULT_BUS_US, LoopSettings, LoopThread, PREFAULT_STACK, Second,
    Totals, WAKE_LIMIT_US, Worst, parse_duration, run_loop,
};
use serde_json::{Value, json};

const USAGE: &str = "\
쓰는 법: runloop [옵션]
  --duration <시간>   돌릴 시간 (예: 90s, 30m, 3h). 기본 60s
  --hz <n>            박자 수 [Hz]. 기본 100
  --fifo <우선순위>   실행 루프 스레드를 SCHED_FIFO로 (0026: 80에서 시작). root가 필요하다
  --mlock             메모리를 모두 잠근다 (mlockall). root가 필요하다
  --cpu <n>           실행 루프를 코어 n에 묶고, 기록 스레드는 나머지 코어에 둔다
  --bus-csv <파일>    첫 연결 통신 시간 기록의 samples.csv (한 박자 줄의 시간을 쓴다)
  --bus-us <n>        한 박자 버스 시간을 고정 값으로 [µs]. 기본 2750
  --compute-us <n>    박자마다 계산을 흉내 내는 시간 [µs]. 기본 1000 (직교 조그의 IK 한 번 상한)
  --seed <n>          버스 시간을 고르는 난수 씨앗. 기본 1
  --name <이름>       기록 폴더 이름에 붙는 말. 기본 runloop
  --note <글>         기록에 남길 메모 (건 부하 등)
  --records <폴더>    기록 폴더. 기본 records (그 아래 bench/에 쓴다)
";

/// 기록 줄의 크기: 100 Hz로 40초 남짓. 기록 스레드가 이만큼 늦으면 버린다.
const CHANNEL_CAPACITY: usize = 4096;
/// 가장 나쁜 박자를 몇 개 남길지
const WORST_KEEP: usize = 20;
/// samples.csv를 디스크에 확정하는 간격 [초]
const SYNC_EVERY_S: u64 = 10;
/// 진행을 알리는 간격 [초]
const REPORT_EVERY_S: u64 = 60;

const SAMPLES_HEADER: &str = "second,time,beats,missed,wake_mean_us,wake_max_us,work_max_us,\
bus_max_us,compute_max_us,bus_errors,dropped,temp_c,cpu_mhz,load1";

#[derive(Debug)]
struct Options {
    duration: Duration,
    hz: u32,
    fifo: Option<i32>,
    mlock: bool,
    cpu: Option<usize>,
    bus_csv: Option<PathBuf>,
    bus_us: Option<u32>,
    compute_us: u32,
    seed: u64,
    name: String,
    note: String,
    records: PathBuf,
}

fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut options = Options {
        duration: Duration::from_secs(60),
        hz: 100,
        fifo: None,
        mlock: false,
        cpu: None,
        bus_csv: None,
        bus_us: None,
        compute_us: 1_000,
        seed: 1,
        name: "runloop".to_owned(),
        note: String::new(),
        records: PathBuf::from("records"),
    };
    fn number<T: std::str::FromStr>(flag: &str, value: Option<String>) -> Result<T, String> {
        let value = value.ok_or(format!("{flag} 뒤에 값이 없다"))?;
        value
            .parse()
            .map_err(|_| format!("{flag}의 값을 읽지 못했다: {value}"))
    }
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--duration" => {
                options.duration = parse_duration(&args.next().ok_or("--duration 뒤에 값이 없다")?)?
            }
            "--hz" => options.hz = number("--hz", args.next())?,
            "--fifo" => options.fifo = Some(number("--fifo", args.next())?),
            "--mlock" => options.mlock = true,
            "--cpu" => options.cpu = Some(number("--cpu", args.next())?),
            "--bus-csv" => {
                options.bus_csv = Some(PathBuf::from(
                    args.next().ok_or("--bus-csv 뒤에 파일이 없다")?,
                ))
            }
            "--bus-us" => options.bus_us = Some(number("--bus-us", args.next())?),
            "--compute-us" => options.compute_us = number("--compute-us", args.next())?,
            "--seed" => options.seed = number("--seed", args.next())?,
            "--name" => options.name = args.next().ok_or("--name 뒤에 값이 없다")?,
            "--note" => options.note = args.next().ok_or("--note 뒤에 값이 없다")?,
            "--records" => {
                options.records = PathBuf::from(args.next().ok_or("--records 뒤에 폴더가 없다")?)
            }
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("모르는 옵션이다: {other}")),
        }
    }
    if options.hz == 0 || options.hz > 1_000 {
        return Err(format!("--hz는 1~1000이다: {}", options.hz));
    }
    if options.bus_csv.is_some() && options.bus_us.is_some() {
        return Err("--bus-csv와 --bus-us는 하나만 준다".to_owned());
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
    match run(&options) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(error) => {
            eprintln!("오류: {error}");
            ExitCode::from(2)
        }
    }
}

fn run(options: &Options) -> Result<bool, String> {
    let bus = match (&options.bus_csv, options.bus_us) {
        (Some(path), _) => {
            let text = fs::read_to_string(path)
                .map_err(|e| format!("{}을 읽지 못했다: {e}", path.display()))?;
            BusModel::from_timing_csv(&text, &path.display().to_string())?
        }
        (None, Some(us)) => BusModel::fixed(us),
        (None, None) => BusModel::fixed(DEFAULT_BUS_US),
    };
    let period_ns = 1_000_000_000 / u64::from(options.hz);
    let settings = LoopSettings {
        period_ns,
        beats: (options.duration.as_secs_f64() * f64::from(options.hz)).round() as u64,
        compute_ns: u64::from(options.compute_us) * 1_000,
        bus,
        seed: options.seed,
        fifo: options.fifo,
        cpu: options.cpu,
    };
    rt::catch_stop_signals().map_err(|e| format!("멈춤 신호를 잡지 못했다: {e}"))?;
    // 다른 스레드를 띄우기 전에 잠가서 모든 스레드의 스택이 잠긴다
    if options.mlock {
        rt::lock_memory().map_err(|e| format!("메모리를 잠그지 못했다 (root로 돌린다): {e}"))?;
    }

    // 실행 루프 스레드: 준비한 결과를 먼저 받고, 기록 폴더는 그 뒤에 만든다
    let (beat_tx, beat_rx) = mpsc::sync_channel::<Beat>(CHANNEL_CAPACITY);
    let (ready_tx, ready_rx) = mpsc::channel::<Result<LoopThread, String>>();
    let dropped = Arc::new(AtomicU64::new(0));
    let loop_settings = settings.clone();
    let loop_dropped = Arc::clone(&dropped);
    let runner = thread::Builder::new()
        .name("runloop".to_owned())
        .stack_size(PREFAULT_STACK + 1024 * 1024)
        .spawn(move || {
            let ready = ready_tx.clone();
            let result = run_loop(
                &loop_settings,
                |thread| {
                    let _ = ready.send(Ok(thread.clone()));
                },
                |beat| {
                    if beat_tx.try_send(beat).is_err() {
                        loop_dropped.fetch_add(1, Ordering::Relaxed);
                    }
                },
            );
            if let Err(error) = &result {
                let _ = ready_tx.send(Err(error.clone()));
            }
            result
        })
        .map_err(|e| format!("실행 루프 스레드를 띄우지 못했다: {e}"))?;
    let loop_thread = match ready_rx.recv() {
        Ok(Ok(thread)) => thread,
        Ok(Err(error)) => {
            let _ = runner.join();
            return Err(error);
        }
        Err(_) => {
            let _ = runner.join();
            return Err("실행 루프 스레드가 준비 전에 멈췄다".to_owned());
        }
    };

    let started = Local::now();
    let host = hostname();
    let folder = options.records.join("bench").join(format!(
        "{}_{}-{host}",
        started.format("%Y-%m-%d_%H%M%S"),
        options.name
    ));
    fs::create_dir_all(&folder)
        .map_err(|e| format!("{}를 만들지 못했다: {e}", folder.display()))?;
    let mut meta = json!({
        "kind": "bench-runloop",
        "started": started.to_rfc3339_opts(SecondsFormat::Secs, false),
        "run": system_info(&host),
        "settings": {
            "hz": options.hz,
            "duration_s": options.duration.as_secs_f64(),
            "beats": settings.beats,
            "fifo_priority": options.fifo,
            "mlock": options.mlock,
            "cpu": options.cpu,
            "compute_us": options.compute_us,
            "seed": options.seed,
            "servos": ARM_SERVOS.iter().map(|(id, _)| id).collect::<Vec<_>>(),
            "bus": settings.bus.describe(),
            "bus_wait": "동기 쓰기, 읽기 요청, 응답의 세 번으로 나눠 잔다 (선 위의 바이트 수에 비례)",
            "wake_limit_us": WAKE_LIMIT_US,
            "note": options.note,
        },
        "loop_thread": {
            "policy": loop_thread.policy,
            "priority": loop_thread.priority,
            "affinity": loop_thread.affinity,
            "memory_locked": options.mlock,
        },
    });
    write_json(&folder.join("meta.json"), &meta)?;
    println!("기록: {}", folder.display());
    println!(
        "{} Hz, {} 박자, 실행 루프 {} {} (코어 {:?}), 버스 {}",
        options.hz,
        settings.beats,
        loop_thread.policy,
        loop_thread.priority,
        loop_thread.affinity,
        settings.bus.describe()
    );

    // 기록 스레드: 실행 루프를 묶은 코어가 있으면 나머지 코어에 둔다
    let other_cores: Option<Vec<usize>> = options.cpu.map(|cpu| {
        let cores = thread::available_parallelism().map_or(1, |n| n.get());
        (0..cores).filter(|&c| c != cpu).collect()
    });
    let collector_folder = folder.clone();
    let collector_dropped = Arc::clone(&dropped);
    let loop_cpu = options.cpu.unwrap_or(0);
    let hz = u64::from(options.hz);
    let collector = thread::Builder::new()
        .name("records".to_owned())
        .spawn(move || {
            if let Some(cores) = other_cores.filter(|c| !c.is_empty()) {
                let _ = rt::pin_to_cpus(&cores);
            }
            collect(beat_rx, &collector_dropped, &collector_folder, hz, loop_cpu)
        })
        .map_err(|e| format!("기록 스레드를 띄우지 못했다: {e}"))?;

    let loop_result = runner
        .join()
        .map_err(|_| "실행 루프 스레드가 죽었다".to_owned())?;
    let collected = collector
        .join()
        .map_err(|_| "기록 스레드가 죽었다".to_owned())?;
    let dropped = dropped.load(Ordering::Relaxed);

    let (end, passed) = match (&loop_result, &collected) {
        (Err(error), _) => (format!("오류: {error}"), false),
        (_, Err(error)) => (format!("기록 오류: {error}"), false),
        (Ok(end), Ok(c)) if end.stopped => ("stopped".to_owned(), c.totals.passed(dropped)),
        (Ok(_), Ok(c)) => ("normal".to_owned(), c.totals.passed(dropped)),
    };
    meta["ended"] = json!(Local::now().to_rfc3339_opts(SecondsFormat::Secs, false));
    meta["end"] = json!(end);
    if let Ok(c) = &collected {
        meta["summary"] = c.totals.summary(dropped);
        meta["seconds"] = json!(c.seconds);
        meta["temperature_c"] =
            json!({ "start": c.temp_start_c, "max": c.temp_max_c, "end": c.temp_end_c });
        meta["worst_wake"] = c.worst_wake.to_json(period_ns);
        meta["worst_work"] = c.worst_work.to_json(period_ns);
        write_histograms(&folder.join("histogram.csv"), &c.totals)?;
    }
    if let Ok(loop_end) = &loop_result {
        meta["ran"] = json!(loop_end.ran);
    }
    write_json(&folder.join("meta.json"), &meta)?;

    println!("끝: {end}");
    if let Some(summary) = meta.get("summary") {
        println!(
            "{}",
            serde_json::to_string_pretty(summary).unwrap_or_default()
        );
    }
    println!(
        "{}",
        if passed {
            "통과: 깨어나기 최악 1 ms 안, 놓친 박자 0, 버린 기록 0"
        } else {
            "통과 못 함 (위 요약의 pass를 본다)"
        }
    );
    Ok(passed)
}

struct Collected {
    totals: Totals,
    worst_wake: Worst,
    worst_work: Worst,
    seconds: u64,
    temp_start_c: Option<f64>,
    temp_max_c: Option<f64>,
    temp_end_c: Option<f64>,
}

/// 박자 기록을 모아 1초마다 samples.csv에 한 줄씩 쓴다. 실행 루프가 끝나 줄이 닫히면 돌아온다.
fn collect(
    rx: Receiver<Beat>,
    dropped: &AtomicU64,
    folder: &Path,
    hz: u64,
    loop_cpu: usize,
) -> Result<Collected, String> {
    let path = folder.join("samples.csv");
    let file = File::create(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut out = BufWriter::new(file);
    let io = |e: std::io::Error| format!("samples.csv를 쓰지 못했다: {e}");
    writeln!(out, "{SAMPLES_HEADER}").map_err(io)?;
    let mut c = Collected {
        totals: Totals::default(),
        worst_wake: Worst::new(WORST_KEEP),
        worst_work: Worst::new(WORST_KEEP),
        seconds: 0,
        temp_start_c: temperature_c(),
        temp_max_c: None,
        temp_end_c: None,
    };
    let mut second = Second::default();
    let mut current = 0u64; // 지금 모으는 초
    let mut minute_wake_max = 0u64;
    let mut write_row = |c: &mut Collected, second: &Second, at: u64| -> Result<(), String> {
        let temp = temperature_c();
        if let Some(t) = temp {
            c.temp_max_c = Some(c.temp_max_c.map_or(t, |m: f64| m.max(t)));
        }
        c.temp_end_c = temp;
        writeln!(
            out,
            "{at},{},{},{},{:.1},{},{},{},{},{},{},{},{},{}",
            Local::now().format("%H:%M:%S"),
            second.beats,
            second.missed,
            second.wake_mean_us(),
            second.wake_max_us,
            second.work_max_us,
            second.bus_max_us,
            second.compute_max_us,
            second.bus_errors,
            dropped.load(Ordering::Relaxed),
            or_empty(temp.map(|t| format!("{t:.1}"))),
            or_empty(cpu_mhz(loop_cpu).map(|m| m.to_string())),
            or_empty(load1()),
        )
        .map_err(io)?;
        out.flush().map_err(io)?;
        c.seconds += 1;
        if c.seconds.is_multiple_of(SYNC_EVERY_S) {
            out.get_ref().sync_data().map_err(io)?;
        }
        Ok(())
    };
    loop {
        match rx.try_recv() {
            Ok(beat) => {
                let at = beat.index / hz;
                while at > current {
                    minute_wake_max = minute_wake_max.max(second.wake_max_us);
                    write_row(&mut c, &second, current)?;
                    if (current + 1).is_multiple_of(REPORT_EVERY_S) {
                        report(&c, current + 1, minute_wake_max, dropped);
                        minute_wake_max = 0;
                    }
                    second = Second::default();
                    current += 1;
                }
                second.add(&beat);
                c.totals.add(&beat);
                c.worst_wake.offer(beat.wake_ns, &beat);
                c.worst_work.offer(beat.work_ns, &beat);
            }
            Err(TryRecvError::Empty) => thread::sleep(Duration::from_millis(50)),
            Err(TryRecvError::Disconnected) => break,
        }
    }
    if second.beats > 0 {
        write_row(&mut c, &second, current)?;
    }
    Ok(c)
}

fn report(c: &Collected, seconds: u64, minute_wake_max: u64, dropped: &AtomicU64) {
    println!(
        "[{:>4}분] 박자 {}, 깨어나기 최대 {} µs (이번 1분 {} µs), 놓친 박자 {}, 버린 기록 {}, {}",
        seconds / 60,
        c.totals.beats,
        c.totals.wake.max_us(),
        minute_wake_max,
        c.totals.missed,
        dropped.load(Ordering::Relaxed),
        c.temp_end_c
            .map_or("온도 모름".to_owned(), |t| format!("{t:.1} °C")),
    );
}

fn or_empty(value: Option<String>) -> String {
    value.unwrap_or_default()
}

/// 네 도수분포를 한 파일에: 값이 있는 µs 칸만 쓴다. 마지막 줄은 상한을 넘은 수다.
fn write_histograms(path: &Path, totals: &Totals) -> Result<(), String> {
    let io = |e: std::io::Error| format!("{}를 쓰지 못했다: {e}", path.display());
    let mut out = BufWriter::new(File::create(path).map_err(io)?);
    writeln!(out, "us,wake,work,bus,compute").map_err(io)?;
    let all = [&totals.wake, &totals.work, &totals.bus, &totals.compute];
    for us in 0..=totals.wake.limit_us() {
        let counts = all.map(|h| h.count_at(us));
        if counts.iter().any(|&n| n > 0) {
            writeln!(
                out,
                "{us},{},{},{},{}",
                counts[0], counts[1], counts[2], counts[3]
            )
            .map_err(io)?;
        }
    }
    let over = all.map(|h| h.over_limit());
    writeln!(out, "over,{},{},{},{}", over[0], over[1], over[2], over[3]).map_err(io)?;
    out.flush().map_err(io)
}

fn write_json(path: &Path, value: &Value) -> Result<(), String> {
    let text = serde_json::to_string_pretty(value).map_err(|e| e.to_string())? + "\n";
    let temporary = path.with_extension("json.tmp");
    fs::write(&temporary, text)
        .and_then(|()| fs::rename(&temporary, path))
        .map_err(|e| format!("{}를 쓰지 못했다: {e}", path.display()))
}

fn read_trimmed(path: &str) -> Option<String> {
    fs::read_to_string(path).ok().map(|s| s.trim().to_owned())
}

fn hostname() -> String {
    read_trimmed("/proc/sys/kernel/hostname")
        .or_else(|| std::env::var("COMPUTERNAME").ok())
        .unwrap_or_else(|| "unknown".to_owned())
        .to_lowercase()
}

/// SoC 온도 [°C] (리눅스).
fn temperature_c() -> Option<f64> {
    let milli: f64 = read_trimmed("/sys/class/thermal/thermal_zone0/temp")?
        .parse()
        .ok()?;
    Some(milli / 1000.0)
}

/// 코어의 지금 클럭 [MHz] (리눅스).
fn cpu_mhz(cpu: usize) -> Option<u64> {
    let khz: u64 = read_trimmed(&format!(
        "/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq"
    ))?
    .parse()
    .ok()?;
    Some(khz / 1000)
}

/// 1분 평균 부하 (리눅스).
fn load1() -> Option<String> {
    read_trimmed("/proc/loadavg")?
        .split_whitespace()
        .next()
        .map(str::to_owned)
}

fn system_info(host: &str) -> Value {
    let uncommitted = match env!("HORILAB_UNCOMMITTED") {
        "true" => json!(true),
        "false" => json!(false),
        _ => Value::Null,
    };
    json!({
        "host": host,
        "kernel": read_trimmed("/proc/version"),
        "realtime_kernel": Path::new("/sys/kernel/realtime").exists(),
        "cmdline": read_trimmed("/proc/cmdline"),
        "cores": thread::available_parallelism().map_or(1, |n| n.get()),
        "governor": read_trimmed("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
        "target": format!(
            "{}-{} ({})",
            std::env::consts::ARCH,
            std::env::consts::OS,
            if cfg!(target_env = "musl") { "musl 정적" } else { "동적" }
        ),
        "commit": env!("HORILAB_COMMIT"),
        "uncommitted_changes": uncommitted,
        "user": std::env::var("SUDO_USER").or_else(|_| std::env::var("USER")).ok(),
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
    fn options_have_defaults_and_read_every_flag() {
        let options = parse_args(args("")).unwrap();
        assert_eq!(options.duration, Duration::from_secs(60));
        assert_eq!(options.hz, 100);
        assert_eq!(options.fifo, None);
        assert!(!options.mlock);
        let options = parse_args(args(
            "--duration 3h --hz 50 --fifo 80 --mlock --cpu 3 --bus-us 3000 --compute-us 500 --seed 9 --name x --records r",
        ))
        .unwrap();
        assert_eq!(options.duration, Duration::from_secs(3 * 3600));
        assert_eq!(options.hz, 50);
        assert_eq!(options.fifo, Some(80));
        assert!(options.mlock);
        assert_eq!(options.cpu, Some(3));
        assert_eq!(options.bus_us, Some(3000));
        assert_eq!(options.compute_us, 500);
        assert_eq!(options.seed, 9);
        assert_eq!(options.name, "x");
        assert_eq!(options.records, PathBuf::from("r"));
    }

    #[test]
    fn bad_options_are_refused() {
        assert!(parse_args(args("--hz 0")).is_err());
        assert!(parse_args(args("--fifo")).is_err());
        assert!(parse_args(args("--what")).is_err());
        assert!(parse_args(args("--bus-us 1 --bus-csv a.csv")).is_err());
        assert_eq!(parse_args(args("--help")).unwrap_err(), "");
    }
}
