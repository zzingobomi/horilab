//! 맨 서보 확인 (docs/decisions/0028-ur-style-joints.md의 시험과 측정, 0024의 만드는 순서 5).
//!
//! 로봇에 달지 않은 예비 서보 하나를 책상에 두고, 서보 프로그램이 위치 경계, 각도 한계, 위치 오프셋,
//! 보호, 끊김에서 어떻게 하는지 몇 분 안에 본다 (0027의 "실물로 미리 확인할 것"). 서보를 움직이는 쓰기는 모두 관절
//! 층([`Joints`])을 거친다. 서보 설정은 잠근 채 잠깐 바꾸고(전원을 끄면 사라진다) 그 단계 끝에 되돌린다.
//! 단계마다 결과를 화면에 보이고 기록(`meta.json`의 results, `samples.csv`, `events.csv`)에 남긴다.
//!
//! 끝나면 토크를 켠 채 두고, 프로그램을 끈 뒤, USB를 뺐다 꽂은 뒤, 서보 전원을 뺀 채, 다시 꽂은 뒤에
//! `check-state`로 상태만 읽는다 ([`state_report`]).

use std::io::{self, BufRead, Write};
use std::ops::RangeInclusive;
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

use horilab_servo::feetech::Port;
use horilab_servo::feetech::registers::{self as reg, Area, Fault, Register};
use serde_json::{Map, Value, json};

use crate::joints::{Enabled, Joints, Snapshot};
use crate::prepare::value_in;
use crate::record::{Log, dump_json};

/// 단계마다 돌아오는 가운데 자리 (서보 값).
pub const CENTER: i32 = 2048;
/// 맨 서보 확인에서 관절 층이 받는 목표 범위. 0~4095 밖의 목표에 서보가 어떻게 하는지 보려고 한 바퀴보다 넓다.
pub const BENCH_RANGE: (i32, i32) = (-1024, 5119);
/// 맨 서보 확인은 예비 서보 ID로만 한다 (0024: 예비 서보는 20번대). 관절에 단 서보를 크게 돌리지 않으려는 것이다.
pub const SPARE_IDS: RangeInclusive<u8> = 20..=29;

/// 과부하 보호를 볼 때의 토크 한도 (0.1 %). 손으로 붙잡을 수 있게 낮춘다: STS3215(12 V, 최대 30 kg·cm)에서
/// 약 1.5 kg·cm이고, 혼 둘레(반지름 10 mm)에서 약 1.5 kgf다.
pub const PUSH_TORQUE_LIMIT: i32 = 50;
/// 과부하 보호를 볼 때 잠깐 바꾸는 값: 4 %를 넘는 힘이 0.5초 이어지면 보호에 들어가고, 들어간 뒤에는 2 %를 낸다.
/// 토크 한도(5 %) > 과부하 기준(4 %) > 버티는 토크(2 %)라서, 보호에 들어간 뒤 힘이 줄었는지와 다시 풀리는지가
/// 부하 값에 드러난다.
const PUSH_TRIAL: [(Register, i32); 3] = [
    (reg::OVERLOAD_TORQUE, 4),
    (reg::PROTECTION_TIME, 50),
    (reg::HOLDING_TORQUE, 2),
];
/// 과부하 보호를 볼 때 미는 거리 (서보 값, 약 53°).
const PUSH: i32 = 600;
/// 전압 보호를 볼 때 잠깐 올리는 전압 하한 (0.1 V). 가변 전원을 조금만 내려도 보호에 들어가게 한다.
const TRIAL_MIN_VOLTAGE: i32 = 100;
/// 한 번 움직일 때 기다리는 가장 긴 시간 [s]. 300 step/s로 한 바퀴 가까이(3,700칸) 도는 데 약 13초다.
const MOVE_TIMEOUT: f64 = 30.0;
/// 여러 바퀴를 볼 때 기다리는 가장 긴 시간 [s]. 멈추지 않고 계속 돌면 이만큼 뒤에 그친다.
const MULTI_TURN_TIMEOUT: f64 = 15.0;
/// 한계 밖에서 토크를 켜 볼 때 잠깐 두는 각도 한계. 가운데(2048)가 아래 한계보다 252칸(약 22°) 밖이다.
const OUTSIDE_LIMITS: (i32, i32) = (2300, 2600);
/// 위치 오프셋을 볼 때 잠깐 더하는 값 (서보 값, 45°).
const OFFSET_SHIFT: i32 = 512;
/// 옛 0/4095 경계 자리를 지나가 볼 때, 그 자리에서 양쪽으로 떨어진 거리 (서보 값, 약 26°).
const OFFSET_CROSS: i32 = 300;
/// 한 바퀴의 서보 값 칸 수.
const TURN: i32 = reg::STEPS_PER_TURN as i32;

/// 단계 이름과 사람이 볼 설명. 이 순서로 한다.
pub const STEPS: [(&str, &str); 12] = [
    ("state", "상태 읽기 (움직이지 않는다)"),
    (
        "enable",
        "토크 켜기: 목표를 지금 위치로 맞추고 토크 20 %로 켠다. 이미 켜져 있으면 이어받는다",
    ),
    (
        "lock",
        "잠금: 잠근 채 P 게인을 8 올려 바로 바뀌는지 본다 (전원을 끄면 돌아오는지는 끝난 뒤 check-state로)",
    ),
    (
        "speed",
        "속도: 90° 갔다 오며 잰 속도와 서보가 알려 주는 속도를 견주고, 도는 쪽에 따라 전류 값을 본다",
    ),
    (
        "boundary",
        "0/4095 경계: 3900에서 200으로 보내 경계를 넘는지 보고, 4300과 -200으로도 보낸다",
    ),
    (
        "multi_turn",
        "여러 바퀴: 각도 한계를 0과 0으로 잠깐 바꾸고 4300과 -200으로 보낸다",
    ),
    (
        "angle_limit",
        "각도 한계: 한계를 1500~2600으로 잠깐 바꾸고 3000으로 보낸다",
    ),
    (
        "limit_enable",
        "한계 밖에서 토크 켜기: 토크를 끄고 한계를 2300~2600으로 잠깐 바꾼 뒤(지금 위치가 한계 밖) 토크를 켠다",
    ),
    (
        "offset",
        "위치 오프셋: 토크를 끄고 오프셋을 512 바꿔 읽는 위치가 옮겨지는지 보고, 옛 0/4095 경계 자리를 지나가 본다",
    ),
    (
        "overload_unload",
        "과부하 보호 1 (힘을 빼는 설정): 혼을 손으로 붙잡은 채 토크 5 %로 밀게 해서 보호에 들게 한다",
    ),
    (
        "overload_hold",
        "과부하 보호 2 (힘을 빼지 않는 설정): 같은 일을 반대쪽으로 한다",
    ),
    (
        "voltage",
        "전압 보호: 전압 하한을 10.0 V로 잠깐 올리고, 가변 전원을 10 V 밑으로 내렸다가 12.0 V로 올린다",
    ),
];

/// 상태 단계에서 화면에 보이는 EEPROM 칸 (기록에는 덤프 전체가 남는다).
const EEPROM_FIELDS: [Register; 11] = [
    reg::MIN_ANGLE_LIMIT,
    reg::MAX_ANGLE_LIMIT,
    reg::MAX_TEMPERATURE,
    reg::MAX_VOLTAGE,
    reg::MIN_VOLTAGE,
    reg::UNLOAD_CONDITION,
    reg::P_GAIN,
    reg::MODE,
    reg::HOLDING_TORQUE,
    reg::PROTECTION_TIME,
    reg::OVERLOAD_TORQUE,
];

/// 끝낸 뒤 끊김 확인의 순서.
const AFTER_TEXT: &str = "\n토크를 켠 채 끝냈다. 이어서 끊겼을 때를 본다. check-state는 상태만 읽고(서보는 움직이지 않는다) \
상황과 손으로 본 것을 묻는다:\n\
  1. 혼을 손으로 살짝 돌려 본 뒤 check-state: 프로그램이 꺼져도 버티는지\n\
  2. 어댑터의 USB를 뽑고 혼을 돌려 본 뒤, 다시 꽂고 check-state\n\
  3. 가변 전원의 + 플러그를 뽑고 혼을 돌려 본 뒤, 뺀 채로 check-state: USB 전기만으로 서보가 살아 있는지\n\
  4. + 플러그를 다시 꽂고 혼을 돌려 본 뒤 check-state: 토크가 꺼졌는지, 잠근 채 바꾼 값이 처음 값으로 돌아왔는지\n\
끝나면 + 플러그를 빼고 가변 전원을 끈다. 전압 손잡이에는 테이프를 다시 붙인다.";

/// 끊김 확인의 상황: 기록 이름과 사람이 볼 말.
pub const SITUATIONS: [(&str, &str); 4] = [
    ("program_ended", "프로그램을 끈 뒤"),
    ("usb_replugged", "USB를 뺐다 꽂은 뒤"),
    ("power_out", "+ 플러그를 뺀 채"),
    ("power_back", "+ 플러그를 다시 꽂은 뒤"),
];

/// 사람과 주고받는 곳.
pub trait Console {
    fn say(&mut self, text: &str);
    /// 질문을 보이고 한 줄을 받는다 (앞뒤 빈칸을 뺀다).
    fn ask(&mut self, question: &str) -> String;
    /// 안내를 보이고 Enter를 기다리기 시작한다. 기다리는 동안 [`Console::enter_pressed`]로 확인한다.
    fn begin_wait(&mut self, prompt: &str);
    fn enter_pressed(&mut self) -> bool;
    /// 기다리는 동안 같은 줄을 고쳐 쓰는 상태 표시.
    fn status(&mut self, text: &str);
}

/// 시계. 시험에서는 기다리지 않는 시계로 바꾼다.
pub trait Clock {
    /// 시작부터의 시간 [s]
    fn now(&self) -> f64;
    fn sleep(&mut self, seconds: f64);
}

/// 터미널 (표준 입출력). 표준 입력은 스레드 하나가 줄마다 읽어 넘긴다.
#[derive(Default)]
pub struct Terminal {
    lines: Option<Receiver<String>>,
}

impl Terminal {
    fn lines(&mut self) -> &Receiver<String> {
        self.lines.get_or_insert_with(|| {
            let (sender, receiver) = mpsc::channel();
            thread::spawn(move || {
                for line in io::stdin().lock().lines() {
                    let Ok(line) = line else { break };
                    if sender.send(line).is_err() {
                        break;
                    }
                }
            });
            receiver
        })
    }

    /// 앞서 눌러 둔 줄을 버린다. 다음 질문의 답으로 잘못 쓰이지 않게 한다.
    fn drain(&mut self) {
        while self.lines().try_recv().is_ok() {}
    }
}

impl Console for Terminal {
    fn say(&mut self, text: &str) {
        println!("{text}");
    }

    fn ask(&mut self, question: &str) -> String {
        self.drain();
        print!("{question}");
        let _ = io::stdout().flush();
        self.lines()
            .recv()
            .map(|line| line.trim().to_owned())
            .unwrap_or_default()
    }

    fn begin_wait(&mut self, prompt: &str) {
        self.drain();
        println!("{prompt}");
    }

    fn enter_pressed(&mut self) -> bool {
        !matches!(self.lines().try_recv(), Err(TryRecvError::Empty))
    }

    fn status(&mut self, text: &str) {
        print!("\r{text:<50}");
        let _ = io::stdout().flush();
    }
}

/// 실제 시계.
pub struct WallClock {
    start: Instant,
}

impl Default for WallClock {
    fn default() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Clock for WallClock {
    fn now(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    fn sleep(&mut self, seconds: f64) {
        thread::sleep(Duration::from_secs_f64(seconds.max(0.0)));
    }
}

/// 시간에 따라 읽은 상태: (읽기 시작부터의 시각 [s], 상태).
type Points = Vec<(f64, Snapshot)>;

/// 한 번 움직이며 읽은 것.
struct Trace {
    points: Points,
    /// 정한 시간 안에 멈추지 않았다
    timed_out: bool,
}

impl Trace {
    /// 멈춘 위치
    fn end(&self) -> Option<i32> {
        self.points.last().map(|(_, s)| s.position)
    }
}

/// 경계 너머로 보낸 움직임이 어느 쪽으로 갔나.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Crossing {
    /// 4095에서 0으로 넘어갔다 (가운데를 지나지 않았다)
    Wrapped,
    /// 가운데(1000~3000)를 지나 먼 길로 갔다
    LongWay,
    /// 샘플로는 가릴 수 없다 (너무 빨리 움직였다)
    Unknown,
}

impl Crossing {
    pub fn name(self) -> &'static str {
        match self {
            Self::Wrapped => "wrapped",
            Self::LongWay => "long_way",
            Self::Unknown => "unknown",
        }
    }
}

/// 3900 근처에서 200 근처로 간 위치들의 흐름에서 경계를 넘었는지 가린다.
pub fn classify_crossing(positions: &[i32]) -> Crossing {
    if positions.iter().any(|p| (1000..=3000).contains(p)) {
        return Crossing::LongWay;
    }
    let jumped = positions
        .windows(2)
        .any(|pair| pair[0] > 3000 && pair[1] < 1000);
    if jumped {
        Crossing::Wrapped
    } else {
        Crossing::Unknown
    }
}

/// 움직이는 동안의 (시각 [s], 위치, 서보가 알려 준 속도)에서 속도 단위를 어림한다.
/// 돌려주는 값: (잰 속도 [step/s], 알려 준 속도의 평균, 둘의 비). 움직인 샘플이 모자라면 없다.
pub fn speed_ratio(points: &[(f64, i32, i32)]) -> Option<(f64, f64, f64)> {
    let moving: Vec<&(f64, i32, i32)> = points.iter().filter(|(_, _, speed)| *speed != 0).collect();
    let (first, last) = (moving.first()?, moving.last()?);
    let duration = last.0 - first.0;
    if moving.len() < 3 || duration <= 0.0 {
        return None;
    }
    let observed = f64::from((last.1 - first.1).abs()) / duration;
    let reported = moving
        .iter()
        .map(|(_, _, speed)| f64::from(speed.abs()))
        .sum::<f64>()
        / moving.len() as f64;
    (reported > 0.0).then_some((observed, reported, observed / reported))
}

/// 비에서 속도 단위 [step/s]를 고른다 (레지스터 표: phase 비트 2에 따라 50 또는 1).
pub fn speed_unit(ratio: f64) -> Option<u32> {
    if (25.0..=100.0).contains(&ratio) {
        Some(50)
    } else if (0.5..=2.0).contains(&ratio) {
        Some(1)
    } else {
        None
    }
}

/// 도는 쪽이 다른 두 묶음의 현재 전류 값(부호 없이 읽은 값)에서 방향 비트를 가린다.
/// "bit15", "bit10"(한쪽에서만 그 비트가 켜짐), "none"(양쪽 다 같은 부호로 읽힘), "unknown"(값이 너무 작음).
pub fn current_sign(plus: &[i32], minus: &[i32]) -> &'static str {
    let has = |values: &[i32], bit: i32| values.iter().any(|v| v & bit != 0);
    let one_side = |bit: i32| has(plus, bit) != has(minus, bit);
    if one_side(0x8000) {
        "bit15"
    } else if one_side(0x400) {
        "bit10"
    } else if plus.iter().any(|v| *v > 3) && minus.iter().any(|v| *v > 3) {
        "none"
    } else {
        "unknown"
    }
}

fn snapshot_json(snapshot: &Snapshot) -> Value {
    let map: Map<String, Value> = snapshot
        .fields()
        .iter()
        .map(|(name, value)| ((*name).to_owned(), json!(value)))
        .collect();
    Value::Object(map)
}

fn registers_json(values: &[(Register, i32)]) -> Value {
    let map: Map<String, Value> = values
        .iter()
        .map(|(register, value)| (register.name.to_owned(), json!(value)))
        .collect();
    Value::Object(map)
}

fn volts(value: i32) -> f64 {
    f64::from(value) / 10.0
}

fn show(position: Option<i32>) -> String {
    position.map_or_else(|| "모름".to_owned(), |p| p.to_string())
}

fn on_off(on: bool) -> &'static str {
    if on { "켜짐" } else { "꺼짐" }
}

/// 사람이 볼 상태 한 줄.
pub fn describe(snapshot: &Snapshot) -> String {
    format!(
        "토크 {}, 위치 {}, 목표 {}, 토크 한도 {} (0.1 %), 전압 {:.1} V, 온도 {} °C, 상태 비트 {:#04x}{}",
        on_off(snapshot.torque_on()),
        snapshot.position,
        snapshot.goal_position,
        snapshot.torque_limit,
        volts(snapshot.voltage),
        snapshot.temperature,
        snapshot.status.0,
        if snapshot.moving {
            ", 움직이는 중"
        } else {
            ""
        },
    )
}

/// 덤프에서 화면에 보일 EEPROM 칸들.
fn eeprom_lines(memory: &[u8]) -> Vec<String> {
    EEPROM_FIELDS
        .iter()
        .filter_map(|&register| {
            value_in(memory, register)
                .map(|value| format!("  {} = {value} ({})", register.name, register.unit))
        })
        .collect()
}

/// 두 덤프 값(이름 → 값)에서 EEPROM 칸 가운데 다른 것.
fn eeprom_differences(before: &Value, after: &Value) -> Vec<String> {
    reg::ALL
        .iter()
        .filter(|register| register.area == Area::Eeprom)
        .filter_map(|register| {
            let (a, b) = (&before[register.name], &after[register.name]);
            (!a.is_null() && a != b).then(|| format!("{} {a} → {b}", register.name))
        })
        .collect()
}

/// 가운데 값.
fn median(values: &[i32]) -> Option<i32> {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted.get(sorted.len() / 2).copied()
}

/// 처음과 끝의 위치가 20칸(약 1.8°) 넘게 다른가.
fn moved(points: &[(f64, Snapshot)]) -> bool {
    match (points.first(), points.last()) {
        (Some((_, a)), Some((_, b))) => (b.position - a.position).abs() > 20,
        _ => false,
    }
}

fn first_time(points: &[(f64, Snapshot)], condition: impl Fn(&Snapshot) -> bool) -> Option<f64> {
    points.iter().find(|(_, s)| condition(s)).map(|(t, _)| *t)
}

/// 맨 서보 확인 하나.
pub struct Check<P: Port, C: Console, K: Clock> {
    joints: Joints<P>,
    id: u8,
    console: C,
    clock: K,
    log: Log,
    /// 단계 이름 → 결과. 끝나면 전류 부호의 결과("current")도 있다.
    pub results: Map<String, Value>,
    /// 도는 쪽(+, -)마다 본 현재 전류 값
    currents: (Vec<i32>, Vec<i32>),
}

impl<P: Port, C: Console, K: Clock> Check<P, C, K> {
    pub fn new(joints: Joints<P>, id: u8, console: C, clock: K, log: Log) -> Self {
        Self {
            joints,
            id,
            console,
            clock,
            log,
            results: Map::new(),
            currents: (Vec::new(), Vec::new()),
        }
    }

    /// 단계를 차례로 한다. 단계마다 `after_step`을 불러 기록을 남기게 한다. 단계가 실패하면 결과에 오류를
    /// 남기고 다음 단계로 간다. 통신이 끊겨 멈췄으면 사람에게 묻고, 그만두면 거기서 끝낸다.
    pub fn run(&mut self, after_step: &mut dyn FnMut(&Map<String, Value>)) -> Result<(), String> {
        for (index, &(name, text)) in STEPS.iter().enumerate() {
            self.console
                .say(&format!("\n[{}/{}] {text}", index + 1, STEPS.len()));
            let result = match name {
                "state" => self.step_state(),
                "enable" => self.step_enable(),
                "lock" => self.step_lock(),
                "speed" => self.step_speed(),
                "boundary" => self.step_boundary(),
                "multi_turn" => self.step_multi_turn(),
                "angle_limit" => self.step_angle_limit(),
                "limit_enable" => self.step_limit_enable(),
                "offset" => self.step_offset(),
                "overload_unload" => self.step_overload(name, true, PUSH),
                "overload_hold" => self.step_overload(name, false, -PUSH),
                _ => self.step_voltage(),
            };
            let value = match result {
                Ok(value) => value,
                Err(error) => {
                    self.console.say(&format!("이 단계는 실패했다: {error}"));
                    self.event(name, &format!("실패: {error}"))?;
                    json!({ "error": error })
                }
            };
            self.results.insert(name.to_owned(), value);
            after_step(&self.results);
            if let Some(reason) = self.joints.stopped().map(str::to_owned) {
                let answer = self.console.ask(&format!(
                    "멈췄다: {reason}. 선과 전원을 확인하고 Enter를 누르면 이어서 한다 (q: 그만둠): "
                ));
                if answer == "q" {
                    return Err(format!("{name} 단계 뒤에 그만뒀다: {reason}"));
                }
                self.joints.resume();
            }
        }
        self.finish()
    }

    fn event(&mut self, step: &str, text: &str) -> Result<(), String> {
        let t = self.clock.now();
        self.log.event(t, step, text)
    }

    fn sample(&mut self, step: &str) -> Result<Snapshot, String> {
        let snapshot = self.joints.snapshot(self.id)?;
        let values: Vec<i32> = snapshot.fields().iter().map(|(_, v)| *v).collect();
        let t = self.clock.now();
        self.log.sample(t, step, &values)?;
        Ok(snapshot)
    }

    /// 토크가 꺼져 있으면 다시 켠다 (앞 단계의 보호가 힘을 뺐을 때).
    fn ensure_torque(&mut self, step: &str) -> Result<(), String> {
        if self.joints.enable(self.id)? == Enabled::TurnedOn {
            self.event(step, "토크가 꺼져 있어 다시 켰다")?;
        }
        Ok(())
    }

    /// 보호를 본 뒤 토크를 껐다가 시작 절차로 다시 켠다. 토크 스위치가 켜진 채 힘을 뺀 상태가 남아도
    /// 지우고, 상한을 다시 쓴다. 그전에 토크가 꺼져 있었는지를 돌려준다.
    fn reset_torque(&mut self, step: &str) -> Result<bool, String> {
        let was_off = !self.joints.snapshot(self.id)?.torque_on();
        self.joints.disable(self.id)?;
        self.joints.enable(self.id)?;
        let text = if was_off {
            "토크가 꺼져 있었다. 다시 켰다"
        } else {
            "토크를 껐다 다시 켰다"
        };
        self.event(step, text)?;
        Ok(was_off)
    }

    /// 목표를 쓰고 멈출 때까지 읽는다.
    fn go(&mut self, step: &str, goal: i32, timeout: f64) -> Result<Trace, String> {
        self.joints.move_raw(self.id, goal)?;
        self.trace(step, goal, timeout)
    }

    /// 멈출 때까지 20 ms마다 읽는다. 목표에 닿아 움직임이 끝났거나, 위치가 0.5초 동안 2칸 넘게 바뀌지
    /// 않으면 멈춘 것으로 본다 (한계에 막혀 목표에 못 닿을 때).
    fn trace(&mut self, step: &str, goal: i32, timeout: f64) -> Result<Trace, String> {
        let start = self.clock.now();
        let mut points = Vec::new();
        let mut anchor: Option<(f64, i32)> = None;
        loop {
            let snapshot = self.sample(step)?;
            let t = self.clock.now() - start;
            points.push((t, snapshot));
            if (snapshot.position - goal).abs() <= 3 && !snapshot.moving {
                break;
            }
            match anchor {
                Some((since, position)) if (snapshot.position - position).abs() <= 2 => {
                    if t - since >= 0.5 {
                        break;
                    }
                }
                _ => anchor = Some((t, snapshot.position)),
            }
            if t >= timeout {
                self.event(
                    step,
                    &format!("{goal}로 보냈는데 {timeout} s 안에 멈추지 않았다"),
                )?;
                return Ok(Trace {
                    points,
                    timed_out: true,
                });
            }
            self.clock.sleep(0.02);
        }
        Ok(Trace {
            points,
            timed_out: false,
        })
    }

    /// 정한 시간 동안 50 ms마다 읽는다.
    fn watch(&mut self, step: &str, seconds: f64) -> Result<Points, String> {
        let start = self.clock.now();
        let mut points = Vec::new();
        while self.clock.now() - start < seconds {
            let snapshot = self.sample(step)?;
            points.push((self.clock.now() - start, snapshot));
            self.clock.sleep(0.05);
        }
        Ok(points)
    }

    fn step_state(&mut self) -> Result<Value, String> {
        let snapshot = self.sample("state")?;
        let memory = self.joints.dump(self.id)?;
        self.console.say(&describe(&snapshot));
        for line in eeprom_lines(&memory) {
            self.console.say(&line);
        }
        Ok(json!({ "snapshot": snapshot_json(&snapshot), "dump": dump_json(&memory) }))
    }

    fn step_enable(&mut self) -> Result<Value, String> {
        let enabled = self.joints.enable(self.id)?;
        let text = match enabled {
            Enabled::TurnedOn => "목표를 지금 위치로 맞추고 토크를 켰다",
            Enabled::AlreadyOn => {
                "토크가 이미 켜져 있어 남은 목표와 설정을 그대로 이어받았다 (아무것도 쓰지 않았다)"
            }
        };
        self.console.say(text);
        self.event("enable", text)?;
        let snapshot = self.sample("enable")?;
        self.console.say(&describe(&snapshot));
        let answer = self
            .console
            .ask("혼을 손가락으로 살짝 돌려 보세요. 버티면 y, 그냥 돌아가면 n: ");
        let enabled = match enabled {
            Enabled::TurnedOn => "turned_on",
            Enabled::AlreadyOn => "already_on",
        };
        Ok(json!({
            "enabled": enabled,
            "snapshot": snapshot_json(&snapshot),
            "holds_by_hand": answer,
        }))
    }

    fn step_lock(&mut self) -> Result<Value, String> {
        let before = self.read_one(reg::P_GAIN)?;
        let trial = (before + 8).min(254);
        self.joints.trial(self.id, &[(reg::P_GAIN, trial)])?;
        let read = self.read_one(reg::P_GAIN)?;
        let applied = read == trial;
        self.console.say(&format!(
            "P 게인 {before} → 잠근 채 {trial}를 써서 {read}로 읽었다 ({}). 전원을 껐다 켜면 {before}로 돌아오는지는 끝난 뒤 check-state로 본다",
            if applied {
                "바로 바뀌었다"
            } else {
                "바뀌지 않았다"
            }
        ));
        Ok(json!({ "p_before": before, "p_trial": trial, "p_read": read, "applied": applied }))
    }

    fn read_one(&mut self, register: Register) -> Result<i32, String> {
        Ok(self.joints.read(self.id, &[register])?[0].1)
    }

    fn step_speed(&mut self) -> Result<Value, String> {
        self.ensure_torque("speed")?;
        self.go("speed", CENTER, MOVE_TIMEOUT)?;
        let out = self.go("speed", CENTER + 1024, MOVE_TIMEOUT)?;
        let back = self.go("speed", CENTER, MOVE_TIMEOUT)?;
        let moving = |trace: &Trace| -> Vec<i32> {
            trace
                .points
                .iter()
                .filter(|(_, s)| s.speed != 0)
                .map(|(_, s)| s.current)
                .collect()
        };
        let (plus, minus) = (moving(&out), moving(&back));
        let current = json!({ "plus_max": plus.iter().max(), "minus_max": minus.iter().max() });
        self.currents.0.extend(plus);
        self.currents.1.extend(minus);
        let points: Vec<(f64, i32, i32)> = out
            .points
            .iter()
            .map(|(t, s)| (*t, s.position, s.speed))
            .collect();
        let Some((observed, reported, ratio)) = speed_ratio(&points) else {
            self.console
                .say("움직이는 동안의 샘플이 모자라 속도 단위를 가리지 못했다");
            return Ok(json!({
                "unit_step_per_s": Value::Null,
                "samples": points.len(),
                "current": current,
            }));
        };
        let unit = speed_unit(ratio);
        self.console.say(&format!(
            "잰 속도 {observed:.0} step/s, 서보가 알려 준 속도 {reported:.1} → 비 {ratio:.1}, 단위 {}",
            unit.map_or_else(|| "모름".to_owned(), |u| format!("{u} step/s"))
        ));
        Ok(json!({
            "observed_step_per_s": observed,
            "reported_mean": reported,
            "ratio": ratio,
            "unit_step_per_s": unit,
            "goal_speed": self.joints.caps().speed,
            "current": current,
        }))
    }

    fn step_boundary(&mut self) -> Result<Value, String> {
        self.ensure_torque("boundary")?;
        self.go("boundary", 3900, MOVE_TIMEOUT)?;
        let crossing = self.go("boundary", 200, MOVE_TIMEOUT)?;
        let positions: Vec<i32> = crossing.points.iter().map(|(_, s)| s.position).collect();
        let kind = classify_crossing(&positions);
        self.go("boundary", CENTER, MOVE_TIMEOUT)?;
        let high = self.go("boundary", 4300, MOVE_TIMEOUT)?.end();
        self.go("boundary", CENTER, MOVE_TIMEOUT)?;
        let low = self.go("boundary", -200, MOVE_TIMEOUT)?.end();
        self.go("boundary", CENTER, MOVE_TIMEOUT)?;
        let text = match kind {
            Crossing::Wrapped => "3900 → 200: 4095에서 0으로 경계를 넘어갔다",
            Crossing::LongWay => "3900 → 200: 경계를 넘지 않고 가운데를 지나 먼 길로 갔다",
            Crossing::Unknown => "3900 → 200: 샘플로는 어느 길로 갔는지 가리지 못했다",
        };
        self.console.say(text);
        self.console.say(&format!(
            "4300으로 보냈더니 {}, -200으로 보냈더니 {}에서 멈췄다",
            show(high),
            show(low)
        ));
        self.event("boundary", text)?;
        Ok(json!({
            "crossing": kind.name(),
            "end_after_200": crossing.end(),
            "end_after_4300": high,
            "end_after_minus_200": low,
        }))
    }

    fn step_multi_turn(&mut self) -> Result<Value, String> {
        self.ensure_torque("multi_turn")?;
        let original = self
            .joints
            .read(self.id, &[reg::MIN_ANGLE_LIMIT, reg::MAX_ANGLE_LIMIT])?;
        self.go("multi_turn", CENTER, MOVE_TIMEOUT)?;
        self.joints.trial(
            self.id,
            &[(reg::MIN_ANGLE_LIMIT, 0), (reg::MAX_ANGLE_LIMIT, 0)],
        )?;
        let moves = self.multi_turn_moves();
        // 결과와 상관없이 한계를 되돌리고 가운데로 보낸다
        self.joints.trial(self.id, &original)?;
        self.go("multi_turn", CENTER, MOVE_TIMEOUT)?;
        let mut value = moves?;
        let near = |key: &str, goal: i64| {
            value[key]
                .as_i64()
                .is_some_and(|end| (end - goal).abs() <= 10)
        };
        let (beyond, text) = if value["kept_turning"] == true {
            (false, "멈추지 않고 계속 돌았다 (위치 목표를 따르지 않았다)")
        } else if near("end_after_4300", 4300) && near("end_after_minus_200", -200) {
            (
                true,
                "4300과 -200까지 갔다: 각도 한계가 0과 0이면 한 바퀴 밖으로 간다 (여러 바퀴)",
            )
        } else {
            (false, "한 바퀴 밖으로 가지 않았다")
        };
        self.console.say(&format!(
            "{text}. 4300으로 보냈더니 {}, -200으로 보냈더니 {}",
            value["end_after_4300"], value["end_after_minus_200"]
        ));
        self.event("multi_turn", text)?;
        value["beyond_one_turn"] = json!(beyond);
        value["restored"] = registers_json(&original);
        Ok(value)
    }

    /// 각도 한계를 0과 0으로 둔 채 한 바퀴 밖으로 보낸다. 멈추지 않고 계속 돌면 거기서 그친다.
    fn multi_turn_moves(&mut self) -> Result<Value, String> {
        let high = self.go("multi_turn", 4300, MULTI_TURN_TIMEOUT)?;
        if high.timed_out {
            return Ok(json!({ "kept_turning": true, "end_after_4300": high.end() }));
        }
        self.go("multi_turn", CENTER, MULTI_TURN_TIMEOUT)?;
        let low = self.go("multi_turn", -200, MULTI_TURN_TIMEOUT)?;
        if !low.timed_out {
            self.go("multi_turn", CENTER, MULTI_TURN_TIMEOUT)?;
        }
        Ok(json!({
            "kept_turning": low.timed_out,
            "end_after_4300": high.end(),
            "end_after_minus_200": low.end(),
        }))
    }

    fn step_angle_limit(&mut self) -> Result<Value, String> {
        self.ensure_torque("angle_limit")?;
        let original = self
            .joints
            .read(self.id, &[reg::MIN_ANGLE_LIMIT, reg::MAX_ANGLE_LIMIT])?;
        self.go("angle_limit", CENTER, MOVE_TIMEOUT)?;
        self.joints.trial(
            self.id,
            &[(reg::MIN_ANGLE_LIMIT, 1500), (reg::MAX_ANGLE_LIMIT, 2600)],
        )?;
        let result = self.go("angle_limit", 3000, MOVE_TIMEOUT);
        let back = self.go("angle_limit", CENTER, MOVE_TIMEOUT);
        // 결과와 상관없이 되돌린다
        self.joints.trial(self.id, &original)?;
        let end = result?.end();
        back?;
        self.console.say(&format!(
            "한계 1500~2600에서 3000으로 보냈더니 {}에서 멈췄다. 한계를 {}~{}로 되돌렸다",
            show(end),
            original[0].1,
            original[1].1
        ));
        Ok(json!({
            "limits": [1500, 2600],
            "goal": 3000,
            "end": end,
            "restored": registers_json(&original),
        }))
    }

    /// 지금 위치가 각도 한계 밖일 때 토크를 켜면 서보가 어떻게 하는지 본다. 기계 멈춤이 프로그램 한계보다
    /// 조금 밖에 있어서(0029), 팔을 손으로 멈춤까지 밀어 둔 채 토크를 켜면 생기는 상황이다.
    fn step_limit_enable(&mut self) -> Result<Value, String> {
        self.ensure_torque("limit_enable")?;
        let original = self
            .joints
            .read(self.id, &[reg::MIN_ANGLE_LIMIT, reg::MAX_ANGLE_LIMIT])?;
        self.go("limit_enable", CENTER, MOVE_TIMEOUT)?;
        self.joints.disable(self.id)?;
        self.event("limit_enable", "토크를 껐다")?;
        let tried = self.enable_outside_limits();
        // 결과와 상관없이 한계를 되돌리고, 토크를 껐다 켜서 가운데로 보낸다
        self.joints.trial(self.id, &original)?;
        self.reset_torque("limit_enable")?;
        self.go("limit_enable", CENTER, MOVE_TIMEOUT)?;
        let mut value = tried?;
        value["restored"] = registers_json(&original);
        Ok(value)
    }

    /// 토크가 꺼진 채 한계를 지금 위치 밖으로 두고, 시작 절차(목표를 지금 위치로, 상한, 토크 켜기)를 한다.
    fn enable_outside_limits(&mut self) -> Result<Value, String> {
        let (low, high) = OUTSIDE_LIMITS;
        let before = self.sample("limit_enable")?;
        self.joints.trial(
            self.id,
            &[(reg::MIN_ANGLE_LIMIT, low), (reg::MAX_ANGLE_LIMIT, high)],
        )?;
        let enable_error = self.joints.enable(self.id).err();
        let points = self.watch("limit_enable", 2.0)?;
        let end = points.last().map_or(before, |(_, s)| *s);
        let moved_by = end.position - before.position;
        let text = if !end.torque_on() {
            "토크가 켜지지 않았다"
        } else if (end.position - low).abs() <= 30 {
            "서보가 한계까지 스스로 움직였다"
        } else if moved_by.abs() <= 20 {
            "움직이지 않고 그 자리에서 버틴다"
        } else {
            "한계가 아닌 자리로 움직였다"
        };
        self.console.say(&format!(
            "위치 {}에서 한계를 {low}~{high}로 두고 토크를 켰더니 {}에서 멈췄다 (서보에 남은 목표 {}): {text}",
            before.position, end.position, end.goal_position
        ));
        self.event("limit_enable", text)?;
        Ok(json!({
            "limits": [low, high],
            "position_before": before.position,
            "enable_error": enable_error,
            "goal_after_enable": points.first().map(|(_, s)| s.goal_position),
            "end": end.position,
            "moved_by": moved_by,
            "moved_at_s": first_time(&points, |s| (s.position - before.position).abs() > 20),
            "torque_on": end.torque_on(),
            "status": end.status.0,
        }))
    }

    /// 위치 오프셋(31)이 읽는 위치와 0/4095 경계를 같이 옮기는지 본다. 옮긴다면 혼 톱니를 맞추지 않고
    /// 오프셋으로 영점을 잡을 수 있다 (0024의 영점). 읽는 위치가 바뀔 때 혼이 따라 돌지 않게 토크를 끄고 바꾼다.
    fn step_offset(&mut self) -> Result<Value, String> {
        self.ensure_torque("offset")?;
        let original = self.read_one(reg::POSITION_OFFSET)?;
        self.go("offset", CENTER, MOVE_TIMEOUT)?;
        self.joints.disable(self.id)?;
        self.event("offset", "토크를 껐다")?;
        let tried = self.offset_moves(original);
        // 결과와 상관없이 되돌린다: 토크를 끄고 오프셋을 되돌린 뒤, 다시 켜서 가운데로 보낸다
        self.joints.disable(self.id)?;
        self.joints
            .trial(self.id, &[(reg::POSITION_OFFSET, original)])?;
        let restored = self.sample("offset")?.position;
        self.joints.enable(self.id)?;
        self.go("offset", CENTER, MOVE_TIMEOUT)?;
        let mut value = tried?;
        value["offset_restored"] = json!(original);
        value["position_after_restore"] = json!(restored);
        Ok(value)
    }

    /// 토크가 꺼진 채 오프셋을 바꿔 읽는 위치가 옮겨지는지 보고, 옮겨졌으면 토크를 켜서 옛 0/4095 경계
    /// 자리를 가로질러 본다.
    fn offset_moves(&mut self, original: i32) -> Result<Value, String> {
        let trial = if original + OFFSET_SHIFT <= 2047 {
            original + OFFSET_SHIFT
        } else {
            original - OFFSET_SHIFT
        };
        let before = self.sample("offset")?.position;
        self.joints
            .trial(self.id, &[(reg::POSITION_OFFSET, trial)])?;
        let after = self.sample("offset")?.position;
        let shift = after - before;
        let mut value = json!({
            "offset_before": original,
            "offset_trial": trial,
            "position_before": before,
            "position_after": after,
            "shift": shift,
        });
        let applied = (shift.abs() - OFFSET_SHIFT).abs() <= 20;
        value["applied"] = json!(applied);
        if !applied {
            let text = format!(
                "오프셋을 {original} → {trial}로 잠근 채 바꿨는데 읽는 위치가 {before} → {after}다: \
                 그만큼 옮겨지지 않았다"
            );
            self.console.say(&text);
            self.event("offset", &text)?;
            return Ok(value);
        }
        self.console.say(&format!(
            "오프셋을 {original} → {trial}로 잠근 채 바꾸자 읽는 위치가 {before} → {after}로 옮겨졌다 ({shift:+})"
        ));
        // 오프셋을 바꾸기 전의 0/4095 경계가 지금 읽히는 자리. 지금 위치 쪽에서 다가가 가로지른다
        let seam = shift.rem_euclid(TURN);
        let (near, far) = if after < seam {
            (seam - OFFSET_CROSS, seam + OFFSET_CROSS)
        } else {
            (seam + OFFSET_CROSS, seam - OFFSET_CROSS)
        };
        value["old_boundary_at"] = json!(seam);
        if !(0..TURN).contains(&near) || !(0..TURN).contains(&far) {
            let text = format!("옛 경계 자리({seam})가 끝에 가까워 가로질러 보지 않았다");
            self.console.say(&text);
            self.event("offset", &text)?;
            return Ok(value);
        }
        self.joints.enable(self.id)?;
        // 목표에서 20칸(약 1.8°) 안에 멈추면 닿은 것으로 본다 (토크 20 %라 조금 못 미쳐 설 수 있다)
        let close = |end: Option<i32>, goal: i32| end.is_some_and(|end| (end - goal).abs() <= 20);
        let end_near = self.go("offset", near, MOVE_TIMEOUT)?.end();
        let at_near = close(end_near, near);
        let crossing = self.go("offset", far, MOVE_TIMEOUT)?;
        let end_far = crossing.end();
        let reached = !crossing.timed_out && close(end_far, far);
        // 옛 경계 자리에서 1000칸 넘게 떨어진 곳을 지났으면 가로지르지 않고 반대쪽으로 돈 것이다
        let long_way = at_near
            && crossing
                .points
                .iter()
                .any(|(_, s)| (s.position - seam).abs() > 1000);
        let boundary_moved = at_near && reached && !long_way;
        let text = if boundary_moved {
            format!(
                "옛 0/4095 경계 자리({seam})를 {near} → {far}로 그냥 지나갔다: 경계도 오프셋과 같이 옮겨진다"
            )
        } else if !at_near {
            format!(
                "옛 0/4095 경계 자리({seam}) 앞({near})까지 가지 못했다: {}에서 멈췄다",
                show(end_near)
            )
        } else {
            format!(
                "옛 0/4095 경계 자리({seam})를 지나가지 못했다: {near}에서 {far}로 보냈더니 {}에서 멈췄다{}",
                show(end_far),
                if long_way {
                    " (반대쪽으로 돌았다)"
                } else {
                    ""
                }
            )
        };
        self.console.say(&text);
        self.event("offset", &text)?;
        value["near"] = json!(near);
        value["far"] = json!(far);
        value["end_near"] = json!(end_near);
        value["end_far"] = json!(end_far);
        value["at_near"] = json!(at_near);
        value["reached"] = json!(reached);
        value["long_way"] = json!(long_way);
        value["boundary_moved"] = json!(boundary_moved);
        Ok(value)
    }

    /// 혼을 손으로 붙잡은 채 밀게 해서 과부하 보호에 들게 한다. `unload`면 과부하에 힘을 빼는 조건을 켜고,
    /// 아니면 끈다. `push`는 미는 쪽과 거리다.
    fn step_overload(&mut self, step: &str, unload: bool, push: i32) -> Result<Value, String> {
        let answer = self.console.ask(
            "한 손으로 서보 몸통을 잡고, 다른 손 엄지와 검지로 혼 둘레를 꽉 잡으세요. 서보가 혼을 돌리려 해도 \
             놓지 마세요 (혼 둘레에서 약 1.5 kgf). 잡았으면 Enter (건너뛰려면 s): ",
        );
        if answer == "s" {
            return Ok(json!({ "skipped": true }));
        }
        self.ensure_torque(step)?;
        let original = self.joints.read(
            self.id,
            &[
                reg::UNLOAD_CONDITION,
                reg::OVERLOAD_TORQUE,
                reg::PROTECTION_TIME,
                reg::HOLDING_TORQUE,
            ],
        )?;
        let overload_bit = i32::from(Fault::OVERLOAD.0);
        let condition = if unload {
            original[0].1 | overload_bit
        } else {
            original[0].1 & !overload_bit
        };
        self.joints.limit_torque(self.id, PUSH_TORQUE_LIMIT)?;
        let mut trial = vec![(reg::UNLOAD_CONDITION, condition)];
        trial.extend(PUSH_TRIAL);
        self.joints.trial(self.id, &trial)?;
        let pushed = self.push_and_release(step, push);
        // 결과와 상관없이 되돌린다
        self.joints.trial(self.id, &original)?;
        let torque_was_off = self.reset_torque(step)?;
        let back = self.go(step, CENTER, MOVE_TIMEOUT)?.end();
        let (held, released) = pushed?;

        let overload_at = first_time(&held, |s| s.status.contains(Fault::OVERLOAD));
        let torque_off_at = first_time(&held, |s| !s.torque_on());
        let loads = |after: bool| -> Vec<i32> {
            held.iter()
                .filter(|(t, _)| overload_at.is_some_and(|at| *t >= at) == after)
                .map(|(_, s)| s.load.abs())
                .collect()
        };
        let (load_before, load_after) = (loads(false), loads(true));
        let limit_after = held
            .iter()
            .filter(|(t, _)| overload_at.is_some_and(|at| *t >= at))
            .map(|(_, s)| s.torque_limit)
            .min();
        let currents: Vec<i32> = held.iter().map(|(_, s)| s.current).collect();
        let current_max = currents.iter().max().copied();
        if push > 0 {
            self.currents.0.extend(currents);
        } else {
            self.currents.1.extend(currents);
        }
        let end = released.last().map(|(_, s)| *s);
        let bit_after_release = end.is_some_and(|s| s.status.contains(Fault::OVERLOAD));
        let torque_after_release = end.is_some_and(|s| s.torque_on());
        let percent = |load: Option<i32>| {
            load.map_or_else(
                || "-".to_owned(),
                |l| format!("{:.1} %", f64::from(l) / 10.0),
            )
        };
        self.console.say(&format!(
            "과부하 비트: {}, 토크: {}, 붙잡은 동안 부하 {} → 보호 뒤 {}, 놓은 뒤 {}, 과부하 비트 {}{}",
            overload_at.map_or_else(|| "안 켜짐".to_owned(), |t| format!("{t:.2} s에 켜짐")),
            torque_off_at.map_or_else(|| "꺼지지 않음".to_owned(), |t| format!("{t:.2} s에 꺼짐")),
            percent(load_before.iter().max().copied()),
            percent(median(&load_after)),
            if moved(&released) {
                "저절로 움직였다"
            } else {
                "움직이지 않았다"
            },
            on_off(bit_after_release),
            if torque_was_off {
                ". 토크가 꺼져 있어 다시 켰다"
            } else {
                ""
            },
        ));
        Ok(json!({
            "unload_condition": condition,
            "torque_limit": PUSH_TORQUE_LIMIT,
            "trial": registers_json(&PUSH_TRIAL),
            "original": registers_json(&original),
            "overload_bit_at_s": overload_at,
            "torque_off_at_s": torque_off_at,
            "load_before_max": load_before.iter().max(),
            "load_after_median": median(&load_after),
            "torque_limit_after_min": limit_after,
            "moved_while_held": moved(&held),
            "moved_after_release": moved(&released),
            "overload_bit_after_release": bit_after_release,
            "torque_on_after_release": torque_after_release,
            "torque_off_before_reset": torque_was_off,
            "end_after_reset": back,
            "current_while_held_max": current_max,
        }))
    }

    /// 밀게 하고 3초 보고, 사람이 놓은 뒤 2초 본다.
    fn push_and_release(&mut self, step: &str, push: i32) -> Result<(Points, Points), String> {
        let start = self.joints.snapshot(self.id)?.position;
        self.joints.move_raw(self.id, start + push)?;
        let held = self.watch(step, 3.0)?;
        let _ = self.console.ask("이제 혼을 놓고 Enter: ");
        let released = self.watch(step, 2.0)?;
        Ok((held, released))
    }

    fn step_voltage(&mut self) -> Result<Value, String> {
        let answer = self
            .console
            .ask("가변 전원의 전압 손잡이에 붙인 테이프를 떼세요. 준비되면 Enter (건너뛰려면 s): ");
        if answer == "s" {
            return Ok(json!({ "skipped": true }));
        }
        self.ensure_torque("voltage")?;
        self.go("voltage", CENTER, MOVE_TIMEOUT)?;
        let original = self
            .joints
            .read(self.id, &[reg::MIN_VOLTAGE, reg::UNLOAD_CONDITION])?;
        let condition = original[1].1 | i32::from(Fault::VOLTAGE.0);
        self.joints.trial(
            self.id,
            &[
                (reg::MIN_VOLTAGE, TRIAL_MIN_VOLTAGE),
                (reg::UNLOAD_CONDITION, condition),
            ],
        )?;
        let sweep = self.voltage_sweep();
        // 결과와 상관없이 되돌린다. 전압을 내린 동안 응답이 끊겼을 수 있고, 사람이 12 V로 되돌렸다
        self.joints.resume();
        self.joints.trial(self.id, &original)?;
        self.reset_torque("voltage")?;
        self.go("voltage", CENTER, MOVE_TIMEOUT)?;
        let mut value = sweep?;
        value["unload_condition"] = json!(condition);
        value["original"] = registers_json(&original);
        Ok(value)
    }

    /// 사람이 전압을 내렸다 올리는 동안 0.1초마다 읽어 보인다. Enter를 누르면 끝난다.
    fn voltage_sweep(&mut self) -> Result<Value, String> {
        self.console.begin_wait(
            "손잡이를 천천히 왼쪽으로 돌려 전압을 내리세요 (9.0 V 밑으로는 내리지 않는다). \
             \"전압 비트가 켜졌다\"가 나오면 오른쪽으로 돌려 12.0 V로 올리고 Enter:",
        );
        let start = self.clock.now();
        let mut lowest: Option<i32> = None;
        let mut bit_on_at: Option<i32> = None;
        let mut bit_off_at: Option<i32> = None;
        let mut torque_off_at: Option<i32> = None;
        let mut no_reply = 0u32;
        while !self.console.enter_pressed() {
            if self.clock.now() - start >= 300.0 {
                self.event("voltage", "5분 안에 Enter가 없었다")?;
                break;
            }
            match self.sample("voltage") {
                Ok(s) => {
                    let bit = s.status.contains(Fault::VOLTAGE);
                    lowest = Some(lowest.map_or(s.voltage, |v| v.min(s.voltage)));
                    if bit && bit_on_at.is_none() {
                        bit_on_at = Some(s.voltage);
                        self.console.say(&format!(
                            "\n전압 비트가 켜졌다 ({:.1} V). 이제 12.0 V로 올리고 Enter",
                            volts(s.voltage)
                        ));
                    }
                    if !bit && bit_on_at.is_some() && bit_off_at.is_none() {
                        bit_off_at = Some(s.voltage);
                    }
                    if !s.torque_on() && torque_off_at.is_none() {
                        torque_off_at = Some(s.voltage);
                    }
                    self.console.status(&format!(
                        "전압 {:.1} V, 전압 비트 {}, 토크 {}",
                        volts(s.voltage),
                        on_off(bit),
                        on_off(s.torque_on())
                    ));
                }
                Err(_) => no_reply += 1,
            }
            self.clock.sleep(0.1);
        }
        self.console.say("");
        self.joints.resume();
        let end = self.sample("voltage")?;
        let at = |value: Option<i32>| value.map(volts);
        let show_v = |value: Option<i32>| {
            value.map_or_else(|| "없음".to_owned(), |v| format!("{:.1} V", volts(v)))
        };
        self.console.say(&format!(
            "가장 낮은 전압 {}, 전압 비트가 켜진 전압 {}, 꺼진 전압 {}, 토크가 꺼진 전압 {}, 응답 없음 {no_reply}번. \
             12 V로 돌아온 뒤 토크 {}. 손잡이에 테이프를 다시 붙이세요",
            show_v(lowest),
            show_v(bit_on_at),
            show_v(bit_off_at),
            show_v(torque_off_at),
            on_off(end.torque_on()),
        ));
        Ok(json!({
            "trial_min_voltage_v": volts(TRIAL_MIN_VOLTAGE),
            "lowest_v": at(lowest),
            "voltage_bit_on_at_v": at(bit_on_at),
            "voltage_bit_off_at_v": at(bit_off_at),
            "torque_off_at_v": at(torque_off_at),
            "no_reply": no_reply,
            "end_voltage_v": volts(end.voltage),
            "torque_on_at_end": end.torque_on(),
            "voltage_bit_at_end": end.status.contains(Fault::VOLTAGE),
        }))
    }

    /// 전류 부호를 정리하고, 토크를 켠 채 가운데에 두고, 끊김 확인 순서를 알린다.
    fn finish(&mut self) -> Result<(), String> {
        let (plus, minus) = &self.currents;
        let sign = current_sign(plus, minus);
        let text = match sign {
            "bit15" => "현재 전류: 도는 쪽에 따라 비트 15가 켜진다 (부호가 있다)",
            "bit10" => "현재 전류: 도는 쪽에 따라 비트 10이 켜진다 (부호가 있다)",
            "none" => "현재 전류: 양쪽 모두 같은 부호로 읽힌다 (부호가 없다)",
            _ => "현재 전류: 값이 작아 부호를 가리지 못했다",
        };
        let current = json!({
            "sign": sign,
            "plus_max": plus.iter().max(),
            "minus_max": minus.iter().max(),
        });
        self.console.say(&format!("\n{text}"));
        self.results.insert("current".to_owned(), current);
        self.ensure_torque("finish")?;
        self.go("finish", CENTER, MOVE_TIMEOUT)?;
        self.console.say(AFTER_TEXT);
        Ok(())
    }
}

/// 끊김 확인에서 상황과 손으로 본 것을 묻는다. 상황을 고르지 않으면 없다.
pub fn ask_situation(console: &mut dyn Console) -> (Option<&'static str>, String) {
    let choices: Vec<String> = SITUATIONS
        .iter()
        .enumerate()
        .map(|(index, (_, text))| format!("{} {text}", index + 1))
        .collect();
    let answer = console.ask(&format!(
        "지금 상황은? ({}, 그 밖은 Enter): ",
        choices.join(", ")
    ));
    let situation = answer
        .parse::<usize>()
        .ok()
        .and_then(|n| n.checked_sub(1))
        .and_then(|index| SITUATIONS.get(index))
        .map(|(name, _)| *name);
    let holds = console.ask("방금 혼을 손으로 살짝 돌려 봤을 때 버텼으면 y, 그냥 돌아갔으면 n: ");
    (situation, holds)
}

/// 상태만 읽어 보이고, 앞선 맨 서보 확인과 견준다 (끊김 확인). 서보를 움직이지 않는다.
///
/// `previous`는 그 확인의 results다. 잠금 단계에서 바꾼 P 게인과, 처음에 뜬 덤프의 EEPROM을 지금과 견준다.
pub fn state_report<P: Port>(
    joints: &mut Joints<P>,
    id: u8,
    console: &mut dyn Console,
    previous: Option<&Value>,
    situation: Option<&str>,
) -> Result<Value, String> {
    let snapshot = joints.snapshot(id)?;
    let memory = joints.dump(id)?;
    console.say(&describe(&snapshot));
    let mut report = json!({ "snapshot": snapshot_json(&snapshot), "dump": dump_json(&memory) });
    let Some(previous) = previous else {
        console.say("견줄 맨 서보 확인 기록이 없다");
        return Ok(report);
    };
    let power_back = situation == Some("power_back");
    let lock = &previous["lock"];
    let p_gain = report["dump"]["values"]["p_gain"].as_i64();
    if let (Some(before), Some(trial), Some(p_gain)) =
        (lock["p_before"].as_i64(), lock["p_trial"].as_i64(), p_gain)
    {
        let text = match (p_gain == before, p_gain == trial, power_back) {
            (true, _, true) => "잠근 채 쓴 값이 전원을 끄자 사라졌다 (메모리 표대로다)",
            (_, true, true) => "전원을 껐다 켰는데 잠근 채 쓴 값이 남았다 (메모리 표와 다르다)",
            (_, true, false) => "잠근 채 쓴 값 그대로다 (전원이 끊기지 않았으니 맞다)",
            (true, _, false) => {
                "처음 값으로 돌아왔다 (전원이 끊기지 않았는데 돌아왔다. 서보가 다시 켜졌을 수 있다)"
            }
            _ => "처음 값도 잠근 채 쓴 값도 아니다",
        };
        console.say(&format!(
            "P 게인 {p_gain} (처음 {before}, 잠근 채 쓴 값 {trial}): {text}"
        ));
        report["lock"] = json!({ "p_gain": p_gain, "p_before": before, "p_trial": trial });
    }
    let first = &previous["state"]["dump"]["values"];
    if !first.is_null() {
        let differences = eeprom_differences(first, &report["dump"]["values"]);
        if differences.is_empty() {
            console.say("EEPROM이 확인을 시작할 때의 덤프와 같다");
        } else {
            console.say(&format!(
                "EEPROM이 확인을 시작할 때와 다른 칸: {}",
                differences.join(", ")
            ));
        }
        report["eeprom_differences"] = json!(differences);
    }
    Ok(report)
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use horilab_servo::feetech::fake::{FakePort, FakeServo, open_fake_bus};

    use super::*;
    use crate::joints::{BENCH_CAPS, Servo};

    /// 정한 답을 차례로 내고, Enter는 세 번 물어본 뒤에 눌린다.
    struct Script {
        answers: VecDeque<String>,
        said: Vec<String>,
        polls: u32,
    }

    impl Script {
        fn new(answers: &[&str]) -> Self {
            Self {
                answers: answers.iter().map(|a| (*a).to_owned()).collect(),
                said: Vec::new(),
                polls: 0,
            }
        }
    }

    impl Console for Script {
        fn say(&mut self, text: &str) {
            self.said.push(text.to_owned());
        }

        fn ask(&mut self, question: &str) -> String {
            self.said.push(question.to_owned());
            self.answers.pop_front().unwrap_or_default()
        }

        fn begin_wait(&mut self, prompt: &str) {
            self.said.push(prompt.to_owned());
            self.polls = 0;
        }

        fn enter_pressed(&mut self) -> bool {
            self.polls += 1;
            self.polls > 3
        }

        fn status(&mut self, _text: &str) {}
    }

    /// 기다리지 않는 시계.
    #[derive(Default)]
    struct Virtual {
        now: f64,
    }

    impl Clock for Virtual {
        fn now(&self) -> f64 {
            self.now
        }

        fn sleep(&mut self, seconds: f64) {
            self.now += seconds;
        }
    }

    fn temp_dir(name: &str) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!("horilab-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&path);
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    fn bench(servo: FakeServo) -> Joints<FakePort> {
        let id = servo.id();
        Joints::open(
            open_fake_bus([servo]),
            vec![Servo::bench(id, 2825, BENCH_RANGE.0, BENCH_RANGE.1)],
            BENCH_CAPS,
        )
        .unwrap()
    }

    fn check(
        servo: FakeServo,
        answers: &[&str],
        folder: &std::path::Path,
    ) -> Check<FakePort, Script, Virtual> {
        let id = servo.id();
        let log = Log::create(folder, &Snapshot::columns()).unwrap();
        Check::new(
            bench(servo),
            id,
            Script::new(answers),
            Virtual::default(),
            log,
        )
    }

    #[test]
    fn whole_check_restores_trial_values_and_leaves_nothing_after_power_cycle() {
        let folder = temp_dir("check");
        let servo = FakeServo::with(20, 2825, 900);
        let flash = servo.flash;
        // 토크 켜기 y, 과부하 두 번(잡음, 놓음), 전압 준비
        let mut run = check(servo, &["y", "", "", "", "", ""], &folder);
        let mut saved = 0;
        run.run(&mut |_| saved += 1).unwrap();
        assert_eq!(saved, STEPS.len());
        for (name, _) in STEPS {
            assert!(run.results.contains_key(name), "{name}");
            let result = &run.results[name];
            assert!(result.get("error").is_none(), "{name}: {result}");
            assert!(result.get("skipped").is_none(), "{name}: {result}");
        }
        assert_eq!(run.results["enable"]["enabled"], "turned_on");
        assert_eq!(run.results["lock"]["applied"], true);
        assert_eq!(run.results["multi_turn"]["beyond_one_turn"], true); // 가짜는 한계를 지키지 않는다
        assert_eq!(run.results["limit_enable"]["moved_by"], 0); // 가짜는 한계 밖에서도 그 자리에 있는다
        assert_eq!(run.results["limit_enable"]["torque_on"], true);
        assert_eq!(run.results["offset"]["applied"], false); // 가짜는 따로 켜지 않으면 읽는 위치를 옮기지 않는다
        assert_eq!(run.results["current"]["sign"], "unknown"); // 가짜는 전류가 0이다
        let state = run.joints.snapshot(20).unwrap();
        assert!(state.torque_on());
        assert_eq!(state.position, CENTER);
        assert_eq!(state.torque_limit, BENCH_CAPS.torque_limit);
        // 잠깐 바꾼 한계, 오프셋, 보호 설정은 되돌렸다
        let registers = [
            reg::MIN_ANGLE_LIMIT,
            reg::MAX_ANGLE_LIMIT,
            reg::MIN_VOLTAGE,
            reg::UNLOAD_CONDITION,
            reg::POSITION_OFFSET,
            reg::HOLDING_TORQUE,
            reg::PROTECTION_TIME,
            reg::OVERLOAD_TORQUE,
        ];
        let values: Vec<i32> = run
            .joints
            .read(20, &registers)
            .unwrap()
            .iter()
            .map(|(_, v)| *v)
            .collect();
        assert_eq!(values, [0, 4095, 40, 44, 0, 20, 200, 80]);
        // 모두 잠근 채 써서, 전원을 껐다 켜면 P 게인까지 처음과 같다
        let fake = &mut run.joints.wire().servos[0];
        assert_eq!((fake.flash, fake.eeprom_saves), (flash, 0));
        fake.power_cycle();
        let previous = Value::Object(run.results.clone());
        let mut console = Script::new(&[]);
        let report = state_report(
            &mut run.joints,
            20,
            &mut console,
            Some(&previous),
            Some("power_back"),
        )
        .unwrap();
        assert_eq!(report["lock"]["p_gain"], 32);
        assert_eq!(report["eeprom_differences"], json!([]));
        assert!(console.said.iter().any(|line| line.contains("사라졌다")));
        let samples = std::fs::read_to_string(folder.join("samples.csv")).unwrap();
        assert!(samples.lines().count() > 100);
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn hand_steps_can_be_skipped() {
        let folder = temp_dir("check-skip");
        let mut run = check(FakeServo::new(20), &["y", "s", "s", "s"], &folder);
        run.run(&mut |_| {}).unwrap();
        for name in ["overload_unload", "overload_hold", "voltage"] {
            assert_eq!(run.results[name]["skipped"], true, "{name}");
        }
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn torque_left_on_is_inherited_on_the_next_run() {
        let folder = temp_dir("check-again");
        let mut servo = FakeServo::with(20, 2825, 1700);
        servo.set(reg::TORQUE_ENABLE, reg::TORQUE_ON);
        let mut run = check(servo, &["y", "s", "s", "s"], &folder);
        run.run(&mut |_| {}).unwrap();
        assert_eq!(run.results["enable"]["enabled"], "already_on");
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn trace_stops_when_the_servo_stops_short_of_the_goal() {
        let folder = temp_dir("check-short");
        let mut servo = FakeServo::new(20);
        servo.faults.ignored_writes_to.extend([42, 43]); // 목표 쓰기를 무시해 제자리에 선다
        let mut run = check(servo, &[], &folder);
        run.joints.enable(20).unwrap();
        let trace = run.go("test", 3000, MOVE_TIMEOUT).unwrap();
        assert!(!trace.timed_out);
        assert_eq!(trace.end(), Some(2048));
        let (last, _) = trace.points.last().unwrap();
        assert!((0.5..0.6).contains(last), "{last}");
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn offset_step_follows_the_reading_and_crosses_the_old_boundary() {
        let folder = temp_dir("check-offset");
        let mut servo = FakeServo::with(20, 2825, 900);
        servo.offset_moves_reading = true;
        servo.set(reg::POSITION_OFFSET, 85); // 가진 서보에 남아 있던 값
        let mut run = check(servo, &[], &folder);
        let value = run.step_offset().unwrap();
        assert_eq!(value["offset_before"], 85);
        assert_eq!(value["offset_trial"], 597);
        // 읽는 위치 = 엔코더 값 - 오프셋이면, 오프셋을 512 올릴 때 읽는 위치가 512 내려간다
        assert_eq!(value["position_before"], CENTER);
        assert_eq!(value["shift"], -512);
        assert_eq!(value["applied"], true);
        // 옛 경계(읽는 값 0)는 이제 3584로 읽힌다. 지금 위치(1536) 쪽인 3284에서 3884로 가로지른다
        assert_eq!(value["old_boundary_at"], 3584);
        assert_eq!(value["near"], 3284);
        assert_eq!(value["far"], 3884);
        assert_eq!(value["end_far"], 3884);
        assert_eq!(value["boundary_moved"], true);
        // 오프셋을 되돌리자 읽는 위치도 돌아왔고(3884 + 512 - 4096), 토크를 켠 채 가운데로 돌아왔다
        assert_eq!(value["position_after_restore"], 300);
        assert_eq!(run.read_one(reg::POSITION_OFFSET).unwrap(), 85);
        let state = run.joints.snapshot(20).unwrap();
        assert!(state.torque_on());
        assert_eq!(state.position, CENTER);
        assert_eq!(run.joints.wire().servos[0].eeprom_saves, 0);
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn offset_step_restores_the_offset_when_the_servo_does_not_cross() {
        let folder = temp_dir("check-offset-stuck");
        let mut servo = FakeServo::new(20);
        servo.offset_moves_reading = true;
        let mut run = check(servo, &[], &folder);
        run.joints.enable(20).unwrap();
        // 목표 쓰기를 무시해 제자리에 서는 서보: 옛 경계 자리를 지나가지 못한 것으로 본다
        run.joints.wire().servos[0]
            .faults
            .ignored_writes_to
            .extend([42, 43]);
        let value = run.step_offset().unwrap();
        assert_eq!(value["applied"], true);
        assert_eq!(value["at_near"], false);
        assert_eq!(value["reached"], false);
        assert_eq!(value["boundary_moved"], false);
        assert_eq!(run.read_one(reg::POSITION_OFFSET).unwrap(), 0);
        assert!(run.joints.snapshot(20).unwrap().torque_on());
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn state_report_without_power_loss_keeps_the_trial_value() {
        let mut servo = FakeServo::new(20);
        servo.set(reg::P_GAIN, 40); // 잠근 채 쓴 값이 남아 있다
        let mut joints = bench(servo);
        let previous = json!({ "lock": { "p_before": 32, "p_trial": 40 } });
        let mut console = Script::new(&[]);
        let report = state_report(
            &mut joints,
            20,
            &mut console,
            Some(&previous),
            Some("usb_replugged"),
        )
        .unwrap();
        assert_eq!(report["lock"]["p_gain"], 40);
        assert!(report.get("eeprom_differences").is_none()); // 처음 덤프가 없다
        assert!(
            console
                .said
                .iter()
                .any(|line| line.contains("끊기지 않았으니"))
        );
    }

    #[test]
    fn situations_are_asked_by_number() {
        let mut console = Script::new(&["2", "y", "", "n", "9", "y"]);
        assert_eq!(
            ask_situation(&mut console),
            (Some("usb_replugged"), "y".to_owned())
        );
        assert_eq!(ask_situation(&mut console), (None, "n".to_owned()));
        assert_eq!(ask_situation(&mut console), (None, "y".to_owned()));
    }

    #[test]
    fn crossings_are_classified_from_positions() {
        assert_eq!(
            classify_crossing(&[3900, 3000, 2048, 1000, 200]),
            Crossing::LongWay
        );
        assert_eq!(classify_crossing(&[3900, 4050, 60, 200]), Crossing::Wrapped);
        assert_eq!(classify_crossing(&[3900, 200]), Crossing::Wrapped);
        assert_eq!(classify_crossing(&[3900]), Crossing::Unknown);
    }

    #[test]
    fn speed_unit_comes_from_the_ratio() {
        // 0.1 s마다 300 step씩: 3000 step/s, 서보는 60을 알려 줌 → 비 50
        let points: Vec<(f64, i32, i32)> = (0..10)
            .map(|i| (f64::from(i) * 0.1, 2048 + 300 * i, 60))
            .collect();
        let (observed, reported, ratio) = speed_ratio(&points).unwrap();
        assert!((observed - 3000.0).abs() < 1.0);
        assert!((reported - 60.0).abs() < 1e-9);
        assert_eq!(speed_unit(ratio), Some(50));
        assert_eq!(speed_unit(1.1), Some(1));
        assert_eq!(speed_unit(9.0), None);
        assert!(speed_ratio(&[(0.0, 2048, 0), (1.0, 3072, 0)]).is_none());
    }

    #[test]
    fn current_sign_comes_from_the_bit_that_changes_with_direction() {
        assert_eq!(current_sign(&[40, 42], &[0x8000 | 41]), "bit15");
        assert_eq!(current_sign(&[0x8000 | 40], &[41]), "bit15");
        assert_eq!(current_sign(&[40], &[0x400 | 41]), "bit10");
        assert_eq!(current_sign(&[40], &[41]), "none");
        assert_eq!(current_sign(&[0, 1], &[2]), "unknown");
        assert_eq!(current_sign(&[], &[]), "unknown");
    }
}
