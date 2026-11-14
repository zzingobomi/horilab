//! 시뮬레이션 버스 (docs/decisions/0033-planning-and-simulation.md): Python MuJoCo 시뮬레이션 서버
//! (`packages/sim`)에 TCP로 붙어 서보 거래를 낸다. 거래는 JSON 한 줄씩이고, 동기 읽기마다 시뮬레이션이 한
//! 주기씩 진행한다 (한 걸음씩). 관절 층과 실행 루프는 [`ServoBus`]만 쓰므로 실물과 같은 코드로 돈다.

use std::collections::BTreeMap;
use std::io::{self, BufRead, BufReader, Write};
use std::net::TcpStream;
use std::time::Duration;

use serde_json::{Value, json};

use crate::feetech::bus::{check_id, check_ram_write};
use crate::feetech::registers::{Fault, Register};
use crate::feetech::{Error, LinkStats, Reply, ServoBus, SyncRead};

/// 시뮬레이션에 둘 서보 하나.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SimServo {
    pub id: u8,
    /// 서보가 돌리는 관절 (MJCF의 관절 이름, 예: j2)
    pub joint: String,
    /// 서보가 알려 줄 모델 번호
    pub model: u16,
}

/// 시뮬레이션 서버에 붙은 서보 버스.
pub struct SimBus {
    reader: BufReader<TcpStream>,
    writer: TcpStream,
    stats: BTreeMap<u8, LinkStats>,
    /// 시뮬레이션 시간 [s] (마지막 동기 읽기 뒤)
    time: f64,
    /// 연결을 버린 까닭 (보내거나 받지 못했다)
    broken: Option<String>,
}

/// 세계를 만드는 첫 요청의 답을 기다리는 시간
const HELLO_TIMEOUT: Duration = Duration::from_secs(10);
/// 그 뒤 요청마다 답을 기다리는 시간. 넘으면 서버가 멈춘 것으로 본다
const REPLY_TIMEOUT: Duration = Duration::from_secs(1);

impl SimBus {
    /// 서버에 붙고 세계를 새로 만든다. `period`는 동기 읽기 한 번에 진행할 시간이다.
    pub fn connect(address: &str, period: Duration, servos: &[SimServo]) -> Result<Self, Error> {
        let writer =
            TcpStream::connect(address).map_err(|e| port("시뮬레이션 서버에 붙지 못했다", e))?;
        writer
            .set_nodelay(true)
            .map_err(|e| port("연결을 설정하지 못했다", e))?;
        // 세계를 만드는 첫 요청은 모델을 읽느라 오래 걸릴 수 있다
        writer
            .set_read_timeout(Some(HELLO_TIMEOUT))
            .map_err(|e| port("연결을 설정하지 못했다", e))?;
        let reader = BufReader::new(
            writer
                .try_clone()
                .map_err(|e| port("연결을 나누지 못했다", e))?,
        );
        let mut bus = Self {
            reader,
            writer,
            stats: BTreeMap::new(),
            time: 0.0,
            broken: None,
        };
        let servos: Vec<Value> = servos
            .iter()
            .map(|s| json!({"id": s.id, "joint": s.joint, "model": s.model}))
            .collect();
        let answer = bus.request(&json!({
            "op": "hello",
            "period": period.as_secs_f64(),
            "servos": servos,
        }))?;
        check(0, &answer)?;
        bus.writer
            .set_read_timeout(Some(REPLY_TIMEOUT))
            .map_err(|e| port("연결을 설정하지 못했다", e))?;
        Ok(bus)
    }

    /// 시뮬레이션 시간 [s].
    pub fn time(&self) -> f64 {
        self.time
    }

    /// 시험용: 서보 전원을 껐다 켠다.
    pub fn power_cycle(&mut self, id: u8) -> Result<(), Error> {
        let answer = self.request(&json!({"op": "power_cycle", "id": id}))?;
        check(id, &answer)
    }

    /// 요청 한 줄을 보내고 답 한 줄을 받는다. 보내거나 받지 못하면(제한 시간 포함) 연결을 버린다: 늦게 온 답이
    /// 다음 요청의 답으로 읽히면 안 된다. 버린 뒤의 요청은 모두 실패하고, 실행 루프는 잇단 실패로 멈춘다.
    fn request(&mut self, body: &Value) -> Result<Value, Error> {
        if let Some(reason) = &self.broken {
            return Err(Error::Invalid(format!(
                "시뮬레이션 연결을 버렸다: {reason}"
            )));
        }
        let mut line = body.to_string();
        line.push('\n');
        if let Err(e) = self.writer.write_all(line.as_bytes()) {
            self.broken = Some(e.to_string());
            return Err(port("요청을 보내지 못했다", e));
        }
        let mut answer = String::new();
        let read = match self.reader.read_line(&mut answer) {
            Ok(read) => read,
            Err(e) => {
                self.broken = Some(e.to_string());
                return Err(port("답을 받지 못했다", e));
            }
        };
        if read == 0 {
            self.broken = Some("서버가 연결을 닫았다".to_owned());
            return Err(port(
                "시뮬레이션 서버가 연결을 닫았다",
                io::Error::from(io::ErrorKind::UnexpectedEof),
            ));
        }
        serde_json::from_str(&answer)
            .map_err(|e| Error::Invalid(format!("시뮬레이션 서버의 답을 읽지 못했다: {e}")))
    }

    /// 서보 하나에 대한 요청: 거절은 오류이고, 서보마다 통신 성적을 센다.
    fn servo_request(&mut self, id: u8, body: &Value) -> Result<Value, Error> {
        let answer = self.request(body)?;
        let result = check(id, &answer).map(|()| answer);
        self.stats
            .entry(id)
            .or_default()
            .record(result.as_ref().map(|_| ()));
        result
    }
}

fn port(what: &str, source: io::Error) -> Error {
    Error::Port {
        what: what.to_owned(),
        source,
    }
}

/// 답이 거절이면 오류다: no_reply는 응답 없음, 그 밖은 잘못된 요청.
fn check(id: u8, answer: &Value) -> Result<(), Error> {
    if answer["ok"] == true {
        return Ok(());
    }
    let error = answer["error"].as_str().unwrap_or("까닭이 없다");
    if error == "no_reply" {
        Err(Error::NoReply {
            id,
            what: "시뮬레이션에 없는 서보다".to_owned(),
        })
    } else {
        Err(Error::Invalid(format!(
            "서보 {id}: 시뮬레이션 서버가 거절했다: {error}"
        )))
    }
}

fn number<T: TryFrom<i64>>(value: &Value) -> Option<T> {
    value.as_i64().and_then(|v| T::try_from(v).ok())
}

/// 답의 값 표를 메모리 표의 레지스터 이름으로 푼다.
fn reply(values: &Value, fault: &Value, registers: &[Register]) -> Result<Reply, String> {
    let mut decoded = BTreeMap::new();
    for register in registers {
        let value: i32 = number(&values[register.name])
            .ok_or_else(|| format!("{}이 답에 없다", register.name))?;
        decoded.insert(register.name, value);
    }
    Ok(Reply {
        values: decoded,
        fault: Fault(number(fault).unwrap_or(0)),
    })
}

impl ServoBus for SimBus {
    fn ping(&mut self, id: u8) -> Result<u16, Error> {
        let answer = self.servo_request(id, &json!({"op": "ping", "id": id}))?;
        number(&answer["model"]).ok_or_else(|| Error::BadReply {
            id,
            what: "모델 번호가 없다".to_owned(),
        })
    }

    fn read(&mut self, id: u8, registers: &[Register]) -> Result<Reply, Error> {
        let names: Vec<&str> = registers.iter().map(|r| r.name).collect();
        let answer =
            self.servo_request(id, &json!({"op": "read", "id": id, "registers": names}))?;
        reply(&answer["values"], &answer["fault"], registers)
            .map_err(|what| Error::BadReply { id, what })
    }

    fn sync_read(&mut self, ids: &[u8], registers: &[Register]) -> Result<SyncRead, Error> {
        let names: Vec<&str> = registers.iter().map(|r| r.name).collect();
        let answer = self.request(&json!({"op": "sync_read", "ids": ids, "registers": names}))?;
        check(0, &answer)?;
        self.time = answer["time"].as_f64().unwrap_or(self.time);
        let mut results = BTreeMap::new();
        for &id in ids {
            let entry = &answer["replies"][id.to_string()];
            let result = if entry.is_null() || entry["error"] == "no_reply" {
                Err(Error::NoReply {
                    id,
                    what: "시뮬레이션 동기 읽기에 응답하지 않았다".to_owned(),
                })
            } else {
                reply(&entry["values"], &entry["fault"], registers)
                    .map_err(|what| Error::BadReply { id, what })
            };
            self.stats
                .entry(id)
                .or_default()
                .record(result.as_ref().map(|_| ()));
            results.insert(id, result);
        }
        Ok(results)
    }

    fn write(&mut self, id: u8, register: Register, value: i32) -> Result<Fault, Error> {
        // 실물 버스와 같은 검사: 관절 층의 쓰기 잘못이 시뮬레이션에서 그냥 지나가지 않게
        check_id(id)?;
        check_ram_write(register, value)?;
        let answer = self.servo_request(
            id,
            &json!({"op": "write", "id": id, "register": register.name, "value": value}),
        )?;
        Ok(Fault(number(&answer["fault"]).unwrap_or(0)))
    }

    fn sync_write(&mut self, register: Register, values: &[(u8, i32)]) -> Result<(), Error> {
        for &(id, value) in values {
            check_id(id)?;
            check_ram_write(register, value)?;
        }
        let answer = self.request(&json!({
            "op": "sync_write",
            "register": register.name,
            "values": values,
        }))?;
        check(0, &answer)
    }

    fn write_trial(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), Error> {
        let pairs: Vec<(&str, i32)> = values.iter().map(|(r, v)| (r.name, *v)).collect();
        self.servo_request(id, &json!({"op": "write_trial", "id": id, "values": pairs}))
            .map(|_| ())
    }

    fn dump(&mut self, id: u8) -> Result<Vec<u8>, Error> {
        let answer = self.servo_request(id, &json!({"op": "dump", "id": id}))?;
        Ok(answer["bytes"]
            .as_array()
            .map(|bytes| bytes.iter().filter_map(number).collect())
            .unwrap_or_default())
    }

    fn stats(&self) -> &BTreeMap<u8, LinkStats> {
        &self.stats
    }

    /// 시뮬레이션의 시간은 동기 읽기로만 간다. 기다리지 않는다.
    fn pause(&mut self, _duration: Duration) {}
}
