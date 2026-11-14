//! 서보 버스: 패킷을 주고받고, 서보마다 결과와 통신 성적을 낸다 (docs/decisions/0024-servo-bus.md).
//!
//! - 레지스터는 이름으로 읽고 쓰고, 부호도 여기서 푼다. 값은 서보의 raw 단위 그대로다.
//!   관절 이름과 rad는 이 위의 층이 다룬다.
//! - 동기 읽기는 응답을 서보마다 나눠 길이와 체크섬을 확인한다. 응답이 없거나 깨진 서보는 그 서보만
//!   오류가 되고, 지난 값을 돌려주지 않는다.
//! - EEPROM은 세 길로만 쓴다: 전원을 꺼도 남게 쓰기(`write_eeprom`), ID 바꾸기(`change_id`),
//!   잠근 채 바꿔 보기(`write_trial`).
//! - 포트는 이 버스 하나가 쥔다. 보내기 전에 받은 채 남은 바이트를 버려서, 통신 도중 오류가 나도
//!   다음 통신이 막히지 않는다.
//! - 응답은 정한 여유(latency)까지 기다리고, 기다리는 동안 CPU를 쓰지 않는다 (포트의 읽기 제한 시간).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use crate::feetech::packet;
use crate::feetech::registers::{self as reg, Area, Fault, OutOfRange, Register};

/// 잠근 채 바꿔 볼 수 있는 레지스터: P·I·D, 응답 지연(7번), 그리고 맨 서보 확인에서 서보의 동작을 보려고
/// 잠깐 바꾸는 각도 한계, 전압 하한, 힘을 빼는 조건, 위치 오프셋, 과부하 보호 설정.
pub const TRIAL_REGISTERS: [Register; 12] = [
    reg::P_GAIN,
    reg::I_GAIN,
    reg::D_GAIN,
    reg::RESPONSE_DELAY,
    reg::MIN_ANGLE_LIMIT,
    reg::MAX_ANGLE_LIMIT,
    reg::MIN_VOLTAGE,
    reg::UNLOAD_CONDITION,
    reg::POSITION_OFFSET,
    reg::HOLDING_TORQUE,
    reg::PROTECTION_TIME,
    reg::OVERLOAD_TORQUE,
];

/// 서보 버스가 쓰는 포트. 실물은 [`SerialLink`], 시험용 가짜는 `fake::FakePort`다.
pub trait Port {
    /// 패킷을 보낸다.
    fn send(&mut self, bytes: &[u8]) -> io::Result<()>;
    /// `buffer`가 `want` 바이트가 될 때까지 받는다. `deadline`이 지나면 그때까지 받은 만큼으로 끝낸다.
    fn receive(&mut self, buffer: &mut Vec<u8>, want: usize, deadline: Instant) -> io::Result<()>;
    /// 받은 채 남은 바이트를 버린다.
    fn clear_input(&mut self) -> io::Result<()>;
    /// 서보가 방금 쓴 값을 반영하도록 기다린다. 가짜 포트는 기다리지 않는다.
    fn pause(&mut self, duration: Duration) {
        std::thread::sleep(duration);
    }
}

/// 실물 시리얼 포트.
pub struct SerialLink {
    port: Box<dyn serialport::SerialPort>,
}

impl Port for SerialLink {
    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()
    }

    fn receive(&mut self, buffer: &mut Vec<u8>, want: usize, deadline: Instant) -> io::Result<()> {
        let mut chunk = [0u8; 256];
        while buffer.len() < want {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            // Windows의 읽기 제한 시간은 ms 단위라 0이 되지 않게 한다.
            self.port.set_timeout(left.max(Duration::from_millis(1)))?;
            let size = (want - buffer.len()).min(chunk.len());
            match self.port.read(&mut chunk[..size]) {
                Ok(0) => break,
                Ok(n) => buffer.extend_from_slice(&chunk[..n]),
                Err(error) if error.kind() == io::ErrorKind::TimedOut => break,
                Err(error) => return Err(error),
            }
        }
        Ok(())
    }

    fn clear_input(&mut self) -> io::Result<()> {
        Ok(self.port.clear(serialport::ClearBuffer::Input)?)
    }
}

/// 서보 버스의 오류.
#[derive(Debug)]
pub enum Error {
    /// 응답이 오지 않았다 (전원이 없거나, 선이 빠졌거나, 그 ID가 없다).
    NoReply { id: u8, what: String },
    /// 응답이 깨졌다 (체크섬, 길이). 같은 ID의 서보가 둘이어도 이렇게 된다.
    BadReply { id: u8, what: String },
    /// 포트를 열거나 쓰거나 읽지 못했다.
    Port { what: String, source: io::Error },
    /// 쓴 값을 다시 읽었더니 다르다.
    Verify { id: u8, what: String },
    /// 안전 조건 때문에 하지 않았다 (토크가 켜져 있음, 새 ID가 이미 있음 등).
    Refused(String),
    /// 받을 수 없는 인자 (범위 밖 ID, 쓸 수 없는 레지스터, 담을 수 없는 값).
    Invalid(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoReply { id, what } => write!(f, "서보 {id}: 응답이 없다 ({what})"),
            Self::BadReply { id, what } => write!(f, "서보 {id}: 응답이 깨졌다 ({what})"),
            Self::Port { what, source } => write!(f, "포트 오류: {what}: {source}"),
            Self::Verify { id, what } => write!(f, "서보 {id}: 다시 읽은 값이 다르다 ({what})"),
            Self::Refused(what) => write!(f, "하지 않았다: {what}"),
            Self::Invalid(what) => write!(f, "받을 수 없다: {what}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Port { source, .. } => Some(source),
            _ => None,
        }
    }
}

impl From<OutOfRange> for Error {
    fn from(error: OutOfRange) -> Self {
        Self::Invalid(error.to_string())
    }
}

/// 서보 하나의 응답.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reply {
    /// 레지스터 이름 → 부호를 푼 값
    pub values: BTreeMap<&'static str, i32>,
    /// 응답의 상태 바이트
    pub fault: Fault,
}

impl Reply {
    pub fn get(&self, register: Register) -> Option<i32> {
        self.values.get(register.name).copied()
    }
}

/// 동기 읽기의 서보마다 결과: ID → 응답 또는 그 서보의 오류.
pub type SyncRead = BTreeMap<u8, Result<Reply, Error>>;

/// 서보 하나와의 통신 성적. 안전 검사는 연속 실패를 본다 (docs/decisions/0009).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkStats {
    pub ok: u64,
    pub no_reply: u64,
    pub bad_reply: u64,
    pub consecutive_failures: u64,
}

impl LinkStats {
    pub(crate) fn record(&mut self, outcome: Result<(), &Error>) {
        match outcome {
            Ok(()) => {
                self.ok += 1;
                self.consecutive_failures = 0;
                return;
            }
            Err(Error::NoReply { .. }) => self.no_reply += 1,
            Err(_) => self.bad_reply += 1,
        }
        self.consecutive_failures += 1;
    }
}

/// 찾기 결과.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ScanResult {
    /// ID → 모델 번호
    pub models: BTreeMap<u8, u16>,
    /// 응답이 깨진 ID. 같은 ID의 서보가 둘일 수 있다
    pub garbled: Vec<u8>,
}

/// 서보 버스 하나. 한 포트를 쓰는 코드는 모두 이 버스 하나를 거친다.
pub struct Bus<P: Port> {
    pub(crate) port: P,
    byte_time: Duration,
    latency: Duration,
    stats: BTreeMap<u8, LinkStats>,
}

impl Bus<SerialLink> {
    /// 실물 포트를 연다. `latency`는 응답 하나를 더 기다리는 여유다.
    pub fn open(port_name: &str, baud: u32, latency: Duration) -> Result<Self, Error> {
        let port = serialport::new(port_name, baud)
            .timeout(latency.max(Duration::from_millis(1)))
            .open()
            .map_err(|error| Error::Port {
                what: format!("{port_name}을 열지 못했다"),
                source: error.into(),
            })?;
        Ok(Self::new(SerialLink { port }, baud, latency))
    }
}

impl<P: Port> Bus<P> {
    pub fn new(port: P, baud: u32, latency: Duration) -> Self {
        // 한 바이트는 시작·정지 비트를 더해 10비트다.
        let byte_time = Duration::from_nanos(10_000_000_000 / u64::from(baud.max(1)));
        Self {
            port,
            byte_time,
            latency,
            stats: BTreeMap::new(),
        }
    }

    /// 서보가 방금 쓴 값을 반영하도록 기다린다 (가짜 포트는 기다리지 않는다).
    pub fn pause(&mut self, duration: Duration) {
        self.port.pause(duration);
    }

    pub fn stats(&self) -> &BTreeMap<u8, LinkStats> {
        &self.stats
    }

    // 찾기와 읽기

    /// 서보가 응답하면 모델 번호를 돌려준다. 모델 번호를 읽는 것으로 핑을 대신한다.
    pub fn ping(&mut self, id: u8) -> Result<u16, Error> {
        let (_, data) = self.read_bytes(id, reg::MODEL.address, reg::MODEL.size)?;
        match data[..] {
            [low, high] => Ok(u16::from_le_bytes([low, high])),
            _ => Err(Error::BadReply {
                id,
                what: "모델 번호가 2바이트가 아니다".to_owned(),
            }),
        }
    }

    /// ID마다 응답하는 서보를 찾는다. 없는 ID마다 응답 여유만큼 기다린다.
    pub fn scan(&mut self, ids: impl IntoIterator<Item = u8>) -> Result<ScanResult, Error> {
        let mut result = ScanResult::default();
        for id in ids {
            match self.ping(id) {
                Ok(model) => {
                    result.models.insert(id, model);
                }
                Err(Error::NoReply { .. }) => {
                    self.stats.remove(&id); // 없는 ID의 성적은 남기지 않는다
                }
                Err(Error::BadReply { .. }) => result.garbled.push(id),
                Err(error) => return Err(error),
            }
        }
        Ok(result)
    }

    /// 레지스터들을 한 번의 읽기로 읽는다. 떨어진 주소면 그 사이까지 한 덩어리로 읽는다.
    pub fn read(&mut self, id: u8, registers: &[Register]) -> Result<Reply, Error> {
        let (first, length) = span(registers)?;
        let (fault, data) = self.read_bytes(id, first, length)?;
        let values = decode(registers, &data, first).ok_or_else(|| Error::BadReply {
            id,
            what: "응답 길이가 레지스터와 맞지 않는다".to_owned(),
        })?;
        Ok(Reply { values, fault })
    }

    /// 주소 0부터 메모리 표 끝까지의 바이트. 서보 설정을 기록할 때 쓴다.
    pub fn dump(&mut self, id: u8) -> Result<Vec<u8>, Error> {
        let size = u8::try_from(reg::MEMORY_SIZE).map_err(|_| {
            Error::Invalid("메모리 표가 한 번에 읽을 수 있는 크기를 넘는다".to_owned())
        })?;
        Ok(self.read_bytes(id, 0, size)?.1)
    }

    /// 여러 서보에서 같은 레지스터들을 한 번에 읽는다.
    ///
    /// 서보마다 결과를 돌려준다. 응답이 없거나 깨진 서보는 [`Reply`] 대신 그 오류가 들어간다.
    pub fn sync_read(&mut self, ids: &[u8], registers: &[Register]) -> Result<SyncRead, Error> {
        for &id in ids {
            check_id(id)?;
        }
        if ids.iter().collect::<BTreeSet<_>>().len() != ids.len() {
            return Err(Error::Invalid(format!("ID가 겹친다: {ids:?}")));
        }
        let (first, length) = span(registers)?;
        if ids.is_empty() {
            return Ok(BTreeMap::new());
        }
        let mut params = vec![first, length];
        params.extend_from_slice(ids);
        let data_len = usize::from(length);
        let expected = ids.len() * (data_len + packet::STATUS_OVERHEAD);
        let request = packet::instruction(packet::BROADCAST_ID, packet::SYNC_READ, &params);
        let bytes = self.exchange(&request, expected, None)?;
        let (mut good, broken) = packet::split_replies(&bytes, data_len);
        let mut results = BTreeMap::new();
        for &id in ids {
            let result = match good.remove(&id) {
                Some((error, data)) => decode(registers, &data, first)
                    .map(|values| Reply {
                        values,
                        fault: Fault(error),
                    })
                    .ok_or_else(|| Error::BadReply {
                        id,
                        what: "동기 읽기 응답 길이가 맞지 않는다".to_owned(),
                    }),
                None if broken.contains(&id) => Err(Error::BadReply {
                    id,
                    what: "동기 읽기 응답이 깨졌다".to_owned(),
                }),
                None => Err(Error::NoReply {
                    id,
                    what: "동기 읽기에 응답하지 않았다".to_owned(),
                }),
            };
            self.stats
                .entry(id)
                .or_default()
                .record(result.as_ref().map(|_| ()));
            results.insert(id, result);
        }
        Ok(results)
    }

    // 쓰기

    /// RAM 레지스터 하나를 쓰고, 응답의 상태 바이트를 돌려준다. EEPROM은 `write_eeprom`이나 `write_trial`로 쓴다.
    pub fn write(&mut self, id: u8, register: Register, value: i32) -> Result<Fault, Error> {
        check_ram_write(register, value)?;
        self.write_bytes(id, register.address, &to_bytes(register, value)?)
    }

    /// 여러 서보의 같은 RAM 레지스터를 한 번에 쓴다. 응답이 없어서 결과는 다음 읽기로 본다.
    pub fn sync_write(&mut self, register: Register, values: &[(u8, i32)]) -> Result<(), Error> {
        let mut params = vec![register.address, register.size];
        for &(id, value) in values {
            check_id(id)?;
            check_ram_write(register, value)?;
            params.push(id);
            params.extend(to_bytes(register, value)?);
        }
        if values.is_empty() {
            return Ok(());
        }
        let request = packet::instruction(packet::BROADCAST_ID, packet::SYNC_WRITE, &params);
        self.exchange(&request, 0, None)?;
        Ok(())
    }

    /// EEPROM 값을 전원을 꺼도 남게 쓰고, 다시 읽어 확인한다.
    ///
    /// 토크가 켜져 있으면 쓰지 않는다. ID는 `change_id`로 바꾸고, 통신 속도는 바꾸지 않는다.
    /// EEPROM은 쓰는 횟수에 수명이 있어서, 값이 같은지는 부르는 쪽이 먼저 본다.
    pub fn write_eeprom(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), Error> {
        if values.is_empty() {
            return Ok(());
        }
        let mut writes = Vec::with_capacity(values.len());
        for &(register, value) in values {
            if register.area != Area::Eeprom || !register.writable {
                return Err(Error::Invalid(format!(
                    "{}은 쓸 수 있는 EEPROM 레지스터가 아니다",
                    register.name
                )));
            }
            if register == reg::ID || register == reg::BAUD_RATE {
                return Err(Error::Invalid(format!(
                    "{}은 write_eeprom으로 바꾸지 않는다",
                    register.name
                )));
            }
            writes.push((register.address, to_bytes(register, value)?));
        }
        self.require_torque_off(id)?;
        self.unlocked(id, &[id], |bus| {
            for (address, data) in &writes {
                bus.write_bytes(id, *address, data)?;
            }
            Ok(())
        })?;
        self.verify(id, values)
    }

    /// 잠근 채 바꿔 보기: 바로 듣고, 전원을 끄면 사라진다. 토크가 켜져 있어도 된다.
    ///
    /// [`TRIAL_REGISTERS`]만 받는다. 잠금이 풀린 채 쓰면 값이 남으므로 먼저 잠근다.
    /// 바꾼 값을 기록에 남기는 것은 부르는 쪽이 한다.
    pub fn write_trial(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), Error> {
        if values.is_empty() {
            return Ok(());
        }
        let mut writes = Vec::with_capacity(values.len());
        for &(register, value) in values {
            if !TRIAL_REGISTERS.contains(&register) {
                return Err(Error::Invalid(format!(
                    "{}은 잠근 채 바꿔 보지 않는다 (TRIAL_REGISTERS만)",
                    register.name
                )));
            }
            writes.push((register.address, to_bytes(register, value)?));
        }
        self.write_bytes(id, reg::LOCK.address, &[1])?;
        for (address, data) in &writes {
            self.write_bytes(id, *address, data)?;
        }
        self.verify(id, values)
    }

    /// 서보 ID를 바꾼다. 버스에 서보가 하나만 물렸을 때만 쓴다 (부르는 쪽이 확인한다).
    ///
    /// ID를 쓰면 서보가 바로 새 ID로 바뀌어서, 그 쓰기의 응답은 새 ID로 오고 버린다.
    /// 잠금은 새 ID로 다시 걸고, 새 ID는 응답하고 옛 ID는 응답하지 않는지 확인한다.
    pub fn change_id(&mut self, old: u8, new: u8) -> Result<(), Error> {
        check_id(old)?;
        check_id(new)?;
        if old == new {
            return Ok(());
        }
        if self.responds(new)? {
            return Err(Error::Refused(format!("ID {new}가 이미 버스에 있다")));
        }
        self.require_torque_off(old)?;
        self.unlocked(old, &[new, old], |bus| {
            let request = packet::instruction(old, packet::WRITE, &[reg::ID.address, new]);
            bus.exchange(&request, packet::STATUS_OVERHEAD, Some(old))?;
            Ok(())
        })?;
        self.ping(new)?;
        if self.responds(old)? {
            return Err(Error::Verify {
                id: old,
                what: format!("ID {new}로 바꿨는데 옛 ID가 아직 응답한다"),
            });
        }
        self.stats.remove(&old);
        Ok(())
    }

    // 안쪽

    /// 명령을 보내고 응답 바이트를 받는다. 보내기 전에 받은 채 남은 바이트를 버린다.
    fn exchange(
        &mut self,
        request: &[u8],
        expected: usize,
        id: Option<u8>,
    ) -> Result<Vec<u8>, Error> {
        let port_error = |what: &str, source: io::Error| Error::Port {
            what: match id {
                Some(id) => format!("서보 {id}: {what}"),
                None => what.to_owned(),
            },
            source,
        };
        self.port
            .clear_input()
            .map_err(|source| port_error("받은 바이트 버리기", source))?;
        self.port
            .send(request)
            .map_err(|source| port_error("보내기", source))?;
        let mut received = Vec::with_capacity(expected);
        if expected > 0 {
            let bytes = u32::try_from(request.len() + expected).unwrap_or(u32::MAX);
            let deadline = Instant::now() + self.byte_time * bytes + self.latency;
            self.port
                .receive(&mut received, expected, deadline)
                .map_err(|source| port_error("받기", source))?;
        }
        Ok(received)
    }

    /// 서보 하나에 명령을 보내고 응답 하나를 받는다. 응답의 (상태 바이트, 데이터).
    fn transact(
        &mut self,
        id: u8,
        instruction: u8,
        params: &[u8],
        data_len: usize,
        what: &str,
    ) -> Result<(Fault, Vec<u8>), Error> {
        check_id(id)?;
        let request = packet::instruction(id, instruction, params);
        let bytes = self.exchange(&request, data_len + packet::STATUS_OVERHEAD, Some(id))?;
        let (mut good, broken) = packet::split_replies(&bytes, data_len);
        let outcome = match good.remove(&id) {
            Some((error, data)) => Ok((Fault(error), data)),
            None if bytes.is_empty() => Err(Error::NoReply {
                id,
                what: what.to_owned(),
            }),
            None if broken.contains(&id) => Err(Error::BadReply {
                id,
                what: format!("{what}: 길이나 체크섬이 맞지 않는다"),
            }),
            None => Err(Error::BadReply {
                id,
                what: format!("{what}: 다른 응답이 왔다"),
            }),
        };
        self.stats
            .entry(id)
            .or_default()
            .record(outcome.as_ref().map(|_| ()));
        outcome
    }

    fn read_bytes(&mut self, id: u8, address: u8, length: u8) -> Result<(Fault, Vec<u8>), Error> {
        let what = format!("{address}번부터 {length}바이트 읽기");
        self.transact(
            id,
            packet::READ,
            &[address, length],
            usize::from(length),
            &what,
        )
    }

    fn write_bytes(&mut self, id: u8, address: u8, data: &[u8]) -> Result<Fault, Error> {
        let mut params = Vec::with_capacity(data.len() + 1);
        params.push(address);
        params.extend_from_slice(data);
        let what = format!("{address}번에 쓰기");
        Ok(self.transact(id, packet::WRITE, &params, 0, &what)?.0)
    }

    /// EEPROM 잠금을 풀고 `body`를 한 뒤 다시 건다. 도중에 실패해도 다시 걸어 보고 원래 오류를 돌려준다.
    fn unlocked<T>(
        &mut self,
        id: u8,
        relock_ids: &[u8],
        body: impl FnOnce(&mut Self) -> Result<T, Error>,
    ) -> Result<T, Error> {
        self.write_bytes(id, reg::LOCK.address, &[0])?;
        match body(self) {
            Ok(value) => {
                let relock_id = relock_ids.first().copied().unwrap_or(id);
                self.write_bytes(relock_id, reg::LOCK.address, &[1])?;
                Ok(value)
            }
            Err(error) => {
                for &relock_id in relock_ids {
                    if self.write_bytes(relock_id, reg::LOCK.address, &[1]).is_ok() {
                        break;
                    }
                }
                Err(error)
            }
        }
    }

    fn verify(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), Error> {
        let registers: Vec<Register> = values.iter().map(|&(register, _)| register).collect();
        let readback = self.read(id, &registers)?;
        let wrong: Vec<String> = values
            .iter()
            .filter_map(|&(register, value)| {
                let read = readback.get(register);
                (read != Some(value))
                    .then(|| format!("{}: 쓴 값 {value}, 읽은 값 {read:?}", register.name))
            })
            .collect();
        if wrong.is_empty() {
            Ok(())
        } else {
            Err(Error::Verify {
                id,
                what: wrong.join(", "),
            })
        }
    }

    fn require_torque_off(&mut self, id: u8) -> Result<(), Error> {
        let torque = self
            .read(id, &[reg::TORQUE_ENABLE])?
            .get(reg::TORQUE_ENABLE);
        if torque == Some(reg::TORQUE_OFF) {
            Ok(())
        } else {
            Err(Error::Refused(format!(
                "서보 {id}의 토크가 켜져 있어서 EEPROM을 쓰지 않는다"
            )))
        }
    }

    fn responds(&mut self, id: u8) -> Result<bool, Error> {
        match self.ping(id) {
            Ok(_) => Ok(true),
            Err(Error::NoReply { .. }) => {
                self.stats.remove(&id);
                Ok(false)
            }
            Err(error) => Err(error),
        }
    }
}

fn check_id(id: u8) -> Result<(), Error> {
    if id <= reg::MAX_ID {
        Ok(())
    } else {
        Err(Error::Invalid(format!(
            "서보 ID는 0~{}다: {id}",
            reg::MAX_ID
        )))
    }
}

fn check_ram_write(register: Register, value: i32) -> Result<(), Error> {
    if !register.writable {
        return Err(Error::Invalid(format!("{}은 읽기 전용이다", register.name)));
    }
    if register.area != Area::Ram {
        return Err(Error::Invalid(format!(
            "{}은 EEPROM이라 write_eeprom이나 write_trial로 쓴다",
            register.name
        )));
    }
    if register == reg::LOCK {
        return Err(Error::Invalid("잠금은 EEPROM 절차가 다룬다".to_owned()));
    }
    if register == reg::TORQUE_ENABLE && !reg::TORQUE_VALUES.contains(&value) {
        return Err(Error::Invalid(format!(
            "토크 스위치에 {value}를 쓰지 않는다 (128은 위치 오프셋을 바꾼다)"
        )));
    }
    register.encode(value)?;
    Ok(())
}

/// 레지스터들을 덮는 첫 주소와 길이.
fn span(registers: &[Register]) -> Result<(u8, u8), Error> {
    let first = registers
        .iter()
        .map(|register| register.address)
        .min()
        .ok_or_else(|| Error::Invalid("읽을 레지스터가 없다".to_owned()))?;
    let end = registers
        .iter()
        .map(|register| register.address + register.size)
        .max()
        .unwrap_or(first);
    Ok((first, end - first))
}

/// 받은 데이터에서 레지스터마다 값을 꺼낸다. 데이터가 모자라면 None.
fn decode(registers: &[Register], data: &[u8], first: u8) -> Option<BTreeMap<&'static str, i32>> {
    registers
        .iter()
        .map(|register| {
            let start = usize::from(register.address.checked_sub(first)?);
            let raw = match data.get(start..start + usize::from(register.size))? {
                [low] => u16::from(*low),
                [low, high] => u16::from_le_bytes([*low, *high]),
                _ => return None,
            };
            Some((register.name, register.decode(raw)))
        })
        .collect()
}

fn to_bytes(register: Register, value: i32) -> Result<Vec<u8>, Error> {
    let [low, high] = register.encode(value)?.to_le_bytes();
    Ok(if register.size == 1 {
        vec![low]
    } else {
        vec![low, high]
    })
}

/// 서보와 거래하는 버스 ([0033](../../../../docs/decisions/0033-planning-and-simulation.md)): 실물 Feetech 버스와
/// 시뮬레이션이 같은 거래를 낸다. 관절 층과 실행 루프는 이 거래만 쓰므로, 그 위의 코드(ID 대응, 영점·방향 변환,
/// 보호 비트, 통신 실패, 실행 루프)가 실물과 시뮬레이션에서 같다.
pub trait ServoBus {
    /// 서보가 응답하면 모델 번호.
    fn ping(&mut self, id: u8) -> Result<u16, Error>;
    /// 한 서보의 레지스터들을 읽는다.
    fn read(&mut self, id: u8, registers: &[Register]) -> Result<Reply, Error>;
    /// 여러 서보의 같은 레지스터들을 한 번에 읽는다. 서보마다 결과다.
    fn sync_read(&mut self, ids: &[u8], registers: &[Register]) -> Result<SyncRead, Error>;
    /// RAM 레지스터 하나를 쓰고 응답의 상태 바이트를 받는다.
    fn write(&mut self, id: u8, register: Register, value: i32) -> Result<Fault, Error>;
    /// 여러 서보의 같은 RAM 레지스터를 한 번에 쓴다.
    fn sync_write(&mut self, register: Register, values: &[(u8, i32)]) -> Result<(), Error>;
    /// 잠근 채 바꿔 보기 (전원을 끄면 사라진다).
    fn write_trial(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), Error>;
    /// 주소 0부터 메모리 표 끝까지의 바이트.
    fn dump(&mut self, id: u8) -> Result<Vec<u8>, Error>;
    /// 서보마다 통신 성적. 안전 검사는 연속 실패를 본다.
    fn stats(&self) -> &BTreeMap<u8, LinkStats>;
    /// 기다린다.
    fn pause(&mut self, duration: Duration);
}

impl<P: Port> ServoBus for Bus<P> {
    fn ping(&mut self, id: u8) -> Result<u16, Error> {
        Bus::ping(self, id)
    }

    fn read(&mut self, id: u8, registers: &[Register]) -> Result<Reply, Error> {
        Bus::read(self, id, registers)
    }

    fn sync_read(&mut self, ids: &[u8], registers: &[Register]) -> Result<SyncRead, Error> {
        Bus::sync_read(self, ids, registers)
    }

    fn write(&mut self, id: u8, register: Register, value: i32) -> Result<Fault, Error> {
        Bus::write(self, id, register, value)
    }

    fn sync_write(&mut self, register: Register, values: &[(u8, i32)]) -> Result<(), Error> {
        Bus::sync_write(self, register, values)
    }

    fn write_trial(&mut self, id: u8, values: &[(Register, i32)]) -> Result<(), Error> {
        Bus::write_trial(self, id, values)
    }

    fn dump(&mut self, id: u8) -> Result<Vec<u8>, Error> {
        Bus::dump(self, id)
    }

    fn stats(&self) -> &BTreeMap<u8, LinkStats> {
        Bus::stats(self)
    }

    fn pause(&mut self, duration: Duration) {
        Bus::pause(self, duration);
    }
}
