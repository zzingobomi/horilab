//! 패킷: `FF FF ID LEN INST PARAMS… CHK` (명령), `FF FF ID LEN ERR DATA… CHK` (응답).
//!
//! LEN은 PARAMS(또는 DATA) 길이 + 2이고, CHK는 ID부터 PARAMS 끝까지 더한 값의 비트 반전이다.

use std::collections::{BTreeMap, BTreeSet};

pub(crate) const PING: u8 = 0x01;
pub(crate) const READ: u8 = 0x02;
pub(crate) const WRITE: u8 = 0x03;
pub(crate) const SYNC_READ: u8 = 0x82;
pub(crate) const SYNC_WRITE: u8 = 0x83;
/// 모든 서보가 받는 ID. 동기 읽기·쓰기에 쓴다.
pub(crate) const BROADCAST_ID: u8 = 0xFE;
/// 응답 패킷에서 데이터 밖의 바이트 수 (FF FF ID LEN ERR … CHK).
pub(crate) const STATUS_OVERHEAD: usize = 6;

/// 서보마다의 응답: ID → (상태 바이트, 데이터).
pub(crate) type Replies = BTreeMap<u8, (u8, Vec<u8>)>;

pub(crate) fn checksum(body: &[u8]) -> u8 {
    !body.iter().fold(0u8, |sum, &byte| sum.wrapping_add(byte))
}

/// 명령 패킷을 만든다.
pub(crate) fn instruction(id: u8, instruction: u8, params: &[u8]) -> Vec<u8> {
    framed(id, instruction, params)
}

/// 응답 패킷을 만든다 (가짜 서보가 쓴다).
pub(crate) fn status(id: u8, error: u8, data: &[u8]) -> Vec<u8> {
    framed(id, error, data)
}

fn framed(id: u8, code: u8, body: &[u8]) -> Vec<u8> {
    // 몸통은 최대 253바이트다 (LEN이 한 바이트). 이 크레이트가 만드는 패킷은 그보다 훨씬 작다.
    let length = u8::try_from(body.len() + 2).unwrap_or(u8::MAX);
    let mut packet = Vec::with_capacity(body.len() + STATUS_OVERHEAD);
    packet.extend_from_slice(&[0xFF, 0xFF, id, length, code]);
    packet.extend_from_slice(body);
    packet.push(checksum(&packet[2..]));
    packet
}

/// 받은 바이트를 서보마다의 응답 패킷으로 나눈다.
///
/// 데이터가 `data_len` 바이트인 응답만 받는다. 길이와 체크섬이 맞는 응답과, 머리(FF FF ID)는
/// 있는데 깨진 응답의 ID를 따로 돌려준다. 깨진 패킷은 머리 3바이트만 건너뛰어서 뒤의 응답을 살린다.
pub(crate) fn split_replies(raw: &[u8], data_len: usize) -> (Replies, BTreeSet<u8>) {
    let size = data_len + STATUS_OVERHEAD;
    let mut good = Replies::new();
    let mut broken = BTreeSet::new();
    let mut i = 0;
    while i + 2 < raw.len() {
        if raw[i] != 0xFF || raw[i + 1] != 0xFF || raw[i + 2] == 0xFF {
            i += 1;
            continue;
        }
        let id = raw[i + 2];
        let packet = &raw[i..raw.len().min(i + size)];
        let valid = packet.len() == size
            && usize::from(packet[3]) == data_len + 2
            && checksum(&packet[2..size - 1]) == packet[size - 1];
        if valid {
            good.insert(id, (packet[4], packet[5..size - 1].to_vec()));
            i += size;
        } else {
            broken.insert(id);
            i += 3;
        }
    }
    broken.retain(|id| !good.contains_key(id));
    (good, broken)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_instruction_matches_protocol() {
        // 서보 1의 56번부터 2바이트 읽기 (프로토콜 문서의 예와 같은 꼴)
        assert_eq!(
            instruction(1, READ, &[56, 2]),
            vec![0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]
        );
    }

    #[test]
    fn splits_replies_and_skips_noise() {
        let mut raw = vec![0x00, 0xFF];
        raw.extend(status(1, 0, &[0x10, 0x00]));
        raw.extend(status(3, 0x04, &[0x20, 0x00]));
        let (good, broken) = split_replies(&raw, 2);
        assert_eq!(good[&1], (0, vec![0x10, 0x00]));
        assert_eq!(good[&3], (0x04, vec![0x20, 0x00]));
        assert!(broken.is_empty());
    }

    #[test]
    fn broken_reply_does_not_hide_the_next_one() {
        let mut first = status(1, 0, &[0x10, 0x00]);
        let last = first.len() - 1;
        first[last] ^= 0xFF;
        let mut raw = first;
        raw.extend(status(3, 0, &[0x20, 0x00]));
        let (good, broken) = split_replies(&raw, 2);
        assert!(good.contains_key(&3));
        assert_eq!(broken.into_iter().collect::<Vec<_>>(), vec![1]);
    }
}
