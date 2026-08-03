//! Wire protocol and timing used by the DW3000 anchor/tag test network.
//!
//! The network is intentionally statically provisioned. One anchor is the
//! timing master; it transmits a synchronization packet once per superframe.
//! Every anchor then owns one deterministic slot per tag. This keeps the RF
//! medium collision-free without requiring a second radio or a host link.

use dw3000_ng::{
    configs::{BitRate, PreambleLength, PulseRepetitionFrequency, UwbChannel},
    time::{Duration, Instant, TIME_MAX},
    Config,
};

pub const PAN_ID: u16 = 0x0D57;
pub const MASTER_ANCHOR_ID: u16 = ANCHOR_IDS[0];
pub const ANCHOR_IDS: [u16; 5] = [0xA001, 0xA002, 0xA003, 0xA004, 0xA005];
pub const TAG_IDS: [u16; 12] = [
    0xB001, 0xB002, 0xB003, 0xB004, 0xB005, 0xB006, 0xB007, 0xB008, 0xB009, 0xB00A, 0xB00B, 0xB00C,
];

pub const ACTIVE_ANCHOR_COUNT: usize = ANCHOR_IDS.len();
pub const ACTIVE_TAG_COUNT: usize = TAG_IDS.len();

pub const SYNC_GUARD_US: u32 = 5_000;
pub const SLOT_US: u32 = 2_000;
pub const REPLY_DELAY_US: u32 = 700;
pub const RX_TIMEOUT_US: u32 = 1_300;

/// Derived from the anchor/tag table so the two always stay consistent: a
/// leading guard, one `SLOT_US` DS-TWR slot per anchor/tag combination, and a
/// trailing guard before the next synchronization frame. For five anchors and
/// twelve tags this comes out to 130 ms, i.e. about 7.7 Hz per tag position.
/// Shrinking either table (e.g. four anchors) automatically shortens the
/// superframe; if the tables grow enough that a superframe would overflow
/// `u32`, this const-evaluates to a build error instead of silently wrapping.
pub const SUPERFRAME_US: u32 =
    SYNC_GUARD_US + (ACTIVE_ANCHOR_COUNT * ACTIVE_TAG_COUNT) as u32 * SLOT_US + SYNC_GUARD_US;

const MAGIC: u8 = 0xD3;
const VERSION: u8 = 1;
const TIME_BYTES: usize = 5;
const TIME_MASK: u64 = TIME_MAX;
const MM_PER_DWT_NUM: i128 = 4_691_764;
const MM_PER_DWT_DEN: i128 = 1_000_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum PacketKind {
    Sync = 1,
    Poll = 2,
    Request = 3,
    Response = 4,
}

impl TryFrom<u8> for PacketKind {
    type Error = PacketError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Sync),
            2 => Ok(Self::Poll),
            3 => Ok(Self::Request),
            4 => Ok(Self::Response),
            _ => Err(PacketError::UnknownKind),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PacketError {
    InvalidHeader,
    InvalidLength,
    UnknownKind,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Packet {
    Sync {
        sequence: u16,
    },
    Poll {
        anchor_id: u16,
        tag_id: u16,
        sequence: u16,
        poll_tx: u64,
    },
    Request {
        anchor_id: u16,
        tag_id: u16,
        sequence: u16,
        poll_rx: u64,
        request_tx: u64,
    },
    Response {
        anchor_id: u16,
        tag_id: u16,
        sequence: u16,
        poll_tx: u64,
        request_rx: u64,
        response_tx: u64,
    },
}

impl Packet {
    pub const MAX_LEN: usize = 24;

    pub fn encode(self, out: &mut [u8; Self::MAX_LEN]) -> usize {
        out.fill(0);
        out[0] = MAGIC;
        out[1] = VERSION;

        match self {
            Self::Sync { sequence } => {
                out[2] = PacketKind::Sync as u8;
                write_u16(&mut out[3..5], sequence);
                5
            }
            Self::Poll {
                anchor_id,
                tag_id,
                sequence,
                poll_tx,
            } => {
                write_common(out, PacketKind::Poll, anchor_id, tag_id, sequence);
                write_time(&mut out[9..14], poll_tx);
                14
            }
            Self::Request {
                anchor_id,
                tag_id,
                sequence,
                poll_rx,
                request_tx,
            } => {
                write_common(out, PacketKind::Request, anchor_id, tag_id, sequence);
                write_time(&mut out[9..14], poll_rx);
                write_time(&mut out[14..19], request_tx);
                19
            }
            Self::Response {
                anchor_id,
                tag_id,
                sequence,
                poll_tx,
                request_rx,
                response_tx,
            } => {
                write_common(out, PacketKind::Response, anchor_id, tag_id, sequence);
                write_time(&mut out[9..14], poll_tx);
                write_time(&mut out[14..19], request_rx);
                write_time(&mut out[19..24], response_tx);
                24
            }
        }
    }

    pub fn decode(data: &[u8]) -> Result<Self, PacketError> {
        if data.len() < 3 || data[0] != MAGIC || data[1] != VERSION {
            return Err(PacketError::InvalidHeader);
        }

        let kind = PacketKind::try_from(data[2])?;
        if matches!(kind, PacketKind::Sync) {
            return if data.len() == 5 {
                Ok(Self::Sync {
                    sequence: read_u16(&data[3..5]),
                })
            } else {
                Err(PacketError::InvalidLength)
            };
        }

        if data.len() < 9 {
            return Err(PacketError::InvalidLength);
        }
        let anchor_id = read_u16(&data[3..5]);
        let tag_id = read_u16(&data[5..7]);
        let sequence = read_u16(&data[7..9]);

        match kind {
            PacketKind::Poll if data.len() == 14 => Ok(Self::Poll {
                anchor_id,
                tag_id,
                sequence,
                poll_tx: read_time(&data[9..14]),
            }),
            PacketKind::Request if data.len() == 19 => Ok(Self::Request {
                anchor_id,
                tag_id,
                sequence,
                poll_rx: read_time(&data[9..14]),
                request_tx: read_time(&data[14..19]),
            }),
            PacketKind::Response if data.len() == 24 => Ok(Self::Response {
                anchor_id,
                tag_id,
                sequence,
                poll_tx: read_time(&data[9..14]),
                request_rx: read_time(&data[14..19]),
                response_tx: read_time(&data[19..24]),
            }),
            _ => Err(PacketError::InvalidLength),
        }
    }
}

/// Radio configuration shared by every node in the test network.
pub fn radio_config() -> Config {
    Config {
        channel: UwbChannel::Channel5,
        pulse_repetition_frequency: PulseRepetitionFrequency::Mhz64,
        preamble_length: PreambleLength::Symbols64,
        bitrate: BitRate::Kbps6800,
        frame_filtering: false,
        ranging_enable: true,
        ..Config::default()
    }
}

pub const fn anchor_slot(anchor_index: usize, tag_index: usize) -> u32 {
    SYNC_GUARD_US + ((tag_index * ACTIVE_ANCHOR_COUNT + anchor_index) as u32 * SLOT_US)
}

/// How long the master must wait, after finishing its own last ranging slot,
/// before issuing the next synchronization frame. Derived from
/// [`SUPERFRAME_US`] and the master's own last slot (anchor index 0) so it
/// never drifts out of sync with the anchor/tag table; a table change that
/// leaves no room for this wait fails to build rather than silently
/// colliding with slave slots.
pub const MASTER_SYNC_WAIT_US: u32 =
    SUPERFRAME_US - (anchor_slot(0, ACTIVE_TAG_COUNT - 1) + SLOT_US);

pub fn delayed_after(base: Instant, delay_us: u32) -> Instant {
    let delay = Duration::from_nanos(delay_us.saturating_mul(1_000));
    let raw = (base + delay).value() & !0x1ff;
    Instant::new(raw).expect("rounded DW3000 timestamp must be valid")
}

/// Computes the tag-side asymmetric double-sided two-way-ranging result.
pub fn distance_mm(
    poll_tx: u64,
    poll_rx: u64,
    request_tx: u64,
    request_rx: u64,
    response_tx: u64,
    response_rx: u64,
) -> Option<u32> {
    let round_anchor = elapsed(request_rx, poll_tx) as i128;
    let reply_anchor = elapsed(response_tx, request_rx) as i128;
    let round_tag = elapsed(response_rx, request_tx) as i128;
    let reply_tag = elapsed(request_tx, poll_rx) as i128;
    let denominator = round_anchor + reply_anchor + round_tag + reply_tag;
    if denominator <= 0 {
        return None;
    }

    let tof = (round_anchor * round_tag - reply_anchor * reply_tag) / denominator;
    if tof < 0 {
        return None;
    }
    let millimetres = tof * MM_PER_DWT_NUM / MM_PER_DWT_DEN;
    u32::try_from(millimetres).ok()
}

fn elapsed(later: u64, earlier: u64) -> u64 {
    later.wrapping_sub(earlier) & TIME_MASK
}

fn write_common(
    out: &mut [u8; Packet::MAX_LEN],
    kind: PacketKind,
    anchor: u16,
    tag: u16,
    seq: u16,
) {
    out[2] = kind as u8;
    write_u16(&mut out[3..5], anchor);
    write_u16(&mut out[5..7], tag);
    write_u16(&mut out[7..9], seq);
}

fn write_u16(out: &mut [u8], value: u16) {
    out.copy_from_slice(&value.to_le_bytes());
}

fn read_u16(input: &[u8]) -> u16 {
    u16::from_le_bytes([input[0], input[1]])
}

fn write_time(out: &mut [u8], value: u64) {
    out[..TIME_BYTES].copy_from_slice(&(value & TIME_MASK).to_le_bytes()[..TIME_BYTES]);
}

fn read_time(input: &[u8]) -> u64 {
    let mut bytes = [0u8; 8];
    bytes[..TIME_BYTES].copy_from_slice(&input[..TIME_BYTES]);
    u64::from_le_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_round_trip() {
        let packet = Packet::Response {
            anchor_id: ANCHOR_IDS[2],
            tag_id: TAG_IDS[4],
            sequence: 42,
            poll_tx: 0x0123_4567_89,
            request_rx: 0x0ABC_DEF0_12,
            response_tx: 0x0000_0000_01,
        };
        let mut buffer = [0; Packet::MAX_LEN];
        let len = packet.encode(&mut buffer);
        assert_eq!(Packet::decode(&buffer[..len]), Ok(packet));
    }

    #[test]
    fn distance_uses_double_sided_formula() {
        // Both clocks advance at the same rate. The propagation time is 500
        // DW3000 units in each direction.
        let distance = distance_mm(10_000, 10_500, 30_500, 31_000, 51_000, 51_500);
        assert_eq!(distance, Some(2));
    }

    #[test]
    fn distance_handles_timestamp_wrap() {
        let before_wrap = TIME_MAX - 10_000;
        let distance = distance_mm(
            before_wrap,
            (before_wrap + 500) & TIME_MASK,
            (before_wrap + 20_500) & TIME_MASK,
            (before_wrap + 21_000) & TIME_MASK,
            (before_wrap + 41_000) & TIME_MASK,
            (before_wrap + 41_500) & TIME_MASK,
        );
        assert!(distance.is_some());
    }
}
