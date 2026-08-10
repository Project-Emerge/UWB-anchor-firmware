//! Wire protocol and timing used by the DW3000 anchor/tag test network.
//!
//! The network is intentionally statically provisioned. One anchor is the
//! timing master; it transmits a synchronization packet once per superframe.
//! Every anchor then owns one deterministic slot per tag. This keeps the RF
//! medium collision-free without requiring a second radio or a host link.

use dw3000_ng::{
    configs::{BitRate, PreambleLength, PulseRepetitionFrequency, UwbChannel},
    time::{Duration, Instant, TIME_MAX},
    Config, Ready, DW3000,
};
use embedded_hal_async::spi::SpiDevice;

pub const PAN_ID: u16 = 0x0D57;
pub const MASTER_ANCHOR_ID: u16 = ANCHOR_IDS[0];
/// Sized to the anchors actually installed: every provisioned-but-absent node
/// still costs a full slot per superframe, so this table directly sets the
/// update rate. Keep at least three for trilateration.
pub const ANCHOR_IDS: [u16; 4] = [0xA001, 0xA002, 0xA003, 0xA004];
pub const TAG_IDS: [u16; 12] = [
    0xB001, 0xB002, 0xB003, 0xB004, 0xB005, 0xB006, 0xB007, 0xB008, 0xB009, 0xB00A, 0xB00B, 0xB00C,
];

pub const ACTIVE_ANCHOR_COUNT: usize = ANCHOR_IDS.len();
pub const ACTIVE_TAG_COUNT: usize = TAG_IDS.len();

pub const SYNC_GUARD_US: u32 = 5_000;

/// Every timing below is dominated by SPI turnaround, not by time of flight.
/// `dw3000-ng` moves the TX and RX frame buffers as 129-byte block transfers
/// and brackets each exchange with a dozen smaller register accesses and two
/// busy-wait loops, so several milliseconds pass between a frame arriving and
/// the reply being programmed - even at 4 MHz.
///
/// If a delayed send is scheduled for an instant that has already passed the
/// DW3000 does not transmit at all: `TXFRS` never fires, so the symptom is a
/// TX IRQ timeout rather than an error. `REPLY_DELAY_US` therefore has to stay
/// above the real turnaround, which the firmware measures and logs at debug
/// level ("reply setup" / "response setup").
///
/// Turnaround measured 1854 us with a 20 MHz core and 4 MHz SPI. The core now
/// runs at 80 MHz and SPI at 20 MHz after init, which should bring it to
/// roughly 400-600 us; 2000 us is sized against a pessimistic 930 us so the
/// link stays reliable even if the speed-up is smaller than expected.
/// Overrunning it is loud rather than silent - it surfaces as the
/// `Poll/Response/Ranging request TX failed` warnings, which stay visible at
/// the default log level - so measure the real figure with `DEFMT_LOG=debug`
/// ("reply setup" / "response setup") and tighten from there.
///
/// A slot has to hold the whole exchange: poll, the tag's reply one
/// `REPLY_DELAY_US` later, and the anchor's response another `REPLY_DELAY_US`
/// after that. It also has to exceed that total so a tag finishes with one
/// anchor before the next anchor's poll for the same tag arrives.
pub const SLOT_US: u32 = 5_500;
pub const REPLY_DELAY_US: u32 = 2_000;
pub const RX_TIMEOUT_US: u32 = 3_000;

/// DW3000 clock ticks per microsecond: the counter runs at 499.2 MHz * 128.
pub const TICKS_PER_US: u64 = 63_898;

/// Antenna delay, in DW3000 clock ticks, applied identically on every node.
///
/// This compensates the signal's travel through the RF front end and antenna,
/// which is *not* part of the over-the-air flight but is included in the
/// hardware timestamps. Leaving it at zero inflates every measurement by the
/// internal delay: at 4.69 mm per tick, ~16385 ticks is roughly 77 m of
/// phantom distance, which is why ranging "worked" but read nonsense.
///
/// 16385 is the value Qorvo's DW3000 examples ship for the 64 MHz PRF
/// configuration this network uses ([`radio_config`]), and it is the correct
/// starting point for a DWM3000 module. It is a *nominal* figure, not a
/// calibrated one: the true delay varies per board and antenna, so expect a
/// residual constant bias of up to a few tens of centimetres. Calibrate by
/// measuring a known distance and adjusting - one tick moves the result by
/// about 4.69 mm, and because the delay applies to both ends of the link, a
/// change of N ticks here shifts the reported distance by roughly 2 * N ticks.
///
/// Both ends must use the same values, so keep these shared rather than
/// duplicating them per binary.
pub const TX_ANTENNA_DELAY: u16 = 16_385;
pub const RX_ANTENNA_DELAY: u16 = 16_385;

/// Derived from the anchor/tag table so the two always stay consistent: a
/// leading guard, one `SLOT_US` DS-TWR slot per anchor/tag combination, and a
/// trailing guard before the next synchronization frame.
///
/// This is also the position update rate, and it is the table sizes - not the
/// timings - that dominate it: the slot count is `anchors * tags`, so every
/// provisioned-but-absent node costs a full slot per superframe. Sizing
/// [`ANCHOR_IDS`] and [`TAG_IDS`] to what is actually deployed is by far the
/// cheapest way to go faster. At `SLOT_US` = 5.5 ms, with four anchors:
///
/// | tags | slots | superframe | rate    |
/// |------|-------|------------|---------|
/// | 12   | 48    | 274 ms     | 3.6 Hz  |
/// | 6    | 24    | 142 ms     | 7.0 Hz  |
/// | 4    | 16    | 98 ms      | 10.2 Hz |
/// | 2    | 8     | 54 ms      | 18.5 Hz |
///
/// Trilateration needs at least three anchors in range, so keep the anchor
/// table at three or more; shrink the tag table to the number of robots that
/// are genuinely deployed.
/// Shrinking either table (e.g. four anchors) automatically shortens the
/// superframe; if the tables grow enough that a superframe would overflow
/// `u32`, this const-evaluates to a build error instead of silently wrapping.
pub const SUPERFRAME_US: u32 =
    SYNC_GUARD_US + (ACTIVE_ANCHOR_COUNT * ACTIVE_TAG_COUNT) as u32 * SLOT_US + SYNC_GUARD_US;

const MAGIC: u8 = 0xD3;
const VERSION: u8 = 1;
const TIME_BYTES: usize = 5;
const TIME_MASK: u64 = TIME_MAX;
/// Nanometres of flight per DW3000 clock tick: the counter runs at
/// 499.2 MHz * 128, so one tick is 15.65 ps, i.e. 4.691764 mm at the speed of
/// light. The numerator is therefore in *nanometres* and the denominator must
/// scale it to millimetres - dividing by 1e9 instead yields metres from a
/// function named `distance_mm`, under-reporting every distance by 1000x.
const MM_PER_DWT_NUM: i128 = 4_691_764;
const MM_PER_DWT_DEN: i128 = 1_000_000;

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

    /// Decodes a packet from a received frame payload.
    ///
    /// Lengths are checked as lower bounds, not exact matches: `dw3000-ng`
    /// appends a footer byte and a pad byte to every frame it sends, and its
    /// `r_wait` reports `RXFLEN` (which includes the two-octet CRC) without
    /// trimming it, so `Ieee802154Frame::payload()` always hands back four
    /// bytes more than were encoded. Every field sits at a fixed offset, so
    /// ignoring the trailing bytes is safe - requiring an exact length instead
    /// rejected every frame on every node.
    pub fn decode(data: &[u8]) -> Result<Self, PacketError> {
        if data.len() < 3 || data[0] != MAGIC || data[1] != VERSION {
            return Err(PacketError::InvalidHeader);
        }

        let kind = PacketKind::try_from(data[2])?;
        if matches!(kind, PacketKind::Sync) {
            return if data.len() >= 5 {
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
            PacketKind::Poll if data.len() >= 14 => Ok(Self::Poll {
                anchor_id,
                tag_id,
                sequence,
                poll_tx: read_time(&data[9..14]),
            }),
            PacketKind::Request if data.len() >= 19 => Ok(Self::Request {
                anchor_id,
                tag_id,
                sequence,
                poll_rx: read_time(&data[9..14]),
                request_tx: read_time(&data[14..19]),
            }),
            PacketKind::Response if data.len() >= 24 => Ok(Self::Response {
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

/// Reports and clears any latched RX event in `SYS_STATUS`, naming the first
/// error found.
///
/// The DW3000 drives its IRQ pin from the *level* of `SYS_STATUS & SYS_ENABLE`,
/// and `dw3000-ng` never clears the RX flags outside `r_wait`'s success path:
/// its error paths return early, and `finish_receiving` only forces the radio
/// idle (unlike `finish_sending`, which also calls `reset_flags`). A single
/// latched RX event - such as the SFD or preamble timeout that happens
/// routinely whenever the receiver is armed in the middle of someone else's
/// transmission - therefore holds IRQ high forever, so `wait_for_rising_edge`
/// never fires again and the node stays deaf for good. Callers must run this
/// after every receive so the line is released before the next receive *or
/// transmit* waits on an edge.
pub async fn take_rx_error<SPI>(radio: &mut DW3000<SPI, Ready>) -> Option<&'static str>
where
    SPI: SpiDevice<u8>,
{
    let error = match radio.ll().sys_status().read().await {
        Ok(status) => {
            if status.rxphe() == 0b1 {
                Some("PHY header error")
            } else if status.rxfce() == 0b1 {
                Some("FCS error")
            } else if status.rxfsl() == 0b1 {
                Some("Reed-Solomon sync loss")
            } else if status.rxsto() == 0b1 {
                Some("SFD timeout")
            } else if status.rxpto() == 0b1 {
                Some("preamble timeout")
            } else if status.rxfto() == 0b1 {
                Some("frame wait timeout")
            } else if status.rxovrr() == 0b1 {
                Some("receiver overrun")
            } else {
                None
            }
        }
        Err(_) => Some("SYS_STATUS read failed"),
    };

    // `SYS_STATUS` is write-1-to-clear, and `write` leaves the bits it doesn't
    // mention at zero, so this clears exactly the RX events - the same set
    // `r_wait` clears when it succeeds.
    if radio
        .ll()
        .sys_status()
        .write(|w| {
            w.rxprd(0b1)
                .rxsfdd(0b1)
                .ciadone(0b1)
                .rxphd(0b1)
                .rxphe(0b1)
                .rxfr(0b1)
                .rxfcg(0b1)
                .rxfce(0b1)
                .rxfsl(0b1)
                .rxfto(0b1)
                .ciaerr(0b1)
                .rxovrr(0b1)
                .rxpto(0b1)
                .rxsto(0b1)
                .rxprej(0b1)
        })
        .await
        .is_err()
    {
        return Some("SYS_STATUS clear failed");
    }

    error
}

/// Microseconds of radio time elapsed since `since`, per the DW3000's own
/// clock.
///
/// Used to measure the true RX-to-TX turnaround: the deadline for a delayed
/// reply is set relative to a hardware receive timestamp, so it is the radio's
/// clock - not the MCU's - that decides whether the reply is programmed in
/// time. Returns `None` if `SYS_TIME` cannot be read.
pub async fn elapsed_since_us<SPI>(radio: &mut DW3000<SPI, Ready>, since: Instant) -> Option<u32>
where
    SPI: SpiDevice<u8>,
{
    // SYS_TIME holds the top 32 bits of the same 40-bit counter the receive
    // timestamps come from, which is why DX_TIME is programmed as `value >> 8`.
    let now = (radio.sys_time().await.ok()? as u64) << 8;
    let ticks = now.wrapping_sub(since.value()) & TIME_MASK;
    Some((ticks / TICKS_PER_US) as u32)
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

/// The timestamp the DW3000 will report for a delayed transmission programmed
/// at `scheduled`.
///
/// `DX_TIME` says when the frame leaves the digital transmitter, but the
/// timestamp the radio reports - the one ranging must use - is that instant
/// plus the configured TX antenna delay. This matters only where a timestamp
/// has to be written into the frame being sent, since the true value cannot be
/// read back until after transmission; everywhere else, prefer the `Instant`
/// that `s_wait` returns.
///
/// Feeding the programmed instant into the DS-TWR maths instead skews three of
/// its four terms - `poll_tx` already comes from `s_wait` and stays correct -
/// and with reply delays of several milliseconds inflates the result by about
/// three quarters of the antenna delay: roughly 58 m at the nominal 16385
/// ticks, regardless of the true distance.
pub fn delayed_tx_timestamp(scheduled: Instant) -> u64 {
    (scheduled.value() + TX_ANTENNA_DELAY as u64) & TIME_MASK
}

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

    /// `dw3000-ng` hands back four bytes more than were encoded (footer, pad
    /// and the untrimmed CRC), so decoding must look past them. Requiring an
    /// exact length here silently dropped every frame on every node.
    #[test]
    fn decode_ignores_driver_trailing_bytes() {
        const DRIVER_EXCESS: usize = 4;
        for packet in [
            Packet::Sync { sequence: 9 },
            Packet::Poll {
                anchor_id: ANCHOR_IDS[0],
                tag_id: TAG_IDS[0],
                sequence: 9,
                poll_tx: 0x0102_0304_05,
            },
            Packet::Request {
                anchor_id: ANCHOR_IDS[1],
                tag_id: TAG_IDS[2],
                sequence: 9,
                poll_rx: 0x0000_0000_11,
                request_tx: 0x0000_0000_22,
            },
            Packet::Response {
                anchor_id: ANCHOR_IDS[2],
                tag_id: TAG_IDS[3],
                sequence: 9,
                poll_tx: 0x0000_0000_33,
                request_rx: 0x0000_0000_44,
                response_tx: 0x0000_0000_55,
            },
        ] {
            let mut buffer = [0; Packet::MAX_LEN];
            let len = packet.encode(&mut buffer);
            // Non-zero trailing bytes so the test fails if they leak into a field.
            let mut framed = [0xAA; Packet::MAX_LEN + DRIVER_EXCESS];
            framed[..len].copy_from_slice(&buffer[..len]);
            assert_eq!(Packet::decode(&framed[..len + DRIVER_EXCESS]), Ok(packet));
        }
    }

    #[test]
    fn decode_still_rejects_truncated_frames() {
        let mut buffer = [0; Packet::MAX_LEN];
        let len = Packet::Response {
            anchor_id: ANCHOR_IDS[0],
            tag_id: TAG_IDS[0],
            sequence: 1,
            poll_tx: 1,
            request_rx: 2,
            response_tx: 3,
        }
        .encode(&mut buffer);
        assert_eq!(
            Packet::decode(&buffer[..len - 1]),
            Err(PacketError::InvalidLength)
        );
    }

    #[test]
    fn distance_uses_double_sided_formula() {
        // Both clocks advance at the same rate. The propagation time is 500
        // DW3000 units in each direction, and one unit is 4.691764 mm of
        // flight, so 500 units is 2345 mm after truncation.
        let distance = distance_mm(10_000, 10_500, 30_500, 31_000, 51_000, 51_500);
        assert_eq!(distance, Some(2345));
    }

    /// Guards the millimetre scale. `MM_PER_DWT_NUM` is nanometres per tick, so
    /// an over-large denominator silently makes `distance_mm` return metres -
    /// which reads as a plausible small integer rather than an obvious error.
    /// The geometry below is a symmetric exchange with a 20000-tick reply delay
    /// where the time of flight resolves to 213 ticks, i.e. very close to 1 m.
    #[test]
    fn one_metre_of_flight_reads_about_one_metre() {
        let distance = distance_mm(0, 213, 20_213, 20_426, 40_426, 40_639)
            .expect("symmetric geometry must yield a distance");
        assert!(
            (950..=1_050).contains(&distance),
            "expected about 1000 mm, got {distance}"
        );
    }

    /// A delayed transmission is reported at `DX_TIME + TX_ANTENNA_DELAY`, so
    /// using the programmed instant skews `reply_anchor`, `round_tag` and
    /// `reply_tag` while `poll_tx` - taken from `s_wait` - stays correct. At
    /// millisecond reply delays that inflates the result by about three
    /// quarters of the antenna delay, independent of the true distance.
    #[test]
    fn programmed_transmit_times_would_inflate_distance() {
        const D: u64 = TX_ANTENNA_DELAY as u64;
        let reply = u64::from(REPLY_DELAY_US) * TICKS_PER_US;
        let tof = 53; // ~25 cm

        let poll_tx = 0;
        let poll_rx = poll_tx + tof;
        let request_tx = poll_rx + reply;
        let request_rx = request_tx + tof;
        let response_tx = request_rx + reply;
        let response_rx = response_tx + tof;

        let correct = distance_mm(
            poll_tx,
            poll_rx,
            request_tx,
            request_rx,
            response_tx,
            response_rx,
        )
        .expect("true timestamps must range");
        assert!(correct < 300, "expected about 250 mm, got {correct}");

        // The programmed instants are the reported ones minus the antenna delay.
        let inflated = distance_mm(
            poll_tx,
            poll_rx,
            request_tx - D,
            request_rx,
            response_tx - D,
            response_rx,
        )
        .expect("skewed timestamps still range");
        assert!(
            (55_000..60_000).contains(&inflated),
            "expected the ~58 m bias this bug produced, got {inflated}"
        );
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
