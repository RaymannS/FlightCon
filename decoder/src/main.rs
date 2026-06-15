// FlightCon telemetry decoder — reads hex packets from stdin, one per line.
//
// Accepts raw 54-char hex or RYLR998 +RCV frames:
//   echo "5C00..." | cargo run
//   cat /dev/ttyUSB0 | cargo run
//
// Packet layout (27 bytes, hex-encoded to 54 chars):
//   Byte  0:     flags — state[7:5] airbrake[4] baro_ok[3] imu_ok[2]
//   Bytes  1- 4: alt_kf   f32 LE  metres
//   Bytes  5- 8: vel_kf   f32 LE  m/s
//   Bytes  9-12: baro_alt f32 LE  metres
//   Bytes 13-16: accel_y  f32 LE  m/s²  (full float32 — near-Mach accuracy)
//   Bytes 17-18: tilt     i16 LE  degrees × 100
//   Bytes 19-20: roll     i16 LE  degrees × 100
//   Bytes 21-22: pitch    i16 LE  degrees × 100
//   Bytes 23-24: yaw      u16 LE  degrees × 100
//   Bytes 25-26: temp     i16 LE  °C      × 100

use std::io::{self, BufRead};

const PACKET_BYTES: usize = 27;

// ─── Flight state ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
enum FlightState {
    Pad,
    Boost,
    Coast,
    Descending,
    Descended,
}

impl FlightState {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Pad),
            1 => Some(Self::Boost),
            2 => Some(Self::Coast),
            3 => Some(Self::Descending),
            4 => Some(Self::Descended),
            _ => None,
        }
    }
}

impl std::fmt::Display for FlightState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Pad        => "PAD",
            Self::Boost      => "BOOST",
            Self::Coast      => "COAST",
            Self::Descending => "DESCENDING",
            Self::Descended  => "DESCENDED",
        })
    }
}

// ─── Telemetry frame ──────────────────────────────────────────────────────────

#[derive(Debug)]
struct Frame {
    state:    FlightState,
    airbrake: bool,
    baro_ok:  bool,
    imu_ok:   bool,
    alt_kf:   f32,   // m
    vel_kf:   f32,   // m/s
    baro_alt: f32,   // m
    accel_y:  f32,   // m/s²
    tilt:     f32,   // deg
    roll:     f32,   // deg
    pitch:    f32,   // deg
    yaw:      f32,   // deg
    temp:     f32,   // °C
}

impl std::fmt::Display for Frame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:<10} ab={} ba={} im={} | \
             altKF={:7.1}m vel={:7.2}m/s baroAlt={:7.1}m | \
             accY={:9.3}m/s² tilt={:5.1}° roll={:6.1}° pitch={:6.1}° yaw={:6.1}° | \
             temp={:.1}°C",
            self.state.to_string(),
            self.airbrake as u8, self.baro_ok as u8, self.imu_ok as u8,
            self.alt_kf,  self.vel_kf, self.baro_alt,
            self.accel_y, self.tilt,   self.roll, self.pitch, self.yaw,
            self.temp,
        )
    }
}

// ─── Decode logic ─────────────────────────────────────────────────────────────

fn hex_nibble(b: u8) -> Result<u8, String> {
    match b {
        b'0'..=b'9' => Ok(b - b'0'),
        b'a'..=b'f' => Ok(b - b'a' + 10),
        b'A'..=b'F' => Ok(b - b'A' + 10),
        _ => Err(format!("invalid hex char '{}'", b as char)),
    }
}

fn parse_hex(hex: &str) -> Result<[u8; PACKET_BYTES], String> {
    let hex = hex.trim();
    if hex.len() != PACKET_BYTES * 2 {
        return Err(format!(
            "expected {} hex chars, got {}",
            PACKET_BYTES * 2,
            hex.len()
        ));
    }
    let mut buf = [0u8; PACKET_BYTES];
    for (i, chunk) in hex.as_bytes().chunks_exact(2).enumerate() {
        let hi = hex_nibble(chunk[0])?;
        let lo = hex_nibble(chunk[1])?;
        buf[i] = (hi << 4) | lo;
    }
    Ok(buf)
}

fn decode(buf: &[u8; PACKET_BYTES]) -> Result<Frame, String> {
    let flags     = buf[0];
    let state_raw = (flags >> 5) & 0x07;

    let state = FlightState::from_u8(state_raw)
        .ok_or_else(|| format!("unknown state value {state_raw}"))?;

    // Slices are exactly 4 or 2 bytes — unwrap() cannot panic here.
    Ok(Frame {
        state,
        airbrake: flags & 0x10 != 0,
        baro_ok:  flags & 0x08 != 0,
        imu_ok:   flags & 0x04 != 0,
        alt_kf:   f32::from_le_bytes(buf[1..5].try_into().unwrap()),
        vel_kf:   f32::from_le_bytes(buf[5..9].try_into().unwrap()),
        baro_alt: f32::from_le_bytes(buf[9..13].try_into().unwrap()),
        accel_y:  f32::from_le_bytes(buf[13..17].try_into().unwrap()),
        tilt:     i16::from_le_bytes(buf[17..19].try_into().unwrap()) as f32 / 100.0,
        roll:     i16::from_le_bytes(buf[19..21].try_into().unwrap()) as f32 / 100.0,
        pitch:    i16::from_le_bytes(buf[21..23].try_into().unwrap()) as f32 / 100.0,
        yaw:      u16::from_le_bytes(buf[23..25].try_into().unwrap()) as f32 / 100.0,
        temp:     i16::from_le_bytes(buf[25..27].try_into().unwrap()) as f32 / 100.0,
    })
}

// ─── RYLR998 frame parser ─────────────────────────────────────────────────────
// The receiver module wraps payloads: +RCV=<sender>,<len>,<data>,<rssi>,<snr>
// This pulls out just the hex data field.

fn extract_rcv_payload(line: &str) -> Option<&str> {
    let inner = line.strip_prefix("+RCV=")?;
    // fields: sender, len, data, rssi, snr
    let mut iter = inner.splitn(5, ',');
    let _sender = iter.next()?;
    let _len    = iter.next()?;
    let data    = iter.next()?;
    Some(data.trim())
}

// ─── Entry point ──────────────────────────────────────────────────────────────

fn main() {
    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let raw = match line {
            Ok(l)  => l,
            Err(e) => { eprintln!("read error: {e}"); break; }
        };

        let raw = raw.trim();
        if raw.is_empty() || raw.starts_with('#') {
            continue;
        }

        // Accept either a bare hex string or a full +RCV= frame
        let hex = extract_rcv_payload(raw).unwrap_or(raw);

        match parse_hex(hex).and_then(|buf| decode(&buf)) {
            Ok(frame) => println!("{frame}"),
            Err(e)    => eprintln!("error: {e}  (raw: {raw})"),
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Build a known packet manually and round-trip it through the decoder.
    //
    // State = COAST (2), airbrake=true, baro_ok=true, imu_ok=true
    // flags = (2<<5) | 0x10 | 0x08 | 0x04 = 0x5C
    // All float fields = 0.0  →  00 00 00 00
    // temp = 20.5°C  →  i16(2050) = 0x0802  →  LE bytes: 02 08
    // hex = "5C" + "00"*24 + "0208"
    fn test_packet() -> [u8; PACKET_BYTES] {
        let mut buf = [0u8; PACKET_BYTES];
        buf[0] = 0x5C; // COAST | airbrake | baro_ok | imu_ok
        // floats stay zero
        // temp = 2050 = 0x0802, little-endian
        buf[25] = 0x02;
        buf[26] = 0x08;
        buf
    }

    #[test]
    fn flags_decode() {
        let f = decode(&test_packet()).unwrap();
        assert_eq!(f.state,    FlightState::Coast);
        assert!(f.airbrake);
        assert!(f.baro_ok);
        assert!(f.imu_ok);
    }

    #[test]
    fn float_fields_zero() {
        let f = decode(&test_packet()).unwrap();
        assert_eq!(f.alt_kf,   0.0);
        assert_eq!(f.vel_kf,   0.0);
        assert_eq!(f.baro_alt, 0.0);
        assert_eq!(f.accel_y,  0.0);
    }

    #[test]
    fn temp_decode() {
        let f = decode(&test_packet()).unwrap();
        assert!((f.temp - 20.5).abs() < 0.01);
    }

    #[test]
    fn hex_round_trip() {
        let buf = test_packet();
        // build hex string the same way telem_pack_hex does it
        let hex: String = buf.iter()
            .flat_map(|b| [char::from_digit((b >> 4) as u32, 16).unwrap().to_ascii_uppercase(),
                           char::from_digit((b & 0xF) as u32, 16).unwrap().to_ascii_uppercase()])
            .collect();
        assert_eq!(hex.len(), PACKET_BYTES * 2);
        let decoded = parse_hex(&hex).and_then(|b| decode(&b)).unwrap();
        assert_eq!(decoded.state, FlightState::Coast);
        assert!((decoded.temp - 20.5).abs() < 0.01);
    }

    #[test]
    fn rcv_frame_strip() {
        let line = "+RCV=2,54,5C000000000000000000000000000000000000000000000000000208,-45,12";
        let payload = extract_rcv_payload(line).unwrap();
        assert_eq!(payload, "5C000000000000000000000000000000000000000000000000000208");
    }

    #[test]
    fn bad_hex_rejected() {
        assert!(parse_hex("ZZZZ").is_err());
        assert!(parse_hex("5C").is_err()); // too short
    }
}
