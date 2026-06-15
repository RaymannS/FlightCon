#pragma once
#include <Arduino.h>
#include <stdint.h>

// Binary telemetry packet — 31 bytes, hex-encoded to 62 chars for LoRa.
//
// Bytes  0- 3: ts_ms    uint32  LE  milliseconds since boot
// Byte   4:    flags
//                bits[7:5] state    (0=PAD 1=BOOST 2=COAST 3=DESCENDING 4=DESCENDED)
//                bit[4]    airbrake
//                bit[3]    baro_ok
//                bit[2]    imu_ok
//                bits[1:0] reserved
// Bytes  5- 8: alt_kf   float32 LE  metres
// Bytes  9-12: vel_kf   float32 LE  m/s
// Bytes 13-16: baro_alt float32 LE  metres
// Bytes 17-20: accel_y  float32 LE  m/s²
// Bytes 21-22: tilt     int16   LE  degrees × 100
// Bytes 23-24: roll     int16   LE  degrees × 100
// Bytes 25-26: pitch    int16   LE  degrees × 100
// Bytes 27-28: yaw      uint16  LE  degrees × 100  (uint to hold 0–360°)
// Bytes 29-30: temp     int16   LE  °C      × 100

static constexpr size_t TELEM_PACKET_BYTES = 31;

// Pack one telemetry frame into buf (must be TELEM_PACKET_BYTES).
// Pass buf to telem_buf_to_hex for LoRa and logger_record for storage.
void telem_pack_buf(uint8_t*  buf,
                    uint32_t  ts_ms,
                    uint8_t   state,
                    bool      airbrake,
                    bool      baro_ok,
                    bool      imu_ok,
                    float     alt_kf,
                    float     vel_kf,
                    float     baro_alt,
                    float     accel_y,
                    float     tilt_deg,
                    float     roll_deg,
                    float     pitch_deg,
                    float     yaw_deg,
                    float     temp_c);

// Hex-encode a raw byte buffer into a String for LoRa transmission.
String telem_buf_to_hex(const uint8_t* buf, size_t len);
