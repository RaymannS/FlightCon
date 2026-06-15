#include "telemetry.h"
#include <string.h>

// ─── Byte-level pack helpers ──────────────────────────────────────────────────

static void pack_u32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >>  8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void pack_f32(uint8_t* p, float v)
{
    memcpy(p, &v, 4);
}

static void pack_i16(uint8_t* p, float v, float scale, float lo, float hi)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    int16_t iv = (int16_t)(v * scale);
    p[0] = (uint8_t)(iv & 0xFF);
    p[1] = (uint8_t)((iv >> 8) & 0xFF);
}

static void pack_u16(uint8_t* p, float v, float scale, float lo, float hi)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    uint16_t uv = (uint16_t)(v * scale);
    p[0] = (uint8_t)(uv & 0xFF);
    p[1] = (uint8_t)((uv >> 8) & 0xFF);
}

// ─── Public implementation ────────────────────────────────────────────────────

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
                    float     temp_c)
{
    pack_u32(buf + 0,  ts_ms);

    buf[4] = (uint8_t)(
        ((state & 0x07u) << 5) |
        (airbrake ? 0x10u : 0u) |
        (baro_ok  ? 0x08u : 0u) |
        (imu_ok   ? 0x04u : 0u)
    );

    pack_f32(buf + 5,  alt_kf);
    pack_f32(buf + 9,  vel_kf);
    pack_f32(buf + 13, baro_alt);
    pack_f32(buf + 17, accel_y);

    pack_i16(buf + 21, tilt_deg,   100.0f,  -180.0f,  180.0f);
    pack_i16(buf + 23, roll_deg,   100.0f,  -180.0f,  180.0f);
    pack_i16(buf + 25, pitch_deg,  100.0f,   -90.0f,   90.0f);
    pack_u16(buf + 27, yaw_deg,    100.0f,    0.0f,   360.0f);
    pack_i16(buf + 29, temp_c,     100.0f,  -100.0f,  100.0f);
}

String telem_buf_to_hex(const uint8_t* buf, size_t len)
{
    static const char kHex[16] = {
        '0','1','2','3','4','5','6','7',
        '8','9','A','B','C','D','E','F'
    };

    String hex;
    hex.reserve(len * 2);
    for (size_t i = 0; i < len; i++) {
        hex += kHex[buf[i] >> 4];
        hex += kHex[buf[i] & 0x0F];
    }
    return hex;
}
