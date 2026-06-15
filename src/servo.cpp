#include "servo.h"
#include "verbose.h"

// ─── Internal State ──────────────────────────────────────────────────────────

// Use Wire1 for servo (shared I2C bus with BMP280, different addresses)
static Adafruit_PWMServoDriver _pca9685(PCA9685_I2C_ADDR, Wire1);
static bool     _initialised          = false;
static float    _current_angle[SERVO_CHANNEL_COUNT];  // cache for servo_sweep

// ─── Helpers ─────────────────────────────────────────────────────────────────

/**
 * Convert an angle in degrees to a 12-bit PCA9685 tick count.
 *
 * The PCA9685 runs at SERVO_FREQ_HZ.  Each PWM period is divided into
 * 4096 ticks.  We map the requested pulse width (derived from the angle)
 * onto that tick range.
 *
 *   tick = pulse_us / period_us * 4096
 *        = pulse_us * freq / 1 000 000 * 4096
 */
static uint16_t _angle_to_tick(float degrees)
{
    degrees = constrain(degrees, 0.0f, 180.0f);

    // Linear map: 0° → SERVO_MIN_US, 180° → SERVO_MAX_US
    float pulse_us = map(degrees, 0.0f, 180.0f,
                         (float)SERVO_MIN_US, (float)SERVO_MAX_US);

    // Convert µs → 12-bit tick
    float period_us = 1000000.0f / SERVO_FREQ_HZ;   // e.g. 20 000 µs at 50 Hz
    uint16_t tick   = (uint16_t)((pulse_us / period_us) * 4096.0f);

    return tick;
}

static uint16_t _us_to_tick(uint16_t pulse_us)
{
    pulse_us = (uint16_t)constrain((int)pulse_us, SERVO_MIN_US, SERVO_MAX_US);
    float period_us = 1000000.0f / SERVO_FREQ_HZ;
    return (uint16_t)((pulse_us / period_us) * 4096.0f);
}

// ─── Public Implementation ───────────────────────────────────────────────────

bool servo_init(uint8_t sda, uint8_t scl)
{
    // Initialize Wire1 (second I2C port) on pins 18/19, shared with BMP280
    // Both devices have different addresses: servo at 0x40, BMP280 at 0x76
    // Wire1.begin(sda, scl);

    _pca9685.begin();

    // Check the device is on the bus by reading its oscillator
    // Adafruit's begin() will hang if the device is absent, so we probe first.
    Wire1.beginTransmission(PCA9685_I2C_ADDR);
    if (Wire1.endTransmission() != 0) {
        Serial.printf("[servo] ERROR: PCA9685 not found at 0x%02X\n",
                      PCA9685_I2C_ADDR);
        return false;
    }

    _pca9685.setOscillatorFrequency(27000000);  // Typical internal oscillator
    _pca9685.setPWMFreq(SERVO_FREQ_HZ);

    // Initialise angle cache and zero all channels
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        _current_angle[ch] = 90.0f;
        _pca9685.setPWM(ch, 0, 0);              // Outputs off
    }

    _initialised = true;
    VLOGF("[servo] PCA9685 ready — SDA:%d SCL:%d addr:0x%02X freq:%dHz\n",
          sda, scl, PCA9685_I2C_ADDR, SERVO_FREQ_HZ);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_set_angle(uint8_t channel, float degrees)
{
    if (!_initialised) {
        Serial.println("[servo] WARN: servo_set_angle() called before servo_init()");
        return;
    }
    if (channel >= SERVO_CHANNEL_COUNT) {
        Serial.printf("[servo] WARN: channel %d out of range (0–%d)\n",
                      channel, SERVO_CHANNEL_COUNT - 1);
        return;
    }

    degrees = constrain(degrees, 0.0f, 180.0f);
    _current_angle[channel] = degrees;

    uint16_t tick = _angle_to_tick(degrees);
    _pca9685.setPWM(channel, 0, tick);

    VLOGF("[servo] ch%02d → %.1f° (tick %d)\n", channel, degrees, tick);
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    if (!_initialised) {
        Serial.println("[servo] WARN: servo_set_pulse_us() called before servo_init()");
        return;
    }
    if (channel >= SERVO_CHANNEL_COUNT) {
        Serial.printf("[servo] WARN: channel %d out of range\n", channel);
        return;
    }

    pulse_us = (uint16_t)constrain((int)pulse_us, SERVO_MIN_US, SERVO_MAX_US);

    // Back-calculate angle for the cache
    _current_angle[channel] = map((float)pulse_us,
                                  (float)SERVO_MIN_US, (float)SERVO_MAX_US,
                                  0.0f, 180.0f);

    uint16_t tick = _us_to_tick(pulse_us);
    _pca9685.setPWM(channel, 0, tick);

    VLOGF("[servo] ch%02d → %dµs (tick %d)\n", channel, pulse_us, tick);
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_release(uint8_t channel)
{
    if (!_initialised) return;
    if (channel >= SERVO_CHANNEL_COUNT) return;

    _pca9685.setPWM(channel, 0, 0);
    VLOGF("[servo] ch%02d released\n", channel);
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_release_all()
{
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        servo_release(ch);
    }
    VLOG("[servo] all channels released");
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_sweep(uint8_t  channel,
                 float    from_deg,
                 float    to_deg,
                 uint32_t duration_ms,
                 uint16_t steps)
{
    if (!_initialised) return;
    if (steps == 0) steps = 1;

    uint32_t step_delay_ms = duration_ms / steps;
    float    delta         = (to_deg - from_deg) / (float)steps;

    VLOGF("[servo] ch%02d sweep %.1f° → %.1f° over %lums (%d steps)\n",
          channel, from_deg, to_deg, duration_ms, steps);

    for (uint16_t i = 0; i <= steps; i++) {
        float angle = from_deg + delta * (float)i;
        servo_set_angle(channel, angle);
        delay(step_delay_ms);
    }
}