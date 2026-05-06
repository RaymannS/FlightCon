/**
 * bar.cpp — BMP280 Barometer driver for ESP32 WROOM-32E
 *
 * Bus   : I2C1 (separate from IMU I2C0 on GPIO 25/26)
 * Addr  : 0x76  (SDO → GND)  |  0x77  (SDO → 3.3V)
 * Lib   : Adafruit BMP280  (install via Library Manager or platformio.ini)
 *
 * platformio.ini dependency:
 *   lib_deps =
 *       adafruit/Adafruit BMP280 Library @ ^2.6.8
 *       adafruit/Adafruit Unified Sensor @ ^1.1.14
 */

#include "bar.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// ─── Internal state ──────────────────────────────────────────────────────────

static Adafruit_BMP280 _bmp(&Wire1);
static bool            _initialised          = false;
static float           _ground_pressure_hpa  = 0.0f;  // set by bar_calibrate()

// ─── Helpers ─────────────────────────────────────────────────────────────────

static float _not_ready(const char *fn)
{
    Serial.printf("[BAR] %s called before successful bar_init()\n", fn);
    return -999.0f;
}

// ─── Public implementation ───────────────────────────────────────────────────

bool bar_init(void)
{
    // Wire1 is shared with the PCA9685 servo driver (both on GPIO 18/19).
    // Only one of them should call Wire1.begin() — bar_init() owns it here.
    // servo_init() must NOT call Wire1.begin() again.
    Wire1.begin(BMP280_SDA_PIN, BMP280_SCL_PIN);

    if (!_bmp.begin(BMP280_I2C_ADDR)) {
        Serial.printf("[BAR] BMP280 not found at 0x%02X — check wiring / SDO pin\n",
                      BMP280_I2C_ADDR);
        _initialised = false;
        return false;
    }

    // ── Sampling config for Kalman filter use ────────────────────────────────
    // We trade some hardware filtering for speed, letting the Kalman filter
    // handle noise reduction instead.
    //
    // STANDBY_MS_1 + SAMPLING_X4 pressure → ~80 Hz output rate
    // (STANDBY_MS_0_5 only exists in the BME280 library, not BMP280)
    // FILTER_X4 keeps light IIR hardware filtering to reject spikes
    //
    // If you are NOT using the Kalman filter, revert to:
    //   SAMPLING_X16, FILTER_X16, STANDBY_MS_500  (~26 Hz, smoother)
    _bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,       // Continuous measurement
        Adafruit_BMP280::SAMPLING_X16,       // Temperature oversampling x2
        Adafruit_BMP280::SAMPLING_X16,       // Pressure oversampling x4 (~80Hz)
        Adafruit_BMP280::FILTER_X16,         // Light IIR filter — Kalman handles rest
        Adafruit_BMP280::STANDBY_MS_500       // 1ms standby — fastest available in BMP280 library
    );

    _initialised = true;
    Serial.printf("[BAR] BMP280 initialised at 0x%02X (SDA=%d SCL=%d)\n",
                  BMP280_I2C_ADDR, BMP280_SDA_PIN, BMP280_SCL_PIN);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

float bar_calibrate(uint16_t samples, uint32_t duration_ms)
{
    if (!_initialised) {
        Serial.println("[BAR] bar_calibrate() called before bar_init()");
        return -999.0f;
    }

    Serial.printf("[BAR] Calibrating ground pressure — sampling for %lu ms...\n",
                  (unsigned long)duration_ms);

    uint32_t interval_ms = duration_ms / samples;
    double   sum         = 0.0;
    uint16_t count       = 0;

    for (uint16_t i = 0; i < samples; i++) {
        float p = _bmp.readPressure() / 100.0f;  // Pa → hPa
        if (p > 0.0f) {
            sum += p;
            count++;
        }
        delay(interval_ms);
    }

    if (count == 0) {
        Serial.println("[BAR] Calibration FAILED — no valid pressure samples");
        return -999.0f;
    }

    _ground_pressure_hpa = (float)(sum / count);
    Serial.printf("[BAR] Calibration complete — ground pressure: %.4f hPa (%d/%d samples good)\n",
                  _ground_pressure_hpa, count, samples);
    return _ground_pressure_hpa;
}

// ─────────────────────────────────────────────────────────────────────────────

float bar_get_ground_pressure(void)
{
    return _ground_pressure_hpa;
}

// ─────────────────────────────────────────────────────────────────────────────

float bar_get_temperature(void)
{
    if (!_initialised) return _not_ready(__func__);
    return _bmp.readTemperature();
}

float bar_get_pressure(void)
{
    if (!_initialised) return _not_ready(__func__);
    return _bmp.readPressure() / 100.0f;
}

float bar_get_altitude(float sea_level_hpa)
{
    if (!_initialised) return _not_ready(__func__);

    float pressure_hpa = _bmp.readPressure() / 100.0f;
    if (pressure_hpa <= 0.0f) return -999.0f;

    return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.1902949f));
}

float bar_get_altitude_agl(void)
{
    if (!_initialised) return _not_ready(__func__);
    if (_ground_pressure_hpa <= 0.0f) {
        Serial.println("[BAR] bar_get_altitude_agl() called before bar_calibrate()");
        return -999.0f;
    }
    return bar_get_altitude(_ground_pressure_hpa);
}

// ─────────────────────────────────────────────────────────────────────────────

bool bar_read_all(float &temp_c,
                  float &pressure_hpa,
                  float &altitude_m)
{
    if (!_initialised) {
        _not_ready(__func__);
        return false;
    }

    temp_c       = _bmp.readTemperature();
    pressure_hpa = _bmp.readPressure() / 100.0f;

    if (pressure_hpa <= 0.0f) {
        altitude_m = -999.0f;
        return false;
    }

    // Always use calibrated ground pressure for AGL.
    // Falls back to ISA standard if bar_calibrate() was never called.
    float ref = (_ground_pressure_hpa > 0.0f) ? _ground_pressure_hpa : 1013.25f;
    altitude_m = 44330.0f * (1.0f - powf(pressure_hpa / ref, 0.1902949f));
    return true;
}