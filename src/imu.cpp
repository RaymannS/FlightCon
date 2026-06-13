/**
 * imu.cpp — BNO055 I2C driver for ESP32
 *
 * Sensor: Teyleten Robot BNO055 9-axis attitude sensor module
 * Bus:    I2C on ESP32 GPIO25 SDA, GPIO26 SCL
 * Mode:   OPERATION_MODE_IMUPLUS — accel + gyro only, no magnetometer
 *
 * Calibration:
 *   - On boot, loads saved offsets from ESP32 flash (EEPROM emulation).
 *   - Once SYS, GYRO, and ACCEL all reach 3, offsets are auto-saved to flash.
 *   - MAG is ignored — not used in IMUPLUS mode.
 *   - After a successful save, every reboot loads instantly with no
 *     recalibration needed as long as the board stays mounted the same way.
 *
 * Axis mapping:
 *   - Set kRocketAxisY = the sensor axis that points along the rocket body
 *     toward the nose. Default is sensor Y axis.
 *   - If accel_y goes negative when the rocket accelerates upward, flip
 *     kRocketAxisYSign to -1.0f.
 */

#include "imu.h"
#include "verbose.h"

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ─── I2C config ───────────────────────────────────────────────────────────────

static constexpr int kImuSdaPin = 25;
static constexpr int kImuSclPin = 26;

// I2C address — 0x28 default, 0x29 if ADD pin is high
static constexpr uint8_t kBno055Address = 0x29;

// External 32.768 kHz crystal improves accuracy if the module actually has one.
// Leave disabled for clone boards that do not populate it.
static constexpr bool kUseExternalCrystal = false;
static constexpr uint32_t kImuI2cClockHz = 100000;

// Keep debug logging from overwhelming the UART when imuRead() runs fast.
static constexpr uint32_t kImuDebugPrintPeriodMs = 1000;
static constexpr uint32_t kImuZeroStallMs = 1500;
static constexpr float kImuZeroEpsilon = 0.05f;

// ─── Axis mapping ─────────────────────────────────────────────────────────────
// Mount the BNO055 so sensor Y points toward the rocket nose.
// If AccY goes negative during upward acceleration, set kRocketAxisYSign = -1.

static constexpr int   kRocketAxisX     =  0;
static constexpr int   kRocketAxisY     =  1;    // vertical axis — Kalman input
static constexpr int   kRocketAxisZ     =  2;
static constexpr float kRocketAxisXSign    =  1.0f;
static constexpr float kRocketAxisYSign    =  1.0f; // flip to -1.0f if needed
static constexpr float kRocketAxisZSign    =  1.0f;
static constexpr float kPitchMountOffsetDeg = 90.0f; // IMU mounted 90° nose-down

// ─── EEPROM / flash calibration storage ──────────────────────────────────────
// Layout:
//   Address 0-3:   magic number (uint32_t) — confirms valid data exists
//   Address 4-25:  adafruit_bno055_offsets_t (22 bytes)

static constexpr uint32_t kCalMagic   = 0xB055CA1A;
static constexpr int      kEepromSize = 64;
static constexpr int      kMagicAddr  = 0;
static constexpr int      kCalAddr    = 4;

// ─── Internal state ───────────────────────────────────────────────────────────

static Adafruit_BNO055 bno = Adafruit_BNO055(55, kBno055Address, &Wire);
static bool _ready    = false;
static bool _calSaved = false;   // true once we've saved this session
static uint32_t _lastDebugPrintMs = 0;
static bool _haveLastGoodOrientation = false;
static float _lastGoodYaw = 0.0f;
static float _lastGoodPitch = 0.0f;
static float _lastGoodRoll = 0.0f;

// Consecutive quaternion-read failure tracking for auto-recovery
static int      _quatFailCount      = 0;
static uint32_t _lastRecoveryMs     = 0;
static constexpr int      kQuatFailRecoverThreshold = 10;
static constexpr uint32_t kMinRecoveryIntervalMs    = 2000;

static bool _loadCalibration();  // forward declaration

static void _recoverI2cBus()
{
    Wire.end();
    delay(10);

    pinMode(kImuSdaPin, INPUT_PULLUP);
    pinMode(kImuSclPin, INPUT_PULLUP);
    delay(2);

    if (digitalRead(kImuSdaPin) == LOW) {
        pinMode(kImuSclPin, OUTPUT_OPEN_DRAIN);
        digitalWrite(kImuSclPin, HIGH);
        for (int i = 0; i < 9; ++i) {
            digitalWrite(kImuSclPin, LOW);
            delayMicroseconds(5);
            digitalWrite(kImuSclPin, HIGH);
            delayMicroseconds(5);
        }
    }

    pinMode(kImuSdaPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(kImuSdaPin, LOW);
    delayMicroseconds(5);
    pinMode(kImuSclPin, INPUT_PULLUP);
    delayMicroseconds(5);
    pinMode(kImuSdaPin, INPUT_PULLUP);
    delay(2);

    Wire.begin(kImuSdaPin, kImuSclPin);
    Wire.setClock(kImuI2cClockHz);
    delay(10);
}

static void _resetImuBus()
{
    _recoverI2cBus();
}

// Re-init the BNO055 after repeated quaternion failures (I2C glitch or sensor
// entering CONFIG mode under vibration). Reloads saved calibration offsets so
// the sensor comes back fully calibrated without a full imuInit() call.
static bool _attemptImuRecovery()
{
    Serial.println("[IMU] Repeated quat failures — attempting sensor recovery.");
    _recoverI2cBus();
    delay(50);

    bool ok = false;
    for (int i = 0; i < 3 && !ok; ++i) {
        ok = bno.begin(OPERATION_MODE_IMUPLUS);
        if (!ok) { _recoverI2cBus(); delay(50); }
    }

    if (ok) {
        _loadCalibration();
        bno.setExtCrystalUse(kUseExternalCrystal);
        Serial.println("[IMU] Recovery succeeded.");
    } else {
        Serial.println("[IMU] Recovery failed — IMU offline.");
        _ready = false;
    }
    return ok;
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

static float _axisValue(const imu::Vector<3>& v, int axis)
{
    switch (axis) {
        case 0: return v.x();
        case 1: return v.y();
        case 2: return v.z();
        default: return 0.0f;
    }
}

/**
 * Tilt angle from vertical using pitch and roll.
 * 0 deg = perfectly vertical, 90 deg = horizontal.
 * Uses dot-product formula for accuracy at large angles.
 */
static float _compute_tilt(float pitch_deg, float roll_deg)
{
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad  = roll_deg  * DEG_TO_RAD;
    float cos_tilt  = cosf(pitch_rad) * cosf(roll_rad);
    cos_tilt = constrain(cos_tilt, -1.0f, 1.0f);
    return acosf(cos_tilt) * RAD_TO_DEG;
}

static void _printCalibration()
{
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    VLOGF("[IMU] Cal: SYS=%u GYRO=%u ACCEL=%u (MAG=%u ignored)\n",
          sys, gyro, accel, mag);
}

static bool _readOrientationFromQuat(ImuSample& sample)
{
    imu::Quaternion q = bno.getQuat();
    double mag = q.magnitude();
    if (!isfinite(mag) || mag < 1.0e-6) {
        return false;
    }

    q.normalize();

    // Axis convention fix adapted from the BNO055 quaternion->Euler workaround.
    // This keeps the output aligned with the rocket-frame logs used elsewhere.
    float temp = q.x();
    q.x() = -q.y();
    q.y() = temp;
    q.z() = -q.z();

    imu::Vector<3> euler = q.toEuler();
    sample.pitch = -euler.z() * RAD_TO_DEG;
    sample.roll  = -euler.x() * RAD_TO_DEG;
    sample.yaw   = -euler.y() * RAD_TO_DEG;
    return true;
}

static void _readOrientationFromEuler(ImuSample& sample)
{
    // BNO055 VECTOR_EULER in IMUPLUS mode:
    //   x = heading / yaw  (0-360 deg)
    //   y = roll           (+-180 deg)
    //   z = pitch          (+-90 deg)
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    sample.pitch = euler.x();
    sample.roll  = euler.y();
    sample.yaw   = euler.z();
}

static void _restoreLastGoodOrientation(ImuSample& sample)
{
    sample.yaw = _lastGoodYaw;
    sample.pitch = _lastGoodPitch;
    sample.roll = _lastGoodRoll;
}

static void _readImuIntoSample(ImuSample& sample)
{
    bool orientationOk = _readOrientationFromQuat(sample);
    if (!orientationOk) {
        delay(2);
        orientationOk = _readOrientationFromQuat(sample);
    }

    if (orientationOk) {
        _quatFailCount = 0;
        _lastGoodYaw   = sample.yaw;
        _lastGoodPitch = sample.pitch;
        _lastGoodRoll  = sample.roll;
        _haveLastGoodOrientation = true;
    } else {
        _quatFailCount++;

        if (_haveLastGoodOrientation) {
            // Keep the last stable orientation rather than trusting a possibly
            // bad Euler frame from the sensor.
            _restoreLastGoodOrientation(sample);
            VLOG("[IMU] Quaternion read failed; reusing last good orientation.");
        } else {
            _readOrientationFromEuler(sample);
        }

        uint32_t now = millis();
        if (_quatFailCount >= kQuatFailRecoverThreshold &&
            (now - _lastRecoveryMs) >= kMinRecoveryIntervalMs) {
            _quatFailCount  = 0;
            _lastRecoveryMs = now;
            _attemptImuRecovery();
        }
    }

    // VECTOR_LINEARACCEL gives m/s^2 with gravity subtracted.
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    sample.accel_x = kRocketAxisXSign * _axisValue(linAccel, kRocketAxisX);
    sample.accel_y = kRocketAxisYSign * _axisValue(linAccel, kRocketAxisY);
    sample.accel_z = kRocketAxisZSign * _axisValue(linAccel, kRocketAxisZ);

    sample.pitch += kPitchMountOffsetDeg;
    sample.tilt_deg = _compute_tilt(sample.pitch, sample.roll);
}

/**
 * Load saved calibration offsets from flash.
 * Returns true if valid offsets were found and applied.
 * Call after bno.begin() but before setExtCrystalUse().
 */
static bool _loadCalibration()
{
    EEPROM.begin(kEepromSize);

    uint32_t magic = 0;
    EEPROM.get(kMagicAddr, magic);

    if (magic != kCalMagic) {
        VLOG("[IMU] No saved calibration in flash — will calibrate fresh.");
        EEPROM.end();
        return false;
    }

    adafruit_bno055_offsets_t offsets;
    EEPROM.get(kCalAddr, offsets);
    EEPROM.end();

    bno.setSensorOffsets(offsets);
    VLOG("[IMU] Loaded calibration offsets from flash.");
    return true;
}

/**
 * Save current calibration offsets to flash.
 * Only called automatically once SYS+GYRO+ACCEL all hit 3.
 */
static void _saveCalibration()
{
    adafruit_bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);

    EEPROM.begin(kEepromSize);
    EEPROM.put(kMagicAddr, kCalMagic);
    EEPROM.put(kCalAddr,   offsets);
    EEPROM.commit();
    EEPROM.end();

    _calSaved = true;
    Serial.println("[IMU] Calibration saved to flash.");
}

// ─── Public implementation ────────────────────────────────────────────────────

bool imuInit()
{
    _resetImuBus();
    delay(100);

    // IMUPLUS mode: accel + gyro fusion only, magnetometer disabled.
    // This avoids mag interference from the rocket airframe and motor.
    bool began = false;
    for (int attempt = 0; attempt < 3 && !began; ++attempt) {
        began = bno.begin(OPERATION_MODE_IMUPLUS);
        if (!began) {
            _resetImuBus();
            delay(50);
        }
    }

    if (!began) {
        Serial.println("[IMU] BNO055 not detected. Check wiring:");
        Serial.printf ("[IMU]   VCC->3.3V | GND->GND | SDA->GPIO%d | SCL->GPIO%d\n",
                       kImuSdaPin, kImuSclPin);
        Serial.printf ("[IMU]   Tried address 0x%02X — change kBno055Address to 0x29 if needed\n",
                       kBno055Address);
        _ready = false;
        return false;
    }

    delay(100);

    // Load saved offsets AFTER begin() but BEFORE setExtCrystalUse()
    bool calLoaded = _loadCalibration();

    bno.setExtCrystalUse(kUseExternalCrystal);

    _ready = true;

    VLOGF("[IMU] BNO055 ready — IMUPLUS (accel+gyro, no mag)\n");
    VLOGF("[IMU] Address=0x%02X SDA=GPIO%d SCL=GPIO%d\n",
          kBno055Address, kImuSdaPin, kImuSclPin);

    if (calLoaded) {
        VLOG("[IMU] Saved offsets loaded — calibration should be fast.");
    } else {
        VLOG("[IMU] No saved offsets. To calibrate:");
        VLOG("[IMU]   GYRO:  leave still for a few seconds");
        VLOG("[IMU]   ACCEL: place board on each of its 6 faces briefly");
        VLOG("[IMU]   Auto-saves when SYS+GYRO+ACCEL all reach 3.");
    }

    _printCalibration();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

bool imuRead(ImuSample &sample)
{
    if (!_ready) return false;

    _readImuIntoSample(sample);

    // ── Auto-save calibration ─────────────────────────────────────────────────
    // Check every read, save once per session when fully calibrated.
    // MAG is intentionally not checked — IMUPLUS mode does not use it.

    // DEBUGGING
    // {
    // uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    // bno.getCalibration(&sys, &gyro, &accel, &mag);
    // Serial.printf("[IMU-CAL] SYS=%u GYRO=%u ACCEL=%u MAG=%u | calSaved=%d\n",
    //               sys, gyro, accel, mag, _calSaved ? 1 : 0);
    // }
    uint32_t now = millis();
    if (now - _lastDebugPrintMs >= kImuDebugPrintPeriodMs) {
        _lastDebugPrintMs = now;
        VLOGF("[IMU-EULER] Yaw=%.1f Pitch=%.1f Roll=%.1f\n",
              sample.yaw, sample.pitch, sample.roll);
    }

    if (!_calSaved) {
        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        if (gyro == 3 && accel == 3) {
            Serial.println("[IMU] Fully calibrated — saving.");
            _saveCalibration();
            _printCalibration();
        }
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

bool imuReady()
{
    return _ready;
}
