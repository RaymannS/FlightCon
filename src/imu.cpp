/**
 * imu.cpp — BNO055 IMU driver for ESP32 WROOM-32E
 *
 * Bus  : I2C0 (Wire) on GPIO 25 (SDA) / 26 (SCL)
 * Addr : 0x29 (ADD pin → 3.3V)
 * Lib  : Adafruit BNO055 + Adafruit Unified Sensor
 *
 * platformio.ini dependencies:
 *   lib_deps =
 *       adafruit/Adafruit BNO055 @ ^1.6.3
 *       adafruit/Adafruit Unified Sensor @ ^1.1.14
 *
 * Arduino IDE: Sketch → Include Library → Manage Libraries → "Adafruit BNO055"
 *
 * Orientation convention:
 *   Y axis = vertical (along rocket body axis)
 *   X axis = along dual airbrake axis
 *   Z axis = perpendicular to both
 *
 * The BNO055 outputs gravity-compensated linear acceleration directly,
 * so no manual gravity subtraction is needed.
 */

#include "imu.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ─── Internal state ──────────────────────────────────────────────────────────

// Sensor ID can be any unique integer — used for EEPROM cal storage (future)
static Adafruit_BNO055 _bno(55, BNO055_ADDR, &Wire);
static bool            _ready = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/**
 * Compute tilt angle from vertical in degrees.
 *
 * The BNO055 Euler angles are:
 *   pitch = rotation around X axis
 *   roll  = rotation around Z axis
 *   yaw   = rotation around Y axis (vertical)
 *
 * When the rocket is perfectly vertical, pitch = 0 and roll = 0.
 * Tilt from vertical = sqrt(pitch^2 + roll^2) for small angles,
 * but we use the proper formula for larger angles.
 */
static float _compute_tilt(float pitch_deg, float roll_deg)
{
    float pitch_rad = pitch_deg * (float)DEG_TO_RAD;
    float roll_rad  = roll_deg  * (float)DEG_TO_RAD;

    // Dot product of rocket axis unit vector with gravity unit vector
    float cos_tilt = cosf(pitch_rad) * cosf(roll_rad);
    cos_tilt = constrain(cos_tilt, -1.0f, 1.0f);
    return acosf(cos_tilt) * (float)RAD_TO_DEG;
}

// ─── Public implementation ───────────────────────────────────────────────────

bool imuInit()
{
    delay(200);

    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);  // 400 kHz fast mode

    if (!_bno.begin()) {
        Serial.printf("[IMU] BNO055 not found at 0x%02X — check wiring / ADD pin\n",
                      BNO055_ADDR);
        _ready = false;
        return false;
    }

    // Use the BNO055's internal crystal for better accuracy
    _bno.setExtCrystalUse(true);

    // NDOF mode: full 9-DOF sensor fusion (accel + gyro + mag)
    // This gives the best Euler angle accuracy but requires magnetometer
    // calibration. If magnetometer cal is poor, switch to IMU mode:
    // _bno.setMode(OPERATION_MODE_IMUPLUS);  // accel + gyro only, no mag
    _bno.setMode(OPERATION_MODE_NDOF);

    _ready = true;
    Serial.printf("[IMU] BNO055 initialised at 0x%02X (SDA=%d SCL=%d)\n",
                  BNO055_ADDR, IMU_SDA_PIN, IMU_SCL_PIN);
    return true;
}

bool imuRead(ImuSample &sample)
{
    if (!_ready) return false;

    // ── Euler angles ─────────────────────────────────────────────────────────
    sensors_event_t orientEvent;
    _bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER);

    sample.yaw   = orientEvent.orientation.x;  // heading (0–360°)
    sample.pitch = orientEvent.orientation.y;  // pitch   (−180 to +180°)
    sample.roll  = orientEvent.orientation.z;  // roll    (−90 to +90°)

    // ── Linear acceleration (gravity removed) ─────────────────────────────────
    sensors_event_t accelEvent;
    _bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

    sample.accel_x = accelEvent.acceleration.x;
    sample.accel_y = accelEvent.acceleration.y;  // vertical axis
    sample.accel_z = accelEvent.acceleration.z;

    // ── Tilt from vertical ────────────────────────────────────────────────────
    sample.tilt_deg = _compute_tilt(sample.pitch, sample.roll);

    return true;
}

bool imuReady()
{
    return _ready;
}