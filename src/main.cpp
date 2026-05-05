#include <Arduino.h>

#include "imu.h"
#include "transmitter.h"
#include "bar.h"
#include "servo.h"
#include "kalman.h"

// ─── Transmit interval ────────────────────────────────────────────────────────
static constexpr uint32_t kTransmitIntervalMs = 50;   // 20 Hz downlink
static uint32_t lastTxMs    = 0;
static uint32_t lastLoopMs  = 0;   // used to compute dt for Kalman

// ─── Airbrake config ──────────────────────────────────────────────────────────
static constexpr int   kServoChannel  = 0;
static constexpr float kAirbrakeAngle = 45.0f;   // degrees — deployed
static constexpr float kNeutralAngle  = 0.0f;    // degrees — retracted / safe

// ─── IREC compliance constants ────────────────────────────────────────────────

// 7.4.1.3.2 — 10K flight: airbrakes locked until 2,000 m AGL
static constexpr float kAltitudeLockoutM = 2000.0f;

// 7.3.1 — retract immediately if tilt exceeds 30° from vertical
static constexpr float kMaxTiltDeg = 30.0f;

// Airbrake deployment altitude (AGL, metres).
// Set this from your OpenRocket simulation — should be above lockout.
// Current value is a placeholder based on 3,258 m predicted apogee.
static constexpr float kDeployAltM        = 2500.0f;
static constexpr float kDeployHysteresisM = 5.0f;   // wider band when using Kalman

// ─── Liftoff / burnout detection ─────────────────────────────────────────────
//
// With a 550 lb thrust motor and 42 lb rocket, off-pad acceleration ≈ 13G.
// Liftoff threshold set to 5G (49 m/s²) — well above pad noise, well below launch.
// accel_y is gravity-compensated by BNO055, so 0 m/s² at rest.

static constexpr float    kLiftoffAccelMs2   = 49.0f;   // 5G — liftoff detect
static constexpr float    kBurnoutAccelMs2   = 2.0f;    // near-zero net accel
static constexpr uint32_t kBurnoutBackstopMs = 6000;    // 4s burn + 2s margin

// ─── Barometer timing ─────────────────────────────────────────────────────────
// BMP280 runs at ~80 Hz. Track last read time to flag fresh baro data
// for the Kalman update step.
static constexpr uint32_t kBaroPeriodMs = 13;   // ~80 Hz = 12.5 ms, use 13
static uint32_t lastBaroMs = 0;

// ─── Flight state machine ─────────────────────────────────────────────────────

enum class FlightState {
    PAD,        // On launch pad. Waiting for liftoff.
    BOOST,      // Motor burning. CAS locked neutral per IREC 7.4.1.
    COAST,      // Motor out. Airbrakes may deploy if all conditions met.
    DESCENDED   // Past apogee, below lockout altitude. Flight over.
};

static FlightState state        = FlightState::PAD;
static uint32_t    boostStartMs = 0;
static bool        airbrakeOut  = false;
static float       lastAltM     = 0.0f;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static const char* stateLabel(FlightState s)
{
    switch (s) {
        case FlightState::PAD:       return "PAD";
        case FlightState::BOOST:     return "BOOST";
        case FlightState::COAST:     return "COAST";
        case FlightState::DESCENDED: return "DESCENDED";
    }
    return "UNKNOWN";
}

static void retractAirbrakes(const char* reason)
{
    if (airbrakeOut) {
        servo_set_angle(kServoChannel, kNeutralAngle);
        airbrakeOut = false;
        Serial.printf("[CTRL] Airbrakes RETRACTED — %s\n", reason);
    }
}

static void deployAirbrakes()
{
    if (!airbrakeOut) {
        servo_set_angle(kServoChannel, kAirbrakeAngle);
        airbrakeOut = true;
        Serial.println("[CTRL] Airbrakes DEPLOYED");
    }
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    delay(200);

    if (!imuInit()) {
        Serial.println("[MAIN] IMU init failed, halting.");
        while (true) { delay(1000); }
    }

    if (!transmitterInit()) {
        Serial.println("[MAIN] Transmitter init failed, halting.");
        while (true) { delay(1000); }
    }

    // bar_init() owns Wire1.begin(18,19) — must run before servo_init()
    if (!bar_init()) {
        Serial.println("[MAIN] BMP280 init failed, halting.");
        while (true) { delay(1000); }
    }

    // 10-second ground pressure calibration — board must be still on pad
    float groundPressure = bar_calibrate();
    if (groundPressure < 0.0f) {
        Serial.println("[MAIN] Barometer calibration failed, halting.");
        while (true) { delay(1000); }
    }

    // servo_init() shares Wire1 — Wire1.begin() already called by bar_init()
    if (!servo_init()) {
        Serial.println("[MAIN] Servo init failed, halting.");
        while (true) { delay(1000); }
    }

    // 7.3.1 — ensure neutral state at boot before anything else
    servo_set_angle(kServoChannel, kNeutralAngle);
    airbrakeOut = false;

    // Initialise Kalman filter with current AGL altitude (should be ~0 on pad)
    float initAlt = bar_get_altitude_agl();
    kf_init(initAlt);

    lastLoopMs = millis();
    lastBaroMs = millis();

    Serial.println("[MAIN] System ready — waiting for liftoff.");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop()
{
    transmitterPoll();

    // ── Timing ────────────────────────────────────────────────────────────────
    uint32_t now  = millis();
    float    dt_s = (now - lastLoopMs) / 1000.0f;
    lastLoopMs    = now;

    // ── IMU read (every loop — BNO055 updates at up to 100 Hz) ───────────────
    ImuSample imu;
    bool imuOk = imuRead(imu);

    // ── Barometer read (flagged fresh at ~80 Hz) ──────────────────────────────
    float temp, pressure, baroAlt;
    bool barOk    = false;
    bool baroFresh = (now - lastBaroMs) >= kBaroPeriodMs;

    if (baroFresh) {
        barOk      = bar_read_all(temp, pressure, baroAlt);
        lastBaroMs = now;
    }

    // ── Kalman filter update ──────────────────────────────────────────────────
    // Run every loop using IMU accel as the physics input (predict step).
    // Baro corrects the estimate only when a fresh reading is available (update step).
    // If IMU is unavailable, pass 0.0f — filter coasts on kinematics only.
    float accelInput = imuOk ? imu.accel_y : 0.0f;
    float baroInput  = barOk ? baroAlt     : lastAltM;

    KfState kf = kf_update(baroInput, accelInput, barOk && baroFresh, dt_s);

    // kf.altitude_m  = best estimate of altitude AGL (metres)
    // kf.velocity_ms = best estimate of vertical velocity (m/s, + = up)

    // ── State machine ─────────────────────────────────────────────────────────
    switch (state) {

        // ── PAD: wait for liftoff ─────────────────────────────────────────────
        case FlightState::PAD:
            retractAirbrakes("PAD state");

            // Liftoff detected when vertical accel exceeds 5G (49 m/s²)
            if (imuOk && imu.accel_y >= kLiftoffAccelMs2) {
                state        = FlightState::BOOST;
                boostStartMs = millis();
                Serial.printf("[SM] PAD → BOOST (accel_y=%.1f m/s², kf_vel=%.1f m/s)\n",
                              imu.accel_y, kf.velocity_ms);
            }
            break;

        // ── BOOST: motor burning, CAS locked neutral per IREC 7.4.1 ──────────
        case FlightState::BOOST:
            retractAirbrakes("BOOST phase");
            {
                uint32_t burnElapsedMs = millis() - boostStartMs;
                bool accelBurnout = imuOk && (imu.accel_y < kBurnoutAccelMs2);
                bool timerBurnout = (burnElapsedMs >= kBurnoutBackstopMs);

                if (accelBurnout || timerBurnout) {
                    state = FlightState::COAST;
                    Serial.printf("[SM] BOOST → COAST (%s at %lums, alt=%.1fm vel=%.1fm/s)\n",
                                  accelBurnout ? "accel" : "timer",
                                  (unsigned long)burnElapsedMs,
                                  kf.altitude_m, kf.velocity_ms);
                }
            }
            break;

        // ── COAST: enforce all three IREC conditions ──────────────────────────
        case FlightState::COAST:
            {
                float altNow = kf.altitude_m;   // use Kalman altitude

                // Condition 1 — IREC 7.4.1.3.2: must be above 2,000 m AGL
                bool altOk   = (altNow >= kAltitudeLockoutM);

                // Condition 2 — IREC 7.3.1: tilt must be within 30° of vertical
                bool tiltOk  = imuOk && (imu.tilt_deg <= kMaxTiltDeg);

                // Condition 3 — deployment altitude reached
                bool deployOk = (altNow >= kDeployAltM);

                // 7.3.1 — tilt exceeded: retract IMMEDIATELY, no exceptions
                if (imuOk && !tiltOk) {
                    retractAirbrakes("tilt > 30deg");
                }
                // All three conditions satisfied — deploy
                else if (altOk && tiltOk && deployOk && !airbrakeOut) {
                    deployAirbrakes();
                }
                // Hysteresis retract
                else if (airbrakeOut && altNow <= (kDeployAltM - kDeployHysteresisM)) {
                    retractAirbrakes("below deploy alt");
                }

                // Transition to DESCENDED: back below lockout AND velocity negative
                if (altNow < kAltitudeLockoutM &&
                    lastAltM >= kAltitudeLockoutM &&
                    kf.velocity_ms < 0.0f) {
                    retractAirbrakes("descended below lockout");
                    state = FlightState::DESCENDED;
                    Serial.printf("[SM] COAST → DESCENDED (alt=%.1fm vel=%.1fm/s)\n",
                                  kf.altitude_m, kf.velocity_ms);
                }
            }
            break;

        // ── DESCENDED: flight over ─────────────────────────────────────────────
        case FlightState::DESCENDED:
            retractAirbrakes("DESCENDED state");
            break;
    }

    lastAltM = kf.altitude_m;

    // ── Serial debug ──────────────────────────────────────────────────────────
    Serial.printf(
        "[DATA] %-10s | AltKF=%6.1fm BaroAlt=%6.1fm | VelKF=%6.1fm/s | "
        "AccY=%6.1f Tilt=%5.1f° | Brake=%d\n",
        stateLabel(state),
        kf.altitude_m,
        barOk ? baroAlt : -999.0f,
        kf.velocity_ms,
        imuOk ? imu.accel_y : -999.0f,
        imuOk ? imu.tilt_deg : -999.0f,
        airbrakeOut ? 1 : 0
    );

    // ── Transmit at fixed interval ────────────────────────────────────────────
    if (now - lastTxMs < kTransmitIntervalMs) return;
    lastTxMs = now;

    String payload;
    payload.reserve(160);
    payload += "State: "     + String(stateLabel(state));
    payload += ", AltKF: "   + String(kf.altitude_m,  2);
    payload += ", VelKF: "   + String(kf.velocity_ms, 2);
    payload += ", BaroAlt: " + String(barOk  ? baroAlt       : -999.0f, 2);
    payload += ", AccY: "    + String(imuOk  ? imu.accel_y   : -999.0f, 2);
    payload += ", Tilt: "    + String(imuOk  ? imu.tilt_deg  : -999.0f, 2);
    payload += ", Roll: "    + String(imuOk  ? imu.roll      : -999.0f, 2);
    payload += ", Pitch: "   + String(imuOk  ? imu.pitch     : -999.0f, 2);
    payload += ", Yaw: "     + String(imuOk  ? imu.yaw       : -999.0f, 2);
    payload += ", Temp: "    + String(barOk  ? temp          : -999.0f, 2);
    payload += ", Airbrake: "+ String(airbrakeOut ? 1 : 0);

    transmitterSend(payload);
}