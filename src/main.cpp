#include <Arduino.h>

#include "imu.h"
#include "transmitter.h"
#include "bar.h"
#include "servo.h"
#include "kalman.h"

// ─── Transmit interval ────────────────────────────────────────────────────────
static constexpr uint32_t kTransmitIntervalMs = 50;   // 20 Hz downlink
static uint32_t lastTxMs   = 0;

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
// Set from OpenRocket simulation. Must be above kAltitudeLockoutM.
static constexpr float kDeployAltM        = 2500.0f;
static constexpr float kDeployHysteresisM = 5.0f;

// ─── Liftoff / burnout detection ─────────────────────────────────────────────
// 550 lb thrust / 42 lb rocket = ~13G off pad.
// Liftoff threshold = 5G (49 m/s²).
// accel_y has gravity removed so reads ~0 m/s² at rest.
static constexpr float    kLiftoffAccelMs2   = 49.0f;   // 5G
static constexpr float    kBurnoutAccelMs2   = 2.0f;    // near-zero = motor out
static constexpr uint32_t kBurnoutBackstopMs = 6000;    // 4s burn + 2s margin

// ─── Landing detection ────────────────────────────────────────────────────────
// DESCENDING → DESCENDED when velocity stays below threshold for 3 seconds.
// Prevents false landing detection from momentary velocity dips during descent.
static constexpr float    kLandedVelThreshMs = 2.0f;    // m/s
static constexpr uint32_t kLandedConfirmMs   = 3000;    // ms
static uint32_t           landedTimerStart   = 0;
static bool               landedTimerRunning = false;

// ─── Barometer timing ─────────────────────────────────────────────────────────
// BMP280 at ~40 Hz (STANDBY_MS_62 + SAMPLING_X8).
// Kalman only runs on fresh baro reads to prevent velocity drift.
static constexpr uint32_t kBaroPeriodMs = 25;
static uint32_t lastBaroMs   = 0;
static uint32_t lastKalmanMs = 0;

// ─── Flight state machine ─────────────────────────────────────────────────────

enum class FlightState {
    PAD,         // Sitting on launch pad. Waiting for liftoff.
    BOOST,       // Motor burning. CAS locked neutral per IREC 7.4.1.
    COAST,       // Motor out, ascending. Airbrakes active if conditions met.
    DESCENDING,  // Past apogee, falling. Airbrakes retracted.
    DESCENDED    // On the ground. Flight over. Velocity forced to zero (ZUPT).
};

static FlightState state        = FlightState::PAD;
static uint32_t    boostStartMs = 0;
static bool        airbrakeOut  = false;
static float       lastAltM     = 0.0f;

// Cache last Kalman state — always have something to transmit
static KfState lastKf = {0.0f, 0.0f};

// ─── Helpers ─────────────────────────────────────────────────────────────────

static const char* stateLabel(FlightState s)
{
    switch (s) {
        case FlightState::PAD:        return "PAD";
        case FlightState::BOOST:      return "BOOST";
        case FlightState::COAST:      return "COAST";
        case FlightState::DESCENDING: return "DESCENDING";
        case FlightState::DESCENDED:  return "DESCENDED";
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
        Serial.println("[MAIN] IMU unavailable — continuing without IMU.");
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

    // ── Kalman init ───────────────────────────────────────────────────────────
    // Average 20 baro readings after calibration settles.
    // Fixes the 2-3m ground offset from single noisy sample.
    Serial.println("[MAIN] Settling baro for Kalman init...");
    delay(500);
    double initSum   = 0.0;
    int    initCount = 0;
    for (int i = 0; i < 20; i++) {
        float t, p, a;
        if (bar_read_all(t, p, a)) {
            initSum += a;
            initCount++;
        }
        delay(25);
    }
    float initAlt = (initCount > 0) ? (float)(initSum / initCount) : 0.0f;
    Serial.printf("[MAIN] Kalman init altitude: %.2f m (%d samples)\n",
                  initAlt, initCount);

    kf_init(initAlt);
    lastKf = {initAlt, 0.0f};

    lastBaroMs   = millis();
    lastKalmanMs = millis();

    Serial.println("[MAIN] System ready.");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop()
{
    transmitterPoll();

    uint32_t now = millis();

    // ── IMU read (every loop — non-blocking) ──────────────────────────────────
    ImuSample imu;
    bool imuOk = imuRead(imu);

    // ── Barometer + Kalman (only at ~40 Hz baro rate) ─────────────────────────
    bool baroFresh = (now - lastBaroMs) >= kBaroPeriodMs;
    float temp = 0.0f, pressure = 0.0f, baroAlt = 0.0f;
    bool barOk = false;

    if (baroFresh) {
        barOk      = bar_read_all(temp, pressure, baroAlt);
        lastBaroMs = now;

        if (barOk) {
            float dt_s = (now - lastKalmanMs) / 1000.0f;
            lastKalmanMs = now;
            if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = kBaroPeriodMs / 1000.0f;

            // IMU accel disabled until board is permanently mounted in rocket
            // and verified in that orientation. Bench accel noise causes
            // Kalman velocity explosion. Re-enable for flight:
            //   float accelInput = imuOk ? imu.accel_y : 0.0f;
            float accelInput = 0.0f;

            lastKf = kf_update(baroAlt, accelInput, true, dt_s);
        }
    }

    // ── ZUPT — Zero Velocity Update ───────────────────────────────────────────
    // Force velocity to zero when rocket is known to be stationary.
    // PAD:       on launch pad before liftoff
    // DESCENDED: on ground after landing
    // Prevents baro noise from drifting velocity when there is no real motion.
    if (state == FlightState::PAD || state == FlightState::DESCENDED) {
        lastKf.velocity_ms = 0.0f;
    }

    // ── State machine ─────────────────────────────────────────────────────────
    switch (state) {

        // ── PAD: wait for liftoff ─────────────────────────────────────────────
        case FlightState::PAD:
            retractAirbrakes("PAD state");
            if (imuOk && imu.accel_y >= kLiftoffAccelMs2) {
                state        = FlightState::BOOST;
                boostStartMs = millis();
                Serial.printf("[SM] PAD -> BOOST (accel_y=%.1f m/s² vel=%.1f m/s)\n",
                              imu.accel_y, lastKf.velocity_ms);
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
                    Serial.printf("[SM] BOOST -> COAST (%s at %lums alt=%.1fm vel=%.1fm/s)\n",
                                  accelBurnout ? "accel" : "timer",
                                  (unsigned long)burnElapsedMs,
                                  lastKf.altitude_m, lastKf.velocity_ms);
                }
            }
            break;

        // ── COAST: ascending after burnout, airbrakes active ──────────────────
        case FlightState::COAST:
            {
                float altNow = lastKf.altitude_m;

                // Condition 1 — IREC 7.4.1.3.2: above 2,000 m AGL
                bool altOk    = (altNow >= kAltitudeLockoutM);

                // Condition 2 — IREC 7.3.1: tilt within 30° of vertical
                // If IMU unavailable assume tilt OK — can't confirm violation
                bool tiltOk   = !imuOk || (imu.tilt_deg <= kMaxTiltDeg);

                // Condition 3 — deployment altitude reached
                bool deployOk = (altNow >= kDeployAltM);

                // 7.3.1 — tilt exceeded: retract IMMEDIATELY
                if (imuOk && imu.tilt_deg > kMaxTiltDeg) {
                    retractAirbrakes("tilt > 30deg");
                }
                // All three conditions met — deploy
                else if (altOk && tiltOk && deployOk && !airbrakeOut) {
                    deployAirbrakes();
                }
                // Hysteresis retract
                else if (airbrakeOut && altNow <= (kDeployAltM - kDeployHysteresisM)) {
                    retractAirbrakes("below deploy alt");
                }

                // Apogee: velocity negative AND altitude falling
                // Require both to avoid single noisy reading triggering transition
                if (lastKf.velocity_ms < 0.0f && altNow < lastAltM) {
                    retractAirbrakes("apogee reached");
                    state = FlightState::DESCENDING;
                    Serial.printf("[SM] COAST -> DESCENDING (alt=%.1fm vel=%.1fm/s)\n",
                                  lastKf.altitude_m, lastKf.velocity_ms);
                }
            }
            break;

        // ── DESCENDING: falling after apogee ──────────────────────────────────
        // Airbrakes retracted. Waiting for landing detection.
        case FlightState::DESCENDING:
            retractAirbrakes("DESCENDING");
            {
                float absVel = fabsf(lastKf.velocity_ms);

                // Landing confirmed when velocity stays near zero for 3 seconds
                if (absVel < kLandedVelThreshMs) {
                    if (!landedTimerRunning) {
                        landedTimerStart   = millis();
                        landedTimerRunning = true;
                    } else if ((millis() - landedTimerStart) >= kLandedConfirmMs) {
                        state              = FlightState::DESCENDED;
                        landedTimerRunning = false;
                        Serial.printf("[SM] DESCENDING -> DESCENDED (alt=%.1fm vel=%.1fm/s)\n",
                                      lastKf.altitude_m, lastKf.velocity_ms);
                    }
                } else {
                    // Still moving — reset landing timer
                    landedTimerRunning = false;
                }
            }
            break;

        // ── DESCENDED: on the ground, flight over ─────────────────────────────
        // ZUPT above forces velocity permanently to zero.
        // Airbrakes stay retracted.
        case FlightState::DESCENDED:
            retractAirbrakes("DESCENDED");
            break;
    }

    if (barOk) lastAltM = lastKf.altitude_m;

    // ── Serial debug ──────────────────────────────────────────────────────────
    Serial.printf(
        "[DATA] %-11s | AltKF=%6.1fm BaroAlt=%6.1fm | VelKF=%6.2fm/s | "
        "AccY=%6.1f Tilt=%5.1f° | Brake=%d\n",
        stateLabel(state),
        lastKf.altitude_m,
        barOk ? baroAlt : -999.0f,
        lastKf.velocity_ms,
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
    payload += ", AltKF: "   + String(lastKf.altitude_m,  2);
    payload += ", VelKF: "   + String(lastKf.velocity_ms, 2);
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