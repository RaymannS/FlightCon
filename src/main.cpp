#include <Arduino.h>

#include "imu.h"
#include "transmitter.h"
#include "bar.h"
#include "servo.h"
#include "kalman.h"

#ifdef ENABLE_OTA
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif

#ifdef ENABLE_OTA // OTA enabled via build flag in platformio.ini
static bool otaEnabled = false;
#endif

// README: How to run:

// pio run -t upload
// -> Get IP Address from console:
// pio run -t upload --upload-port ADDR (e.g., 192.168.1.50)


// ─── Transmit interval ────────────────────────────────────────────────────────
static constexpr uint32_t kTransmitIntervalMs = 50;   // 20 Hz downlink
static uint32_t lastTxMs   = 0;

// ─── Airbrake config ──────────────────────────────────────────────────────────
static constexpr int   kServoChannel  = 0;
static constexpr float kAirbrakeAngle = 45.0f;   // degrees — deployed
static constexpr float kNeutralAngle  = 8.0f;    // degrees — retracted / safe

// ─── IREC compliance constants ────────────────────────────────────────────────

// 7.4.1.3.2 — 10K flight: airbrakes locked until 2,000 m AGL (1,200 m AGL for Test launch)
static constexpr float kAltitudeLockoutM = 1200.0f;

// 7.3.1 — retract immediately if tilt exceeds 30° from vertical
static constexpr float kMaxTiltDeg = 30.0f;

// Airbrake deployment altitude (AGL, metres).
// Set from OpenRocket simulation. Must be above kAltitudeLockoutM.
static constexpr float kDeployAltM        = 1500.0f; // 2500 m for real -- (1,500 m AGL for Test launch)
static constexpr float kDeployHysteresisM = 5.0f;

// ─── Liftoff / burnout detection ─────────────────────────────────────────────
// 550 lb thrust / 42 lb rocket = ~13G off pad.
//
// BNO055 in fusion mode clips at ~4G (39.2 m/s²) so we cannot use a 5G
// threshold directly. Instead we use a dual confirm:
//   1. IMU accel_y exceeds 2G (19.6 m/s²) — below clip limit, reliable
//   2. Baro altitude has risen at least 20m above launch altitude
// Both must be true simultaneously to trigger PAD -> BOOST.
// This prevents false triggers from pad bumps (IMU alone) or baro noise (baro alone).
static constexpr float    kLiftoffAccelMs2   = 20.0f;   // 2G — below BNO055 4G clip limit
static constexpr float    kLiftoffAltM       = 20.0f;   // metres AGL — baro confirm
static constexpr float    kBurnoutAccelMs2   = 2.0f;    // near-zero = motor out
static constexpr uint32_t kBurnoutBackstopMs = 5000;    // REAL 4s burn + 2s margin (test is 3.5s burn + 1.5s margin)

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

// TODO: change back to FlightState::PAD before flight
static FlightState state        = FlightState::PAD;
static uint32_t    boostStartMs = 0;
static bool        airbrakeOut  = false;
static float       lastAltM     = 0.0f;
static uint32_t apogeeConditionStartMs = 0;
static constexpr uint32_t kApogeeConfirmMs = 2000; // 3 seconds

// Cache last Kalman state — always have something to transmit
static KfState lastKf = {0.0f, 0.0f};
static float baroBuffer[5] = {0};
static int baroIdx = 0;

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

static float median5(const float values[5])
{
    float sorted[5];

    for (int i = 0; i < 5; i++) {
        sorted[i] = values[i];
    }

    // Simple sort for 5 values
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (sorted[j] < sorted[i]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }

    return sorted[2]; // middle value
}

#ifdef ENABLE_OTA
static void initOTA()
{
    WiFi.mode(WIFI_STA);

    WiFi.begin("Raymann", "Flerds@1!");

    Serial.print("[OTA] Connecting WiFi");

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
        delay(250);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[OTA] WiFi failed — OTA disabled");
        otaEnabled = false;
        return;
    }

    // DHCP-assigned IP (this is what matters now)
    Serial.printf("\n[OTA] Connected\n");
    Serial.printf("[OTA] IP address: %s\n", WiFi.localIP().toString().c_str());

    ArduinoOTA.setHostname("esp32-flight");
    ArduinoOTA.setPassword("Flerds@1!");

    ArduinoOTA.begin();
    otaEnabled = true;

    Serial.println("[OTA] Ready");
}
#endif

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
    // Start in PAD tuning
    kf_set_phase(KF_PHASE_PAD);
    for (int i = 0; i < 5; i++) {
        baroBuffer[i] = initAlt;
    }
    baroIdx = 0;

    lastBaroMs   = millis();
    lastKalmanMs = millis();

    #ifdef ENABLE_OTA
    initOTA();
    #endif

    Serial.println("[MAIN] System ready.");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop()
{
    transmitterPoll();

    #ifdef ENABLE_OTA
    if (otaEnabled) ArduinoOTA.handle();
    #endif

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
            float accelInput = imuOk ? imu.accel_y : 0.0f;
            // float accelInput = 0.0f; // TODO: Replace me back

            // TODO: Replace back deadband
            if (imuOk && state != FlightState::PAD && state != FlightState::DESCENDED) {
                accelInput = imu.accel_y;

                // Kill tiny bias/noise near zero
                if (fabsf(accelInput) < 0.25f) {
                    accelInput = 0.0f;
                }
            }


            baroBuffer[baroIdx] = baroAlt;
            baroIdx = (baroIdx + 1) % 5;

            // Feed median to Kalman instead of raw reading
            float baroFiltered = median5(baroBuffer);
            lastKf = kf_update(baroFiltered, accelInput, true, dt_s);
        }
    }

    // ── ZUPT — Zero Velocity Update ───────────────────────────────────────────
    // Force velocity to zero when rocket is known to be stationary.
    // PAD:       on launch pad before liftoff
    // DESCENDED: on ground after landing
    // Prevents baro noise from drifting velocity when there is no real motion.
    if (state == FlightState::PAD || state == FlightState::DESCENDED) {
        kf_zero_velocity();
        lastKf = kf_get_state();
    }

    // ── State machine ─────────────────────────────────────────────────────────
    switch (state) {

        // ── PAD: wait for liftoff ─────────────────────────────────────────────
        // Dual confirm required:
        //   1. IMU accel_y >= 2G (19.6 m/s2) — below BNO055 4G fusion clip
        //   2. Kalman altitude >= 20m AGL — baro confirmation
        // Both must be true simultaneously to prevent false triggers.
        case FlightState::PAD:
            retractAirbrakes("PAD state");
            {
                bool accelTriggered = imuOk && (imu.accel_y >= kLiftoffAccelMs2);
                bool baroTriggered  = (lastKf.altitude_m >= kLiftoffAltM);

                if (accelTriggered && baroTriggered) {
                    state        = FlightState::BOOST;
                    boostStartMs = millis();
                    // Switch Kalman to boost tuning
                    kf_set_phase(KF_PHASE_BOOST);
                    Serial.printf("[SM] PAD -> BOOST (accel_y=%.1f m/s2 alt=%.1fm vel=%.1fm/s)\n",
                                  imu.accel_y, lastKf.altitude_m, lastKf.velocity_ms);
                }
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
                    // Enter COAST tuning
                    kf_set_phase(KF_PHASE_COAST);
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

                // Apogee candidate: velocity negative AND altitude falling
                // Must stay true continuously for X seconds before changing state
                bool apogeeCondition = (lastKf.velocity_ms < 0.0f && altNow < lastAltM);

                if (apogeeCondition) {
                    if (apogeeConditionStartMs == 0) {
                        apogeeConditionStartMs = millis();
                        Serial.println("[SM] Apogee condition detected — starting 3s confirmation timer");
                    }

                    if (millis() - apogeeConditionStartMs >= kApogeeConfirmMs) {
                        retractAirbrakes("apogee reached");
                        state = FlightState::DESCENDING;
                        // Enter descent tuning
                        kf_set_phase(KF_PHASE_DESCENT);
                        apogeeConditionStartMs = 0;

                        Serial.printf("[SM] COAST -> DESCENDING after 3s confirmation (alt=%.1fm vel=%.1fm/s)\n",
                                    lastKf.altitude_m, lastKf.velocity_ms);
                    }
                } else {
                    // Reset timer if the condition breaks even once
                    if (apogeeConditionStartMs != 0) {
                        Serial.println("[SM] Apogee condition lost — confirmation timer reset");
                    }

                    apogeeConditionStartMs = 0;
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
                        // Set landing/ground tuning on confirmed touchdown
                        kf_set_phase(KF_PHASE_LANDING);
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
        "[DATA] %-11s | AltKF=%6.1fm BaroAlt=%6.1fm | VelKF=%6.3fm/s | "
        "AccY=%6.2f Tilt=%5.1f° | Brake=%d\n",
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