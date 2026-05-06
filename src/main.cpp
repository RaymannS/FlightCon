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
static constexpr float kAirbrakeAngle = 42.0f;   // degrees — deployed
static constexpr float kNeutralAngle  = 8.0f;    // degrees — retracted / safe

// ─── IREC compliance constants ────────────────────────────────────────────────

// 7.4.1.3.2 — 10K flight: airbrakes locked until 2,000 m AGL
static constexpr float kAltitudeLockoutM = 2000.0f; // 2000.0f

// 7.3.1 — retract immediately if tilt exceeds 30° from vertical
static constexpr float kMaxTiltDeg = 30.0f;

// Airbrake deployment altitude (AGL, metres).
// Set this from your OpenRocket simulation — should be above lockout.
// Current value is a placeholder based on 3,258 m predicted apogee.
static constexpr float kDeployAltM        = 2500.0f;
static constexpr float kDeployHysteresisM = 5.0f;

// ─── Liftoff / burnout detection ─────────────────────────────────────────────
// With 550 lb thrust, 42 lb rocket → ~13G off pad.
// Liftoff threshold = 5G (49 m/s²) — well above pad noise, well below launch.
// accel_y has gravity removed in imu.cpp so reads ~0 m/s² at rest.
static constexpr float    kLiftoffAccelMs2   = 49.0f;   // 5G
static constexpr float    kBurnoutAccelMs2   = 2.0f;    // near-zero = motor out
static constexpr uint32_t kBurnoutBackstopMs = 6000;    // 4s burn + 2s margin

// ─── Barometer timing ─────────────────────────────────────────────────────────
// BMP280 runs at ~80 Hz with STANDBY_MS_1 + SAMPLING_X4.
// Kalman only runs when baro is fresh — this prevents velocity drift
// that occurs when the filter runs at full loop speed with no real input.
static constexpr uint32_t kBaroPeriodMs = 13;   // ~80 Hz = 12.5 ms, use 13
static uint32_t lastBaroMs  = 0;
static uint32_t lastKalmanMs = 0;   // tracks dt between Kalman updates

// ─── Flight state machine ─────────────────────────────────────────────────────

enum class FlightState {
    PAD,        // On launch pad. Waiting for liftoff.
    BOOST,      // Motor burning. CAS locked neutral per IREC 7.4.1.
    COAST,      // Motor out. Airbrakes may deploy if all conditions met.
    DESCENDED   // Past apogee, below lockout altitude. Flight over.
};

// TODO: change back to FlightState::PAD before flight
static FlightState state        = FlightState::COAST;
static uint32_t    boostStartMs = 0;
static bool        airbrakeOut  = false;
static float       lastAltM     = 0.0f;

// Cache last good Kalman state so we always have something to transmit
// even on loops where baro wasn't fresh
static KfState lastKf = {0.0f, 0.0f};

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
    // Wait for BMP280 to fully settle after calibration, then average
    // 20 readings to get a stable initial altitude rather than a single
    // noisy sample. This fixes the 2-3m offset seen on the ground.
    Serial.println("[MAIN] Settling baro for Kalman init...");
    delay(500);
    double initSum = 0.0;
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
    Serial.printf("[MAIN] Kalman init altitude: %.2f m (from %d samples)\n",
                  initAlt, initCount);

    kf_init(initAlt);
    lastKf = {initAlt, 0.0f};

    lastBaroMs   = millis();
    lastKalmanMs = millis();

    #ifdef ENABLE_OTA
    initOTA();
    #endif

    Serial.println("[MAIN] System ready.");

    // TEMPORARY GROUND TESTS
    // servo_set_angle(kServoChannel, kNeutralAngle);
    // delay(2000);
    // servo_set_angle(kServoChannel, kAirbrakeAngle);
    // delay(2000);
    // servo_set_angle(kServoChannel, kNeutralAngle);
    // delay(2000);
    // servo_set_angle(kServoChannel, kAirbrakeAngle);
    // delay(2000);
    // servo_set_angle(kServoChannel, kNeutralAngle);
    // delay(2000);
    // TEMPORARY GROUND TESTS
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

    // ── Barometer + Kalman (only at ~80 Hz baro rate) ────────────────────────
    // Running Kalman only when baro is fresh prevents velocity drift.
    // Without IMU acceleration to drive the predict step, running the
    // filter faster than the baro provides no benefit and causes drift.
    bool baroFresh = (now - lastBaroMs) >= kBaroPeriodMs;
    float temp = 0.0f, pressure = 0.0f, baroAlt = 0.0f;
    bool barOk = false;

    if (baroFresh) {
        barOk      = bar_read_all(temp, pressure, baroAlt);
        lastBaroMs = now;

        if (barOk) {
            // Compute dt only between Kalman updates — not full loop speed
            float dt_s = (now - lastKalmanMs) / 1000.0f;
            lastKalmanMs = now;

            // Clamp dt — first call or stall protection
            if (dt_s <= 0.0f || dt_s > 0.5f) dt_s = kBaroPeriodMs / 1000.0f;

            // accel_y = 0 when IMU unavailable — filter runs baro-only mode
            float accelInput = imuOk ? imu.accel_y : 0.0f;

            lastKf = kf_update(baroAlt, accelInput, true, dt_s);
        }
    }

    // ── State machine ─────────────────────────────────────────────────────────
    switch (state) {

        // ── PAD: wait for liftoff ─────────────────────────────────────────────
        case FlightState::PAD:
            retractAirbrakes("PAD state");
            if (imuOk && imu.accel_y >= kLiftoffAccelMs2) {
                state        = FlightState::BOOST;
                boostStartMs = millis();
                Serial.printf("[SM] PAD → BOOST (accel_y=%.1f m/s², kf_vel=%.1f m/s)\n",
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
                    Serial.printf("[SM] BOOST → COAST (%s at %lums, alt=%.1fm vel=%.1fm/s)\n",
                                  accelBurnout ? "accel" : "timer",
                                  (unsigned long)burnElapsedMs,
                                  lastKf.altitude_m, lastKf.velocity_ms);
                }
            }
            break;

        // ── COAST: enforce all three IREC conditions ──────────────────────────
        case FlightState::COAST:
            {
                float altNow = lastKf.altitude_m;

                // Condition 1 — IREC 7.4.1.3.2: must be above 2,000 m AGL
                bool altOk    = (altNow >= kAltitudeLockoutM);

                // Condition 2 — IREC 7.3.1: tilt within 30° of vertical
                // If IMU unavailable, assume tilt is OK (can't confirm violation)
                bool tiltOk   = !imuOk || (imu.tilt_deg <= kMaxTiltDeg);

                // Condition 3 — deployment altitude reached
                bool deployOk = (altNow >= kDeployAltM);

                // 7.3.1 — tilt exceeded: retract IMMEDIATELY, no exceptions
                if (imuOk && imu.tilt_deg > kMaxTiltDeg) {
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
                    lastAltM < kAltitudeLockoutM &&
                    lastKf.velocity_ms < 0.0f) {
                    retractAirbrakes("descended below lockout");
                    state = FlightState::DESCENDED;
                    Serial.printf("[SM] COAST → DESCENDED (alt=%.1fm vel=%.1fm/s)\n",
                                  lastKf.altitude_m, lastKf.velocity_ms);
                }
            }
            break;

        // ── DESCENDED: flight over ─────────────────────────────────────────────
        case FlightState::DESCENDED:
            retractAirbrakes("DESCENDED state");
            break;
    }

    if (barOk) lastAltM = lastKf.altitude_m;

    // ── Serial debug (every loop) ─────────────────────────────────────────────
    Serial.printf(
        "[DATA] %-10s | AltKF=%6.1fm BaroAlt=%6.1fm | VelKF=%6.2fm/s | "
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