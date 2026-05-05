#include <Arduino.h>

#include "imu.h"
#include "transmitter.h"
#include "bar.h"
#include "servo.h"

// ─── Transmit interval ────────────────────────────────────────────────────────
static constexpr uint32_t kTransmitIntervalMs = 50;
static uint32_t lastTxMs = 0;

// ─── Airbrake config ──────────────────────────────────────────────────────────
static constexpr int   kServoChannel    = 0;
static constexpr float kAirbrakeAngle   = 45.0f;   // degrees — deployed
static constexpr float kNeutralAngle    = 0.0f;    // degrees — retracted / safe

// ─── IREC compliance constants ────────────────────────────────────────────────

// 7.4.1.3.2 — 10K flight: airbrakes locked until 2,000 m AGL
static constexpr float kAltitudeLockoutM = 2000.0f;

// 7.3.1 — retract immediately if tilt exceeds 30° from vertical
static constexpr float kMaxTiltDeg = 30.0f;

// Airbrake deployment altitude target (AGL). Must be above lockout.
// Adjust based on OpenRocket simulation for your motor + airframe.
static constexpr float kDeployAltM = 2500.0f;
static constexpr float kDeployHysteresisM = 1.0f;  // avoids chattering

// ─── Liftoff / burnout detection ─────────────────────────────────────────────

// Liftoff: vertical accel (Y axis) exceeds this threshold (m/s^2)
// 1.5G = ~14.7 m/s^2. Tune if needed.
static constexpr float kLiftoffAccelMs2 = 14.7f;

// Burnout: vertical accel drops below this (motor off, ~0G net)
// Using 0 m/s^2 — gravity is already removed by BNO055 linear accel
static constexpr float kBurnoutAccelMs2 = 2.0f;

// Burnout backstop: if still in BOOST after this many ms, force transition
static constexpr uint32_t kBurnoutBackstopMs = 6000;  // 4s burn + 2s margin

// Apogee / descent detection: altitude must be falling by this much per sample
static constexpr float kDescentThresholdM = -1.0f;

// ─── Flight state machine ─────────────────────────────────────────────────────

enum class FlightState {
    PAD,        // Sitting on launch pad. Waiting for liftoff.
    BOOST,      // Motor burning. CAS locked neutral per 7.4.1.
    COAST,      // Motor out. Airbrakes may deploy if conditions met.
    DESCENDED   // Below deployment alt after apogee. Airbrakes retracted.
};

static FlightState state          = FlightState::PAD;
static uint32_t    boostStartMs   = 0;
static bool        airbrakeOut    = false;
static float       lastAltM       = 0.0f;

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

/** Retract airbrakes and update flag. */
static void retractAirbrakes(const char* reason)
{
    if (airbrakeOut) {
        servo_set_angle(kServoChannel, kNeutralAngle);
        airbrakeOut = false;
        Serial.printf("[CTRL] Airbrakes RETRACTED — %s\n", reason);
    }
}

/** Deploy airbrakes and update flag. */
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

    // bar_init() owns Wire1.begin(18, 19) — must run before servo_init()
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

    // 7.3.1 — ensure neutral state at boot
    servo_set_angle(kServoChannel, kNeutralAngle);
    airbrakeOut = false;

    Serial.println("[MAIN] System ready — waiting for liftoff.");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop()
{
    transmitterPoll();

    // ── Sensor reads ──────────────────────────────────────────────────────────
    float temp, pressure, altAGL;
    bool barOk = bar_read_all(temp, pressure, altAGL);

    ImuSample imu;
    bool imuOk = imuRead(imu);

    // ── State machine ─────────────────────────────────────────────────────────
    switch (state) {

        // ── PAD: wait for liftoff ─────────────────────────────────────────────
        case FlightState::PAD:
            // Airbrakes always neutral on pad
            retractAirbrakes("PAD state");

            if (imuOk && imu.accel_y >= kLiftoffAccelMs2) {
                state       = FlightState::BOOST;
                boostStartMs = millis();
                Serial.printf("[SM] PAD → BOOST (accel_y=%.2f m/s²)\n", imu.accel_y);
            }
            break;

        // ── BOOST: motor burning, CAS locked neutral per IREC 7.4.1 ──────────
        case FlightState::BOOST:
            // 7.4.1 — CAS must remain neutral during boost
            retractAirbrakes("BOOST phase");

            {
                uint32_t burnElapsedMs = millis() - boostStartMs;
                bool accelBurnout  = imuOk && (imu.accel_y < kBurnoutAccelMs2);
                bool timerBurnout  = (burnElapsedMs >= kBurnoutBackstopMs);

                if (accelBurnout || timerBurnout) {
                    state = FlightState::COAST;
                    Serial.printf("[SM] BOOST → COAST (%s, elapsed=%lums)\n",
                                  accelBurnout ? "accel" : "timer",
                                  (unsigned long)burnElapsedMs);
                }
            }
            break;

        // ── COAST: all three IREC conditions enforced ─────────────────────────
        case FlightState::COAST:
            {
                // Evaluate all three deployment conditions
                bool altOk    = barOk  && (altAGL >= kAltitudeLockoutM);   // 7.4.1.3.2
                bool tiltOk   = imuOk  && (imu.tilt_deg <= kMaxTiltDeg);   // 7.3.1
                bool deployAlt = barOk && (altAGL >= kDeployAltM);

                // 7.3.1 — if tilt exceeded, retract immediately regardless
                if (imuOk && !tiltOk) {
                    retractAirbrakes("tilt > 30deg");
                }
                // All three conditions met — deploy
                else if (altOk && tiltOk && deployAlt && !airbrakeOut) {
                    deployAirbrakes();
                }
                // Hysteresis — retract if we drop below deploy altitude
                else if (airbrakeOut && barOk &&
                         altAGL <= (kDeployAltM - kDeployHysteresisM)) {
                    retractAirbrakes("below deploy alt");
                }

                // Transition to DESCENDED once we drop below lockout altitude
                // after having been above it (i.e., past apogee)
                if (barOk && altAGL < kAltitudeLockoutM &&
                    lastAltM >= kAltitudeLockoutM) {
                    retractAirbrakes("descended below lockout");
                    state = FlightState::DESCENDED;
                    Serial.println("[SM] COAST → DESCENDED");
                }
            }
            break;

        // ── DESCENDED: flight over, stay neutral ──────────────────────────────
        case FlightState::DESCENDED:
            retractAirbrakes("DESCENDED state");
            break;
    }

    // Update last altitude for descent detection
    if (barOk) lastAltM = altAGL;

    // ── Serial debug ──────────────────────────────────────────────────────────
    if (barOk && imuOk) {
        Serial.printf("[DATA] State=%-10s Alt=%.2fm Tilt=%.1f° AccY=%.2f AirbrakeOut=%d\n",
                      stateLabel(state), altAGL, imu.tilt_deg, imu.accel_y,
                      airbrakeOut ? 1 : 0);
    }

    // ── Transmit at fixed interval ────────────────────────────────────────────
    const uint32_t now = millis();
    if (now - lastTxMs < kTransmitIntervalMs) return;
    lastTxMs = now;

    String payload;
    payload.reserve(128);
    payload += "State: "    + String(stateLabel(state));
    payload += ", AltAGL: " + String(barOk  ? altAGL        : -999.0f, 2);
    payload += ", Tilt: "   + String(imuOk  ? imu.tilt_deg  : -999.0f, 2);
    payload += ", AccY: "   + String(imuOk  ? imu.accel_y   : -999.0f, 2);
    payload += ", Roll: "   + String(imuOk  ? imu.roll      : -999.0f, 2);
    payload += ", Pitch: "  + String(imuOk  ? imu.pitch     : -999.0f, 2);
    payload += ", Yaw: "    + String(imuOk  ? imu.yaw       : -999.0f, 2);
    payload += ", Temp: "   + String(barOk  ? temp          : -999.0f, 2);
    payload += ", Airbrake: " + String(airbrakeOut ? 1 : 0);

    transmitterSend(payload);
}