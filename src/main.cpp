#include <Arduino.h>

#include "imu.h"
#include "transmitter.h"
#include "bar.h"
#include "servo.h"

static constexpr uint32_t kTransmitIntervalMs = 50;
static uint32_t lastTxMs = 0;

// Servo altitude trigger settings (AGL metres)
static constexpr float kServoAltTrigger    = 242.3f;  // metres AGL (adjust as needed)
static constexpr float kServoAltHysteresis = 1.0f;    // metres — avoids chattering
static constexpr int   maxAngle            = 45;       // max airbrake angle (degrees)
static bool servo_triggered = false;

void setup() {
  Serial.begin(115200);
  delay(200);

  // if (!imuInit()) {
  //   Serial.println("IMU init failed, halting.");
  //   while (true) { delay(1000); }
  // }

  if (!transmitterInit()) {
    Serial.println("Transmitter init failed, halting.");
    while (true) { delay(1000); }
  }

  // bar_init() calls Wire1.begin(18, 19) — must come before servo_init()
  // so Wire1 is only initialised once.
  if (!bar_init()) {
    Serial.println("BMP280 init failed, halting.");
    while (true) { delay(1000); }
  }

  // Sample ground pressure over 10 seconds for AGL reference.
  // Board must be stationary on the launch pad during this window.
  float groundPressure = bar_calibrate();  // 100 samples, 10 000 ms
  if (groundPressure < 0.0f) {
    Serial.println("Barometer calibration failed, halting.");
    while (true) { delay(1000); }
  }

  // servo_init() shares Wire1 — Wire1.begin() is already done above.
  if (!servo_init()) {
    Serial.println("Servo init failed, halting.");
    while (true) { delay(1000); }
  }
  servo_set_angle(0, 90);  // Test: move channel 0 to 90° on startup

  Serial.println("System ready.");
}

void loop() {
  transmitterPoll();

  float temp, pressure, altitude;
  if (bar_read_all(temp, pressure, altitude)) {
    Serial.printf("Temp: %.2f °C | Pressure: %.2f hPa | Altitude AGL: %.2f m\n",
                  temp, pressure, altitude);
  } else {
    Serial.println("Failed to read from BMP280");
  }

  // ImuSample sample;
  // if (!imuRead(sample)) { return; }

  const uint32_t now = millis();
  if (now - lastTxMs < kTransmitIntervalMs) {
    return;
  }
  lastTxMs = now;

  String payload;
  payload.reserve(96);
  // payload += "Roll: " + String(sample.roll, 2) + ", Pitch: " + String(sample.pitch, 2) + ", Yaw: " + String(sample.yaw, 2) + ", ";
  payload += "Temp: "     + String(temp,     2);
  payload += ", Pressure: " + String(pressure, 2);
  payload += ", AltAGL: "   + String(altitude, 2);
  payload += ", ServoTriggered: " + String(servo_triggered ? 1 : 0);

  Serial.println(payload);
  transmitterSend(payload);

  // Altitude-based airbrake control with hysteresis
  if (altitude >= kServoAltTrigger && !servo_triggered) {
    servo_set_angle(0, maxAngle);
    servo_triggered = true;
    Serial.println("Airbrake deployed");
  } else if (servo_triggered && altitude <= (kServoAltTrigger - kServoAltHysteresis)) {
    servo_set_angle(0, 0);
    servo_triggered = false;
    Serial.println("Airbrake retracted");
  }
}