#pragma once
#include <Arduino.h>
#include "telemetry.h"

// Mount LittleFS only — no buffer allocation. Call in setup() so the old log
// is readable via HTTP during the WiFi window before logging begins.
void logger_mount_fs();

// Open /flight.log for writing and begin recording.
// Call after the WiFi window closes so a new flight never overwrites an
// undownloaded log. Without ENABLE_OTA, call this directly in setup().
void logger_init();

// Append one binary record (TELEM_PACKET_BYTES = 31 bytes).
// pkt must come from telem_pack_buf — ts_ms is already packed at bytes 0-3.
// Called once per transmit cycle (1 Hz) so log and downlink are byte-identical.
void logger_record(const uint8_t* pkt);

// Write PSRAM buffer to /flight.log on LittleFS. Idempotent — safe to call
// multiple times; only the first call writes. Call on DESCENDED transition.
void logger_flush_to_fs();

// Register HTTP routes and start server on port 80.
// Call after WiFi AP is up (after initOTA).
void logger_start_http();

// Drive the HTTP server. Call every loop() alongside ArduinoOTA.handle().
void logger_handle_http();
