#include "logger.h"
#include <LittleFS.h>
#include <WebServer.h>

// Binary log format — TELEM_PACKET_BYTES (31) bytes per record.
// File header: 4 magic bytes { 'F','C','1', TELEM_PACKET_BYTES }

static constexpr size_t kRecordLen = TELEM_PACKET_BYTES;

static const uint8_t kMagic[4] = { 'F', 'C', '1', (uint8_t)TELEM_PACKET_BYTES };

static File sLogFile;
static bool sFlushed = false;

static WebServer sHttpServer(80);

// ─── Internal ─────────────────────────────────────────────────────────────────

// Flush and close the write handle so the file can be opened for reading.
// Re-open in append mode after streaming so recording continues.
static void prepare_for_download()
{
    if (sLogFile) {
        sLogFile.flush();
        sLogFile.close();
    }
}

static void reopen_for_append()
{
    if (!sFlushed) {
        sLogFile = LittleFS.open("/flight.log", "a");
        if (!sLogFile) Serial.println("[LOG] Warning: failed to reopen log after download");
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────

void logger_mount_fs()
{
    if (!LittleFS.begin(true)) {
        Serial.println("[LOG] LittleFS mount failed");
    }
}

void logger_init()
{
    if (!LittleFS.begin(true)) {
        Serial.println("[LOG] LittleFS mount failed");
        return;
    }

    sLogFile = LittleFS.open("/flight.log", "w");
    if (!sLogFile) {
        Serial.println("[LOG] Failed to open /flight.log — logging disabled");
        return;
    }
    sLogFile.write(kMagic, sizeof(kMagic));
    Serial.println("[LOG] Ready — logging to LittleFS");
}

void logger_record(const uint8_t* pkt)
{
    if (sLogFile) sLogFile.write(pkt, kRecordLen);
}

void logger_flush_to_fs()
{
    if (sFlushed) return;
    if (sLogFile) {
        sLogFile.flush();
        sLogFile.close();
    }
    sFlushed = true;
    Serial.println("[LOG] Log closed on landing");
}

void logger_start_http()
{
    sHttpServer.on("/log", HTTP_GET, []() {
        prepare_for_download();
        if (!LittleFS.exists("/flight.log")) {
            sHttpServer.send(404, "text/plain", "No log on disk\n");
            reopen_for_append();
            return;
        }
        File f = LittleFS.open("/flight.log", "r");
        sHttpServer.streamFile(f, "application/octet-stream");
        f.close();
        reopen_for_append();
    });

    sHttpServer.on("/log/status", HTTP_GET, []() {
        size_t fileSize = 0;
        size_t records  = 0;
        if (LittleFS.exists("/flight.log")) {
            File f = LittleFS.open("/flight.log", "r");
            if (f) {
                fileSize = f.size();
                records  = (fileSize > sizeof(kMagic)) ? (fileSize - sizeof(kMagic)) / kRecordLen : 0;
                f.close();
            }
        }
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "file_bytes=%u  records=%u  flushed=%d  fs_exists=%d\n",
                 (unsigned)fileSize,
                 (unsigned)records,
                 sFlushed ? 1 : 0,
                 LittleFS.exists("/flight.log") ? 1 : 0);
        sHttpServer.send(200, "text/plain", msg);
    });

    sHttpServer.on("/log/clear", HTTP_GET, []() {
        if (sLogFile) sLogFile.close();
        LittleFS.remove("/flight.log");
        sFlushed = false;
        sLogFile = LittleFS.open("/flight.log", "w");
        if (sLogFile) sLogFile.write(kMagic, sizeof(kMagic));
        sHttpServer.send(200, "text/plain", "Log cleared — ready for next flight\n");
    });

    sHttpServer.begin();
    Serial.println("[LOG] HTTP server on port 80");
    Serial.println("[LOG]   GET /log         — download binary log");
    Serial.println("[LOG]   GET /log/status  — file stats");
    Serial.println("[LOG]   GET /log/clear   — reset for next flight");
}

void logger_handle_http()
{
    sHttpServer.handleClient();
}
