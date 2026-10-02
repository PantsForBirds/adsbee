#include "at_client.hh"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "target_uart.hh"

static void SendCommand(const char* command) {
    TargetUartFlushInput();
    // Bytes at a wrong rate (and the autobaud trigger's held-low line) leave garbage in the console's AT line buffer,
    // often NUL bytes that would truncate the command appended to it. A leading line end closes that line first; the
    // console ignores blank lines.
    static const uint8_t kLineEnd[] = {'\r', '\n'};
    TargetUartWriteBlocking(kLineEnd, sizeof(kLineEnd));
    TargetUartWriteBlocking((const uint8_t*)command, strlen(command));
}

// Reads until `token` appears in the stream or timeout_ms passes with no complete match.
static bool WaitForToken(const char* token, uint32_t timeout_ms) {
    size_t matched = 0;
    size_t token_len = strlen(token);
    absolute_time_t deadline = delayed_by_ms(get_absolute_time(), timeout_ms);
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        int byte = TargetUartReadByteTimeout(20);
        if (byte < 0) continue;
        matched = (byte == token[matched]) ? matched + 1 : (byte == token[0] ? 1 : 0);
        if (matched == token_len) return true;
    }
    return false;
}

// Milliseconds to clock num_bytes out at baud (10 bits per byte).
static uint32_t BytesToMs(uint32_t num_bytes, uint32_t baud) {
    return (uint32_t)(((uint64_t)num_bytes * 10u * 1000u + baud - 1) / baud);
}

uint32_t AtProbeConsoleBaud() {
    uint32_t timeout_ms = 250 + 2 * BytesToMs(1024, TargetUartGetBaud());
    SendCommand("AT+BAUD_RATE?\r\n");
    // Silent-success query: one "BAUD_RATE=CONSOLE,<baud>" line, no OK.
    if (!WaitForToken("BAUD_RATE=CONSOLE,", timeout_ms < 1000 ? timeout_ms : 1000)) return 0;
    uint32_t baud = 0;
    for (int i = 0; i < 12; i++) {
        int byte = TargetUartReadByteTimeout(20);
        if (byte < '0' || byte > '9') break;
        baud = baud * 10 + (uint32_t)(byte - '0');
    }
    // Consume the rest of the line so it never reaches a host that the bridge resumes forwarding to.
    for (int i = 0; i < 4; i++) {
        int byte = TargetUartReadByteTimeout(20);
        if (byte < 0 || byte == '\n') break;
    }
    return baud;
}

bool AtQueryVersion(char* version_out, size_t max_len) {
    static const char kPrefix[] = "CC1314R10 Firmware Version:";
    SendCommand("AT+DEVICE_INFO?\r\n");

    char buf[512];
    size_t len = 0;
    // Silent-success query: accumulate until 300 ms of quiet (mirrors the web console). The reply runs past the buffer
    // (the OTA keys); the rest is read and dropped, so it never reaches the host when pass-through starts.
    while (true) {
        int byte = TargetUartReadByteTimeout(300);
        if (byte < 0) break;
        if (len < sizeof(buf) - 1) buf[len++] = (char)byte;
    }
    buf[len] = '\0';

    char* line = strstr(buf, kPrefix);
    if (line == nullptr) return false;
    line += sizeof(kPrefix) - 1;
    while (*line == ' ') line++;
    size_t out = 0;
    while (out < max_len - 1 && *line != '\0' && *line != '\r' && *line != '\n') {
        version_out[out++] = *line++;
    }
    version_out[out] = '\0';
    return out > 0;
}
