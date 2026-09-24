#include "at_client.hh"

#include <stdio.h>
#include <string.h>

#include "board.hh"
#include "pico/stdlib.h"
#include "target_uart.hh"

static void SendCommand(const char* command) {
    TargetUartFlushInput();
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

bool AtProbeAlive(uint32_t timeout_ms) {
    SendCommand("AT+DEVICE_INFO?\r\n");
    // AT+DEVICE_INFO? is a silent-success query (no OK); its body lines include the part
    // ("Part Code:", "CC1314R10 Unique ID:", "CC1314R10 Firmware Version:").
    return WaitForToken("CC1314R10", timeout_ms);
}

uint32_t AtFindConsoleBaud(uint32_t per_rate_timeout_ms) {
    for (uint32_t baud : kConsoleBaudCandidates) {
        TargetUartSetBaud(baud);
        if (AtProbeAlive(per_rate_timeout_ms)) return baud;
    }
    return 0;
}

bool AtQueryVersion(char* version_out, size_t max_len) {
    static const char kPrefix[] = "CC1314R10 Firmware Version:";
    SendCommand("AT+DEVICE_INFO?\r\n");

    char buf[512];
    size_t len = 0;
    // Silent-success query: accumulate until 300 ms of quiet (mirrors the web console).
    while (len < sizeof(buf) - 1) {
        int byte = TargetUartReadByteTimeout(300);
        if (byte < 0) break;
        buf[len++] = (char)byte;
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

bool AtSetConsoleBaud(uint32_t baud) {
    char command[48];
    snprintf(command, sizeof(command), "AT+BAUD_RATE=CONSOLE,%lu\r\n", (unsigned long)baud);
    SendCommand(command);
    return WaitForToken("OK", 500);  // OK arrives at the old baud, then the device switches.
}
