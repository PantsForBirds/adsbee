#include "at_client.hh"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "target_uart.hh"

static void SendCommand(const char* command) {
    TargetUartFlushInput();
    // Bytes at a wrong rate leave garbage in the console's AT line buffer. A leading line end closes that line first;
    // the console ignores blank lines.
    static const uint8_t kLineEnd[] = {'\r', '\n'};
    TargetUartWriteBlocking(kLineEnd, sizeof(kLineEnd));
    TargetUartWriteBlocking((const uint8_t*)command, strlen(command));
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
