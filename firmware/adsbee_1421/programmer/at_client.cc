#include "at_client.hh"

#include <stdio.h>
#include <string.h>

#include "baud_follower.hh"
#include "baud_store.hh"
#include "board.hh"
#include "pico/stdlib.h"
#include "target_uart.hh"

static uint32_t last_console_baud = 0;  // Last rate the console answered at.
static uint32_t last_boot_baud = 0;     // Last rate the console booted at (0: see the flash store).

static void SendCommand(const char* command) {
    TargetUartFlushInput();
    // Probes at a wrong rate (and host bytes sent at one) leave a garbage line in the console's AT buffer, often with
    // NUL bytes that would truncate the command appended to it. A leading line end closes that line first; the
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

uint32_t AtProbeTimeoutMs(uint32_t baud) {
    uint32_t timeout_ms = 250 + 2 * BytesToMs(1024, baud);
    return timeout_ms < 1000 ? timeout_ms : 1000;
}

bool AtProbeAlive(uint32_t timeout_ms) {
    if (timeout_ms == 0) timeout_ms = AtProbeTimeoutMs(TargetUartGetBaud());
    SendCommand("AT+BAUD_RATE?\r\n");
    // Silent-success query: one "BAUD_RATE=CONSOLE,<baud>" line, no OK. Consume the rest of the line so it never
    // reaches a host that the bridge resumes forwarding to.
    if (!WaitForToken("BAUD_RATE=CONSOLE,", timeout_ms)) return false;
    for (int i = 0; i < 12; i++) {
        int byte = TargetUartReadByteTimeout(20);
        if (byte < 0 || byte == '\n') break;
    }
    last_console_baud = TargetUartGetBaud();
    return true;
}

uint32_t AtFindConsoleBaud(const uint32_t* candidates, size_t num_candidates, AtAbortFn abort) {
    for (size_t i = 0; i < num_candidates; i++) {
        if (abort != nullptr && abort()) return 0;
        TargetUartSetBaud(candidates[i]);
        if (AtProbeAlive()) return candidates[i];
    }
    return 0;
}

// Default probe order (AtFindConsoleBaud()), with `first` ahead of the remembered rates.
static size_t DefaultCandidates(const uint32_t* first, size_t num_first, uint32_t* out, size_t max_out) {
    uint32_t preferred[8];
    size_t num_preferred = 0;
    for (size_t i = 0; i < num_first && num_preferred < 5; i++) preferred[num_preferred++] = first[i];
    preferred[num_preferred++] = last_console_baud;
    preferred[num_preferred++] = AtBootBaud();
    return BuildConsoleBaudCandidates(preferred, num_preferred, out, max_out);
}

static constexpr size_t kMaxCandidates = 24;

uint32_t AtFindConsoleBaud() {
    uint32_t candidates[kMaxCandidates];
    size_t count = DefaultCandidates(nullptr, 0, candidates, kMaxCandidates);
    return AtFindConsoleBaud(candidates, count);
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
    // The OK queues behind report output already in the console's 8 kB TX ring, which takes a while to drain at low
    // rates. "OK\r\n" rather than "OK" so binary report data (MAVLink) is unlikely to fake it; a false match is caught
    // by the probe at the new rate anyway.
    uint32_t timeout_ms = 300 + BytesToMs(8192, TargetUartGetBaud());
    return WaitForToken("OK\r\n", timeout_ms < 2500 ? timeout_ms : 2500);
}

// Lets the console finish switching (it closes and reopens its UART after the OK) while USB stays serviced.
static void WaitForSwitch() {
    for (int i = 0; i < 5; i++) (void)TargetUartReadByteTimeout(10);
}

// With the UART at the console's rate, moves it to `target` and confirms it answers there.
static bool MoveConsole(uint32_t target) {
    if (!AtSetConsoleBaud(target)) return false;
    WaitForSwitch();
    TargetUartSetBaud(target);
    return AtProbeAlive() || AtProbeAlive();
}

uint32_t AtRenegotiateConsole(uint32_t target_baud, const uint32_t* likely, size_t num_likely, AtAbortFn abort) {
    // Where the console probably is: one probe each, and straight to the move when it answers.
    for (size_t i = 0; i < num_likely; i++) {
        uint32_t from = likely[i];
        if (from == 0) continue;
        bool tried = false;
        for (size_t j = 0; j < i; j++) tried |= likely[j] == from;
        if (tried) continue;
        if (abort != nullptr && abort()) return 0;
        TargetUartSetBaud(from);
        if (target_baud == 0 || from == target_baud) {
            if (AtProbeAlive()) return from;
            continue;
        }
        // An OK proves the console was at `from`. Without one it may be elsewhere (a probe would cost as much as the
        // command), so move on.
        if (AtSetConsoleBaud(target_baud)) {
            WaitForSwitch();
            TargetUartSetBaud(target_baud);
            if (AtProbeAlive() || AtProbeAlive()) return target_baud;
            break;  // Acknowledged but silent at the new rate: search everywhere.
        }
    }

    // Not where expected: search the default probe order, then move it from wherever it is.
    uint32_t candidates[kMaxCandidates];
    uint32_t first[] = {target_baud};
    size_t count = DefaultCandidates(first, target_baud != 0 ? 1 : 0, candidates, kMaxCandidates);
    uint32_t found = AtFindConsoleBaud(candidates, count, abort);
    if (found == 0) {
        if (target_baud != 0) TargetUartSetBaud(target_baud);
        return 0;
    }
    if (target_baud == 0 || found == target_baud) return found;
    if (abort != nullptr && abort()) return found;
    if (MoveConsole(target_baud)) return target_baud;
    // Refused (an image that doesn't accept the rate) or lost on the way: find it again.
    TargetUartSetBaud(found);
    if (AtProbeAlive()) return found;
    return AtFindConsoleBaud(candidates, count, abort);
}

void AtNoteBootBaud(uint32_t baud) {
    if (baud == 0) return;
    last_boot_baud = baud;
    if (BaudStoreLoad() != baud) BaudStoreSave(baud);
}

uint32_t AtBootBaud() { return last_boot_baud != 0 ? last_boot_baud : BaudStoreLoad(); }
