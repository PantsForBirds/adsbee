#pragma once

#include <stddef.h>
#include <string.h>

// Status text formatting for CdcPrintf() (status.cc). Pure so it can be host-tested (host_test/test_cdc_text.cc).

// CdcPrintf() formats into a fixed stack buffer of this size. Keep each message well below it; split long ones
// across calls.
static constexpr size_t kCdcTextMax = 256;

// Length of the text to send, given vsnprintf's return value `len` for a buffer of `size` bytes. When the message
// didn't fit, the end of the buffer is overwritten with "...\r\n", so a cut-off line is visible as one and still ends
// the line. Returns 0 for an encoding error.
inline size_t CdcTextFinish(char* buf, size_t size, int len) {
    if (len <= 0 || size == 0) return 0;
    if ((size_t)len < size) return (size_t)len;
    static const char kMark[] = "...\r\n";
    const size_t mark_len = sizeof(kMark) - 1;
    if (size - 1 < mark_len) return size - 1;  // Too small to mark; vsnprintf already terminated it.
    memcpy(buf + size - 1 - mark_len, kMark, mark_len + 1);
    return size - 1;
}
