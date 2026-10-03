#pragma once

#include <stddef.h>
#include <string.h>

// Status text formatting for CdcPrintf() (status.cc). Pure so it can be host-tested (host_test/test_cdc_text.cc).

// CdcPrintf()'s stack buffer size. Split longer messages across calls.
static constexpr size_t kCdcTextMax = 256;

// Length to send, given vsnprintf's return `len` for a `size`-byte buffer. A message that didn't fit ends in
// "...\r\n". Returns 0 for an encoding error.
inline size_t CdcTextFinish(char* buf, size_t size, int len) {
    if (len <= 0 || size == 0) return 0;
    if ((size_t)len < size) return (size_t)len;
    static const char kMark[] = "...\r\n";
    const size_t mark_len = sizeof(kMark) - 1;
    if (size - 1 < mark_len) return size - 1;  // Too small to mark; vsnprintf already terminated it.
    memcpy(buf + size - 1 - mark_len, kMark, mark_len + 1);
    return size - 1;
}
