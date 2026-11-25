#pragma once

#include <stdint.h>
#include <stdio.h>

/**
 * A HardFault record in four 32-bit words: the RP2040 watchdog scratch registers 0-3. They survive the watchdog reset
 * the HardFault handler triggers (and any other watchdog or software reset), but not a power-on or RUN pin reset. The
 * bootrom and the SDK only use scratch 4-7 (boot to an address, watchdog_reboot()).
 *
 *   word 0: [31:20] magic, [19:16] faults since power-on (saturates at 15), [15:8] check byte,
 *           [7] frame valid, [6] core, [5:0] IPSR of the faulting context (0 = thread mode)
 *   word 1: stacked PC     word 2: stacked LR     word 3: SP the exception frame was pushed to
 *
 * Magic kMagicUnreported: a fault not yet reported. A second fault before it is reported only counts (the first one
 * is usually the cause). kMagicReported: no record, only the count. Anything else (power-on: all zero) is no record.
 *
 * The functions are always inlined: the HardFault handler runs from RAM and must not call into flash.
 */
#define FAULT_RECORD_INLINE inline __attribute__((always_inline))

namespace FaultRecord {

static constexpr uint32_t kMagicUnreported = 0xFA1;
static constexpr uint32_t kMagicReported = 0xFA0;
static constexpr uint8_t kMaxCount = 15;
static constexpr uint16_t kNumWords = 4;

struct Record {
    uint8_t core = 0;          // Core that faulted (0 or 1).
    bool frame_valid = false;  // pc, lr and ipsr come from an exception frame on a sane stack; otherwise they are 0.
    uint8_t ipsr = 0;          // Exception that was running when the fault hit (0 = thread mode).
    uint32_t pc = 0;           // Stacked PC: the faulting instruction.
    uint32_t lr = 0;           // Stacked LR.
    uint32_t sp = 0;           // Stack pointer the frame was pushed to (MSP or PSP, as EXC_RETURN said).
};

struct State {
    bool has_record = false;  // An unreported fault is held in record.
    uint8_t count = 0;        // Faults since power-on.
    Record record;
};

FAULT_RECORD_INLINE uint8_t CheckByte(uint32_t w0_without_check, uint32_t w1, uint32_t w2, uint32_t w3) {
    uint32_t x = w0_without_check ^ w1 ^ (w2 << 1 | w2 >> 31) ^ (w3 << 2 | w3 >> 30) ^ 0x5A13A342u;
    x ^= x >> 16;
    x ^= x >> 8;
    return static_cast<uint8_t>(x);
}

FAULT_RECORD_INLINE void Encode(const State& state, uint32_t words[kNumWords]) {
    uint8_t count = state.count > kMaxCount ? kMaxCount : state.count;
    uint32_t w0, w1 = 0, w2 = 0, w3 = 0;
    if (state.has_record) {
        const Record& r = state.record;
        w0 = kMagicUnreported << 20 | static_cast<uint32_t>(count) << 16 | (r.frame_valid ? 1u << 7 : 0) |
             (r.core & 1u) << 6 | (r.ipsr & 0x3Fu);
        w1 = r.pc;
        w2 = r.lr;
        w3 = r.sp;
    } else {
        w0 = kMagicReported << 20 | static_cast<uint32_t>(count) << 16;
    }
    words[0] = w0 | static_cast<uint32_t>(CheckByte(w0, w1, w2, w3)) << 8;
    words[1] = w1;
    words[2] = w2;
    words[3] = w3;
}

/** Anything that isn't a well-formed record (power-on zeros, another user of the registers) decodes as nothing. */
FAULT_RECORD_INLINE State Decode(const uint32_t words[kNumWords]) {
    State state;
    uint32_t magic = words[0] >> 20;
    uint32_t w0 = words[0] & ~(0xFFu << 8);
    if ((magic != kMagicUnreported && magic != kMagicReported) ||
        CheckByte(w0, words[1], words[2], words[3]) != ((words[0] >> 8) & 0xFF)) {
        return state;
    }
    state.count = (words[0] >> 16) & 0xF;
    if (magic == kMagicReported) {
        if ((words[0] & 0xFF) || words[1] || words[2] || words[3]) {
            return State();  // A reported record keeps nothing but the count.
        }
        return state;
    }
    state.has_record = true;
    state.record.ipsr = words[0] & 0x3F;
    state.record.core = (words[0] >> 6) & 1;
    state.record.frame_valid = (words[0] >> 7) & 1;
    state.record.pc = words[1];
    state.record.lr = words[2];
    state.record.sp = words[3];
    return state;
}

/** A new fault: recorded unless an unreported one is already held (then only counted). */
FAULT_RECORD_INLINE State AddFault(const State& previous, const Record& fault) {
    State state = previous;
    if (state.count < kMaxCount) state.count++;
    if (!state.has_record) {
        state.has_record = true;
        state.record = fault;
    }
    return state;
}

/** After the record has been reported: keep only the count. */
FAULT_RECORD_INLINE State MarkReported(const State& state) {
    State reported;
    reported.count = state.count;
    return reported;
}

/** One line for AT+DEVICE_INFO?: "core 1 PC 0x1006c10a LR 0x2000013b (2 since power-on)" or "none". */
inline int Format(const State& state, char* buf, size_t buf_len) {
    if (!state.has_record) {
        if (state.count) return snprintf(buf, buf_len, "none (%u since power-on)", state.count);
        return snprintf(buf, buf_len, "none");
    }
    const Record& r = state.record;
    if (!r.frame_valid) {
        return snprintf(buf, buf_len, "core %u, bad stack SP 0x%08lx (%u since power-on)", r.core,
                        static_cast<unsigned long>(r.sp), state.count);
    }
    return snprintf(buf, buf_len, "core %u PC 0x%08lx LR 0x%08lx (%u since power-on)", r.core,
                    static_cast<unsigned long>(r.pc), static_cast<unsigned long>(r.lr), state.count);
}

}  // namespace FaultRecord
