#include "hard_fault.hh"

#include "comms.hh"
#include "hardware/address_mapped.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/watchdog.h"
#include "hardware/sync.h"

namespace {

// The rest of the frame, for the boot log. In RAM that survives the watchdog reset (our bootloader only uses the first
// ~12 KB of main RAM and scratch Y); check ties it to the scratch record, so a stale or overwritten block is ignored.
struct Detail {
    uint32_t r0, r1, r2, r3, r12, xpsr, exc_return;
    uint32_t pc;  // Same as the scratch record's.
    uint32_t check;
};
constexpr uint32_t kDetailSeed = 0xDE7A1342u;
__attribute__((always_inline)) inline uint32_t DetailCheck(const Detail& d) {
    return d.r0 ^ d.r1 ^ d.r2 ^ d.r3 ^ d.r12 ^ d.xpsr ^ d.exc_return ^ d.pc ^ kDetailSeed;
}
Detail detail __attribute__((section(".uninitialized_data.hard_fault_detail")));

// The handler's own stack, one per core: the faulting code's stack may be the problem.
constexpr uint32_t kStackWords = 32;
uint32_t __attribute__((used)) handler_stacks[2][kStackWords];

// Hardware spin lock that keeps two cores faulting at once from interleaving their records. -1 until Init().
volatile int8_t lock_num = -1;

FaultRecord::State boot_state;
Detail boot_detail;
bool boot_detail_valid = false;

__attribute__((always_inline)) inline void ReadScratch(uint32_t words[FaultRecord::kNumWords]) {
    for (uint16_t i = 0; i < FaultRecord::kNumWords; i++) words[i] = watchdog_hw->scratch[i];
}
__attribute__((always_inline)) inline void WriteScratch(const uint32_t words[FaultRecord::kNumWords]) {
    for (uint16_t i = 0; i < FaultRecord::kNumWords; i++) watchdog_hw->scratch[i] = words[i];
}

}  // namespace

// Called by isr_hardfault on the handler stack: msp and psp as they were at the fault, EXC_RETURN, the core number.
extern "C" void __attribute__((noreturn, used, noinline, section(".time_critical.hard_fault_record")))
hard_fault_record(uint32_t msp, uint32_t psp, uint32_t exc_return, uint32_t core) {
    FaultRecord::Record fault;
    fault.core = static_cast<uint8_t>(core & 1);
    fault.sp = (exc_return & 0x4) ? psp : msp;
    const uint32_t* frame = nullptr;
    if ((fault.sp & 0x3) == 0 && fault.sp >= SRAM_BASE && fault.sp <= SRAM_END - 8 * sizeof(uint32_t)) {
        frame = reinterpret_cast<const uint32_t*>(fault.sp);
        fault.frame_valid = true;
        fault.lr = frame[5];
        fault.pc = frame[6];
        fault.ipsr = frame[7] & 0x3F;
    }

    // Both cores may fault at once (or one in the middle of the other's record): one at a time. Bounded, so a lock
    // left held can't stop the reset.
    io_rw_32* lock = lock_num >= 0 ? &sio_hw->spinlock[lock_num] : nullptr;
    for (uint32_t tries = 0; lock && !*lock && tries < 100000; tries++) {
    }

    uint32_t words[FaultRecord::kNumWords];
    ReadScratch(words);
    FaultRecord::State previous = FaultRecord::Decode(words);
    FaultRecord::State state = FaultRecord::AddFault(previous, fault);
    FaultRecord::Encode(state, words);
    WriteScratch(words);
    if (!previous.has_record && frame) {
        detail.r0 = frame[0];
        detail.r1 = frame[1];
        detail.r2 = frame[2];
        detail.r3 = frame[3];
        detail.r12 = frame[4];
        detail.xpsr = frame[7];
        detail.exc_return = exc_return;
        detail.pc = fault.pc;
        detail.check = DetailCheck(detail);
    }
    if (lock) *lock = 0;  // Release: a fault on the other core meanwhile still gets counted.

    // Reset everything but the oscillators through the watchdog, at once (as watchdog_reboot(), without its delay
    // and without the flash calls). Scratch 4 = 0: the bootrom boots normally.
    watchdog_hw->scratch[4] = 0;
    psm_hw->wdsel = PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS);
    watchdog_hw->ctrl = WATCHDOG_CTRL_TRIGGER_BITS;
    while (true) {
        __asm volatile("nop");
    }
}

// Replaces the SDK's weak isr_hardfault (a bkpt). Switches to this core's handler stack before any C code runs.
extern "C" void __attribute__((naked, used, section(".time_critical.isr_hardfault"))) isr_hardfault() {
    __asm volatile(
        "mrs r0, msp\n"
        "mrs r1, psp\n"
        "mov r2, lr\n"            // EXC_RETURN
        "ldr r3, =0xd0000000\n"   // SIO CPUID
        "ldr r3, [r3]\n"
        "ldr r4, =handler_stacks\n"
        "lsls r5, r3, #7\n"       // 128 bytes (kStackWords words) per core
        "adds r4, r4, r5\n"
        "adds r4, #128\n"
        "mov sp, r4\n"
        "ldr r4, =hard_fault_record\n"
        "bx r4\n"
        ".ltorg\n");
}
static_assert(kStackWords * sizeof(uint32_t) == 128, "isr_hardfault assumes 128-byte handler stacks");

namespace HardFault {

void Init() {
    uint32_t words[FaultRecord::kNumWords];
    ReadScratch(words);
    boot_state = FaultRecord::Decode(words);
    boot_detail = detail;
    boot_detail_valid =
        boot_state.has_record && boot_detail.pc == boot_state.record.pc && boot_detail.check == DetailCheck(boot_detail);
    detail.check = ~DetailCheck(detail);  // Spent: a later record must write its own.
    lock_num = static_cast<int8_t>(spin_lock_claim_unused(true));
}

void ReportBootFault() {
    if (!boot_state.has_record) {
        return;
    }
    const FaultRecord::Record& r = boot_state.record;
    CONSOLE_ERROR("HardFault", "Reset by a HardFault on core %u: PC 0x%08lx LR 0x%08lx SP 0x%08lx IPSR %u (%u since power-on).",
                  r.core, static_cast<unsigned long>(r.pc), static_cast<unsigned long>(r.lr),
                  static_cast<unsigned long>(r.sp), r.ipsr, boot_state.count);
    if (boot_detail_valid) {
        const Detail& d = boot_detail;
        CONSOLE_ERROR("HardFault", "R0 0x%08lx R1 0x%08lx R2 0x%08lx R3 0x%08lx R12 0x%08lx xPSR 0x%08lx EXC_RETURN 0x%08lx",
                      static_cast<unsigned long>(d.r0), static_cast<unsigned long>(d.r1),
                      static_cast<unsigned long>(d.r2), static_cast<unsigned long>(d.r3),
                      static_cast<unsigned long>(d.r12), static_cast<unsigned long>(d.xpsr),
                      static_cast<unsigned long>(d.exc_return));
    }
    // Reported: keep only the count (unless a new fault was recorded since, which is then the next boot's).
    uint32_t words[FaultRecord::kNumWords];
    ReadScratch(words);
    FaultRecord::State now = FaultRecord::Decode(words);
    if (now.has_record && now.record.pc == r.pc && now.record.lr == r.lr && now.record.sp == r.sp) {
        FaultRecord::Encode(FaultRecord::MarkReported(now), words);
        WriteScratch(words);
    }
}

const FaultRecord::State& BootState() { return boot_state; }

#ifdef HARDWARE_UNIT_TESTS
void __attribute__((noinline)) TriggerFault() {
    uint32_t value;
    uint32_t address = SRAM_BASE + 1;  // Unaligned word load: a HardFault on the Cortex-M0+.
    __asm volatile("ldr %0, [%1]" : "=r"(value) : "r"(address) : "memory");
    (void)value;
    while (true) {
    }
}
#endif

}  // namespace HardFault
