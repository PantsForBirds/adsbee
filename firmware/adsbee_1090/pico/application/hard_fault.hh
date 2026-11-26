#pragma once

#include "utils/fault_record.hh"

/**
 * HardFault on either core: record it and reset the chip through the watchdog, so a fault never leaves the unit up
 * but not receiving (a core 1 fault used to stop only core 1, while core 0 kept feeding the watchdog).
 *
 * The record (core, PC, LR, IPSR, SP, faults since power-on) is kept in watchdog scratch 0-3 (see fault_record.hh).
 * R0-R3, R12, xPSR and EXC_RETURN go to a small uninitialized RAM block, reported only if it is intact and belongs to
 * the same fault. The handler is isr_hardfault (it replaces the SDK's bkpt) and runs from RAM on its own stack.
 */
namespace HardFault {

/** At the top of main(): reads the record left by the previous boot and reserves the handler's spin lock. */
void Init();

/** Logs the record left by the previous boot (once, at error level), then marks it reported. */
void ReportBootFault();

/** Core 0 found core 1 stalled and stops feeding the watchdog: record why the coming reset happens. */
void RecordStall(uint32_t stalled_ms, uint32_t passes);

/** The record found at this boot (kept for AT+DEVICE_INFO? after it has been reported). */
const FaultRecord::State& BootState();

#ifdef HARDWARE_UNIT_TESTS
/** AT+TEST=HARDFAULT_*: an unaligned load (a HardFault on the M0+) on the calling core. Never returns. */
void TriggerFault();
#endif

}  // namespace HardFault
