#pragma once

#include "ccfg_bootloader.hh"

// Device side of AT+BOOTLOADER_PIN: reads the live CCFG and rewrites its BL_CONFIG word. See
// ccfg_bootloader.hh for the CCFG layout and the reasoning behind each write method.
namespace CcfgBootloader {

/**
 * Reads BL_CONFIG from the CCFG flash sector (the live value the boot ROM uses at the next reset).
 */
uint32_t ReadBlConfig();

/**
 * Copies the whole CCFG sector to RAM and computes the new image and write method for enabling or
 * disabling the SYNC backdoor. Touches no flash. The RAM images are kept for Apply().
 */
Plan Prepare(bool enable);

/**
 * Writes the plan from the most recent Prepare() to flash and verifies it by reading the sector
 * back. Masks interrupts and turns the VIMS cache and line buffer off for the duration, as TI
 * requires for flash writes. The caller should drain console TX first.
 *
 * kEraseAndProgram: erase, program the new image, compare the whole sector. If any step fails it
 * erases again and reprograms the original image (up to kRestoreAttempts times) so the sector
 * isn't left erased.
 *
 * @param[in] plan Result of the most recent Prepare().
 * @param[out] error Set to a description of the failure when returning false.
 * @param[out] restored Set to true if the write failed and the original CCFG was restored and
 * verified.
 * @retval True if the sector now holds plan.new_bl_config and nothing else changed.
 */
bool Apply(const Plan& plan, const char*& error, bool& restored);

static constexpr int kRestoreAttempts = 3;

}  // namespace CcfgBootloader
