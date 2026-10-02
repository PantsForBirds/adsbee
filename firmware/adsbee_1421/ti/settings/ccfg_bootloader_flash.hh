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
 *
 * If an earlier erase-path write failed and its original CCFG could not be put back (RestorePending()), the
 * plan instead rewrites that original image, kept in RAM since then (Plan::restore_retry), whatever `enable` is.
 */
Plan Prepare(bool enable);

/**
 * True while the CCFG may be erased or half written: an erase-path write failed and so did every attempt to
 * write the original back. Cleared by a successful Apply() of the restore_retry plan Prepare() then returns.
 */
bool RestorePending();

/**
 * Writes the plan from the most recent Prepare() to flash and verifies it by reading the whole sector
 * back. Masks interrupts and turns the VIMS cache and line buffer off for the duration, as TI
 * requires for flash writes. The caller should drain console TX first.
 *
 * kEraseAndProgram: erase, program the new image, compare the whole sector. If any step fails it
 * erases again and reprograms the original image (up to kRestoreAttempts times) so the sector
 * isn't left erased.
 *
 * @param[in] plan Result of the most recent Prepare().
 * @param[in] enable The requested backdoor state (for the report).
 * @retval What happened, with the read-back BL_CONFIG and CCFG CRC. For report.Failed(), print
 * FailureBanner(report).
 */
WriteReport Apply(const Plan& plan, bool enable);

}  // namespace CcfgBootloader
