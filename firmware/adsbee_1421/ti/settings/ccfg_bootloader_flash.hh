#pragma once

#include "ccfg_bootloader.hh"

// Device side of AT+BOOTLOADER_PIN. See ccfg_bootloader.hh for the CCFG layout and write methods.
namespace CcfgBootloader {

// Live BL_CONFIG, used by the boot ROM at the next reset.
uint32_t ReadBlConfig();

/**
 * Copies the CCFG to RAM and plans the change for Apply(); writes no flash. While RestorePending(), the plan
 * instead rewrites the original CCFG kept in RAM, whatever `enable` is.
 */
Plan Prepare(bool enable);

// True while the CCFG may be erased or half written because restoring the original failed.
bool RestorePending();

/**
 * Writes and verifies the plan from the most recent Prepare(). Masks interrupts and disables the flash cache while
 * writing; drain console TX first. If an erase-path write fails, rewrites the original image (up to
 * kRestoreAttempts times) so the sector isn't left erased.
 *
 * @param[in] plan Result of the most recent Prepare().
 * @param[in] enable The requested backdoor state (for the report).
 * @retval What happened, with read-back values. For report.Failed(), print FailureBanner(report).
 */
WriteReport Apply(const Plan& plan, bool enable);

}  // namespace CcfgBootloader
