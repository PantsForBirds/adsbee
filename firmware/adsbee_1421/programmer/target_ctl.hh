#pragma once

// SYNC / RESET_N control for the attached ADSBee 1421.
//
// SYNC (bootloader backdoor, active high) is push-pull: the module's pull-down means low is
// the safe idle. RESET_N is pseudo open-drain: pulled up (module + RP2040 pulls) except while
// actively pulsing low, so an attached debugger or the module itself is never fought.

void TargetCtlInit();

void TargetSetSync(bool high);
void TargetPulseReset();  // Drive RESET_N low for kResetPulseMs, then release.

void TargetResetIntoApp();         // SYNC low + reset pulse: boot ROM runs the application.
void TargetResetIntoBootloader();  // SYNC high + reset pulse, SYNC left high: ROM serial bootloader.
