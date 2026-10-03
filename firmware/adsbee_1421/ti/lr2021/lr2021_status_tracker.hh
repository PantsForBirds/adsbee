#pragma once

// Tracks which LR2021 command a Stat word reports on. The LR2021 returns a command's status at the start of the next
// SPI frame, so a command frame's Stat word reports the previous command and a read's data frame reports the read.
// The last command of a sequence needs a later frame to check it (LR2021::CheckLastCommandStatus).
// Header-only and SDK-free for the host tests.

#include <cstdint>

class LR2021StatusTracker {
   public:
    // A frame was clocked: `opcode` is its first two bytes, 0 for a frame with no command in it.
    void OnFrame(uint16_t opcode) {
        last_frame_was_command_ = opcode != 0;
        if (last_frame_was_command_) {
            previous_command_ = last_command_;
            last_command_ = opcode;
        }
    }

    // After a hardware reset there is no earlier command for a Stat word to report on.
    void Reset() { *this = LR2021StatusTracker(); }

    // Opcode of the command the Stat word of the last frame reports on; 0 when that is no command since the reset.
    uint16_t status_opcode() const { return last_frame_was_command_ ? previous_command_ : last_command_; }

   private:
    uint16_t last_command_ = 0;      // Opcode of the last command frame.
    uint16_t previous_command_ = 0;  // The command frame before it.
    bool last_frame_was_command_ = false;
};
