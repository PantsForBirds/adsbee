#pragma once

// Which LR2021 command a Stat word reports on.
//
// The LR2021 puts a command's result in the Stat word at the start of the NEXT SPI frame ("Returns CMD_PERR in the
// status of the next command", LR20xx datasheet, and the CommandStatus definitions of GetStatus: "the latest
// command"). So the Stat word read in a command frame reports the command frame before it, and the Stat word of a
// read's data frame (no opcode; the host clocks out zeros) reports the read itself. A command's own frame never
// carries its own status: the last command of a sequence is only checked by a frame sent after it
// (LR2021::CheckLastCommandStatus).
//
// Header-only and SDK-free, so the host tests check it (host_test/test_lr2021_status_tracker.cc).

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
