#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "cpp_at.hh"

/**
 * Prints the AT+HELP listing in the same format as cppAT's built-in help, one command entry at a time:
 *   AT Command Help Menu:\r\n
 *   <COMMAND>: \r\n
 *   \t<help_string>\r\n      (or whatever the command's help_callback prints)
 * Help strings go to the writer whole, so they are never cut to a printf buffer's size. A firmware registers its own
 * "HELP" command (found before cppAT's built-in one) that calls Start(), then either PrintAll() where console writes
 * block until sent, or PrintNext() from the main loop whenever NextEntryMaxBytes() fit in its TX buffer, so a long
 * listing never overfills the buffer or holds up the main loop.
 */
class ATHelpPrinter {
   public:
    static constexpr std::string_view kHeader = "AT Command Help Menu:\r\n";
    // Space reserved for an entry whose help_callback prints its own text. Callbacks must print less than this.
    static constexpr size_t kHelpCallbackMaxBytes = 2048;

    ATHelpPrinter(const CppAT::ATCommandDef_t* commands, uint16_t num_commands)
        : commands_(commands), num_commands_(num_commands) {}

    // Begins a listing. A listing still in progress starts over.
    void Start() {
        header_pending_ = true;
        next_index_ = 0;
        active_ = true;
    }
    // Abandons a listing in progress (e.g. when the console output is dropped).
    void Stop() { active_ = false; }
    bool Active() const { return active_; }

    // Upper bound on the bytes the next PrintNext() call writes (0 when no listing is in progress).
    size_t NextEntryMaxBytes() const {
        if (!active_) return 0;
        size_t bytes = header_pending_ ? kHeader.length() : 0;
        if (next_index_ < num_commands_) bytes += EntryMaxBytes(commands_[next_index_]);
        return bytes;
    }

    // Bytes an entry writes, or kHelpCallbackMaxBytes plus its name line if it has a help_callback.
    static size_t EntryMaxBytes(const CppAT::ATCommandDef_t& def) {
        size_t name_line = def.command.length() + kNameSuffix.length();
        if (def.help_callback) return name_line + kHelpCallbackMaxBytes;
        return name_line + kHelpPrefix.length() + def.help_string.length() + kLineEnd.length();
    }

    /**
     * Writes the header (first call only) and the next command's entry.
     * @param[in] write Callable bool(const char* buf, size_t len) that queues bytes on the console.
     * @retval True while more entries remain.
     */
    template <typename Write>
    bool PrintNext(Write&& write) {
        if (!active_) return false;
        if (header_pending_) {
            write(kHeader.data(), kHeader.length());
            header_pending_ = false;
        }
        if (next_index_ < num_commands_) {
            PrintEntry(commands_[next_index_], write);
            next_index_++;
        }
        active_ = next_index_ < num_commands_;
        return active_;
    }

    // Writes a whole listing at once. For consoles whose writes wait for room instead of dropping.
    template <typename Write>
    void PrintAll(Write&& write) {
        Start();
        while (PrintNext(write)) {
        }
    }

   private:
    static constexpr std::string_view kNameSuffix = ": \r\n";
    static constexpr std::string_view kHelpPrefix = "\t";
    static constexpr std::string_view kLineEnd = "\r\n";

    template <typename Write>
    static void PrintEntry(const CppAT::ATCommandDef_t& def, Write& write) {
        write(def.command.data(), def.command.length());
        write(kNameSuffix.data(), kNameSuffix.length());
        if (def.help_callback) {
            def.help_callback();
            return;
        }
        write(kHelpPrefix.data(), kHelpPrefix.length());
        write(def.help_string.data(), def.help_string.length());
        write(kLineEnd.data(), kLineEnd.length());
    }

    const CppAT::ATCommandDef_t* commands_;
    uint16_t num_commands_;
    uint16_t next_index_ = 0;
    bool header_pending_ = false;
    bool active_ = false;
};
