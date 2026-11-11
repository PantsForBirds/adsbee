#pragma once

#include <cstddef>
#include <cstdint>

/**
 * Character sink for a printf that takes an output function (e.g. the Pico SDK's vfctprintf). Characters collect in a
 * small chunk that is handed to the writer whenever it fills, so a formatted output of any length comes out whole
 * without a buffer sized for the longest one.
 *   ChunkedPrintfSink sink(write);
 *   int len = vfctprintf(ChunkedPrintfSink<Write>::Putc, &sink, format, args);
 *   bool ok = sink.Flush();  // Writes the last partial chunk.
 */
template <typename Write, size_t kChunkBytes = 128>
class ChunkedPrintfSink {
   public:
    // @param[in] write Callable bool(const char* buf, size_t len) that queues bytes on the output.
    explicit ChunkedPrintfSink(Write write) : write_(write) {}

    static void Putc(char c, void* arg) { static_cast<ChunkedPrintfSink*>(arg)->Push(c); }

    void Push(char c) {
        chunk_[len_++] = c;
        if (len_ == kChunkBytes) Flush();
    }

    // Writes what is left in the chunk. Returns false if any write so far failed.
    bool Flush() {
        if (len_ > 0 && !write_(chunk_, len_)) ok_ = false;
        len_ = 0;
        return ok_;
    }

   private:
    Write write_;
    char chunk_[kChunkBytes];
    size_t len_ = 0;
    bool ok_ = true;
};
