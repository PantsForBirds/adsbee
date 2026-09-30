#pragma once

#include <stdint.h>

/**
 * Optional internal-RAM diagnostics for tuning memory use. Compiled in only when the build defines HEAP_DIAGNOSTICS
 * (idf.py -DHEAP_DIAGNOSTICS=1 build); otherwise every call is an empty inline function.
 *
 * Mark() records free internal heap at a point during boot. Report() logs, at WARNINGS level so it shows with the
 * default log level: the boot marks, current internal free / largest free block / minimum ever free, DMA-capable free,
 * and each task's stack high-water mark.
 */
namespace HeapDiagnostics {
#ifdef HEAP_DIAGNOSTICS
void Mark(const char* label);
void Report();
#else
inline void Mark(const char* label) {}
inline void Report() {}
#endif
}  // namespace HeapDiagnostics
