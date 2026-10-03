#pragma once

#include <stdint.h>

/**
 * Internal RAM diagnostics, compiled in only with `idf.py -DHEAP_DIAGNOSTICS=1 build`. Mark() records free heap at a
 * boot step; Report() logs the marks, current heap stats and task stack high-water marks.
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
