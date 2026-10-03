#include "comms.hh"  // For debug logging.
#include "utest.h"

// #define TEST_EEPROM

CPP_AT_CALLBACK(ATTestCallback);

/**
 * Runs a function on the demodulator ISR core (core 1 with ISRS_ON_CORE1, else the caller) and waits for it to return.
 * Lets tests on core 0 read per-core state such as SysTick.
 * @param[in] function Function to run.
 * @param[in] arg Argument passed to the function. Must stay valid even if this times out.
 * @param[in] timeout_ms How long to wait for the function to run.
 * @retval True if the function ran, false if it timed out.
 */
bool RunOnISRCore(void (*function)(void*), void* arg, uint32_t timeout_ms = 1000);
CPP_AT_CALLBACK(ATIngestModeSCallback);  // AT+INGEST_MODE_S
CPP_AT_CALLBACK(ATIngestUATCallback);    // AT+INGEST_UAT