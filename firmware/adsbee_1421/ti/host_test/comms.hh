#ifndef COMMS_HH_
#define COMMS_HH_

// Console logging for host tests, as firmware/adsbee_1090/pico/host_test/comms.hh: common/utils/crc/crc.cpp logs
// through these.
#include <cstdio>

#define CONSOLE_INFO(tag, format, ...)    printf("INFO: " tag ": " format "\r\n" __VA_OPT__(, ) __VA_ARGS__)
#define CONSOLE_WARNING(tag, format, ...) printf("WARNING: " tag ": " format "\r\n" __VA_OPT__(, ) __VA_ARGS__)
#define CONSOLE_ERROR(tag, format, ...)   printf("ERROR: " tag ": " format "\r\n" __VA_OPT__(, ) __VA_ARGS__)
#define CONSOLE_PRINTF(format, ...)       printf(format "\r\n" __VA_OPT__(, ) __VA_ARGS__)

#endif /* COMMS_HH_ */
