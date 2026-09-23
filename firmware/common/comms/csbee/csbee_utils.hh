#pragma once
#include "aircraft_dictionary.hh"
#include "macros.hh"
#include "stdio.h"

const uint16_t kCSBeeProtocolVersion = 2;

const uint16_t kCSBeeMessageStrMaxLen = 200;
const uint16_t kCRCMaxNumChars = 4;  // 16 bits = 4 hex characters.
const uint16_t kEOLNumChars = 2;

/**
 * Dumps an Aircraft object into a string buffer in CSBee format. String buffer must be of length
 * kCSBeeMessageStrMaxLen.
 * @param[out] message_buf Character array to write into.
 * @param[in] aircraft Aircraft object to dump the contents of.
 * @retval Number of characters written to the string buffer, or a negative value if something went wrong.
 */
inline int16_t WriteCSBeeModeSAircraftMessageStr(char message_buf[], const ModeSAircraft& aircraft) {
    // #A:ICAO,FLAGS,CALL,SQ,LAT,LON,ALT_BARO,TRACK,VELH,VELV,SIGS,SIGQ,FPS,NICNAC,ALT_GEO,ECAT,CRC\r\n

    // Build up squawk string.
    char squawk_str[5] = "?";  // Squawk is 4 digits, so 4 characters plus null terminator.
    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        snprintf(squawk_str, sizeof(squawk_str), "%04u", aircraft.squawk % 10000);
    }

    // Build up NICNAC bitfield.
    uint16_t nicnac = 0;
    nicnac |= ((aircraft.surveillance_integrity_level & 0b11) << 14);            // SIL bitfield.
    nicnac |= ((aircraft.geometric_vertical_accuracy & 0b11) << 12);             // GVA bitfield
    nicnac |= ((aircraft.navigation_accuracy_category_position & 0b1111) << 8);  // NAC_p bitfield.
    nicnac |= ((aircraft.navigation_accuracy_category_velocity & 0b111) << 5);   // NAC_v bitfield.
    nicnac |= ((aircraft.navigation_integrity_category_baro & 0b1) << 4);        // NIC_baro bitfield.
    nicnac |= ((aircraft.navigation_integrity_category & 0b1111));               // NIC bitfield.

    // Build up ACDIMS bitfield.
    uint32_t acdims = (aircraft.system_design_assurance & 0b11);
    acdims |= (aircraft.width_m & 0b1111111) << 2;
    acdims |= (aircraft.length_m & 0b1111111) << 9;
    acdims |= (static_cast<int8_t>(aircraft.gnss_antenna_offset_right_of_reference_point_m) << 16);
    acdims |= (static_cast<int8_t>(0) << 24);

    int16_t num_chars =  // Print everything except CRC into string buffer.
        snprintf(message_buf, kCSBeeMessageStrMaxLen - kCRCMaxNumChars - 1,
#if defined(ON_PICO) || defined(ON_HOST)
                 "#A:%06X,"  // ICAO, e.g. 3C65AC
                 "%X,"       // FLAGS, e.g. 123F35648
                 "%s,"       // CALL, e.g. N61ZP
                 "%s,"       // SQUAWK, e.g. 7232
                 "%d,"       // ECAT, e.g. 14
                 "%.5f,"     // LAT, e.g. 57.57634
                 "%.5f,"     // LON, e.g. 17.59554
                 "%d,"       // BARO_ALT, e.g. 5000
                 "%d,"       // GNSS_ALT, e.g. 5000
                 "%.0f,"     // DIR, e.g. 35
                 "%d,"       // SPEED, e.g. 464
                 "%d,"       // BARO_VRATE, e.g. -1344
                 "%d,"       // GNSS_VRATE, e.g. -1344
                 "%X,"       // NICNAC, e.g. 31BE89F2
                 "%X,"       // ACDIMS, e.g. 31BE89F2
                 "%d,"       // VERSION
                 "%d,"       // SIGS, e.g. -92
                 "%d,"       // SIGQ, e.g. 2
                 "%d,"       // SFPS, e.g. 3
                 "%d,",      // ESFPS, e.g. 5
#else
                 // ESP32 and TI require 32-bit values to be formatted as "long" types.
                 "#A:%06lX,"  // ICAO, e.g. 3C65AC
                 "%lX,"       // FLAGS, e.g. 123F35648
                 "%s,"        // CALL, e.g. N61ZP
                 "%s,"        // SQUAWK, e.g. 7232
                 "%d,"        // ECAT, e.g. 14
                 "%.5f,"      // LAT, e.g. 57.57634
                 "%.5f,"      // LON, e.g. 17.59554
                 "%ld,"       // BARO_ALT, e.g. 5000
                 "%ld,"       // GNSS_ALT, e.g. 5000
                 "%.0f,"      // DIR, e.g. 35
                 "%ld,"       // SPEED, e.g. 464
                 "%ld,"       // BARO_VRATE, e.g. -1344
                 "%ld,"       // GNSS_VRATE, e.g. -1344
                 "%X,"        // NICNAC, e.g. 31BE89F2
                 "%lX,"       // ACDIMS, e.g. 31BE89F2
                 "%d,"        // VERSION
                 "%d,"        // SIGS, e.g. -92
                 "%d,"        // SIGQ, e.g. 2
                 "%d,"        // SFPS, e.g. 3
                 "%d,",       // ESFPS, e.g. 5
#endif
                 aircraft.icao_address,                           // ICAO
                 aircraft.flags,                                  // FLAGS
                 aircraft.callsign,                               // CALL
                 squawk_str,                                      // SQUAWK
                 aircraft.emitter_category,                       // ECAT
                 aircraft.latitude_deg,                           // LAT
                 aircraft.longitude_deg,                          // LON
                 aircraft.baro_altitude_ft,                       // BARO_ALT
                 aircraft.gnss_altitude_ft,                       // GNSS_ALT
                 aircraft.direction_deg,                          // DIR
                 aircraft.speed_kts,                              // SPEED
                 aircraft.baro_vertical_rate_fpm,                 // BARO_VRATE
                 aircraft.gnss_vertical_rate_fpm,                 // GNSS_VRATE
                 nicnac,                                          // NICNAC
                 acdims,                                          // ACDIMS
                 aircraft.adsb_version,                           // VERSION
                 aircraft.last_message_signal_strength_dbm,       // SIGS
                 aircraft.last_message_signal_quality_db,         // SIGQ
                 aircraft.metrics.valid_squitter_frames,          // SFPS
                 aircraft.metrics.valid_extended_squitter_frames  // ESFPS
        );
    if (num_chars < 0) return num_chars;  // Check if snprintf call got busted.

    // Append a CRC.
    uint16_t crc = CalculateCRC16((uint8_t*)message_buf, num_chars);
    uint16_t crc_num_chars = snprintf(message_buf + num_chars, kCRCMaxNumChars + kEOLNumChars + 1, "%X\r\n",
                                      crc);  // Add CRC to the message buffer.

    return num_chars + crc_num_chars;
}

/**
 * Dumps an Aircraft object into a string buffer in CSBee format. String buffer must be of length
 * kCSBeeMessageStrMaxLen.
 * @param[out] message_buf Character array to write into.
 * @param[in] aircraft Aircraft object to dump the contents of.
 * @retval Number of characters written to the string buffer, or a negative value if something went wrong.
 */
inline int16_t WriteCSBeeUATAircraftMessageStr(char message_buf[], const UATAircraft& aircraft) {
    // #U:ICAO,FLAGS,CALL,SQUAWK,ECAT,LAT,LON,ALT_BARO,ALT_GEO,DIR,VELH,BARO_VELV,GNSS_VELV,SIGS,SIGQ,SFPS,ESFPS,SYSINFO,CRC\r\n

    // Build squawk string.
    char squawk_str[5] = "?";  // Default to "????" if squawk code hasn't been received yet.
    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        snprintf(squawk_str, sizeof(squawk_str), "%04u", aircraft.squawk % 10000);
    }

    // Build up NICNAC bitfield.
    uint16_t nicnac = 0;
    nicnac |= ((aircraft.surveillance_integrity_level & 0b11) << 14);            // SIL bitfield.
    nicnac |= ((aircraft.geometric_vertical_accuracy & 0b11) << 12);             // GVA bitfield
    nicnac |= ((aircraft.navigation_accuracy_category_position & 0b1111) << 8);  // NAC_p bitfield.
    nicnac |= ((aircraft.navigation_accuracy_category_velocity & 0b111) << 5);   // NAC_v bitfield.
    nicnac |= ((aircraft.navigation_integrity_category_baro & 0b1) << 4);        // NIC_baro bitfield.
    nicnac |= ((aircraft.navigation_integrity_category & 0b1111));               // NIC bitfield.

    // Build up ACDIMS bitfield.
    uint32_t acdims = (aircraft.system_design_assurance & 0b11);
    acdims |= (aircraft.width_m & 0b1111111) << 2;
    acdims |= (aircraft.length_m & 0b1111111) << 9;
    acdims |= (static_cast<int8_t>(aircraft.gnss_antenna_offset_right_of_reference_point_m) << 16);
    acdims |= (static_cast<int8_t>(aircraft.gnss_antenna_offset_forward_of_reference_point_m) << 24);

    int16_t num_chars =  // Print everything except CRC into string buffer.
        snprintf(message_buf, kCSBeeMessageStrMaxLen - kCRCMaxNumChars - 1,
#if defined(ON_PICO) || defined(ON_HOST)
                 "#U:%06X,"  // ICAO, e.g. 3C65AC
                 "%X,"       // UAT_FLAGS, e.g. 123F35648
                 "%s,"       // CALL, e.g. N61ZP
                 "%s,"       // SQUAWK, e.g. 7232
                 "%d,"       // ECAT, e.g. 14
                 "%.5f,"     // LAT, e.g. 57.57634
                 "%.5f,"     // LON, e.g. 17.59554
                 "%d,"       // BARO_ALT, e.g. 5000
                 "%d,"       // GNSS_ALT, e.g. 5000
                 "%.0f,"     // DIR, e.g. 35
                 "%d,"       // SPEED, e.g. 464
                 "%d,"       // BARO_VRATE, e.g. -1344
                 "%d,"       // GNSS_VRATE, e.g. -1344
                 "%d,"       // UAT_EMERG
                 "%X,"       // NICNAC, e.g. 31BE89F2
                 "%X,"       // ACDIMS, e.g. 31BE89F2
                 "%d,"       // VERSION
                 "%d,"       // SIGS, e.g. -92
                 "%d,"       // SIGQ, e.g. 2
                 "%d,",      // UATFPS, e.g. 1
#else
                 // ESP32 and TI require 32-bit values to be formatted as "long" types.
                 "#U:%06lX,"  // ICAO, e.g. 3C65AC
                 "%lX,"       // UAT_FLAGS, e.g. 123F35648
                 "%s,"        // CALL, e.g. N61ZP
                 "%s,"        // SQUAWK, e.g. 7232
                 "%d,"        // ECAT, e.g. 14
                 "%.5f,"      // LAT, e.g. 57.57634
                 "%.5f,"      // LON, e.g. 17.59554
                 "%ld,"       // BARO_ALT, e.g. 5000
                 "%ld,"       // GNSS_ALT, e.g. 5000
                 "%.0f,"      // DIR, e.g. 35
                 "%ld,"       // SPEED, e.g. 464
                 "%ld,"       // BARO_VRATE, e.g. -1344
                 "%ld,"       // GNSS_VRATE, e.g. -1344
                 "%d,"        // UAT_EMERG
                 "%X,"        // NICNAC, e.g. 31BE89F2
                 "%lX,"       // ACDIMS, e.g. 31BE89F2
                 "%d,"        // VERSION
                 "%d,"        // SIGS, e.g. -92
                 "%d,"        // SIGQ, e.g. 2
                 "%d,",       // UATFPS, e.g. 1
#endif
                 aircraft.icao_address,                      // ICAO and address qualfiier
                 aircraft.flags,                             // FLAGS
                 aircraft.callsign,                          // CALL
                 squawk_str,                                 // SQUAWK
                 aircraft.emitter_category,                  // ECAT
                 aircraft.latitude_deg,                      // LAT
                 aircraft.longitude_deg,                     // LON
                 aircraft.baro_altitude_ft,                  // BARO_ALT
                 aircraft.gnss_altitude_ft,                  // GNSS_ALT
                 aircraft.direction_deg,                     // DIR
                 aircraft.speed_kts,                         // SPEED
                 aircraft.baro_vertical_rate_fpm,            // BARO_VRATE
                 aircraft.gnss_vertical_rate_fpm,            // GNSS_VRATE
                 aircraft.emergency_priority_status,         // UAT_EMERG
                 nicnac,                                     // NICNAC
                 acdims,                                     // ACDIMS
                 aircraft.uat_version,                       // VERSION
                 aircraft.last_message_signal_strength_dbm,  // SIGS
                 aircraft.last_message_signal_quality_bits,  // SIGQ
                 aircraft.metrics.valid_frames               // UATFPS
        );
    if (num_chars < 0) return num_chars;  // Check if snprintf call got busted.

    // Append a CRC.
    uint16_t crc = CalculateCRC16((uint8_t*)message_buf, num_chars);
    uint16_t crc_num_chars = snprintf(message_buf + num_chars, kCRCMaxNumChars + kEOLNumChars + 1, "%X\r\n",
                                      crc);  // Add CRC to the message buffer.

    return num_chars + crc_num_chars;
}

inline int16_t WriteCSBeeStatisticsMessageStr(char message_buf[], uint16_t sdps, uint16_t raw_sfps, uint16_t sfps,
                                              uint16_t raw_esfps, uint16_t esfps, uint16_t raw_uatfps, uint16_t uatfps,
                                              uint16_t num_aircraft, uint32_t tscal, uint32_t uptime) {
    int16_t num_chars =  // Print everything except for CRC into string buffer.
        snprintf(message_buf, kCSBeeMessageStrMaxLen - kCRCMaxNumChars - 1,
#if defined(ON_PICO) || defined(ON_HOST)
                 "#S:%d,"  // CSBee Protocol Version
                 "%d,"     // SDPS, e.g. 106
                 "%d,"     // RAW_SFPS e.g. 30
                 "%d,"     // SFPS, e.g. 20
                 "%d,"     // RAW_ESFPS e.g. 15
                 "%d,"     // ESFPS, e.g. 13
                 "%d,"     // RAW_UATFPS, e.g. 5
                 "%d,"     // UATFPS, e.g. 4
                 "%d,"     // NUM_AIRCRAFT, e.g. 13
                 "%d,"     // TSCAL, e.g. 13999415
                 "%d,",    // UPTIME, e.g. 134
#else
                 // ESP32 and TI require 32-bit values to be formatted as "long" types.
                 "#S:%d,"  // CSBee Protocol Version
                 "%d,"     // SDPS, e.g. 106
                 "%d,"     // RAW_SFPS e.g. 30
                 "%d,"     // SFPS, e.g. 20
                 "%d,"     // RAW_ESFPS e.g. 15
                 "%d,"     // ESFPS, e.g. 13
                 "%d,"     // RAW_UATFPS, e.g. 5
                 "%d,"     // UATFPS, e.g. 4
                 "%d,"     // NUM_AIRCRAFT, e.g. 13
                 "%ld,"    // TSCAL, e.g. 13999415
                 "%ld,",   // UPTIME, e.g. 134
#endif
                 kCSBeeProtocolVersion, sdps, raw_sfps, sfps, raw_esfps, esfps, raw_uatfps, uatfps, num_aircraft, tscal,
                 uptime);
    if (num_chars < 0) return num_chars;  // Check if snprintf call got busted.

    // Append a CRC.
    uint16_t crc = CalculateCRC16((uint8_t*)message_buf, num_chars);
    uint16_t crc_num_chars = snprintf(message_buf + num_chars, kCRCMaxNumChars + kEOLNumChars + 1, "%X\r\n",
                                      crc);  // Add CRC to the message buffer.

    return num_chars + crc_num_chars;
}