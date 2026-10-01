#pragma once

#include "aircraft_dictionary.hh"
#include "mavlink.h"
#include "stdint.h"

// Value of HEARTBEAT.mavlink_version. The MAVLink spec fixes it at 3 for both MAVLink 1 and MAVLink 2 framing (the
// generated mavlink_msg_heartbeat_pack/_send functions hard-code it); the framing version is carried by the start byte.
static constexpr uint8_t kMAVLINKHeartbeatMavlinkVersion = 3;

/**
 * Builds the HEARTBEAT this receiver sends at the start of each MAVLink reporting round: an ADS-B receiver component
 * (MAV_TYPE_ADSB, MAV_AUTOPILOT_INVALID) that is active.
 * @retval HEARTBEAT message struct, ready for mavlink_msg_heartbeat_send_struct.
 */
mavlink_heartbeat_t MAVLINKHeartbeatMessage();

uint8_t AircraftCategoryToMAVLINKEmitterType(ADSBTypes::EmitterCategory emitter_category);

mavlink_adsb_vehicle_t ModeSAircraftToMAVLINKADSBVehicleMessage(const ModeSAircraft &aircraft);
mavlink_adsb_vehicle_t UATAircraftToMAVLINKADSBVehicleMessage(const UATAircraft &aircraft);