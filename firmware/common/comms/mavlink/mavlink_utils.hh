#pragma once

#include "aircraft_dictionary.hh"
#include "mavlink.h"
#include "stdint.h"

// HEARTBEAT.mavlink_version: always 3 for both MAVLink 1 and 2 (the start byte carries the framing version).
static constexpr uint8_t kMAVLINKHeartbeatMavlinkVersion = 3;

/**
 * HEARTBEAT sent at the start of each MAVLink reporting round: an active ADS-B receiver.
 */
mavlink_heartbeat_t MAVLINKHeartbeatMessage();

uint8_t AircraftCategoryToMAVLINKEmitterType(ADSBTypes::EmitterCategory emitter_category);

mavlink_adsb_vehicle_t ModeSAircraftToMAVLINKADSBVehicleMessage(const ModeSAircraft &aircraft);
mavlink_adsb_vehicle_t UATAircraftToMAVLINKADSBVehicleMessage(const UATAircraft &aircraft);