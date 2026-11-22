#pragma once

// Stub shadowing adsbee_1090/pico/application/peripherals/gnss/gnss_interface.hh for the 1421
// build. The shared common/comms/comms_reporting.cpp includes this header on non-ESP32 targets
// and reads `gnss->fix()` (UTC time valid, geoid separation) and GetAltitudeStdDevCm() when building
// GDL90 heartbeats and ownship messages. The 1421 has no GNSS receiver, so `gnss` is always nullptr
// and nothing from it is reported.
class GNSSReceiver {
   public:
    struct GNSSFix {
        bool utc_time_valid = false;
        bool geoid_separation_valid = false;
        int32_t geoid_separation_cm = 0;
    };

    const GNSSFix& fix() const { return fix_; }
    bool GetAltitudeStdDevCm(int32_t& /*std_dev_cm*/) const { return false; }

   private:
    GNSSFix fix_;
};

inline GNSSReceiver* gnss = nullptr;
