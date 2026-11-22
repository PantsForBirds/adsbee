#pragma once

// Stub shadowing adsbee_1090/pico/application/peripherals/gnss/gnss_interface.hh for the 1421
// build. The shared common/comms/comms_reporting.cpp includes this header on non-ESP32 targets
// and reads `gnss->fix()` (UTC time valid, geoid separation) and GetAltitudeStdDevM() when building
// GDL90 heartbeats and ownship messages. The 1421 has no GNSS receiver, so `gnss` is always nullptr
// and nothing from it is reported.
class GNSSReceiver {
   public:
    struct GNSSFix {
        bool utc_time_valid = false;
        bool geoid_separation_valid = false;
        float geoid_separation_m = 0.0f;
    };

    const GNSSFix& fix() const { return fix_; }
    bool GetAltitudeStdDevM(float& /*std_dev_m*/) const { return false; }

   private:
    GNSSFix fix_;
};

inline GNSSReceiver* gnss = nullptr;
