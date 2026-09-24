#pragma once

// Stub shadowing adsbee_1090/pico/application/peripherals/gnss/gnss_interface.hh for the 1421
// build. The shared common/comms/comms_reporting.cpp includes this header on non-ESP32 targets
// and reads `gnss->fix().utc_time_valid` when building GDL90 heartbeats. The 1421 has no GNSS
// receiver, so `gnss` is always nullptr and the UTC-timing-valid flag reports false.
class GNSSReceiver {
   public:
    struct GNSSFix {
        bool utc_time_valid = false;
    };

    const GNSSFix& fix() const { return fix_; }

   private:
    GNSSFix fix_;
};

inline GNSSReceiver* gnss = nullptr;
