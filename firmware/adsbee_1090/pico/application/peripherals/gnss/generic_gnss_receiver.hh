#pragma once

#include "gnss_receiver.hh"

/**
 * Generic GNSS receiver that emits standard NMEA-0183 messages, including CASIC/AT6558-based modules.
 * Initialization only power-cycles the module around UART configuration; NMEA is parsed by the
 * inherited Update() loop without probing or transmitting receiver-specific commands.
 */
class GenericGNSSReceiver : public GNSSReceiver {
   public:
    GenericGNSSReceiver(GNSSReceiverConfig config_in) : GNSSReceiver(config_in, BSP::kGNSSModuleGeneric) {}
    bool Init() override;

   protected:
    uint32_t GetDefaultBaudrate() const override {
        return settings_manager.settings.baud_rates[SettingsManager::kGNSSUART];
    }
    bool SendInitCommands() override { return true; }
};
