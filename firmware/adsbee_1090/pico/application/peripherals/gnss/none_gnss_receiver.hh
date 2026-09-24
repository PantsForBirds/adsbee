#pragma once

#include "gnss_receiver.hh"

/**
 * Placeholder GNSS receiver for boards with no GNSS module populated.
 *
 * GNSSReceiver::Init() recognizes its NONE type and short-circuits without touching any GNSS hardware
 * (no UART claim, no enable-pin toggle, no baud change) and reports the module as absent. The
 * application falls back to its non-GNSS position source, and Update() is the inherited no-op (nothing
 * streams in), so it is always safe to call unconditionally from the main loop.
 */
class NoneGNSSReceiver : public GNSSReceiver {
   public:
    NoneGNSSReceiver(GNSSReceiverConfig config_in) : GNSSReceiver(config_in, BSP::kGNSSModuleNone) {}

   protected:
    // Never called (Init() short-circuits before using these), but required to make the class concrete.
    uint32_t GetDefaultBaudrate() const override { return 9600; }
    bool SendInitCommands() override { return false; }
};
