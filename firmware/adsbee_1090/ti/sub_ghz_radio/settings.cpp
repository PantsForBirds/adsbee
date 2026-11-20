#include "settings.hh"

#include "sub_ghz_radio.hh"

bool SettingsManager::Apply() {
    subg_radio.rx_enabled = settings.subg_rx_enabled;
    subg_radio.SetBiasTeeEnable(settings.subg_bias_tee_enabled);
    subg_radio.SetMode(settings.subg_mode);
    return true;
}