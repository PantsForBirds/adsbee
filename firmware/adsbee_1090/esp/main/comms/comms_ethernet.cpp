#include "comms.hh"
#include "device_info.hh"  // For getting MAC address.
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_eth_driver.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal.hh"
#include "sdkconfig.h"
#include "task_utils.hh"  // For delayed reconnect callbacks.

/** "Pass-Through" functions used to access member functions in callbacks. **/
void ethernet_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.EthernetEventHandler(arg, event_base, event_id, event_data);
}
inline void connect_to_ethernet(void* arg = nullptr) { comms_manager.ConnectToEthernet(); }
/** End "Pass-Through" functions. **/

bool CommsManager::ConnectToEthernet() {
    if (esp_eth_start(ethernet_handle_) != ESP_OK) {
        CONSOLE_ERROR("connect_to_ethernet", "Failed to connect to Ethernet.");
        return false;
    }
    return true;
}

bool CommsManager::EthernetDeInit() {
    if (!ethernet_was_initialized_) return true;  // Don't try de-initializing if it was never initialized.

    // The de-init functions are not yet supported by ESP IDF, so the best bet is to just restart.
    esp_restart();  // Software reset.
    return false;   // abort didn't work
}

void CommsManager::EthernetEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t*)event_data;

    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED: {
            uint8_t mac_addr[6] = {0};
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            CONSOLE_INFO("CommsManager::EthernetEventHandler", "Ethernet Link Up");
            CONSOLE_INFO("CommsManager::EthernetEventHandler", "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            ethernet_connected_ = true;
            ethernet_link_up_timestamp_ms_ = get_time_since_boot_ms();
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED: {
            CONSOLE_INFO("CommsManager::EthernetEventHandler", "Ethernet Link Down");
            ethernet_connected_ = false;
            ethernet_has_ip_ = false;
            ethernet_has_ip6_ = false;
            // ESP_ERROR_CHECK(esp_netif_dhcpc_stop(ethernet_netif_));
            esp_err_t stop_err = esp_eth_stop(eth_handle);
            if (stop_err != ESP_OK) {
                CONSOLE_ERROR("CommsManager::EthernetEventHandler", "Failed to stop Ethernet: %s",
                              esp_err_to_name(stop_err));
            }
            if (ethernet_enabled) {
                ScheduleDelayedFunctionCall(kEthernetReconnectIntervalMs, &connect_to_ethernet);
            }
            break;
        }
        case ETHERNET_EVENT_START: {
            CONSOLE_INFO("CommsManager::EthernetEventHandler", "Ethernet Started");
            break;
        }
        case ETHERNET_EVENT_STOP: {
            CONSOLE_INFO("CommsManager::EthernetEventHandler", "Ethernet Stopped");
            break;
        }
        default: {
            break;
        }
    }
}

bool CommsManager::EthernetInit() {
    //  Create instance(s) of esp-netif for SPI Ethernet(s)
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    ApplyIPModeToNetifConfig(base);
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    cfg.base = &base;
    ethernet_netif_ = esp_netif_new(&cfg);

    ESP_ERROR_CHECK(esp_netif_set_hostname(ethernet_netif_, hostname));

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &ethernet_event_handler, NULL));
    if (!ip_event_handler_was_initialized_) {
        IPInit();
    }

    // SPI bus configuration.
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = config_.aux_spi_mosi_pin;
    buscfg.miso_io_num = config_.aux_spi_miso_pin;
    buscfg.sclk_io_num = config_.aux_spi_clk_pin;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    // W5500 SPI device configuration.
    // command_bits = 16 (address phase in W5500 SPI frame), address_bits = 8 (control phase) are left at 0.
    spi_device_interface_config_t spi_devcfg = {};
    spi_devcfg.mode = 0;
    spi_devcfg.clock_speed_hz = config_.aux_spi_clk_rate_hz;
    spi_devcfg.spics_io_num = config_.aux_spi_cs_pin;
    spi_devcfg.queue_size = 20;

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(config_.aux_spi_handle, &spi_devcfg);
    w5500_config.int_gpio_num = config_.aux_io_c_pin;
    // TODO: no reset pin?

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = -1;  // PHY reset is controlled by MAC.

    ethernet_was_initialized_ = true;

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret == ESP_ERR_INVALID_STATE) {
        // ISR handler has been already installed so no issues
        CONSOLE_INFO("CommsManager::EthernetInit", "GPIO ISR handler has been already installed");
    } else if (ret != ESP_OK) {
        CONSOLE_ERROR("CommsManager::EthernetInit", "GPIO ISR handler install failed");
        return false;
    }

    ESP_ERROR_CHECK(spi_bus_initialize(config_.aux_spi_handle, &buscfg, SPI_DMA_CH_AUTO));
    esp_eth_mac_t* mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t* phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &ethernet_handle_));

    // Set MAC address.
    ObjectDictionary::ESP32DeviceInfo device_info = GetESP32DeviceInfo();
    ESP_ERROR_CHECK(esp_eth_ioctl(ethernet_handle_, ETH_CMD_S_MAC_ADDR, device_info.ethernet_mac));

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(ethernet_netif_, esp_eth_new_netif_glue(ethernet_handle_)));

    // After the netif glue's handlers, so the netif is up when the link-local address is created.
    ESP_ERROR_CHECK(
        esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_CONNECTED, &ip6_link_up_event_handler, NULL));

    // Start Ethernet driver
    ESP_ERROR_CHECK(esp_eth_start(ethernet_handle_));

    return true;
}