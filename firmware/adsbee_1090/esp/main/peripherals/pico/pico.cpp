#include "pico.hh"

// Runs when a transaction is ready for the master; raises the handshake line if it solicits a transfer.
void IRAM_ATTR esp_spi_post_setup_cb(spi_slave_transaction_t *trans) {
    pico_ll.SetSPIHandshakePinLevel(reinterpret_cast<uintptr_t>(trans->user) == Pico::kSolicitTransfer);
}

// Called after transaction is sent/received. We use this to set the handshake line low.
void IRAM_ATTR esp_spi_post_trans_cb(spi_slave_transaction_t *trans) { pico_ll.SetSPIHandshakePinLevel(0); }

Pico::Pico(PicoConfig config_in) : config_(config_in) {
    spi_rx_buf_ =
        static_cast<uint8_t *>(heap_caps_malloc(SPICoprocessorPacket::kSPITransactionMaxLenBytes, MALLOC_CAP_DMA));
    spi_tx_buf_ =
        static_cast<uint8_t *>(heap_caps_malloc(SPICoprocessorPacket::kSPITransactionMaxLenBytes, MALLOC_CAP_DMA));

    if (!spi_rx_buf_ || !spi_tx_buf_) {
        CONSOLE_ERROR("Pico::Pico", "Failed to allocate SPI tx/rx buffers.");
    }
    memset(spi_rx_buf_, 0x0, SPICoprocessorPacket::kSPITransactionMaxLenBytes);
    memset(spi_tx_buf_, 0x0, SPICoprocessorPacket::kSPITransactionMaxLenBytes);
}

Pico::~Pico() {
    heap_caps_free(spi_rx_buf_);
    heap_caps_free(spi_tx_buf_);
    vSemaphoreDelete(spi_mutex_);
    vSemaphoreDelete(spi_next_transaction_mutex_);
}

bool Pico::Init() {
    gpio_set_direction(config_.network_led_pin, GPIO_MODE_OUTPUT);
    spi_bus_config_t spi_buscfg = {.mosi_io_num = config_.spi_mosi_pin,
                                   .miso_io_num = config_.spi_miso_pin,
                                   .sclk_io_num = config_.spi_clk_pin,
                                   .data2_io_num = -1,  // union with quadwp_io_num
                                   .data3_io_num = -1,  // union with quadhd_io_num
                                   .data4_io_num = -1,
                                   .data5_io_num = -1,
                                   .data6_io_num = -1,
                                   .data7_io_num = -1,
                                   .data_io_default_level = false,  // keep lines LO when not in use
                                   .max_transfer_sz = SPICoprocessorPacket::kSPITransactionMaxLenBytes,
                                   .flags = 0,
                                   .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                                   .intr_flags = 0};
    spi_slave_interface_config_t spi_slvcfg = {.spics_io_num = config_.spi_cs_pin,
                                               .flags = 0,
                                               .queue_size = 3,
                                               .mode = 0,
                                               .post_setup_cb = esp_spi_post_setup_cb,
                                               .post_trans_cb = esp_spi_post_trans_cb};
    gpio_config_t handshake_io_conf = {
        .pin_bit_mask = (static_cast<uint64_t>(0b1) << config_.spi_handshake_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&handshake_io_conf);
    gpio_set_pull_mode(config_.spi_mosi_pin, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(config_.spi_miso_pin, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(config_.spi_clk_pin, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(config_.spi_cs_pin, GPIO_PULLUP_ONLY);

    // Adjust drive strength on MISO pin.
    gpio_set_drive_capability(config_.spi_miso_pin, GPIO_DRIVE_CAP_MAX);

    esp_err_t status = spi_slave_initialize(config_.spi_handle, &spi_buscfg, &spi_slvcfg, SPI_DMA_CH_AUTO);
    if (status != ESP_OK) {
        ESP_LOGE("SPICoprocessor::SPIInit", "SPI initialization failed with code 0x%x.", status);
        return false;
    }

    spi_mutex_ = xSemaphoreCreateMutex();
    spi_next_transaction_mutex_ = xSemaphoreCreateMutex();

    return true;
}

bool Pico::DeInit() {
    spi_receive_task_should_exit_ = true;
    return true;
}

int Pico::SPIWriteReadBlocking(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len_bytes, bool end_transaction) {
    esp_err_t status = ESP_OK;
    spi_slave_transaction_t *done_trans = nullptr;

    // Wait for transactions to complete. Allow this task to block if no SPI transaction is received until max
    // delay. Currently, setting the delay here to anything other than portMAX_DELAY (which allows blocking
    // indefinitely) causes an error in spi_slave.c due to extra transactions getting stuck in the SPI peripheral queue.
    if (tx_buf == nullptr && rx_prequeued_ && len_bytes == SPICoprocessorPacket::kSPITransactionMaxLenBytes) {
        // Full-length receive that was already queued behind the last response: just collect it.
        rx_prequeued_ = false;
        status = spi_slave_get_trans_result(config_.spi_handle, &done_trans, portMAX_DELAY);
    } else {
        if (rx_prequeued_) {
            // Only Update()'s full-length receive should follow a response; let the queued receive finish first.
            CONSOLE_ERROR("Pico::SPIWriteReadBlocking",
                          "Unexpected %d Byte %s while a receive is queued; waiting for the queued receive first.",
                          len_bytes, tx_buf ? "write" : "read");
            rx_prequeued_ = false;
            status = spi_slave_get_trans_result(config_.spi_handle, &done_trans, portMAX_DELAY);
            if (status != ESP_OK) {
                CONSOLE_ERROR("Pico::SPIWriteReadBlocking", "Queued receive failed with code 0x%x.", status);
                return kErrorGeneric;
            }
        }
        spi_slave_transaction_t &t = tx_buf != nullptr ? spi_tx_trans_ : spi_rx_trans_;
        memset(&t, 0, sizeof(t));
        t.length = len_bytes * kBitsPerByte;  // Transaction length is in bits
        t.tx_buffer = tx_buf == nullptr ? nullptr : spi_tx_buf_;
        t.rx_buffer = rx_buf == nullptr ? nullptr : spi_rx_buf_;
        t.user = reinterpret_cast<void *>(use_handshake_pin_ ? kSolicitTransfer : 0);
        if (tx_buf != nullptr) {
            memcpy(spi_tx_buf_, tx_buf, len_bytes);
        }
        if (tx_buf != nullptr && rx_buf == nullptr) {
            // Response: queue the next receive right behind it (see rx_prequeued_). The master can read the response
            // within microseconds, so suspend the scheduler to queue both together. The queue is empty, so no waiting.
            memset(&spi_rx_trans_, 0, sizeof(spi_rx_trans_));
            spi_rx_trans_.length = SPICoprocessorPacket::kSPITransactionMaxLenBytes * kBitsPerByte;
            spi_rx_trans_.rx_buffer = spi_rx_buf_;
            spi_rx_trans_.user = nullptr;  // Wait for the master; don't solicit a transfer.
            vTaskSuspendAll();
            status = spi_slave_queue_trans(config_.spi_handle, &t, 0);
            rx_prequeued_ = status == ESP_OK && spi_slave_queue_trans(config_.spi_handle, &spi_rx_trans_, 0) == ESP_OK;
            xTaskResumeAll();
            if (status == ESP_ERR_TIMEOUT) {
                status = spi_slave_queue_trans(config_.spi_handle, &t, portMAX_DELAY);  // Queue unexpectedly full.
            }
        } else {
            status = spi_slave_queue_trans(config_.spi_handle, &t, portMAX_DELAY);
        }
        if (status == ESP_OK) {
            status = spi_slave_get_trans_result(config_.spi_handle, &done_trans, portMAX_DELAY);
        }
    }

    if (status != ESP_OK) {
        if (status == ESP_ERR_TIMEOUT) {
            return ReturnCode::kErrorTimeout;  // Timeouts fail silently.
        }
        CONSOLE_ERROR("SPICoprocesor::SPIWriteReadBlocking", "SPI transaction failed unexpectedly with code 0x%x.",
                      status);
        return kErrorGeneric;
    }
    int bytes_written = CeilBitsToBytes(done_trans->trans_len);
    if (rx_buf != nullptr) {
        memcpy(rx_buf, spi_rx_buf_, len_bytes);
    }
    last_bytes_transacted_ = bytes_written;
    return bytes_written;
}