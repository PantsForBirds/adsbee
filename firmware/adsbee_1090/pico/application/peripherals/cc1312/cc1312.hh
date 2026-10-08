#pragma once

#include "bsp.hh"
#include "comms.hh"
#include "hardware/spi.h"
#include "led_flasher.hh"
#include "spi_coprocessor.hh"

class CC1312 : public SPICoprocessorSlaveInterface {
   public:
    static const uint16_t kMaxNumPropertiesAtOnce = 12;
    static const uint32_t kCTSCheckIntervalUs = 40;
    static const uint32_t kBootupDelayMs = 100;
    static const uint32_t kShutdownDelayMs = 20;
    static const uint32_t kSPITransactionTimeoutMs =
        500;  // Set to be quite long to allow CRC calculation time for full app image after flashing.
    // How long we wait to start a transaction after the last one is completed. Can be overridden if the handshake line
    // goes high after kSPIHandshakeLockoutUs. Does not apply in bootloader mode, since we wait for acks.
    static constexpr uint32_t kSPIPostTransmitLockoutUs = 1000;
    static constexpr uint32_t kSPIHandshakeLockoutUs =
        200;  // How long to wait after a transaction to allow the handshake pin to override the post transmit lockout.
              // Used to override the stock value since the CC1312 is slow.

    static const uint32_t kBootupMaxCommsWaitIntervalMs =
        5000;  // How long to wait for the CC1312 to boot up and be ready for comms before giving up.
    static const uint32_t kBootupProbeIntervalMs = 20;  // Pause between application readiness probes after boot.

    struct BootloaderCCFGConfig {
        // Note: Struct only includes the CCFG fields that we are interested in.
        bool bank_erase_disabled = false;
        bool chip_erase_disabled = false;
        bool bl_backdoor_enabled = true;
        uint8_t bl_backdoor_pin = bsp.subg_bootloader_backdoor_pin;
        bool bl_backdoor_level = true;  // Active high.
        bool bl_enabled = true;         // Bootloader enabled.

        // Equality comparison operator for BootloaderCCFGConfig
        bool operator==(const BootloaderCCFGConfig& other) const {
            return bank_erase_disabled == other.bank_erase_disabled &&
                   chip_erase_disabled == other.chip_erase_disabled &&
                   bl_backdoor_enabled == other.bl_backdoor_enabled && bl_backdoor_pin == other.bl_backdoor_pin &&
                   bl_backdoor_level == other.bl_backdoor_level && bl_enabled == other.bl_enabled;
        }

        // Not-equal operator based on equality operator
        bool operator!=(const BootloaderCCFGConfig& other) const { return !(*this == other); }

        // Print values to buffer for debuging.
        void print_to_buffer(char* buffer, size_t buffer_size) const {
            snprintf(buffer, buffer_size,
                     "\tBank Erase Disabled: %s\t\n\tChip Erase Disabled: %s\t\n\t"
                     "Bootloader Backdoor Enabled: %s\r\n\tBackdoor Pin: %d\r\n\tBackdoor Level: %s\r\n\t"
                     "Bootloader Enabled: %s\r\n",
                     bank_erase_disabled ? "true" : "false", chip_erase_disabled ? "true" : "false",
                     bl_backdoor_enabled ? "true" : "false", bl_backdoor_pin, bl_backdoor_level ? "high" : "low",
                     bl_enabled ? "true" : "false");
        }
    };

    struct CC1312Config {
        // This clock frequency is used to talk to the CC1312.
        uint32_t spi_clk_freq_hz = bsp.subg_spi_clk_freq_hz;
        spi_inst_t* spi_handle = bsp.copro_spi_handle;
        uint16_t spi_clk_pin = bsp.copro_spi_clk_pin;  // Pin for SPI clock (SCK).
        uint16_t spi_mosi_pin = bsp.copro_spi_mosi_pin;
        uint16_t spi_miso_pin = bsp.copro_spi_miso_pin;
        uint16_t spi_cs_pin = bsp.subg_cs_pin;
        gpio_drive_strength spi_drive_strength = bsp.copro_spi_drive_strength;
        bool spi_pullup = bsp.copro_spi_pullup;
        bool spi_pulldown = bsp.copro_spi_pulldown;
        uint16_t enable_pin = bsp.subg_enable_pin;  // Pin to enable the CC1312.
        uint16_t irq_pin = bsp.subg_irq_pin;        // Pin for CC1312 IRQ.
        uint16_t sync_pin = bsp.sync_pin;           // Pin for sync and CC1312 bootloader backdoor.

        BootloaderCCFGConfig ccfg_config = {};  // CCFG configuration for the bootloader (use default values).
    };

    // The bootloader has a weird SPI configuration, so we need a struct to keep track of it.
    struct SPIPeripheralConfig {
        uint16_t bits_per_transfer = 8;
        spi_cpol_t cpol = SPI_CPOL_0;
        spi_cpha_t cpha = SPI_CPHA_0;
        spi_order_t order = SPI_MSB_FIRST;
    };

    enum ProtocolByte : uint8_t {
        kProtocolByteAck = 0xCC,
        kProtocolByteNack = 0x33,
    };

    enum Command : uint8_t {
        kCmdPing = 0x20,
        kCmdDownload = 0x21,
        kCmdGetStatus = 0x23,
        kCmdSendData = 0x24,
        kCmdReset = 0x25,
        kCmdSectorErase = 0x26,
        kCmdCRC32 = 0x27,
        kCmdGetChipID = 0x26,
        kCmdMemoryRead = 0x2A,
        kCmdMemoryWrite = 0x2B,
        kCmdBankErase = 0x2C,
        kCmdSetCCFG = 0x2D,
        kCmdDownloadCRC = 0x2F
    };

    enum CommandReturnStatus : uint8_t {
        kCmdRetDriverError = 0x00,  // Made up (non-TI status) for indicating a generic error in our code.
        kCmdRetSuccess = 0x40,      // Status for successful command.
        kCmdRetUnknownCmd = 0x41,   // Status for unknown command.
        kCmdRetInvalidCmd = 0x42,   // Status for invlaid command (in other words, incorrect packet size).
        kCmdRetInvalidAdr = 0x43,   // Status for invalid input address.
        kCmdRetFlashFail = 0x44     // Status for failing flash erase or program operation.
    };

    const char* CommandReturnStatusToString(CommandReturnStatus status) {
        switch (status) {
            case kCmdRetDriverError:
                return "Driver error";
            case kCmdRetSuccess:
                return "Success";
            case kCmdRetUnknownCmd:
                return "Unknown command";
            case kCmdRetInvalidCmd:
                return "Invalid command";
            case kCmdRetInvalidAdr:
                return "Invalid address";
            case kCmdRetFlashFail:
                return "Flash fail";
            default:
                return "Unknown status";
        }
    }

    // CCFG field IDs used to set CCFG values via memory mapped CCFG registers.
    enum BootloaderCCFGFieldID : uint32_t {
        kCCFGFieldIDSectorProt = 0,  // Flash sector number of sector to protect from program and erase.
        kCCFGFieldIDImageValid = 1,  // Address of flash image vector table to allow jump to application.
        kCCFGFieldIDTestTAPOLck = 2,
        kCCFGFieldIDPwrprofTAPLck = 3,
        kCCFGFieldIDCPUDAPLck = 4,
        kCCFGFieldIDAONTAPLck = 5,
        kCCFGFieldIDPBIST1TAPLck = 6,
        kCCFGFieldIDPBIST2TAPLck = 7,
        kCCFGFieldIDIDBankEraseDis = 8,  // Set to 0 to disable bank erase from bootloader using COMMAND_BANK_ERASE.
        kCCFGFieldIDChipEraseDis = 9,    // Set to 0 to disable any chip erase operation.
        kCCFGFieldIDTIFAEnable = 10,     // Set to non-0x5 value to disable TI FA in following boot.
        kCCFGFieldIDBLBackdoorEn = 11,   // Set to non-0x5 to disable bootloader backdoor in following boot.
        kCCFGFieldIDBLBackdoorPin = 12,  // Set to DIO pin number to use for bootloader backdoor. Must be in Bit[7:0].
        kCCFGFieldIDBackdoorLevel = 13,  // Set to the active level of the bootloader backdoor pin. Must be in Bit[0].
        kCCFGFieldIDBLEnable = 14        // Set to non-0x5 to disable the bootloader.
    };

    // Register base addresses from datasheet table 3-1.
    static const uint32_t kBaseAddrFlashMem = 0x0000000;  // Program Flash Memory
    static const uint32_t kBaseAddrSRAM = 0x20000000;     // SRAM
    static const uint32_t kBaseAddrFCFG1 = 0x50001000;    // Factory Configuration
    static const uint32_t kBaseAddrCCFG = 0x50003000;     // Customer COnfiguration

    // Register offsets used to read FCFG values from memory.
    static const uint16_t kFCFG1RegOffUserID = 0x0294;

    // Register offsets used to read CCFG values from memory.
    static const uint32_t kCCFGRegOffExtLFClk = 0x1FA8;         // External LF clock configuration
    static const uint32_t kCCFGRegOffModeConf1 = 0x1FAC;        // Mode Configuration 1
    static const uint32_t kCCFGRegOffSizeAndDisFlags = 0x1FB0;  // CCFG Size and Disable Flags
    static const uint32_t kCCFGRegOffModeConf = 0x1FB4;         // Mode Configuration 0
    static const uint32_t kCCFGRegOffVoltLoad0 = 0x1FB8;        // Voltage Load 0
    static const uint32_t kCCFGRegOffVoltLoad1 = 0x1FBC;        // Voltage Load 1
    static const uint32_t kCCFGRegOffRTCOffset = 0x1FC0;        // Real Time Clock Offset
    static const uint32_t kCCFGRegOffFreqOffset = 0x1FC4;       // Frequency Offset
    static const uint32_t kCCFGRegOffIEEEMAC0 = 0x1FC8;         // IEEE MAC Address 0
    static const uint32_t kCCFGRegOffIEEEMAC1 = 0x1FCC;         // IEEE MAC Address 1
    static const uint32_t kCCFGRegOffIEEEBLE0 = 0x1FD0;         // IEEE BLE Address 0
    static const uint32_t kCCFGRegOffIEEEBLE1 = 0x1FD4;         // IEEE BLE Address 1
    static const uint32_t kCCFGRegOffBLConfig = 0x1FD8;         // Bootloader Configuration
    static const uint32_t kCCFGRegOffEraseConf = 0x1FDC;        // Erase Configuration
    static const uint32_t kCCFGRegOffTIOptions = 0x1FE0;        // TI Options
    static const uint32_t kCCFGRegOffTapDap0 = 0x1FE4;          // Test Access Points Enable 0
    static const uint32_t kCCFGRegOffTapDap1 = 0x1FE8;          // Test Access Points Enable 1
    static const uint32_t kCCFGRegOffImageValidConf = 0x1FEC;   // Image Valid
    static const uint32_t kCCFGRegOffProt31_0 = 0x1FF0;         // Protect Sectors 0-31
    static const uint32_t kCCFGRegOffProt63_32 = 0x1FF4;        // Protect Sectors 32-63
    static const uint32_t kCCFGRegOffProt95_64 = 0x1FF8;        // Protect Sectors 64-95
    static const uint32_t kCCFGRegOffProt127_96 = 0x1FFC;       // Protect Sectors 96-127

    // Max packet write lengths.
    static const uint32_t kBootloaderCommandSendDataMaxLenBytes =
        255 - 3;  // 255 bytes total, minus 3 for the command byte, length byte, and checksum byte.
    static const uint16_t kBootloaderCommandMemoryWriteUint8MaxNumBytes = 247;
    static const uint16_t kBootloaderCommandMemoryWriteUint32MaxNumBytes = 244;

    /**
     * Initialization function. Currently unused.
     * @retval True if the initialization was successful, false otherwise.
     */
    inline bool Init() {
        return Init(true);  // Assume SPI bus already initialize.
    }

    /**
     * De-initialization function. Currently unused.
     * @retval True if the de-initialization was successful, false otherwise.
     */
    inline bool DeInit() { return true; };

    bool Update();

    /**
     * Adjusts the SPI peripheral to be able to talk to the CC1312. Nominally adjusts clock rate, but also adjusts CPHA
     * and CPOL if the bootloader is active.
     */
    bool SPIBeginTransaction();
    /**
     * Restores the SPI peripheral to its previous state. Nominally restores clock rate, but also restores CPHA and CPOL
     * if the bootloader is active.
     */
    void SPIEndTransaction();
    inline bool SPIClaimNextTransaction() {
        return true;  // Not multi-threaded, no need for this.
    }
    inline void SPIReleaseNextTransaction() {
        // Not multi-threaded, no need for this.
    }
    inline bool SPIGetHandshakePinLevel() {
        return !gpio_get(config_.irq_pin);  // Return the level of the (active LOW).
    }
    int SPIWriteReadBlocking(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len_bytes = 0, bool end_transaction = true);
    inline int SPIWriteBlocking(uint8_t* tx_buf, uint16_t len_bytes = 0, bool end_transaction = true) {
        return SPIWriteReadBlocking(tx_buf, nullptr, len_bytes, end_transaction);
    }
    inline int SPIReadBlocking(uint8_t* rx_buf, uint16_t len_bytes = 0, bool end_transaction = true) {
        return SPIWriteReadBlocking(nullptr, rx_buf, len_bytes, end_transaction);
    }

    /**
     * Constructor for CC1312.
     * @param[in] config_in Configuration struct for the CC1312.
     */
    CC1312(CC1312Config config_in) : config_(config_in) {
        // Override the default SPI handshake lockout since the CC1312 is a slow boi.
        spi_handshake_lockout_us_ = kSPIHandshakeLockoutUs;
    };

    /**
     * Destructor for CC1312.
     */
    ~CC1312() {};

    /**
     * Initialize CC1312.
     * @param[in] spi_already_initialized Set to true if using a shared SPI bus such that the SPI bus is already
     * initialized.
     */
    bool Init(bool spi_already_initialized = false);

    /**
     * Checks to see if the IEEE 802.3 CRC32 of the application on the CC1312 matches the CRC32 of the CC1312
     * application binary stored within the RP2040 firmware. MUST be called from within bootloader mode!
     * @retval True if the application is up to date, false otherwise.
     */
    bool ApplicationIsUpToDate();

    /**
     * Sends one device status read request and checks for a valid reply, without retries or console output. Used to
     * detect when the CC1312 application is ready for comms after a reset.
     * @retval True if the CC1312 application answered, false otherwise.
     */
    bool ProbeApplication();

    /**
     * Erases all user-accessible flash memory on the CC1312 using the bootloader COMMAND_BANK_ERASE command, including
     * the CCFG registers. CCFG values will need to be re-written after running this command.BootloaderCCFGConfig
     *
     * NOTE: This command will only work if bank_erase_disabled is set to false in the CCFG configuration.
     * @retval True if the command was successful, false otherwise.
     */
    bool BootloaderCommandBankErase();

    /**
     * Reads the CC1312 flash memory starting from the specified address, and calculates a CRC32 across num_bytes bytes.
     * If read_repeat_count is 0 (default), each data location will only be read once.
     * @param[out] crc Reference to a uint32_t variable where the calculated CRC32 will be stored.
     * @param[in] address Address to start reading from.
     * @param[in] num_bytes Number of bytes to read and calculate CRC32 over.
     * @param[in] read_repeat_count Number of times to read each data location. Default is 0, meaning each location is
     * read once.
     * @retval True if the command was successful, false otherwise.
     */
    bool BootloaderCommandCRC32(uint32_t& crc, uint32_t address, uint32_t num_bytes, uint32_t read_repeat_count = 0);

    /**
     * Initiates a firmware download and checks to make sure that the firmware image and flash location are valid for
     * the device. Does not perform any erase or flashing operation, needs to be followed up with CommandSendData.
     * @param[in] address Address to start downloading the firmware to.
     * @param[in] num_bytes Number of bytes to download.
     * @retval True if the command was successful, false otherwise.
     */
    bool BootloaderCommandDownload(uint32_t address, uint32_t num_bytes);

    /**
     * Verifies that the last command sent to the CC1312 bootloader was successful by sending a COMMAND_GET_STATUS
     * command and checking that the value returned is COMMAND_RET_SUCCESS.
     */
    CommandReturnStatus BootloaderCommandGetStatus();

    /**
     * Sends a COMMAND_MEMORY_READ to the CC1312 bootloader and reads the requested memory into the provided buffer.
     * Works automatically with buffers of type uint32_t or uint8_t. If the buffer is of type uint8_t, note that words
     * are written into the buffer LSB first.
     * @param[in] address Address to read from.
     * @param[out] buf Buffer to read into.
     * @param[in] buf_len Length of the buffer to read into (in number of elements, not bytes).
     * @retval True if the command was successful, false otherwise.
     */
    template <typename T>
    bool BootloaderCommandMemoryRead(uint32_t address, T* buf, uint8_t buf_len) {
        static const uint16_t kMemoryReadUint8MaxNumBytes = 253;
        static const uint16_t kMemoryReadUint32MaxNumBytes = 63;

        static_assert(std::is_same<T, uint32_t>::value || std::is_same<T, uint8_t>::value,
                      "T must be either uint32_t or uint8_t");

        bool is_32bit = sizeof(T) == 4;

        if ((is_32bit && buf_len > kMemoryReadUint32MaxNumBytes) ||
            (!is_32bit && buf_len > kMemoryReadUint8MaxNumBytes)) {
            CONSOLE_ERROR("CC1312::BootloaderCommandMemoryRead", "Buffer length too large, max is %d.",
                          is_32bit ? kMemoryReadUint32MaxNumBytes : kMemoryReadUint8MaxNumBytes);
            return false;
        }
        uint8_t cmd_buf[] = {kCmdMemoryRead,
                             static_cast<uint8_t>((address >> 24) & 0xFFu),
                             static_cast<uint8_t>((address >> 16) & 0xFFu),
                             static_cast<uint8_t>((address >> 8) & 0xFFu),
                             static_cast<uint8_t>(address & 0xFFu),
                             static_cast<uint8_t>(is_32bit ? 0x01u : 0x00u),  // 1 for 32-bit, 0 for 8-bit.
                             buf_len};

        if (!BootloaderSendBuffer(cmd_buf, sizeof(cmd_buf))) {
            CONSOLE_ERROR("CC1312::BootloaderCommandMemoryRead", "Failed to send memory read command.");
            return false;
        }

        uint8_t data_buf[is_32bit ? 4 * buf_len : buf_len] = {0};
        if (!BootloaderReceiveBuffer(data_buf, sizeof(data_buf))) {
            CONSOLE_ERROR("CC1312::BootloaderCommandMemoryRead", "Failed to receive reply to memory read command.");
            return false;
        }

        if (is_32bit) {
            // Memory buffer is given as LSB first, so we need to reverse the order of the bytes.
            for (uint8_t i = 0; i < buf_len; i++) {
                buf[i] = static_cast<T>((data_buf[4 * i + 3] << 24) | (data_buf[4 * i + 2] << 16) |
                                        (data_buf[4 * i + 1] << 8) | data_buf[4 * i]);
            }
        } else {
            for (uint8_t i = 0; i < buf_len; i++) {
                buf[i] = data_buf[i];
            }
        }
        return true;
    }

    /**
     * Writes to an arbitrary memory address on the CC1312 using COMMAND_MEMORY_WRITE via the bootloader. The buffer can
     * be of type uint8_t or uint32_t. If the buffer is of tpe uint8_t, it is assumed to be in LSB format, meaning that
     * the first byte is the least significant byte of the first word to write to (assuming word-aligned access). If the
     * bfufer is of type uint32_t, each element is a full word that will automatically be transposed to LSB format
     * before writing to memory.
     * NOTE: This function cannot be used to write to flash memory, and will cause issues if writing to the lower 4kB of
     * SRAM (0x20000000 to 0x20000FFF) or to the hardware register controllign the fucntionality of the serial interface
     * being used by the bootloader.
     * @param[in] address Address to write to.
     * @param[in] buf Buffer to write from.
     * @param[in] buf_len Length of the buffer to write from (in number of elements, not bytes).
     * @retval True if the command was successful, false otherwise.
     */
    template <typename T>
    bool BootloaderCommandMemoryWrite(uint32_t address, T* buf, uint8_t buf_len) {
        static_assert(std::is_same<T, uint32_t>::value || std::is_same<T, uint8_t>::value,
                      "T must be either uint32_t or uint8_t");

        bool is_32bit = sizeof(T) == 4;

        if ((is_32bit && buf_len > kBootloaderCommandMemoryWriteUint32MaxNumBytes) ||
            (!is_32bit && buf_len > kBootloaderCommandMemoryWriteUint8MaxNumBytes)) {
            CONSOLE_ERROR("CC1312::BootloaderCommandMemoryWrite", "Buffer length too large, max is %d.",
                          is_32bit ? kBootloaderCommandMemoryWriteUint32MaxNumBytes
                                   : kBootloaderCommandMemoryWriteUint8MaxNumBytes);
            return false;
        }

        uint8_t cmd_buf[buf_len * sizeof(T) + 6];
        cmd_buf[0] = kCmdMemoryWrite;
        cmd_buf[1] = static_cast<uint8_t>((address >> 24) & 0xFFu);
        cmd_buf[2] = static_cast<uint8_t>((address >> 16) & 0xFFu);
        cmd_buf[3] = static_cast<uint8_t>((address >> 8) & 0xFFu);
        cmd_buf[4] = static_cast<uint8_t>(address & 0xFFu);
        cmd_buf[5] = static_cast<uint8_t>(is_32bit ? 0x01u : 0x00u);  // 1 for 32-bit, 0 for 8-bit.

        if (is_32bit) {
            for (uint16_t i = 0; i < buf_len; i++) {
                // Reshuffle 32-bit words into LSB formatted bytes.
                cmd_buf[6 + 4 * i] = static_cast<uint8_t>(buf[i] & 0xFFu);
                cmd_buf[6 + 4 * i + 1] = static_cast<uint8_t>((buf[i] >> 8) & 0xFFu);
                cmd_buf[6 + 4 * i + 2] = static_cast<uint8_t>((buf[i] >> 16) & 0xFFu);
                cmd_buf[6 + 4 * i + 3] = static_cast<uint8_t>((buf[i] >> 24) & 0xFFu);
            }
        } else {
            // 8-bit incoming buffer is assumed to already be in LSB format.
            memcpy(cmd_buf + 6, buf, buf_len);  // Copy the buffer to the command buffer.
        }

        if (!BootloaderSendBufferCheckSuccess(cmd_buf, sizeof(cmd_buf))) {
            CONSOLE_ERROR("CC1312::BootloaderCommandMemoryWrite", "Failed to send memory write command.");
            return false;
        }

        return true;
    }

    /**
     * Sends a COMMAND_PING to the CC1312 in bootloader mode, and returns true if an ACK is received.
     * @retval True if the CC1312 is in bootloader mode, false otherwise.
     */
    bool BootloaderCommandPing();

    /**
     * Initiates a reset of the CC1312. Used t boot into a new app after flashing.
     * @retval True if the command was ACKed (no get status check), false otherwise.
     */
    bool BootloaderCommandReset();

    /**
     * Sends data to be programmed following a COMMAND_DOWNLOAD command. Verifies by checking for ACK and a get status
     * success code after programming. If the programming was successful, returns true. If the programming failed,
     * returns false. Programming address only needs to be incremented if programming was successful.
     * @param[in] data_buf Pointer to the data buffer to program, least significant Byte first.
     * @param[in] data_len_bytes Length of the data buffer in bytes.
     * @retval True if the programming was successful, false otherwise.
     */
    bool BootloaderCommandSendData(const uint8_t* data_buf, uint32_t data_len_bytes);

    /**
     * Sets values in CCFG registers using COMMAND_SET_CCFG. Used by the BootloaderWriteCCFGConfig function.
     * @param[in] field_id CCFG field ID to set.
     * @param[in] value Value to set the CCFG field to.
     * @retval True if the command was successful, false otherwise.
     */
    bool BootloaderCommandSetCCFG(BootloaderCCFGFieldID field_id, uint32_t value);

    /**
     * Reads the CCFG configuration from the CC1312.
     * @param[out] ccfg_config Struct to read the CCFG configuration into.
     * @retval True if the command was successful, false otherwise.
     */
    bool BootloaderReadCCFGConfig(BootloaderCCFGConfig& ccfg_config);

    /**
     * Writes a CCFG configuration to the CC1312. Note that under the hood, this function can only change bits in flash
     * from 1 to 0. In order to override CCFG values from an arbitrary state, the CCFG flash region needs to be cleared
     * first.
     * @param[in] ccfg_config CCFG configuration to write.
     * @retval True if the command was successful, false otherwise.
     */
    bool BootloaderWriteCCFGConfig(const BootloaderCCFGConfig& ccfg_config);

    /**
     * Brings the CC1312 into bootloader mode and sets the SPI peripheral to be able to communicate with the CC1312.
     * NOTE: SPI peripheral uses CPOL=1, CPHA=1 in bootloader mode, CPOL=0, CPHA=0 in normal mode.
     */
    bool EnterBootloader();

    bool ExitBootloader();

    // Low-level bootloader functions.
    bool BootloaderReceiveBuffer(uint8_t* buf, uint16_t buf_len_bytes);
    bool BootloaderSendBuffer(uint8_t* buf, uint16_t buf_len_bytes);
    bool BootloaderSendBufferCheckSuccess(uint8_t* buf, uint16_t buf_len_bytes);

    /**
     * Erases the chip, re-writes its CCFG register, and flashes the application image.
     */
    bool Flash();

    /**
     * Set the enable pin.
     * @param[in] enabled True to enable the CC1312, false to disable.
     */
    inline void SetEnableState(SettingsManager::EnableState enabled) {
        if (enabled == SettingsManager::EnableState::kEnableStateExternal) {
            gpio_set_dir(config_.enable_pin, GPIO_IN);
            gpio_set_pulls(config_.enable_pin, true, false);  // Pull pin up.
        } else {
            // Drive GPIO pin with low impedance output.
            // Enable is active HI
            gpio_set_dir(config_.enable_pin, GPIO_OUT);
            gpio_put(config_.enable_pin, enabled == SettingsManager::EnableState::kEnableStateEnabled ? 1 : 0);
        }
        enabled_ = enabled;

        if (IsEnabled()) {
            sleep_ms(kBootupDelayMs);  // Wait for the CC1312 to boot up.
        } else {
            sleep_ms(kShutdownDelayMs);  // Wait for the CC1312 to shut down. This only really matters if there is a cap
                                         // on the RST line.
        }
    }

    /**
     * Wrapper function that satisfies the parent class requirement for a boolean SetEnable function. Uses the
     * three-state enable function under the hood.
     * @param[in] enabled True to enable the CC1312, false to disable.
     */
    inline void SetEnable(bool enabled) {
        SetEnableState(enabled ? SettingsManager::EnableState::kEnableStateEnabled
                               : SettingsManager::EnableState::kEnableStateDisabled);
    }

    /**
     * Returns whether the CC1312 is currently enabled.
     * @retval True if enabled, false if disabled or set to external.
     */
    inline SettingsManager::EnableState IsEnabledState() { return enabled_; }

    /**
     * Wrapper function that checks if the CC1312 is enabled. Uses the three state IsEnabledState functio nunder the
     * hood.
     * @retval True if enabled, false if disabled or set to external.
     */
    inline bool IsEnabled() {
        return enabled_ == SettingsManager::EnableState::kEnableStateEnabled ||
               enabled_ == SettingsManager::EnableState::kEnableStateExternal;
    }

    ObjectDictionary::SubGHzDeviceStatus device_status;

   private:
    CC1312Config config_;
    SettingsManager::EnableState enabled_ = SettingsManager::EnableState::kEnableStateDisabled;
    bool enable_is_external_ = false;
    bool in_bootloader_ = false;

    // Clock config struct used to set the SPI peripheral to the correct clock rate.
    struct SPIClkConfig {
        uint prescale = 0;
        uint postdiv = 0;
    };
    SPIClkConfig standby_clk_config_;
    SPIClkConfig active_clk_config_;

    /**
     * Helper function that stores the current SPI clock configuration for a SPI peripheral.
     * @retval SPIClkConfig struct with the SPI clock parameters.
     */
    inline SPIClkConfig spi_get_clk() {
        return {
            .prescale = spi_get_const_hw(config_.spi_handle)->cpsr,
            .postdiv = ((spi_get_const_hw(config_.spi_handle)->cr0 & SPI_SSPCR0_SCR_BITS) >> SPI_SSPCR0_SCR_LSB) + 1};
    }

    /**
     * Helper function that restores a SPI clock configuration for a SPI peripheral.
     * @param[in] clk_config SPI clock configuration to restore.
     */
    inline void spi_set_clk(const SPIClkConfig& clk_config) {
        spi_get_hw(config_.spi_handle)->cpsr = clk_config.prescale;
        hw_write_masked(&spi_get_hw(config_.spi_handle)->cr0, (clk_config.postdiv - 1) << SPI_SSPCR0_SCR_LSB,
                        SPI_SSPCR0_SCR_BITS);
    }

    LEDFlasher led_flasher_ = LEDFlasher({});
};