#include "firmware_update.hh"
#include "hardware_unit_tests.hh"

int16_t own_partition = -1;
int16_t other_partition = -1;

UTEST(Flash, AmWithinFlashPartition) {
    for (uint16_t i = 0; i < FirmwareUpdateManager::kNumPartitions; i++) {
        if (FirmwareUpdateManager::AmWithinFlashPartition(i)) {
            own_partition = i;
        } else {
            other_partition = i;
        }
    }
    ASSERT_GE_MSG(own_partition, 0, "Never found own partition.");
    ASSERT_GE_MSG(other_partition, 0, "Never found other partition.");
    ASSERT_NE_MSG(own_partition, other_partition, "Partitions can't match!");
}

UTEST(Flash, CRC32SymmetricWordAligned) {
    uint8_t sequence[] = {0xEF, 0xBE, 0xAD, 0xBE, 0xEF, 0xBE, 0xAD, 0xBE};
    uint32_t expected_crc = 0x3341B647;
    ASSERT_EQ(FirmwareUpdateManager::CalculateCRC32(sequence, sizeof(sequence)), expected_crc);
}

UTEST(Flash, CRC32SymmetricNotWordAligned) {
    uint8_t sequence[] = {0xEF, 0xBE, 0xAD, 0xEF, 0xBE, 0xAD};
    uint32_t expected_crc = 0x36b9b4a7;
    ASSERT_EQ(FirmwareUpdateManager::CalculateCRC32(sequence, sizeof(sequence)), expected_crc);
}

UTEST(Flash, CRC32AsymmetricNotWordAligned) {
    uint8_t sequence[] = {0xEF, 0xBE, 0xAD, 0xBE, 0xEF, 0xBE, 0xAD, 0xBE, 0x7, 0x8, 0x00};
    uint32_t expected_crc = 0xf446a6d1;
    ASSERT_EQ(FirmwareUpdateManager::CalculateCRC32(sequence, sizeof(sequence)), expected_crc);
}

UTEST(Flash, CRC32AsymmetricWordAligned) {
    uint8_t sequence[] = {0xEF, 0xBE, 0xAD, 0xBE, 0xEF, 0xBE, 0xAD, 0xBE, 0x7, 0x8, 0x00, 0x01};
    uint32_t expected_crc = 0x54257bff;
    ASSERT_EQ(FirmwareUpdateManager::CalculateCRC32(sequence, sizeof(sequence)), expected_crc);
}

// NOTE: Self verify does NOT work unless the firmware image was flashed from a .OTA file, since the application.bin
// used for checksum calculation differs from the result of the combined linking process. The combined image
// (combined.uf2 or combined.elf) is linked separately from app0.bin, and its partition 0 header carries app0.bin's
// length and CRC with the status forced to VALID so that the bootloader skips the check. Install the .ota file to run
// this test.
UTEST(Flash, VerifyOwnPartition) {
#ifdef COMBINED_IMAGE
    UTEST_SKIP("Running the combined image, whose partition header describes app0.bin. Install the .ota file to run.");
#else
    ASSERT_TRUE(FirmwareUpdateManager::VerifyFlashPartition(own_partition));
#endif
}