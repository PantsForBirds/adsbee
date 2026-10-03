#ifndef CRC_HH_
#define CRC_HH_

#include <cstdint>

// Mode S CRC-24 generator polynomial x^24 + 0xFFF409 (x^24 implied). generate_crc_tables.py must use the same value.
static constexpr uint32_t kCRC24Generator = 0xFFF409;

/**
 * Calculates the CRC24 of a buffer. Note that the CRC is calculated over the entire buffer, so buffer_len_bytes should
 * only inclue the payload and not the trailing CRC.
 * @param[in] buffer Pointer to the buffer to calculate a CRC for.
 * @param[in] buffer_len_bytes Number of bytes to calculate the CRC over (payload only).
 * @param[in] initial_value Initial value of the CRC.
 * @retval 24-bit CRC.
 */
uint32_t crc24(const uint8_t *buffer, uint16_t buffer_len_bytes, uint32_t initial_value = 0x0);

/**
 * Calculates the CRC24 syndrome of a buffer. This will return 0 if the calculated CRC matches the CRC contained in the
 * last 3 words of the buffer. If the return is non-zero, it can be used to find the location of bit errors.
 * @param[in] buffer Pointer to the buffer to calculate a CRC for.
 * @param[in] buffer_len_bytes Number of bytes to calculate the CRC over, plus the CRC to check against (payload + CRC).
 * @param[in] initial_value Initial value of the CRC.
 * @retval CRC24 syndrome.
 */
uint32_t crc24_syndrome(const uint8_t *buffer, uint16_t buffer_len_bytes, uint32_t initial_value = 0x0);

/**
 * Finds the index of a single-bit error in a CRC24 message.
 * @param[in] syndrome CRC24 syndrome.
 * @param[in] message_len_bits Length of the message in bits.
 * @retval Index of the single-bit error, or -1 if no error is found. -2 is returned if the number of bits is invalid.
 */
int16_t crc24_find_single_bit_error(uint32_t syndrome, uint16_t message_len_bits);

/**
 * Flips a single bit in a message at a given index in a Byte array.
 */
void flip_bit(uint8_t *message, uint16_t index);

/**
 * Flips a single bit in a message at a given index in a Word array.
 */
void flip_bit(uint32_t *message, uint16_t index);

/**
 * Calculates a CRC32 of a buffer. Uses the tables in crc_tables.hh, which are seeded with the IEEE 802.3 generator
 * polynomial (same as zlib default implementation). Note that this CRC function uses an initial value of 0xFFFFFFFF to
 * comply with the IEEE 802.3 standard, and also includes a final XOR with 0xFFFFFFFF.
 * @param[in] buffer Pointer to the buffer to calculate a CRC for.
 * @param[in] buffer_len_bytes Number of bytes to calculate the CRC over.
 * @param[in] initial_value Initial value of the CRC.
 * @retval 32-bit CRC.
 */
uint32_t crc32_ieee_802_3(const uint8_t *buffer, uint32_t buffer_len_bytes, uint32_t initial_value = 0xFFFFFFFF);

#endif /* CRC_HH_ */