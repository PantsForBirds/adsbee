#pragma once

#include <cstdint>

#include "unit_conversions.hh"

#define CHAR_TO_HEX(c)       ((c >= 'A') ? (c >= 'a') ? (c - 'a' + 10) : (c - 'A' + 10) : (c - '0'))
#define HEX_TO_CHAR_LOWER(h) ((h) < 10 ? ('0' + (h)) : ('a' + ((h) - 10)))
#define HEX_TO_CHAR_UPPER(h) ((h) < 10 ? ('0' + (h)) : ('A' + ((h) - 10)))

/**
 * Check that the contents of a byte buffer match a string of hex characters. Must use a string of full Bytes.
 * @param[in] buffer Buffer to check.
 * @param[in] str String to check against. Must be a string of hex characters, with an even number of nibbles (full
 * Bytes).
 * @retval True if the buffer matches the string, false otherwise.
 */
bool ByteBufferMatchesString(const uint8_t *buffer, const char *str);

void PrintBinary32(uint32_t);  // for debugging

uint32_t Get24BitsFromWordBuffer(uint32_t first_bit_index, const uint32_t buffer[]);

/**
 * Extract an n-bit word from a big-endian buffer of 32-bit words. Does NOT guard against falling off the end of
 * the buffer, so be careful!
 * @param[in] n Bitlength of word to extract.
 * @param[in] first_bit_index Bit index begin reading from (index of MSb of word to read). MSb of first word in buffer
 * is bit 0.
 * @param[in] buffer Buffer to read from.
 * @retval Right-aligned n-bit word that was read from the buffer.
 */
uint32_t GetNBitsFromWordBuffer(uint16_t n, uint32_t first_bit_index, const uint32_t buffer[]);

/**
 * Extract an n-bit word from a big-endian buffer of bytes. Does NOT guard against falling off the end of
 * the buffer, so be careful!
 * @param[in] n Bitlength of word to extract.
 * @param[in] first_bit_index Bit index begin reading from (index of MSb of word to read). MSb of first byte in buffer
 * is bit 0.
 * @param[in] buffer Buffer to read from.
 * @retval Right-aligned n-bit word that was read from the buffer.
 */
uint32_t GetNBitsFromByteBuffer(uint16_t n, uint32_t first_bit_index, const uint8_t buffer[]);

/**
 * Insert an n-bit word into a big-endian buffer of 32-bit words. Does NOT guard against falling off the end of
 * the buffer, so be careful!
 * @param[in] n Bitlength of word to insert.
 * @param[in] word Word to insert. Must be right-aligned.
 * @param[in] first_bit_index Bit index where the MSb of word should be inserted. MSb of first word in buffer is bit 0.
 * @param[in] buffer Buffer to insert into.
 */
void SetNBitsInWordBuffer(uint16_t n, uint32_t word, uint32_t first_bit_index, uint32_t buffer[]);

inline void ByteBufferToWordBuffer(const uint8_t byte_buffer[], uint32_t word_buffer[], uint16_t num_bytes) {
    uint16_t num_words = num_bytes / kBytesPerWord + (num_bytes % kBytesPerWord ? 1 : 0);
    for (uint16_t i = 0; i < num_words; i++) {
        uint16_t bytes_remaining = num_bytes - i * kBytesPerWord;

        word_buffer[i] = 0x0;
        word_buffer[i] |= byte_buffer[i * kBytesPerWord] << 24;
        if (--bytes_remaining == 0) break;
        word_buffer[i] |= byte_buffer[i * kBytesPerWord + 1] << 16;
        if (--bytes_remaining == 0) break;
        word_buffer[i] |= byte_buffer[i * kBytesPerWord + 2] << 8;
        if (--bytes_remaining == 0) break;
        word_buffer[i] |= byte_buffer[i * kBytesPerWord + 3];
    }
}

inline void WordBufferToByteBuffer(const uint32_t word_buffer[], uint8_t byte_buffer[], uint16_t num_bytes) {
    uint16_t num_words = num_bytes / kBytesPerWord + (num_bytes % kBytesPerWord ? 1 : 0);
    for (uint16_t i = 0; i < num_words; i++) {
        uint16_t bytes_remaining = num_bytes - i * kBytesPerWord;

        byte_buffer[i * kBytesPerWord] = word_buffer[i] >> 24;
        if (--bytes_remaining == 0) break;
        byte_buffer[i * kBytesPerWord + 1] = (word_buffer[i] >> 16) & 0xFF;
        if (--bytes_remaining == 0) break;
        byte_buffer[i * kBytesPerWord + 2] = (word_buffer[i] >> 8) & 0xFF;
        if (--bytes_remaining == 0) break;
        byte_buffer[i * kBytesPerWord + 3] = word_buffer[i] & 0xFF;
    }
}

// CRC16 is used for inter-processor communication and reporting, not for ADS-B message decode.

/**
 * Calculates the 16-bit CRC of a buffer.
 * @param[in] buf Pointer to the buffer to calculate a CRC for.
 * @param[in] buf_len_bytes Number of bytes to calculate the CRC over.
 * @retval 16-bit CRC.
 */
uint16_t CalculateCRC16(const uint8_t *buf, int32_t buf_len_bytes);

/**
 * Converts a hex string to a byte buffer. Writes till the end of the string or max_bytes, whichever is less.
 * @param[out] byte_buffer Pointer to the output byte buffer.
 * @param[in] hex_string Pointer to a string of hex characters. Must be an even number of nibbles (full Bytes).
 * @param[in] max_bytes Maximum number of bytes to write to the output buffer.
 * @retval Number of bytes written to the output buffer.
 */
uint16_t HexStringToByteBuffer(uint8_t *byte_buffer, const char *hex_string, uint16_t max_bytes);

/**
 * Converts a byte buffer to a hex string. Writes till max_bytes is reached.
 * @param[out] hex_string Pointer to the output string buffer. Must be at least 2*max_bytes + 1 bytes long.
 * @param[in] byte_buffer Pointer to the input byte buffer.
 * @param[in] num_bytes Number of bytes to convert from the input buffer.
 * @param[in] uppercase Whether to use uppercase letters for hex digits. Defaults to false (lowercase).
 * @retval Number of characters written to the output string (not including null terminator).
 */
uint16_t ByteBufferToHexString(char *hex_string, const uint8_t *byte_buffer, uint16_t num_bytes,
                               bool uppercase = false);

/**
 * Converts a byte buffer to a padded Base64 string (RFC 4648).
 * @param[out] base64_string Output string buffer. Must hold 4 * ceil(num_bytes / 3) + 1 chars.
 * @param[in] byte_buffer Pointer to the input byte buffer.
 * @param[in] num_bytes Number of bytes to convert from the input buffer.
 * @retval Number of characters written to the output string (not including null terminator).
 */
uint16_t ByteBufferToBase64String(char *base64_string, const uint8_t *byte_buffer, uint16_t num_bytes);

/**
 * Prints a byte buffer as a hex string to the console.
 * @param[in] tag Tag to use for the console print.
 * @param[in] prefix Prefix string to print before the hex string.
 * @param[in] byte_buffer Pointer to the input byte buffer.
 * @param[in] num_bytes Number of bytes to print from the input buffer.
 */
/**
 * Writes a number of tenths as a decimal string: "-12.3", "0.5", never "-0.0".
 * @param[out] buf Output.
 * @param[in] len Size of buf.
 * @param[in] tenths The value times 10.
 */
void WriteTenths(char* buf, size_t len, int32_t tenths);

void PrintByteBuffer(const char *prefix, const uint8_t *byte_buffer, uint16_t num_bytes);