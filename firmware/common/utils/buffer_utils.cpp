#include "buffer_utils.hh"
#include "core1_ram.hh"

#include <array>
#include <cstring>  // For strlen.

#include "comms.hh"
#include "macros.hh"  // for MAX

#define BITMASK_32_ALL   0xFFFFFFFF
#define WORD_32_NUM_BITS 32

bool ByteBufferMatchesString(const uint8_t *buffer, const char *str) {
    if (buffer == nullptr || str == nullptr) {
        return false;  // Invalid input.
    }
    uint16_t len_nibbles = strlen(str);
    if (len_nibbles % kNibblesPerByte != 0) {
        return false;  // String length must be a multiple of 2.
    }
    uint16_t len_bytes = strlen(str) / kNibblesPerByte;

    for (uint16_t i = 0; i < len_bytes; i++) {
        if (buffer[i] != (CHAR_TO_HEX(str[i * kNibblesPerByte]) << 4 | CHAR_TO_HEX(str[i * kNibblesPerByte + 1]))) {
            return false;
        }
    }
    return true;
}

// NOTE: Buffer operations are done big-endian (oldest bits are stored in the MSB), since input buffer shifts left.

uint32_t CORE1_RAM_FUNC(Get24BitsFromWordBuffer)(uint32_t first_bit_index, const uint32_t buffer[]) {
    return GetNBitsFromWordBuffer(24, first_bit_index, buffer);
}

uint32_t GetNBitsFromWordBuffer(uint16_t n, uint32_t first_bit_index, const uint32_t buffer[]) {
    // NOTE: Bit 0 is the MSb in this format, since the input shift register shifts left (oldest bit is MSb).
    if (n > WORD_32_NUM_BITS || n < 1) {
        CONSOLE_ERROR("GetNBitsFromWordBuffer",
                      "Tried to get %d bit word from buffer, but word bitlength must be between 1 "
                      "and 32.\r\n",
                      n);
        return 0;
    }
    uint32_t first_word_index_32 = first_bit_index / WORD_32_NUM_BITS;
    uint16_t bit_offset_32 = first_bit_index % WORD_32_NUM_BITS;
    // Get a 32-bit word, then mask it down to n bits.
    // Grab the lower portion of the word.
    uint32_t word_n = ((buffer[first_word_index_32] << bit_offset_32));
    uint16_t first_subword_length = WORD_32_NUM_BITS - bit_offset_32;
    if (first_subword_length < n) {
        // Grab the upper portion of the word from the next 32-bit word in the buffer.
        word_n |= (buffer[first_word_index_32 + 1] >> (first_subword_length));
    }
    word_n &= BITMASK_32_ALL << (32 - n);  // mask to the upper n bits
    word_n >>= (WORD_32_NUM_BITS - n);     // right-align
    return word_n;
}

uint32_t GetNBitsFromByteBuffer(uint16_t n, uint32_t first_bit_index, const uint8_t buffer[]) {
    // NOTE: Bit 0 is the MSb in this format, since the input shift register shifts left (oldest bit is MSb).
    if (n > WORD_32_NUM_BITS || n < 1) {
        CONSOLE_ERROR("GetNBitsFromByteBuffer",
                      "Tried to get %d bit word from buffer, but word bitlength must be between 1 "
                      "and 32.\r\n",
                      n);
        return 0;
    }

    uint16_t byte_index = first_bit_index / kBitsPerByte;
    uint16_t bit_offset_8 = first_bit_index % kBitsPerByte;
    uint16_t chunk_len_bits = MIN(kBitsPerByte - bit_offset_8, n);
    uint32_t chunk = buffer[byte_index] >> (kBitsPerByte - chunk_len_bits - bit_offset_8) &
                     (0xFF >> (kBitsPerByte - chunk_len_bits));
    int16_t bits_remaining = n - chunk_len_bits;
    uint32_t word_n = chunk;

    while (bits_remaining > 0) {
        byte_index++;
        chunk_len_bits = bits_remaining >= kBitsPerByte ? kBitsPerByte : bits_remaining;
        chunk = buffer[byte_index] >> (kBitsPerByte - chunk_len_bits);
        word_n = (word_n << chunk_len_bits) | chunk;
        bits_remaining -= chunk_len_bits;
    }
    return word_n;
}

void SetNBitsInWordBuffer(uint16_t n, uint32_t word, uint32_t first_bit_index, uint32_t buffer[]) {
    if (n > WORD_32_NUM_BITS || n < 1) {
        CONSOLE_ERROR("SetNBitsInWordBuffer",
                      "Tried to set %d-bit word in buffer, but word bitlength must be between 1 and "
                      "32.\r\n",
                      n);
        return;
    }

    word &= BITMASK_32_ALL >> (WORD_32_NUM_BITS - n);  // mask insertion word to n LSb's.
    word <<= (WORD_32_NUM_BITS - n);                   // left-align insertion word for easier handling.

    uint32_t first_word_index_32 = first_bit_index / WORD_32_NUM_BITS;
    uint16_t bit_offset_32 = first_bit_index % WORD_32_NUM_BITS;
    // Blank out up to n least significant bits of first word starting at bit_offset_32.
    buffer[first_word_index_32] &= ~((BITMASK_32_ALL << (WORD_32_NUM_BITS - n)) >> bit_offset_32);
    buffer[first_word_index_32] |= word >> bit_offset_32;  // insert insertion word into first word of buffer
    uint16_t first_subword_length = WORD_32_NUM_BITS - bit_offset_32;
    if (first_subword_length < n) {
        // Insertion word spills over into second word of buffer.
        // Blank n-first_subword_length most significant bits of second word.
        buffer[first_word_index_32 + 1] &= BITMASK_32_ALL >> (n - first_subword_length);
        buffer[first_word_index_32 + 1] |= word << first_subword_length;
    }
}

void PrintBinary32(uint32_t value) {
    CONSOLE_PRINTF("\t0b");
    for (int j = 31; j >= 0; j--) {
        CONSOLE_PRINTF(value & (0b1 << j) ? "1" : "0");
    }
    CONSOLE_PRINTF("\r\n");
}

/**
 * Swaps the endianness of a 16-bit value.
 * @param[in] value Value to swap the MSB and LSB of.
 * @retval Value with endianness swapped.
 */
uint16_t swap16(uint16_t value) { return (value << 8) | (value >> 8); }

// CRC-16/CCITT (polynomial 0x1021) lookup tables: [0][b] is the CRC of byte b, [1][b] the CRC of byte b followed by a
// zero byte. With both, two bytes cost one loop pass. The CC1312 checksums every SPI packet in a callback that holds
// off its RF callback, where this is twice as fast as one shift-and-xor pass per byte.
static constexpr auto kCRC16Tables = [] {
    std::array<std::array<uint16_t, 256>, 2> tables = {};
    for (uint16_t byte = 0; byte < 256; byte++) {
        uint16_t crc = static_cast<uint16_t>(byte << 8);
        for (int bit = 0; bit < 8; bit++) {
            crc = static_cast<uint16_t>(crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1);
        }
        tables[0][byte] = crc;
    }
    for (uint16_t byte = 0; byte < 256; byte++) {
        uint16_t crc = tables[0][byte];
        tables[1][byte] = static_cast<uint16_t>(crc << 8) ^ tables[0][crc >> 8];
    }
    return tables;
}();

uint16_t CalculateCRC16(const uint8_t *buf, int32_t buf_len_bytes) {
    if (buf == nullptr) {
        CONSOLE_ERROR("CalculateCRC16", "Attempted to calculate CRC on null buffer!");
        return 0;
    }
    if (buf_len_bytes < 0) {
        CONSOLE_ERROR("CalculateCRC16", "Attempted to calculate CRC on buffer with negative length!");
        return 0;
    }
    uint16_t crc = 0xFFFF;
    for (; buf_len_bytes >= 2; buf_len_bytes -= 2, buf += 2) {
        crc = kCRC16Tables[1][(crc >> 8) ^ buf[0]] ^ kCRC16Tables[0][(crc & 0xFF) ^ buf[1]];
    }
    if (buf_len_bytes > 0) {
        crc = static_cast<uint16_t>(crc << 8) ^ kCRC16Tables[0][(crc >> 8) ^ buf[0]];
    }
    return swap16(crc);
}

uint16_t HexStringToByteBuffer(uint8_t *byte_buffer, const char *hex_string, uint16_t max_bytes) {
    uint16_t bytes_written = 0;
    for (uint16_t i = 0; i < max_bytes; i++) {
        if (hex_string[i * kNibblesPerByte] == '\0' || hex_string[i * kNibblesPerByte + 1] == '\0') {
            break;  // Stop if we hit the end of the string.
        }
        byte_buffer[i] = CHAR_TO_HEX(hex_string[i * kNibblesPerByte]) << 4;
        byte_buffer[i] |= CHAR_TO_HEX(hex_string[i * kNibblesPerByte + 1]);
        bytes_written++;
    }
    return bytes_written;
}

uint16_t ByteBufferToHexString(char *hex_string, const uint8_t *byte_buffer, uint16_t num_bytes, bool uppercase) {
    for (uint16_t i = 0; i < num_bytes; i++) {
        hex_string[i * kNibblesPerByte] = uppercase ? HEX_TO_CHAR_UPPER((byte_buffer[i] >> 4) & 0x0F)
                                                    : HEX_TO_CHAR_LOWER((byte_buffer[i] >> 4) & 0x0F);
        hex_string[i * kNibblesPerByte + 1] =
            uppercase ? HEX_TO_CHAR_UPPER(byte_buffer[i] & 0x0F) : HEX_TO_CHAR_LOWER(byte_buffer[i] & 0x0F);
    }
    hex_string[num_bytes * kNibblesPerByte] = '\0';  // Null-terminate the string.
    return num_bytes * kNibblesPerByte;
}

uint16_t ByteBufferToBase64String(char *base64_string, const uint8_t *byte_buffer, uint16_t num_bytes) {
    static const char kBase64Chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    uint16_t num_chars = 0;
    for (uint16_t i = 0; i < num_bytes; i += 3) {
        uint16_t num_group_bytes = num_bytes - i < 3 ? num_bytes - i : 3;
        uint32_t group = byte_buffer[i] << 16;
        if (num_group_bytes > 1) group |= byte_buffer[i + 1] << 8;
        if (num_group_bytes > 2) group |= byte_buffer[i + 2];
        base64_string[num_chars++] = kBase64Chars[(group >> 18) & 0x3F];
        base64_string[num_chars++] = kBase64Chars[(group >> 12) & 0x3F];
        base64_string[num_chars++] = num_group_bytes > 1 ? kBase64Chars[(group >> 6) & 0x3F] : '=';
        base64_string[num_chars++] = num_group_bytes > 2 ? kBase64Chars[group & 0x3F] : '=';
    }
    base64_string[num_chars] = '\0';
    return num_chars;
}

void PrintByteBuffer(const char *prefix, const uint8_t *byte_buffer, uint16_t num_bytes) {
    char hex_string[num_bytes * kNibblesPerByte + 1];
    ByteBufferToHexString(hex_string, byte_buffer, num_bytes);
    CONSOLE_PRINTF("%s %s\r\n", prefix, hex_string);
}