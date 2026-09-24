#pragma once

#include "data_structures.hh"
#include "mode_s_packet.hh"
#include "remote_id_packet.hh"
#include "stdint.h"
#include "stdio.h"
#include "uat_packet.hh"

/**
 * CompositeArrays are used to shuffle packets around between internal functions and on the SPI bus. With multiple
 * coexisting packet types, it becomes inefficient to run multiple SPI transactions, one for each packet type, and
 * likewise it's inefficient to allocate multiple separate packet buffers internally, since not all of the buffer space
 * will get used, and the code becomes harder to change as we add more packet types.
 *
 * The CompositeArray class solves this problem by providing a nice way to pass around a single buffer that contains
 * zero or more packets of each kind. The buffer starts with a CompositeArray::RawPackets::Header, which describes how
 * many packets of each type are present, followed by arrays of raw packets of each type.
 */
class CompositeArray {
   public:
    struct RawPackets {
        static constexpr uint16_t kErrorMessageMaxLen = 256;
        static constexpr uint16_t kMaxLenBytes =
            2000;  // Arbitrary maximum size, needs to fit within SCResponsePacket data bytes.

        // This header is used on the SPI bus to precede a packed buffer of packets.
        struct __attribute__((__packed__)) Header {
            uint16_t num_mode_s_packets = 0;
            uint16_t num_uat_adsb_packets = 0;
            uint16_t num_uat_uplink_packets = 0;
            // Formerly a padding word (kept the Header at 8 bytes / word-aligned). Now repurposed to count Remote ID
            // packets, which the ESP32 forwards up to the RP2040. Firmwares are rebuilt in lockstep, and any firmware
            // that predates Remote ID simply wrote 0 here, so reusing the field is backwards compatible on the wire.
            uint16_t num_remote_id_packets = 0;
        };

        /**
         * Validates the CompositeArray::RawPackets struct to ensure that the header is present and that the length
         * is sufficient to hold the declared number of packets. If an error_msg buffer is provided, it will be filled
         * with a description of the error if the struct is invalid.
         * @param[in,out] error_msg Optional buffer to fill with an error message if the struct is invalid.
         * @retval True if the struct is valid, false otherwise.
         */
        bool IsValid(char* error_msg = nullptr) const {
            if (header == nullptr) {
                if (error_msg) {
                    snprintf(error_msg, kErrorMessageMaxLen, "Invalid CompositeArray::RawPackets: null header.");
                }
                return false;
            }
            if (len_bytes < sizeof(Header)) {
                if (error_msg) {
                    snprintf(error_msg, kErrorMessageMaxLen,
                             "Invalid CompositeArray::RawPackets: insufficient length for header.");
                }
                return false;
            }
            uint16_t expected_len_bytes = sizeof(Header) + header->num_mode_s_packets * sizeof(RawModeSPacket) +
                                          header->num_uat_adsb_packets * sizeof(RawUATADSBPacket) +
                                          header->num_uat_uplink_packets * sizeof(RawUATUplinkPacket) +
                                          header->num_remote_id_packets * sizeof(RawRemoteIDPacket);
            if (len_bytes < expected_len_bytes) {
                if (error_msg) {
                    snprintf(error_msg, kErrorMessageMaxLen,
                             "Invalid CompositeArray::RawPackets: insufficient length for %u Mode S packet(s), %u UAT "
                             "ADSB packet(s), %u UAT Uplink packet(s), %u Remote ID packet(s) (expected %u bytes, got "
                             "%u bytes).",
                             header->num_mode_s_packets, header->num_uat_adsb_packets, header->num_uat_uplink_packets,
                             header->num_remote_id_packets, expected_len_bytes, len_bytes);
                }
                return false;
            }
            return true;
        }

        uint16_t len_bytes = 0;  // Total length of the entire array in bytes, including header.

        Header* header;
        RawModeSPacket* mode_s_packets = nullptr;          // Array of RawModeSPacket structs.
        RawUATADSBPacket* uat_adsb_packets = nullptr;      // Array of RawUATADSBPacket structs.
        RawUATUplinkPacket* uat_uplink_packets = nullptr;  // Array of RawUATUplinkPacket structs.
        RawRemoteIDPacket* remote_id_packets = nullptr;    // Array of RawRemoteIDPacket structs.
    };

    // Ensure that things are nicely word-aliged so that we can do direct array access without memcpy.
    static_assert(sizeof(RawPackets::Header) % 4 == 0);  // Make sure it's word-aligned.
    static_assert(sizeof(RawModeSPacket) % 4 == 0);
    static_assert(sizeof(RawUATADSBPacket) % 4 == 0);
    static_assert(sizeof(RawUATUplinkPacket) % 4 == 0);
    static_assert(sizeof(RawRemoteIDPacket) % 4 == 0);

    /**
     * Fills a buffer as a CompositeArray (RawPackets::Header) followed by arrays of raw packets of each kind. Takes
     * pointers to queues to dequeue the packets from; any queues that are passed as a nullptr are skipped.
     * @param[out] buf Buffer to fill with header and packed packets.
     * @param[out] buf_len_bytes Length of the buffer, in bytes.
     * @param[in] mode_s_queue Pointer to a PFBQueue of RawModeSPacket to dequeue packets from, or nullptr to skip.
     * @param[in] uat_adsb_queue Pointer to a PFBQueue of RawUATADSBPacket to dequeue packets from, or nullptr to skip.
     * @param[in] uat_uplink_queue Pointer to a PFBQueue of RawUATUplinkPacket to dequeue packets from, or nullptr to
     * skip.
     * @param[in] remote_id_queue Pointer to a PFBQueue of RawRemoteIDPacket to dequeue packets from, or nullptr to skip.
     * @retval CompositeArray::RawPackets struct that describes the contents of the buffer.
     */
    static RawPackets PackRawPacketsBuffer(uint8_t* buf, uint16_t buf_len_bytes, PFBQueue<RawModeSPacket>* mode_s_queue,
                                           PFBQueue<RawUATADSBPacket>* uat_adsb_queue,
                                           PFBQueue<RawUATUplinkPacket>* uat_uplink_queue,
                                           PFBQueue<RawRemoteIDPacket>* remote_id_queue = nullptr);

    /**
     * Unpacks a buffer containing a CompositeArray (RawPackets::Header) followed by arrays of raw packets of each kind,
     * and returns a CompositeArray::RawPackets struct that describes the contents of the buffer.
     * @param[out] packets Struct to fill with pointers to the arrays in the buffer.
     * @param[in] buf Buffer containing header and packed packets.
     * @param[in] buf_len_bytes Length of the buffer, in bytes.
     * @retval CompositeArray::RawPackets struct that describes the contents of the buffer.
     */
    static bool UnpackRawPacketsBuffer(CompositeArray::RawPackets& packets, uint8_t* buf, uint16_t buf_len_bytes);

    /**
     * Unpacks a buffer containing a CompositeArray (RawPackets::Header) followed by arrays of raw packets of each kind,
     * and enqueues the packets into the provided queues. Any queues that are passed as a nullptr are skipped. Returns
     * true if every packet that was passed in found a home, false otherwise.
     * @param[in] buf Buffer containing header and packed packets.
     * @param[in] buf_len_bytes Length of the buffer, in bytes.
     * @param[out] mode_s_queue Pointer to a PFBQueue of RawModeSPacket to enqueue packets into, or nullptr to skip.
     * @param[out] uat_adsb_queue Pointer to a PFBQueue of RawUATADSBPacket to enqueue packets into, or nullptr to skip.
     * @param[out] uat_uplink_queue Pointer to a PFBQueue of RawUATUplinkPacket to enqueue packets into, or nullptr to
     * skip.
     * @param[out] remote_id_queue Pointer to a PFBQueue of RawRemoteIDPacket to enqueue packets into, or nullptr to
     * skip.
     * @retval True if all packets were successfully enqueued, false otherwise.
     */
    static bool UnpackRawPacketsBufferToQueues(uint8_t* buf, uint16_t buf_len_bytes,
                                               PFBQueue<RawModeSPacket>* mode_s_queue,
                                               PFBQueue<RawUATADSBPacket>* uat_adsb_queue,
                                               PFBQueue<RawUATUplinkPacket>* uat_uplink_queue,
                                               PFBQueue<RawRemoteIDPacket>* remote_id_queue = nullptr);

    /**
     * Calculates the length in bytes of a raw packet buffer that would be formed by emptying the contents of the
     * provided queues. This function does not modify the queues; it only calculates the size.
     * @param[in] mode_s_queue Pointer to a PFBQueue of RawModeSPacket to count packets from, or nullptr to skip.
     * @param[in] uat_adsb_queue Pointer to a PFBQueue of RawUATADSBPacket to count packets from, or nullptr to skip.
     * @param[in] uat_uplink_queue Pointer to a PFBQueue of RawUATUplinkPacket to count packets from, or nullptr to
     * skip.
     * @param[in] remote_id_queue Pointer to a PFBQueue of RawRemoteIDPacket to count packets from, or nullptr to skip.
     * @retval The total length in bytes, including the header and all packets from the queues.
     */
    static uint16_t CalculateRawPacketsBufferLength(PFBQueue<RawModeSPacket>* mode_s_queue,
                                                    PFBQueue<RawUATADSBPacket>* uat_adsb_queue,
                                                    PFBQueue<RawUATUplinkPacket>* uat_uplink_queue,
                                                    PFBQueue<RawRemoteIDPacket>* remote_id_queue = nullptr);

    /**
     * Merges packets from src_buf into dst_buf in-place. Packets of each type from src are appended after the
     * corresponding type section in dst, preserving the [Header|ModeS|ADSB|Uplink] layout in dst_buf.
     * @param[in,out] dst_buf Buffer to merge into. Modified in-place on success.
     * @param[in] src_buf Buffer to read packets from. Not modified.
     * @retval True if merge succeeded. False if the combined content would exceed kMaxLenBytes, in which case
     * dst_buf is left unmodified.
     */
    static bool MergeRawPacketsBuffers(uint8_t* dst_buf, const uint8_t* src_buf);
};