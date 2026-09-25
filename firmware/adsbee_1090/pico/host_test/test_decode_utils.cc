#include "decode_utils.hh"  // for location calculation utility functions
#include "gtest/gtest.h"

TEST(DecodeUtils, GrayCodeConversion) {
    EXPECT_EQ(GrayToBinary(0b0000), 0u);
    EXPECT_EQ(GrayToBinary(0b0001), 1u);
    EXPECT_EQ(GrayToBinary(0b0011), 2u);
    EXPECT_EQ(GrayToBinary(0b0010), 3u);
    EXPECT_EQ(GrayToBinary(0b0110), 4u);
    EXPECT_EQ(GrayToBinary(0b0111), 5u);
    EXPECT_EQ(GrayToBinary(0b0101), 6u);
    EXPECT_EQ(GrayToBinary(0b0100), 7u);
    EXPECT_EQ(GrayToBinary(0b1100), 8u);
    EXPECT_EQ(GrayToBinary(0b1101), 9u);
    EXPECT_EQ(GrayToBinary(0b1111), 10u);
    EXPECT_EQ(GrayToBinary(0b1110), 11u);
    EXPECT_EQ(GrayToBinary(0b1010), 12u);
    EXPECT_EQ(GrayToBinary(0b1011), 13u);
    EXPECT_EQ(GrayToBinary(0b1001), 14u);
    EXPECT_EQ(GrayToBinary(0b1000), 15u);
}

TEST(DecodeUtils, AltitudeCodeToGillham) {
    // Set D124 = 0b111.
    EXPECT_EQ(AltitudeCodeToGillham(0b0000000010101), 0b111000000000);
    // Set D124 = 0b101.
    EXPECT_EQ(AltitudeCodeToGillham(0b0000000010001), 0b101000000000);
    // Set D124 = 0b110.
    EXPECT_EQ(AltitudeCodeToGillham(0b0000000010100), 0b110000000000);

    // Set A124 = 0b111.
    EXPECT_EQ(AltitudeCodeToGillham(0b0101010000000), 0b000111000000);
    // Set A124 = 0b101.
    EXPECT_EQ(AltitudeCodeToGillham(0b0100010000000), 0b000101000000);
    // Set A124 = 0b110.
    EXPECT_EQ(AltitudeCodeToGillham(0b0101000000000), 0b000110000000);

    // Set B124 = 0b111.
    EXPECT_EQ(AltitudeCodeToGillham(0b0000000101010), 0b000000111000);
    // Set B124 = 0b101.
    EXPECT_EQ(AltitudeCodeToGillham(0b0000000100010), 0b000000101000);
    // Set B124 = 0b110.
    EXPECT_EQ(AltitudeCodeToGillham(0b0000000101000), 0b000000110000);

    // Set C124 = 0b111.
    EXPECT_EQ(AltitudeCodeToGillham(0b1010100000000), 0b000000000111);
    // Set C124 = 0b101.
    EXPECT_EQ(AltitudeCodeToGillham(0b1000100000000), 0b000000000101);
    // Set C124 = 0b110.
    EXPECT_EQ(AltitudeCodeToGillham(0b1010000000000), 0b000000000110);

    // All 0's except X bit which is ignored.
    EXPECT_EQ(AltitudeCodeToGillham(0b0000001000000), 0b000000000000);

    // All 1's except X bit which is ignored.
    EXPECT_EQ(AltitudeCodeToGillham(0b1111110111111), 0b111111111111);
}

TEST(DecodeUtils, GillhamToAltitudeFt) {
    EXPECT_EQ(-1200, GillhamToAltitudeFt(0b000000000001));
    EXPECT_EQ(-1100, GillhamToAltitudeFt(0b000000000011));
    EXPECT_EQ(-1000, GillhamToAltitudeFt(0b000000000010));
    EXPECT_EQ(-900, GillhamToAltitudeFt(0b000000000110));
    EXPECT_EQ(-800, GillhamToAltitudeFt(0b000000000100));
    EXPECT_EQ(-700, GillhamToAltitudeFt(0b000000001100));
    EXPECT_EQ(-600, GillhamToAltitudeFt(0b000000001110));
    EXPECT_EQ(-500, GillhamToAltitudeFt(0b000000001010));
    EXPECT_EQ(-400, GillhamToAltitudeFt(0b000000001011));
    EXPECT_EQ(-300, GillhamToAltitudeFt(0b000000001001));
    EXPECT_EQ(-200, GillhamToAltitudeFt(0b000000011001));
    EXPECT_EQ(-100, GillhamToAltitudeFt(0b000000011011));
    EXPECT_EQ(0, GillhamToAltitudeFt(0b000000011010));
    EXPECT_EQ(100, GillhamToAltitudeFt(0b000000011110));
    EXPECT_EQ(200, GillhamToAltitudeFt(0b000000011100));
    EXPECT_EQ(300, GillhamToAltitudeFt(0b000000010100));
    EXPECT_EQ(400, GillhamToAltitudeFt(0b000000010110));
    EXPECT_EQ(500, GillhamToAltitudeFt(0b000000010010));
    EXPECT_EQ(600, GillhamToAltitudeFt(0b000000010011));
    EXPECT_EQ(700, GillhamToAltitudeFt(0b000000010001));
    EXPECT_EQ(800, GillhamToAltitudeFt(0b000000110001));
    EXPECT_EQ(900, GillhamToAltitudeFt(0b000000110011));
    EXPECT_EQ(1000, GillhamToAltitudeFt(0b000000110010));
    EXPECT_EQ(1100, GillhamToAltitudeFt(0b000000110110));
    EXPECT_EQ(1200, GillhamToAltitudeFt(0b000000110100));
    EXPECT_EQ(1300, GillhamToAltitudeFt(0b000000111100));
    EXPECT_EQ(1400, GillhamToAltitudeFt(0b000000111110));
    EXPECT_EQ(1500, GillhamToAltitudeFt(0b000000111010));
    EXPECT_EQ(1600, GillhamToAltitudeFt(0b000000111011));
    EXPECT_EQ(1700, GillhamToAltitudeFt(0b000000111001));
    EXPECT_EQ(1800, GillhamToAltitudeFt(0b000000101001));
    EXPECT_EQ(1900, GillhamToAltitudeFt(0b000000101011));
    EXPECT_EQ(2000, GillhamToAltitudeFt(0b000000101010));
    EXPECT_EQ(2100, GillhamToAltitudeFt(0b000000101110));
    EXPECT_EQ(2200, GillhamToAltitudeFt(0b000000101100));
    EXPECT_EQ(2300, GillhamToAltitudeFt(0b000000100100));
    EXPECT_EQ(2400, GillhamToAltitudeFt(0b000000100110));
    EXPECT_EQ(2500, GillhamToAltitudeFt(0b000000100010));
    EXPECT_EQ(2600, GillhamToAltitudeFt(0b000000100011));
    EXPECT_EQ(2700, GillhamToAltitudeFt(0b000000100001));
    EXPECT_EQ(2800, GillhamToAltitudeFt(0b000001100001));
    EXPECT_EQ(2900, GillhamToAltitudeFt(0b000001100011));
    EXPECT_EQ(3000, GillhamToAltitudeFt(0b000001100010));
    EXPECT_EQ(3100, GillhamToAltitudeFt(0b000001100110));
    EXPECT_EQ(3200, GillhamToAltitudeFt(0b000001100100));
    EXPECT_EQ(3300, GillhamToAltitudeFt(0b000001101100));
    EXPECT_EQ(3400, GillhamToAltitudeFt(0b000001101110));
    EXPECT_EQ(3500, GillhamToAltitudeFt(0b000001101010));
    EXPECT_EQ(3600, GillhamToAltitudeFt(0b000001101011));
    EXPECT_EQ(3700, GillhamToAltitudeFt(0b000001101001));
    EXPECT_EQ(3800, GillhamToAltitudeFt(0b000001111001));
    EXPECT_EQ(3900, GillhamToAltitudeFt(0b000001111011));
    EXPECT_EQ(4000, GillhamToAltitudeFt(0b000001111010));
    EXPECT_EQ(4100, GillhamToAltitudeFt(0b000001111110));
    EXPECT_EQ(4200, GillhamToAltitudeFt(0b000001111100));
    EXPECT_EQ(4300, GillhamToAltitudeFt(0b000001110100));
    EXPECT_EQ(4400, GillhamToAltitudeFt(0b000001110110));
    EXPECT_EQ(4500, GillhamToAltitudeFt(0b000001110010));
    EXPECT_EQ(4600, GillhamToAltitudeFt(0b000001110011));
    EXPECT_EQ(4700, GillhamToAltitudeFt(0b000001110001));
    EXPECT_EQ(4800, GillhamToAltitudeFt(0b000001010001));
    EXPECT_EQ(4900, GillhamToAltitudeFt(0b000001010011));
    EXPECT_EQ(5000, GillhamToAltitudeFt(0b000001010010));
    EXPECT_EQ(5100, GillhamToAltitudeFt(0b000001010110));
    EXPECT_EQ(5200, GillhamToAltitudeFt(0b000001010100));
    EXPECT_EQ(5300, GillhamToAltitudeFt(0b000001011100));
    EXPECT_EQ(5400, GillhamToAltitudeFt(0b000001011110));
    EXPECT_EQ(5500, GillhamToAltitudeFt(0b000001011010));
    EXPECT_EQ(5600, GillhamToAltitudeFt(0b000001011011));
    EXPECT_EQ(5700, GillhamToAltitudeFt(0b000001011001));
    EXPECT_EQ(5800, GillhamToAltitudeFt(0b000001001001));
    EXPECT_EQ(5900, GillhamToAltitudeFt(0b000001001011));
    EXPECT_EQ(6000, GillhamToAltitudeFt(0b000001001010));
    EXPECT_EQ(6100, GillhamToAltitudeFt(0b000001001110));
    EXPECT_EQ(6200, GillhamToAltitudeFt(0b000001001100));
    EXPECT_EQ(6300, GillhamToAltitudeFt(0b000001000100));
    EXPECT_EQ(6400, GillhamToAltitudeFt(0b000001000110));
    EXPECT_EQ(6500, GillhamToAltitudeFt(0b000001000010));
    EXPECT_EQ(6600, GillhamToAltitudeFt(0b000001000011));
    EXPECT_EQ(6700, GillhamToAltitudeFt(0b000001000001));
    EXPECT_EQ(6800, GillhamToAltitudeFt(0b000011000001));
    EXPECT_EQ(6900, GillhamToAltitudeFt(0b000011000011));
    EXPECT_EQ(7000, GillhamToAltitudeFt(0b000011000010));
    EXPECT_EQ(7100, GillhamToAltitudeFt(0b000011000110));
    EXPECT_EQ(7200, GillhamToAltitudeFt(0b000011000100));
    EXPECT_EQ(7300, GillhamToAltitudeFt(0b000011001100));
    EXPECT_EQ(7400, GillhamToAltitudeFt(0b000011001110));
    EXPECT_EQ(7500, GillhamToAltitudeFt(0b000011001010));
    EXPECT_EQ(7600, GillhamToAltitudeFt(0b000011001011));
    EXPECT_EQ(7700, GillhamToAltitudeFt(0b000011001001));
    EXPECT_EQ(7800, GillhamToAltitudeFt(0b000011011001));
    EXPECT_EQ(7900, GillhamToAltitudeFt(0b000011011011));
    EXPECT_EQ(8000, GillhamToAltitudeFt(0b000011011010));
    EXPECT_EQ(8100, GillhamToAltitudeFt(0b000011011110));
    EXPECT_EQ(8200, GillhamToAltitudeFt(0b000011011100));
    EXPECT_EQ(8300, GillhamToAltitudeFt(0b000011010100));
    EXPECT_EQ(8400, GillhamToAltitudeFt(0b000011010110));
    EXPECT_EQ(8500, GillhamToAltitudeFt(0b000011010010));
    EXPECT_EQ(8600, GillhamToAltitudeFt(0b000011010011));
    EXPECT_EQ(8700, GillhamToAltitudeFt(0b000011010001));
    EXPECT_EQ(8800, GillhamToAltitudeFt(0b000011110001));
    EXPECT_EQ(8900, GillhamToAltitudeFt(0b000011110011));
    EXPECT_EQ(9000, GillhamToAltitudeFt(0b000011110010));
    EXPECT_EQ(9100, GillhamToAltitudeFt(0b000011110110));
    EXPECT_EQ(9200, GillhamToAltitudeFt(0b000011110100));
    EXPECT_EQ(9300, GillhamToAltitudeFt(0b000011111100));
    EXPECT_EQ(9400, GillhamToAltitudeFt(0b000011111110));
    EXPECT_EQ(9500, GillhamToAltitudeFt(0b000011111010));
    EXPECT_EQ(9600, GillhamToAltitudeFt(0b000011111011));
    EXPECT_EQ(9700, GillhamToAltitudeFt(0b000011111001));
    EXPECT_EQ(9800, GillhamToAltitudeFt(0b000011101001));
    EXPECT_EQ(9900, GillhamToAltitudeFt(0b000011101011));
    EXPECT_EQ(10000, GillhamToAltitudeFt(0b000011101010));
    EXPECT_EQ(10100, GillhamToAltitudeFt(0b000011101110));
    EXPECT_EQ(10200, GillhamToAltitudeFt(0b000011101100));
    EXPECT_EQ(10300, GillhamToAltitudeFt(0b000011100100));
    EXPECT_EQ(10400, GillhamToAltitudeFt(0b000011100110));
    EXPECT_EQ(10500, GillhamToAltitudeFt(0b000011100010));
    EXPECT_EQ(10600, GillhamToAltitudeFt(0b000011100011));
    EXPECT_EQ(10700, GillhamToAltitudeFt(0b000011100001));
    EXPECT_EQ(10800, GillhamToAltitudeFt(0b000010100001));
    EXPECT_EQ(10900, GillhamToAltitudeFt(0b000010100011));
    EXPECT_EQ(11000, GillhamToAltitudeFt(0b000010100010));
    EXPECT_EQ(11100, GillhamToAltitudeFt(0b000010100110));
    EXPECT_EQ(11200, GillhamToAltitudeFt(0b000010100100));
    EXPECT_EQ(11300, GillhamToAltitudeFt(0b000010101100));
    EXPECT_EQ(11400, GillhamToAltitudeFt(0b000010101110));
    EXPECT_EQ(11500, GillhamToAltitudeFt(0b000010101010));
    EXPECT_EQ(11600, GillhamToAltitudeFt(0b000010101011));
    EXPECT_EQ(11700, GillhamToAltitudeFt(0b000010101001));
    EXPECT_EQ(11800, GillhamToAltitudeFt(0b000010111001));
    EXPECT_EQ(11900, GillhamToAltitudeFt(0b000010111011));
    EXPECT_EQ(12000, GillhamToAltitudeFt(0b000010111010));
    EXPECT_EQ(12100, GillhamToAltitudeFt(0b000010111110));
    EXPECT_EQ(12200, GillhamToAltitudeFt(0b000010111100));
    EXPECT_EQ(12300, GillhamToAltitudeFt(0b000010110100));
    EXPECT_EQ(12400, GillhamToAltitudeFt(0b000010110110));
    EXPECT_EQ(12500, GillhamToAltitudeFt(0b000010110010));
    EXPECT_EQ(12600, GillhamToAltitudeFt(0b000010110011));
    EXPECT_EQ(12700, GillhamToAltitudeFt(0b000010110001));
    EXPECT_EQ(12800, GillhamToAltitudeFt(0b000010010001));
    EXPECT_EQ(12900, GillhamToAltitudeFt(0b000010010011));
    EXPECT_EQ(13000, GillhamToAltitudeFt(0b000010010010));
    EXPECT_EQ(13100, GillhamToAltitudeFt(0b000010010110));
    EXPECT_EQ(13200, GillhamToAltitudeFt(0b000010010100));
    EXPECT_EQ(13300, GillhamToAltitudeFt(0b000010011100));
    EXPECT_EQ(13400, GillhamToAltitudeFt(0b000010011110));
    EXPECT_EQ(13500, GillhamToAltitudeFt(0b000010011010));
    EXPECT_EQ(13600, GillhamToAltitudeFt(0b000010011011));
    EXPECT_EQ(13700, GillhamToAltitudeFt(0b000010011001));
    EXPECT_EQ(13800, GillhamToAltitudeFt(0b000010001001));
    EXPECT_EQ(13900, GillhamToAltitudeFt(0b000010001011));
    EXPECT_EQ(14000, GillhamToAltitudeFt(0b000010001010));
    EXPECT_EQ(14100, GillhamToAltitudeFt(0b000010001110));
    EXPECT_EQ(14200, GillhamToAltitudeFt(0b000010001100));
    EXPECT_EQ(14300, GillhamToAltitudeFt(0b000010000100));
    EXPECT_EQ(14400, GillhamToAltitudeFt(0b000010000110));
    EXPECT_EQ(14500, GillhamToAltitudeFt(0b000010000010));
    EXPECT_EQ(14600, GillhamToAltitudeFt(0b000010000011));
    EXPECT_EQ(14700, GillhamToAltitudeFt(0b000010000001));
    EXPECT_EQ(14800, GillhamToAltitudeFt(0b000110000001));
    EXPECT_EQ(14900, GillhamToAltitudeFt(0b000110000011));
    EXPECT_EQ(15000, GillhamToAltitudeFt(0b000110000010));
    EXPECT_EQ(15100, GillhamToAltitudeFt(0b000110000110));
    EXPECT_EQ(15200, GillhamToAltitudeFt(0b000110000100));
    EXPECT_EQ(15300, GillhamToAltitudeFt(0b000110001100));
    EXPECT_EQ(15400, GillhamToAltitudeFt(0b000110001110));
    EXPECT_EQ(15500, GillhamToAltitudeFt(0b000110001010));
    EXPECT_EQ(15600, GillhamToAltitudeFt(0b000110001011));
    EXPECT_EQ(15700, GillhamToAltitudeFt(0b000110001001));
    EXPECT_EQ(15800, GillhamToAltitudeFt(0b000110011001));
    EXPECT_EQ(15900, GillhamToAltitudeFt(0b000110011011));
    EXPECT_EQ(16000, GillhamToAltitudeFt(0b000110011010));
    EXPECT_EQ(16100, GillhamToAltitudeFt(0b000110011110));
    EXPECT_EQ(16200, GillhamToAltitudeFt(0b000110011100));
    EXPECT_EQ(16300, GillhamToAltitudeFt(0b000110010100));
    EXPECT_EQ(16400, GillhamToAltitudeFt(0b000110010110));
    EXPECT_EQ(16500, GillhamToAltitudeFt(0b000110010010));
    EXPECT_EQ(16600, GillhamToAltitudeFt(0b000110010011));
    EXPECT_EQ(16700, GillhamToAltitudeFt(0b000110010001));
    EXPECT_EQ(16800, GillhamToAltitudeFt(0b000110110001));
    EXPECT_EQ(16900, GillhamToAltitudeFt(0b000110110011));
    EXPECT_EQ(17000, GillhamToAltitudeFt(0b000110110010));
    EXPECT_EQ(17100, GillhamToAltitudeFt(0b000110110110));
    EXPECT_EQ(17200, GillhamToAltitudeFt(0b000110110100));
    EXPECT_EQ(17300, GillhamToAltitudeFt(0b000110111100));
    EXPECT_EQ(17400, GillhamToAltitudeFt(0b000110111110));
    EXPECT_EQ(17500, GillhamToAltitudeFt(0b000110111010));
    EXPECT_EQ(17600, GillhamToAltitudeFt(0b000110111011));
    EXPECT_EQ(17700, GillhamToAltitudeFt(0b000110111001));
    EXPECT_EQ(17800, GillhamToAltitudeFt(0b000110101001));
    EXPECT_EQ(17900, GillhamToAltitudeFt(0b000110101011));
    EXPECT_EQ(18000, GillhamToAltitudeFt(0b000110101010));
    EXPECT_EQ(18100, GillhamToAltitudeFt(0b000110101110));
    EXPECT_EQ(18200, GillhamToAltitudeFt(0b000110101100));
    EXPECT_EQ(18300, GillhamToAltitudeFt(0b000110100100));
    EXPECT_EQ(18400, GillhamToAltitudeFt(0b000110100110));
    EXPECT_EQ(18500, GillhamToAltitudeFt(0b000110100010));
    EXPECT_EQ(18600, GillhamToAltitudeFt(0b000110100011));
    EXPECT_EQ(18700, GillhamToAltitudeFt(0b000110100001));
    EXPECT_EQ(18800, GillhamToAltitudeFt(0b000111100001));
    EXPECT_EQ(18900, GillhamToAltitudeFt(0b000111100011));
    EXPECT_EQ(19000, GillhamToAltitudeFt(0b000111100010));
    EXPECT_EQ(19100, GillhamToAltitudeFt(0b000111100110));
    EXPECT_EQ(19200, GillhamToAltitudeFt(0b000111100100));
    EXPECT_EQ(19300, GillhamToAltitudeFt(0b000111101100));
    EXPECT_EQ(19400, GillhamToAltitudeFt(0b000111101110));
    EXPECT_EQ(19500, GillhamToAltitudeFt(0b000111101010));
    EXPECT_EQ(19600, GillhamToAltitudeFt(0b000111101011));
    EXPECT_EQ(19700, GillhamToAltitudeFt(0b000111101001));
    EXPECT_EQ(19800, GillhamToAltitudeFt(0b000111111001));
    EXPECT_EQ(19900, GillhamToAltitudeFt(0b000111111011));
    EXPECT_EQ(20000, GillhamToAltitudeFt(0b000111111010));
    EXPECT_EQ(20100, GillhamToAltitudeFt(0b000111111110));
    EXPECT_EQ(20200, GillhamToAltitudeFt(0b000111111100));
    EXPECT_EQ(20300, GillhamToAltitudeFt(0b000111110100));
    EXPECT_EQ(20400, GillhamToAltitudeFt(0b000111110110));
    EXPECT_EQ(20500, GillhamToAltitudeFt(0b000111110010));
    EXPECT_EQ(20600, GillhamToAltitudeFt(0b000111110011));
    EXPECT_EQ(20700, GillhamToAltitudeFt(0b000111110001));
    EXPECT_EQ(20800, GillhamToAltitudeFt(0b000111010001));
    EXPECT_EQ(20900, GillhamToAltitudeFt(0b000111010011));
    EXPECT_EQ(21000, GillhamToAltitudeFt(0b000111010010));
    EXPECT_EQ(21100, GillhamToAltitudeFt(0b000111010110));
    EXPECT_EQ(21200, GillhamToAltitudeFt(0b000111010100));
    EXPECT_EQ(21300, GillhamToAltitudeFt(0b000111011100));
    EXPECT_EQ(21400, GillhamToAltitudeFt(0b000111011110));
    EXPECT_EQ(21500, GillhamToAltitudeFt(0b000111011010));
    EXPECT_EQ(21600, GillhamToAltitudeFt(0b000111011011));
    EXPECT_EQ(21700, GillhamToAltitudeFt(0b000111011001));
    EXPECT_EQ(21800, GillhamToAltitudeFt(0b000111001001));
    EXPECT_EQ(21900, GillhamToAltitudeFt(0b000111001011));
    EXPECT_EQ(22000, GillhamToAltitudeFt(0b000111001010));
    EXPECT_EQ(22100, GillhamToAltitudeFt(0b000111001110));
    EXPECT_EQ(22200, GillhamToAltitudeFt(0b000111001100));
    EXPECT_EQ(22300, GillhamToAltitudeFt(0b000111000100));
    EXPECT_EQ(22400, GillhamToAltitudeFt(0b000111000110));
    EXPECT_EQ(22500, GillhamToAltitudeFt(0b000111000010));
    EXPECT_EQ(22600, GillhamToAltitudeFt(0b000111000011));
    EXPECT_EQ(22700, GillhamToAltitudeFt(0b000111000001));
    EXPECT_EQ(22800, GillhamToAltitudeFt(0b000101000001));
    EXPECT_EQ(22900, GillhamToAltitudeFt(0b000101000011));
    EXPECT_EQ(23000, GillhamToAltitudeFt(0b000101000010));
    EXPECT_EQ(23100, GillhamToAltitudeFt(0b000101000110));
    EXPECT_EQ(23200, GillhamToAltitudeFt(0b000101000100));
    EXPECT_EQ(23300, GillhamToAltitudeFt(0b000101001100));
    EXPECT_EQ(23400, GillhamToAltitudeFt(0b000101001110));
    EXPECT_EQ(23500, GillhamToAltitudeFt(0b000101001010));
    EXPECT_EQ(23600, GillhamToAltitudeFt(0b000101001011));
    EXPECT_EQ(23700, GillhamToAltitudeFt(0b000101001001));
    EXPECT_EQ(23800, GillhamToAltitudeFt(0b000101011001));
    EXPECT_EQ(23900, GillhamToAltitudeFt(0b000101011011));
    EXPECT_EQ(24000, GillhamToAltitudeFt(0b000101011010));
    EXPECT_EQ(24100, GillhamToAltitudeFt(0b000101011110));
    EXPECT_EQ(24200, GillhamToAltitudeFt(0b000101011100));
    EXPECT_EQ(24300, GillhamToAltitudeFt(0b000101010100));
    EXPECT_EQ(24400, GillhamToAltitudeFt(0b000101010110));
    EXPECT_EQ(24500, GillhamToAltitudeFt(0b000101010010));
    EXPECT_EQ(24600, GillhamToAltitudeFt(0b000101010011));
    EXPECT_EQ(24700, GillhamToAltitudeFt(0b000101010001));
    EXPECT_EQ(24800, GillhamToAltitudeFt(0b000101110001));
    EXPECT_EQ(24900, GillhamToAltitudeFt(0b000101110011));
    EXPECT_EQ(25000, GillhamToAltitudeFt(0b000101110010));
    EXPECT_EQ(25100, GillhamToAltitudeFt(0b000101110110));
    EXPECT_EQ(25200, GillhamToAltitudeFt(0b000101110100));
    EXPECT_EQ(25300, GillhamToAltitudeFt(0b000101111100));
    EXPECT_EQ(25400, GillhamToAltitudeFt(0b000101111110));
    EXPECT_EQ(25500, GillhamToAltitudeFt(0b000101111010));
    EXPECT_EQ(25600, GillhamToAltitudeFt(0b000101111011));
    EXPECT_EQ(25700, GillhamToAltitudeFt(0b000101111001));
    EXPECT_EQ(25800, GillhamToAltitudeFt(0b000101101001));
    EXPECT_EQ(25900, GillhamToAltitudeFt(0b000101101011));
    EXPECT_EQ(26000, GillhamToAltitudeFt(0b000101101010));
    EXPECT_EQ(26100, GillhamToAltitudeFt(0b000101101110));
    EXPECT_EQ(26200, GillhamToAltitudeFt(0b000101101100));
    EXPECT_EQ(26300, GillhamToAltitudeFt(0b000101100100));
    EXPECT_EQ(26400, GillhamToAltitudeFt(0b000101100110));
    EXPECT_EQ(26500, GillhamToAltitudeFt(0b000101100010));
    EXPECT_EQ(26600, GillhamToAltitudeFt(0b000101100011));
    EXPECT_EQ(26700, GillhamToAltitudeFt(0b000101100001));
    EXPECT_EQ(26800, GillhamToAltitudeFt(0b000100100001));
    EXPECT_EQ(26900, GillhamToAltitudeFt(0b000100100011));
    EXPECT_EQ(27000, GillhamToAltitudeFt(0b000100100010));
    EXPECT_EQ(27100, GillhamToAltitudeFt(0b000100100110));
    EXPECT_EQ(27200, GillhamToAltitudeFt(0b000100100100));
    EXPECT_EQ(27300, GillhamToAltitudeFt(0b000100101100));
    EXPECT_EQ(27400, GillhamToAltitudeFt(0b000100101110));
    EXPECT_EQ(27500, GillhamToAltitudeFt(0b000100101010));
    EXPECT_EQ(27600, GillhamToAltitudeFt(0b000100101011));
    EXPECT_EQ(27700, GillhamToAltitudeFt(0b000100101001));
    EXPECT_EQ(27800, GillhamToAltitudeFt(0b000100111001));
    EXPECT_EQ(27900, GillhamToAltitudeFt(0b000100111011));
    EXPECT_EQ(28000, GillhamToAltitudeFt(0b000100111010));
    EXPECT_EQ(28100, GillhamToAltitudeFt(0b000100111110));
    EXPECT_EQ(28200, GillhamToAltitudeFt(0b000100111100));
    EXPECT_EQ(28300, GillhamToAltitudeFt(0b000100110100));
    EXPECT_EQ(28400, GillhamToAltitudeFt(0b000100110110));
    EXPECT_EQ(28500, GillhamToAltitudeFt(0b000100110010));
    EXPECT_EQ(28600, GillhamToAltitudeFt(0b000100110011));
    EXPECT_EQ(28700, GillhamToAltitudeFt(0b000100110001));
    EXPECT_EQ(28800, GillhamToAltitudeFt(0b000100010001));
    EXPECT_EQ(28900, GillhamToAltitudeFt(0b000100010011));
    EXPECT_EQ(29000, GillhamToAltitudeFt(0b000100010010));
    EXPECT_EQ(29100, GillhamToAltitudeFt(0b000100010110));
    EXPECT_EQ(29200, GillhamToAltitudeFt(0b000100010100));
    EXPECT_EQ(29300, GillhamToAltitudeFt(0b000100011100));
    EXPECT_EQ(29400, GillhamToAltitudeFt(0b000100011110));
    EXPECT_EQ(29500, GillhamToAltitudeFt(0b000100011010));
    EXPECT_EQ(29600, GillhamToAltitudeFt(0b000100011011));
    EXPECT_EQ(29700, GillhamToAltitudeFt(0b000100011001));
    EXPECT_EQ(29800, GillhamToAltitudeFt(0b000100001001));
    EXPECT_EQ(29900, GillhamToAltitudeFt(0b000100001011));
    EXPECT_EQ(30000, GillhamToAltitudeFt(0b000100001010));
    EXPECT_EQ(30100, GillhamToAltitudeFt(0b000100001110));
    EXPECT_EQ(30200, GillhamToAltitudeFt(0b000100001100));
    EXPECT_EQ(30300, GillhamToAltitudeFt(0b000100000100));
    EXPECT_EQ(30400, GillhamToAltitudeFt(0b000100000110));
    EXPECT_EQ(30500, GillhamToAltitudeFt(0b000100000010));
    EXPECT_EQ(30600, GillhamToAltitudeFt(0b000100000011));
    EXPECT_EQ(30700, GillhamToAltitudeFt(0b000100000001));
    EXPECT_EQ(30800, GillhamToAltitudeFt(0b001100000001));
    EXPECT_EQ(126400, GillhamToAltitudeFt(0b010000000110));
    EXPECT_EQ(126500, GillhamToAltitudeFt(0b010000000010));
    EXPECT_EQ(126600, GillhamToAltitudeFt(0b010000000011));
    EXPECT_EQ(126700, GillhamToAltitudeFt(0b010000000001));
}

TEST(DecodeUtils, AC12ToAltitudeFt) {
    EXPECT_EQ(AC12ToAltitudeFt(0), kAltitudeDecodeErrorNotAvailableOrInvalid);
    // Q=1: 25ft increments with the Q bit (bit 4) removed. N=1440 -> 35000ft.
    EXPECT_EQ(AC12ToAltitudeFt(0xB50), 35000);
    // Q=1: N=0 -> -1000ft.
    EXPECT_EQ(AC12ToAltitudeFt(0b000000010000), -1000);
    // Q=0: Gillham code, same result as the equivalent 13-bit Mode C altitude code with M=0.
    EXPECT_EQ(AC12ToAltitudeFt(0xA0A), AltitudeCodeToAltitudeFt(0b1010000001010));
}

TEST(DecodeUtils, AltitudeCodeToAltitudeFt) {
    // Altitude in meters.
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0101111011100), 4921);
    // Altitude in 25ft increments.
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b1011100011000), 36000);
    // Mode C Altitude Reply
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0011010100010), 10000);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b1010000100000), 2400);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0001100011011), 4075);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0001100011110), 4150);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0111000110110), 21950);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0000010011001), 25);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0011110100010), 9900);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0111000111001), 22025);
    EXPECT_EQ(AltitudeCodeToAltitudeFt(0b0000100011100), 900);
}

TEST(DecodeUtils, IdentityCodeToSquawk) {
    EXPECT_EQ(IdentityCodeToSquawk(0b1000101101101), 0356);  // Octal 0356.
}