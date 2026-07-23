// ============================================================================
//  Unit tests for the CRC-8 wire protocol.
//
//  These MUST agree bit-for-bit with the firmware's MB_Crc8 in
//  stm32/Core/Src/minibot_protocol.c. If the two ever diverge, the STM32
//  silently rejects every command and the robot just sits there while you
//  debug Nav2 for a day. That is exactly the failure this test exists to
//  prevent, which is why the known-answer vectors below are pinned.
// ============================================================================
#include <gtest/gtest.h>
#include <cstring>
#include "minibot_hardware/serial_link.hpp"

using minibot_hardware::SerialLink;

TEST(Crc8, KnownAnswerVectors)
{
  EXPECT_EQ(SerialLink::crc8("", 0), 0x00);

  // Pinned against the firmware. Verified on host with the real
  // minibot_protocol.c before this test was written.
  const char * v = "V,1000,-1000";
  EXPECT_EQ(SerialLink::crc8(v, std::strlen(v)), 0x47);
}

TEST(Crc8, DetectsEverySingleBitFlip)
{
  const char * base = "V,1000,-1000";
  const size_t n = std::strlen(base);
  const uint8_t good = SerialLink::crc8(base, n);

  for (size_t i = 0; i < n; ++i) {
    for (int b = 0; b < 8; ++b) {
      char m[32];
      std::memcpy(m, base, n + 1);
      m[i] = static_cast<char>(m[i] ^ (1 << b));
      EXPECT_NE(SerialLink::crc8(m, n), good)
        << "single-bit flip at byte " << i << " bit " << b << " UNDETECTED";
    }
  }
}

TEST(Crc8, DetectsBurstErrors)
{
  const char * base = "V,1000,-1000";
  const size_t n = std::strlen(base);
  const uint8_t good = SerialLink::crc8(base, n);

  // Any contiguous burst up to 8 bits must be caught by CRC-8.
  for (size_t i = 0; i + 1 < n; ++i) {
    char m[32];
    std::memcpy(m, base, n + 1);
    m[i]     = static_cast<char>(m[i] ^ 0xFF);
    EXPECT_NE(SerialLink::crc8(m, n), good) << "byte burst at " << i << " UNDETECTED";
  }
}
