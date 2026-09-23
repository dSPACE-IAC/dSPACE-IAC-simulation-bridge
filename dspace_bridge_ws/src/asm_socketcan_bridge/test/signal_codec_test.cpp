#include "signal_codec.h"

#include <array>
#include <cstdint>
#include <iostream>

namespace
{
bool expect(bool condition, const char *description)
{
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    return false;
  }
  return true;
}

Signal make_signal(std::uint8_t start_bit, std::uint8_t length,
                   std::uint8_t endian, bool is_signed)
{
  return Signal{"test", start_bit, length, endian, is_signed,
                1.0F, 0.0F, 0.0F, 0.0F, ""};
}
}  // namespace

int main()
{
  using asm_socketcan_bridge::convert_to_mt_bit_ordering;
  using asm_socketcan_bridge::pack_signal_bits;
  using asm_socketcan_bridge::unpack_signal_bits;

  if (!expect(convert_to_mt_bit_ordering(0) == 56, "first bit conversion") ||
      !expect(convert_to_mt_bit_ordering(7) == 63, "last bit in first byte conversion") ||
      !expect(convert_to_mt_bit_ordering(8) == 48, "second byte conversion") ||
      !expect(convert_to_mt_bit_ordering(0, 4) == 24, "custom DLC conversion")) {
    return 1;
  }

  {
    std::array<std::uint8_t, 8> data{0xAA, 0xAA, 0xAA, 0xAA,
                                     0xAA, 0xAA, 0xAA, 0xAA};
    const auto signal = make_signal(16, 16, 1, false);
    pack_signal_bits(data.data(), signal, 0x1234);
    if (!expect(data[2] == 0x34 && data[3] == 0x12,
                "little-endian byte-aligned packing") ||
        !expect(data[0] == 0xAA && data[1] == 0xAA && data[4] == 0xAA &&
                  data[5] == 0xAA && data[6] == 0xAA && data[7] == 0xAA,
                "byte-aligned packing preserves other bytes") ||
        !expect(unpack_signal_bits(data.data(), signal) == 0x1234,
                "little-endian byte-aligned unpacking")) {
      return 1;
    }
  }

  {
    std::array<std::uint8_t, 8> data{};
    const auto signal = make_signal(4, 12, 1, false);
    pack_signal_bits(data.data(), signal, 0xABC);
    if (!expect(data[0] == 0xC0 && data[1] == 0xAB,
                "little-endian cross-byte packing") ||
        !expect(unpack_signal_bits(data.data(), signal) == 0xABC,
                "little-endian cross-byte unpacking")) {
      return 1;
    }
  }

  {
    std::array<std::uint8_t, 8> data{};
    const auto signal = make_signal(11, 12, 0, false);
    pack_signal_bits(data.data(), signal, 0xABC);
    if (!expect(data[1] == 0x0A && data[2] == 0xBC,
                "big-endian cross-byte packing") ||
        !expect(unpack_signal_bits(data.data(), signal) == 0xABC,
                "big-endian cross-byte unpacking")) {
      return 1;
    }
  }

  {
    std::array<std::uint8_t, 8> data{};
    const auto signal = make_signal(20, 12, 1, true);
    pack_signal_bits(data.data(), signal, 0xFFB);
    if (!expect(unpack_signal_bits(data.data(), signal) == -5,
                "signed signal sign extension") ||
        !expect(unpack_signal_bits(data.data(), make_signal(20, 12, 1, false)) == 0xFFB,
                "unsigned signal keeps raw sign bit")) {
      return 1;
    }
  }

  {
    std::array<std::uint8_t, 8> data{0x05, 0, 0, 0, 0, 0, 0, 0};
    const auto signal = make_signal(4, 4, 1, false);
    pack_signal_bits(data.data(), signal, 0xA);
    if (!expect(data[0] == 0xA5, "partial-byte packing preserves adjacent bits") ||
        !expect(unpack_signal_bits(data.data(), signal) == 0xA,
                "partial-byte unpacking")) {
      return 1;
    }
  }

  {
    std::array<std::uint8_t, 8> data{};
    const auto first_bit = make_signal(0, 1, 1, false);
    const auto last_bit = make_signal(63, 1, 1, false);
    pack_signal_bits(data.data(), first_bit, 1);
    pack_signal_bits(data.data(), last_bit, 1);
    if (!expect(data[0] == 0x01 && data[7] == 0x80,
                "single-bit boundary packing") ||
        !expect(unpack_signal_bits(data.data(), first_bit) == 1 &&
                  unpack_signal_bits(data.data(), last_bit) == 1,
                "single-bit boundary unpacking")) {
      return 1;
    }
  }

  {
    std::array<std::uint8_t, 8> data{};
    const auto signal = make_signal(0, 32, 1, false);
    pack_signal_bits(data.data(), signal, 0x12345678);
    if (!expect(data[0] == 0x78 && data[1] == 0x56 && data[2] == 0x34 &&
                  data[3] == 0x12,
                "32-bit signal packing") ||
        !expect(unpack_signal_bits(data.data(), signal) == 0x12345678,
                "32-bit signal unpacking")) {
      return 1;
    }
  }

  return 0;
}