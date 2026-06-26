#include "signal_codec.h"

namespace asm_socketcan_bridge
{

  namespace
  {
    constexpr int32_t kCanWordBytes = 8;
  }  // namespace

  int32_t convert_to_mt_bit_ordering(uint32_t bit, uint32_t dlc)
  {
    const int32_t message_bit_length = static_cast<int32_t>(dlc * 8U);
    const int32_t row = static_cast<int32_t>(bit / 8U);
    const int32_t offset = static_cast<int32_t>(bit % 8U);
    return (message_bit_length - (row + 1) * 8) + offset;
  }

  int32_t unpack_signal_bits(const uint8_t *data, const Signal &signal_information)
  {
    if (signal_information.length == 0) {
      return 0;
    }

    const bool little_endian = signal_information.endian != 0U;
    const int32_t length = static_cast<int32_t>(signal_information.length);

    int32_t start_bit = static_cast<int32_t>(signal_information.start_bit);
    if (little_endian) {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit);
    } else {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit) - (length - 1);
    }

    const int32_t bit = start_bit % 8;
    const bool is_exactly_byte = ((bit + length) % 8) == 0;
    const uint32_t num_bytes =
      static_cast<uint32_t>((is_exactly_byte ? 0 : 1) + ((bit + length) / 8));

    int32_t byte_index = kCanWordBytes - (start_bit / 8) - 1;
    int32_t bits_remaining = length;
    int32_t mask_shift = bit;
    int32_t right_shift = 0;

    uint32_t unsigned_result = 0;
    for (uint32_t i = 0; i < num_bytes; ++i) {
      if (byte_index < 0 || byte_index >= kCanWordBytes) {
        return 0;
      }

      int32_t mask = 0xFF;
      if (bits_remaining < 8) {
        mask >>= (8 - bits_remaining);
      }
      mask <<= mask_shift;

      const int32_t extracted_byte = (data[byte_index] & mask) >> mask_shift;
      unsigned_result |=
        static_cast<uint32_t>(extracted_byte) << (8 * static_cast<int32_t>(i) - right_shift);

      if (!little_endian) {
        if ((byte_index % kCanWordBytes) == 0) {
          byte_index += 2 * kCanWordBytes - 1;
        } else {
          --byte_index;
        }
      } else {
        ++byte_index;
      }

      bits_remaining -= (8 - mask_shift);
      right_shift += mask_shift;
      mask_shift = 0;
    }

    if (signal_information.is_signed) {
      const int32_t sign_index = length - 1;
      if (sign_index >= 0 && sign_index < 32 &&
        (unsigned_result & (1U << sign_index)) != 0U)
      {
        if (length < 32) {
          unsigned_result |= (0xFFFFFFFFU << length);
        }
      }
      return static_cast<int32_t>(unsigned_result);
    }

    return static_cast<int32_t>(unsigned_result);
  }

  void pack_signal_bits(uint8_t *data, const Signal &signal_information, uint64_t raw_value)
  {
    if (signal_information.length == 0) {
      return;
    }

    const bool little_endian = signal_information.endian != 0U;
    const int32_t length = static_cast<int32_t>(signal_information.length);

    int32_t start_bit = static_cast<int32_t>(signal_information.start_bit);
    if (little_endian) {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit);
    } else {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit) - (length - 1);
    }

    const int32_t bit = start_bit % 8;
    const bool is_exactly_byte = ((bit + length) % 8) == 0;
    const uint32_t num_bytes =
      static_cast<uint32_t>((is_exactly_byte ? 0 : 1) + ((bit + length) / 8));

    int32_t byte_index = kCanWordBytes - (start_bit / 8) - 1;
    int32_t bits_remaining = length;
    int32_t mask_shift = bit;
    int32_t right_shift = 0;

    for (uint32_t i = 0; i < num_bytes; ++i) {
      if (byte_index < 0 || byte_index >= kCanWordBytes) {
        return;
      }

      uint8_t mask = 0xFF;
      if (bits_remaining < 8) {
        mask >>= (8 - bits_remaining);
      }
      mask = static_cast<uint8_t>(mask << mask_shift);

      const uint64_t extracted_byte =
        (raw_value >> (8 * static_cast<int32_t>(i) - right_shift)) & 0xFFULL;

      data[byte_index] = static_cast<uint8_t>(data[byte_index] & ~mask);
      data[byte_index] |= static_cast<uint8_t>((extracted_byte << mask_shift) & mask);

      if (!little_endian) {
        if ((byte_index % kCanWordBytes) == 0) {
          byte_index += 2 * kCanWordBytes - 1;
        } else {
          --byte_index;
        }
      } else {
        ++byte_index;
      }

      bits_remaining -= (8 - mask_shift);
      right_shift += mask_shift;
      mask_shift = 0;
    }
  }

}  // namespace asm_socketcan_bridge
