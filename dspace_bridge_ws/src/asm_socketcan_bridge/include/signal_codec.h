#ifndef ASM_SOCKETCAN_BRIDGE__SIGNAL_CODEC_H_
#define ASM_SOCKETCAN_BRIDGE__SIGNAL_CODEC_H_

#include <cstdint>

#include "dbc_structure.h"

namespace asm_socketcan_bridge
{

  // Convert a DBC bit position into dSPACE/Motorola (MSB-first) bit ordering.
  int32_t convert_to_mt_bit_ordering(uint32_t bit, uint32_t dlc = 8U);

  // Extract a signal's raw value from an 8-byte CAN payload (sign-extended when signed).
  int32_t unpack_signal_bits(const uint8_t *data, const Signal &signal_information);

  // Pack a raw value into an 8-byte CAN payload at the signal's bit position.
  void pack_signal_bits(uint8_t *data, const Signal &signal_information, uint64_t raw_value);

}  // namespace asm_socketcan_bridge

#endif  // ASM_SOCKETCAN_BRIDGE__SIGNAL_CODEC_H_
