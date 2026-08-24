#ifndef NPC_CONTROLLER__SIGNAL_CODEC_H_
#define NPC_CONTROLLER__SIGNAL_CODEC_H_

#include <cstdint>

#include "dbc_structure.h"

namespace controller
{

  int32_t convert_to_mt_bit_ordering(uint32_t bit, uint32_t dlc = 8U);
  int32_t unpack_signal_bits(const uint8_t *data, const Signal &signal_information);
  void pack_signal_bits(uint8_t *data, const Signal &signal_information, uint64_t raw_value);

}  // namespace controller

#endif  // NPC_CONTROLLER__SIGNAL_CODEC_H_