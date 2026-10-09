#ifndef IAC_SIM_TIME__SIM_STEP_MARKER_HPP_
#define IAC_SIM_TIME__SIM_STEP_MARKER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace iac_sim_time
{
inline constexpr std::uint32_t kSimStepMarkerCanId = 0x7FFU;
inline constexpr std::size_t kSimStepMarkerPayloadSize = 8U;
using SimStepMarkerPayload = std::array<std::uint8_t, kSimStepMarkerPayloadSize>;

inline SimStepMarkerPayload encodeSimStepMarkerPayload(std::uint64_t step) noexcept
{
  SimStepMarkerPayload payload{};
  for (std::size_t index = 0; index < payload.size(); ++index) {
    payload[index] = static_cast<std::uint8_t>((step >> (index * 8U)) & 0xFFU);
  }
  return payload;
}
}  // namespace iac_sim_time

#endif  // IAC_SIM_TIME__SIM_STEP_MARKER_HPP_