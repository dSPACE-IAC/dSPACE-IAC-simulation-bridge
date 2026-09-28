#ifndef IAC_SIM_TIME__SIM_STEP_MARKER_HPP_
#define IAC_SIM_TIME__SIM_STEP_MARKER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

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

inline std::optional<std::uint64_t> decodeSimStepMarkerPayload(
  const std::uint8_t * data,
  std::size_t size) noexcept
{
  if (data == nullptr || size != kSimStepMarkerPayloadSize) {
    return std::nullopt;
  }

  std::uint64_t step = 0;
  for (std::size_t index = 0; index < size; ++index) {
    step |= static_cast<std::uint64_t>(data[index]) << (index * 8U);
  }
  if (step == 0) {
    return std::nullopt;
  }
  return step;
}

enum class SimStepMarkerResult
{
  accepted,
  non_monotonic
};

class SimStepMarkerSequence
{
public:
  SimStepMarkerResult accept(std::uint64_t step) noexcept
  {
    if (step == 0 || step <= last_step_) {
      ++non_monotonic_count_;
      return SimStepMarkerResult::non_monotonic;
    }

    if (step - last_step_ > 1U) {
      gap_count_ += step - last_step_ - 1U;
    }
    last_step_ = step;
    return SimStepMarkerResult::accepted;
  }

  std::uint64_t lastStep() const noexcept { return last_step_; }
  std::uint64_t gapCount() const noexcept { return gap_count_; }
  std::uint64_t nonMonotonicCount() const noexcept { return non_monotonic_count_; }

private:
  std::uint64_t last_step_ = 0;
  std::uint64_t gap_count_ = 0;
  std::uint64_t non_monotonic_count_ = 0;
};
}  // namespace iac_sim_time

#endif  // IAC_SIM_TIME__SIM_STEP_MARKER_HPP_