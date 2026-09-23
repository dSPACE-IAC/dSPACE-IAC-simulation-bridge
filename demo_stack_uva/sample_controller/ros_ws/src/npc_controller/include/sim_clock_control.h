#ifndef NPC_CONTROLLER__SIM_CLOCK_CONTROL_H_
#define NPC_CONTROLLER__SIM_CLOCK_CONTROL_H_

#include <cstdint>

namespace controller
{
constexpr bool shouldCreatePeriodicControlTimers(bool sim_mode) noexcept
{
  return !sim_mode;
}

inline bool shouldRunSimTimeControl(int32_t seconds, uint32_t nanoseconds) noexcept
{
  return seconds != 0 || nanoseconds != 0;
}

constexpr std::uint64_t simStepForClockMessage(std::uint64_t clock_message_count) noexcept
{
  return clock_message_count > 0 ? clock_message_count - 1U : 0U;
}

constexpr bool shouldWaitForSimStepMarker(bool sim_mode, bool use_raptor_dbw_node) noexcept
{
  return sim_mode && !use_raptor_dbw_node;
}

template <typename ControlFunction, typename HandshakeFunction>
bool runSimTimeControlStep(
  int32_t seconds,
  uint32_t nanoseconds,
  ControlFunction control,
  HandshakeFunction send_handshake)
{
  const bool control_ran = shouldRunSimTimeControl(seconds, nanoseconds);
  if (control_ran) {
    control();
  }
  send_handshake();
  return control_ran;
}
}  // namespace controller

#endif  // NPC_CONTROLLER__SIM_CLOCK_CONTROL_H_