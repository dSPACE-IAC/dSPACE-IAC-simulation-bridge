#ifndef NPC_CONTROLLER__SIM_CLOCK_CONTROL_H_
#define NPC_CONTROLLER__SIM_CLOCK_CONTROL_H_

#include <cstdint>

namespace controller
{
inline bool shouldRunSimTimeControl(int32_t seconds, uint32_t nanoseconds) noexcept
{
  return seconds != 0 || nanoseconds != 0;
}
}  // namespace controller

#endif  // NPC_CONTROLLER__SIM_CLOCK_CONTROL_H_