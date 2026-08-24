#ifndef ASM_SOCKETCAN_BRIDGE__SIM_TIME_H_
#define ASM_SOCKETCAN_BRIDGE__SIM_TIME_H_

#include <cstdint>

namespace asm_socketcan_bridge
{
class SimTime
{
public:
  uint64_t totalMilliseconds() const noexcept
  {
    return total_milliseconds_;
  }

  uint32_t seconds() const noexcept
  {
    return static_cast<uint32_t>(total_milliseconds_ / kMillisecondsPerSecond);
  }

  uint32_t nanoseconds() const noexcept
  {
    return static_cast<uint32_t>(
      (total_milliseconds_ % kMillisecondsPerSecond) * kNanosecondsPerMillisecond);
  }

  void advanceMilliseconds(uint64_t milliseconds) noexcept
  {
    total_milliseconds_ += milliseconds;
  }

private:
  static constexpr uint64_t kMillisecondsPerSecond = 1000;
  static constexpr uint64_t kNanosecondsPerMillisecond = 1000000;

  uint64_t total_milliseconds_ = 0;
};
}  // namespace asm_socketcan_bridge

#endif  // ASM_SOCKETCAN_BRIDGE__SIM_TIME_H_