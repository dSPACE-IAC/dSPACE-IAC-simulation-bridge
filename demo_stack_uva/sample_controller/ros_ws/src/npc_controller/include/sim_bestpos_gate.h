#ifndef NPC_CONTROLLER__SIM_BESTPOS_GATE_H_
#define NPC_CONTROLLER__SIM_BESTPOS_GATE_H_

#include <cstdint>
#include <optional>

namespace controller
{
class SimBestPosGate
{
public:
  void observeClock(int32_t seconds, uint32_t nanoseconds) noexcept
  {
    clock_stamp_ = Stamp{seconds, nanoseconds};
  }

  void observeBestPos(int32_t seconds, uint32_t nanoseconds) noexcept
  {
    bestpos_stamp_ = Stamp{seconds, nanoseconds};
  }

  bool readyForStep(std::uint64_t step) const noexcept
  {
    if (!clock_stamp_) {
      return false;
    }
    if (step == 0) {
      return true;
    }
    return bestpos_stamp_ && sameStamp(*clock_stamp_, *bestpos_stamp_);
  }

  bool hasMismatchedPosition() const noexcept
  {
    return clock_stamp_ && bestpos_stamp_ && !sameStamp(*clock_stamp_, *bestpos_stamp_);
  }

  void reset() noexcept
  {
    clock_stamp_.reset();
    bestpos_stamp_.reset();
  }

private:
  struct Stamp
  {
    int32_t seconds;
    uint32_t nanoseconds;
  };

  static bool sameStamp(const Stamp &left, const Stamp &right) noexcept
  {
    return left.seconds == right.seconds && left.nanoseconds == right.nanoseconds;
  }

  std::optional<Stamp> clock_stamp_;
  std::optional<Stamp> bestpos_stamp_;
};
}  // namespace controller

#endif  // NPC_CONTROLLER__SIM_BESTPOS_GATE_H_