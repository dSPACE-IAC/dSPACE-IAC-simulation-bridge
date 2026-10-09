#ifndef ASM_SOCKETCAN_BRIDGE__COMMAND_LATCH_H_
#define ASM_SOCKETCAN_BRIDGE__COMMAND_LATCH_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace asm_socketcan_bridge
{
// One command snapshot held constant for every V-ESI sub-step of a logical step (PF1, F11).
template <typename Command>
class CommandLatch
{
public:
  void latch(const Command & live) noexcept
  {
    snapshot_ = live;
    latched_ = true;
  }

  bool hasSnapshot() const noexcept {return latched_;}
  const Command & snapshot() const noexcept {return snapshot_;}

private:
  Command snapshot_{};
  bool latched_ = false;
};

// Command IDs that carry a rolling counter, in the order reported by the bridge summary.
enum CommandCounterSlot : std::size_t
{
  kBrakeCommandCounter = 0,     // 0x578
  kThrottleCommandCounter,      // 0x579
  kSteeringCommandCounter,      // 0x57A
  kCtReportCounter,             // 0x57C
  kCommandCounterSlotCount
};

constexpr std::array<const char *, kCommandCounterSlotCount> kCommandCounterNames = {
  "0x578", "0x579", "0x57A", "0x57C"};

// Counts latches whose rolling counter equals the previous latch of the same ID.
template <std::size_t SlotCount>
class StaleCounterTracker
{
public:
  bool observe(std::size_t slot, std::uint8_t counter) noexcept
  {
    Slot & entry = slots_[slot];
    const bool stale = entry.seen && entry.last == counter;
    entry.seen = true;
    entry.last = counter;
    ++entry.observations;
    if (stale) {
      ++entry.stale;
    }
    return stale;
  }

  std::uint64_t observations(std::size_t slot) const noexcept {return slots_[slot].observations;}
  std::uint64_t staleCount(std::size_t slot) const noexcept {return slots_[slot].stale;}

private:
  struct Slot
  {
    bool seen = false;
    std::uint8_t last = 0;
    std::uint64_t observations = 0;
    std::uint64_t stale = 0;
  };

  std::array<Slot, SlotCount> slots_{};
};
}  // namespace asm_socketcan_bridge

#endif  // ASM_SOCKETCAN_BRIDGE__COMMAND_LATCH_H_
