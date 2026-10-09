#ifndef ASM_SOCKETCAN_BRIDGE__OUTPUT_SCHEDULE_H_
#define ASM_SOCKETCAN_BRIDGE__OUTPUT_SCHEDULE_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace asm_socketcan_bridge
{
constexpr std::uint32_t kDefaultOutputIntervalMs = 10;
constexpr std::uint32_t kMaxOutputIntervalMs = 1000;

struct OutputInterval
{
  std::uint32_t interval_ms;  // 0 = output disabled
  bool valid;                 // false: the raw value was out of range and the default was used
};

// 0 disables an output, 1..kMaxOutputIntervalMs is the publication interval (F.3).
constexpr OutputInterval normalizeOutputInterval(std::int64_t raw) noexcept
{
  if (raw >= 0 && raw <= static_cast<std::int64_t>(kMaxOutputIntervalMs)) {
    return {static_cast<std::uint32_t>(raw), true};
  }
  return {kDefaultOutputIntervalMs, false};
}

// Drift-free step scheduler: an output runs at the first step with t >= next_due, then
// next_due += interval. Outputs run in registration order.
class OutputSchedule
{
public:
  void add(std::string name, std::uint32_t interval_ms, std::function<void()> action)
  {
    if (interval_ms == 0) {
      return;
    }
    entries_.push_back({std::move(name), interval_ms, 0, std::move(action)});
  }

  std::size_t size() const noexcept {return entries_.size();}

  std::size_t runDue(std::uint64_t t_ms)
  {
    std::size_t executed = 0;
    for (auto & entry : entries_) {
      if (t_ms >= entry.next_due) {
        entry.next_due += entry.interval_ms;
        entry.action();
        ++executed;
      }
    }
    return executed;
  }

  // Outputs whose interval is shorter than the step run once per step, so their rate follows the step.
  std::vector<std::string> namesShorterThan(std::uint32_t step_ms) const
  {
    std::vector<std::string> names;
    for (const auto & entry : entries_) {
      if (entry.interval_ms < step_ms) {
        names.push_back(entry.name);
      }
    }
    return names;
  }

private:
  struct Entry
  {
    std::string name;
    std::uint32_t interval_ms;
    std::uint64_t next_due;
    std::function<void()> action;
  };

  std::vector<Entry> entries_;
};
}  // namespace asm_socketcan_bridge

#endif  // ASM_SOCKETCAN_BRIDGE__OUTPUT_SCHEDULE_H_
