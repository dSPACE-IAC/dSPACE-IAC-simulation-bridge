#ifndef ASM_SOCKETCAN_BRIDGE__STEP_COORDINATOR_H_
#define ASM_SOCKETCAN_BRIDGE__STEP_COORDINATOR_H_

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace asm_socketcan_bridge
{
// One raw command frame observed on can0. The counter is the DBC rolling counter, if the ID has one.
struct CommandFrame
{
  std::uint32_t id = 0;
  std::uint8_t dlc = 0;
  std::array<std::uint8_t, 8> data{};
  bool has_counter = false;
  std::uint8_t counter = 0;
};

// A command ID awaited at every step whose time reaches next_due; first_due_ms is the first such time.
struct CommandSchedule
{
  std::uint32_t id = 0;
  std::uint32_t period_ms = 10;
  std::uint32_t first_due_ms = 0;
};

enum class StepState
{
  kIdle,        // before the first step
  kStepping,    // V-ESI sub-steps run against the latched snapshot
  kPublishing,  // environment outputs and marker are published
  kReleased,    // /clock published; command frames are accepted
  kClosed       // complete or timed out
};

enum class FrameDisposition
{
  kAccepted,
  kDuplicate,  // rolling counter equals the last accepted one
  kLate        // arrived outside the release window; never merged
};

struct StepArrival
{
  std::uint32_t id;
  std::int64_t latency_ns;  // steady time from release to arrival
};

struct StepRecord
{
  std::uint64_t step = 0;
  std::uint64_t t_ms = 0;
  std::int64_t release_ns = 0;
  std::int64_t close_ns = 0;
  bool timed_out = false;
  std::vector<std::uint32_t> due_ids;
  std::vector<std::uint32_t> missing_ids;
  std::vector<StepArrival> arrivals;  // fresh frames accepted in the release window
  std::uint32_t accepted = 0;
  std::uint32_t duplicates = 0;
  std::uint32_t late = 0;   // after close, before the next step began
  std::uint32_t early = 0;  // after the step began, before its release
};

struct SnapshotEntry
{
  CommandFrame frame;
  bool fresh;  // accepted since the previous snapshot; false = held last value
};

struct StepStart
{
  std::uint64_t step;
  std::uint64_t t_ms;
  std::vector<SnapshotEntry> snapshot;  // S(n-1), ascending CAN ID
  std::optional<StepRecord> finalized;  // record of step n-1
};

struct CoordinatorTotals
{
  std::uint64_t steps_closed = 0;
  std::uint64_t timeouts = 0;
  std::uint64_t consecutive_timeouts = 0;
  std::uint64_t max_consecutive_timeouts = 0;
  std::uint64_t frames = 0;
  std::uint64_t accepted = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t late = 0;
  std::uint64_t early = 0;
  std::uint64_t rephased = 0;  // scheduled IDs re-anchored by a fresh frame outside the release window
};

// Environment-owned step protocol (F.2), pure logic. Not thread-safe; time is injected as steady ns.
// Call order per step: beginStep, markPublishing, release, then onFrame/poll until closed.
class StepCoordinator
{
public:
  struct Config
  {
    std::uint32_t step_ms = 10;
    std::vector<CommandSchedule> schedule;  // IDs not listed are latched, never awaited
    std::int64_t timeout_ns = 20'000'000;
    // Adaptation phase: accept frames in every state instead of only after release.
    bool accept_outside_release = false;
    // A fresh frame of a scheduled ID outside the release window is still dropped, but moves the
    // ID's next due time so that later cycles fall into the window (mis-detected phase).
    bool rephase = true;
  };

  explicit StepCoordinator(Config config)
  {
    applyConfig(std::move(config));
  }

  // Only between steps; time and held values continue.
  void reconfigure(Config config)
  {
    if (state_ != StepState::kIdle && state_ != StepState::kClosed) {
      throw std::logic_error("StepCoordinator::reconfigure during an open step");
    }
    applyConfig(std::move(config));
  }

  StepStart beginStep()
  {
    if (state_ != StepState::kIdle && state_ != StepState::kClosed) {
      throw std::logic_error("StepCoordinator::beginStep during an open step");
    }
    StepStart start;
    if (step_ > 0) {
      start.finalized = record_;
    }
    start.snapshot = takeSnapshot();
    t_ms_ += config_.step_ms;
    ++step_;
    record_ = StepRecord{};
    record_.step = step_;
    record_.t_ms = t_ms_;
    waiting_.clear();
    state_ = StepState::kStepping;
    start.step = step_;
    start.t_ms = t_ms_;
    return start;
  }

  void markPublishing()
  {
    if (state_ != StepState::kStepping) {
      throw std::logic_error("StepCoordinator::markPublishing outside STEPPING");
    }
    state_ = StepState::kPublishing;
  }

  // Publishing /clock is the release; a step with no due command closes immediately.
  void release(std::int64_t now_ns)
  {
    if (state_ != StepState::kPublishing) {
      throw std::logic_error("StepCoordinator::release outside PUBLISHING");
    }
    state_ = StepState::kReleased;
    record_.release_ns = now_ns;
    for (auto & entry : schedule_) {
      if (t_ms_ >= entry.next_due) {
        record_.due_ids.push_back(entry.id);
        entry.next_due += entry.period_ms * (1 + (t_ms_ - entry.next_due) / entry.period_ms);
      }
    }
    waiting_ = record_.due_ids;
    if (waiting_.empty()) {
      close(now_ns, false);
    }
  }

  FrameDisposition onFrame(const CommandFrame & frame, std::int64_t now_ns)
  {
    ++totals_.frames;
    const bool in_window = state_ == StepState::kReleased;
    if (!in_window && !config_.accept_outside_release) {
      classifyLate();
      if (config_.rephase && !isDuplicate(frame)) {
        rephase(frame.id);
      }
      return FrameDisposition::kLate;
    }

    Held & held = held_[frame.id];
    if (isDuplicate(frame)) {
      ++record_.duplicates;
      ++totals_.duplicates;
      return FrameDisposition::kDuplicate;
    }
    held.frame = frame;
    held.valid = true;
    held.fresh = true;
    ++record_.accepted;
    ++totals_.accepted;

    if (in_window) {
      record_.arrivals.push_back({frame.id, now_ns - record_.release_ns});
      const auto it = std::find(waiting_.begin(), waiting_.end(), frame.id);
      if (it != waiting_.end()) {
        waiting_.erase(it);
        if (waiting_.empty()) {
          close(now_ns, false);
        }
      } else if (config_.rephase &&
        std::find(record_.due_ids.begin(), record_.due_ids.end(), frame.id) == record_.due_ids.end())
      {
        rephase(frame.id);
      }
    }
    return FrameDisposition::kAccepted;
  }

  // Closes the released step on timeout; returns true once the step is closed.
  bool poll(std::int64_t now_ns)
  {
    if (state_ == StepState::kReleased && now_ns >= deadlineNs()) {
      close(now_ns, true);
    }
    return state_ == StepState::kClosed;
  }

  // Steady time at which the released step times out.
  std::int64_t deadlineNs() const noexcept {return record_.release_ns + config_.timeout_ns;}

  StepState state() const noexcept {return state_;}
  std::uint64_t step() const noexcept {return step_;}
  std::uint64_t timeMs() const noexcept {return t_ms_;}
  const StepRecord & record() const noexcept {return record_;}
  const CoordinatorTotals & totals() const noexcept {return totals_;}
  const Config & config() const noexcept {return config_;}

private:
  struct Held
  {
    CommandFrame frame;
    bool valid = false;
    bool fresh = false;
  };

  struct ScheduleState
  {
    std::uint32_t id;
    std::uint32_t period_ms;
    std::uint64_t next_due;
  };

  void applyConfig(Config config)
  {
    if (config.step_ms == 0) {
      throw std::invalid_argument("StepCoordinator step_ms must be positive");
    }
    if (config.timeout_ns <= 0) {
      throw std::invalid_argument("StepCoordinator timeout_ns must be positive");
    }
    schedule_.clear();
    for (const auto & entry : config.schedule) {
      if (entry.period_ms == 0) {
        throw std::invalid_argument("StepCoordinator schedule period_ms must be positive");
      }
      schedule_.push_back({entry.id, entry.period_ms, entry.first_due_ms});
    }
    config_ = std::move(config);
  }

  std::vector<SnapshotEntry> takeSnapshot()
  {
    std::vector<SnapshotEntry> snapshot;
    snapshot.reserve(held_.size());
    for (auto & [id, held] : held_) {
      if (held.valid) {
        snapshot.push_back({held.frame, held.fresh});
        held.fresh = false;
      }
    }
    return snapshot;
  }

  bool isDuplicate(const CommandFrame & frame) const
  {
    const auto it = held_.find(frame.id);
    return it != held_.end() && it->second.valid && frame.has_counter &&
           it->second.frame.has_counter && it->second.frame.counter == frame.counter;
  }

  // The stack saw the clock of the last released step: t_ms_ once the step is released, the
  // previous step while the next one is still stepping or publishing.
  void rephase(std::uint32_t id)
  {
    std::uint64_t seen_ms;
    if (state_ == StepState::kClosed || state_ == StepState::kReleased) {
      seen_ms = t_ms_;
    } else if ((state_ == StepState::kStepping || state_ == StepState::kPublishing) &&
      t_ms_ >= config_.step_ms)
    {
      seen_ms = t_ms_ - config_.step_ms;
    } else {
      return;
    }
    for (auto & entry : schedule_) {
      if (entry.id == id) {
        const std::uint64_t next_due = seen_ms + entry.period_ms;
        if (entry.next_due != next_due) {
          entry.next_due = next_due;
          ++totals_.rephased;
        }
        return;
      }
    }
  }

  void classifyLate()
  {
    if (state_ == StepState::kStepping || state_ == StepState::kPublishing) {
      ++record_.early;
      ++totals_.early;
      return;
    }
    ++totals_.late;
    if (state_ == StepState::kClosed) {
      ++record_.late;
    }
  }

  void close(std::int64_t now_ns, bool timed_out)
  {
    state_ = StepState::kClosed;
    record_.close_ns = now_ns;
    record_.timed_out = timed_out;
    record_.missing_ids = waiting_;
    waiting_.clear();
    ++totals_.steps_closed;
    if (timed_out) {
      ++totals_.timeouts;
      ++totals_.consecutive_timeouts;
      totals_.max_consecutive_timeouts =
        std::max(totals_.max_consecutive_timeouts, totals_.consecutive_timeouts);
    } else {
      totals_.consecutive_timeouts = 0;
    }
  }

  Config config_;
  std::vector<ScheduleState> schedule_;
  std::map<std::uint32_t, Held> held_;
  std::vector<std::uint32_t> waiting_;
  StepRecord record_;
  CoordinatorTotals totals_;
  StepState state_ = StepState::kIdle;
  std::uint64_t step_ = 0;
  std::uint64_t t_ms_ = 0;
};
}  // namespace asm_socketcan_bridge

#endif  // ASM_SOCKETCAN_BRIDGE__STEP_COORDINATOR_H_
