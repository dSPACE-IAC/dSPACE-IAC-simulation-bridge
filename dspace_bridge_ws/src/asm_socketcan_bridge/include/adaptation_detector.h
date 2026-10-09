#ifndef ASM_SOCKETCAN_BRIDGE__ADAPTATION_DETECTOR_H_
#define ASM_SOCKETCAN_BRIDGE__ADAPTATION_DETECTOR_H_

#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <map>
#include <numeric>
#include <vector>

#include "step_coordinator.h"

namespace asm_socketcan_bridge
{
struct AdaptationConfig
{
  std::uint32_t min_arrivals = 3;      // fewer arrivals: unscheduled
  std::uint32_t max_jitter_ms = 1;     // spread around the periodic grid, one sub-step
  // Wall time of one adaptation step: a reaction slower than that is stamped later, so p99
  // latency in steps is added to the jitter tolerance.
  std::int64_t wall_step_ns = 1'000'000;
  std::uint32_t min_step_ms = 1;
  std::uint32_t max_step_ms = 100;
  std::uint32_t default_step_ms = 10;  // step when neither commands nor outputs constrain it
};

struct IdClassification
{
  std::uint32_t id = 0;
  std::uint32_t arrivals = 0;
  std::uint32_t period_ms = 0;     // 0 when fewer than two arrivals
  std::uint32_t anchor_ms = 0;     // earliest grid time consistent with all arrivals
  std::uint32_t first_due_ms = 0;  // first grid time at or after the switch-over
  std::uint32_t jitter_ms = 0;
  std::uint32_t missing_cycles = 0;
  bool scheduled = false;
  bool wall_timer_suspect = false;  // unscheduled but evenly spaced in wall time
  std::int64_t latency_p50_ns = 0;
  std::int64_t latency_p95_ns = 0;
  std::int64_t latency_p99_ns = 0;
  std::int64_t latency_max_ns = 0;
};

struct DerivedSchedule
{
  std::uint32_t step_ms = 10;
  std::vector<CommandSchedule> schedule;  // scheduled IDs only
  std::vector<IdClassification> ids;      // every observed ID, ascending
};

// Learns per command ID the cycle time, phase and wall latency from a trace of fresh frames
// (F.4). Pure logic; arrivals are stamped with the simulation time the stack last saw.
class AdaptationDetector
{
public:
  explicit AdaptationDetector(AdaptationConfig config = {})
  : config_(config) {}

  void onArrival(
    std::uint32_t id, std::uint64_t sim_ms, std::int64_t wall_ns, std::int64_t latency_ns)
  {
    traces_[id].push_back({sim_ms, wall_ns, latency_ns < 0 ? 0 : latency_ns});
  }

  std::size_t observedIds() const noexcept {return traces_.size();}

  // output_intervals_ms lists the enabled environment outputs (0 entries are ignored);
  // not_before_ms is the first simulation time at which the derived schedule may wait.
  DerivedSchedule derive(
    const std::vector<std::uint32_t> & output_intervals_ms, std::uint64_t not_before_ms) const
  {
    DerivedSchedule derived;
    std::uint32_t divisor = 0;
    for (const auto & [id, trace] : traces_) {
      IdClassification c = classify(id, trace);
      if (c.scheduled) {
        divisor = std::gcd(divisor, c.period_ms);
      }
      derived.ids.push_back(c);
    }
    for (const auto interval : output_intervals_ms) {
      divisor = std::gcd(divisor, interval);
    }
    derived.step_ms = clampStep(divisor == 0 ? config_.default_step_ms : divisor);

    for (auto & c : derived.ids) {
      if (!c.scheduled) {
        continue;
      }
      c.first_due_ms = firstGridTimeAtOrAfter(c.anchor_ms, c.period_ms, not_before_ms);
      derived.schedule.push_back({c.id, c.period_ms, c.first_due_ms});
    }
    return derived;
  }

  static std::uint32_t firstGridTimeAtOrAfter(
    std::uint32_t anchor, std::uint32_t period, std::uint64_t not_before)
  {
    if (anchor >= not_before) {
      return anchor;
    }
    const std::uint64_t cycles = (not_before - anchor + period - 1) / period;
    return static_cast<std::uint32_t>(anchor + cycles * period);
  }

private:
  struct Arrival
  {
    std::uint64_t sim_ms;
    std::int64_t wall_ns;
    std::int64_t latency_ns;
  };

  struct GridFit
  {
    std::int64_t min_dev = 0;
    std::int64_t max_dev = 0;
    std::vector<std::int64_t> slots;  // grid slot of each arrival
  };

  // Deviation of every arrival from the grid first, first+period, ...
  static GridFit fitGrid(const std::vector<Arrival> & trace, std::int64_t first, std::int64_t period)
  {
    GridFit fit;
    for (const auto & arrival : trace) {
      const std::int64_t offset = static_cast<std::int64_t>(arrival.sim_ms) - first;
      const std::int64_t slot = (offset + period / 2) / period;
      const std::int64_t deviation = offset - slot * period;
      fit.min_dev = std::min(fit.min_dev, deviation);
      fit.max_dev = std::max(fit.max_dev, deviation);
      fit.slots.push_back(slot);
    }
    return fit;
  }

  static std::int64_t percentile(std::vector<std::int64_t> values, double fraction)
  {
    if (values.empty()) {
      return 0;
    }
    std::sort(values.begin(), values.end());
    const auto index = std::min(values.size() - 1, static_cast<std::size_t>(fraction * values.size()));
    return values[index];
  }

  // Largest divisor of the step that fits the range; a step above the maximum is split evenly.
  std::uint32_t clampStep(std::uint32_t step) const
  {
    if (step > config_.max_step_ms) {
      for (std::uint32_t parts = 2; parts <= step; ++parts) {
        if (step % parts == 0 && step / parts <= config_.max_step_ms) {
          step /= parts;
          break;
        }
      }
    }
    return std::max(config_.min_step_ms, std::min(step, config_.max_step_ms));
  }

  IdClassification classify(std::uint32_t id, const std::vector<Arrival> & trace) const
  {
    IdClassification c;
    c.id = id;
    c.arrivals = static_cast<std::uint32_t>(trace.size());

    std::vector<std::int64_t> latencies;
    latencies.reserve(trace.size());
    for (const auto & arrival : trace) {
      latencies.push_back(arrival.latency_ns);
    }
    c.latency_p50_ns = percentile(latencies, 0.50);
    c.latency_p95_ns = percentile(latencies, 0.95);
    c.latency_p99_ns = percentile(latencies, 0.99);
    c.latency_max_ns = latencies.empty() ? 0 : *std::max_element(latencies.begin(), latencies.end());
    if (trace.size() < 2) {
      return c;
    }

    std::vector<std::int64_t> gaps;
    for (std::size_t i = 1; i < trace.size(); ++i) {
      gaps.push_back(static_cast<std::int64_t>(trace[i].sim_ms) -
                     static_cast<std::int64_t>(trace[i - 1].sim_ms));
    }
    std::vector<std::int64_t> sorted_gaps = gaps;
    std::nth_element(sorted_gaps.begin(), sorted_gaps.begin() + sorted_gaps.size() / 2,
                     sorted_gaps.end());
    const std::int64_t median_gap = std::max<std::int64_t>(1, sorted_gaps[sorted_gaps.size() / 2]);

    // Stamps may land one step late, which makes single gaps 9 or 11 for a 10 ms loop; the
    // candidate around the median with the tightest grid fit is the period.
    const std::int64_t first = static_cast<std::int64_t>(trace.front().sim_ms);
    GridFit best;
    std::int64_t best_period = median_gap;
    for (std::int64_t candidate = std::max<std::int64_t>(1, median_gap - 2);
         candidate <= median_gap + 2; ++candidate)
    {
      const GridFit fit = fitGrid(trace, first, candidate);
      const std::int64_t spread = fit.max_dev - fit.min_dev;
      const std::int64_t best_spread = best.max_dev - best.min_dev;
      if (best.slots.empty() || spread < best_spread ||
        (spread == best_spread &&
        std::llabs(candidate - median_gap) < std::llabs(best_period - median_gap)))
      {
        best = fit;
        best_period = candidate;
      }
    }
    c.period_ms = static_cast<std::uint32_t>(best_period);
    c.jitter_ms = static_cast<std::uint32_t>(best.max_dev - best.min_dev);
    c.anchor_ms = static_cast<std::uint32_t>(std::max<std::int64_t>(0, first + best.min_dev));
    std::vector<std::int64_t> slots = best.slots;
    std::sort(slots.begin(), slots.end());
    const auto distinct = static_cast<std::int64_t>(
      std::unique(slots.begin(), slots.end()) - slots.begin());
    c.missing_cycles = static_cast<std::uint32_t>(slots.back() + 1 - distinct);
    const auto latency_steps = static_cast<std::uint32_t>(
      (c.latency_p99_ns + config_.wall_step_ns - 1) / config_.wall_step_ns);
    const std::uint32_t allowed_jitter = std::max(config_.max_jitter_ms, latency_steps);
    c.scheduled = c.arrivals >= config_.min_arrivals && c.jitter_ms <= allowed_jitter;

    if (!c.scheduled && trace.size() >= 5) {
      std::vector<std::int64_t> wall_gaps;
      for (std::size_t i = 1; i < trace.size(); ++i) {
        wall_gaps.push_back(trace[i].wall_ns - trace[i - 1].wall_ns);
      }
      std::vector<std::int64_t> sorted_wall = wall_gaps;
      std::sort(sorted_wall.begin(), sorted_wall.end());
      const std::int64_t median = sorted_wall[sorted_wall.size() / 2];
      const std::int64_t spread = sorted_wall.back() - sorted_wall.front();
      c.wall_timer_suspect = median > 0 && spread * 4 <= median;
    }
    return c;
  }

  AdaptationConfig config_;
  std::map<std::uint32_t, std::vector<Arrival>> traces_;
};
}  // namespace asm_socketcan_bridge

#endif  // ASM_SOCKETCAN_BRIDGE__ADAPTATION_DETECTOR_H_
