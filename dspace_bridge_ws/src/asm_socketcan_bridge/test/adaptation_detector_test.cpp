#include "adaptation_detector.h"

#include <iostream>
#include <vector>

namespace
{
using asm_socketcan_bridge::AdaptationConfig;
using asm_socketcan_bridge::AdaptationDetector;
using asm_socketcan_bridge::DerivedSchedule;
using asm_socketcan_bridge::IdClassification;

constexpr std::int64_t kMs = 1'000'000;

bool expect(bool condition, const char *description)
{
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    return false;
  }
  return true;
}

// Feeds arrivals at start, start+period, ... with a wall spacing of one simulated ms per ms.
void feedPeriodic(
  AdaptationDetector &detector, std::uint32_t id, std::uint32_t period, std::uint32_t start,
  std::uint32_t until, std::int64_t latency_ns = kMs, std::uint32_t late_every_other = 0)
{
  std::uint32_t n = 0;
  for (std::uint32_t t = start; t <= until; t += period, ++n) {
    const std::uint32_t stamp = t + ((n % 2 == 1) ? late_every_other : 0);
    detector.onArrival(id, stamp, static_cast<std::int64_t>(stamp) * kMs, latency_ns);
  }
}

const IdClassification *find(const DerivedSchedule &derived, std::uint32_t id)
{
  for (const auto &c : derived.ids) {
    if (c.id == id) {
      return &c;
    }
  }
  return nullptr;
}

bool testReferenceStack()
{
  AdaptationDetector detector;
  for (const std::uint32_t id : {0x578u, 0x579u, 0x57Au, 0x57Bu}) {
    feedPeriodic(detector, id, 10, 10, 3000);
  }
  feedPeriodic(detector, 0x57C, 500, 501, 3000);
  const auto derived = detector.derive({10, 20, 100}, 3001);
  const auto *c578 = find(derived, 0x578);
  const auto *c57c = find(derived, 0x57C);
  return expect(derived.step_ms == 10, "reference stack derives a 10 ms step") &&
         expect(derived.schedule.size() == 5, "all five IDs are scheduled") &&
         expect(c578 && c578->scheduled && c578->period_ms == 10 && c578->jitter_ms == 0,
                "10 ms loop detected") &&
         expect(c578->first_due_ms == 3010, "first due is the first grid time after the window") &&
         expect(c57c && c57c->period_ms == 500 && c57c->first_due_ms == 3001,
                "500 ms timer keeps its phase");
}

bool testStampJitterAndPhase()
{
  AdaptationDetector detector;
  // The reaction lands in the step of the clock or in the next one; the earliest grid is kept.
  feedPeriodic(detector, 0x578, 10, 13, 400, kMs, 1);
  const auto derived = detector.derive({}, 0);
  const auto *c = find(derived, 0x578);
  return expect(c && c->scheduled && c->jitter_ms == 1, "one sub-step of stamp jitter is accepted") &&
         expect(c->anchor_ms == 13 && c->period_ms == 10, "anchor is the earliest arrival on the grid");
}

bool testMixedPeriodsAndStepDerivation()
{
  AdaptationDetector fast;
  feedPeriodic(fast, 0x578, 5, 5, 500);
  feedPeriodic(fast, 0x57A, 20, 20, 500);
  if (!expect(fast.derive({}, 501).step_ms == 5, "5 ms and 20 ms give a 5 ms step")) {
    return false;
  }

  AdaptationDetector mixed;
  feedPeriodic(mixed, 0x578, 7, 7, 700);
  feedPeriodic(mixed, 0x579, 10, 10, 700);
  const auto derived = mixed.derive({}, 701);
  if (!expect(derived.step_ms == 1, "7 ms and 10 ms give a 1 ms step") ||
      !expect(derived.schedule.size() == 2 && derived.schedule[0].period_ms == 7 &&
              derived.schedule[1].period_ms == 10, "each ID keeps its own period")) {
    return false;
  }

  AdaptationDetector outputs;
  feedPeriodic(outputs, 0x578, 10, 10, 500);
  const auto with_output = outputs.derive({7, 0, 10}, 501);
  return expect(with_output.step_ms == 1, "an enabled 7 ms output shrinks the step") &&
         expect(outputs.derive({0, 0}, 501).step_ms == 10, "disabled outputs do not constrain it");
}

bool testUnscheduledClassification()
{
  AdaptationDetector detector;
  detector.onArrival(0x57D, 100, 100 * kMs, kMs);
  detector.onArrival(0x57D, 900, 900 * kMs, kMs);
  // Irregular in simulation time, evenly spaced in wall time: reported as wall-timer driven.
  const std::uint32_t irregular[] = {10, 25, 30, 47, 52, 71, 76};
  std::int64_t wall = 0;
  for (const auto t : irregular) {
    detector.onArrival(0x57E, t, wall += 10 * kMs, kMs);
  }
  feedPeriodic(detector, 0x578, 10, 10, 200);
  const auto derived = detector.derive({}, 201);
  const auto *sparse = find(derived, 0x57D);
  const auto *wall_timer = find(derived, 0x57E);
  return expect(sparse && !sparse->scheduled, "an ID seen fewer than three times is unscheduled") &&
         expect(wall_timer && !wall_timer->scheduled && wall_timer->wall_timer_suspect,
                "large jitter is unscheduled and flagged as wall-timer driven") &&
         expect(derived.schedule.size() == 1 && derived.step_ms == 10,
                "unscheduled IDs do not constrain the step");
}

bool testJitterFollowsLatency()
{
  // Two steps of stamp spread are plausible when the reaction takes over one step of wall time.
  AdaptationDetector slow;
  feedPeriodic(slow, 0x578, 10, 10, 400, 1100 * 1000, 2);
  const auto slow_derived = slow.derive({}, 401);
  AdaptationDetector fast;
  feedPeriodic(fast, 0x578, 10, 10, 400, 400 * 1000, 2);
  const auto fast_derived = fast.derive({}, 401);
  return expect(find(slow_derived, 0x578)->scheduled, "spread explained by latency is accepted") &&
         expect(!find(fast_derived, 0x578)->scheduled, "the same spread with a fast reaction is not");
}

bool testMissingCycles()
{
  AdaptationDetector detector;
  for (const std::uint32_t t : {10u, 20u, 40u, 50u}) {
    detector.onArrival(0x578, t, t * kMs, kMs);
  }
  const auto derived = detector.derive({}, 0);
  const auto *c = find(derived, 0x578);
  return expect(c && c->scheduled && c->missing_cycles == 1, "a skipped cycle is counted");
}

bool testStepClampAndDefault()
{
  AdaptationDetector slow;
  feedPeriodic(slow, 0x578, 300, 300, 3000);
  if (!expect(slow.derive({}, 3001).step_ms == 100, "a 300 ms ID is split into 100 ms steps")) {
    return false;
  }
  AdaptationDetector coprime;
  feedPeriodic(coprime, 0x578, 250, 250, 3000);
  if (!expect(coprime.derive({}, 3001).step_ms == 50, "a 250 ms ID is split into 50 ms steps")) {
    return false;
  }
  AdaptationDetector empty;
  if (!expect(empty.derive({}, 0).step_ms == 10, "nothing constrains the step: default")) {
    return false;
  }
  return expect(empty.derive({20, 50}, 0).step_ms == 10, "output intervals alone set the step");
}
}  // namespace

int main()
{
  if (!testReferenceStack() || !testStampJitterAndPhase() || !testMixedPeriodsAndStepDerivation() ||
      !testUnscheduledClassification() || !testJitterFollowsLatency() || !testMissingCycles() ||
      !testStepClampAndDefault()) {
    return 1;
  }
  std::cout << "adaptation_detector_test passed\n";
  return 0;
}
