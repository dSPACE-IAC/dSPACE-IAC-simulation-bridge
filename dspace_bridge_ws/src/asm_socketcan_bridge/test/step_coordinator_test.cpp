#include "step_coordinator.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using asm_socketcan_bridge::CommandFrame;
using asm_socketcan_bridge::CommandSchedule;
using asm_socketcan_bridge::FrameDisposition;
using asm_socketcan_bridge::StepCoordinator;
using asm_socketcan_bridge::StepState;

constexpr std::int64_t kMs = 1'000'000;

bool expect(bool condition, const char *description)
{
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    return false;
  }
  return true;
}

CommandFrame counterFrame(std::uint32_t id, std::uint8_t counter, std::uint8_t value = 0)
{
  CommandFrame frame;
  frame.id = id;
  frame.dlc = 8;
  frame.data[0] = value;
  frame.has_counter = true;
  frame.counter = counter;
  return frame;
}

CommandFrame plainFrame(std::uint32_t id, std::uint8_t value = 0)
{
  CommandFrame frame;
  frame.id = id;
  frame.dlc = 8;
  frame.data[0] = value;
  return frame;
}

StepCoordinator::Config threeIds(std::uint32_t step_ms)
{
  StepCoordinator::Config config;
  config.step_ms = step_ms;
  config.timeout_ns = 20 * kMs;
  config.schedule = {{0x578, 10, step_ms}, {0x579, 10, step_ms}, {0x57A, 10, step_ms}};
  return config;
}

// Runs beginStep..release and returns the release time.
std::int64_t startStep(StepCoordinator &coordinator, std::int64_t release_ns)
{
  coordinator.beginStep();
  coordinator.markPublishing();
  coordinator.release(release_ns);
  return release_ns;
}

bool isDue(const StepCoordinator &coordinator, std::uint32_t id)
{
  const auto &due = coordinator.record().due_ids;
  return std::find(due.begin(), due.end(), id) != due.end();
}

bool testCompleteStep()
{
  StepCoordinator coordinator(threeIds(10));
  const auto start = coordinator.beginStep();
  if (!expect(start.step == 1 && start.t_ms == 10, "first step is 1 at t=10 ms") ||
      !expect(start.snapshot.empty() && !start.finalized.has_value(), "no snapshot before any frame") ||
      !expect(coordinator.state() == StepState::kStepping, "STEPPING after beginStep")) {
    return false;
  }
  coordinator.markPublishing();
  coordinator.release(100 * kMs);
  if (!expect(coordinator.state() == StepState::kReleased, "RELEASED after release") ||
      !expect(coordinator.record().due_ids.size() == 3, "three IDs due")) {
    return false;
  }
  coordinator.onFrame(counterFrame(0x578, 1), 101 * kMs);
  coordinator.onFrame(counterFrame(0x579, 1), 102 * kMs);
  if (!expect(coordinator.state() == StepState::kReleased, "open until every due ID arrived")) {
    return false;
  }
  coordinator.onFrame(counterFrame(0x57A, 1), 104 * kMs);
  const auto &record = coordinator.record();
  if (!expect(coordinator.state() == StepState::kClosed, "closed on the last due ID") ||
      !expect(!record.timed_out && record.missing_ids.empty(), "complete step is not timed out") ||
      !expect(record.close_ns == 104 * kMs, "close time is the last arrival") ||
      !expect(record.arrivals.size() == 3 && record.arrivals[2].latency_ns == 4 * kMs,
              "latency measured from release") ||
      !expect(coordinator.totals().steps_closed == 1 && coordinator.totals().timeouts == 0,
              "totals count one closed step")) {
    return false;
  }
  // Polling a closed step never changes it.
  return expect(coordinator.poll(1000 * kMs) && !coordinator.record().timed_out,
                "poll after close keeps the result");
}

bool testTimeoutHoldsLast()
{
  StepCoordinator coordinator(threeIds(10));
  startStep(coordinator, 0);
  coordinator.onFrame(counterFrame(0x578, 1, 11), kMs);
  coordinator.onFrame(counterFrame(0x579, 1, 12), kMs);
  coordinator.onFrame(counterFrame(0x57A, 1, 13), kMs);

  startStep(coordinator, 100 * kMs);
  coordinator.onFrame(counterFrame(0x578, 2, 21), 101 * kMs);
  coordinator.onFrame(counterFrame(0x579, 2, 22), 101 * kMs);
  if (!expect(!coordinator.poll(119 * kMs), "still waiting before the deadline") ||
      !expect(coordinator.deadlineNs() == 120 * kMs, "deadline is release plus timeout") ||
      !expect(coordinator.poll(120 * kMs), "closes at the deadline")) {
    return false;
  }
  const auto &record = coordinator.record();
  if (!expect(record.timed_out, "step marked timed out") ||
      !expect(record.missing_ids == std::vector<std::uint32_t>{0x57A}, "missing ID reported") ||
      !expect(coordinator.totals().timeouts == 1 &&
              coordinator.totals().consecutive_timeouts == 1, "timeout counted")) {
    return false;
  }

  const auto next = coordinator.beginStep();
  if (!expect(next.finalized.has_value() && next.finalized->step == 2, "previous record handed over") ||
      !expect(next.snapshot.size() == 3, "snapshot holds every ID")) {
    return false;
  }
  bool ok = true;
  for (const auto &entry : next.snapshot) {
    if (entry.frame.id == 0x57A) {
      ok = ok && entry.frame.data[0] == 13 && !entry.fresh;
    } else {
      ok = ok && entry.fresh;
    }
  }
  if (!expect(ok, "missing ID holds its last accepted value and is not fresh")) {
    return false;
  }
  coordinator.markPublishing();
  coordinator.release(200 * kMs);
  coordinator.poll(300 * kMs);
  return expect(coordinator.totals().consecutive_timeouts == 2 &&
                coordinator.totals().max_consecutive_timeouts == 2, "consecutive timeouts counted");
}

bool testDuplicateCounter()
{
  StepCoordinator coordinator(threeIds(10));
  startStep(coordinator, 0);
  coordinator.onFrame(counterFrame(0x578, 5), kMs);
  coordinator.onFrame(counterFrame(0x579, 5), kMs);
  coordinator.onFrame(counterFrame(0x57A, 5), kMs);

  startStep(coordinator, 100 * kMs);
  if (!expect(coordinator.onFrame(counterFrame(0x578, 5), 101 * kMs) == FrameDisposition::kDuplicate,
              "repeated counter is a duplicate") ||
      !expect(coordinator.onFrame(counterFrame(0x579, 6), 101 * kMs) == FrameDisposition::kAccepted,
              "new counter is accepted") ||
      !expect(coordinator.record().duplicates == 1 && coordinator.totals().duplicates == 1,
              "duplicate counted") ||
      !expect(coordinator.record().missing_ids.empty() && coordinator.state() == StepState::kReleased,
              "duplicate does not satisfy a due ID")) {
    return false;
  }
  // IDs without a counter: every frame is fresh.
  StepCoordinator plain([]() {
    StepCoordinator::Config config;
    config.schedule = {{0x57B, 10, 10}};
    return config;
  }());
  startStep(plain, 0);
  if (!expect(plain.onFrame(plainFrame(0x57B, 1), kMs) == FrameDisposition::kAccepted,
              "counterless frame accepted")) {
    return false;
  }
  startStep(plain, 10 * kMs);
  return expect(plain.onFrame(plainFrame(0x57B, 1), 11 * kMs) == FrameDisposition::kAccepted,
                "identical counterless frame is still fresh");
}

bool testLateAndEarly()
{
  StepCoordinator coordinator(threeIds(10));
  if (!expect(coordinator.onFrame(counterFrame(0x578, 1), 0) == FrameDisposition::kLate,
              "frame before the first step is late")) {
    return false;
  }
  coordinator.beginStep();
  if (!expect(coordinator.onFrame(counterFrame(0x578, 1), 0) == FrameDisposition::kLate,
              "frame during STEPPING is not accepted")) {
    return false;
  }
  coordinator.markPublishing();
  coordinator.onFrame(counterFrame(0x578, 1), 0);
  coordinator.release(10 * kMs);
  coordinator.onFrame(counterFrame(0x578, 2, 1), 11 * kMs);
  coordinator.onFrame(counterFrame(0x579, 2, 2), 11 * kMs);
  coordinator.onFrame(counterFrame(0x57A, 2, 3), 11 * kMs);
  if (!expect(coordinator.record().early == 2, "frames before release counted as early")) {
    return false;
  }
  // Late frame after close is counted and never merged into the next snapshot.
  if (!expect(coordinator.onFrame(counterFrame(0x578, 3, 99), 12 * kMs) == FrameDisposition::kLate,
              "frame after close is late") ||
      !expect(coordinator.record().late == 1 && coordinator.totals().late == 2, "late counted")) {
    return false;
  }
  const auto next = coordinator.beginStep();
  for (const auto &entry : next.snapshot) {
    if (entry.frame.id == 0x578 && !expect(entry.frame.data[0] == 1, "late frame not merged")) {
      return false;
    }
  }
  if (!expect(next.finalized.has_value() && next.finalized->late == 1, "finalized record keeps late count")) {
    return false;
  }
  coordinator.markPublishing();
  coordinator.release(20 * kMs);
  // A frame after release of the next step is fresh again.
  return expect(coordinator.onFrame(counterFrame(0x578, 3, 99), 21 * kMs) == FrameDisposition::kAccepted,
                "frame after the next release is accepted");
}

bool testMultiRateSchedule()
{
  StepCoordinator::Config config;
  config.step_ms = 10;
  config.schedule = {{0x578, 10, 10}, {0x57A, 20, 20}};
  StepCoordinator coordinator(config);
  std::vector<std::size_t> due_counts;
  std::int64_t now = 0;
  for (int n = 0; n < 4; ++n) {
    startStep(coordinator, now);
    due_counts.push_back(coordinator.record().due_ids.size());
    const auto counter = static_cast<std::uint8_t>(n);
    coordinator.onFrame(counterFrame(0x578, counter), now + kMs);
    if (isDue(coordinator, 0x57A)) {
      coordinator.onFrame(counterFrame(0x57A, counter), now + kMs);
    }
    if (!expect(coordinator.state() == StepState::kClosed, "multi-rate step closes")) {
      return false;
    }
    now += 100 * kMs;
  }
  return expect(due_counts == std::vector<std::size_t>{1, 2, 1, 2},
                "20 ms ID is due on every second 10 ms step");
}

bool testStepDurations()
{
  StepCoordinator::Config config;
  config.step_ms = 1;
  config.accept_outside_release = true;
  StepCoordinator coordinator(config);
  for (int i = 0; i < 3; ++i) {
    startStep(coordinator, 0);
  }
  if (!expect(coordinator.timeMs() == 3, "1 ms steps accumulate")) {
    return false;
  }
  config.step_ms = 10;
  coordinator.reconfigure(config);
  startStep(coordinator, 0);
  config.step_ms = 100;
  coordinator.reconfigure(config);
  startStep(coordinator, 0);
  if (!expect(coordinator.timeMs() == 113 && coordinator.step() == 5,
              "time stays continuous over step changes")) {
    return false;
  }
  return expect(coordinator.record().t_ms == 113, "record carries the step time");
}

bool testNoDueAndUnscheduled()
{
  StepCoordinator::Config config;
  config.step_ms = 1;
  config.schedule = {{0x578, 10, 10}};
  StepCoordinator coordinator(config);
  startStep(coordinator, 0);
  if (!expect(coordinator.state() == StepState::kClosed && !coordinator.record().timed_out,
              "step with no due ID closes at release")) {
    return false;
  }
  // Unscheduled IDs are latched when present in the release window but never awaited.
  StepCoordinator awaiting(threeIds(10));
  startStep(awaiting, 0);
  awaiting.onFrame(plainFrame(0x57B, 7), kMs);
  if (!expect(awaiting.state() == StepState::kReleased, "unscheduled ID does not close a step")) {
    return false;
  }
  awaiting.onFrame(counterFrame(0x578, 1), kMs);
  awaiting.onFrame(counterFrame(0x579, 1), kMs);
  awaiting.onFrame(counterFrame(0x57A, 1), kMs);
  const auto next = awaiting.beginStep();
  bool found = false;
  for (const auto &entry : next.snapshot) {
    found = found || (entry.frame.id == 0x57B && entry.fresh && entry.frame.data[0] == 7);
  }
  if (!expect(found, "unscheduled ID is latched")) {
    return false;
  }
  // Adaptation window: frames are accepted in every state.
  StepCoordinator::Config adapting;
  adapting.step_ms = 1;
  adapting.accept_outside_release = true;
  StepCoordinator adapt(adapting);
  adapt.beginStep();
  return expect(adapt.onFrame(plainFrame(0x57A, 1), 0) == FrameDisposition::kAccepted,
                "adaptation accepts frames while stepping");
}

// A frame just after a step that did not wait for its ID moves the ID onto that step's grid.
bool testRephaseLateFrame()
{
  StepCoordinator::Config config;
  config.step_ms = 1;
  config.schedule = {{0x578, 10, 11}};  // detected one step late
  StepCoordinator coordinator(config);
  std::uint8_t counter = 0;
  bool phase_ok = false;
  for (int step = 1; step <= 30; ++step) {
    startStep(coordinator, step * kMs);
    if (coordinator.timeMs() % 10 == 0 && coordinator.state() == StepState::kClosed) {
      // The stack reacts to clock 10, 20, ...; the coordinator does not wait, so the frame is late.
      if (!expect(coordinator.onFrame(counterFrame(0x578, ++counter), step * kMs) ==
                  FrameDisposition::kLate, "frame after an unawaited step is late")) {
        return false;
      }
    } else if (isDue(coordinator, 0x578)) {
      phase_ok = phase_ok || coordinator.timeMs() == 20;
      coordinator.onFrame(counterFrame(0x578, ++counter), step * kMs);
    }
  }
  return expect(phase_ok && coordinator.totals().rephased >= 1 && coordinator.totals().timeouts == 0,
                "late frame re-anchors the ID to the grid of its arrival step");
}

// A fresh frame in a release window for which the ID was not due also re-anchors it.
bool testRephaseInWindowFrame()
{
  StepCoordinator::Config config;
  config.step_ms = 10;
  config.schedule = {{0x578, 10, 10}, {0x57C, 500, 500}};  // the stack really sends at 510
  StepCoordinator coordinator(config);
  std::uint8_t counter = 0;
  std::vector<std::uint64_t> due_times;
  for (int step = 1; step <= 110; ++step) {
    startStep(coordinator, step * kMs);
    if (isDue(coordinator, 0x57C)) {
      due_times.push_back(coordinator.timeMs());
    }
    if (coordinator.timeMs() == 510 || coordinator.timeMs() == 1010) {
      coordinator.onFrame(counterFrame(0x57C, ++counter), step * kMs);
    }
    coordinator.onFrame(counterFrame(0x578, ++counter), step * kMs);
    coordinator.poll(step * kMs + 30 * kMs);
  }
  // Due at 500 (timeout, the stack is late), then aligned to the stack's 510, 1010.
  return expect(due_times == std::vector<std::uint64_t>{500, 1010},
                "in-window frame re-anchors the ID") &&
         expect(coordinator.totals().timeouts == 1, "only the mis-phased first cycle times out");
}

bool testRephaseIgnoresDuplicatesAndCanBeDisabled()
{
  StepCoordinator::Config config;
  config.step_ms = 10;
  config.schedule = {{0x578, 10, 10}, {0x57C, 500, 500}};
  StepCoordinator coordinator(config);
  startStep(coordinator, 0);
  coordinator.onFrame(counterFrame(0x57C, 1), 1);  // accepted while not due: re-anchors
  coordinator.onFrame(counterFrame(0x578, 1), 1);
  const auto after_first = coordinator.totals().rephased;
  startStep(coordinator, 10 * kMs);
  coordinator.onFrame(counterFrame(0x57C, 1), 10 * kMs + 1);  // repeated counter: duplicate
  coordinator.onFrame(counterFrame(0x578, 2), 10 * kMs + 1);
  if (!expect(after_first == 1 && coordinator.totals().rephased == 1,
              "a duplicate does not re-anchor")) {
    return false;
  }
  config.rephase = false;
  StepCoordinator fixed(config);
  startStep(fixed, 0);
  fixed.onFrame(counterFrame(0x57C, 1), 1);
  fixed.onFrame(counterFrame(0x578, 1), 1);
  return expect(fixed.totals().rephased == 0, "re-anchoring can be disabled");
}

bool testMisuse()
{
  StepCoordinator coordinator(threeIds(10));
  bool threw = false;
  try {
    coordinator.release(0);
  } catch (const std::logic_error &) {
    threw = true;
  }
  if (!expect(threw, "release before beginStep throws")) {
    return false;
  }
  coordinator.beginStep();
  threw = false;
  try {
    coordinator.beginStep();
  } catch (const std::logic_error &) {
    threw = true;
  }
  if (!expect(threw, "beginStep during a step throws")) {
    return false;
  }
  threw = false;
  try {
    StepCoordinator::Config config;
    config.step_ms = 0;
    StepCoordinator bad(config);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  return expect(threw, "zero step duration is rejected");
}
}  // namespace

int main()
{
  if (!testCompleteStep() || !testTimeoutHoldsLast() || !testDuplicateCounter() ||
      !testLateAndEarly() || !testMultiRateSchedule() || !testStepDurations() ||
      !testNoDueAndUnscheduled() || !testRephaseLateFrame() || !testRephaseInWindowFrame() ||
      !testRephaseIgnoresDuplicatesAndCanBeDisabled() || !testMisuse()) {
    return 1;
  }
  std::cout << "step_coordinator_test passed\n";
  return 0;
}
