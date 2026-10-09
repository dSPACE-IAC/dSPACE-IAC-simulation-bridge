#include "output_schedule.h"

#include <iostream>
#include <string>

namespace
{
bool expect(bool condition, const char *description)
{
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    return false;
  }
  return true;
}

// Fire times per output.
struct Trace
{
  std::vector<std::uint64_t> fast;
  std::vector<std::uint64_t> slow;
};
}  // namespace

int main()
{
  using asm_socketcan_bridge::normalizeOutputInterval;
  using asm_socketcan_bridge::OutputSchedule;

  if (!expect(normalizeOutputInterval(0).interval_ms == 0 && normalizeOutputInterval(0).valid,
              "0 disables an output") ||
      !expect(normalizeOutputInterval(1).interval_ms == 1 && normalizeOutputInterval(1).valid,
              "1 ms accepted") ||
      !expect(normalizeOutputInterval(1000).interval_ms == 1000 && normalizeOutputInterval(1000).valid,
              "1000 ms accepted") ||
      !expect(normalizeOutputInterval(1001).interval_ms == 10 && !normalizeOutputInterval(1001).valid,
              "above range falls back to default") ||
      !expect(normalizeOutputInterval(-5).interval_ms == 10 && !normalizeOutputInterval(-5).valid,
              "negative falls back to default")) {
    return 1;
  }

  // 10 ms and 20 ms outputs on a 10 ms step, due at t=0.
  {
    Trace trace;
    std::string order;
    OutputSchedule schedule;
    schedule.add("fast", 10, [&]() { trace.fast.push_back(0); order += 'f'; });
    schedule.add("slow", 20, [&]() { trace.slow.push_back(0); order += 's'; });
    schedule.add("disabled", 0, [&]() { order += 'x'; });
    if (!expect(schedule.size() == 2, "disabled output is not registered")) {
      return 1;
    }
    for (std::uint64_t t = 0; t <= 60; t += 10) {
      schedule.runDue(t);
    }
    if (!expect(trace.fast.size() == 7, "10 ms output runs every step including t=0") ||
        !expect(trace.slow.size() == 4, "20 ms output runs every second step including t=0") ||
        !expect(order.substr(0, 3) == "fsf", "registration order is kept, slow skipped at t=10") ||
        !expect(order.find('x') == std::string::npos, "disabled output never runs")) {
      return 1;
    }
  }

  // 30 ms interval on a 20 ms step: late samples, exact average rate.
  {
    std::vector<std::uint64_t> fired;
    std::uint64_t now = 0;
    OutputSchedule schedule;
    schedule.add("odd", 30, [&]() { fired.push_back(now); });
    for (now = 0; now <= 120; now += 20) {
      schedule.runDue(now);
    }
    const std::vector<std::uint64_t> expected = {0, 40, 60, 100, 120};
    if (!expect(fired == expected, "non-divisible interval is late by less than one step")) {
      return 1;
    }
  }

  // Interval shorter than the step: once per step, reported by namesShorterThan.
  {
    int runs = 0;
    OutputSchedule schedule;
    schedule.add("short", 3, [&]() { ++runs; });
    schedule.add("exact", 10, []() {});
    for (std::uint64_t t = 0; t <= 40; t += 10) {
      schedule.runDue(t);
    }
    const auto shorter = schedule.namesShorterThan(10);
    if (!expect(runs == 5, "interval shorter than the step runs once per step") ||
        !expect(shorter.size() == 1 && shorter[0] == "short", "short output is reported")) {
      return 1;
    }
  }

  return 0;
}
