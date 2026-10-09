#include "command_latch.h"

#include <iostream>

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

struct Command
{
  double value;
  unsigned char counter;
};
}  // namespace

int main()
{
  using asm_socketcan_bridge::CommandLatch;
  using asm_socketcan_bridge::StaleCounterTracker;

  Command live{1.0, 1};
  CommandLatch<Command> latch;
  if (!expect(!latch.hasSnapshot(), "no snapshot before first latch")) {
    return 1;
  }
  latch.latch(live);
  live.value = 2.0;
  live.counter = 2;
  if (!expect(latch.hasSnapshot(), "snapshot after latch") ||
      !expect(latch.snapshot().value == 1.0 && latch.snapshot().counter == 1,
              "mutation of the live command does not change the snapshot")) {
    return 1;
  }
  latch.latch(live);
  if (!expect(latch.snapshot().value == 2.0, "next latch takes the new command")) {
    return 1;
  }

  StaleCounterTracker<2> tracker;
  if (!expect(!tracker.observe(0, 5), "first observation is never stale") ||
      !expect(!tracker.observe(0, 6), "advanced counter is fresh") ||
      !expect(tracker.observe(0, 6), "repeated counter is stale") ||
      !expect(tracker.observe(0, 6), "repeated counter stays stale") ||
      !expect(!tracker.observe(0, 7), "advance after stale is fresh") ||
      !expect(!tracker.observe(0, 0), "wrapped counter is fresh") ||
      !expect(tracker.observations(0) == 6, "observation count") ||
      !expect(tracker.staleCount(0) == 2, "stale count")) {
    return 1;
  }
  if (!expect(!tracker.observe(1, 6), "slots are independent") ||
      !expect(tracker.staleCount(1) == 0, "other slot has no stale copies")) {
    return 1;
  }

  return 0;
}
