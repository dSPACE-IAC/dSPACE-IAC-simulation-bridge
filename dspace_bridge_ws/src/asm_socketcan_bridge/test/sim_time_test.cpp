#include "sim_time.h"

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
}  // namespace

int main()
{
  asm_socketcan_bridge::SimTime time;
  if (!expect(time.totalMilliseconds() == 0, "initial total time") ||
      !expect(time.seconds() == 0, "initial seconds") ||
      !expect(time.nanoseconds() == 0, "initial nanoseconds")) {
    return 1;
  }

  int step_count = 0;
  int clock_publish_count = 0;
  uint64_t published_clock_milliseconds = 0;
  asm_socketcan_bridge::runSimTimeHandshake(
    10,
    [&]() {
      ++step_count;
      time.advanceMilliseconds(1);
    },
    [&]() {
      ++clock_publish_count;
      published_clock_milliseconds = time.totalMilliseconds();
    });
  if (!expect(step_count == 10, "ten V-ESI step callbacks") ||
      !expect(clock_publish_count == 1, "one clock publication per handshake") ||
      !expect(published_clock_milliseconds == 10, "clock published after ten steps") ||
      !expect(time.totalMilliseconds() == 10, "ten millisecond handshake") ||
      !expect(time.seconds() == 0, "ten millisecond seconds") ||
      !expect(time.nanoseconds() == 10000000, "ten millisecond nanoseconds")) {
    return 1;
  }

  time.advanceMilliseconds(990);
  if (!expect(time.totalMilliseconds() == 1000, "exact second total") ||
      !expect(time.seconds() == 1, "exact second seconds") ||
      !expect(time.nanoseconds() == 0, "exact second nanoseconds")) {
    return 1;
  }

  time.advanceMilliseconds(10);
  if (!expect(time.totalMilliseconds() == 1010, "post-boundary total") ||
      !expect(time.seconds() == 1, "post-boundary seconds") ||
      !expect(time.nanoseconds() == 10000000, "post-boundary nanoseconds")) {
    return 1;
  }

  return 0;
}