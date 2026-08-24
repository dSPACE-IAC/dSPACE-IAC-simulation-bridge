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

  time.advanceMilliseconds(1);
  if (!expect(time.totalMilliseconds() == 1, "first millisecond total") ||
      !expect(time.seconds() == 0, "first millisecond seconds") ||
      !expect(time.nanoseconds() == 1000000, "first millisecond nanoseconds")) {
    return 1;
  }

  for (int step = 0; step < 9; ++step) {
    time.advanceMilliseconds(1);
  }
  if (!expect(time.totalMilliseconds() == 10, "ten millisecond handshake") ||
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