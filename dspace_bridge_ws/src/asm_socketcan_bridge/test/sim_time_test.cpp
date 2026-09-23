#include "sim_time.h"

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
  int sensor_batch_count = 0;
  int marker_count = 0;
  int clock_publish_count = 0;
  uint64_t published_clock_milliseconds = 0;
  std::string event_order;
  const bool handshake_completed = asm_socketcan_bridge::runSimTimeHandshake(
    10,
    [&]() {
      ++step_count;
      event_order += 'S';
      time.advanceMilliseconds(1);
    },
    [&]() {
      ++sensor_batch_count;
      event_order += 'F';
    },
    [&]() {
      ++marker_count;
      event_order += 'M';
      return true;
    },
    [&]() {
      ++clock_publish_count;
      event_order += 'C';
      published_clock_milliseconds = time.totalMilliseconds();
    });
  if (!expect(handshake_completed, "marker allows handshake completion") ||
      !expect(step_count == 10, "ten V-ESI step callbacks") ||
      !expect(sensor_batch_count == 1, "one sensor CAN batch per handshake") ||
      !expect(marker_count == 1, "one step marker per handshake") ||
      !expect(clock_publish_count == 1, "one clock publication per handshake") ||
      !expect(event_order == std::string(10, 'S') + "FMC",
              "sensor frames and marker precede the clock publication") ||
      !expect(published_clock_milliseconds == 10, "clock published after ten steps") ||
      !expect(time.totalMilliseconds() == 10, "ten millisecond handshake") ||
      !expect(time.seconds() == 0, "ten millisecond seconds") ||
      !expect(time.nanoseconds() == 10000000, "ten millisecond nanoseconds")) {
    return 1;
  }

  int clock_after_failed_marker = 0;
  const bool failed_handshake_completed = asm_socketcan_bridge::runSimTimeHandshake(
    1,
    []() {},
    []() {},
    []() { return false; },
    [&]() { ++clock_after_failed_marker; });
  if (!expect(!failed_handshake_completed, "marker write failure aborts handshake") ||
      !expect(clock_after_failed_marker == 0, "clock is not published after marker failure")) {
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

  if (!expect(asm_socketcan_bridge::shouldCreateWallClockAcquisitionTimer(false),
              "wall mode creates acquisition timer") ||
      !expect(!asm_socketcan_bridge::shouldCreateWallClockAcquisitionTimer(true),
              "sim mode skips acquisition timer")) {
    return 1;
  }

  return 0;
}