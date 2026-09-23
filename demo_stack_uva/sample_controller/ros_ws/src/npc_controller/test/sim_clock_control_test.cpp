#include "sim_clock_control.h"

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
  if (!expect(controller::shouldCreatePeriodicControlTimers(false),
              "wall mode creates periodic control timers") ||
      !expect(!controller::shouldCreatePeriodicControlTimers(true),
              "sim mode skips periodic control timers") ||
      !expect(!controller::shouldRunSimTimeControl(0, 0), "zero clock does not run control") ||
      !expect(controller::shouldRunSimTimeControl(0, 1000000),
              "first nonzero nanosecond runs control") ||
      !expect(controller::shouldRunSimTimeControl(1, 0),
              "exact whole second runs control")) {
    return 1;
  }

  int control_invocations = 0;
  int handshakes = 0;
  std::string event_order;
  const auto process_clock = [&](int32_t seconds, uint32_t nanoseconds) {
    return controller::runSimTimeControlStep(
      seconds,
      nanoseconds,
      [&]() {
        ++control_invocations;
        event_order += 'C';
      },
      [&]() {
        ++handshakes;
        event_order += 'H';
      });
  };

  if (!expect(!process_clock(0, 0), "zero clock skips control callback") ||
      !expect(process_clock(0, 1000000), "nonzero clock runs control callback") ||
      !expect(process_clock(1, 0), "whole-second clock runs control callback") ||
      !expect(control_invocations == 2, "one control callback per nonzero clock") ||
      !expect(handshakes == 3, "one handshake per clock message") ||
      !expect(event_order == "HCHCH", "control runs before each nonzero-clock handshake")) {
    return 1;
  }

  return 0;
}