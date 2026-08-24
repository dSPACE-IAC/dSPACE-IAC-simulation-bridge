#include "sim_clock_control.h"

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
  if (!expect(!controller::shouldRunSimTimeControl(0, 0), "zero clock does not run control") ||
      !expect(controller::shouldRunSimTimeControl(0, 1000000),
              "first nonzero nanosecond runs control") ||
      !expect(controller::shouldRunSimTimeControl(1, 0),
              "exact whole second runs control")) {
    return 1;
  }

  return 0;
}