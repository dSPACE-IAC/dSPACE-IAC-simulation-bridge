#include "iac_sim_time/sim_clock_mode.hpp"

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
  const char *true_values[] = {"true", "TRUE", "True", "1", "yes", "YES", "on", "ON"};
  for (const char *value : true_values) {
    const auto parsed = iac_sim_time::parse_sim_clock_mode(value);
    if (!expect(parsed.has_value() && parsed.value(), "true sim clock value parses as true")) {
      return 1;
    }
  }

  const char *false_values[] = {"false", "FALSE", "False", "0", "no", "NO", "off", "OFF"};
  for (const char *value : false_values) {
    const auto parsed = iac_sim_time::parse_sim_clock_mode(value);
    if (!expect(parsed.has_value() && !parsed.value(), "false sim clock value parses as false")) {
      return 1;
    }
  }

  return expect(!iac_sim_time::parse_sim_clock_mode("maybe").has_value(),
                "invalid sim clock value is rejected") &&
           expect(!iac_sim_time::parse_sim_clock_mode(nullptr).has_value(),
                  "missing sim clock value is rejected") ? 0 : 1;
}