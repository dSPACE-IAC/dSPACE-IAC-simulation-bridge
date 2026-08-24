#ifndef IAC_SIM_TIME__SIM_CLOCK_MODE_HPP_
#define IAC_SIM_TIME__SIM_CLOCK_MODE_HPP_

#include <cctype>
#include <optional>
#include <string>

namespace iac_sim_time
{
inline std::optional<bool> parse_sim_clock_mode(const char *value)
{
  if (value == nullptr) {
    return std::nullopt;
  }

  std::string normalized(value);
  for (char &character : normalized) {
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  }

  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    return false;
  }
  return std::nullopt;
}
}  // namespace iac_sim_time

#endif  // IAC_SIM_TIME__SIM_CLOCK_MODE_HPP_