#include "iac_sim_time/sim_step_marker.hpp"

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
    const std::uint64_t encoded_step = 0x0102030405060708ULL;
    const auto payload = iac_sim_time::encodeSimStepMarkerPayload(encoded_step);
    const iac_sim_time::SimStepMarkerPayload expected_payload{
      0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U};
    if (!expect(payload == expected_payload, "step marker payload is little-endian")) {
      return 1;
    }

    return expect(iac_sim_time::kSimStepMarkerCanId == 0x7FFU,
      "sim step marker uses its dedicated standard CAN ID") ? 0 : 1;
}