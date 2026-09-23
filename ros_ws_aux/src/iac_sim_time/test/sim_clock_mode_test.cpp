#include "iac_sim_time/sim_clock_mode.hpp"
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

    if (!expect(!iac_sim_time::parse_sim_clock_mode("maybe").has_value(),
          "invalid sim clock value is rejected") ||
        !expect(!iac_sim_time::parse_sim_clock_mode(nullptr).has_value(),
          "missing sim clock value is rejected")) {
      return 1;
    }

    const std::uint64_t encoded_step = 0x0102030405060708ULL;
    const auto payload = iac_sim_time::encodeSimStepMarkerPayload(encoded_step);
    const iac_sim_time::SimStepMarkerPayload expected_payload{
      0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U};
    if (!expect(payload == expected_payload, "step marker payload is little-endian") ||
        !expect(iac_sim_time::decodeSimStepMarkerPayload(payload.data(), payload.size()) == encoded_step,
          "step marker payload decodes to its logical step") ||
        !expect(!iac_sim_time::decodeSimStepMarkerPayload(payload.data(), payload.size() - 1U),
          "short step marker payload is rejected") ||
        !expect(!iac_sim_time::decodeSimStepMarkerPayload(nullptr, payload.size()),
          "null step marker payload is rejected")) {
      return 1;
    }

    iac_sim_time::SimStepMarkerSequence sequence;
    if (!expect(sequence.accept(1) == iac_sim_time::SimStepMarkerResult::accepted,
          "first step marker is accepted") ||
        !expect(sequence.accept(2) == iac_sim_time::SimStepMarkerResult::accepted,
          "consecutive step marker is accepted") ||
        !expect(sequence.accept(5) == iac_sim_time::SimStepMarkerResult::accepted,
          "forward step marker is accepted") ||
        !expect(sequence.gapCount() == 2, "missing logical steps are counted") ||
        !expect(sequence.accept(4) == iac_sim_time::SimStepMarkerResult::non_monotonic,
          "backward step marker is rejected") ||
        !expect(sequence.lastStep() == 5, "rejected marker does not move the sequence") ||
        !expect(sequence.nonMonotonicCount() == 1, "non-monotonic markers are counted")) {
      return 1;
    }

    return expect(iac_sim_time::kSimStepMarkerCanId == 0x7FFU,
      "sim step marker uses its dedicated standard CAN ID") ? 0 : 1;
}