#include "command_replay.h"

#include <iostream>
#include <sstream>

namespace
{
bool expect(bool condition, const char * description)
{
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    return false;
  }
  return true;
}

using namespace asm_socketcan_bridge;

SnapshotEntry entry(std::uint32_t id, std::uint8_t dlc, std::uint8_t first, bool fresh)
{
  SnapshotEntry snapshot;
  snapshot.frame.id = id;
  snapshot.frame.dlc = dlc;
  snapshot.frame.data = {first, 0x00, 0xFF, 0x10, 0x20, 0x30, 0x40, 0x7F};
  snapshot.fresh = fresh;
  return snapshot;
}

bool load(const std::string & text, std::vector<ReplayStep> & steps, std::string & error)
{
  std::istringstream input(text);
  return loadReplay(input, steps, error);
}
}  // namespace

int main()
{
  const std::vector<SnapshotEntry> snapshot = {
    entry(1400, 8, 0xAB, true), entry(1401, 4, 0x01, false), entry(1404, 8, 0x02, true)};
  const auto row = formatReplayRow(7, 12, snapshot);
  if (!expect(row == "7,12,1400:8:AB00FF102030407F|1404:8:0200FF102030407F",
      "row lists fresh frames only") ||
    !expect(formatReplayRow(2, 2, {}) == "2,2,", "a step without frames has an empty column"))
  {
    return 1;
  }

  const auto parsed = parseReplayRow(row);
  if (!expect(parsed && parsed->step == 7 && parsed->t_ms == 12 && parsed->frames.size() == 2,
      "row parses back") ||
    !expect(parsed->frames[0].id == 1400 && parsed->frames[0].dlc == 8 &&
      parsed->frames[0].data == snapshot[0].frame.data, "payload round-trips") ||
    !expect(parsed->frames[1].id == 1404, "order is kept"))
  {
    return 1;
  }

  const std::string valid =
    std::string(replayHeader()) + "\n1,1,\n2,2," + "1400:8:0000000000000000\n3,10,\n";
  std::vector<ReplayStep> steps;
  std::string error;
  if (!expect(load(valid, steps, error), "valid file loads") ||
    !expect(steps.size() == 3 && steps[1].frames.size() == 1 && steps[2].t_ms == 10,
      "loaded content, including a step-size change"))
  {
    return 1;
  }

  if (!expect(!load("", steps, error), "empty file is rejected") ||
    !expect(!load(std::string(replayHeader()) + "\n", steps, error), "header only is rejected") ||
    !expect(!load("1,1,\n", steps, error), "missing header is rejected") ||
    !expect(!load(std::string(replayHeader()) + "\n2,1,\n", steps, error), "first step must be 1") ||
    !expect(!load(std::string(replayHeader()) + "\n1,1,\n1,2,\n", steps, error),
      "step numbers must be contiguous") ||
    !expect(!load(std::string(replayHeader()) + "\n1,5,\n2,5,\n", steps, error),
      "times must rise") ||
    !expect(!load(std::string(replayHeader()) + "\n1,0,\n", steps, error), "time zero is rejected") ||
    !expect(!load(std::string(replayHeader()) + "\n1,1,1400:8:00\n", steps, error),
      "short payload is rejected") ||
    !expect(!load(std::string(replayHeader()) + "\n1,1,4000:8:0000000000000000\n", steps, error),
      "non-standard ID is rejected") ||
    !expect(!load(std::string(replayHeader()) + "\n1,1,1400:9:0000000000000000\n", steps, error),
      "DLC above 8 is rejected") ||
    !expect(!load(std::string(replayHeader()) + "\n1,x,\n", steps, error), "non-numeric time is rejected"))
  {
    return 1;
  }
  if (!expect(error.find("no recorded steps") != std::string::npos ||
      error.find("line") != std::string::npos, "errors describe the failure"))
  {
    return 1;
  }

  std::istringstream crlf(std::string(replayHeader()) + "\r\n1,1,\r\n");
  if (!expect(loadReplay(crlf, steps, error) && steps.size() == 1, "CRLF line ends are accepted")) {
    return 1;
  }

  const auto make = [](std::initializer_list<std::uint64_t> times) {
      std::vector<ReplayStep> result;
      for (const auto t : times) {
        ReplayStep step;
        step.step = result.size() + 1;
        step.t_ms = t;
        result.push_back(step);
      }
      return result;
    };
  const auto adapted = replaySchedule(make({1, 2, 3, 13, 23}));
  const auto uniform = replaySchedule(make({10, 20, 30}));
  const auto fine = replaySchedule(make({1, 2, 3}));
  if (!expect(adapted.step_ms == 10 && adapted.switch_ms == 3, "switch after the 1 ms phase") ||
    !expect(uniform.step_ms == 10 && uniform.switch_ms == 0, "constant step has no switch") ||
    !expect(fine.step_ms == 1 && fine.switch_ms == 0, "a derived 1 ms step has no switch") ||
    !expect(replaySchedule({}).step_ms == 0, "empty replay has no schedule"))
  {
    return 1;
  }
  return 0;
}
