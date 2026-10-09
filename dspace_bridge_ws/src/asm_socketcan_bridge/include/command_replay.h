#ifndef ASM_SOCKETCAN_BRIDGE__COMMAND_REPLAY_H_
#define ASM_SOCKETCAN_BRIDGE__COMMAND_REPLAY_H_

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <istream>
#include <optional>
#include <string>
#include <vector>

#include "step_coordinator.h"

namespace asm_socketcan_bridge
{
// Commands applied at the start of one logical step (the snapshot S(n-1) of F.2), fresh frames only.
struct ReplayStep
{
  std::uint64_t step = 0;
  std::uint64_t t_ms = 0;
  std::vector<CommandFrame> frames;
};

inline const char * replayHeader() noexcept
{
  return "step,t_ms,frames";
}

// One CSV row: step,t_ms,ID:DLC:HEX16|ID:DLC:HEX16 (decimal ID, DLC and hex payload of all 8 bytes).
inline std::string formatReplayRow(
  std::uint64_t step, std::uint64_t t_ms, const std::vector<SnapshotEntry> & snapshot)
{
  std::string row = std::to_string(step) + ',' + std::to_string(t_ms) + ',';
  bool first = true;
  for (const auto & entry : snapshot) {
    if (!entry.fresh) {
      continue;
    }
    if (!first) {
      row += '|';
    }
    first = false;
    row += std::to_string(entry.frame.id) + ':' + std::to_string(entry.frame.dlc) + ':';
    for (const auto byte : entry.frame.data) {
      char hex[3];
      std::snprintf(hex, sizeof(hex), "%02X", byte);
      row += hex;
    }
  }
  return row;
}

namespace replay_detail
{
inline bool parseUnsigned(const std::string & text, std::uint64_t & value, int base = 10)
{
  if (text.empty() || text.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
    return false;
  }
  try {
    std::size_t used = 0;
    value = std::stoull(text, &used, base);
    return used == text.size();
  } catch (...) {
    return false;
  }
}

inline std::vector<std::string> split(const std::string & text, char separator)
{
  std::vector<std::string> parts;
  std::size_t begin = 0;
  while (true) {
    const auto end = text.find(separator, begin);
    parts.push_back(text.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      return parts;
    }
    begin = end + 1;
  }
}
}  // namespace replay_detail

inline std::optional<ReplayStep> parseReplayRow(const std::string & row)
{
  const auto columns = replay_detail::split(row, ',');
  if (columns.size() != 3) {
    return std::nullopt;
  }
  ReplayStep step;
  if (!replay_detail::parseUnsigned(columns[0], step.step) ||
    !replay_detail::parseUnsigned(columns[1], step.t_ms))
  {
    return std::nullopt;
  }
  if (columns[2].empty()) {
    return step;
  }
  for (const auto & text : replay_detail::split(columns[2], '|')) {
    const auto fields = replay_detail::split(text, ':');
    std::uint64_t id = 0;
    std::uint64_t dlc = 0;
    if (fields.size() != 3 || !replay_detail::parseUnsigned(fields[0], id) ||
      !replay_detail::parseUnsigned(fields[1], dlc) || id > 0x7FF || dlc > 8 ||
      fields[2].size() != 16)
    {
      return std::nullopt;
    }
    CommandFrame frame;
    frame.id = static_cast<std::uint32_t>(id);
    frame.dlc = static_cast<std::uint8_t>(dlc);
    for (std::size_t index = 0; index < frame.data.size(); ++index) {
      std::uint64_t byte = 0;
      if (!replay_detail::parseUnsigned(fields[2].substr(index * 2, 2), byte, 16)) {
        return std::nullopt;
      }
      frame.data[index] = static_cast<std::uint8_t>(byte);
    }
    step.frames.push_back(frame);
  }
  return step;
}

// Step size after the adaptation switch and the time of that switch (0 without a 1 ms phase).
struct ReplaySchedule
{
  std::uint32_t step_ms = 0;
  std::uint64_t switch_ms = 0;
};

inline ReplaySchedule replaySchedule(const std::vector<ReplayStep> & steps)
{
  ReplaySchedule schedule;
  if (steps.empty()) {
    return schedule;
  }
  const auto delta = [&steps](std::size_t index) {
      return steps[index].t_ms - (index == 0 ? 0 : steps[index - 1].t_ms);
    };
  schedule.step_ms = static_cast<std::uint32_t>(delta(steps.size() - 1));
  std::size_t first_final = steps.size() - 1;
  while (first_final > 0 && delta(first_final - 1) == schedule.step_ms) {
    --first_final;
  }
  schedule.switch_ms = first_final == 0 ? 0 : steps[first_final - 1].t_ms;
  return schedule;
}

// Reads a recorded command file; steps must be numbered 1..N with strictly rising times.
// On failure returns false and describes the first offending line in error.
inline bool loadReplay(std::istream & input, std::vector<ReplayStep> & steps, std::string & error)
{
  steps.clear();
  std::string line;
  std::size_t line_number = 0;
  bool header_seen = false;
  while (std::getline(input, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    if (!header_seen) {
      header_seen = true;
      if (line != replayHeader()) {
        error = "line " + std::to_string(line_number) + ": expected header '" + replayHeader() + "'";
        return false;
      }
      continue;
    }
    auto step = parseReplayRow(line);
    if (!step) {
      error = "line " + std::to_string(line_number) + ": malformed row";
      return false;
    }
    const std::uint64_t previous_time = steps.empty() ? 0 : steps.back().t_ms;
    if (step->step != steps.size() + 1 || step->t_ms <= previous_time) {
      error = "line " + std::to_string(line_number) +
        ": steps must be numbered from 1 and their times must rise";
      return false;
    }
    steps.push_back(std::move(*step));
  }
  if (!header_seen || steps.empty()) {
    error = "no recorded steps";
    return false;
  }
  return true;
}
}  // namespace asm_socketcan_bridge

#endif  // ASM_SOCKETCAN_BRIDGE__COMMAND_REPLAY_H_
