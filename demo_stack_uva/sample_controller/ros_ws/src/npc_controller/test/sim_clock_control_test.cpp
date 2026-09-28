#include "sim_clock_control.h"
#include "sim_bestpos_gate.h"

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

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
  {
    controller::SimBestPosGate clock_first;
    clock_first.observeClock(0, 10000000);
    if (!expect(!clock_first.readyForStep(1),
                "clock alone cannot release a nonzero sim step")) {
      return 1;
    }
    clock_first.observeBestPos(0, 10000000);
    if (!expect(clock_first.readyForStep(1),
                "matching BESTPOS releases a clock-first step")) {
      return 1;
    }

    controller::SimBestPosGate position_first;
    position_first.observeBestPos(0, 20000000);
    if (!expect(!position_first.readyForStep(2),
                "BESTPOS alone cannot release a nonzero sim step")) {
      return 1;
    }
    position_first.observeClock(0, 20000000);
    if (!expect(position_first.readyForStep(2),
                "matching clock releases a BESTPOS-first step")) {
      return 1;
    }

    controller::SimBestPosGate mismatched;
    mismatched.observeClock(0, 30000000);
    mismatched.observeBestPos(0, 20000000);
    if (!expect(!mismatched.readyForStep(3) && mismatched.hasMismatchedPosition(),
                "a position from another sim time cannot release the step")) {
      return 1;
    }

    controller::SimBestPosGate zero_time;
    zero_time.observeClock(0, 0);
    if (!expect(zero_time.readyForStep(0), "zero-time baseline needs no BESTPOS")) {
      return 1;
    }
  }

  {
    struct SharedInputs
    {
      int velocity;
      int acceleration;
    } shared_inputs{1, 2};

    std::mutex feedback_mutex;
    std::mutex signal_mutex;
    std::condition_variable signal_condition;
    bool snapshot_started = false;
    bool mutation_attempted = false;
    bool mutation_blocked = false;

    std::thread mutator([&]() {
      {
        std::unique_lock<std::mutex> lock(signal_mutex);
        signal_condition.wait(lock, [&]() { return snapshot_started; });
      }

      if (feedback_mutex.try_lock()) {
        shared_inputs = {10, 20};
        feedback_mutex.unlock();
      } else {
        mutation_blocked = true;
      }
      {
        std::lock_guard<std::mutex> lock(signal_mutex);
        mutation_attempted = true;
      }
      signal_condition.notify_one();

      if (mutation_blocked) {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        shared_inputs = {10, 20};
      }
    });

    const auto snapshot = controller::captureSimControlInputs(feedback_mutex, [&]() {
      {
        std::lock_guard<std::mutex> lock(signal_mutex);
        snapshot_started = true;
      }
      signal_condition.notify_one();
      {
        std::unique_lock<std::mutex> lock(signal_mutex);
        signal_condition.wait(lock, [&]() { return mutation_attempted; });
      }
      return shared_inputs;
    });
    mutator.join();

    if (!expect(mutation_blocked, "concurrent mutation waits for the step snapshot") ||
        !expect(snapshot.velocity == 1 && snapshot.acceleration == 2,
                "step snapshot contains one consistent input set") ||
        !expect(snapshot.velocity + snapshot.acceleration == 3,
                "control outcome uses the captured inputs after shared state changes") ||
        !expect(shared_inputs.velocity == 10 && shared_inputs.acceleration == 20,
                "concurrent mutation proceeds after the snapshot is complete")) {
      return 1;
    }
  }

  if (!expect(controller::shouldCreatePeriodicControlTimers(false),
              "wall mode creates periodic control timers") ||
      !expect(!controller::shouldCreatePeriodicControlTimers(true),
              "sim mode skips periodic control timers") ||
      !expect(!controller::shouldRunSimTimeControl(0, 0), "zero clock does not run control") ||
      !expect(controller::shouldRunSimTimeControl(0, 1000000),
              "first nonzero nanosecond runs control") ||
      !expect(controller::shouldRunSimTimeControl(1, 0),
              "exact whole second runs control") ||
            !expect(controller::shouldWaitForSimStepMarker(true, false),
              "sim direct-CAN mode waits for the step marker") ||
            !expect(!controller::shouldWaitForSimStepMarker(true, true),
              "Raptor DBW mode does not wait on its absent CAN reader") ||
            !expect(!controller::shouldWaitForSimStepMarker(false, false),
              "wall mode does not wait for sim markers") ||
            !expect(controller::simStepForClockMessage(0) == 0,
              "no received clock has no completed sim step") ||
            !expect(controller::simStepForClockMessage(1) == 0,
              "initial clock is the step-zero baseline") ||
            !expect(controller::simStepForClockMessage(2) == 1,
              "second clock consumes marker one") ||
            !expect(controller::simStepForClockMessage(301) == 300,
              "clock count maps to its completed logical step")) {
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