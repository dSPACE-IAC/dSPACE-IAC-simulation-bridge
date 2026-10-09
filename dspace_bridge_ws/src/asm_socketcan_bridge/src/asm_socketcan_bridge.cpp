#include "asm_socketcan_bridge.h"

namespace asm_socketcan_bridge {

  AsmSocketCanBridgeNode::AsmSocketCanBridgeNode() : Node("asm_socketcan_bridge_node")
  {
    this->canBus = nullptr;
    configureConnectionParameters();
    configurePublisherTimers();
    configureRuntimeParameters();
    if (!connectToSimulation()) {
      return;
    }
    if (!initializeCanInterface()) {
      return;
    }
    initializeRosInterfaces();
    initializeTimeRecording();

    RCLCPP_INFO(get_logger(), "Setup done.");
  }

  AsmSocketCanBridgeNode::~AsmSocketCanBridgeNode()
  {
    stop_stepping_.store(true);
    stop_reader_.store(true);
    coordinator_cv_.notify_all();
    if (reader_thread1.joinable()) {
      reader_thread1.join();
    }
    if (stepThread_.joinable()) {
      stepThread_.join();
    }
    if (can_socket >= 0) {
      close(can_socket);
      can_socket = -1;
    }
    if (simModeEnabled) {
      std::string stale_fields;
      for (std::size_t slot = 0; slot < kCommandCounterSlotCount; ++slot) {
        stale_fields += " stale_cmd_" + std::string(kCommandCounterNames[slot]) + "=" +
          std::to_string(staleCommandCounters_.staleCount(slot)) + "/" +
          std::to_string(staleCommandCounters_.observations(slot));
      }
      if (stepCoordinator_) {
        CoordinatorTotals totals;
        {
          const std::lock_guard<std::mutex> lock(coordinator_mutex_);
          totals = stepCoordinator_->totals();
        }
        stale_fields += " env_steps_closed=" + std::to_string(totals.steps_closed) +
          " env_timeouts=" + std::to_string(totals.timeouts) +
          " env_max_consecutive_timeouts=" + std::to_string(totals.max_consecutive_timeouts) +
          " env_frames=" + std::to_string(totals.frames) +
          " env_accepted=" + std::to_string(totals.accepted) +
          " env_duplicates=" + std::to_string(totals.duplicates) +
          " env_late=" + std::to_string(totals.late) +
          " env_early=" + std::to_string(totals.early) +
          " env_rephased=" + std::to_string(totals.rephased) +
          " env_time_mismatches=" + std::to_string(sim_time_mismatches_.load());
        if (replayActive_) {
          stale_fields += " env_replay_steps=" + std::to_string(replayStepsDone_.load()) + "/" +
            std::to_string(replaySteps_.size());
        }
        if (logRealTimeFactor_ && rtfTotal_.sim_ms > 0) {
          const double env_ms = static_cast<double>(rtfTotal_.env_ns) / 1e6;
          const double wait_ms = static_cast<double>(rtfTotal_.wait_ns) / 1e6;
          stale_fields += " realtime_factor=" + std::to_string(static_cast<double>(rtfTotal_.sim_ms) / (env_ms + wait_ms)) +
            " env_wall_ms=" + std::to_string(env_ms) + " wait_wall_ms=" + std::to_string(wait_ms);
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "SIM_OBS bridge summary=1 cumulative_substeps=%llu clock_published=%llu "
        "substep_mismatches=%llu step_markers_sent=%llu marker_write_failures=%llu "
        "sim_time_ms=%llu%s",
        static_cast<unsigned long long>(sim_substeps_completed_.load()),
        static_cast<unsigned long long>(sim_clock_publications_.load()),
        static_cast<unsigned long long>(sim_substep_mismatches_.load()),
        static_cast<unsigned long long>(sim_step_markers_sent_.load()),
        static_cast<unsigned long long>(sim_step_marker_write_failures_.load()),
        static_cast<unsigned long long>(simTime_.totalMilliseconds()),
        stale_fields.c_str());
    }
  }


  void AsmSocketCanBridgeNode::switchRaceControlSourceCallback(const std_msgs::msg::Bool & msg)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "switchRaceControlSourceCallback");

    this->useCustomRaceControl = msg.data;
  }

} // namespace asm_socketcan_bridge
