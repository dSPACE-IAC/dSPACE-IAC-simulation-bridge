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
    if (simModeEnabled) {
      RCLCPP_INFO(
        get_logger(),
        "SIM_OBS bridge summary=1 handshakes_received=%llu requested_substeps=%llu "
        "cumulative_substeps=%llu clock_published=%llu non_ten_handshakes=%llu "
        "substep_mismatches=%llu sim_time_ms=%llu",
        static_cast<unsigned long long>(sim_handshakes_received_.load()),
        static_cast<unsigned long long>(sim_requested_substeps_.load()),
        static_cast<unsigned long long>(sim_substeps_completed_.load()),
        static_cast<unsigned long long>(sim_clock_publications_.load()),
        static_cast<unsigned long long>(sim_non_ten_handshakes_.load()),
        static_cast<unsigned long long>(sim_substep_mismatches_.load()),
        static_cast<unsigned long long>(simTime_.totalMilliseconds()));
    }
    stop_reader_.store(true);
    if (can_socket >= 0) {
      close(can_socket);
      can_socket = -1;
    }
    if (reader_thread1.joinable()) {
      reader_thread1.join();
    }
  }


  void AsmSocketCanBridgeNode::switchRaceControlSourceCallback(const std_msgs::msg::Bool & msg)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "switchRaceControlSourceCallback");

    this->useCustomRaceControl = msg.data;
  }

} // namespace asm_socketcan_bridge
