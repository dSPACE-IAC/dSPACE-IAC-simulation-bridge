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
