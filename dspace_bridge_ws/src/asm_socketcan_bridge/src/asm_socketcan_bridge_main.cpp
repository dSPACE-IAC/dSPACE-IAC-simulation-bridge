#include "asm_socketcan_bridge.h"

#include <algorithm>
#include <iostream>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::Node::SharedPtr AsmSocketCanBridgeNodePtr;
  try
  {
    rclcpp::init(argc, argv);

    AsmSocketCanBridgeNodePtr = std::make_shared<asm_socketcan_bridge::AsmSocketCanBridgeNode>();

    // Deterministic sim mode: serialize every callback on one thread so stepping and
    // publishing order is reproducible. Wall mode keeps the multi-threaded executor for
    // maximum throughput (unchanged behavior).
    const bool sim_mode = AsmSocketCanBridgeNodePtr->get_parameter("use_sim_time").as_bool();
    if (sim_mode) {
      RCLCPP_INFO(AsmSocketCanBridgeNodePtr->get_logger(),
                  "Spinning with single-threaded executor (deterministic sim mode).");
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(AsmSocketCanBridgeNodePtr);
      executor.spin();
    } else {
      const auto hardware_threads = std::thread::hardware_concurrency();
      size_t executor_threads = 4;
      if (hardware_threads == 0) {
        executor_threads = 4;
      } else {
        size_t half = hardware_threads / 2;
        if (half == 0) {
          executor_threads = 1;
        } else {
          executor_threads = std::min<size_t>(8, half);
          if (half >= 4 && executor_threads < 4) {
            executor_threads = 4;
          }
        }
      }
      rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), executor_threads);
      executor.add_node(AsmSocketCanBridgeNodePtr);
      executor.spin();
    }
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception& e)
  {
    if (AsmSocketCanBridgeNodePtr) {
      RCLCPP_ERROR(AsmSocketCanBridgeNodePtr->get_logger(), "Failed to initialize ASM-SocketCAN-Bridge node: %s", e.what());
    } else {
      std::cerr << "Failed to initialize ASM-SocketCAN-Bridge node: " << e.what() << std::endl;
    }
    return 1;
  }
}
