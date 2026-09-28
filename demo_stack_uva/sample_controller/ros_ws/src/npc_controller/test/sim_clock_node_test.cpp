#include "npc_controller.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <novatel_oem7_msgs/msg/bestpos.hpp>

namespace
{
template <typename Predicate>
bool spinUntil(rclcpp::executors::SingleThreadedExecutor &executor, Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  executor.spin_some();
  return predicate();
}

bool expect(bool condition, const char *description)
{
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    return false;
  }
  return true;
}
}  // namespace

int main(int argc, char **argv)
{
  unsetenv("SIM_CLOCK_MODE");
  rclcpp::init(argc, argv);
  int result = 0;

  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("connection.useRaptorDbwNode", true),
    });
    auto controller_node = std::make_shared<controller::ControllerNode>(options);
    auto test_node = std::make_shared<rclcpp::Node>("sim_clock_node_test_driver");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller_node);
    executor.add_node(test_node);

    auto clock_publisher = test_node->create_publisher<rosgraph_msgs::msg::Clock>(
      "clock", rclcpp::QoS(10).reliable().transient_local());
    auto bestpos_publisher = test_node->create_publisher<novatel_oem7_msgs::msg::BESTPOS>(
      "bestpos", rclcpp::QoS(10).reliable());
    std::vector<std::uint16_t> handshakes;
    std::vector<std::uint64_t> debug_steps;
    auto handshake_subscription = test_node->create_subscription<std_msgs::msg::UInt16>(
      "sim_time_increase", rclcpp::QoS(10),
      [&](std_msgs::msg::UInt16::ConstSharedPtr message) {
        handshakes.push_back(message->data);
      });
    auto debug_subscription = test_node->create_subscription<npc_controller_msgs::msg::NPCDebug>(
      "debug", rclcpp::QoS(10),
      [&](npc_controller_msgs::msg::NPCDebug::ConstSharedPtr message) {
        debug_steps.push_back(message->sim_step);
      });

    const auto publish_clock = [&](int32_t seconds, uint32_t nanoseconds) {
      rosgraph_msgs::msg::Clock message;
      message.clock.sec = seconds;
      message.clock.nanosec = nanoseconds;
      clock_publisher->publish(message);
    };
    const auto publish_bestpos = [&](int32_t seconds, uint32_t nanoseconds, double latitude) {
      novatel_oem7_msgs::msg::BESTPOS message;
      message.header.stamp.sec = seconds;
      message.header.stamp.nanosec = nanoseconds;
      message.lat = latitude;
      message.lon = -86.2352425671970906;
      message.hgt = 224.1435846661534015;
      bestpos_publisher->publish(message);
    };
    const auto wait_for_counts = [&](std::size_t expected_handshakes,
                                     std::size_t expected_debug_records) {
      return spinUntil(executor, [&]() {
        return handshakes.size() >= expected_handshakes &&
               debug_steps.size() >= expected_debug_records;
      });
    };
    const auto publish_clock_and_wait = [&](int32_t seconds, uint32_t nanoseconds,
                                            std::size_t expected_handshakes,
                                            std::size_t expected_debug_records) {
      publish_clock(seconds, nanoseconds);
      return wait_for_counts(expected_handshakes, expected_debug_records);
    };

    if (!expect(controller_node->get_parameter("use_sim_time").as_bool(),
                "node starts with simulated time enabled") ||
        !expect(spinUntil(executor, [&]() {
                  return clock_publisher->get_subscription_count() > 0 &&
                  bestpos_publisher->get_subscription_count() > 0 &&
                         handshake_subscription->get_publisher_count() > 0 &&
                         debug_subscription->get_publisher_count() > 0;
                }),
                "clock and result topics discover their endpoints") ||
        !expect(publish_clock_and_wait(0, 0, 1, 0),
                "zero-time baseline publishes one handshake without control") ||
        !expect((publish_bestpos(0, 10000000, 39.7947350319205384), true),
          "publish BESTPOS before the first nonzero clock") ||
        !expect(publish_clock_and_wait(0, 10000000, 2, 1),
          "a BESTPOS-first step invokes control once") ||
        !expect((publish_clock(0, 20000000), true),
          "publish the second nonzero clock before BESTPOS") ||
        !expect((executor.spin_some(), true), "process a clock-first sim step") ||
        !expect(handshakes.size() == 2 && debug_steps.size() == 1,
          "clock-first step waits without publishing control or handshake") ||
        !expect((publish_bestpos(0, 20000000, 39.7947351319205384), true),
          "publish matching BESTPOS after the second clock") ||
        !expect(wait_for_counts(3, 2),
          "matching BESTPOS releases the deferred clock-first step") ||
        !expect((publish_bestpos(1, 0, 39.7947352319205384), true),
          "publish whole-second BESTPOS before its clock") ||
        !expect(publish_clock_and_wait(1, 0, 4, 3),
                "whole-second clock invokes control once")) {
      result = 1;
    }

    if (!expect(handshakes.size() == 4, "one handshake is published for every clock") ||
        !expect(debug_steps == std::vector<std::uint64_t>{1, 2, 3},
                "one control debug record is published per nonzero clock") ||
        !expect(std::all_of(handshakes.begin(), handshakes.end(),
                            [](std::uint16_t value) { return value == 10; }),
                "each clock requests ten simulation substeps")) {
      result = 1;
    }
  }

  rclcpp::shutdown();
  return result;
}