#include "asm_socketcan_bridge.h"

namespace asm_socketcan_bridge {

  // asm communication
  void AsmSocketCanBridgeNode::vesiCallback()
  {
    auto now_ns = []() {
      return std::chrono::time_point_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now()).time_since_epoch().count();
    };

    const bool record_metrics = this->enableTimeRecord;

    double interval_ms = 0.0;
    double send_feedback_ms = 0.0;
    double request_data_ms = 0.0;
    double cast_ms = 0.0;

    if (record_metrics) {
      const auto now = now_ns();
      const auto previous_start = last_callback_start_ns;
      if (previous_start != 0) {
        interval_ms =
          static_cast<double>(static_cast<long long>(now) - static_cast<long long>(previous_start)) /
          1000000.0;
      }
      last_callback_start_ns = now;
    }

    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "vesiCallback");

    auto measure = [&](auto &&callable, double &out_ms) {
      if (!record_metrics) {
        std::forward<decltype(callable)>(callable)();
        out_ms = 0.0;
        return;
      }
      const auto start = now_ns();
      std::forward<decltype(callable)>(callable)();
      const auto end = now_ns();
      out_ms =
        static_cast<double>(static_cast<long long>(end) - static_cast<long long>(start)) / 1000000.0;
    };

    bool active_maneuver = false;
    {
      std::shared_lock<std::shared_mutex> lock(can_bus_mutex_);
      active_maneuver = this->maneuverStarted;
    }

    bool publish_reset = false;

    try {
      if (active_maneuver) {
        measure([&]() { AsmSocketCanBridgeNode::sendVehicleFeedbackToSimulation(); },
                send_feedback_ms);
        measure([&]() { this->api.requestCustomData(&canbus_raw_buffer_); }, request_data_ms);
      } else {
        send_feedback_ms = 0.0;
        measure([&]() { this->api.requestCustomData(&canbus_raw_buffer_); }, request_data_ms);
      }

      measure([&]() {
        constexpr auto required_canbus_size = sizeof(ASMBus);
        std::unique_lock<std::shared_mutex> lock(can_bus_mutex_);
        if (active_maneuver) {
          this->simTime_.advanceMilliseconds(this->simModeEnabled ? 1 : 10);
        }

        if (canbus_raw_buffer_.size() >= required_canbus_size) {
          std::memcpy(&canBusStorage_, canbus_raw_buffer_.data(), required_canbus_size);
          this->canBus = &canBusStorage_;
          if (this->canBus->maneuverInfo.maneuverState == 3 &&
              !this->maneuverStarted) {
            this->maneuverStarted = true;
            RCLCPP_INFO(get_logger(), "Maneuver started. Data will be published.");
          } else if (this->canBus->maneuverInfo.maneuverState != 3 &&
                     this->maneuverStarted) {
            RCLCPP_INFO(get_logger(), "Maneuver stopped. System will be reset.");
            initializeFeedback();
            publish_reset = true;
          }
          this->vesiDataAvailabe = true;
        } else {
          this->canBus = nullptr;
          this->vesiDataAvailabe = false;
          if (canbus_raw_buffer_.empty()) {
            if (this->verbosePrinting) {
              RCLCPP_WARN(get_logger(), "No Custom Data available.");
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(),
                                   *this->get_clock(),
                                   5000,
                                   "No Custom Data available.");
            }
          } else {
            RCLCPP_ERROR(get_logger(),
                         "Custom data buffer size (%zu) smaller than ASMBus (%zu); ignoring frame.",
                         canbus_raw_buffer_.size(),
                         required_canbus_size);
          }
        }
      }, cast_ms);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to request data from ASM: %s", e.what());
      return;
    }

    if (publish_reset) {
      std_msgs::msg::Bool resetMsg;
      resetMsg.data = true;
      this->resetCommandPublisher_->publish(resetMsg);
    }

    if (record_metrics) {
      std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
      this->myfile.open(std::string(this->pathTimeRecord) + "/duration_recording.csv",
                        std::ios_base::app);
      this->myfile << std::to_string(send_feedback_ms) << ","
                   << std::to_string(request_data_ms) << ","
                   << std::to_string(cast_ms) << ","
                   << std::to_string(0.0) << ","
                   << std::to_string(interval_ms) << "\n";
      this->myfile.close();
    }
    // NOTE (deterministic sim mode): /clock is published exactly once per sim_time_increase
    // handshake, from simClockTimeCallback() after all sub-steps complete -- NOT on every
    // vesiCallback. Publishing per sub-step made the controller run its control loop at the
    // sub-step (~1 ms) rate and amplified handshake traffic, which (with KeepLast(10) queues)
    // caused timing-dependent sample drops. One /clock per handshake keeps a clean 1:1 step
    // and a fixed control cadence.
  }

  void AsmSocketCanBridgeNode::sendVehicleFeedbackToSimulation()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "sendVehicleFeedbackToSimulation");
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);

      static bool raptor_connection_announced = false;
      if (!this->raptorDataAvailabe) {
        if (this->verbosePrinting) {
          RCLCPP_WARN(get_logger(),
                      "Did not receive to_raptor message. This might lead to unexpected behavior of the RaceControl e.g. setting of flags and P2P is not available. Check that your stack is alive.");
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), this->warning_throttle_intervall, "Did not receive to_raptor message. This might lead to unexpected behavior of the RaceControl e.g. setting of flags and P2P is not available. Check that your stack is alive.");
        }
        raptor_connection_announced = false;
      } else if (!raptor_connection_announced) {
        RCLCPP_INFO(get_logger(), "to_raptor message received.");
        raptor_connection_announced = true;
      }

      if (this->feedbackDataAvailabe == false && this->stackFeedbackConnectionWarningSent == false)
      {
        RCLCPP_WARN(get_logger(), "Did not receive vehicle_inputs message. The vehicle might move in an unexpected way. Check that your stack is alive.");
        this->stackFeedbackConnectionWarningSent = true;
      }
      else if (this->feedbackDataAvailabe == true && this->stackFeedbackConnectionWarningSent == true)
      {
        RCLCPP_INFO(get_logger(), "vehicle_inputs message received.");
        this->stackFeedbackConnectionWarningSent = false;
      }

      this->api.sendControlData(22222,std::addressof(this->feedbackCmd),sizeof(this->feedbackCmd));
    }
    if(this->simModeEnabled)
      this->api.increaseSimulationTime(0.001);
    else
      this->api.increaseSimulationTime(0.01);
  }


  // ros
  void AsmSocketCanBridgeNode::simClockTimeCallback()
  {
    std::unique_lock<std::shared_mutex> lock(can_bus_mutex_);
    simClockTime.clock = rclcpp::Time(this->simTime_.seconds(), this->simTime_.nanoseconds());
    this->simClockTimePublisher_->publish(simClockTime);
  }

  void AsmSocketCanBridgeNode::simTimeIncreaseCallback(const std_msgs::msg::UInt16 & msg)
  {
    runSimTimeHandshake(
      msg.data,
      [this]() { this->vesiCallback(); },
      [this]() { this->simClockTimeCallback(); });
  }

} // namespace asm_socketcan_bridge
