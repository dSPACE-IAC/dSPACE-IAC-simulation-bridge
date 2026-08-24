#include "asm_socketcan_bridge.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <linux/can.h>
#include <string>
#include <unistd.h>

namespace asm_socketcan_bridge {

  void AsmSocketCanBridgeNode::initializeFeedback()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "initializeFeedback");

    this->maneuverStarted = false;
    this->feedbackDataAvailabe = false;
    this->raptorDataAvailabe = false;

    this->feedbackCmd.vehicle_inputs.throttle_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.throttle_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_throttle_cmd = 0;

    this->feedbackCmd.vehicle_inputs.brake_cmd_front = 0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_rear = 0;
    this->feedbackCmd.vehicle_inputs.brake_bias_switch = 0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_brake_cmd = 0;

    this->feedbackCmd.vehicle_inputs.steering_cmd = 0;
    this->feedbackCmd.vehicle_inputs.steering_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_steering_cmd = 0;
    this->feedbackCmd.vehicle_inputs.drive_steering_FF_cntrl_switch = 0;
    this->feedbackCmd.vehicle_inputs.driver_steering_FF_cmd = 0.0f;
    this->feedbackCmd.vehicle_inputs.drive_steering_gain_cntrl_switch = 0;
    this->feedbackCmd.vehicle_inputs.driver_steering_P_cmd = 0.0f;
    this->feedbackCmd.vehicle_inputs.driver_steering_I_cmd = 0.0f;
    this->feedbackCmd.vehicle_inputs.driver_steering_D_cmd = 0.0f;

    this->feedbackCmd.vehicle_inputs.driver_traction_aim_switch = 0;
    this->feedbackCmd.vehicle_inputs.driver_traction_range_switch = 0;

    this->feedbackCmd.vehicle_inputs.gear_cmd = 1;
    this->feedbackCmd.vehicle_inputs.enable_gear_cmd = 0;
    
    this->feedbackCmd.to_raptor.track_cond_ack = 0;
    this->feedbackCmd.to_raptor.veh_sig_ack = 0;
    this->feedbackCmd.to_raptor.marelli_sector_flag_ack = 0;
    this->feedbackCmd.to_raptor.ct_state = 0;
    this->feedbackCmd.to_raptor.rolling_counter = 0;
    this->feedbackCmd.to_raptor.veh_num = 255;

    this->feedbackCmd.to_raptor.push2pass_switch = 0;
    this->feedbackCmd.to_raptor.push2pass_request = 0;
  }

  void AsmSocketCanBridgeNode::can_reader_loop(int sock, const std::string &bus_id)
  {
    RCLCPP_INFO(get_logger(), "Can reader loop started for %s", bus_id.c_str());
    struct can_frame in_frame{};
    while (rclcpp::ok() && !stop_reader_.load()) {
      if (this->receivedMessagePrinting)
        RCLCPP_INFO(get_logger(), "Can reader loop...");

      int nbytes = read(sock, &in_frame, sizeof(in_frame));
      if (nbytes < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (stop_reader_.load()) {
          break;
        }
        RCLCPP_ERROR(get_logger(), "CAN read failed: %s", strerror(errno));
        continue;
      }
      if (nbytes == 0) {
        continue;
      }

      if (this->receivedMessagePrinting)
        RCLCPP_INFO(get_logger(), "received: 0x%03X [%d] ",in_frame.can_id, in_frame.can_dlc);
      for (int i = 0; i < in_frame.can_dlc; i++) {
        if (this->receivedMessagePrinting)
          RCLCPP_INFO(get_logger(), "received: %02X ", in_frame.data[i]);
      }

      const auto warn_missing_metadata = [&](uint32_t message_id) {
        if (this->verbosePrinting) {
          RCLCPP_WARN(get_logger(),
                      "No metadata for CAN ID %u; dropping frame.",
                      message_id);
        } else {
          RCLCPP_WARN_ONCE(get_logger(),
                           "No metadata for CAN ID %u; dropping frame.",
                           message_id);
        }
      };

      switch (in_frame.can_id) {
        case 1400: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "brake_pressure_cmd");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("brk_pressure_cmd_counter")) {
            this->feedbackCmd.vehicle_inputs.brake_cmd_count =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("F_brake_pressure_cmd")) {
            this->feedbackCmd.vehicle_inputs.brake_cmd_front =
              static_cast<uint16_t>(std::round(*value));
          }
          if (const auto value = get_scaled("R_brake_pressure_cmd")) {
            this->feedbackCmd.vehicle_inputs.brake_cmd_rear =
              static_cast<uint16_t>(std::round(*value));
          }
          this->feedbackCmd.vehicle_inputs.enable_brake_cmd = 1;
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "brake_cmd_count: %d  brake_cmd_front: %f  brake_cmd_rear: %f  enable_brake_cmd: %d",
                        this->feedbackCmd.vehicle_inputs.brake_cmd_count,
                        static_cast<double>(this->feedbackCmd.vehicle_inputs.brake_cmd_front),
                        static_cast<double>(this->feedbackCmd.vehicle_inputs.brake_cmd_rear),
                        this->feedbackCmd.vehicle_inputs.enable_brake_cmd);
          }
          break;
        }
        case 1401: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "accelerator_cmd");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("acc_pedal_cmd_counter")) {
            this->feedbackCmd.vehicle_inputs.throttle_cmd_count = static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("acc_pedal_cmd")) {
            this->feedbackCmd.vehicle_inputs.throttle_cmd = *value;
          }
          this->feedbackCmd.vehicle_inputs.enable_throttle_cmd = 1;
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "throttle_cmd_count: %d  throttle_cmd: %f  enable_brake_cmd: %d",
                        this->feedbackCmd.vehicle_inputs.throttle_cmd_count,
                        static_cast<double>(this->feedbackCmd.vehicle_inputs.throttle_cmd),
                        this->feedbackCmd.vehicle_inputs.enable_throttle_cmd);
          }
          break;
        }
        case 1402: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "steering_cmd");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("steering_motor_cmd_counter")) {
            this->feedbackCmd.vehicle_inputs.steering_cmd_count = static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("steering_motor_ang_cmd")) {
            this->feedbackCmd.vehicle_inputs.steering_cmd = static_cast<int16_t>(std::round(*value));
          }
          if (const auto value = get_scaled("driver_steering_FF_cmd")) {
            this->feedbackCmd.vehicle_inputs.driver_steering_FF_cmd = static_cast<float>(*value);;
          }
          if (const auto value = get_scaled("driver_steering_P_cmd")) {
            // Todo priority high: When the drive_steering_gain_cntrl_switch is 1 - these commanded gains should be used for PID loop on IAC software for steering control
            this->feedbackCmd.vehicle_inputs.driver_steering_P_cmd = static_cast<float>(*value);;
          }
          if (const auto value = get_scaled("driver_steering_I_cmd")) {
            // Todo priority high: When the drive_steering_gain_cntrl_switch is 1 - these commanded gains should be used for PID loop on IAC software for steering control
            this->feedbackCmd.vehicle_inputs.driver_steering_I_cmd = static_cast<float>(*value);;
          }
          if (const auto value = get_scaled("driver_steering_D_cmd")) {
            // Todo priority high: When the drive_steering_gain_cntrl_switch is 1 - these commanded gains should be used for PID loop on IAC software for steering control
            this->feedbackCmd.vehicle_inputs.driver_steering_D_cmd = static_cast<float>(*value);;
          }
          this->feedbackCmd.vehicle_inputs.enable_steering_cmd = 1;
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "steering_cmd_count: %d  steering_cmd: %f  enable_steering_cmd: %d  driver_steering_FF_cmd: %f  driver_steering_P_cmd: %f  driver_steering_I_cmd: %f  driver_steering_D_cmd: %f",
                        this->feedbackCmd.vehicle_inputs.steering_cmd_count,
                        static_cast<double>(this->feedbackCmd.vehicle_inputs.steering_cmd),
                        this->feedbackCmd.vehicle_inputs.enable_steering_cmd,
                        this->feedbackCmd.vehicle_inputs.driver_steering_FF_cmd,
                        this->feedbackCmd.vehicle_inputs.driver_steering_P_cmd,
                        this->feedbackCmd.vehicle_inputs.driver_steering_I_cmd,
                        this->feedbackCmd.vehicle_inputs.driver_steering_D_cmd);
          }
          break;
        }
        case 1403: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "gear_shift_cmd");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          if (const auto value = extractSignalScaled(in_frame.can_id, "desired_gear",
                                                     in_frame.data)) {
            this->feedbackCmd.vehicle_inputs.gear_cmd = static_cast<uint8_t>(std::floor(*value));
          }
          this->feedbackCmd.vehicle_inputs.enable_gear_cmd = 1;
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "desired_gear: %d",
                        this->feedbackCmd.vehicle_inputs.gear_cmd);
          }
          break;
        }
        case 1404: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "ct_report");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("track_cond_ack")) {
            this->feedbackCmd.to_raptor.track_cond_ack =
              static_cast<uint16_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("veh_sig_ack")) {
            this->feedbackCmd.to_raptor.veh_sig_ack =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("ct_state")) {
            this->feedbackCmd.to_raptor.ct_state =
              static_cast<uint16_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("ct_state_rolling_counter")) {
            this->feedbackCmd.to_raptor.rolling_counter =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("veh_num")) {
            this->feedbackCmd.to_raptor.veh_num =
              static_cast<uint8_t>(std::floor(*value));
          }
          this->raptorDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "track_cond_ack: %d  veh_sig_ack: %d  ct_state: %d  ct_state_rolling_counter: %d  veh_num: %d",
                        this->feedbackCmd.to_raptor.track_cond_ack,
                        this->feedbackCmd.to_raptor.veh_sig_ack,
                        this->feedbackCmd.to_raptor.ct_state,
                        this->feedbackCmd.to_raptor.rolling_counter,
                        this->feedbackCmd.to_raptor.veh_num);
          }
          break;
        }
        case 1405: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "ct_report_2");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          uint8_t marelli_sector_flag_ack = 0U;
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("marelli_track_flag_ack")) {
            this->feedbackCmd.to_raptor.track_cond_ack =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("marelli_vehicle_flag_ack")) {
            this->feedbackCmd.to_raptor.veh_sig_ack =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("marelli_sector_flag_ack")) {
            marelli_sector_flag_ack =
              static_cast<uint8_t>(std::floor(*value));
            this->feedbackCmd.to_raptor.marelli_sector_flag_ack = marelli_sector_flag_ack;
          }
          this->raptorDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "marelli_track_flag_ack: %d  marelli_vehicle_flag_ack: %d  marelli_sector_flag_ack: %d",
                        this->feedbackCmd.to_raptor.track_cond_ack,
                        this->feedbackCmd.to_raptor.veh_sig_ack,
                        marelli_sector_flag_ack);
          }
          break;
        }
        case 1406: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "dash_switches_cmd");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("brake_bias_aim_switch")) {
            this->feedbackCmd.vehicle_inputs.brake_bias_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("push2pass_switch")) {
            this->feedbackCmd.to_raptor.push2pass_switch =
              static_cast<uint8_t>(std::floor(*value));
            this->feedbackCmd.to_raptor.push2pass_request =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("driver_traction_aim_switch")) {
            this->feedbackCmd.vehicle_inputs.driver_traction_aim_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("driver_traction_range_switch")) {
            this->feedbackCmd.vehicle_inputs.driver_traction_range_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("drive_steering_gain_cntrl_switch")) {
            this->feedbackCmd.vehicle_inputs.drive_steering_gain_cntrl_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("drive_steering_FF_cntrl_switch")) {
            this->feedbackCmd.vehicle_inputs.drive_steering_FF_cntrl_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "brake_bias_aim_switch: %d  push2pass_switch: %d  driver_traction_aim_switch: %d  driver_traction_range_switch: %d  drive_steering_gain_cntrl_switch: %d  drive_steering_FF_cntrl_switch: %d",
                        this->feedbackCmd.vehicle_inputs.brake_bias_switch,
                        this->feedbackCmd.to_raptor.push2pass_switch,
                        this->feedbackCmd.vehicle_inputs.driver_traction_aim_switch,
                        this->feedbackCmd.vehicle_inputs.driver_traction_range_switch,
                        this->feedbackCmd.vehicle_inputs.drive_steering_gain_cntrl_switch,
                        this->feedbackCmd.vehicle_inputs.drive_steering_FF_cntrl_switch);
          }
          break;
        }
        case 1450: {
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "ct_vehicle_acc_feedback");
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (!findMessageByID(in_frame.can_id)) {
            warn_missing_metadata(in_frame.can_id);
            break;
          }
          float long_ct_vehicle_acc_fbk = 0.0f;
          float lat_ct_vehicle_acc_fbk = 0.0f;
          float vertical_ct_vehicle_acc_fbk = 0.0f;
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("long_ct_vehicle_acc_fbk")) {
            long_ct_vehicle_acc_fbk = static_cast<float>(*value);
          }
          if (const auto value = get_scaled("lat_ct_vehicle_acc_fbk")) {
            lat_ct_vehicle_acc_fbk = static_cast<float>(*value);
          }
          if (const auto value = get_scaled("vertical_ct_vehicle_acc_fbk")) {
            vertical_ct_vehicle_acc_fbk = static_cast<float>(*value);
          }
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "long_ct_vehicle_acc_fbk: %f  long_ct_vehicle_acc_fbk: %f  long_ct_vehicle_acc_fbk: %f",
                        static_cast<double>(long_ct_vehicle_acc_fbk),
                        static_cast<double>(lat_ct_vehicle_acc_fbk),
                        static_cast<double>(vertical_ct_vehicle_acc_fbk));
          }
          break;
        }
        default:
          if (this->receivedMessagePrinting)
            RCLCPP_INFO(get_logger(), "Message with unknown CAN ID received: 0x%03X [%d] ",in_frame.can_id, in_frame.can_dlc);
          break;
      }
    }

    if (!stop_reader_.load() && sock >= 0) {
      close(sock);
    }
  }

} // namespace asm_socketcan_bridge
