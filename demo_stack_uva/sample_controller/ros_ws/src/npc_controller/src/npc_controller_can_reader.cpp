#include "npc_controller.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <unistd.h>

namespace controller
{

    void ControllerNode::can_reader_loop(int sock, const std::string &bus_id)
    {
        RCLCPP_INFO(this->get_logger(), "Can reader loop started for %s", bus_id.c_str());
        struct can_frame in_frame{};
        while (rclcpp::ok() && !stop_reader_.load()) {
            if (this->receivedMessagePrinting) RCLCPP_INFO(this->get_logger(), "Can reader loop...");

            RCLCPP_DEBUG(get_logger(), "About to read CAN fd=%d", sock);
            int nbytes = read(sock, &in_frame, sizeof(in_frame));
            if (nbytes < 0) {
                if (stop_reader_.load()) break;

                if (errno == EINTR) continue;
                RCLCPP_ERROR(get_logger(), "CAN read failed: %s (sock=%d)", strerror(errno), sock);

                if (errno == EBADF) {
                    RCLCPP_ERROR(get_logger(), "Socket became invalid (EBADF). Exiting reader loop.");
                    break;
                }
                continue;
            }
            if (nbytes == 0) {
                continue;
            }

            if (this->receivedMessagePrinting)
                RCLCPP_INFO(this->get_logger(), "received: 0x%03X [%d] ",in_frame.can_id, in_frame.can_dlc);
            for (int i = 0; i < in_frame.can_dlc; i++) {
                if (this->receivedMessagePrinting)
                RCLCPP_INFO(this->get_logger(), "received: %02X ", in_frame.data[i]);
            }

            const auto warn_missing_metadata = [&](uint32_t message_id) {
                if (this->verbosePrinting) {
                RCLCPP_WARN(this->get_logger(),
                            "No metadata for CAN ID %u; dropping frame.",
                            message_id);
                } else {
                RCLCPP_WARN_ONCE(this->get_logger(),
                                "No metadata for CAN ID %u; dropping frame.",
                                message_id);
                }
            };

            switch (in_frame.can_id) {
                case 1300: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "wheel_speed_report");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("wheel_speed_RL")) {
                        this->ws_rear_left =static_cast<float>(*value);
                    }
                    if (const auto value = get_scaled("wheel_speed_FR")) {
                        this->ws_front_right = static_cast<float>(*value);
                    }
                    if (const auto value = get_scaled("wheel_speed_FL")) {
                        this->ws_front_left = static_cast<float>(*value);
                    }
                    if (const auto value = get_scaled("wheel_speed_RR")) {
                        this->ws_rear_right = static_cast<float>(*value);
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "wheel_speed_RL: %f  wheel_speed_FR: %f  wheel_speed_FL: %f  wheel_speed_RR: %f",
                                    this->ws_rear_left,
                                    this->ws_front_right,
                                    this->ws_front_left,
                                    this->ws_rear_right);
                    }
                    this->wheel_speed_callback();
                    break;
                }
                case 1340: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "pt_report_1");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("throttle_position")) {
                        this->throttle_position =
                        static_cast<float>(*value);
                    }
                    if (const auto value = get_scaled("current_gear")) {
                        this->current_gear =
                        static_cast<int8_t>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("engine_speed_rpm")) {
                        this->engine_rpm =
                        static_cast<float>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "throttle_position: %f  current_gear: %d  engine_speed_rpm: %f",
                                    this->throttle_position,
                                    this->current_gear,
                                    this->engine_rpm);
                    }
                    this->receivePtReport();
                    break;
                }
                case 1250: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "marelli_report_1");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("marelli_track_flag")) {
                        this->track_flag =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("marelli_vehicle_flag")) {
                        this->veh_flag =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "marelli_track_flag: %d  marelli_vehicle_flag: %d",
                                    this->track_flag,
                                    this->veh_flag);
                    }
                    this->receiveFlags();
                    break;
                }
                case 1200: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "base_to_car_summary");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("round_target_speed")) {
                        this->round_target_speed =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "round_target_speed: %d",
                                    this->round_target_speed);
                    }
                    this->receiveFlags();
                    break;
                }
                case 1304: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "misc_report");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("sys_state")) {
                        this->sys_state =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(), "sys_state: %d", this->sys_state);
                    }
                    this->receiveFlags();
                    break;
                }
            }
        }
    }

} // namespace controller