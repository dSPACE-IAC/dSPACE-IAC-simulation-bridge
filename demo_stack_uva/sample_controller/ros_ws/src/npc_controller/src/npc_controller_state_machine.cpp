#include "npc_controller.hpp"

#include <cmath>

namespace controller
{

    void ControllerNode::state_machine()
    {
        if (!position_received || !wheel_speed_received || !path_loaded)
        {
            // RCLCPP_INFO(this->get_logger(),"position_received: %d  wheel_speed_received: %d  path_loaded: %d",
            //                                 position_received,
            //                                 wheel_speed_received,
            //                                 path_loaded);
            return;
        }
        // Calculate Path Base Projections
        PathPoint current_position;
        current_position.x = vehicle_state_.x;
        current_position.y = vehicle_state_.y;
        current_position.z = vehicle_state_.z;
        current_position.yaw = vehicle_state_.yaw;

        // TODO: Adjust for multiple static lines
        int center_bp = calculate_base_projections(center_line_, current_position);
        center_line_s_ = center_line_.points[center_bp].s;
        int pit_bp = calculate_base_projections(pit_line_, current_position);
        pit_line_s_ = pit_line_.points[pit_bp].s;
        int optimal_bp = calculate_base_projections(optimal_line_, current_position);
        optimal_line_s_ = optimal_line_.points[optimal_bp].s;
        const double dx_opt = optimal_line_.points[optimal_bp].x - current_position.x;
        const double dy_opt = optimal_line_.points[optimal_bp].y - current_position.y;
        optimal_line_distance_ = std::sqrt(dx_opt * dx_opt + dy_opt * dy_opt);
        // Signed cross-track error: positive = right of line, negative = left of line
        // Using cross product of (vehicle - line_point) with line tangent
        const double optimal_yaw = optimal_line_.points[optimal_bp].yaw;
        const double tangent_x = std::cos(optimal_yaw);
        const double tangent_y = std::sin(optimal_yaw);
        optimal_line_signed_error_ = dx_opt * tangent_y - dy_opt * tangent_x;

        // Process Drive by Wire State Machine
        ct_state_ = dbw_state_machine(ct_state_, track_flag_, vehicle_flag_, sys_state_, estop_, ct_input_, vehicle_state_.vx, disableStateMachine);

        // Process Lap State Machine
        lap_state_inputs_.ct_state = ct_state_;
        lap_state_inputs_.vehicle_flag = vehicle_flag_;
        lap_state_inputs_.track_flag = track_flag_;
        lap_state_inputs_.target_speed = target_speed_;
        lap_state_inputs_.current_speed = vehicle_state_.vx;
        lap_state_inputs_.center_line_s = center_line_s_;
        lap_state_inputs_.pit_lane_s = pit_line_s_;
        lap_state_inputs_.speed_profile = speed_profile_;

        lap_state_machine_.transition(lap_state_inputs_, lap_state_outputs_);

        // Log speed profile / state machine output
        static int sm_log_counter = 0;
        sm_log_counter++;

        if (sm_log_counter % 10 == 0) {

            RCLCPP_DEBUG(
                this->get_logger(),
                "[STATE_MACHINE] lap_state=%d des_vel=%.1f mph center_s=%.1f pit_s=%.1f | opt_line_dist=%.2f m opt_err=%.3f",
                static_cast<int>(lap_state_outputs_.lap_state),
                lap_state_outputs_.des_vel,
                center_line_s_,
                pit_line_s_,
                optimal_line_distance_,
                optimal_line_signed_error_);

            // Special logging for corkscrew section (center_line_s between 5200–6800 m)
            if (center_line_s_ > 5200 && center_line_s_ < 6800) {

                RCLCPP_DEBUG(
                    this->get_logger(),
                    "[CORKSCREW_SM] s=%.0f des_v=%.1f mph lap_state=%d opt_dist=%.2f m opt_err=%.3f drivable=%d",
                    center_line_s_,
                    lap_state_outputs_.des_vel,
                    static_cast<int>(lap_state_outputs_.lap_state),
                    optimal_line_distance_,
                    optimal_line_signed_error_,
                    lap_state_outputs_.driveable);
            }
        }

        desired_velocity_ = lap_state_outputs_.des_vel / 2.23694; // Convert MPH to m/s
        if (lap_state_outputs_.path == "pits")
        {
            current_path_ = &pit_line_;
        }
        else
        {
            current_path_ = &center_line_;
        }

        // Publish CT Report
        ct_report_msg_.ct_state = static_cast<int>(ct_state_);
        ct_report_msg_.header.stamp = this->now();

        ct_report_msg_.track_cond_ack = static_cast<int>(track_flag_);
        ct_report_msg_.veh_sig_ack = static_cast<int>(vehicle_flag_);
        ct_report_msg_.rolling_counter = ct_counter_;
        ct_report_msg_.veh_num = veh_num;
        push2pass_counter = (push2pass_counter + 1) % 500;
        if (push2pass_counter == 42)
        {
            if (current_push2pass_request)
            {
                current_push2pass_request = false;
            }
            else
            {
                current_push2pass_request = true;
            }
            // current_push2pass_request = true;
        }

        ct_report_msg_.push2pass_request = current_push2pass_request;
        ct_report_pub_->publish(ct_report_msg_);
        ct_counter_ = (ct_counter_ + 1) % 8;

        if (!this->useRaptorDbwNode) {
            publishCanMessage("ct_report", [&](PreparedCanMessage &message) {
                auto assign = [&](std::string_view signal_name, auto value) {
                    if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
                        insertBits(message.frame.data, *signal, value);
                    }
                };
                assign("track_cond_ack", static_cast<int>(track_flag_));
                assign("veh_sig_ack", static_cast<int>(vehicle_flag_));
                assign("ct_state", static_cast<int>(ct_state_));
                assign("ct_state_rolling_counter", ct_counter_);
                assign("veh_num", veh_num);
            });
        } else if (this->useRaptorDbwNode || this->publish_ros_all) {
            // Publish CT Report
            npc_ct_report_msg_.ct_state = static_cast<int>(ct_state_);
            npc_ct_report_msg_.track_flag_ack = static_cast<int>(track_flag_);
            npc_ct_report_msg_.veh_flag_ack = static_cast<int>(vehicle_flag_);
            npc_ct_report_msg_.target_speed = target_speed_;
            npc_ct_report_msg_.rolling_counter = ct_counter_;
            npc_ct_report_msg_.veh_num = veh_num;
            npc_ct_report_pub_->publish(npc_ct_report_msg_);
        }

        // Populate Debug Message
        debug_msg_.ct_state = static_cast<int>(ct_state_);
        debug_msg_.track_flag = static_cast<int>(track_flag_);
        debug_msg_.vehicle_flag = static_cast<int>(vehicle_flag_);
        debug_msg_.sys_state = static_cast<int>(sys_state_);
        debug_msg_.track_loc = static_cast<int>(lap_state_inputs_.track_loc);
        debug_msg_.lap_state = static_cast<int>(lap_state_inputs_.lap_state);
        debug_msg_.center_s = center_line_s_;
        debug_msg_.pit_s = pit_line_s_;
        debug_msg_.optimal_s = optimal_line_s_;
        debug_msg_.optimal_line_distance = optimal_line_distance_;
        debug_msg_.optimal_line_signed_error = optimal_line_signed_error_;

        // Log state machine info, especially around corkscrew (s ~ 5600-6400)
        if (center_line_s_ > 5200 && center_line_s_ < 6800) {
            double desired_speed_ms = desired_velocity_ / 2.237;  // Convert MPH to m/s
            double actual_speed_ms = vehicle_state_.vx;
            RCLCPP_DEBUG(this->get_logger(),
                "[CORKSCREW] s=%.1f | des_v=%.1f mph (%.2f m/s) act_v=%.1f mph (%.2f m/s) | steering=%.3f rad | dist=%.2f m",
                center_line_s_, desired_velocity_, desired_speed_ms, actual_speed_ms * 2.237, actual_speed_ms,
                pure_pursuit_steering_angle, optimal_line_distance_);
        }

        // Lateral Control Prerequisites
        debug_msg_.position_received = position_received;
        debug_msg_.wheel_speed_received = wheel_speed_received;
        debug_msg_.path_loaded = path_loaded;
    }

} // namespace controller