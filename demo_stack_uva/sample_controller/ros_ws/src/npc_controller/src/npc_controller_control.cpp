#include "npc_controller.hpp"

#include <algorithm>
#include <cmath>

namespace controller
{

    void ControllerNode::long_control()
    {
        // Update Long Control Parameters
        min_throttle_ = get_parameter("vehicle.min_throttle").as_double();
        max_throttle_ = get_parameter("vehicle.max_throttle").as_double();
        min_brake_ = get_parameter("vehicle.min_brake").as_double();
        max_brake_ = get_parameter("vehicle.max_brake").as_double();
        max_acc_ = get_parameter("vehicle.max_acc").as_double();
        min_acc_ = get_parameter("vehicle.min_acc").as_double();
        vel_kp_ = get_parameter("vehicle.vel_kp").as_double();
        vel_ki_ = get_parameter("vehicle.vel_ki").as_double();
        vel_kd_ = get_parameter("vehicle.vel_kd").as_double();
        throttle_kp_ = get_parameter("vehicle.throttle_kp").as_double();
        throttle_ki_ = get_parameter("vehicle.throttle_ki").as_double();
        throttle_kd_ = get_parameter("vehicle.throttle_kd").as_double();
        brake_kp_ = get_parameter("vehicle.braking_kp").as_double();
        brake_ki_ = get_parameter("vehicle.braking_ki").as_double();
        brake_kd_ = get_parameter("vehicle.braking_kd").as_double();

        // Calculate Desired Acceleration
        double desired_acceleration = calc_acceleration(desired_velocity_);
        desired_acceleration = std::max(min_acc_, std::min(desired_acceleration, max_acc_));
        debug_msg_.desired_acceleration_clamped = desired_acceleration;
        debug_msg_.non_brake_decel = non_brake_decel_;
        // double throttle = vehicle_state_.throttle;
        // bool is_accelerating = throttle > 0;

        calc_throttle(desired_acceleration);
        calc_brake(desired_acceleration);

        double max_thr = ((4.0 * 2.23694 * vehicle_state_.vx) / 8.0) + 25.0;
        double throttle_cmd = std::max(min_throttle_, std::min(vehicle_state_.throttle, max_thr));

        vehicle_cmd_msg_.throttle_cmd = throttle_cmd;
        vehicle_cmd_msg_.throttle_cmd_count = rolling_counter;

        double brake_cmd_front = 0.5 * std::max(min_brake_, std::min(vehicle_state_.brake, max_brake_));
        double brake_cmd_rear = 0.5 * std::max(min_brake_, std::min(vehicle_state_.brake, max_brake_));

        vehicle_cmd_msg_.brake_cmd_front = static_cast<uint16_t>(std::round(brake_cmd_front));
        vehicle_cmd_msg_.brake_cmd_rear = static_cast<uint16_t>(std::round(brake_cmd_rear));
        vehicle_cmd_msg_.brake_cmd_count = rolling_counter;

        uint8_t gear_cmd = get_gear_shift_cmd();
        vehicle_cmd_msg_.gear_cmd = gear_cmd;

        vehicle_cmd_msg_.header.stamp = this->now();
        vehicle_cmd_pub_->publish(vehicle_cmd_msg_);

        if (!this->useRaptorDbwNode) {
            publishCanMessage("gear_shift_cmd", [&](PreparedCanMessage &message) {
                auto assign = [&](std::string_view signal_name, auto value) {
                    if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
                        insertBits(message.frame.data, *signal, value);
                    }
                };
                assign("desired_gear", gear_cmd);
            });
            publishCanMessage("accelerator_cmd", [&](PreparedCanMessage &message) {
                auto assign = [&](std::string_view signal_name, auto value) {
                    if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
                        insertBits(message.frame.data, *signal, value);
                    }
                };
                assign("acc_pedal_cmd_counter", rolling_counter);
                assign("acc_pedal_cmd", throttle_cmd);
            });
            publishCanMessage("brake_pressure_cmd", [&](PreparedCanMessage &message) {
                auto assign = [&](std::string_view signal_name, auto value) {
                    if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
                        insertBits(message.frame.data, *signal, value);
                    }
                };
                assign("brk_pressure_cmd_counter", rolling_counter);
                assign("F_brake_pressure_cmd", brake_cmd_front);
                assign("R_brake_pressure_cmd", brake_cmd_rear);
            });
        } else if (this->useRaptorDbwNode || this->publish_ros_all) {
            raptor_dbw_msgs::msg::AcceleratorPedalCmd ros_accel_cmd;
            ros_accel_cmd.pedal_cmd = throttle_cmd;
            ros_accel_cmd.enable  = true;
            ros_accel_cmd.rolling_counter = rolling_counter;
            throttle_cmd_pub_->publish(ros_accel_cmd);

            raptor_dbw_msgs::msg::BrakeCmd ros_brake_cmd;
            ros_brake_cmd.pedal_cmd = 2 * brake_cmd_front;
            ros_brake_cmd.enable  = true;
            ros_brake_cmd.rolling_counter = rolling_counter;
            brake_cmd_pub_->publish(ros_brake_cmd);

            std_msgs::msg::UInt8 ros_gear_cmd;
            ros_gear_cmd.data = gear_cmd;
            gear_cmd_pub_->publish(ros_gear_cmd);
        }

        // Debug Publisher
        debug_msg_.desired_velocity = desired_velocity_;
        debug_msg_.current_velocity = vehicle_state_.vx;
        debug_msg_.desired_acceleration = desired_acceleration;
        debug_msg_.current_acceleration = vehicle_state_.ax;
        debug_msg_.output_throttle = throttle_cmd;
        debug_msg_.output_brake = brake_cmd_front;
        debug_msg_.max_throttle = max_thr;

        debug_pub_->publish(debug_msg_);
    }

    void ControllerNode::lateral_control()
    {
        /**
         * @brief This function is called at a fixed rate to compute the steering angle
         */
        if (!position_received || !wheel_speed_received || !path_loaded) {return;}

        // Update Lateral Control Parameters
        wheelbase_ = get_parameter("vehicle.wheelbase").as_double();
        steering_ratio_ = get_parameter("vehicle.steering_ratio").as_double();
        steering_cmd_sign_ = get_parameter("vehicle.steering_cmd_sign").as_double();
        min_steer_ = get_parameter("vehicle.min_steer").as_double();
        max_steer_ = get_parameter("vehicle.max_steer").as_double();
        min_lookahead_dist_ = get_parameter("vehicle.min_lookahead_dist").as_double();
        max_lookahead_dist_ = get_parameter("vehicle.max_lookahead_dist").as_double();
        lookahead_gain_ = get_parameter("vehicle.lookahead_gain").as_double();

        // Saturate and Translate Wheel Angle to Steering Wheel Angle
        double steering_angle = std::max(min_steer_, std::min(pure_pursuit_steering_angle, max_steer_));
        double steering_cmd_raw = steering_angle * (180.0 / M_PI) * steering_ratio_ * steering_cmd_sign_;
        double steering_cmd = steering_cmd_raw;

        // SIL can produce rapid sign flips; apply a steering slew-rate limiter in deg/s.
        const double now_sec = this->simModeEnabled
                                   ? (static_cast<double>(this->sec) + static_cast<double>(this->nsec) * 1e-9)
                                   : this->now().seconds();
        double dt = now_sec - prev_steer_time_;
        if (!std::isfinite(dt) || dt <= 1e-4) {
            dt = 0.01;
        }
        const double max_steer_rate_dps = 120.0;
        const double max_delta_cmd = max_steer_rate_dps * dt;
        if (prev_steer_time_ > 0.0) {
            steering_cmd = std::clamp(
                steering_cmd,
                prev_steering_cmd_ - max_delta_cmd,
                prev_steering_cmd_ + max_delta_cmd);
        }

        debug_msg_.steering_angle_clipped = steering_angle;
        debug_msg_.steering_cmd_raw = steering_cmd_raw;
        debug_msg_.steering_cmd_limited = steering_cmd;
        debug_msg_.steering_dt = dt;
        debug_msg_.steering_saturated = (pure_pursuit_steering_angle < min_steer_) || (pure_pursuit_steering_angle > max_steer_);
        debug_msg_.steering_rate_limited = std::fabs(steering_cmd - steering_cmd_raw) > 1e-6;
        const double alpha_dir_deadband = 0.02;
        const double steering_dir_deadband = 1.0;
        debug_msg_.steering_direction_mismatch =
            (std::fabs(debug_msg_.pp_alpha) > alpha_dir_deadband) &&
            (std::fabs(steering_cmd_raw) > steering_dir_deadband) &&
            ((debug_msg_.pp_alpha * steering_cmd_raw) < 0.0);
        prev_steering_cmd_ = steering_cmd;
        prev_steer_time_ = now_sec;
        // if (sys_state_ != SysState::SS9_DRIVING) {steering_cmd = 0;}
        vehicle_cmd_msg_.steering_cmd = steering_cmd;
        debug_msg_.output_steering = steering_cmd;
        vehicle_cmd_msg_.steering_cmd_count = rolling_counter;
        rolling_counter++;
        if (rolling_counter >= 8)
        {
            rolling_counter = 0;
        }

        if (!this->useRaptorDbwNode) {
            publishCanMessage("steering_cmd", [&](PreparedCanMessage &message) {
                auto assign = [&](std::string_view signal_name, auto value) {
                    if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
                        insertBits(message.frame.data, *signal, value);
                    }
                };
                assign("steering_motor_cmd_counter", rolling_counter);
                assign("steering_motor_ang_cmd", steering_cmd);
                assign("driver_steering_FF_cmd", 0);
                assign("driver_steering_P_cmd", 0);
                assign("driver_steering_I_cmd", 0);
                assign("driver_steering_D_cmd", 0);
            });
        } else if (this->useRaptorDbwNode || this->publish_ros_all) {
            raptor_dbw_msgs::msg::SteeringCmd ros_steering_cmd;
            ros_steering_cmd.angle_cmd = steering_cmd;
            ros_steering_cmd.enable = true;
            ros_steering_cmd.rolling_counter = rolling_counter;

            steering_cmd_pub_->publish(ros_steering_cmd);
        }
    }

    uint8_t ControllerNode::get_gear_shift_cmd()
    {
        // Sets command to current gear if engine is not on or shift attempts denied over the limit
        int MS_PER_SHIFT_CALLBACK_CALL;
        if (this->simModeEnabled){MS_PER_SHIFT_CALLBACK_CALL = 100;}
        else {MS_PER_SHIFT_CALLBACK_CALL = 10;}

        if (!engine_running_ || shifting_counter_ * MS_PER_SHIFT_CALLBACK_CALL >= shift_time_limit)
        {
            shifting_counter_ = 0;
            return current_gear_;
        }

        // Check our speed against the shift table, and see if we should go down a gear or up a gear.
        float current_speed = 2.23694 * vehicle_state_.vx;

        if (current_gear_ > min_gear && engine_speed_ < downshift_rpm[current_gear_] && current_speed < downshift_speed[current_gear_])
        {
            shifting_counter_++;
            return current_gear_ - 1;
        }
        else if (current_gear_ < max_gear && engine_speed_ > upshift_rpm[current_gear_] && current_speed > upshift_speed[current_gear_])
        {
            shifting_counter_++;
            shift_up_ = true;
            last_gear_ = current_gear_;
            return current_gear_ + 1;
        }
        else if (shift_up_ == true && current_gear_ == last_gear_)
        {
            shifting_counter_++;
            return current_gear_ + 1;
        }
        else
        {
            shifting_counter_ = 0;
            shift_up_ = false;
            return current_gear_;
        }
    }

} // namespace controller