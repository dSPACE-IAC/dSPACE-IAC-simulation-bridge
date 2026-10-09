#include "npc_controller.hpp"

#include <chrono>
#include <cmath>

namespace controller
{

    void ControllerNode::bestpos_callback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
    {
        applyBestPosMessage(msg);
    }

    void ControllerNode::applyBestPosMessage(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
    {
        double lat = msg->lat;
        double lon = msg->lon;
        double height = msg->hgt;
        double x, y, z;
        gps_map_.Forward(lat, lon, height, x, y, z);

        double vehicle_x;
        double vehicle_y;
        double vehicle_z;
        double vehicle_yaw;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            // Calculate Heading from Position Difference
            double dx = x - prev_x_;
            double dy = y - prev_y_;
            vehicle_yaw = std::atan2(dy, dx);
            prev_x_ = x;
            prev_y_ = y;

            // Convert from Antenna to Rear Axle of Vehicle
            vehicle_x = x - 3.175 * std::cos(vehicle_yaw);
            vehicle_y = y - 3.175 * std::sin(vehicle_yaw);
            vehicle_z = z;

            vehicle_state_.yaw = vehicle_yaw;
            vehicle_state_.x = vehicle_x;
            vehicle_state_.y = vehicle_y;
            vehicle_state_.z = vehicle_z;
            previous_state_ = vehicle_state_;
            position_received = true;
        }
        RCLCPP_WARN_ONCE(get_logger(),"position_received");

        // Publish Odometry
        odometry_msg_.header.stamp = this->now();
        odometry_msg_.header.frame_id = "map";
        odometry_msg_.child_frame_id = "base_link";
        odometry_msg_.pose.pose.position.x = vehicle_x;
        odometry_msg_.pose.pose.position.y = vehicle_y;
        odometry_msg_.pose.pose.position.z = vehicle_z;

        double cy = std::cos(vehicle_yaw * 0.5);
        double sy = std::sin(vehicle_yaw * 0.5);

        odometry_msg_.pose.pose.orientation.x = 0.0;
        odometry_msg_.pose.pose.orientation.y = 0.0;
        odometry_msg_.pose.pose.orientation.z = sy;
        odometry_msg_.pose.pose.orientation.w = cy;

        odometry_pub_->publish(odometry_msg_);
    }

    void ControllerNode::wheel_speed_callback()
    {
        double fl = this->ws_front_left;
        double fr = this->ws_front_right;
        double rl = this->ws_rear_left;
        double rr = this->ws_rear_right;
        double avg_ws = (fl + fr + rl + rr) / 4.0 / 3.6;

        const double current_time = this->now().seconds() + this->now().nanoseconds() * 1e-9;

        double dt = current_time - prev_time_;
        double raw_acceleration = (avg_ws - previous_state_.vx) / dt;
        double accel = vel_filter_.processSample(static_cast<float>(raw_acceleration));
        previous_state_ = vehicle_state_;
        prev_time_ = current_time;
        vehicle_state_.vx = avg_ws; // Convert to m/s
        vehicle_state_.ax = accel;

        double vsquared = vehicle_state_.vx * vehicle_state_.vx;

        aerodynamic_drag_force_ = 0.5 * vsquared * AIR_DENSITY * AERO_DRAG_COEF * AERO_CROSS_AREA;
        rear_rolling_decel_ = (0.01 * 2) * (9.81);
        front_rolling_decel_ = (0.008 * 2) * (9.81);
        non_brake_decel_ = (rear_rolling_decel_ + front_rolling_decel_) + (aerodynamic_drag_force_ + (engine_braking_decel / REAR_WHEEL_RAD)) / VEHICLE_MASS_KG;
        non_brake_decel_ = ((vehicle_state_.vx > 10.0) ? non_brake_decel_ : 0.0);

        wheel_speed_received = true;
    }

    void ControllerNode::wheel_speed_callback_ros_msg(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        double fl = msg->front_left;
        double fr = msg->front_right;
        double rl = msg->rear_left;
        double rr = msg->rear_right;
        double avg_ws = (fl + fr + rl + rr) / 4.0 / 3.6;

        const double current_time = this->now().seconds() + this->now().nanoseconds() * 1e-9;

        double dt = current_time - prev_time_;
        double raw_acceleration = (avg_ws - previous_state_.vx) / dt;
        double accel = vel_filter_.processSample(static_cast<float>(raw_acceleration));
        previous_state_ = vehicle_state_;
        prev_time_ = current_time;
        vehicle_state_.vx = avg_ws; // Convert to m/s
        vehicle_state_.ax = accel;

        double vsquared = vehicle_state_.vx * vehicle_state_.vx;

        aerodynamic_drag_force_ = 0.5 * vsquared * AIR_DENSITY * AERO_DRAG_COEF * AERO_CROSS_AREA;
        rear_rolling_decel_ = (0.01 * 2) * (9.81);
        front_rolling_decel_ = (0.008 * 2) * (9.81);
        non_brake_decel_ = (rear_rolling_decel_ + front_rolling_decel_) + (aerodynamic_drag_force_ + (engine_braking_decel / REAR_WHEEL_RAD)) / VEHICLE_MASS_KG;
        non_brake_decel_ = ((vehicle_state_.vx > 10.0) ? non_brake_decel_ : 0.0);

        wheel_speed_received = true;
    }

    void ControllerNode::receiveCtInput(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        ct_input_ = msg->data;
    }

    void ControllerNode::receiveFlags()
    {
        track_flag_ = int2tf(this->track_flag);
        vehicle_flag_ = int2vf(this->veh_flag);
        target_speed_ = this->round_target_speed;
        sys_state_ = int2sys(this->sys_state);
    }

    void ControllerNode::receiveFlags_ros_msg(const npc_controller_msgs::msg::RcToCt::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        track_flag_ = int2tf(msg->track_flag);
        vehicle_flag_ = int2vf(msg->veh_flag);
        target_speed_ = msg->target_speed;
    }

    void ControllerNode::receiveSysState_ros_msg(const npc_controller_msgs::msg::MiscReport::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        sys_state_ = int2sys(msg->sys_state);
    }

    void ControllerNode::receivePtReport_ros_msg(const npc_controller_msgs::msg::PtReport::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        current_gear_ = msg->current_gear;
        engine_speed_ = msg->engine_rpm;
        engine_running_ = bool( msg->engine_rpm > 500 );
        reported_throttle_ = msg->throttle_position;
        engine_braking_decel = ((engine_speed_ > 1300 && vehicle_state_.vx > 5.0 && reported_throttle_ < 5.0) ? (30 * GEAR_RATIOS[current_gear_] * FINAL_DRIVE_RATIO) : 0.0);
    }

    void ControllerNode::receivePtReport()
    {
        current_gear_ = this->current_gear;
        engine_speed_ = this->engine_rpm;
        engine_running_ = bool(this->engine_rpm > 500);
        reported_throttle_ = this->throttle_position;
        engine_braking_decel = ((engine_speed_ > 1300 && vehicle_state_.vx > 5.0 && reported_throttle_ < 5.0) ? (30 * GEAR_RATIOS[current_gear_] * FINAL_DRIVE_RATIO) : 0.0);
    }

} // namespace controller