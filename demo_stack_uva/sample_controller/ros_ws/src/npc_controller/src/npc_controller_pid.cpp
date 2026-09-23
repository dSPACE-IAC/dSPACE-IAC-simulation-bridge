#include "npc_controller.hpp"

#include <algorithm>
#include <cmath>

namespace controller
{

    double ControllerNode::calc_acceleration(double set_point, SimControlInputs *inputs)
    {
        const VehicleState &control_state = inputs ? inputs->vehicle_state : vehicle_state_;
        const double control_time = inputs ? inputs->sim_time :
            (this->simModeEnabled
                 ? static_cast<double>(this->sec) + static_cast<double>(this->nsec) * 1e-9
                 : this->now().seconds() + this->now().nanoseconds() * 1e-9);

        // removes oldest instant vel error from vector
        if (vel_error_arr_.size() >= 10)
        {
            vel_integral_error_ -= vel_error_arr_[0];
            vel_error_arr_.erase(vel_error_arr_.begin());
        }

        // Calculate Acceleration Required using PID
        vel_integral_error_ += prev_vel_error_ / 10.0;
        vel_error_arr_.push_back(prev_vel_error_ / 10.0);

        double vel_error = set_point - control_state.vx;
        double dt;
        double vel_dt = 0.01;
        double acc_dt = 0.01;
        dt = control_time - prev_vel_time_;
        if (!std::isfinite(dt) || dt <= 1e-4)
            dt = 0.01;
        vel_dt = dt;
        double vel_derivative_error_ = (vel_error - prev_vel_error_) / dt;
        if (!std::isfinite(vel_derivative_error_))
            vel_derivative_error_ = 0.0;

        double des_accel = (vel_kp_ * vel_error + vel_ki_ * vel_integral_error_ + vel_kd_ * vel_derivative_error_);
        prev_vel_error_ = vel_error;
        prev_vel_time_ = control_time;
        // removes oldest instant vel error from vector
        if (acc_error_arr_.size() >= 10)
        {
            acc_integral_error_ -= acc_error_arr_[0];
            acc_error_arr_.erase(acc_error_arr_.begin());
        }

        // Calculate Throttle using PID
        acc_integral_error_ += prev_acc_error_ / double(10);
        acc_error_arr_.push_back(prev_acc_error_ / double(10));

        acc_error_ = des_accel - control_state.ax;
        dt = control_time - prev_acc_time_;

        if (!std::isfinite(dt) || dt <= 1e-4)
            dt = 0.01;
        acc_dt = dt;
        acc_derivative_error_ = (acc_error_ - prev_acc_error_) / dt;
        if (!std::isfinite(acc_derivative_error_))
            acc_derivative_error_ = 0.0;

        debug_msg_.vel_pid_dt = vel_dt;
        debug_msg_.acc_pid_dt = acc_dt;
        debug_msg_.vel_derivative_error = vel_derivative_error_;
        debug_msg_.acc_derivative_error = acc_derivative_error_;

        prev_acc_error_ = acc_error_;

        prev_acc_time_ = control_time;

        debug_msg_.vel_p = vel_kp_ * vel_error;
        debug_msg_.vel_i = vel_ki_ * vel_integral_error_;
        debug_msg_.vel_d = vel_kd_ * vel_derivative_error_;
        debug_msg_.thr_p = throttle_kp_ * acc_error_;
        debug_msg_.thr_i = throttle_ki_ * acc_integral_error_; // NaN
        debug_msg_.thr_d = throttle_kd_ * acc_derivative_error_;
        debug_msg_.brk_p = brake_kp_ * acc_error_;
        debug_msg_.brk_i = brake_ki_ * acc_integral_error_; // NaN
        debug_msg_.brk_d = brake_kd_ * acc_derivative_error_;
        debug_msg_.error_acceleration = acc_error_;
        debug_msg_.error_velocity = vel_error;

        return des_accel;
    }

    void ControllerNode::calc_throttle(double desired_acceleration, SimControlInputs *inputs)
    {
        VehicleState &control_state = inputs ? inputs->vehicle_state : vehicle_state_;
        const double control_non_brake_decel = inputs ? inputs->non_brake_decel : non_brake_decel_;

        // Calculate Deadband
        double db = -control_non_brake_decel;
        if (control_state.brake > 250.0)
        {
            control_state.throttle = 0.0;
        }
        else if (desired_acceleration > db || (desired_acceleration > 0.1 && control_state.vx < 5.0))
        {
            double delta_throttle = (throttle_kp_ * acc_error_ + throttle_ki_ * acc_integral_error_ + throttle_kd_ * acc_derivative_error_);
            if (!std::isfinite(delta_throttle))
                delta_throttle = 0.0;
            debug_msg_.throttle_delta_cmd = delta_throttle;
            control_state.throttle += delta_throttle;
            control_state.throttle = std::clamp(control_state.throttle, min_throttle_, max_throttle_);
        }
        else
        {
            debug_msg_.throttle_delta_cmd = 0.0;
            control_state.throttle = 0.0;
        }
    }

    void ControllerNode::calc_brake(double desired_acceleration, SimControlInputs *inputs)
    {
        VehicleState &control_state = inputs ? inputs->vehicle_state : vehicle_state_;
        const double control_non_brake_decel = inputs ? inputs->non_brake_decel : non_brake_decel_;

        // Calculate Deadband
        double db = -control_non_brake_decel + 0.05;
        double brake_setpoint = (acc_error_ - db);
        debug_msg_.brake_deadband = db;
        if (control_state.vx < 1.0 && desired_acceleration < 0.0)
        {
            control_state.brake = 0.0;
        }
        else if (desired_acceleration < db)
        {

            control_state.brake = -(desired_acceleration - db) * VEHICLE_MASS_KG * REAR_WHEEL_RAD;
            control_state.brake += -(brake_kp_ * brake_setpoint + brake_ki_ * acc_integral_error_ + brake_kd_ * acc_derivative_error_) * this->REAR_WHEEL_RAD * this->VEHICLE_MASS_KG;
        }
        else
        {
            control_state.brake = 0.0;
        }
    }

} // namespace controller