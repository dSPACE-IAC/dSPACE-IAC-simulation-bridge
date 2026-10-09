// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "raptor_dbw_can/DbwNode.hpp"
#include <iostream>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/create_timer.hpp>

namespace raptor_dbw_can
{

namespace
{
template<typename ValueType>
bool readSignal(NewEagle::DbcMessage & message, const std::string & name, ValueType & value)
{
  auto * signal = message.GetSignal(name);
  if (!signal) {
    return false;
  }
  value = static_cast<ValueType>(signal->GetResult());
  return true;
}
}

DbwNode::DbwNode(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{
  dbcFile_ = this->declare_parameter("dbw_dbc_file", "");
  // Initializing tire report 

  for (int i = 0; i < 16; i++) {
    tire_report_msg.fl_tire_temperature.push_back(0.0);
    tire_report_msg.fr_tire_temperature.push_back(0.0);
    tire_report_msg.rl_tire_temperature.push_back(0.0);
    tire_report_msg.rr_tire_temperature.push_back(0.0);
  }

  

  // Set up Publishers
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 20);
  pub_accel_pedal_ = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>(
    "accelerator_pedal_report", 20);
  pub_steering_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringReport>("steering_report", 20);
  pub_steering_ext_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringExtendedReport>("steering_extended_report", 20);
  pub_wheel_speeds_ = this->create_publisher<raptor_dbw_msgs::msg::WheelSpeedReport>(
    "wheel_speed_report", 20);


  pub_brake_2_report_ = this->create_publisher<raptor_dbw_msgs::msg::Brake2Report>(
    "brake_2_report",
    20);

  pub_misc_do_ = this->create_publisher<npc_controller_msgs::msg::MiscReport>("misc_report_do", 10);
  pub_rc_to_ct_ = this->create_publisher<npc_controller_msgs::msg::RcToCt>("rc_to_ct", 10);
  pub_tire_report_ = this->create_publisher<npc_controller_msgs::msg::TireReport>("tire_report", 10);
  pub_pt_report_ = this->create_publisher<npc_controller_msgs::msg::PtReport>("pt_report", 10);
  pub_diagnostic_report_ = this->create_publisher<raptor_dbw_msgs::msg::DiagnosticReport>("diagnostic_report", 10);
  pub_tire_pressure_report_ = this->create_publisher<raptor_dbw_msgs::msg::TirePressureReport>(
    "tire_pressure_report", 20);
  pub_wheel_position_report_ = this->create_publisher<raptor_dbw_msgs::msg::WheelPositionReport>(
    "wheel_position_report", 20);
  pub_motec_report_ = this->create_publisher<raptor_dbw_msgs::msg::MotecReport>(
    "motec_report", 20);
  pub_gear_report_ = this->create_publisher<raptor_dbw_msgs::msg::GearReport>(
    "gear_report", 20);
  pub_low_voltage_report_ = this->create_publisher<raptor_dbw_msgs::msg::LowVoltageSystemReport>(
    "low_voltage_system_report", 20);

  // Set up Subscribers
  sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_rx", 500, std::bind(&DbwNode::recvCAN, this, std::placeholders::_1));

  sub_brake_ = this->create_subscription<raptor_dbw_msgs::msg::BrakeCmd>(
    "brake_cmd", 1, std::bind(&DbwNode::recvBrakeCmd, this, std::placeholders::_1));

  sub_accelerator_pedal_ = this->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>(
    "accelerator_pedal_cmd", 1,
    std::bind(&DbwNode::recvAcceleratorPedalCmd, this, std::placeholders::_1));

  sub_steering_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringCmd>(
    "steering_cmd", 1, std::bind(&DbwNode::recvSteeringCmd, this, std::placeholders::_1));

  sub_gear_shift_cmd_ = this->create_subscription<std_msgs::msg::UInt8>(
      "gear_cmd", 10, std::bind(&DbwNode::recvGearShiftCmd, this, std::placeholders::_1));

  sub_ct_report_ = this->create_subscription<npc_controller_msgs::msg::CtReport>(
      "ct_report", 1, std::bind(&DbwNode::recvCtReport, this, std::placeholders::_1));

  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);

  // Set up Timer
  
  // Node-clock timers: they follow /clock when use_sim_time is set, and wall time otherwise.
  timer_tire_report_ = rclcpp::create_timer(
    this->get_node_base_interface(), this->get_node_timers_interface(), this->get_clock(),
    std::chrono::milliseconds(10), std::bind(&DbwNode::timerTireCallback, this));

  timer_pt_report_ = rclcpp::create_timer(
    this->get_node_base_interface(), this->get_node_timers_interface(), this->get_clock(),
    std::chrono::milliseconds(10), std::bind(&DbwNode::timerPtCallback, this));

}

DbwNode::~DbwNode()
{
}

void DbwNode::recvCAN(const can_msgs::msg::Frame::SharedPtr msg)
{
  
  if (msg->is_rtr || msg->is_error) {
    return;
    printf("Early return");
  }

  if (msg->id >= ID_TIRE_TEMP_FL_1 && msg->id <= ID_TIRE_TEMP_RR_4) {
    auto * message = dbwDbc_.GetMessageById(msg->id);
    if (!message || msg->dlc < message->GetDlc()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid tire temperature frame 0x%X", msg->id);
      return;
    }
    message->SetFrame(msg);

    const auto block_index = static_cast<std::size_t>(msg->id - ID_TIRE_TEMP_FL_1);
    const auto wheel_index = block_index / 4;
    const auto first_temperature = (block_index % 4) * 4;
    const char * wheel_prefixes[] = {"FL", "FR", "RL", "RR"};
    std::vector<float> * temperatures[] = {
      &tire_report_msg.fl_tire_temperature,
      &tire_report_msg.fr_tire_temperature,
      &tire_report_msg.rl_tire_temperature,
      &tire_report_msg.rr_tire_temperature};

    for (std::size_t index = 0; index < 4; ++index) {
      const auto temperature_number = first_temperature + index + 1;
      const auto suffix = temperature_number < 10 ? "0" : "";
      const std::string signal_name = std::string(wheel_prefixes[wheel_index]) +
        "_Tire_Temp_" + suffix + std::to_string(temperature_number);
      if (!readSignal(*message, signal_name, temperatures[wheel_index]->at(temperature_number - 1))) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Missing DBC signal %s in frame 0x%X", signal_name.c_str(), msg->id);
        return;
      }
    }
    tire_report_msg.stamp = msg->header.stamp;
    return;
  }

  switch (msg->id) {
    case ID_WHEEL_SPEED_REPORT_DO:
      {
        auto * message = dbwDbc_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);
        if (!message) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DBC message lookup failed for ID_WHEEL_SPEED_REPORT_DO (0x%X)", ID_WHEEL_SPEED_REPORT_DO);
          return;
        }
        
        if (msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DLC too small for ID 0x%X: got %u expected >= %u", msg->id, msg->dlc, message->GetDlc());
          return;
        }

        message->SetFrame(msg);
        auto * sig_fl = message->GetSignal("wheel_speed_FL");
        auto * sig_fr = message->GetSignal("wheel_speed_FR");
        auto * sig_rl = message->GetSignal("wheel_speed_RL");
        auto * sig_rr = message->GetSignal("wheel_speed_RR");

        if (!sig_fl || !sig_fr || !sig_rl || !sig_rr) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Missing one or more signals in DBC for ID_WHEEL_SPEED_REPORT_DO");
          return;
        }

        raptor_dbw_msgs::msg::WheelSpeedReport out;
        out.header.stamp = msg->header.stamp;
        out.front_left = sig_fl->GetResult();
        out.front_right = sig_fr->GetResult();
        out.rear_left = sig_rl->GetResult();
        out.rear_right = sig_rr->GetResult();
        motec_report_msg_.header.stamp = msg->header.stamp;
        motec_report_msg_.wheel_speed_front_left = sig_fl->GetResult();
        motec_report_msg_.wheel_speed_front_right = sig_fr->GetResult();
        motec_report_msg_.wheel_speed_rear_left = sig_rl->GetResult();
        motec_report_msg_.wheel_speed_rear_right = sig_rr->GetResult();
        pub_wheel_speeds_->publish(out);
        break;
      }

    case ID_BRAKE_PRESSURE_REPORT_DO:
      {
        auto * message = dbwDbc_.GetMessageById(ID_BRAKE_PRESSURE_REPORT_DO);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid brake pressure report frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        raptor_dbw_msgs::msg::Brake2Report out;
        out.header.stamp = msg->header.stamp;
        double front_pressure_kpa = 0.0;
        double rear_pressure_kpa = 0.0;
        if (!readSignal(*message, "brake_pressure_fdbk_front", front_pressure_kpa) ||
          !readSignal(*message, "brake_pressure_fdbk_rear", rear_pressure_kpa) ||
          !readSignal(*message, "brk_pressure_fdbk_counter", out.rolling_counter))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing brake pressure report signal in DBC");
          return;
        }
        out.front_brake_pressure = static_cast<float>(front_pressure_kpa / 100.0);
        out.rear_brake_pressure = static_cast<float>(rear_pressure_kpa / 100.0);
        pub_brake_2_report_->publish(out);
        break;
      }

    case ID_ACCELERATOR_REPORT_DO:
      {
        auto * message = dbwDbc_.GetMessageById(ID_ACCELERATOR_REPORT_DO);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid accelerator report frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        raptor_dbw_msgs::msg::AcceleratorPedalReport out;
        out.header.stamp = msg->header.stamp;
        if (!readSignal(*message, "acc_pedal_fdbk", out.pedal_input) ||
          !readSignal(*message, "acc_pedal_fdbk_counter", out.rolling_counter))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing accelerator report signal in DBC");
          return;
        }
        pub_accel_pedal_->publish(out);
        break;
      }

    case ID_STEERING_REPORT_DO:
      {
        auto * message = dbwDbc_.GetMessageById(ID_STEERING_REPORT_DO);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid steering report frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        raptor_dbw_msgs::msg::SteeringReport out;
        out.header.stamp = msg->header.stamp;
        if (!readSignal(*message, "steering_motor_fdbk_counter", out.rolling_counter)) {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing steering report counter in DBC");
          return;
        }
        pub_steering_->publish(out);
        break;
      }

    case ID_STEERING_REPORT_EXTD:
      {
        auto * message = dbwDbc_.GetMessageById(ID_STEERING_REPORT_EXTD);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid extended steering report frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        raptor_dbw_msgs::msg::SteeringExtendedReport out;
        out.header.stamp = msg->header.stamp;
        if (!readSignal(*message, "primary_steering_angle_fbk", out.steering_motor_ang_1) ||
          !readSignal(*message, "secondary_steering_ang_fdbk", out.steering_motor_ang_2) ||
          !readSignal(*message, "average_steering_ang_fdbk", out.steering_motor_ang_3))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing extended steering report signal in DBC");
          return;
        }
        pub_steering_ext_->publish(out);
        break;
      }

    case ID_DIAGNOSTIC_REPORT:
      {
        auto * message = dbwDbc_.GetMessageById(ID_DIAGNOSTIC_REPORT);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid diagnostic report frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        raptor_dbw_msgs::msg::DiagnosticReport out;
        out.header.stamp = msg->header.stamp;
        if (!readSignal(*message, "sd_system_warning", out.sd_system_warning) ||
          !readSignal(*message, "sd_system_failure", out.sd_system_failure) ||
          !readSignal(*message, "sd_brake_warning1", out.sd_brake_warning1) ||
          !readSignal(*message, "sd_brake_warning2", out.sd_brake_warning2) ||
          !readSignal(*message, "sd_brake_warning3", out.sd_brake_warning3) ||
          !readSignal(*message, "sd_steer_warning1", out.sd_steer_warning1) ||
          !readSignal(*message, "sd_steer_warning2", out.sd_steer_warning2) ||
          !readSignal(*message, "sd_steer_warning3", out.sd_steer_warning3) ||
          !readSignal(*message, "motec_warning", out.motec_warning) ||
          !readSignal(*message, "est1_oos_front_brk", out.front_brk_failure) ||
          !readSignal(*message, "est2_oos_rear_brk", out.rear_brk_failure) ||
          !readSignal(*message, "est3_low_eng_speed", out.low_eng_speed) ||
          !readSignal(*message, "est4_sd_comms_loss", out.sd_comms_loss) ||
          !readSignal(*message, "est5_motec_comms_loss", out.motec_comms_loss) ||
          !readSignal(*message, "est6_sd_ebrake", out.sd_ebrake) ||
          !readSignal(*message, "adlink_hb_lost", out.adlink_hb_lost) ||
          !readSignal(*message, "rc_lost", out.rc_lost))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing diagnostic report signal in DBC");
          return;
        }
        motec_report_msg_.header.stamp = msg->header.stamp;
        readSignal(*message, "motec_warning", motec_report_msg_.warning_source);
        pub_diagnostic_report_->publish(out);
        break;
      }

    case ID_WHEEL_STRAIN_GAUGE:
      {
        auto * message = dbwDbc_.GetMessageById(ID_WHEEL_STRAIN_GAUGE);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid wheel strain gauge frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        if (!readSignal(*message, "wheel_strain_gauge_FL", tire_report_msg.fl_wheel_load) ||
          !readSignal(*message, "wheel_strain_gauge_FR", tire_report_msg.fr_wheel_load) ||
          !readSignal(*message, "wheel_strain_gauge_RL", tire_report_msg.rl_wheel_load) ||
          !readSignal(*message, "wheel_strain_gauge_RR", tire_report_msg.rr_wheel_load))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing wheel strain gauge signal in DBC");
          return;
        }
        tire_report_msg.stamp = msg->header.stamp;
        break;
      }

    case ID_WHEEL_POTENTIOMETER:
      {
        auto * message = dbwDbc_.GetMessageById(ID_WHEEL_POTENTIOMETER);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid wheel potentiometer frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        raptor_dbw_msgs::msg::WheelPositionReport out;
        out.header.stamp = msg->header.stamp;
        const char * signal_names[] = {
          "wheel_potentiometer_FL", "wheel_potentiometer_FR",
          "wheel_potentiometer_RL", "wheel_potentiometer_RR"};
        int16_t * positions[] = {&out.front_left, &out.front_right, &out.rear_left, &out.rear_right};
        float * dampers[] = {
          &tire_report_msg.fl_damper_linear_potentiometer,
          &tire_report_msg.fr_damper_linear_potentiometer,
          &tire_report_msg.rl_damper_linear_potentiometer,
          &tire_report_msg.rr_damper_linear_potentiometer};
        for (std::size_t index = 0; index < 4; ++index) {
          auto * signal = message->GetSignal(signal_names[index]);
          if (!signal) {
            RCLCPP_ERROR_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "Missing wheel potentiometer signal %s", signal_names[index]);
            return;
          }
          *positions[index] = static_cast<int16_t>(std::lround(signal->GetResult() / signal->GetGain()));
          *dampers[index] = static_cast<float>(signal->GetResult());
        }
        tire_report_msg.stamp = msg->header.stamp;
        pub_wheel_position_report_->publish(out);
        break;
      }

    case ID_TIRE_PRESSURE_FL:
    case ID_TIRE_PRESSURE_FR:
    case ID_TIRE_PRESSURE_RL:
    case ID_TIRE_PRESSURE_RR:
      {
        auto * message = dbwDbc_.GetMessageById(msg->id);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid tire pressure frame 0x%X", msg->id);
          return;
        }

        message->SetFrame(msg);
        const std::size_t wheel_index = static_cast<std::size_t>(msg->id - ID_TIRE_PRESSURE_FL);
        const char * wheel_prefixes[] = {"FL", "FR", "RL", "RR"};
        const std::string prefix = wheel_prefixes[wheel_index];
        double pressure = 0.0;
        double gauge_pressure = 0.0;
        if (!readSignal(*message, prefix + "_Tire_Pressure", pressure) ||
          !readSignal(*message, prefix + "_Tire_Pressure_Gauge", gauge_pressure))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing tire pressure signal for %s", prefix.c_str());
          return;
        }

        const float pressure_kpa = static_cast<float>(pressure * 0.1);
        const float gauge_pressure_kpa = static_cast<float>(gauge_pressure * 0.1);
        switch (wheel_index) {
          case 0:
            tire_report_msg.fl_tire_pressure = pressure_kpa;
            tire_report_msg.fl_tire_pressure_gauge = gauge_pressure_kpa;
            tire_pressure_report_msg_.front_left = gauge_pressure_kpa;
            break;
          case 1:
            tire_report_msg.fr_tire_pressure = pressure_kpa;
            tire_report_msg.fr_tire_pressure_gauge = gauge_pressure_kpa;
            tire_pressure_report_msg_.front_right = gauge_pressure_kpa;
            break;
          case 2:
            tire_report_msg.rl_tire_pressure = pressure_kpa;
            tire_report_msg.rl_tire_pressure_gauge = gauge_pressure_kpa;
            tire_pressure_report_msg_.rear_left = gauge_pressure_kpa;
            break;
          case 3:
            tire_report_msg.rr_tire_pressure = pressure_kpa;
            tire_report_msg.rr_tire_pressure_gauge = gauge_pressure_kpa;
            tire_pressure_report_msg_.rear_right = gauge_pressure_kpa;
            break;
        }
        tire_report_msg.stamp = msg->header.stamp;
        tire_pressure_report_msg_.header.stamp = msg->header.stamp;
        pub_tire_pressure_report_->publish(tire_pressure_report_msg_);
        break;
      }

    case ID_MISC_REPORT_DO:
      {
        auto * message = dbwDbc_.GetMessageById(ID_MISC_REPORT_DO);
        if (!message) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DBC message lookup failed for ID_MISC_REPORT_DO (0x%X)", ID_MISC_REPORT_DO);
          return;
        }
        
        if (msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DLC too small for ID 0x%X: got %u expected >= %u", msg->id, msg->dlc, message->GetDlc());
          return;
        }

        message->SetFrame(msg);
        auto * sig_sys_state = message->GetSignal("sys_state");
        auto * sig_safety_switch_state = message->GetSignal("safety_switch_state");
        auto * sig_mode_switch_state = message->GetSignal("mode_switch_state");
        auto * sig_battery_voltage = message->GetSignal("battery_voltage");

        if (!sig_sys_state || !sig_safety_switch_state || !sig_mode_switch_state || !sig_battery_voltage) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Missing one or more signals in DBC for ID_MISC_REPORT_DO");
          return;
        }

        npc_controller_msgs::msg::MiscReport out;
        out.stamp = msg->header.stamp;
        out.sys_state = sig_sys_state->GetResult();
        out.safety_switch_state = sig_safety_switch_state->GetResult();
        out.mode_switch_state = sig_mode_switch_state->GetResult();
        out.battery_voltage = sig_battery_voltage->GetResult();
        motec_report_msg_.header.stamp = msg->header.stamp;
        motec_report_msg_.ecu_battery_voltage = sig_battery_voltage->GetResult();
        raptor_dbw_msgs::msg::LowVoltageSystemReport low_voltage;
        low_voltage.header.stamp = msg->header.stamp;
        low_voltage.dbw_battery_volts = sig_battery_voltage->GetResult();
        pub_misc_do_->publish(out);
        pub_low_voltage_report_->publish(low_voltage);
        publishRcToCt(msg->header.stamp);
        break;
      }

    case ID_MARELLI_REPORT_1:
      {
        auto * message = dbwDbc_.GetMessageById(ID_MARELLI_REPORT_1);
        if (!message) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DBC message lookup failed for ID_MARELLI_REPORT_1 (0x%X)", ID_MARELLI_REPORT_1);
          return;
        }

        if (msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DLC too small for ID 0x%X: got %u expected >= %u", msg->id, msg->dlc, message->GetDlc());
          return;
        }

        message->SetFrame(msg);
        auto * sig_marelli_track_flag = message->GetSignal("marelli_track_flag");
        auto * sig_marelli_vehicle_flag = message->GetSignal("marelli_vehicle_flag");

        if (!sig_marelli_track_flag || !sig_marelli_vehicle_flag) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Missing one or more signals in DBC for ID_MARELLI_REPORT_1");
          return;
        }

        rc_to_ct_msg_.track_flag = sig_marelli_track_flag->GetResult();
        rc_to_ct_msg_.veh_flag = sig_marelli_vehicle_flag->GetResult();
        publishRcToCt(msg->header.stamp);
        break;
      }

    case ID_RC_TO_CT:
      {
        auto * message = dbwDbc_.GetMessageById(ID_RC_TO_CT);
        if (!message) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DBC message lookup failed for ID_RC_TO_CT (0x%X)", ID_RC_TO_CT);
          return;
        }
        
        if (msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DLC too small for ID 0x%X: got %u expected >= %u", msg->id, msg->dlc, message->GetDlc());
          return;
        }

        message->SetFrame(msg);
        auto * sig_track_flag = message->GetSignal("track_flag");
        auto * sig_veh_flag = message->GetSignal("veh_flag");
        auto * sig_veh_rank = message->GetSignal("veh_rank");
        auto * sig_lap_count = message->GetSignal("lap_count");
        auto * sig_lap_distance = message->GetSignal("lap_distance");
        auto * sig_round_target_speed = message->GetSignal("round_target_speed");
        auto * sig_base_to_car_heartbeat = message->GetSignal("base_to_car_heartbeat");

        if (!sig_track_flag || !sig_veh_flag || !sig_veh_rank || !sig_lap_count ||
          !sig_lap_distance || !sig_round_target_speed || !sig_base_to_car_heartbeat)
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Missing one or more signals in DBC for ID_RC_TO_CT");
          return;
        }
        rc_to_ct_msg_.track_flag = sig_track_flag->GetResult();
        rc_to_ct_msg_.veh_flag = sig_veh_flag->GetResult();
        rc_to_ct_msg_.veh_rank = sig_veh_rank->GetResult();
        rc_to_ct_msg_.lap_count = sig_lap_count->GetResult();
        rc_to_ct_msg_.lap_distance = sig_lap_distance->GetResult();
        rc_to_ct_msg_.target_speed = sig_round_target_speed->GetResult();
        rc_to_ct_msg_.rolling_counter = sig_base_to_car_heartbeat->GetResult();

        publishRcToCt(msg->header.stamp);
        break;
      }

    case ID_PT_REPORT_1:
      {
        auto * message = dbwDbc_.GetMessageById(ID_PT_REPORT_1);
        if (!message) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DBC message lookup failed for ID_PT_REPORT_1 (0x%X)", ID_PT_REPORT_1);
          return;
        }
        
        if (msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "DLC too small for ID 0x%X: got %u expected >= %u", msg->id, msg->dlc, message->GetDlc());
          return;
        }

        message->SetFrame(msg);
        auto * sig_throttle_position = message->GetSignal("throttle_position");
        auto * sig_engine_run_switch_status = message->GetSignal("engine_run_switch");
        auto * sig_current_gear = message->GetSignal("current_gear");
        auto * sig_engine_rpm = message->GetSignal("engine_speed_rpm");
        auto * sig_vehicle_speed_kmph = message->GetSignal("vehicle_speed_kmph");

        if (!sig_throttle_position || !sig_engine_run_switch_status || !sig_current_gear || !sig_engine_rpm || !sig_vehicle_speed_kmph) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Missing one or more signals in DBC for ID_PT_REPORT_1");
          return;
        }

        pt_report_msg.stamp = msg->header.stamp;
        pt_report_msg.throttle_position = sig_throttle_position->GetResult();
        pt_report_msg.engine_run_switch_status = sig_engine_run_switch_status->GetResult();
        pt_report_msg.current_gear = sig_current_gear->GetResult();
        pt_report_msg.engine_rpm = sig_engine_rpm->GetResult();
        pt_report_msg.vehicle_speed_kmph = sig_vehicle_speed_kmph->GetResult();
        if (!readSignal(*message, "engine_state", pt_report_msg.engine_on_status) ||
          !readSignal(*message, "gear_shift_status", pt_report_msg.gear_shift_status))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing PT report 1 signal in DBC");
          return;
        }
        motec_report_msg_.header.stamp = msg->header.stamp;
        motec_report_msg_.engine_speed = static_cast<uint16_t>(sig_engine_rpm->GetResult());
        motec_report_msg_.throttle_position = sig_throttle_position->GetResult();
        motec_report_msg_.throttle_pedal = static_cast<uint16_t>(sig_throttle_position->GetResult());
        motec_report_msg_.fuel_pump_state = sig_engine_run_switch_status->GetResult();
        motec_report_msg_.engine_state = pt_report_msg.engine_on_status;
        motec_report_msg_.gear = sig_current_gear->GetResult();
        motec_report_msg_.gear_shift = pt_report_msg.gear_shift_status;
        motec_report_msg_.vehicle_speed = sig_vehicle_speed_kmph->GetResult();
        raptor_dbw_msgs::msg::GearReport gear_report;
        gear_report.header.stamp = msg->header.stamp;
        gear_report.state.gear = static_cast<uint8_t>(sig_current_gear->GetResult());
        gear_report.reject = pt_report_msg.gear_shift_status == 2;
        pub_gear_report_->publish(gear_report);
        break;
      }

    case ID_PT_REPORT_2:
      {
        auto * message = dbwDbc_.GetMessageById(ID_PT_REPORT_2);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid PT report 2 frame 0x%X", msg->id);
          return;
        }
        message->SetFrame(msg);
        if (!readSignal(*message, "fuel_pressure_kPa", pt_report_msg.fuel_pressure) ||
          !readSignal(*message, "engine_oil_pressure_kPa", pt_report_msg.engine_oil_pressure) ||
          !readSignal(*message, "coolant_temperature", pt_report_msg.engine_coolant_temperature) ||
          !readSignal(*message, "transmission_temperature", pt_report_msg.transmission_oil_temperature) ||
          !readSignal(*message, "transmission_pressure_kPa", pt_report_msg.transmission_oil_pressure))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing PT report 2 signal in DBC");
          return;
        }
        pt_report_msg.stamp = msg->header.stamp;
        motec_report_msg_.header.stamp = msg->header.stamp;
        motec_report_msg_.fuel_pressure_sensor = pt_report_msg.fuel_pressure;
        motec_report_msg_.engine_oil_pressure = pt_report_msg.engine_oil_pressure;
        motec_report_msg_.coolant_temperature = pt_report_msg.engine_coolant_temperature;
        break;
      }

    case ID_PT_REPORT_3:
      {
        auto * message = dbwDbc_.GetMessageById(ID_PT_REPORT_3);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid PT report 3 frame 0x%X", msg->id);
          return;
        }
        message->SetFrame(msg);
        if (!readSignal(*message, "engine_oil_temperature", pt_report_msg.engine_oil_temperature) ||
          !readSignal(*message, "torque_wheels", pt_report_msg.torque_wheels))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing PT report 3 signal in DBC");
          return;
        }
        pt_report_msg.stamp = msg->header.stamp;
        motec_report_msg_.header.stamp = msg->header.stamp;
        motec_report_msg_.engine_oil_temperature = pt_report_msg.engine_oil_temperature;
        motec_report_msg_.torque_wheels = pt_report_msg.torque_wheels;
        if (!readSignal(*message, "driver_traction_aim_swicth_fbk", motec_report_msg_.driver_switch_1) ||
          !readSignal(*message, "driver_traction_range_switch_fbk", motec_report_msg_.driver_switch_2))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing PT report 3 driver-switch signal in DBC");
          return;
        }
        break;
      }

    case ID_PT_REPORT_4:
      {
        auto * message = dbwDbc_.GetMessageById(ID_PT_REPORT_4);
        if (!message || msg->dlc < message->GetDlc()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid PT report 4 frame 0x%X", msg->id);
          return;
        }
        message->SetFrame(msg);
        if (!readSignal(*message, "intake_manifold_press_kPa", pt_report_msg.map_sensor)) {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing PT report 4 intake manifold pressure in DBC");
          return;
        }
        pt_report_msg.stamp = msg->header.stamp;
        motec_report_msg_.header.stamp = msg->header.stamp;
        if (!readSignal(*message, "boost_aim_psi", motec_report_msg_.boost_aim) ||
          !readSignal(*message, "boost_press_psi", motec_report_msg_.boost_pressure) ||
          !readSignal(*message, "intake_manifold_press_kPa", motec_report_msg_.inlet_manifold_pressure) ||
          !readSignal(*message, "intake_air_temp_degC", motec_report_msg_.inlet_air_temperature))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Missing PT report 4 signal in DBC");
          return;
        }
        break;
      }
  }
}

void DbwNode::publishRcToCt(const builtin_interfaces::msg::Time & stamp)
{
  rc_to_ct_msg_.stamp = stamp;

  pub_rc_to_ct_->publish(rc_to_ct_msg_);
}


void DbwNode::recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg)
{
  auto* message = dbwDbc_.GetMessage("brake_pressure_cmd");
  if (!message) return;

  auto* sig_f = message->GetSignal("F_brake_pressure_cmd");
  auto* sig_r = message->GetSignal("R_brake_pressure_cmd");
  auto* sig_ctr = message->GetSignal("brk_pressure_cmd_counter");

  if (!sig_f || !sig_r || !sig_ctr) return;

  sig_f->SetResult(msg->pedal_cmd * 0.5);
  sig_r->SetResult(msg->pedal_cmd * 0.5);
  sig_ctr->SetResult(msg->rolling_counter);

  pub_can_->publish(message->GetFrame());
}

void DbwNode::recvAcceleratorPedalCmd(const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("accelerator_cmd");


  message->GetSignal("acc_pedal_cmd")->SetResult(msg->pedal_cmd);
  message->GetSignal("acc_pedal_cmd_counter")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void DbwNode::recvSteeringCmd(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("steering_cmd");

  message->GetSignal("steering_motor_ang_cmd")->SetResult(msg->angle_cmd);
  message->GetSignal("steering_motor_cmd_counter")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void DbwNode::recvCtReport(const npc_controller_msgs::msg::CtReport::SharedPtr msg) {
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("ct_report");
  message->GetSignal("track_cond_ack")->SetResult(msg->track_flag_ack); 
  message->GetSignal("veh_sig_ack")->SetResult(msg->veh_flag_ack);
  message->GetSignal("ct_state")->SetResult(msg->ct_state);
  message->GetSignal("ct_state_rolling_counter")->SetResult(msg->rolling_counter);
  message->GetSignal("veh_num")->SetResult(msg->veh_num);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void DbwNode::recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg) {

  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("gear_shift_cmd");
  message->GetSignal("desired_gear")->SetResult(msg->data);
  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void DbwNode::timerTireCallback() {
    pub_tire_report_->publish(tire_report_msg);
}

void DbwNode::timerPtCallback() {
    pub_pt_report_->publish(pt_report_msg);
  pub_motec_report_->publish(motec_report_msg_);
}

}  // namespace raptor_dbw_can
