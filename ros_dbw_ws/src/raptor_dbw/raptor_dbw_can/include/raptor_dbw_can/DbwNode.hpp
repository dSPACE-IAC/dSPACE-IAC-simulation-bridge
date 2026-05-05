// Copyright (c) 2020 New Eagle, All rights reserved.
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

#ifndef RAPTOR_DBW_CAN__DBWNODE_HPP_
#define RAPTOR_DBW_CAN__DBWNODE_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/actuator_control_mode.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/brake_report.hpp>
#include <raptor_dbw_msgs/msg/driver_input_report.hpp>
#include <raptor_dbw_msgs/msg/fault_actions_report.hpp>
#include <raptor_dbw_msgs/msg/gear_report.hpp>
#include <raptor_dbw_msgs/msg/steering2_report.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/surround_report.hpp>
#include <raptor_dbw_msgs/msg/tire_pressure_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_position_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/diagnostic_report.hpp>

#include "npc_controller_msgs/msg/misc_report.hpp"
#include "npc_controller_msgs/msg/rc_to_ct.hpp"
#include "npc_controller_msgs/msg/pt_report.hpp"
#include "npc_controller_msgs/msg/ct_report.hpp"
#include "npc_controller_msgs/msg/tire_report.hpp"


#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "raptor_dbw_can/dispatch.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace raptor_dbw_can
{
class DbwNode : public rclcpp::Node
{
public:
  explicit DbwNode(const rclcpp::NodeOptions & options);
  ~DbwNode();

private:
  void timerTireCallback();
  void timerPtCallback();
  void recvEnable(const std_msgs::msg::Empty::SharedPtr msg);
  void recvDisable(const std_msgs::msg::Empty::SharedPtr msg);
  void recvCAN(const can_msgs::msg::Frame::SharedPtr msg);
  void recvCanImu(const std::vector<can_msgs::msg::Frame> msgs);
  void recvCanGps(const std::vector<can_msgs::msg::Frame> msgs);
  void recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg);
  void recvAcceleratorPedalCmd(const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg);
  void recvSteeringCmd(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg);
  void recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg);
  void recvCtReport(const npc_controller_msgs::msg::CtReport::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_tire_report_;
  rclcpp::TimerBase::SharedPtr timer_pt_report_;

  // kinematic state timed publisher for HIL test
  rclcpp::TimerBase::SharedPtr timer_kinematic_state_;

  bool prev_enable_;
  bool enable_;
  bool override_brake_;
  bool override_accelerator_pedal_;
  bool override_steering_;
  bool override_gear_;
  bool fault_brakes_;
  bool fault_accelerator_pedal_;
  bool fault_steering_;
  bool fault_steering_cal_;
  bool fault_watchdog_;
  bool fault_watchdog_using_brakes_;
  bool fault_watchdog_warned_;
  bool timeout_brakes_;
  bool timeout_accelerator_pedal_;
  bool timeout_steering_;
  bool enabled_brakes_;
  bool enabled_accelerator_pedal_;
  bool enabled_steering_;
  bool gear_warned_;

  npc_controller_msgs::msg::PtReport pt_report_msg;
  npc_controller_msgs::msg::TireReport tire_report_msg;

  // Licensing
  std::string vin_;

  // Frame ID
  std::string frame_id_;

  // Buttons (enable/disable)
  bool buttons_;

  // Ackermann steering
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;

  // Subscribed topics
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr sub_accelerator_pedal_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr sub_steering_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_shift_cmd_;
  rclcpp::Subscription<npc_controller_msgs::msg::CtReport>::SharedPtr sub_ct_report_;

  // Published topics
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>::SharedPtr pub_accel_pedal_; // acc pedal report do
  rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr pub_steering_; //steering report do
  rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr pub_steering_ext_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr pub_wheel_speeds_; // wheelspeedreport do

  rclcpp::Publisher<raptor_dbw_msgs::msg::Brake2Report>::SharedPtr pub_brake_2_report_; // brake report do
  rclcpp::Publisher<raptor_dbw_msgs::msg::DiagnosticReport>::SharedPtr pub_diagnostic_report_;

  // Custom Messages specific to NPC Controller
  rclcpp::Publisher<npc_controller_msgs::msg::MiscReport>::SharedPtr pub_misc_do_;
  rclcpp::Publisher<npc_controller_msgs::msg::RcToCt>::SharedPtr pub_rc_to_ct_;
  rclcpp::Publisher<npc_controller_msgs::msg::TireReport>::SharedPtr pub_tire_report_;
  rclcpp::Publisher<npc_controller_msgs::msg::PtReport>::SharedPtr pub_pt_report_;

  NewEagle::Dbc dbwDbc_;
  std::string dbcFile_;
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DBWNODE_HPP_
