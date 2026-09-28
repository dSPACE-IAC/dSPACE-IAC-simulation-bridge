#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "raptor_dbw_can/DbwNode.hpp"
#include "raptor_dbw_can/dispatch.hpp"

#include <can_dbc_parser/DbcBuilder.hpp>

namespace
{
using namespace std::chrono_literals;

void require(bool condition, const std::string & description)
{
  if (!condition) {
    throw std::runtime_error(description);
  }
}

void requireNear(double actual, double expected, const std::string & description)
{
  if (std::fabs(actual - expected) > 1e-3) {
    throw std::runtime_error(
            description + ": expected " + std::to_string(expected) +
            ", got " + std::to_string(actual));
  }
}

bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & ready)
{
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (ready()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  executor.spin_some();
  return ready();
}

void runReportForwardingTest()
{
  rclcpp::NodeOptions dbw_options;
  dbw_options.parameter_overrides({
    rclcpp::Parameter("dbw_dbc_file", RAPTOR_DBC_FILE_PATH)});
  auto dbw_node = std::make_shared<raptor_dbw_can::DbwNode>(dbw_options);
  auto test_node = std::make_shared<rclcpp::Node>("dbw_report_forwarding_test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dbw_node);
  executor.add_node(test_node);

  auto can_publisher = test_node->create_publisher<can_msgs::msg::Frame>("/can_rx", 500);
  std::shared_ptr<const raptor_dbw_msgs::msg::WheelSpeedReport> wheel_speed;
  std::shared_ptr<const raptor_dbw_msgs::msg::AcceleratorPedalReport> accelerator;
  std::shared_ptr<const raptor_dbw_msgs::msg::Brake2Report> brake;
  std::shared_ptr<const raptor_dbw_msgs::msg::SteeringReport> steering;
  std::shared_ptr<const raptor_dbw_msgs::msg::SteeringExtendedReport> steering_extended;
  std::shared_ptr<const raptor_dbw_msgs::msg::DiagnosticReport> diagnostic;
  std::shared_ptr<const raptor_dbw_msgs::msg::GearReport> gear_report;
  std::shared_ptr<const raptor_dbw_msgs::msg::LowVoltageSystemReport> low_voltage;
  std::shared_ptr<const raptor_dbw_msgs::msg::TirePressureReport> tire_pressure;
  std::shared_ptr<const raptor_dbw_msgs::msg::WheelPositionReport> wheel_position;
  std::shared_ptr<const npc_controller_msgs::msg::MiscReport> misc;
  std::shared_ptr<const npc_controller_msgs::msg::RcToCt> rc_to_ct;
  std::shared_ptr<const npc_controller_msgs::msg::PtReport> powertrain;
  std::shared_ptr<const npc_controller_msgs::msg::TireReport> tire;
  std::shared_ptr<const raptor_dbw_msgs::msg::MotecReport> motec;
  std::size_t tire_pressure_messages = 0;

  auto wheel_speed_sub = test_node->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
    "/wheel_speed_report", 10,
    [&wheel_speed](raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr message) {
      wheel_speed = message;
    });
  auto accelerator_sub = test_node->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalReport>(
    "/accelerator_pedal_report", 10,
    [&accelerator](raptor_dbw_msgs::msg::AcceleratorPedalReport::ConstSharedPtr message) {
      accelerator = message;
    });
  auto brake_sub = test_node->create_subscription<raptor_dbw_msgs::msg::Brake2Report>(
    "/brake_2_report", 10,
    [&brake](raptor_dbw_msgs::msg::Brake2Report::ConstSharedPtr message) {
      brake = message;
    });
  auto steering_sub = test_node->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
    "/steering_report", 10,
    [&steering](raptor_dbw_msgs::msg::SteeringReport::ConstSharedPtr message) {
      steering = message;
    });
  auto steering_extended_sub = test_node->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
    "/steering_extended_report", 10,
    [&steering_extended](raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr message) {
      steering_extended = message;
    });
  auto diagnostic_sub = test_node->create_subscription<raptor_dbw_msgs::msg::DiagnosticReport>(
    "/diagnostic_report", 10,
    [&diagnostic](raptor_dbw_msgs::msg::DiagnosticReport::ConstSharedPtr message) {
      diagnostic = message;
    });
  auto gear_report_sub = test_node->create_subscription<raptor_dbw_msgs::msg::GearReport>(
    "/gear_report", 10,
    [&gear_report](raptor_dbw_msgs::msg::GearReport::ConstSharedPtr message) {
      gear_report = message;
    });
  auto low_voltage_sub = test_node->create_subscription<raptor_dbw_msgs::msg::LowVoltageSystemReport>(
    "/low_voltage_system_report", 10,
    [&low_voltage](raptor_dbw_msgs::msg::LowVoltageSystemReport::ConstSharedPtr message) {
      low_voltage = message;
    });
  auto tire_pressure_sub = test_node->create_subscription<raptor_dbw_msgs::msg::TirePressureReport>(
    "/tire_pressure_report", 10,
    [&tire_pressure, &tire_pressure_messages](
      raptor_dbw_msgs::msg::TirePressureReport::ConstSharedPtr message)
    {
      tire_pressure = message;
      ++tire_pressure_messages;
    });
  auto wheel_position_sub = test_node->create_subscription<raptor_dbw_msgs::msg::WheelPositionReport>(
    "/wheel_position_report", 10,
    [&wheel_position](raptor_dbw_msgs::msg::WheelPositionReport::ConstSharedPtr message) {
      wheel_position = message;
    });
  auto misc_sub = test_node->create_subscription<npc_controller_msgs::msg::MiscReport>(
    "/misc_report_do", 10,
    [&misc](npc_controller_msgs::msg::MiscReport::ConstSharedPtr message) {
      misc = message;
    });
  auto rc_to_ct_sub = test_node->create_subscription<npc_controller_msgs::msg::RcToCt>(
    "/rc_to_ct", 10,
    [&rc_to_ct](npc_controller_msgs::msg::RcToCt::ConstSharedPtr message) {
      rc_to_ct = message;
    });
  auto powertrain_sub = test_node->create_subscription<npc_controller_msgs::msg::PtReport>(
    "/pt_report", 10,
    [&powertrain](npc_controller_msgs::msg::PtReport::ConstSharedPtr message) {
      powertrain = message;
    });
  auto tire_sub = test_node->create_subscription<npc_controller_msgs::msg::TireReport>(
    "/tire_report", 10,
    [&tire](npc_controller_msgs::msg::TireReport::ConstSharedPtr message) {
      tire = message;
    });
  auto motec_sub = test_node->create_subscription<raptor_dbw_msgs::msg::MotecReport>(
    "/motec_report", 10,
    [&motec](raptor_dbw_msgs::msg::MotecReport::ConstSharedPtr message) {
      motec = message;
    });

  NewEagle::Dbc dbc = NewEagle::DbcBuilder().NewDbc(RAPTOR_DBC_FILE_PATH);
  uint32_t stamp_nanoseconds = 1;
  auto send_frame = [&](uint32_t id, std::initializer_list<std::pair<std::string, double>> values) {
      auto * message = dbc.GetMessageById(id);
      require(message != nullptr, "DBC message not found for test frame");
      for (auto & entry : *message->GetSignals()) {
        entry.second.SetResult(0.0);
      }
      for (const auto & value : values) {
        auto * signal = message->GetSignal(value.first);
        require(signal != nullptr, "DBC signal not found: " + value.first);
        signal->SetResult(value.second);
      }
      auto frame = message->GetFrame();
      frame.header.stamp.sec = 1;
      frame.header.stamp.nanosec = stamp_nanoseconds++;
      can_publisher->publish(frame);
    };

  require(spinUntil(executor, [&]() {return can_publisher->get_subscription_count() > 0;}),
    "CAN input subscription did not match");

  send_frame(raptor_dbw_can::ID_WHEEL_SPEED_REPORT_DO, {
      {"wheel_speed_FL", 11.0}, {"wheel_speed_FR", 22.0},
      {"wheel_speed_RL", 33.0}, {"wheel_speed_RR", 44.0}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(wheel_speed);}),
    "wheel speed report was not published");
  requireNear(wheel_speed->front_left, 11.0, "wheel speed front left");
  requireNear(wheel_speed->front_right, 22.0, "wheel speed front right");
  requireNear(wheel_speed->rear_left, 33.0, "wheel speed rear left");
  requireNear(wheel_speed->rear_right, 44.0, "wheel speed rear right");

  send_frame(raptor_dbw_can::ID_BRAKE_PRESSURE_REPORT_DO, {
      {"brake_pressure_fdbk_front", 151.0}, {"brake_pressure_fdbk_rear", 252.0},
      {"brk_pressure_fdbk_counter", 6.0}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(brake);}),
    "brake report was not published");
  requireNear(brake->front_brake_pressure, 1.51, "front brake pressure");
  requireNear(brake->rear_brake_pressure, 2.52, "rear brake pressure");
  require(brake->rolling_counter == 6, "brake report rolling counter");

  send_frame(raptor_dbw_can::ID_ACCELERATOR_REPORT_DO, {
      {"acc_pedal_fdbk", 63.2}, {"acc_pedal_fdbk_counter", 5.0}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(accelerator);}),
    "accelerator report was not published");
  requireNear(accelerator->pedal_input, 63.2, "accelerator pedal input");
  require(accelerator->rolling_counter == 5, "accelerator rolling counter");

  send_frame(raptor_dbw_can::ID_STEERING_REPORT_DO, {
      {"steering_motor_fdbk_counter", 7.0}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(steering);}),
    "steering report was not published");
  require(steering->rolling_counter == 7, "steering rolling counter");

  send_frame(raptor_dbw_can::ID_STEERING_REPORT_EXTD, {
      {"primary_steering_angle_fbk", 10.5},
      {"secondary_steering_ang_fdbk", -20.0},
      {"average_steering_ang_fdbk", 30.5}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(steering_extended);}),
    "extended steering report was not published");
  requireNear(steering_extended->steering_motor_ang_1, 10.5, "primary steering angle");
  requireNear(steering_extended->steering_motor_ang_2, -20.0, "secondary steering angle");
  requireNear(steering_extended->steering_motor_ang_3, 30.5, "average steering angle");

  send_frame(raptor_dbw_can::ID_DIAGNOSTIC_REPORT, {
      {"sd_system_warning", 1}, {"sd_system_failure", 0},
      {"sd_brake_warning1", 1}, {"sd_brake_warning2", 0}, {"sd_brake_warning3", 1},
      {"sd_steer_warning1", 0}, {"sd_steer_warning2", 1}, {"sd_steer_warning3", 0},
      {"motec_warning", 13}, {"est1_oos_front_brk", 1}, {"est2_oos_rear_brk", 0},
      {"est3_low_eng_speed", 1}, {"est4_sd_comms_loss", 0},
      {"est5_motec_comms_loss", 1}, {"est6_sd_ebrake", 0},
      {"adlink_hb_lost", 1}, {"rc_lost", 0}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(diagnostic);}),
    "diagnostic report was not published");
  require(diagnostic->sd_system_warning && !diagnostic->sd_system_failure,
    "system diagnostic flags");
  require(diagnostic->sd_brake_warning1 && !diagnostic->sd_brake_warning2 &&
    diagnostic->sd_brake_warning3, "brake diagnostic flags");
  require(!diagnostic->sd_steer_warning1 && diagnostic->sd_steer_warning2 &&
    !diagnostic->sd_steer_warning3, "steering diagnostic flags");
  require(diagnostic->motec_warning == 13, "Motec diagnostic value");
  require(diagnostic->front_brk_failure && !diagnostic->rear_brk_failure &&
    diagnostic->low_eng_speed && !diagnostic->sd_comms_loss &&
    diagnostic->motec_comms_loss && !diagnostic->sd_ebrake &&
    diagnostic->adlink_hb_lost && !diagnostic->rc_lost,
    "diagnostic action flags");

  send_frame(raptor_dbw_can::ID_WHEEL_POTENTIOMETER, {
      {"wheel_potentiometer_FL", 10.01}, {"wheel_potentiometer_FR", 20.02},
      {"wheel_potentiometer_RL", 30.03}, {"wheel_potentiometer_RR", 40.04}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(wheel_position);}),
    "wheel position report was not published");
  require(wheel_position->front_left == 1001 && wheel_position->front_right == 2002 &&
    wheel_position->rear_left == 3003 && wheel_position->rear_right == 4004,
    "wheel position raw counts");

  const char * corners[] = {"FL", "FR", "RL", "RR"};
  for (std::size_t wheel = 0; wheel < 4; ++wheel) {
    const auto id = static_cast<uint32_t>(raptor_dbw_can::ID_TIRE_PRESSURE_FL + wheel);
    const auto pressure = 1000.0 + 100.0 * wheel;
    const auto gauge = pressure + 50.0;
    send_frame(id, {
        {std::string(corners[wheel]) + "_Tire_Pressure", pressure},
        {std::string(corners[wheel]) + "_Tire_Pressure_Gauge", gauge}});
  }
  require(spinUntil(executor, [&]() {return tire_pressure_messages >= 4 && tire_pressure;}),
    "tire pressure reports were not published for all corners");
  requireNear(tire_pressure->front_left, 105.0, "front-left tire pressure kPa");
  requireNear(tire_pressure->front_right, 115.0, "front-right tire pressure kPa");
  requireNear(tire_pressure->rear_left, 125.0, "rear-left tire pressure kPa");
  requireNear(tire_pressure->rear_right, 135.0, "rear-right tire pressure kPa");

  for (std::size_t wheel = 0; wheel < 4; ++wheel) {
    for (std::size_t block = 0; block < 4; ++block) {
      const auto id = static_cast<uint32_t>(raptor_dbw_can::ID_TIRE_TEMP_FL_1 + wheel * 4 + block);
      std::initializer_list<std::pair<std::string, double>> values = {};
      std::vector<std::pair<std::string, double>> block_values;
      for (std::size_t index = 0; index < 4; ++index) {
        const auto temperature_number = block * 4 + index + 1;
        const auto number = temperature_number < 10 ? "0" : "";
        block_values.emplace_back(
          std::string(corners[wheel]) + "_Tire_Temp_" + number +
          std::to_string(temperature_number),
          20.0 + 16.0 * wheel + 4.0 * block + index);
      }
      auto * message = dbc.GetMessageById(id);
      require(message != nullptr, "tire temperature DBC block missing");
      for (auto & entry : *message->GetSignals()) {
        entry.second.SetResult(0.0);
      }
      for (const auto & value : block_values) {
        auto * signal = message->GetSignal(value.first);
        require(signal != nullptr, "tire temperature DBC signal missing: " + value.first);
        signal->SetResult(value.second);
      }
      auto frame = message->GetFrame();
      frame.header.stamp.sec = 1;
      frame.header.stamp.nanosec = stamp_nanoseconds++;
      can_publisher->publish(frame);
    }
  }

  send_frame(raptor_dbw_can::ID_WHEEL_STRAIN_GAUGE, {
      {"wheel_strain_gauge_FL", 120.0}, {"wheel_strain_gauge_FR", 130.0},
      {"wheel_strain_gauge_RL", 140.0}, {"wheel_strain_gauge_RR", 150.0}});

  send_frame(raptor_dbw_can::ID_MISC_REPORT_DO, {
      {"battery_voltage", 12.5}, {"safety_switch_state", 5},
      {"mode_switch_state", 1}, {"sys_state", 3}});
  require(spinUntil(executor, [&]() {return static_cast<bool>(misc);}),
    "misc report was not published");
  requireNear(misc->battery_voltage, 12.5, "misc battery voltage");
  require(misc->safety_switch_state == 5 && misc->mode_switch_state && misc->sys_state == 3,
    "misc state signals");
  require(spinUntil(executor, [&]() {
    return low_voltage && low_voltage->header.stamp.nanosec == stamp_nanoseconds - 1;
  }), "low-voltage report was not published");
  requireNear(low_voltage->dbw_battery_volts, 12.5, "DBW battery voltage");

  send_frame(raptor_dbw_can::ID_MARELLI_REPORT_1, {
      {"marelli_track_flag", 2}, {"marelli_vehicle_flag", 4}});
  send_frame(raptor_dbw_can::ID_RC_TO_CT, {
      {"track_flag", 2}, {"veh_flag", 4}, {"veh_rank", 3}, {"lap_count", 8}, {"lap_distance", 42},
      {"round_target_speed", 35}, {"base_to_car_heartbeat", 7}});
  require(spinUntil(executor, [&]() {
    return rc_to_ct && rc_to_ct->stamp.nanosec == stamp_nanoseconds - 1;
  }),
    "race control report was not published");
  require(rc_to_ct->track_flag == 2 && rc_to_ct->veh_flag == 4 && rc_to_ct->veh_rank == 3 &&
    rc_to_ct->lap_count == 8 && rc_to_ct->lap_distance == 42 &&
    rc_to_ct->target_speed == 35 && rc_to_ct->rolling_counter == 7,
    "race control signals");

  send_frame(raptor_dbw_can::ID_PT_REPORT_1, {
      {"throttle_position", 65.5}, {"engine_run_switch", 1}, {"current_gear", 4},
      {"engine_speed_rpm", 4200}, {"vehicle_speed_kmph", 100.5},
      {"engine_state", 1}, {"gear_shift_status", 2}});
  send_frame(raptor_dbw_can::ID_PT_REPORT_2, {
      {"fuel_pressure_kPa", 543.2}, {"engine_oil_pressure_kPa", 321.0},
      {"coolant_temperature", 92}, {"transmission_temperature", 83},
      {"transmission_pressure_kPa", 876}});
  send_frame(raptor_dbw_can::ID_PT_REPORT_3, {
      {"engine_oil_temperature", 110}, {"torque_wheels", 230.5},
      {"driver_traction_aim_swicth_fbk", 3}, {"driver_traction_range_switch_fbk", 2}});
  send_frame(raptor_dbw_can::ID_PT_REPORT_4, {
      {"boost_aim_psi", 24.5}, {"boost_press_psi", 22.5},
      {"intake_manifold_press_kPa", 180.5}, {"intake_air_temp_degC", 35}});
  require(spinUntil(executor, [&]() {return powertrain && powertrain->stamp.nanosec == stamp_nanoseconds - 1;}),
    "powertrain report did not publish the final PT frame");
  requireNear(powertrain->throttle_position, 65.5, "PT throttle position");
  require(powertrain->engine_run_switch_status && powertrain->engine_on_status,
    "PT engine switches");
  require(powertrain->current_gear == 4 && powertrain->gear_shift_status == 2,
    "PT gear signals");
  require(gear_report && gear_report->header.stamp.nanosec == stamp_nanoseconds - 4 &&
    gear_report->state.gear == raptor_dbw_msgs::msg::Gear::DRIVE && gear_report->reject,
    "gear state and lockout report");
  requireNear(powertrain->engine_rpm, 4200, "PT engine speed");
  requireNear(powertrain->vehicle_speed_kmph, 100.5, "PT vehicle speed");
  requireNear(powertrain->fuel_pressure, 543.2, "PT fuel pressure");
  requireNear(powertrain->engine_oil_pressure, 321.0, "PT oil pressure");
  requireNear(powertrain->engine_coolant_temperature, 92, "PT coolant temperature");
  requireNear(powertrain->transmission_oil_temperature, 83, "PT transmission temperature");
  requireNear(powertrain->transmission_oil_pressure, 876, "PT transmission pressure");
  requireNear(powertrain->engine_oil_temperature, 110, "PT oil temperature");
  requireNear(powertrain->torque_wheels, 230.5, "PT wheel torque");
  requireNear(powertrain->map_sensor, 180.5, "PT intake manifold pressure");

  require(spinUntil(executor, [&]() {
    return motec && motec->header.stamp.nanosec == stamp_nanoseconds - 1;
  }), "Motec report did not publish PT data");
  require(motec->engine_speed == 4200 && motec->throttle_pedal == 65 &&
    motec->fuel_pump_state == 1 && motec->engine_state == 1 && motec->gear == 4 &&
    motec->gear_shift == 2 && motec->warning_source == 13,
    "Motec PT1/diagnostic signals");
  requireNear(motec->throttle_position, 65.5, "Motec throttle position");
  requireNear(motec->fuel_pressure_sensor, 543.2, "Motec fuel pressure");
  requireNear(motec->engine_oil_pressure, 321.0, "Motec oil pressure");
  requireNear(motec->coolant_temperature, 92, "Motec coolant temperature");
  requireNear(motec->engine_oil_temperature, 110, "Motec oil temperature");
  requireNear(motec->torque_wheels, 230.5, "Motec wheel torque");
  require(motec->driver_switch_1 == 3 && motec->driver_switch_2 == 2,
    "Motec driver switch feedback");
  requireNear(motec->boost_aim, 24.5, "Motec boost aim");
  requireNear(motec->boost_pressure, 22.5, "Motec boost pressure");
  requireNear(motec->inlet_manifold_pressure, 180.5, "Motec intake pressure");
  requireNear(motec->inlet_air_temperature, 35, "Motec intake temperature");
  requireNear(motec->vehicle_speed, 100.5, "Motec vehicle speed");
  requireNear(motec->wheel_speed_front_left, 11.0, "Motec front-left wheel speed");
  requireNear(motec->ecu_battery_voltage, 12.5, "Motec ECU battery voltage");

  require(spinUntil(executor, [&]() {return tire && tire->fl_wheel_load == 120.0;}),
    "tire report did not publish the final sensor frame");
  for (std::size_t wheel = 0; wheel < 4; ++wheel) {
    const std::vector<float> * temperatures[] = {
      &tire->fl_tire_temperature, &tire->fr_tire_temperature,
      &tire->rl_tire_temperature, &tire->rr_tire_temperature};
    const float tire_pressures[] = {
      tire->fl_tire_pressure, tire->fr_tire_pressure,
      tire->rl_tire_pressure, tire->rr_tire_pressure};
    const float gauge_pressures[] = {
      tire->fl_tire_pressure_gauge, tire->fr_tire_pressure_gauge,
      tire->rl_tire_pressure_gauge, tire->rr_tire_pressure_gauge};
    const float damper_positions[] = {
      tire->fl_damper_linear_potentiometer, tire->fr_damper_linear_potentiometer,
      tire->rl_damper_linear_potentiometer, tire->rr_damper_linear_potentiometer};
    const float wheel_loads[] = {
      tire->fl_wheel_load, tire->fr_wheel_load, tire->rl_wheel_load, tire->rr_wheel_load};
    require(temperatures[wheel]->size() == 16, "tire temperature array size");
    requireNear(tire_pressures[wheel], 100.0 + 10.0 * wheel, "tire pressure kPa");
    requireNear(gauge_pressures[wheel], 105.0 + 10.0 * wheel, "tire gauge pressure kPa");
    requireNear(damper_positions[wheel], 10.01 + 10.01 * wheel, "damper position");
    requireNear(wheel_loads[wheel], 120.0 + 10.0 * wheel, "wheel strain gauge value");
    for (std::size_t index = 0; index < 16; ++index) {
      requireNear((*temperatures[wheel])[index], 20.0 + 16.0 * wheel + index,
        "tire temperature slot");
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    runReportForwardingTest();
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}