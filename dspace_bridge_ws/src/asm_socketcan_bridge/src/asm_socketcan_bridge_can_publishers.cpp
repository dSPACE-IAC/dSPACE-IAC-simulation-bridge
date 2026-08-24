#include "asm_socketcan_bridge.h"

namespace asm_socketcan_bridge {
  void AsmSocketCanBridgeNode::publish_novatel_report()
  {
    publishCanMessage("novatel_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("novatel_lat", bus.nova_tel_pwr_pak1_var.best_pos_var.lat);
      assign("novatel_long", bus.nova_tel_pwr_pak1_var.best_pos_var.lon);
    });
  }

  void AsmSocketCanBridgeNode::publish_base_to_car_summary()
  {
    publishCanMessage("base_to_car_summary", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &race_control = bus.race_control_var;
      assign("base_to_car_heartbeat", race_control.base_to_car_heartbeat);
      assign("track_flag", race_control.track_flag);
      assign("veh_flag", race_control.veh_flag);
      assign("veh_rank", race_control.veh_rank);
      assign("lap_count", race_control.lap_count);
      assign("lap_distance", race_control.lap_distance);
      assign("round_target_speed", race_control.round_target_speed);
    });
  }

  void AsmSocketCanBridgeNode::publish_marelli_report_1()
  {
    publishCanMessage("marelli_report_1", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("marelli_track_flag", bus.race_control_var.track_flag);
      assign("marelli_vehicle_flag", bus.race_control_var.veh_flag);
      assign("marelli_sector_flag", bus.race_control_var.marelli_sector_flag);
      assign("marelli_rc_base_sync_check", bus.race_control_var.marelli_rc_base_sync_check);
      // Not important -> no need to fill other values than 0
      assign("marelli_rc_lte_rssi", 0);
    });
  }

  void AsmSocketCanBridgeNode::publish_marelli_report_2()
  {
    publishCanMessage("marelli_report_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      // Todo low priority: Change to dedicated signal later
      assign("marelli_gps_lat", bus.nova_tel_pwr_pak1_var.best_pos_var.lat);
      assign("marelli_gps_long", bus.nova_tel_pwr_pak1_var.best_pos_var.lon);
    });
  }

  void AsmSocketCanBridgeNode::publish_base_to_car_timing()
  {
    publishCanMessage("base_to_car_timing", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &race_control = bus.race_control_var;
      assign("laps", race_control.laps);
      assign("lap_time", race_control.lap_time);
      assign("time_stamp", race_control.time_stamp);
    });
  }

  void AsmSocketCanBridgeNode::publish_rest_of_field()
  {
    publishCanMessage("rest_of_field", [&](PreparedCanMessage &message, const ASMBus &) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      // Obsolete -> no need to fill other values than 0
      assign("comp_veh_num", 0);
      assign("comp_rank", 0);
      assign("comp_veh_flag", 0);
      assign("comp_laps_count", 0);
      assign("comp_lap_distance", 0);
      assign("comp_speed", 0);
    });
  }

  void AsmSocketCanBridgeNode::publish_pt_report_1()
  {
    publishCanMessage("pt_report_1", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &powertrain = bus.vehicle_sensors_var.power_train_data_var;
      assign("throttle_position", powertrain.throttle_position);
      assign("current_gear", powertrain.current_gear);
      assign("engine_speed_rpm", powertrain.engine_rpm);
      assign("vehicle_speed_kmph", powertrain.vehicle_speed_kmph);
      assign("engine_run_switch", powertrain.engine_run_switch_status);
      assign("engine_state", powertrain.engine_on_status);
      assign("gear_shift_status", powertrain.gear_shift_status);
    });
  }

  void AsmSocketCanBridgeNode::publish_pt_report_2()
  {
    publishCanMessage("pt_report_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &powertrain = bus.vehicle_sensors_var.power_train_data_var;
      assign("fuel_pressure_kPa", powertrain.fuel_pressure);
      assign("engine_oil_pressure_kPa", powertrain.engine_oil_pressure);
      assign("coolant_temperature", powertrain.engine_coolant_temperature);
      assign("transmission_temperature", powertrain.transmission_oil_temperature);
      assign("transmission_pressure_kPa", powertrain.transmission_oil_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_pt_report_3()
  {
    publishCanMessage("pt_report_3", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &powertrain = bus.vehicle_sensors_var.power_train_data_var;
      assign("engine_oil_temperature", powertrain.engine_oil_temperature);
      assign("torque_wheels", powertrain.torque_wheels_nm);
      assign("driver_traction_aim_swicth_fbk", powertrain.driver_traction_aim_switch_fbk);
      assign("driver_traction_range_switch_fbk", powertrain.driver_traction_range_switch_fbk);
      const auto &race_control = bus.race_control_var;
      assign("push2pass_status", race_control.push2pass_status);
      assign("push2pass_budget_s", race_control.push2pass_budget_s);
      assign("push2pass_active_app_limit", race_control.push2pass_active_app_limit);
    });
  }

  void AsmSocketCanBridgeNode::publish_pt_report_4()
  {
    publishCanMessage("pt_report_4", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &powertrain = bus.vehicle_sensors_var.power_train_data_var;
      assign("boost_aim_psi", powertrain.boost_aim_psi);
      assign("boost_press_psi", powertrain.boost_press_psi);
      assign("intake_manifold_press_kPa", powertrain.intake_manifold_press_kPa);
      assign("intake_air_temp_degC", powertrain.intake_air_temp_degC);
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report()
  {
    publishCanMessage("steering_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("steering_motor_fdbk_counter", this->steering_motor_fdbk_counter++);
      assign("primary_steering_angular_rate", bus.vehicle_sensors_var.vehicle_data_var.steering_var.primary_steering_angular_rate);
      assign("commanded_steering_rate", bus.vehicle_sensors_var.vehicle_data_var.steering_var.commanded_steering_rate);
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report_extd()
  {
    publishCanMessage("steering_report_extd", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &steering_angle = bus.vehicle_sensors_var.vehicle_data_var.steering_var.steering_wheel_angle;
      assign("average_steering_ang_fdbk", steering_angle);
      assign("primary_steering_angle_fbk", steering_angle);
      assign("secondary_steering_ang_fdbk", steering_angle);
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report_extd_2()
  {
    publishCanMessage("steering_report_extd_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &steering = bus.vehicle_sensors_var.vehicle_data_var.steering_var;
      assign("motor_duty_cycle_cmd", steering.motor_duty_cycle_cmd);
      assign("motor_duty_cycle_fbk", steering.motor_duty_cycle_fbk);
      assign("motor_current_fbk", steering.motor_current_fbk);
      // Not important: Feedback of voltage measured at DBW(steer) ECU
      assign("sbw_ecu_voltage", 0);
      // Not important: Feedback of voltage measured at DBW(steer) ECU
      assign("sbw_ecu_temp", 0);
      // Not important: Diagnostics
      assign("sbw_error_code", 0);
      assign("sbw_motor_torque_estimate", steering.sbw_motor_torque_estimate);
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report_extd_3()
  {
    publishCanMessage("steering_report_extd_3", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &steering = bus.vehicle_sensors_var.vehicle_data_var.steering_var;
      assign("steering_p_contribution", steering.steering_p_contribution);
      assign("steering_i_contribution", steering.steering_i_contribution);
      assign("steering_d_contribution", steering.steering_d_contribution);
    });
  }

  void AsmSocketCanBridgeNode::publish_brake_pressure_report()
  {
    publishCanMessage("brake_pressure_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("brk_pressure_fdbk_counter", this->brk_pressure_fdbk_counter++);
      const auto &brake = bus.vehicle_sensors_var.vehicle_data_var.brake_var;
      assign("brake_pressure_fdbk_rear", brake.rear_brake_pressure);
      assign("brake_pressure_fdbk_front", brake.front_brake_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_brake_report_extd()
  {
    publishCanMessage("brake_report_extd", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &brake = bus.vehicle_sensors_var.vehicle_data_var.brake_var;
      assign("F_brk_pos_cmd", brake.f_brk_pos_cmd);
      assign("F_brk_pos_fbk", brake.f_brk_pos_fbk);
      assign("R_brk_pos_cmd", brake.r_brk_pos_cmd);
      assign("R_brk_pos_fbk", brake.r_brk_pos_fbk);
    });
  }

  void AsmSocketCanBridgeNode::publish_brake_report_extd_2()
  {
    publishCanMessage("brake_report_extd_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &brake = bus.vehicle_sensors_var.vehicle_data_var.brake_var;
      assign("f_brake_act_force", brake.f_brake_act_force);
      assign("r_brake_act_force", brake.r_brake_act_force);
    });
  }

  void AsmSocketCanBridgeNode::publish_accelerator_report()
  {
    publishCanMessage("accelerator_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("acc_pedal_fdbk_counter", this->acc_pedal_fdbk_counter++);
      assign("acc_pedal_fdbk", bus.vehicle_sensors_var.vehicle_data_var.accelerator_var.accel_pedal_output);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RR_1()
  {
    publishCanMessage("Tire_Temp_RR_1", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_right_var.rr_tire_temperature;
      assign("RR_Tire_Temp_04", temp);
      assign("RR_Tire_Temp_03", temp);
      assign("RR_Tire_Temp_02", temp);
      assign("RR_Tire_Temp_01", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RR_2()
  {
    publishCanMessage("Tire_Temp_RR_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_right_var.rr_tire_temperature;
      assign("RR_Tire_Temp_08", temp);
      assign("RR_Tire_Temp_07", temp);
      assign("RR_Tire_Temp_06", temp);
      assign("RR_Tire_Temp_05", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RR_3()
  {
    publishCanMessage("Tire_Temp_RR_3", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_right_var.rr_tire_temperature;
      assign("RR_Tire_Temp_12", temp);
      assign("RR_Tire_Temp_11", temp);
      assign("RR_Tire_Temp_10", temp);
      assign("RR_Tire_Temp_09", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RR_4()
  {
    publishCanMessage("Tire_Temp_RR_4", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_right_var.rr_tire_temperature;
      assign("RR_Tire_Temp_16", temp);
      assign("RR_Tire_Temp_15", temp);
      assign("RR_Tire_Temp_14", temp);
      assign("RR_Tire_Temp_13", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RL_1()
  {
    publishCanMessage("Tire_Temp_RL_1", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_left_tire_var.rl_tire_temperature;
      assign("RL_Tire_Temp_04", temp);
      assign("RL_Tire_Temp_03", temp);
      assign("RL_Tire_Temp_02", temp);
      assign("RL_Tire_Temp_01", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RL_2()
  {
    publishCanMessage("Tire_Temp_RL_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_left_tire_var.rl_tire_temperature;
      assign("RL_Tire_Temp_08", temp);
      assign("RL_Tire_Temp_07", temp);
      assign("RL_Tire_Temp_06", temp);
      assign("RL_Tire_Temp_05", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RL_3()
  {
    publishCanMessage("Tire_Temp_RL_3", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_left_tire_var.rl_tire_temperature;
      assign("RL_Tire_Temp_12", temp);
      assign("RL_Tire_Temp_11", temp);
      assign("RL_Tire_Temp_10", temp);
      assign("RL_Tire_Temp_09", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_RL_4()
  {
    publishCanMessage("Tire_Temp_RL_4", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_left_tire_var.rl_tire_temperature;
      assign("RL_Tire_Temp_16", temp);
      assign("RL_Tire_Temp_15", temp);
      assign("RL_Tire_Temp_14", temp);
      assign("RL_Tire_Temp_13", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FR_1()
  {
    publishCanMessage("Tire_Temp_FR_1", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_right_tire_var.fr_tire_temperature;
      assign("FR_Tire_Temp_04", temp);
      assign("FR_Tire_Temp_03", temp);
      assign("FR_Tire_Temp_02", temp);
      assign("FR_Tire_Temp_01", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FR_2()
  {
    publishCanMessage("Tire_Temp_FR_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_right_tire_var.fr_tire_temperature;
      assign("FR_Tire_Temp_08", temp);
      assign("FR_Tire_Temp_07", temp);
      assign("FR_Tire_Temp_06", temp);
      assign("FR_Tire_Temp_05", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FR_3()
  {
    publishCanMessage("Tire_Temp_FR_3", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_right_tire_var.fr_tire_temperature;
      assign("FR_Tire_Temp_12", temp);
      assign("FR_Tire_Temp_11", temp);
      assign("FR_Tire_Temp_10", temp);
      assign("FR_Tire_Temp_09", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FR_4()
  {
    publishCanMessage("Tire_Temp_FR_4", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_right_tire_var.fr_tire_temperature;
      assign("FR_Tire_Temp_16", temp);
      assign("FR_Tire_Temp_15", temp);
      assign("FR_Tire_Temp_14", temp);
      assign("FR_Tire_Temp_13", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FL_1()
  {
    publishCanMessage("Tire_Temp_FL_1", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_left_tire_var.fl_tire_temperature;
      assign("FL_Tire_Temp_04", temp);
      assign("FL_Tire_Temp_03", temp);
      assign("FL_Tire_Temp_02", temp);
      assign("FL_Tire_Temp_01", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FL_2()
  {
    publishCanMessage("Tire_Temp_FL_2", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_left_tire_var.fl_tire_temperature;
      assign("FL_Tire_Temp_08", temp);
      assign("FL_Tire_Temp_07", temp);
      assign("FL_Tire_Temp_06", temp);
      assign("FL_Tire_Temp_05", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FL_3()
  {
    publishCanMessage("Tire_Temp_FL_3", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_left_tire_var.fl_tire_temperature;
      assign("FL_Tire_Temp_12", temp);
      assign("FL_Tire_Temp_11", temp);
      assign("FL_Tire_Temp_10", temp);
      assign("FL_Tire_Temp_09", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Temp_FL_4()
  {
    publishCanMessage("Tire_Temp_FL_4", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto temp = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_left_tire_var.fl_tire_temperature;
      assign("FL_Tire_Temp_16", temp);
      assign("FL_Tire_Temp_15", temp);
      assign("FL_Tire_Temp_14", temp);
      assign("FL_Tire_Temp_13", temp);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Pressure_RR()
  {
    publishCanMessage("Tire_Pressure_RR", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &tire = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_right_var;
      assign("RR_Tire_Pressure_Gauge", tire.rr_tire_pressure_gauge);
      assign("RR_Tire_Pressure", tire.rr_tire_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Pressure_RL()
  {
    publishCanMessage("Tire_Pressure_RL", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &tire = bus.vehicle_sensors_var.vehicle_data_var.tires_var.rear_left_tire_var;
      assign("RL_Tire_Pressure_Gauge", tire.rl_tire_pressure_gauge);
      assign("RL_Tire_Pressure", tire.rl_tire_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Pressure_FR()
  {
    publishCanMessage("Tire_Pressure_FR", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &tire = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_right_tire_var;
      assign("FR_Tire_Pressure_Gauge", tire.fr_tire_pressure_gauge);
      assign("FR_Tire_Pressure", tire.fr_tire_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_Tire_Pressure_FL()
  {
    publishCanMessage("Tire_Pressure_FL", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &tire = bus.vehicle_sensors_var.vehicle_data_var.tires_var.front_left_tire_var;
      assign("FL_Tire_Pressure_Gauge", tire.fl_tire_pressure_gauge);
      assign("FL_Tire_Pressure", tire.fl_tire_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_wheel_strain_gauge()
  {
    publishCanMessage("wheel_strain_gauge", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &tires = bus.vehicle_sensors_var.vehicle_data_var.tires_var;
      assign("wheel_strain_gauge_RR", tires.rear_right_var.rr_wheel_load);
      assign("wheel_strain_gauge_RL", tires.rear_left_tire_var.rl_wheel_load);
      assign("wheel_strain_gauge_FR", tires.front_right_tire_var.fr_wheel_load);
      assign("wheel_strain_gauge_FL", tires.front_left_tire_var.fl_wheel_load);
    });
  }

  void AsmSocketCanBridgeNode::publish_wheel_potentiometer_data()
  {
    publishCanMessage("wheel_potentiometer_data", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &suspension = bus.vehicle_sensors_var.vehicle_data_var.suspension_var;
      assign("wheel_potentiometer_RR", suspension.rr_damper_linear_potentiometer);
      assign("wheel_potentiometer_RL", suspension.rl_damper_linear_potentiometer);
      assign("wheel_potentiometer_FR", suspension.fr_damper_linear_potentiometer);
      assign("wheel_potentiometer_FL", suspension.fl_damper_linear_potentiometer);
    });
  }

  void AsmSocketCanBridgeNode::publish_wheel_speed_report()
  {
    publishCanMessage("wheel_speed_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &wheel_speed = bus.vehicle_sensors_var.vehicle_data_var.wheel_speed_var;
      assign("wheel_speed_RR", wheel_speed.ws_rear_right);
      assign("wheel_speed_RL", wheel_speed.ws_rear_left);
      assign("wheel_speed_FR", wheel_speed.ws_front_right);
      assign("wheel_speed_FL", wheel_speed.ws_front_left);
    });
  }

  void AsmSocketCanBridgeNode::publish_misc_report()
  {
    publishCanMessage("misc_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("battery_voltage", bus.vehicle_sensors_var.vehicle_data_var.misc_report_var.battery_voltage);
      assign("safety_switch_state", bus.vehicle_sensors_var.vehicle_data_var.misc_report_var.safety_switch_state);
      assign("mode_switch_state", bus.vehicle_sensors_var.vehicle_data_var.misc_report_var.mode_switch_state);
      assign("sys_state", bus.race_control_var.sys_state);
      assign("target_speed_multi_car_race", bus.race_control_var.target_speed_multi_car_race);
      assign("raptor_rolling_counter", this->raptor_rolling_counter);
      this->raptor_rolling_counter = (this->raptor_rolling_counter + 1u) % 16u;
    });
  }

  void AsmSocketCanBridgeNode::publish_diagnostic_report()
  {
    publishCanMessage("diagnostic_report", [&](PreparedCanMessage &message, const ASMBus &) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_system_warning", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_system_failure", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_brake_warning1", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_brake_warning2", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_brake_warning3", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_steer_warning1", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_steer_warning2", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("sd_steer_warning3", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("motec_warning", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("est1_oos_front_brk", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("est2_oos_rear_brk", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("est3_low_eng_speed", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("est4_sd_comms_loss", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("est5_motec_comms_loss", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("est6_sd_ebrake", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("adlink_hb_lost", 0);
      // Todo priority medium: Works for now but could be filled with dedicated signal to enable failure injection from ControlDesk
      assign("rc_lost", 0);
    });
  }


} // namespace asm_socketcan_bridge
