#include "asm_socketcan_bridge.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {

  int sanitize_port_value(int64_t value,
                          int default_value,
                          const rclcpp::Logger &logger,
                          const std::string &description)
  {
    if (value < 0 || value > std::numeric_limits<int>::max()) {
      RCLCPP_WARN(logger,
                  "%s out of range (%lld); using default %d",
                  description.c_str(),
                  static_cast<long long>(value),
                  default_value);
      return default_value;
    }
    return static_cast<int>(value);
  }

  uint32_t sanitize_interval_value(int64_t value,
                                  uint32_t default_value,
                                  const rclcpp::Logger &logger,
                                  const std::string &description)
  {
    if (value < 0 || value > std::numeric_limits<uint32_t>::max()) {
      RCLCPP_WARN(logger,
                  "%s out of range (%lld); using default %u",
                  description.c_str(),
                  static_cast<long long>(value),
                  default_value);
      return default_value;
    }
    return static_cast<uint32_t>(value);
  }

  int16_t sanitize_retry_value(int64_t value,
                              int16_t default_value,
                              const rclcpp::Logger &logger,
                              const std::string &description)
  {
    if (value < 0) {
      RCLCPP_WARN(logger,
                  "%s below zero (%lld); using default %d",
                  description.c_str(),
                  static_cast<long long>(value),
                  default_value);
      return default_value;
    }

    if (value > std::numeric_limits<int16_t>::max()) {
      const auto clamped = std::numeric_limits<int16_t>::max();
      RCLCPP_WARN(logger,
                  "%s above int16_t max (%lld); clamping to %d",
                  description.c_str(),
                  static_cast<long long>(value),
                  static_cast<int>(clamped));
      return clamped;
    }

    return static_cast<int16_t>(value);
  }

  constexpr std::size_t kNovatelTopIndex = 0;
  constexpr std::size_t kNovatelBottomIndex = 1;
  constexpr std::size_t kVectorNavGpsLeftIndex = 0;
  constexpr std::size_t kVectorNavGpsRightIndex = 1;

}  // namespace

namespace asm_socketcan_bridge {

  AsmSocketCanBridgeNode::AsmSocketCanBridgeNode() : Node("asm_socketcan_bridge_node")
  {
    this->canBus = nullptr;

    const auto sim_manager_host =
      this->declare_parameter<std::string>("sim_manager.host", "127.0.0.1");
    RCLCPP_INFO(this->get_logger(), "SimManager Host IP: %s", sim_manager_host.c_str());
    this->api.setSimManagerHost(sim_manager_host);

    const auto asm_host =
      this->declare_parameter<std::string>("asm.host", "127.0.0.1");
    RCLCPP_INFO(this->get_logger(), "ASM Host IP: %s", asm_host.c_str());
    this->api.setASMHost(asm_host);

    const int64_t sim_manager_port_param =
      this->declare_parameter<int64_t>("sim_manager.port", 12345);
    const int sim_manager_port =
      sanitize_port_value(sim_manager_port_param, 12345, this->get_logger(), "SimManager port");
    RCLCPP_INFO(this->get_logger(), "SimManager Port: %d", sim_manager_port);
    this->api.setSimManagerPort(sim_manager_port);

    RCLCPP_INFO(this->get_logger(), "Configuring publisher timers (milliseconds)");
    publisher_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    publisher_timers_.reserve(80);
    auto register_timer = [&](const std::string &suffix, auto &&callable) {
      const std::string parameter_name = "publish_intervals." + suffix;
      const int64_t raw_value = this->declare_parameter<int64_t>(parameter_name, 10);
      const auto interval = sanitize_interval_value(raw_value,
                                                    10U,
                                                    this->get_logger(),
                                                    parameter_name);
      RCLCPP_INFO(this->get_logger(), "%s: %u ms", parameter_name.c_str(), interval);
      auto timer = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_clock(),
        std::chrono::milliseconds(interval),
        std::forward<decltype(callable)>(callable),
        publisher_callback_group_);
      publisher_timers_.push_back(timer);
    };
    register_timer("publish_map2d_ego_position_ms",
                   [this]() { this->publish_map2d_ego_position(); });
    register_timer("publish_map2d_fellow1_position_ms",
                   [this]() { this->publish_map2d_fellow1_position(); });
    register_timer("publish_map2d_fellow2_position_ms",
                   [this]() { this->publish_map2d_fellow2_position(); });
    register_timer("publish_map2d_fellow3_position_ms",
                   [this]() { this->publish_map2d_fellow3_position(); });
    register_timer("publish_base_to_car_summary_ms",
                   [this]() { this->publish_base_to_car_summary(); });
    register_timer("publish_marelli_report_1_ms",
                   [this]() { this->publish_marelli_report_1(); });
    register_timer("publish_marelli_report_2_ms",
                   [this]() { this->publish_marelli_report_2(); });
    register_timer("publish_base_to_car_timing_ms",
                   [this]() { this->publish_base_to_car_timing(); });
    register_timer("publish_rest_of_field_ms",
                   [this]() { this->publish_rest_of_field(); });
    register_timer("publish_pt_report_1_ms",
                   [this]() { this->publish_pt_report_1(); });
    register_timer("publish_pt_report_2_ms",
                   [this]() { this->publish_pt_report_2(); });
    register_timer("publish_pt_report_3_ms",
                   [this]() { this->publish_pt_report_3(); });
    register_timer("publish_steering_report_ms",
                   [this]() { this->publish_steering_report(); });
    register_timer("publish_steering_report_extd_ms",
                   [this]() { this->publish_steering_report_extd(); });
    register_timer("publish_steering_report_extd_2_ms",
                   [this]() { this->publish_steering_report_extd_2(); });
    register_timer("publish_steering_report_extd_3_ms",
                   [this]() { this->publish_steering_report_extd_3(); });
    register_timer("publish_brake_pressure_report_ms",
                   [this]() { this->publish_brake_pressure_report(); });
    register_timer("publish_brake_report_extd_ms",
                   [this]() { this->publish_brake_report_extd(); });
    register_timer("publish_brake_report_extd_2_ms",
                   [this]() { this->publish_brake_report_extd_2(); });
    register_timer("publish_accelerator_report_ms",
                   [this]() { this->publish_accelerator_report(); });
    register_timer("publish_Tire_Temp_RR_1_ms",
                   [this]() { this->publish_Tire_Temp_RR_1(); });
    register_timer("publish_Tire_Temp_RR_2_ms",
                   [this]() { this->publish_Tire_Temp_RR_2(); });
    register_timer("publish_Tire_Temp_RR_3_ms",
                   [this]() { this->publish_Tire_Temp_RR_3(); });
    register_timer("publish_Tire_Temp_RR_4_ms",
                   [this]() { this->publish_Tire_Temp_RR_4(); });
    register_timer("publish_Tire_Temp_RL_1_ms",
                   [this]() { this->publish_Tire_Temp_RL_1(); });
    register_timer("publish_Tire_Temp_RL_2_ms",
                   [this]() { this->publish_Tire_Temp_RL_2(); });
    register_timer("publish_Tire_Temp_RL_3_ms",
                   [this]() { this->publish_Tire_Temp_RL_3(); });
    register_timer("publish_Tire_Temp_RL_4_ms",
                   [this]() { this->publish_Tire_Temp_RL_4(); });
    register_timer("publish_Tire_Temp_FR_1_ms",
                   [this]() { this->publish_Tire_Temp_FR_1(); });
    register_timer("publish_Tire_Temp_FR_2_ms",
                   [this]() { this->publish_Tire_Temp_FR_2(); });
    register_timer("publish_Tire_Temp_FR_3_ms",
                   [this]() { this->publish_Tire_Temp_FR_3(); });
    register_timer("publish_Tire_Temp_FR_4_ms",
                   [this]() { this->publish_Tire_Temp_FR_4(); });
    register_timer("publish_Tire_Temp_FL_1_ms",
                   [this]() { this->publish_Tire_Temp_FL_1(); });
    register_timer("publish_Tire_Temp_FL_2_ms",
                   [this]() { this->publish_Tire_Temp_FL_2(); });
    register_timer("publish_Tire_Temp_FL_3_ms",
                   [this]() { this->publish_Tire_Temp_FL_3(); });
    register_timer("publish_Tire_Temp_FL_4_ms",
                   [this]() { this->publish_Tire_Temp_FL_4(); });
    register_timer("publish_Tire_Pressure_RR_ms",
                   [this]() { this->publish_Tire_Pressure_RR(); });
    register_timer("publish_Tire_Pressure_RL_ms",
                   [this]() { this->publish_Tire_Pressure_RL(); });
    register_timer("publish_Tire_Pressure_FR_ms",
                   [this]() { this->publish_Tire_Pressure_FR(); });
    register_timer("publish_Tire_Pressure_FL_ms",
                   [this]() { this->publish_Tire_Pressure_FL(); });
    register_timer("publish_wheel_strain_gauge_ms",
                   [this]() { this->publish_wheel_strain_gauge(); });
    register_timer("publish_wheel_potentiometer_data_ms",
                   [this]() { this->publish_wheel_potentiometer_data(); });
    register_timer("publish_wheel_speed_report_ms",
                   [this]() { this->publish_wheel_speed_report(); });
    register_timer("publish_misc_report_ms",
                   [this]() { this->publish_misc_report(); });
    register_timer("publish_diagnostic_report_ms",
                   [this]() { this->publish_diagnostic_report(); });
    register_timer("publish_VECTOR__INDEPENDENT_SIG_MSG_ms",
                   [this]() { this->publish_VECTOR__INDEPENDENT_SIG_MSG(); });
    register_timer("publish_novatel_report_ms",
                   [this]() { this->publish_novatel_report(); });
    register_timer("publish_novatel_bestpos1_ms",
                   [this]() { this->publish_novatel_bestpos(1); });
    register_timer("publish_novatel_bestpos2_ms",
                   [this]() { this->publish_novatel_bestpos(2); });
    register_timer("publish_novatel_bestgnsspos1_ms",
                   [this]() { this->publish_novatel_bestgnsspos(1); });
    register_timer("publish_novatel_bestgnsspos2_ms",
                   [this]() { this->publish_novatel_bestgnsspos(2); });
    register_timer("publish_novatel_bestvel1_ms",
                   [this]() { this->publish_novatel_bestvel(1); });
    register_timer("publish_novatel_bestvel2_ms",
                   [this]() { this->publish_novatel_bestvel(2); });
    register_timer("publish_novatel_bestgnssvel1_ms",
                   [this]() { this->publish_novatel_bestgnssvel(1); });
    register_timer("publish_novatel_bestgnssvel2_ms",
                   [this]() { this->publish_novatel_bestgnssvel(2); });
    register_timer("publish_novatel_inspva1_ms",
                   [this]() { this->publish_novatel_inspva(1); });
    register_timer("publish_novatel_inspva2_ms",
                   [this]() { this->publish_novatel_inspva(2); });
    register_timer("publish_novatel_heading21_ms",
                   [this]() { this->publish_novatel_heading2(1); });
    register_timer("publish_novatel_heading22_ms",
                   [this]() { this->publish_novatel_heading2(2); });
    register_timer("publish_novatel_rawimu1_ms",
                   [this]() { this->publish_novatel_rawimu(1); });
    register_timer("publish_novatel_rawimu2_ms",
                   [this]() { this->publish_novatel_rawimu(2); });
    register_timer("publish_novatel_rawimux1_ms",
                   [this]() { this->publish_novatel_rawimux(1); });
    register_timer("publish_novatel_rawimux2_ms",
                   [this]() { this->publish_novatel_rawimux(2); });
    register_timer("publish_vectornav_attitude_group_ms",
                   [this]() { this->publish_vectornav_attitude_group(); });
    register_timer("publish_vectornav_common_group_ms",
                   [this]() { this->publish_vectornav_common_group(); });
    register_timer("publish_vectornav_imu_group_ms",
                   [this]() { this->publish_vectornav_imu_group(); });
    register_timer("publish_vectornav_gps_group_left_ms",
                   [this]() { this->publish_vectornav_gps_group_left(); });
    register_timer("publish_vectornav_gps_group_right_ms",
                   [this]() { this->publish_vectornav_gps_group_right(); });
    register_timer("publish_vectornav_ins_group_ms",
                   [this]() { this->publish_vectornav_ins_group(); });
    register_timer("publish_vectornav_time_group_ms",
                   [this]() { this->publish_vectornav_time_group(); });
    register_timer("publishGroundTruthArray_ms",
                   [this]() { this->publishGroundTruthArray(); });

    this->pathTimeRecord = this->declare_parameter<std::string>(
      "logging.path",
      "/root/record_log");
    RCLCPP_INFO(this->get_logger(), "Execution time log path: %s", this->pathTimeRecord.c_str());

    this->enableTimeRecord = this->declare_parameter<bool>(
      "logging.cycle_time",
      false);
    RCLCPP_INFO(this->get_logger(),
                "Execution cycle time logging %s",
                this->enableTimeRecord ? "enabled" : "disabled");

    const int16_t default_max_retries = 1;
    const int64_t raw_max_retries = this->declare_parameter<int64_t>(
      "connection.max_retries",
      static_cast<int64_t>(default_max_retries));
    max_retries = sanitize_retry_value(raw_max_retries,
                                       default_max_retries,
                                       this->get_logger(),
                                       "Maximum connection retries");
    RCLCPP_INFO(this->get_logger(),
                "Maximum connection retries: %d",
                static_cast<int>(max_retries));

    this->verbosePrinting = this->declare_parameter<bool>(
      "logging.verbose",
      false);
    this->receivedMessagePrinting = this->declare_parameter<bool>(
      "logging.received_can_frames",
      false);
    this->receivedDecodedMessagePrinting = this->declare_parameter<bool>(
      "logging.received_decoded_frames",
      false);
    this->sentMessagePrinting = this->declare_parameter<bool>(
      "logging.sent_can_frames",
      false);

    if (this->verbosePrinting) {
      RCLCPP_INFO(this->get_logger(), "Verbose printing enabled");
    }
    if (this->receivedMessagePrinting) {
      RCLCPP_INFO(this->get_logger(), "Raw CAN frame logging enabled");
    }
    if (this->receivedDecodedMessagePrinting) {
      RCLCPP_INFO(this->get_logger(), "Decoded CAN frame logging enabled");
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(this->get_logger(), "Sent CAN frame logging enabled");
    }

    bool use_sim_time = false;
    if (!this->get_parameter("use_sim_time", use_sim_time)) {
      use_sim_time = false;
    }

    this->simModeEnabled = use_sim_time;
    RCLCPP_INFO(this->get_logger(),
                "Simulation clock mode %s",
                this->simModeEnabled ? "enabled" : "disabled");

    RCLCPP_INFO(get_logger(), "Set Custom Data required to: true");
    this->api.setCustomDataRequired(true);

    RCLCPP_INFO(get_logger(), "Trying to connect to V-ESI at SimManager IP and Port.");
    RCLCPP_INFO(get_logger(), "Trying to connect to ASM at ASM IP with CustomDataInterface set to: true");

    const int16_t vesi_attempts = max_retries > 0 ? max_retries : static_cast<int16_t>(1);
    for (int16_t attempt = 1; attempt <= vesi_attempts; ++attempt) {
      try
      {
        std::list<uint16_t> providedControlDataIDs{22222};
        api.setProvidedControlDataIDs(providedControlDataIDs, true);

        this->api.connect();
        RCLCPP_INFO(get_logger(), "V-ESI connection configured.");
        break;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "Failed to configure V-ESI: %s", e.what());

        if (attempt == vesi_attempts)
        {
          RCLCPP_FATAL(get_logger(), "Failed to often to initialize V-ESI connection; exiting.");
          rclcpp::shutdown();
          return;
        }

        RCLCPP_WARN(get_logger(),
                    "Failed to configure V-ESI (%s), attempt %d/%d; retrying in 1 s",
                    e.what(), attempt, vesi_attempts);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }

    auto pkg_share = ament_index_cpp::get_package_share_directory("asm_socketcan_bridge");

    can1_dbc_path = this->declare_parameter<std::string>(
      "can.can1_dbc_path",
      pkg_share + "/config/CAN1-INDY-V23.dbc");
    auto dbc_path = std::filesystem::path(can1_dbc_path);
    if (!dbc_path.is_absolute()) {
      dbc_path = std::filesystem::path(pkg_share) / dbc_path;
      can1_dbc_path = dbc_path.lexically_normal().string();
      RCLCPP_INFO(this->get_logger(),
                  "CAN1 DBC path resolved relative to package share: %s",
                  can1_dbc_path.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "CAN1 DBC path: %s", can1_dbc_path.c_str());
    }

    can_iface = this->declare_parameter<std::string>(
      "can.interface",
      "vcan0");
    RCLCPP_INFO(this->get_logger(), "CAN interface: %s", can_iface.c_str());

    const int16_t dbc_attempts = max_retries > 0 ? max_retries : static_cast<int16_t>(1);
    for (int16_t attempt = 1; attempt <= dbc_attempts; ++attempt) {
      if (access(can1_dbc_path.c_str(), F_OK) == 0) {
        RCLCPP_INFO(get_logger(), "Found CAN1 DBC at %s", can1_dbc_path.c_str());
        break;
      }

      if (attempt == dbc_attempts)
      {
        RCLCPP_FATAL(get_logger(), "Failed to often to find CAN1 DBC; exiting.");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_WARN(get_logger(),
                  "CAN1 DBC not found (%s), attempt %d/%d; retrying in 1 s",
                  can1_dbc_path.c_str(), attempt, dbc_attempts);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    this->can_socket = open_socket(can_iface);
    if (this->can_socket < 0) {
      RCLCPP_FATAL(get_logger(), "Could not open one or more CAN interfaces; shutting down.");
      if (this->can_socket >= 0) close(this->can_socket);
      rclcpp::shutdown();
      return;
    }
    can_message_info = initialize_messages();
    buildMessageLookup();
    RCLCPP_INFO(get_logger(), "can message structure: %u", can_message_info[0].id);

    reader_thread1 = std::thread([this]() {
      can_reader_loop(this->can_socket, "CAN1");
    });

    RCLCPP_INFO(get_logger(), "CAN initialization done");
    
    try
    {
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_iac);
      const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sim_clock);

      this->groundTruthArrayPublisher_ = this->create_publisher<autonoma_msgs::msg::GroundTruthArray>("ground_truth_array", qos);

      vectornav_publishers_.common =
        this->create_publisher<vectornav_msgs::msg::CommonGroup>("vectornav/raw/common", qos);
      vectornav_publishers_.attitude =
        this->create_publisher<vectornav_msgs::msg::AttitudeGroup>("vectornav/raw/attitude", qos);
      vectornav_publishers_.imu =
        this->create_publisher<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", qos);
      vectornav_publishers_.ins =
        this->create_publisher<vectornav_msgs::msg::InsGroup>("vectornav/raw/ins", qos);
      vectornav_publishers_.gps[kVectorNavGpsLeftIndex] =
        this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps_left", qos);
      vectornav_publishers_.gps[kVectorNavGpsRightIndex] =
        this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps_right", qos);
      vectornav_publishers_.time =
        this->create_publisher<vectornav_msgs::msg::TimeGroup>("vectornav/raw/time", qos);

      auto &novatel_top = novatel_publishers_[kNovatelTopIndex];
      novatel_top.best_pos =
        this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_top/bestpos", qos);
      novatel_top.best_gnss_pos =
        this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_top/bestgnsspos", qos);
      novatel_top.best_vel =
        this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_top/bestvel", qos);
      novatel_top.best_gnss_vel =
        this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_top/bestgnssvel", qos);
      novatel_top.inspva =
        this->create_publisher<novatel_oem7_msgs::msg::INSPVA>("novatel_top/inspva", qos);
      novatel_top.heading2 =
        this->create_publisher<novatel_oem7_msgs::msg::HEADING2>("novatel_top/heading2", qos);
      novatel_top.raw_imu =
        this->create_publisher<novatel_oem7_msgs::msg::RAWIMU>("novatel_top/rawimu", qos);
      novatel_top.raw_imu_x =
        this->create_publisher<sensor_msgs::msg::Imu>("novatel_top/rawimux", qos);

      auto &novatel_bottom = novatel_publishers_[kNovatelBottomIndex];
      novatel_bottom.best_pos =
        this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_bottom/bestpos", qos);
      novatel_bottom.best_gnss_pos =
        this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_bottom/bestgnsspos", qos);
      novatel_bottom.best_vel =
        this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_bottom/bestvel", qos);
      novatel_bottom.best_gnss_vel =
        this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_bottom/bestgnssvel", qos);
      novatel_bottom.inspva =
        this->create_publisher<novatel_oem7_msgs::msg::INSPVA>("novatel_bottom/inspva", qos);
      novatel_bottom.heading2 =
        this->create_publisher<novatel_oem7_msgs::msg::HEADING2>("novatel_bottom/heading2", qos);
      novatel_bottom.raw_imu =
        this->create_publisher<novatel_oem7_msgs::msg::RAWIMU>("novatel_bottom/rawimu", qos);
      novatel_bottom.raw_imu_x =
        this->create_publisher<sensor_msgs::msg::Imu>("novatel_bottom/rawimux", qos);

      this->foxgloveMapPublisher0_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("map2d_ego_position", qos);
      this->foxgloveMapPublisher1_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("map2d_fellow1_position", qos);
      this->foxgloveMapPublisher2_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("map2d_fellow2_position", qos);
      this->foxgloveMapPublisher3_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("map2d_fellow3_position", qos);
      this->resetCommandPublisher_ = this->create_publisher<std_msgs::msg::Bool>("maneuver_reset", qos);

      this->useCustomRaceControlSource_ = this->create_subscription<std_msgs::msg::Bool>("use_custom_race_control", qos, std::bind(&AsmSocketCanBridgeNode::switchRaceControlSourceCallback, this, _1));
      initializeFeedback();

      const auto acquisition_period = this->simModeEnabled ? 1ms : 10ms;
      this->vesiAcquisitionTimer_ = this->create_wall_timer(
        acquisition_period,
        std::bind(&AsmSocketCanBridgeNode::vesiCallback, this));

      if(this->simModeEnabled)
      {
        RCLCPP_INFO(get_logger(), "Use Simulated Clock.");
        this->simClockTimePublisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", sim_qos);      
        this->simTimeIncrease_ = this->create_subscription<std_msgs::msg::UInt16>("sim_time_increase", sim_qos, std::bind(&AsmSocketCanBridgeNode::simTimeIncreaseCallback, this, _1));
        vesiCallback();
        this->simClockTime.clock = rclcpp::Time(this->sec, this->nsec);
        this->simClockTimePublisher_->publish(this->simClockTime);
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Use Wall Clock (system clock).");
        vesiCallback();
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create object for ASM-ROS2-Bridge node: %s", e.what());
    }

    if (this->enableTimeRecord)
    {
      this->myfile.open(std::string(this->pathTimeRecord) + "/duration_recording.csv");
      this->myfile << "sendVehicleFeedbackToSimulation" << ","
                   << "requestCustomData" << ","
                   << "castCanbus_raw" << ","
                   << "publishers" << ","
                   << "VESICallBackInterval" << "\n";
      this->myfile.close();
      RCLCPP_INFO(get_logger(), "Log created under path: %s", this->pathTimeRecord.c_str());
    }

    RCLCPP_INFO(get_logger(), "Setup done.");
  }

  AsmSocketCanBridgeNode::~AsmSocketCanBridgeNode()
  {
    stop_reader_.store(true);
    if (can_socket >= 0) {
      close(can_socket);
      can_socket = -1;
    }
    if (reader_thread1.joinable()) {
      reader_thread1.join();
    }
  }

  template <typename T>
  void AsmSocketCanBridgeNode::insertBits(uint8_t* data,
                                          Signal signal_information,
                                          T physical_value)
  {
    if (signal_information.length == 0) {
      return;
    }

    const auto length = signal_information.length;
    const auto mask = length >= 64 ? std::numeric_limits<uint64_t>::max()
                                   : ((1ULL << length) - 1ULL);
    const long double scaled_value =
      (static_cast<long double>(physical_value) - static_cast<long double>(signal_information.offset)) /
      static_cast<long double>(signal_information.factor);

    uint64_t raw_value = 0;

    if (signal_information.is_signed) {
      int64_t min_value;
      int64_t max_value;
      if (length >= 64) {
        min_value = std::numeric_limits<int64_t>::min();
        max_value = std::numeric_limits<int64_t>::max();
      } else {
        const int64_t magnitude = static_cast<int64_t>(1ULL << (length - 1));
        min_value = -magnitude;
        max_value = magnitude - 1;
      }
      const long double rounded = std::round(scaled_value);
      const long double clamped =
        std::clamp<long double>(rounded,
                                static_cast<long double>(min_value),
                                static_cast<long double>(max_value));
      const auto quantized = static_cast<int64_t>(clamped);
      raw_value = static_cast<uint64_t>(quantized) & mask;
    } else {
      const long double rounded = std::round(scaled_value);
      const long double clamped =
        std::clamp<long double>(rounded, 0.0L, static_cast<long double>(mask));
      raw_value = static_cast<uint64_t>(clamped);
    }

    for (int i = 0; i < signal_information.length; ++i) {
      const int bitIndex =
        signal_information.endian ? (signal_information.start_bit + i)
                                  : (signal_information.start_bit - i);
      const int byteIndex = bitIndex / 8;
      const int bitInByte =
        signal_information.endian ? (bitIndex % 8) : (7 - (bitIndex % 8));

      const uint8_t bitVal = (raw_value >> i) & 0x01;
      data[byteIndex] &= ~(1 << bitInByte);
      data[byteIndex] |= (bitVal << bitInByte);
    }
  }

  int32_t AsmSocketCanBridgeNode::extractBits(const uint8_t* data,
                                              Signal signal_information) const
  {
    int32_t result = 0;
    for (int i = 0; i < signal_information.length; ++i) {
      int bitIndex = signal_information.endian ? (signal_information.start_bit + i)
                                                : (signal_information.start_bit - i);
      int byteIndex = bitIndex / 8;
      int bitInByte = signal_information.endian ? (bitIndex % 8)
                                                : (7 - (bitIndex % 8));
      uint8_t bitVal = (data[byteIndex] >> bitInByte) & 0x01;
      result |= (bitVal << i);
    }
    if (signal_information.is_signed) {
      int signBit = 1 << (signal_information.length - 1);
      if (result & signBit) {
        result |= ~((1 << signal_information.length) - 1);
      }
    }

    return result;
  }

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

    this->feedbackCmd.vehicle_inputs.brake_cmd_front = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_rear = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_bias_switch = 0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_brake_cmd = 0;

    this->feedbackCmd.vehicle_inputs.steering_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.steering_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_steering_cmd = 0;

    this->feedbackCmd.vehicle_inputs.gear_cmd = 1;
    this->feedbackCmd.vehicle_inputs.enable_gear_cmd = 0;
    
    this->feedbackCmd.to_raptor.track_cond_ack = 0;
    this->feedbackCmd.to_raptor.veh_sig_ack = 0;
    this->feedbackCmd.to_raptor.ct_state = 0;
    this->feedbackCmd.to_raptor.rolling_counter = 0;
    this->feedbackCmd.to_raptor.veh_num = 255;

    this->feedbackCmd.to_raptor.push2pass_request = false;
  }

  int AsmSocketCanBridgeNode::open_socket(const std::string &iface)
  {
    RCLCPP_INFO(get_logger(), "Socket open...");
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
      RCLCPP_ERROR(get_logger(),
                  "socket() failed for interface %s: %s",
                  iface.c_str(), strerror(errno));
      return -1;
    }

    struct ifreq ifr {};
    strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(),
                  "ioctl(SIOCGIFINDEX) failed for %s: %s",
                  iface.c_str(), strerror(errno));
      close(s);
      return -1;
    }

    RCLCPP_INFO(get_logger(), "Socket bind...");
    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(),
                  "bind() failed for %s: %s",
                  iface.c_str(), strerror(errno));
      close(s);
      return -1;
    }

    return s;
  }

  void AsmSocketCanBridgeNode::can_reader_loop(int sock,
                                               const std::string &bus_id)
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
          RCLCPP_INFO(get_logger(), "received: %02X ",in_frame.data[i]);
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
              static_cast<uint16_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("R_brake_pressure_cmd")) {
            this->feedbackCmd.vehicle_inputs.brake_cmd_rear =
              static_cast<uint16_t>(std::floor(*value));
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
            this->feedbackCmd.vehicle_inputs.throttle_cmd_count =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("acc_pedal_cmd")) {
            this->feedbackCmd.vehicle_inputs.throttle_cmd =
              static_cast<float>(*value);
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
          float driver_steering_P_cmd = 0.0f;
          float driver_steering_I_cmd = 0.0f;
          float driver_steering_D_cmd = 0.0f;
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("steering_motor_cmd_counter")) {
            this->feedbackCmd.vehicle_inputs.steering_cmd_count =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("steering_motor_ang_cmd")) {
            this->feedbackCmd.vehicle_inputs.steering_cmd =
              static_cast<float>(*value);
          }
          if (const auto value = get_scaled("driver_steering_P_cmd")) {
            driver_steering_P_cmd = static_cast<float>(*value);
          }
          if (const auto value = get_scaled("driver_steering_I_cmd")) {
            driver_steering_I_cmd = static_cast<float>(*value);
          }
          if (const auto value = get_scaled("driver_steering_D_cmd")) {
            driver_steering_D_cmd = static_cast<float>(*value);
          }
          this->feedbackCmd.vehicle_inputs.enable_steering_cmd = 1;
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "steering_cmd_count: %d  steering_cmd: %f  enable_steering_cmd: %d  driver_steering_P_cmd: %f  driver_steering_I_cmd: %f  driver_steering_D_cmd: %f  ",
                        this->feedbackCmd.vehicle_inputs.steering_cmd_count,
                        static_cast<double>(this->feedbackCmd.vehicle_inputs.steering_cmd),
                        this->feedbackCmd.vehicle_inputs.enable_steering_cmd,
                        static_cast<double>(driver_steering_P_cmd),
                        static_cast<double>(driver_steering_I_cmd),
                        static_cast<double>(driver_steering_D_cmd));
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
          if (const auto value = extractSignalScaled(in_frame.can_id,
                                                     "desired_gear",
                                                     in_frame.data)) {
            this->feedbackCmd.vehicle_inputs.brake_cmd_count =
              static_cast<uint8_t>(std::floor(*value));
          }
          this->feedbackCmd.vehicle_inputs.enable_gear_cmd = 1;
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "desired_gear: %d",
                        this->feedbackCmd.vehicle_inputs.brake_cmd_count);
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
          uint8_t driver_traction_aim_switch = 0U;
          uint8_t driver_traction_range_switch = 0U;
          uint8_t drive_steering_gain_cntrl_switch = 0U;
          const auto get_scaled = [&](std::string_view name) {
            return extractSignalScaled(in_frame.can_id, name, in_frame.data);
          };
          if (const auto value = get_scaled("brake_bias_aim_switch")) {
            this->feedbackCmd.vehicle_inputs.brake_bias_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("push2pass_request")) {
            this->feedbackCmd.to_raptor.push2pass_request =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("driver_traction_aim_switch")) {
            driver_traction_aim_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("driver_traction_range_switch")) {
            driver_traction_range_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          if (const auto value = get_scaled("drive_steering_gain_cntrl_switch")) {
            drive_steering_gain_cntrl_switch =
              static_cast<uint8_t>(std::floor(*value));
          }
          this->feedbackDataAvailabe = true;
          if (this->receivedDecodedMessagePrinting) {
            RCLCPP_INFO(get_logger(),
                        "brake_bias_aim_switch: %d  push2pass_request: %d  driver_traction_aim_switch: %d  driver_traction_range_switch: %d  drive_steering_gain_cntrl_switch: %d",
                        this->feedbackCmd.vehicle_inputs.brake_bias_switch,
                        this->feedbackCmd.to_raptor.push2pass_request,
                        driver_traction_aim_switch,
                        driver_traction_range_switch,
                        drive_steering_gain_cntrl_switch);
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
          this->feedbackDataAvailabe = true;
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

  void AsmSocketCanBridgeNode::can_write(int sock,
                                         const struct can_frame &frame)
  {
    if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(frame)) {
      perror("Write");
      return;
    }
  }

  void AsmSocketCanBridgeNode::buildMessageLookup()
  {
    message_lookup_.clear();
    message_signal_lookup_.clear();
    message_name_lookup_.clear();
    message_lookup_.reserve(can_message_info.size());
    message_signal_lookup_.reserve(can_message_info.size());
    message_name_lookup_.reserve(can_message_info.size());

    for (auto &message : can_message_info) {
      message_lookup_.emplace(message.id, &message);
      message_name_lookup_.emplace(std::string_view(message.name), &message);
      auto &signal_map = message_signal_lookup_[message.id];
      signal_map.reserve(message.signals.size());
      for (const auto &signal : message.signals) {
        signal_map.emplace(signal.name, &signal);
      }
    }
  }

  const Message* AsmSocketCanBridgeNode::findMessageByID(uint32_t message_id) const
  {
    const auto iter = message_lookup_.find(message_id);
    if (iter == message_lookup_.end()) {
      return nullptr;
    }
    return iter->second;
  }

  const AsmSocketCanBridgeNode::SignalLookupMap* AsmSocketCanBridgeNode::findSignalLookup(uint32_t message_id) const
  {
    const auto iter = message_signal_lookup_.find(message_id);
    if (iter == message_signal_lookup_.end()) {
      return nullptr;
    }
    return &iter->second;
  }

  const Message* AsmSocketCanBridgeNode::findMessageByName(std::string_view message_name) const
  {
    const auto iter = message_name_lookup_.find(message_name);
    if (iter == message_name_lookup_.end()) {
      return nullptr;
    }
    return iter->second;
  }

  std::optional<AsmSocketCanBridgeNode::PreparedCanMessage>
  AsmSocketCanBridgeNode::prepareCanMessage(std::string_view message_name)
  {
    const auto *message = findMessageByName(message_name);
    if (!message) {
      RCLCPP_ERROR(get_logger(),
                   "CAN metadata missing for message %.*s",
                   static_cast<int>(message_name.size()),
                   message_name.data());
      return std::nullopt;
    }
    PreparedCanMessage prepared{};
    prepared.metadata = message;
    prepared.frame.can_id = message->id;
    prepared.frame.can_dlc = message->dlc;
    std::memset(prepared.frame.data, 0, sizeof(prepared.frame.data));
    return prepared;
  }

  void AsmSocketCanBridgeNode::finalizeCanMessage(const PreparedCanMessage &message)
  {
    if (sentMessagePrinting && message.metadata) {
      RCLCPP_INFO(get_logger(), "can_out::%s", message.metadata->name);
      RCLCPP_INFO(get_logger(),
                  "send: 0x%03X [%d] ",
                  message.metadata->id,
                  static_cast<int>(message.frame.can_dlc));
      for (int i = 0; i < message.frame.can_dlc; i++) {
        RCLCPP_INFO(get_logger(), "send: %02X ", message.frame.data[i]);
      }
    }
    const std::lock_guard<std::mutex> socket_lock(can_socket_mutex_);
    can_write(can_socket, message.frame);
  }

  void AsmSocketCanBridgeNode::setHeader(std_msgs::msg::Header &header,
                                         std::string_view frame_id) const
  {
    header.frame_id = std::string(frame_id);
    if (simModeEnabled) {
      header.stamp.sec = sec;
      header.stamp.nanosec = nsec;
      return;
    }
    const auto now = std::chrono::system_clock::now();
    const auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now).time_since_epoch().count();
    const auto nsecs_total = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();
    header.stamp.sec = static_cast<int32_t>(secs);
    header.stamp.nanosec = static_cast<uint32_t>(nsecs_total - (secs * 1000000000LL));
  }

  void AsmSocketCanBridgeNode::populateBestPosMessage(novatel_oem7_msgs::msg::BESTPOS &message,
                                                      const nova_tel_pwr_pak &data) const
  {
    const auto &source = data.best_pos_var;
    message.nov_header.message_name = source.nov_header_var.message_name[0];
    message.nov_header.message_id = source.nov_header_var.message_id;
    message.nov_header.message_type = source.nov_header_var.message_type;
    message.nov_header.sequence_number = source.nov_header_var.sequence_number;
    message.nov_header.time_status = source.nov_header_var.time_status;
    message.nov_header.gps_week_number = source.nov_header_var.gps_week_number;
    message.nov_header.gps_week_milliseconds = source.nov_header_var.gps_week_milliseconds;
    message.nov_header.idle_time = source.nov_header_var.idle_time;
    message.sol_status.status = source.sol_status;
    message.pos_type.type = source.pos_type;
    message.lat = source.lat;
    message.lon = source.lon;
    message.hgt = source.hgt;
    message.undulation = source.undulation;
    message.datum_id = source.datum_id;
    message.lat_stdev = source.lat_stdev;
    message.lon_stdev = source.lon_stdev;
    message.hgt_stdev = source.hgt_stdev;
    for (size_t i = 0; i < message.stn_id.size(); ++i) {
      message.stn_id[i] = source.stn_id[i];
    }
    message.diff_age = source.diff_age;
    message.sol_age = source.sol_age;
    message.num_svs = source.num_svs;
    message.num_sol_svs = source.num_sol_svs;
    message.num_sol_l1_svs = source.num_sol_l1_svs;
    message.num_sol_multi_svs = source.num_sol_multi_svs;
    message.reserved = source.reserved;
    message.ext_sol_stat.status = source.ext_sol_stat;
    message.galileo_beidou_sig_mask = source.galileo_beidou_sig_mask;
    message.gps_glonass_sig_mask = source.gps_glonass_sig_mask;
    setHeader(message.header, "world");
  }

  void AsmSocketCanBridgeNode::populateBestVelMessage(novatel_oem7_msgs::msg::BESTVEL &message,
                                                      const nova_tel_pwr_pak &data) const
  {
    const auto &source = data.best_vel_var;
    message.nov_header.message_name = source.nov_header_var.message_name[0];
    message.nov_header.message_id = source.nov_header_var.message_id;
    message.nov_header.message_type = source.nov_header_var.message_type;
    message.nov_header.sequence_number = source.nov_header_var.sequence_number;
    message.nov_header.time_status = source.nov_header_var.time_status;
    message.nov_header.gps_week_number = source.nov_header_var.gps_week_number;
    message.nov_header.gps_week_milliseconds = source.nov_header_var.gps_week_milliseconds;
    message.nov_header.idle_time = source.nov_header_var.idle_time;
    message.sol_status.status = source.sol_status;
    message.vel_type.type = source.vel_type;
    message.latency = source.latency;
    message.diff_age = source.diff_age;
    message.hor_speed = source.hor_speed;
    message.trk_gnd = source.trk_gnd;
    message.ver_speed = source.ver_speed;
    message.reserved = source.reserved;
    setHeader(message.header, "ego");
  }

  void AsmSocketCanBridgeNode::populateInspvaMessage(novatel_oem7_msgs::msg::INSPVA &message,
                                                      const nova_tel_pwr_pak &data) const
  {
    const auto &source = data.inspava_var;
    message.nov_header.message_name = source.nov_header_var.message_name[0];
    message.nov_header.message_id = source.nov_header_var.message_id;
    message.nov_header.message_type = source.nov_header_var.message_type;
    message.nov_header.sequence_number = source.nov_header_var.sequence_number;
    message.nov_header.time_status = source.nov_header_var.time_status;
    message.nov_header.gps_week_number = source.nov_header_var.gps_week_number;
    message.nov_header.gps_week_milliseconds = source.nov_header_var.gps_week_milliseconds;
    message.nov_header.idle_time = source.nov_header_var.idle_time;
    message.latitude = source.latitude;
    message.longitude = source.longitude;
    message.height = source.height;
    message.north_velocity = source.north_velocity;
    message.east_velocity = source.east_velocity;
    message.up_velocity = source.up_velocity;
    message.roll = source.roll;
    message.pitch = source.pitch;
    message.azimuth = source.azimuth;
    message.status.status = source.status_var.status_var;
    setHeader(message.header, "world");
  }

  void AsmSocketCanBridgeNode::populateHeading2Message(novatel_oem7_msgs::msg::HEADING2 &message,
                                                       const nova_tel_pwr_pak &data) const
  {
    const auto &source = data.heading_2_var;
    message.nov_header.message_name = source.nov_header_var.message_name[0];
    message.nov_header.message_id = source.nov_header_var.message_id;
    message.nov_header.message_type = source.nov_header_var.message_type;
    message.nov_header.sequence_number = source.nov_header_var.sequence_number;
    message.nov_header.time_status = source.nov_header_var.time_status;
    message.nov_header.gps_week_number = source.nov_header_var.gps_week_number;
    message.nov_header.gps_week_milliseconds = source.nov_header_var.gps_week_milliseconds;
    message.nov_header.idle_time = source.nov_header_var.idle_time;
    message.sol_status.status = source.sol_status;
    message.pos_type.type = source.pos_type;
    message.length = source.length;
    message.heading = source.heading;
    message.pitch = source.pitch;
    message.reserved = source.reserved;
    message.heading_stdev = source.heading_stdev;
    message.pitch_stdev = source.pitch_stdev;
    for (size_t i = 0; i < message.rover_stn_id.size(); ++i) {
      message.rover_stn_id[i] = source.rover_stn_id[i];
    }
    for (size_t i = 0; i < message.master_stn_id.size(); ++i) {
      message.master_stn_id[i] = source.master_stn_id[i];
    }
    message.num_sv_tracked = source.num_sv_tracked;
    message.num_sv_in_sol = source.num_sv_in_sol;
    message.num_sv_obs = source.num_sv_obs;
    message.num_sv_multi = source.num_sv_multi;
    message.sol_source.source = source.sol_source;
    message.ext_sol_status.status = source.ext_sol_status;
    message.galileo_beidou_sig_mask = source.galileo_beidou_sig_mask;
    message.gps_glonass_sig_mask = source.gps_glonass_sig_mask;
    setHeader(message.header, "world");
  }

  void AsmSocketCanBridgeNode::populateRawImuMessage(novatel_oem7_msgs::msg::RAWIMU &message,
                                                     const nova_tel_pwr_pak &data) const
  {
    const auto &source = data.raw_imu_var;
    message.nov_header.message_name = source.nov_header_var.message_name[0];
    message.nov_header.message_id = source.nov_header_var.message_id;
    message.nov_header.message_type = source.nov_header_var.message_type;
    message.nov_header.sequence_number = source.nov_header_var.sequence_number;
    message.nov_header.time_status = source.nov_header_var.time_status;
    message.nov_header.gps_week_number = source.nov_header_var.gps_week_number;
    message.nov_header.gps_week_milliseconds = source.nov_header_var.gps_week_milliseconds;
    message.nov_header.idle_time = source.nov_header_var.idle_time;
    message.gnss_week = source.gnss_week;
    message.gnss_seconds = source.gnss_seconds;
    message.status = source.status_var;
    message.linear_acceleration.x = source.linear_acceleration_var.x;
    message.linear_acceleration.y = source.linear_acceleration_var.y;
    message.linear_acceleration.z = source.linear_acceleration_var.z;
    message.angular_velocity.x = source.angular_velocity_var.x;
    message.angular_velocity.y = source.angular_velocity_var.y;
    message.angular_velocity.z = source.angular_velocity_var.z;
    setHeader(message.header, "ego");
  }

  void AsmSocketCanBridgeNode::populateRawImuXMessage(sensor_msgs::msg::Imu &message,
                                                      const nova_tel_pwr_pak &data) const
  {
    const auto &source = data.raw_imu_var;
    setHeader(message.header, "ego");
    message.orientation.x = 0.0;
    message.orientation.y = 0.0;
    message.orientation.z = 0.0;
    message.orientation.w = 0.0;
    for (auto &value : message.orientation_covariance) {
      value = -1.0;
    }
    message.angular_velocity.x = source.angular_velocity_var.x;
    message.angular_velocity.y = source.angular_velocity_var.y;
    message.angular_velocity.z = source.angular_velocity_var.z;
    for (auto &value : message.angular_velocity_covariance) {
      value = 0.0;
    }
    message.linear_acceleration.x = source.linear_acceleration_var.x;
    message.linear_acceleration.y = source.linear_acceleration_var.y;
    message.linear_acceleration.z = source.linear_acceleration_var.z;
    for (auto &value : message.linear_acceleration_covariance) {
      value = 0.0;
    }
  }

  void AsmSocketCanBridgeNode::populateGpsGroupMessage(vectornav_msgs::msg::GpsGroup &message,
                                                       const gps_group &source)
  {
    setHeader(message.header, "world");
    message.utc.year = source.utc_var.year;
    message.utc.month = source.utc_var.month;
    message.utc.day = source.utc_var.day;
    message.utc.hour = source.utc_var.hour;
    message.utc.min = source.utc_var.min;
    message.utc.sec = source.utc_var.sec;
    message.utc.ms = source.utc_var.ms;
    message.tow = source.tow;
    message.week = source.week;
    message.numsats = source.numsats;
    message.fix = source.fix;
    message.poslla.x = source.poslla_var.x;
    message.poslla.y = source.poslla_var.y;
    message.poslla.z = source.poslla_var.z;
    message.posecef.x = source.posecef_var.x;
    message.posecef.y = source.posecef_var.y;
    message.posecef.z = source.posecef_var.z;
    message.velned.x = source.velned_var.x;
    message.velned.y = source.velned_var.y;
    message.velned.z = source.velned_var.z;
    message.velecef.x = source.velecef_var.x;
    message.velecef.y = source.velecef_var.y;
    message.velecef.z = source.velecef_var.z;
    message.posu.x = source.posu_var.x;
    message.posu.y = source.posu_var.y;
    message.posu.z = source.posu_var.z;
    message.velu = source.velu;
    message.timeu = source.timeu;
    message.timeinfo_status = source.timeinfo_status;
    message.timeinfo_leapseconds = source.timeinfo_leapseconds;
    message.dop.g = source.dop_var.g;
    message.dop.p = source.dop_var.p;
    message.dop.t = source.dop_var.t;
    message.dop.v = source.dop_var.v;
    message.dop.h = source.dop_var.h;
    message.dop.n = source.dop_var.n;
    message.dop.e = source.dop_var.e;
  }

  const Signal* AsmSocketCanBridgeNode::findSignal(uint32_t message_id,
                                                   std::string_view signal_name) const
  {
    const auto *signal_map = findSignalLookup(message_id);
    if (!signal_map) {
      return nullptr;
    }

    const auto iter = signal_map->find(signal_name);
    if (iter == signal_map->end()) {
      return nullptr;
    }
    return iter->second;
  }

  std::optional<double>
  AsmSocketCanBridgeNode::extractSignalScaled(uint32_t message_id,
                                              std::string_view signal_name,
                                              const uint8_t* data) const
  {
    const auto *signal = findSignal(message_id, signal_name);
    if (!signal) {
      return std::nullopt;
    }
    const auto raw = extractBits(data, *signal);
    return static_cast<double>(raw) * signal->factor + signal->offset;
  }

  void AsmSocketCanBridgeNode::simClockTimeCallback()
  {
    std::unique_lock<std::shared_mutex> lock(can_bus_mutex_);
    simClockTime.clock = rclcpp::Time(this->sec,this->nsec);
    this->simClockTimePublisher_->publish(simClockTime);
  }

  void AsmSocketCanBridgeNode::simTimeIncreaseCallback(const std_msgs::msg::UInt16 & msg)
  {
    for (uint16_t timeIncreaseStep = 0; timeIncreaseStep < msg.data; ++timeIncreaseStep) {
      vesiCallback();
    }
    simClockTimeCallback();
  }

  void AsmSocketCanBridgeNode::vesiCallback()
  {
    auto now_ns = []() {
      return std::chrono::time_point_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now()).time_since_epoch().count();
    };

    const bool record_metrics = this->enableTimeRecord;

    double interval_ms = 0.0;
    double send_feedback_ms = 0.0;
    double request_data_ms = 0.0;
    double cast_ms = 0.0;

    if (record_metrics) {
      const auto now = now_ns();
      const auto previous_start = last_callback_start_ns;
      if (previous_start != 0) {
        interval_ms =
          static_cast<double>(static_cast<long long>(now) - static_cast<long long>(previous_start)) /
          1000000.0;
      }
      last_callback_start_ns = now;
    }

    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "vesiCallback");

    auto measure = [&](auto &&callable, double &out_ms) {
      if (!record_metrics) {
        std::forward<decltype(callable)>(callable)();
        out_ms = 0.0;
        return;
      }
      const auto start = now_ns();
      std::forward<decltype(callable)>(callable)();
      const auto end = now_ns();
      out_ms =
        static_cast<double>(static_cast<long long>(end) - static_cast<long long>(start)) / 1000000.0;
    };

    bool active_maneuver = false;
    {
      std::shared_lock<std::shared_mutex> lock(can_bus_mutex_);
      active_maneuver = this->maneuverStarted;
    }

    bool publish_reset = false;

    try {
      if (active_maneuver) {
        measure([&]() { AsmSocketCanBridgeNode::sendVehicleFeedbackToSimulation(); },
                send_feedback_ms);
        measure([&]() { this->api.requestCustomData(&canbus_raw_buffer_); }, request_data_ms);
      } else {
        send_feedback_ms = 0.0;
        measure([&]() { this->api.requestCustomData(&canbus_raw_buffer_); }, request_data_ms);
      }

      measure([&]() {
        constexpr auto required_canbus_size = sizeof(ASMBus);
        std::unique_lock<std::shared_mutex> lock(can_bus_mutex_);
        if (active_maneuver) {
          if (this->simModeEnabled) {
            this->simTotalMsec += 1;
            this->sec = this->simTotalMsec / 1000;
            this->nsec = (this->simTotalMsec % 1000) * 1000000;
          } else {
            this->simTotalMsec += 10;
          }
        }

        if (canbus_raw_buffer_.size() >= required_canbus_size) {
          std::memcpy(&canBusStorage_, canbus_raw_buffer_.data(), required_canbus_size);
          this->canBus = &canBusStorage_;
          if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState == 3 &&
              !this->maneuverStarted) {
            this->maneuverStarted = true;
            RCLCPP_INFO(get_logger(), "Maneuver started. Data will be published.");
          } else if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState != 3 &&
                     this->maneuverStarted) {
            RCLCPP_INFO(get_logger(), "Maneuver stopped. System will be reset.");
            initializeFeedback();
            publish_reset = true;
          }
          this->vesiDataAvailabe = true;
        } else {
          this->canBus = nullptr;
          this->vesiDataAvailabe = false;
          if (canbus_raw_buffer_.empty()) {
            if (this->verbosePrinting) {
              RCLCPP_WARN(get_logger(), "No Custom Data available.");
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(),
                                   *this->get_clock(),
                                   5000,
                                   "No Custom Data available.");
            }
          } else {
            RCLCPP_ERROR(get_logger(),
                         "Custom data buffer size (%zu) smaller than ASMBus (%zu); ignoring frame.",
                         canbus_raw_buffer_.size(),
                         required_canbus_size);
          }
        }
      }, cast_ms);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to request data from ASM: %s", e.what());
      return;
    }

    if (publish_reset) {
      std_msgs::msg::Bool resetMsg;
      resetMsg.data = true;
      this->resetCommandPublisher_->publish(resetMsg);
    }

    if (record_metrics) {
      std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
      this->myfile.open(std::string(this->pathTimeRecord) + "/duration_recording.csv",
                        std::ios_base::app);
      this->myfile << std::to_string(send_feedback_ms) << ","
                   << std::to_string(request_data_ms) << ","
                   << std::to_string(cast_ms) << ","
                   << std::to_string(0.0) << ","
                   << std::to_string(interval_ms) << "\n";
      this->myfile.close();
    }
    if (this->simModeEnabled && this->simClockTimePublisher_) {
      std::unique_lock<std::shared_mutex> lock(can_bus_mutex_);
      simClockTime.clock = rclcpp::Time(this->sec, this->nsec);
      this->simClockTimePublisher_->publish(simClockTime);
    }
  }

  void AsmSocketCanBridgeNode::publish_map2d_ego_position()
  {
    this->publishFoxgloveMapEntry(0);
  }

  void AsmSocketCanBridgeNode::publish_map2d_fellow1_position()
  {
    this->publishFoxgloveMapEntry(1);
  }

  void AsmSocketCanBridgeNode::publish_map2d_fellow2_position()
  {
    this->publishFoxgloveMapEntry(2);
  }

  void AsmSocketCanBridgeNode::publish_map2d_fellow3_position()
  {
    this->publishFoxgloveMapEntry(3);
  }

  void AsmSocketCanBridgeNode::publishFoxgloveMapEntry(uint8_t fellowID)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "publishFoxgloveMap fellowID: %d", fellowID);

    auto foxgloveMap = sensor_msgs::msg::NavSatFix();

    foxgloveMap.status.status = -1;
    foxgloveMap.status.service = 1;

    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      if (fellowID == 0) {
        foxgloveMap.latitude = bus.sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat;
        foxgloveMap.longitude = bus.sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon;
        foxgloveMap.altitude = bus.sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.hgt;
        populated = true;
        return;
      }
      if (fellowID == 1) {
        foxgloveMap.latitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[0];
        foxgloveMap.longitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[0];
        foxgloveMap.altitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[0];
        populated = true;
        return;
      }
      if (fellowID == 2) {
        foxgloveMap.latitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[1];
        foxgloveMap.longitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[1];
        foxgloveMap.altitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[1];
        populated = true;
        return;
      }
      if (fellowID == 3) {
        foxgloveMap.latitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[2];
        foxgloveMap.longitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[2];
        foxgloveMap.altitude = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[2];
        populated = true;
        return;
      }
    })) {
      return;
    }
    if (!populated) {
      RCLCPP_ERROR(get_logger(), "Unknown Fellow ID. Only three Fellows are supported.");
      return;
    }

    foxgloveMap.position_covariance_type = 0;
    setHeader(foxgloveMap.header, "world");

    switch (fellowID) {
      case 0:
        if (!this->foxgloveMapPublisher0_) {
          RCLCPP_ERROR(get_logger(), "Foxglove publisher unavailable for fellow ID %u", static_cast<unsigned>(fellowID));
          return;
        }
        this->foxgloveMapPublisher0_->publish(foxgloveMap);
        break;
      case 1:
        if (!this->foxgloveMapPublisher1_) {
          RCLCPP_ERROR(get_logger(), "Foxglove publisher unavailable for fellow ID %u", static_cast<unsigned>(fellowID));
          return;
        }
        this->foxgloveMapPublisher1_->publish(foxgloveMap);
        break;
      case 2:
        if (!this->foxgloveMapPublisher2_) {
          RCLCPP_ERROR(get_logger(), "Foxglove publisher unavailable for fellow ID %u", static_cast<unsigned>(fellowID));
          return;
        }
        this->foxgloveMapPublisher2_->publish(foxgloveMap);
        break;
      case 3:
        if (!this->foxgloveMapPublisher3_) {
          RCLCPP_ERROR(get_logger(), "Foxglove publisher unavailable for fellow ID %u", static_cast<unsigned>(fellowID));
          return;
        }
        this->foxgloveMapPublisher3_->publish(foxgloveMap);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Foxglove publisher selection failed for fellow ID %u", static_cast<unsigned>(fellowID));
        break;
    }
  }

  void AsmSocketCanBridgeNode::sendVehicleFeedbackToSimulation()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "sendVehicleFeedbackToSimulation");
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);

      static bool raptor_connection_announced = false;
      if (!this->raptorDataAvailabe) {
        if (this->verbosePrinting) {
          RCLCPP_WARN(get_logger(),
                      "Did not receive to_raptor message. This might lead to unexpected behavior of the RaceControl e.g. setting of flags and P2P is not available. Check that your stack is alive.");
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(),
                               *this->get_clock(),
                               5000,
                               "Did not receive to_raptor message. This might lead to unexpected behavior of the RaceControl e.g. setting of flags and P2P is not available. Check that your stack is alive.");
        }
        raptor_connection_announced = false;
      } else if (!raptor_connection_announced) {
        RCLCPP_INFO(get_logger(), "to_raptor message received.");
        raptor_connection_announced = true;
      }

      if (this->feedbackDataAvailabe == false && this->stackFeedbackConnectionWarningSent == false)
      {
        RCLCPP_WARN(get_logger(), "Did not receive vehicle_inputs message. The vehicle might move in an unexpected way. Check that your stack is alive.");
        this->stackFeedbackConnectionWarningSent = true;
      }
      else if (this->feedbackDataAvailabe == true && this->stackFeedbackConnectionWarningSent == true)
      {
        RCLCPP_INFO(get_logger(), "vehicle_inputs message received.");
        this->stackFeedbackConnectionWarningSent = false;
      }

      this->api.sendControlData(22222,std::addressof(this->feedbackCmd),sizeof(this->feedbackCmd));
    }
    if(this->simModeEnabled)
      this->api.increaseSimulationTime(0.001);
    else
      this->api.increaseSimulationTime(0.01);
  }

  void AsmSocketCanBridgeNode::switchRaceControlSourceCallback(const std_msgs::msg::Bool & msg)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "switchRaceControlSourceCallback");

    this->useCustomRaceControl = msg.data;
  }

  void AsmSocketCanBridgeNode::publish_base_to_car_summary()
  {
    publishCanMessage("base_to_car_summary", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("base_to_car_heartbeat", bus.asm_bus_var.race_control_var.base_to_car_heartbeat);
      assign("track_flag", bus.asm_bus_var.race_control_var.track_flag);
      assign("veh_flag", bus.asm_bus_var.race_control_var.veh_flag);
      assign("veh_rank", bus.asm_bus_var.race_control_var.veh_rank);
      assign("lap_count", bus.asm_bus_var.race_control_var.lap_count);
      assign("lap_distance", bus.asm_bus_var.race_control_var.lap_distance);
      assign("round_target_speed", bus.asm_bus_var.race_control_var.round_target_speed);
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
      assign("marelli_track_flag", bus.asm_bus_var.race_control_var.track_flag);
      assign("marelli_vehicle_flag", bus.asm_bus_var.race_control_var.veh_flag);
      assign("marelli_sector_flag", bus.asm_bus_var.race_control_var.track_flag);
      assign("marelli_rc_base_sync_check", static_cast<uint8_t>(1));
      assign("marelli_rc_lte_rssi", static_cast<uint8_t>(255));
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
      assign("marelli_gps_lat", bus.sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat);
      assign("marelli_gps_long", bus.sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon);
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
      assign("laps", bus.asm_bus_var.race_control_var.lap_count);
      assign("lap_time", bus.asm_bus_var.race_control_var.lap_time);
      assign("time_stamp", bus.asm_bus_var.race_control_var.time_stamp);
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
      const auto &powertrain = bus.sim_interface_var.vehicle_sensors_var.power_train_data_var;
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
      const auto &powertrain = bus.sim_interface_var.vehicle_sensors_var.power_train_data_var;
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
      const auto &powertrain = bus.sim_interface_var.vehicle_sensors_var.power_train_data_var;
      assign("engine_oil_temperature", powertrain.engine_oil_temperature);
      assign("torque_wheels", powertrain.torque_wheels_nm);
      assign("driver_traction_aim_swicth_fbk", 0);
      assign("driver_traction_range_switch_fbk", 0);
      const auto &race_control = bus.asm_bus_var.race_control_var;
      assign("push2pass_status", race_control.push2pass_status);
      assign("push2pass_budget_s", race_control.push2pass_budget_s);
      assign("push2pass_active_app_limit", race_control.push2pass_active_app_limit);
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report()
  {
    publishCanMessage("steering_report", [&](PreparedCanMessage &message, const ASMBus &) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("steering_motor_fdbk_counter", 0);
      assign("primary_steering_angular_rate", 0);
      assign("commanded_steering_rate", 0);
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report_extd()
  {
    publishCanMessage("steering_report_extd", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle);
        }
      };
      assign("average_steering_ang_fdbk");
      assign("primary_steering_angle_fbk");
      assign("secondary_steering_ang_fdbk");
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report_extd_2()
  {
    publishCanMessage("steering_report_extd_2", [&](PreparedCanMessage &message, const ASMBus &) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("motor_duty_cycle_cmd", 0);
      assign("motor_duty_cycle_fbk", 0);
      assign("motor_current_fbk", 0);
      assign("sbw_ecu_voltage", 0);
      assign("sbw_ecu_temp", 0);
      assign("sbw_error_code", 0);
      assign("sbw_motor_torque_estimate", 0);
    });
  }

  void AsmSocketCanBridgeNode::publish_steering_report_extd_3()
  {
    publishCanMessage("steering_report_extd_3", [&](PreparedCanMessage &message, const ASMBus &) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("steering_p_contribution", 0);
      assign("steering_i_contribution", 0);
      assign("steering_d_contribution", 0);
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
      assign("brk_pressure_fdbk_counter", 0);
      const auto &vehicle = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var;
      assign("brake_pressure_fdbk_rear", vehicle.rear_brake_pressure);
      assign("brake_pressure_fdbk_front", vehicle.front_brake_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_brake_report_extd()
  {
    publishCanMessage("brake_report_extd", [&](PreparedCanMessage &message, const ASMBus &) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("F_brk_pos_cmd", 0);
      assign("F_brk_pos_fbk", 0);
      assign("R_brk_pos_cmd", 0);
      assign("R_brk_pos_fbk", 0);
    });
  }

  void AsmSocketCanBridgeNode::publish_brake_report_extd_2()
  {
    publishCanMessage("brake_report_extd_2", [&](PreparedCanMessage &message, const ASMBus &) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("f_brake_act_force", 0);
      assign("r_brake_act_force", 0);
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
      assign("acc_pedal_fdbk_counter", 0);
      assign("acc_pedal_fdbk", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.accel_pedal_output);
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature;
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
      const auto temp = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature;
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
      const auto &vehicle = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var;
      assign("RR_Tire_Pressure_Gauge", vehicle.rr_tire_pressure_gauge);
      assign("RR_Tire_Pressure", vehicle.rr_tire_pressure);
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
      const auto &vehicle = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var;
      assign("RL_Tire_Pressure_Gauge", vehicle.rl_tire_pressure_gauge);
      assign("RL_Tire_Pressure", vehicle.rl_tire_pressure);
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
      const auto &vehicle = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var;
      assign("FR_Tire_Pressure_Gauge", vehicle.fr_tire_pressure_gauge);
      assign("FR_Tire_Pressure", vehicle.fr_tire_pressure);
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
      const auto &vehicle = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var;
      assign("FL_Tire_Pressure_Gauge", vehicle.fl_tire_pressure_gauge);
      assign("FL_Tire_Pressure", vehicle.fl_tire_pressure);
    });
  }

  void AsmSocketCanBridgeNode::publish_wheel_strain_gauge()
  {
    publishCanMessage("wheel_strain_gauge", [&](PreparedCanMessage &message, const ASMBus &bus) {
      for (const auto &[name, value] : std::initializer_list<std::pair<std::string_view, double>>{
             {"wheel_strain_gauge_RR", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_wheel_load},
             {"wheel_strain_gauge_RL", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_wheel_load},
             {"wheel_strain_gauge_FR", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_wheel_load},
             {"wheel_strain_gauge_FL", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_wheel_load}}) {
        if (const auto *signal = findSignal(message.metadata->id, name)) {
          insertBits(message.frame.data, *signal, value);
        }
      }
    });
  }

  void AsmSocketCanBridgeNode::publish_wheel_potentiometer_data()
  {
    publishCanMessage("wheel_potentiometer_data", [&](PreparedCanMessage &message, const ASMBus &bus) {
      for (const auto &[name, value] : std::initializer_list<std::pair<std::string_view, double>>{
             {"wheel_potentiometer_RR", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_damper_linear_potentiometer},
             {"wheel_potentiometer_RL", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_damper_linear_potentiometer},
             {"wheel_potentiometer_FR", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_damper_linear_potentiometer},
             {"wheel_potentiometer_FL", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_damper_linear_potentiometer}}) {
        if (const auto *signal = findSignal(message.metadata->id, name)) {
          insertBits(message.frame.data, *signal, value);
        }
      }
    });
  }

  void AsmSocketCanBridgeNode::publish_wheel_speed_report()
  {
    publishCanMessage("wheel_speed_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      for (const auto &[name, value] : std::initializer_list<std::pair<std::string_view, double>>{
             {"wheel_speed_RR", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_right},
             {"wheel_speed_RL", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_left},
             {"wheel_speed_FR", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_right},
             {"wheel_speed_FL", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_left}}) {
        if (const auto *signal = findSignal(message.metadata->id, name)) {
          insertBits(message.frame.data, *signal, value);
        }
      }
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
      const auto &vehicle = bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var;
      assign("battery_voltage", vehicle.battery_voltage);
      assign("safety_switch_state", vehicle.safety_switch_state);
      assign("mode_switch_state", vehicle.mode_switch_state);
      assign("sys_state", vehicle.sys_state);
      assign("raptor_rolling_counter", 0);
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
      assign("sd_system_warning", 0);
      assign("sd_system_failure", 0);
      assign("sd_brake_warning1", 0);
      assign("sd_brake_warning2", 0);
      assign("sd_brake_warning3", 0);
      assign("sd_steer_warning1", 0);
      assign("sd_steer_warning2", 0);
      assign("sd_steer_warning3", 0);
      assign("motec_warning", 0);
      assign("est1_oos_front_brk", 0);
      assign("est2_oos_rear_brk", 0);
      assign("est3_low_eng_speed", 0);
      assign("est4_sd_comms_loss", 0);
      assign("est5_motec_comms_loss", 0);
      assign("est6_sd_ebrake", 0);
      assign("adlink_hb_lost", 0);
      assign("rc_lost", 0);
    });
  }

  void AsmSocketCanBridgeNode::publish_VECTOR__INDEPENDENT_SIG_MSG()
  {
    publishCanMessage("VECTOR__INDEPENDENT_SIG_MSG", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      const auto &vector_nav = bus.sim_interface_var.vector_nav_vn1_var.common_group_var;
      assign("ang_heading", vector_nav.insstatus_var.gps_heading_ins);
      assign("pos_y", vector_nav.position_var.y);
      assign("pos_x", vector_nav.position_var.x);
      assign("yaw_rate", vector_nav.angularrate_var.z);
      assign("velocity_long", vector_nav.velocity_var.x);
      assign("velocity_lat", vector_nav.velocity_var.y);
      assign("motor_angle", bus.sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle);
      assign("acceleration", vector_nav.accel_var.x);
      assign("rc_base_sync_check", static_cast<uint8_t>(1));
      assign("rc_lte_rssi", 0);
      assign("duty_cycle_fbk", 0);
      assign("duty_cycle_dmd", 0);
      assign("steering_motor_ang_avg_fdbk", 0);
    });
  }

  void AsmSocketCanBridgeNode::publish_novatel_bestpos(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].best_pos;
    if (!publisher) {
      return;
    }
    novatel_oem7_msgs::msg::BESTPOS message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateBestPosMessage(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_bestgnsspos(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].best_gnss_pos;
    if (!publisher) {
      return;
    }
    novatel_oem7_msgs::msg::BESTPOS message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateBestPosMessage(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_bestvel(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].best_vel;
    if (!publisher) {
      return;
    }
    novatel_oem7_msgs::msg::BESTVEL message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateBestVelMessage(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_bestgnssvel(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].best_gnss_vel;
    if (!publisher) {
      return;
    }
    novatel_oem7_msgs::msg::BESTVEL message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateBestVelMessage(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_inspva(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].inspva;
    if (!publisher) {
      return;
    }
    novatel_oem7_msgs::msg::INSPVA message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateInspvaMessage(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_heading2(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].heading2;
    if (!publisher) {
      return;
    }
    novatel_oem7_msgs::msg::HEADING2 message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateHeading2Message(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_rawimu(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].raw_imu;
    if (!publisher) {
      return;
    }
    novatel_oem7_msgs::msg::RAWIMU message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateRawImuMessage(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_rawimux(uint8_t novatel_id)
  {
    const std::size_t index =
      novatel_id == 1 ? kNovatelTopIndex
                      : novatel_id == 2 ? kNovatelBottomIndex : novatel_publishers_.size();
    if (index >= novatel_publishers_.size()) {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
      return;
    }
    auto publisher = novatel_publishers_[index].raw_imu_x;
    if (!publisher) {
      return;
    }
    sensor_msgs::msg::Imu message;
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto *data = index == kNovatelTopIndex ? &bus.sim_interface_var.nova_tel_pwr_pak1_var
                                                   : &bus.sim_interface_var.nova_tel_pwr_pak2_var;
      populateRawImuXMessage(message, *data);
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    publisher->publish(message);
  }

  void AsmSocketCanBridgeNode::publish_novatel_report()
  {
    publishCanMessage("novatel_report", [&](PreparedCanMessage &message, const ASMBus &bus) {
      auto assign = [&](std::string_view signal_name, auto value) {
        if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
          insertBits(message.frame.data, *signal, value);
        }
      };
      assign("novatel_lat", bus.sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat);
      assign("novatel_long", bus.sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon);
    });
  }

  void AsmSocketCanBridgeNode::publish_vectornav_attitude_group()
  {
    vectornav_msgs::msg::AttitudeGroup attitudeGroup;
    setHeader(attitudeGroup.header, "world");
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto &source = bus.sim_interface_var.vector_nav_vn1_var.attitude_group_var;
      attitudeGroup.vpestatus.attitude_quality = source.vpestatus_var.attitude_quality;
      attitudeGroup.vpestatus.gyro_saturation = source.vpestatus_var.gyro_saturation;
      attitudeGroup.vpestatus.gyro_saturation_recovery = source.vpestatus_var.gyro_saturation_recovery;
      attitudeGroup.vpestatus.mag_disturbance = source.vpestatus_var.mag_disturbance;
      attitudeGroup.vpestatus.mag_saturation = source.vpestatus_var.mag_saturation;
      attitudeGroup.vpestatus.acc_disturbance = source.vpestatus_var.acc_disturbance;
      attitudeGroup.vpestatus.acc_saturation = source.vpestatus_var.acc_saturation;
      attitudeGroup.vpestatus.known_mag_disturbance = source.vpestatus_var.known_mag_disturbance;
      attitudeGroup.vpestatus.known_accel_disturbance = source.vpestatus_var.known_accel_disturbance;
      attitudeGroup.yawpitchroll.x = source.yawpitchroll_var.x;
      attitudeGroup.yawpitchroll.y = source.yawpitchroll_var.y;
      attitudeGroup.yawpitchroll.z = source.yawpitchroll_var.z;
      attitudeGroup.quaternion.w = source.quaternion_var.w;
      attitudeGroup.quaternion.x = source.quaternion_var.x;
      attitudeGroup.quaternion.y = source.quaternion_var.y;
      attitudeGroup.quaternion.z = source.quaternion_var.z;
      for (size_t i = 0; i < attitudeGroup.dcm.size(); ++i) {
        attitudeGroup.dcm[i] = source.dcm[i];
      }
      attitudeGroup.magned.x = source.magned_var.x;
      attitudeGroup.magned.y = source.magned_var.y;
      attitudeGroup.magned.z = source.magned_var.z;
      attitudeGroup.accelned.x = source.accelned_var.x;
      attitudeGroup.accelned.y = source.accelned_var.y;
      attitudeGroup.accelned.z = source.accelned_var.z;
      attitudeGroup.linearaccelbody.x = source.linearaccelbody_var.x;
      attitudeGroup.linearaccelbody.y = source.linearaccelbody_var.y;
      attitudeGroup.linearaccelbody.z = source.linearaccelbody_var.z;
      attitudeGroup.linearaccelned.x = source.linearaccelned_var.x;
      attitudeGroup.linearaccelned.y = source.linearaccelned_var.y;
      attitudeGroup.linearaccelned.z = source.linearaccelned_var.z;
      attitudeGroup.ypru.x = source.ypru_var.x;
      attitudeGroup.ypru.y = source.ypru_var.y;
      attitudeGroup.ypru.z = source.ypru_var.z;
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    auto publisher = vectornav_publishers_.attitude;
    if (!publisher) {
      return;
    }
    publisher->publish(attitudeGroup);
  }

  void AsmSocketCanBridgeNode::publish_vectornav_common_group()
  {
    vectornav_msgs::msg::CommonGroup commonGroup;
    setHeader(commonGroup.header, "world");
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto &source = bus.sim_interface_var.vector_nav_vn1_var.common_group_var;
      commonGroup.timestartup = source.timestartup;
      commonGroup.timegps = source.timegps;
      commonGroup.timesyncin = source.timesyncin;
      commonGroup.yawpitchroll.x = source.yawpitchroll_var.x;
      commonGroup.yawpitchroll.y = source.yawpitchroll_var.y;
      commonGroup.yawpitchroll.z = source.yawpitchroll_var.z;
      commonGroup.quaternion.w = source.quaternion_var.w;
      commonGroup.quaternion.x = source.quaternion_var.x;
      commonGroup.quaternion.y = source.quaternion_var.y;
      commonGroup.quaternion.z = source.quaternion_var.z;
      commonGroup.angularrate.x = source.angularrate_var.x;
      commonGroup.angularrate.y = source.angularrate_var.y;
      commonGroup.angularrate.z = source.angularrate_var.z;
      commonGroup.position.x = source.position_var.x;
      commonGroup.position.y = source.position_var.y;
      commonGroup.position.z = source.position_var.z;
      commonGroup.velocity.x = source.velocity_var.x;
      commonGroup.velocity.y = source.velocity_var.y;
      commonGroup.velocity.z = source.velocity_var.z;
      commonGroup.accel.x = source.accel_var.x;
      commonGroup.accel.y = source.accel_var.y;
      commonGroup.accel.z = source.accel_var.z;
      commonGroup.imu_accel.x = source.imu_accel_var.x;
      commonGroup.imu_accel.y = source.imu_accel_var.y;
      commonGroup.imu_accel.z = source.imu_accel_var.z;
      commonGroup.imu_rate.x = source.imu_rate_var.x;
      commonGroup.imu_rate.y = source.imu_rate_var.y;
      commonGroup.imu_rate.z = source.imu_rate_var.z;
      commonGroup.magpres_mag.x = source.magpres_mag_var.x;
      commonGroup.magpres_mag.y = source.magpres_mag_var.y;
      commonGroup.magpres_mag.z = source.magpres_mag_var.z;
      commonGroup.magpres_temp = source.magpres_temp;
      commonGroup.magpres_pres = source.magpres_pres;
      commonGroup.deltatheta_dtime = source.deltatheta_dtime;
      commonGroup.deltatheta_dtheta.x = source.deltatheta_dtheta_var.x;
      commonGroup.deltatheta_dtheta.y = source.deltatheta_dtheta_var.y;
      commonGroup.deltatheta_dtheta.z = source.deltatheta_dtheta_var.z;
      commonGroup.deltatheta_dvel.x = source.deltatheta_dvel_var.x;
      commonGroup.deltatheta_dvel.y = source.deltatheta_dvel_var.y;
      commonGroup.deltatheta_dvel.z = source.deltatheta_dvel_var.z;
      commonGroup.insstatus.gps_fix = source.insstatus_var.gps_fix;
      commonGroup.insstatus.time_error = source.insstatus_var.time_error;
      commonGroup.insstatus.imu_error = source.insstatus_var.imu_error;
      commonGroup.insstatus.mag_pres_error = source.insstatus_var.mag_pres_error;
      commonGroup.insstatus.gps_error = source.insstatus_var.gps_error;
      commonGroup.insstatus.gps_heading_ins = source.insstatus_var.gps_heading_ins;
      commonGroup.insstatus.gps_compass = source.insstatus_var.gps_compass;
      commonGroup.syncincnt = source.syncincnt;
      commonGroup.timegpspps = source.timegpspps;
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    auto publisher = vectornav_publishers_.common;
    if (!publisher) {
      return;
    }
    publisher->publish(commonGroup);
  }

  void AsmSocketCanBridgeNode::publish_vectornav_imu_group()
  {
    vectornav_msgs::msg::ImuGroup imuGroup;
    setHeader(imuGroup.header, "world");
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto &source = bus.sim_interface_var.vector_nav_vn1_var.imu_group_var;
      imuGroup.imustatus = source.imustatus;
      imuGroup.uncompmag.x = source.uncompmag_var.x;
      imuGroup.uncompmag.y = source.uncompmag_var.y;
      imuGroup.uncompmag.z = source.uncompmag_var.z;
      imuGroup.uncompaccel.x = source.uncompaccel_var.x;
      imuGroup.uncompaccel.y = source.uncompaccel_var.y;
      imuGroup.uncompaccel.z = source.uncompaccel_var.z;
      imuGroup.uncompgyro.x = source.uncompgyro_var.x;
      imuGroup.uncompgyro.y = source.uncompgyro_var.y;
      imuGroup.uncompgyro.z = source.uncompgyro_var.z;
      imuGroup.temp = source.temp;
      imuGroup.pres = source.pres;
      imuGroup.deltatheta_time = source.deltatheta_time;
      imuGroup.deltatheta_dtheta.x = source.deltatheta_dtheta_var.x;
      imuGroup.deltatheta_dtheta.y = source.deltatheta_dtheta_var.y;
      imuGroup.deltatheta_dtheta.z = source.deltatheta_dtheta_var.z;
      imuGroup.deltavel.x = source.deltavel_var.x;
      imuGroup.deltavel.y = source.deltavel_var.y;
      imuGroup.deltavel.z = source.deltavel_var.z;
      imuGroup.mag.x = source.mag_var.x;
      imuGroup.mag.y = source.mag_var.y;
      imuGroup.mag.z = source.mag_var.z;
      imuGroup.accel.x = source.accel_var.x;
      imuGroup.accel.y = source.accel_var.y;
      imuGroup.accel.z = source.accel_var.z;
      imuGroup.angularrate.x = source.angularrate_var.x;
      imuGroup.angularrate.y = source.angularrate_var.y;
      imuGroup.angularrate.z = source.angularrate_var.z;
      imuGroup.sensat = source.sensat;
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    auto publisher = vectornav_publishers_.imu;
    if (!publisher) {
      return;
    }
    publisher->publish(imuGroup);
  }

  void AsmSocketCanBridgeNode::publish_vectornav_gps_group_left()
  {
    vectornav_msgs::msg::GpsGroup gpsGroup;
    if (!withCanBusShared([&](const ASMBus &bus) {
      populateGpsGroupMessage(gpsGroup, bus.sim_interface_var.vector_nav_vn1_var.gps_group1_var);
    })) {
      return;
    }
    auto publisher = vectornav_publishers_.gps[kVectorNavGpsLeftIndex];
    if (!publisher) {
      return;
    }
    publisher->publish(gpsGroup);
  }

  void AsmSocketCanBridgeNode::publish_vectornav_gps_group_right()
  {
    vectornav_msgs::msg::GpsGroup gpsGroup;
    if (!withCanBusShared([&](const ASMBus &bus) {
      populateGpsGroupMessage(gpsGroup, bus.sim_interface_var.vector_nav_vn1_var.gps_group2_var);
    })) {
      return;
    }
    auto publisher = vectornav_publishers_.gps[kVectorNavGpsRightIndex];
    if (!publisher) {
      return;
    }
    publisher->publish(gpsGroup);
  }

  void AsmSocketCanBridgeNode::publish_vectornav_ins_group()
  {
    vectornav_msgs::msg::InsGroup insGroup;
    setHeader(insGroup.header, "world");
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto &source = bus.sim_interface_var.vector_nav_vn1_var.ins_group_var;
      insGroup.insstatus.gps_fix = source.insstatus_var.gps_fix;
      insGroup.insstatus.time_error = source.insstatus_var.time_error;
      insGroup.insstatus.imu_error = source.insstatus_var.imu_error;
      insGroup.insstatus.mag_pres_error = source.insstatus_var.mag_pres_error;
      insGroup.insstatus.gps_error = source.insstatus_var.gps_error;
      insGroup.insstatus.gps_heading_ins = source.insstatus_var.gps_heading_ins;
      insGroup.insstatus.gps_compass = source.insstatus_var.gps_compass;
      insGroup.poslla.x = source.poslla_var.x;
      insGroup.poslla.y = source.poslla_var.y;
      insGroup.poslla.z = source.poslla_var.z;
      insGroup.posecef.x = source.posecef_var.x;
      insGroup.posecef.y = source.posecef_var.y;
      insGroup.posecef.z = source.posecef_var.z;
      insGroup.velbody.x = source.velbody_var.x;
      insGroup.velbody.y = source.velbody_var.y;
      insGroup.velbody.z = source.velbody_var.z;
      insGroup.velned.x = source.velned_var.x;
      insGroup.velned.y = source.velned_var.y;
      insGroup.velned.z = source.velned_var.z;
      insGroup.velecef.x = source.velecef_var.x;
      insGroup.velecef.y = source.velecef_var.y;
      insGroup.velecef.z = source.velecef_var.z;
      insGroup.magecef.x = source.magecef_var.x;
      insGroup.magecef.y = source.magecef_var.y;
      insGroup.magecef.z = source.magecef_var.z;
      insGroup.accelecef.x = source.accelecef_var.x;
      insGroup.accelecef.y = source.accelecef_var.y;
      insGroup.accelecef.z = source.accelecef_var.z;
      insGroup.linearaccelecef.x = source.linearaccelecef_var.x;
      insGroup.linearaccelecef.y = source.linearaccelecef_var.y;
      insGroup.linearaccelecef.z = source.linearaccelecef_var.z;
      insGroup.posu = source.posu_var;
      insGroup.velu = source.velu;
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    auto publisher = vectornav_publishers_.ins;
    if (!publisher) {
      return;
    }
    publisher->publish(insGroup);
  }

  void AsmSocketCanBridgeNode::publish_vectornav_time_group()
  {
    vectornav_msgs::msg::TimeGroup timeGroup;
    setHeader(timeGroup.header, "");
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto &source = bus.sim_interface_var.vector_nav_vn1_var.time_group_var;
      timeGroup.timestartup = source.timestartup;
      timeGroup.timegps = source.timegps;
      timeGroup.gpstow = source.gpstow;
      timeGroup.gpsweek = source.gpsweek;
      timeGroup.timesyncin = source.timesyncin;
      timeGroup.timegpspps = source.timegpspps;
      timeGroup.timeutc.year = source.timeutc_var.year;
      timeGroup.timeutc.month = source.timeutc_var.month;
      timeGroup.timeutc.day = source.timeutc_var.day;
      timeGroup.timeutc.hour = source.timeutc_var.hour;
      timeGroup.timeutc.min = source.timeutc_var.min;
      timeGroup.timeutc.sec = source.timeutc_var.sec;
      timeGroup.timeutc.ms = source.timeutc_var.ms;
      timeGroup.syncincnt = source.syncincnt;
      timeGroup.syncoutcnt = source.syncoutcnt;
      timeGroup.timestatus.time_ok = source.timestatus_var.time_ok;
      timeGroup.timestatus.date_ok = source.timestatus_var.date_ok;
      timeGroup.timestatus.utctime_ok = source.timestatus_var.utctime_ok;
      populated = true;
    })) {
      return;
    }
    if (!populated) {
      return;
    }
    auto publisher = vectornav_publishers_.time;
    if (!publisher) {
      return;
    }
    publisher->publish(timeGroup);
  }

  void AsmSocketCanBridgeNode::publishGroundTruthArray()
  {
    if (this->verbosePrinting) {
      RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray");
    }

    auto groundTruthArray = autonoma_msgs::msg::GroundTruthArray();

    groundTruthArray.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      groundTruthArray.header.stamp.sec = this->sec;
      groundTruthArray.header.stamp.nanosec = this->nsec;
    }
    else
    {
      groundTruthArray.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      groundTruthArray.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (groundTruthArray.header.stamp.sec*1000000000);
    }
    if (this->verbosePrinting) {
      RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 1");
    }

    bool groundTruthArrayFilled = false;

    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto fellow_count_raw = bus.sim_interface_var.vehicle_sensors_var.fellow_count;
      const auto fellow_count = static_cast<size_t>(fellow_count_raw);
      if (fellow_count == 0) {
        groundTruthArray.vehicles.resize(1);
        return;
      }
      groundTruthArray.vehicles.resize(fellow_count);
      for (size_t vehicleID = 0; vehicleID < fellow_count; vehicleID++) {
        if (this->verbosePrinting) {
          RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 2");
        }
        auto &vehicle = groundTruthArray.vehicles[vehicleID];
        vehicle.header.frame_id = groundTruthArray.header.frame_id;
        vehicle.header.stamp.sec = groundTruthArray.header.stamp.sec;
        vehicle.header.stamp.nanosec = groundTruthArray.header.stamp.nanosec;
        if (this->verbosePrinting) {
          RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 3");
        }

        vehicle.car_num = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.car_num[vehicleID];
        vehicle.lat = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[vehicleID];
        vehicle.lon = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[vehicleID];
        vehicle.hgt = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[vehicleID];
        vehicle.vx = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.vx[vehicleID];
        vehicle.vy = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.vy[vehicleID];
        vehicle.vz = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.vz[vehicleID];
        if (this->verbosePrinting) {
          RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 4");
        }
        vehicle.yaw = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.yaw[vehicleID];
        vehicle.pitch = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.pitch[vehicleID];
        vehicle.roll = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.roll[vehicleID];
        if (this->verbosePrinting) {
          RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 5");
        }
        vehicle.del_x = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.del_x[vehicleID];
        vehicle.del_y = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.del_y[vehicleID];
        vehicle.del_z = bus.sim_interface_var.vehicle_sensors_var.ground_truth_var.del_z[vehicleID];
        groundTruthArrayFilled = true;
      }
    })) {
      return;
    }
    if (this->verbosePrinting) {
      RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 6");
    }
    
    if (!groundTruthArrayFilled)
    {
      groundTruthArray.vehicles[0].header.frame_id = groundTruthArray.header.frame_id;
      groundTruthArray.vehicles[0].header.stamp.sec = groundTruthArray.header.stamp.sec;
      groundTruthArray.vehicles[0].header.stamp.nanosec = groundTruthArray.header.stamp.nanosec;
      groundTruthArray.vehicles[0].car_num = 255;
      groundTruthArray.vehicles[0].lat = 0;
      groundTruthArray.vehicles[0].lon = 0;
      groundTruthArray.vehicles[0].hgt = 0;
      groundTruthArray.vehicles[0].vx = 0;
      groundTruthArray.vehicles[0].vy = 0;
      groundTruthArray.vehicles[0].vz = 0;
      groundTruthArray.vehicles[0].yaw = 0;
      groundTruthArray.vehicles[0].pitch = 0;
      groundTruthArray.vehicles[0].roll = 0;
      groundTruthArray.vehicles[0].del_x = 0;
      groundTruthArray.vehicles[0].del_y = 0;
      groundTruthArray.vehicles[0].del_z = 0;
    }
    
    if (this->verbosePrinting) {
      RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 7");
    }
    this->groundTruthArrayPublisher_->publish(groundTruthArray);
    if (this->verbosePrinting) {
      RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 8");
    }
  }

} // namespace asm_socketcan_bridge


int main(int argc, char * argv[])
{
  rclcpp::Node::SharedPtr AsmSocketCanBridgeNodePtr;
  try
  {
    rclcpp::init(argc, argv);

    AsmSocketCanBridgeNodePtr = std::make_shared<asm_socketcan_bridge::AsmSocketCanBridgeNode>();
    const auto hardware_threads = std::thread::hardware_concurrency();
    size_t executor_threads = 4;
    if (hardware_threads == 0) {
      executor_threads = 4;
    } else {
      size_t half = hardware_threads / 2;
      if (half == 0) {
        executor_threads = 1;
      } else {
        executor_threads = std::min<size_t>(8, half);
        if (half >= 4 && executor_threads < 4) {
          executor_threads = 4;
        }
      }
    }
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), executor_threads);
    executor.add_node(AsmSocketCanBridgeNodePtr);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception& e)
  {
    if (AsmSocketCanBridgeNodePtr) {
      RCLCPP_ERROR(AsmSocketCanBridgeNodePtr->get_logger(), "Failed to initialize ASM-SocketCAN-Bridge node: %s", e.what());
    } else {
      std::cerr << "Failed to initialize ASM-SocketCAN-Bridge node: " << e.what() << std::endl;
    }
    return 1;
  }
}
