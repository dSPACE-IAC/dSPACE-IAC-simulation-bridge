#include "asm_socketcan_bridge.h"

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <list>
#include <thread>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/create_timer.hpp>

#include "iac_sim_time/sim_clock_mode.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {

  int sanitize_port_value(int64_t value, int default_value, const rclcpp::Logger &logger, const std::string &description)
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

  uint32_t sanitize_interval_value(int64_t value, uint32_t default_value, const rclcpp::Logger &logger, const std::string &description)
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

  int16_t sanitize_retry_value(int64_t value, int16_t default_value, const rclcpp::Logger &logger, const std::string &description)
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

  void AsmSocketCanBridgeNode::configureConnectionParameters()
  {
    if (std::getenv("VESI_IP")){
      this->api.setSimManagerHost(std::getenv("VESI_IP"));
      RCLCPP_INFO(this->get_logger(), "SimManager Host IP override from environment variable: %s", std::getenv("VESI_IP"));
    } else {
      const auto sim_manager_host = this->declare_parameter<std::string>("sim_manager.host", "127.0.0.1");
      RCLCPP_INFO(this->get_logger(), "SimManager Host IP: %s", sim_manager_host.c_str());
      this->api.setSimManagerHost(sim_manager_host);
    }
    if (std::getenv("ASM_IP")){
      this->api.setASMHost(std::getenv("ASM_IP"));
      RCLCPP_INFO(this->get_logger(), "ASM Host IP override from environment variable: %s", std::getenv("ASM_IP"));
    } else {
      const auto asm_host = this->declare_parameter<std::string>("asm.host", "127.0.0.1");
      RCLCPP_INFO(this->get_logger(), "ASM Host IP: %s", asm_host.c_str());
      this->api.setASMHost(asm_host);
    }

    const int64_t sim_manager_port_param = this->declare_parameter<int64_t>("sim_manager.port", 12345);
    const int sim_manager_port = sanitize_port_value(sim_manager_port_param, 12345, this->get_logger(), "SimManager port");
    RCLCPP_INFO(this->get_logger(), "SimManager Port: %d", sim_manager_port);
    this->api.setSimManagerPort(sim_manager_port);

    this->warning_throttle_intervall = this->declare_parameter<int64_t>("warn.throttle_interval", 478000);
  }

  void AsmSocketCanBridgeNode::configurePublisherTimers()
  {
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
    register_timer("publish_pt_report_4_ms",
                   [this]() { this->publish_pt_report_4(); });
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
  }

  void AsmSocketCanBridgeNode::configureRuntimeParameters()
  {
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

    if (const char *sim_clock_mode = std::getenv("SIM_CLOCK_MODE")) {
      const auto environment_setting = iac_sim_time::parse_sim_clock_mode(sim_clock_mode);
      if (environment_setting.has_value()) {
        use_sim_time = environment_setting.value();
        this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
        RCLCPP_INFO(this->get_logger(),
                    "SIM_CLOCK_MODE environment override: %s",
                    use_sim_time ? "true" : "false");
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Ignoring invalid SIM_CLOCK_MODE value '%s'; using use_sim_time parameter",
                    sim_clock_mode);
      }
    }

    this->simModeEnabled = use_sim_time;
    RCLCPP_INFO(this->get_logger(),
                "Simulation clock mode %s",
                this->simModeEnabled ? "enabled" : "disabled");
  }

  bool AsmSocketCanBridgeNode::connectToSimulation()
  {
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
        return true;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "Failed to configure V-ESI: %s", e.what());

        if (attempt == vesi_attempts)
        {
          RCLCPP_FATAL(get_logger(), "Failed to often to initialize V-ESI connection; exiting.");
          rclcpp::shutdown();
          return false;
        }

        RCLCPP_WARN(get_logger(),
                    "Failed to configure V-ESI (%s), attempt %d/%d; retrying in 1 s",
                    e.what(), attempt, vesi_attempts);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
    return false;
  }

  bool AsmSocketCanBridgeNode::initializeCanInterface()
  {
    const auto pkg_share = ament_index_cpp::get_package_share_directory("asm_socketcan_bridge");

    can1_dbc_path = this->declare_parameter<std::string>(
      "can.can1_dbc_path",
      pkg_share + "/config/CAN1-INDY-V26.dbc");
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
        return false;
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
      return false;
    }
    can_message_info = initialize_messages();
    buildMessageLookup();
    RCLCPP_INFO(get_logger(), "can message structure: %u", can_message_info[0].id);

    reader_thread1 = std::thread([this]() {
      can_reader_loop(this->can_socket, "CAN1");
    });

    RCLCPP_INFO(get_logger(), "CAN initialization done");
    return true;
  }

  void AsmSocketCanBridgeNode::initializeRosInterfaces()
  {
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

      if(this->simModeEnabled)
      {
        RCLCPP_INFO(get_logger(), "Use Simulated Clock.");
        this->simClockTimePublisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", sim_qos);
        this->simTimeIncrease_ = this->create_subscription<std_msgs::msg::UInt16>("sim_time_increase", sim_qos, std::bind(&AsmSocketCanBridgeNode::simTimeIncreaseCallback, this, _1));
        vesiCallback();
        this->simClockTime.clock = rclcpp::Time(
          this->simTime_.seconds(), this->simTime_.nanoseconds());
        sim_clock_publications_.fetch_add(1);
        this->simClockTimePublisher_->publish(this->simClockTime);
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Use Wall Clock (system clock).");
        // Wall-clock mode: free-running acquisition timer drives the ASM exchange as fast
        // as the period allows (unchanged behavior).
        this->vesiAcquisitionTimer_ = this->create_wall_timer(
          10ms,
          std::bind(&AsmSocketCanBridgeNode::vesiCallback, this));
        vesiCallback();
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create object for ASM-ROS2-Bridge node: %s", e.what());
    }
  }

  void AsmSocketCanBridgeNode::initializeTimeRecording()
  {
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
  }

} // namespace asm_socketcan_bridge
