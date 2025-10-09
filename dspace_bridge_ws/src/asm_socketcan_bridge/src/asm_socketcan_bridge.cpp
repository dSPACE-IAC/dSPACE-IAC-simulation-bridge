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

}  // namespace

namespace asm_socketcan_bridge {

  AsmSocketCanBridgeNode::AsmSocketCanBridgeNode() : Node("asm_socketcan_bridge_node")
  {
    const auto logger = get_logger();

    this->canBus = nullptr;

    const auto sim_manager_host =
      this->declare_parameter<std::string>("sim_manager.host", "127.0.0.1");
    RCLCPP_INFO(logger, "SimManager Host IP: %s", sim_manager_host.c_str());
    this->api.setSimManagerHost(sim_manager_host);

    const auto asm_host =
      this->declare_parameter<std::string>("asm.host", "127.0.0.1");
    RCLCPP_INFO(logger, "ASM Host IP: %s", asm_host.c_str());
    this->api.setASMHost(asm_host);

    const int64_t sim_manager_port_param =
      this->declare_parameter<int64_t>("sim_manager.port", 12345);
    const int sim_manager_port =
      sanitize_port_value(sim_manager_port_param, 12345, logger, "SimManager port");
    RCLCPP_INFO(logger, "SimManager Port: %d", sim_manager_port);
    this->api.setSimManagerPort(sim_manager_port);

    auto declare_interval = [this, logger](const std::string &name,
                                           const std::string &description,
                                           uint32_t default_value) {
      const int64_t raw_value = this->declare_parameter<int64_t>(
        name,
        static_cast<int64_t>(default_value));
      const auto sanitized =
        sanitize_interval_value(raw_value, default_value, logger, description);
      RCLCPP_INFO(logger, "%s: %u ms", description.c_str(), sanitized);
      return sanitized;
    };

    RCLCPP_INFO(logger, "Configuring publish intervals (milliseconds)");
    this->pubIntervalRaceControlData = declare_interval(
      "publish_intervals.race_control_ms",
      "Race control publish interval",
      10U);
    this->pubIntervalVehicleData = declare_interval(
      "publish_intervals.vehicle_ms",
      "Vehicle data publish interval",
      10U);
    this->pubIntervalPowertrainData = declare_interval(
      "publish_intervals.powertrain_ms",
      "Powertrain data publish interval",
      10U);
    this->pubIntervalGroundTruthArray = declare_interval(
      "publish_intervals.ground_truth_ms",
      "Ground truth publish interval",
      10U);
    this->pubIntervalVectorNavData = declare_interval(
      "publish_intervals.vectornav_ms",
      "VectorNav data publish interval",
      10U);
    this->pubIntervalNovatelData = declare_interval(
      "publish_intervals.novatel_ms",
      "NovAtel data publish interval",
      10U);
    this->pubIntervalFoxgloveMap = declare_interval(
      "publish_intervals.foxglove_map_ms",
      "Foxglove map publish interval",
      10U);

    this->pathTimeRecord = this->declare_parameter<std::string>(
      "logging.path",
      "/root/record_log");
    RCLCPP_INFO(logger, "Execution time log path: %s", this->pathTimeRecord.c_str());

    this->enableTimeRecord = this->declare_parameter<bool>(
      "logging.cycle_time",
      false);
    RCLCPP_INFO(logger,
                "Execution cycle time logging %s",
                this->enableTimeRecord ? "enabled" : "disabled");

    const int16_t default_max_retries = 1;
    const int64_t raw_max_retries = this->declare_parameter<int64_t>(
      "connection.max_retries",
      static_cast<int64_t>(default_max_retries));
    max_retries = sanitize_retry_value(raw_max_retries,
                                       default_max_retries,
                                       logger,
                                       "Maximum connection retries");
    RCLCPP_INFO(logger,
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
      RCLCPP_INFO(logger, "Verbose printing enabled");
    }
    if (this->receivedMessagePrinting) {
      RCLCPP_INFO(logger, "Raw CAN frame logging enabled");
    }
    if (this->receivedDecodedMessagePrinting) {
      RCLCPP_INFO(logger, "Decoded CAN frame logging enabled");
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(logger, "Sent CAN frame logging enabled");
    }

    bool use_sim_time = false;
    if (!this->get_parameter("use_sim_time", use_sim_time)) {
      use_sim_time = false;
    }

    this->simModeEnabled = use_sim_time;
    RCLCPP_INFO(logger,
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
      RCLCPP_INFO(logger,
                  "CAN1 DBC path resolved relative to package share: %s",
                  can1_dbc_path.c_str());
    } else {
      RCLCPP_INFO(logger, "CAN1 DBC path: %s", can1_dbc_path.c_str());
    }

    // determine socketcan interface names
    can_iface = this->declare_parameter<std::string>(
      "can.interface",
      "vcan0");
    RCLCPP_INFO(logger, "CAN interface: %s", can_iface.c_str());

    // load & retry for CAN1 DBC
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

    // open both CAN sockets
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
    // launch reader thread for CAN1
    reader_thread1 = std::thread([this]() {
      can_reader_loop(this->can_socket, "CAN1");
    });

    RCLCPP_INFO(get_logger(), "CAN initialization done");
    
    try
    {
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_iac);
      const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sim_clock);

      this->groundTruthArrayPublisher_ = this->create_publisher<autonoma_msgs::msg::GroundTruthArray>("ground_truth_array", qos);

      this->verctorNavCommonGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::CommonGroup>("vectornav/raw/common", qos);
      this->verctorNavAttitudeGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::AttitudeGroup>("vectornav/raw/attitude", qos);
      this->verctorNavImuGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", qos);
      this->verctorNavInsGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::InsGroup>("vectornav/raw/ins", qos);
      this->verctorNavGpsGroupLeftPublisher_ = this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps_left", qos);
      this->verctorNavGpsGroupRightPublisher_ = this->create_publisher<vectornav_msgs::msg::GpsGroup>("vectornav/raw/gps_right", qos);
      this->verctorNavTimeGroupPublisher_ = this->create_publisher<vectornav_msgs::msg::TimeGroup>("vectornav/raw/time", qos);

      this->novaTelBestPosPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_top/bestpos", qos);
      this->novaTelBestGNSSPosPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_top/bestgnsspos", qos);
      this->novaTelBestVelPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_top/bestvel", qos);
      this->novaTelBestGNSSVelPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_top/bestgnssvel", qos);
      this->novaTelInspvaPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::INSPVA>("novatel_top/inspva", qos);
      this->novaTelHeading2Publisher1_ = this->create_publisher<novatel_oem7_msgs::msg::HEADING2>("novatel_top/heading2", qos);
      this->novaTelRawImuPublisher1_ = this->create_publisher<novatel_oem7_msgs::msg::RAWIMU>("novatel_top/rawimu", qos);
      this->novaTelRawImuXPublisher1_ = this->create_publisher<sensor_msgs::msg::Imu>("novatel_top/rawimux", qos);

      this->novaTelBestPosPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_bottom/bestpos", qos);
      this->novaTelBestGNSSPosPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTPOS>("novatel_bottom/bestgnsspos", qos);
      this->novaTelBestVelPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_bottom/bestvel", qos);
      this->novaTelBestGNSSVelPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::BESTVEL>("novatel_bottom/bestgnssvel", qos);
      this->novaTelInspvaPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::INSPVA>("novatel_bottom/inspva", qos);
      this->novaTelHeading2Publisher2_ = this->create_publisher<novatel_oem7_msgs::msg::HEADING2>("novatel_bottom/heading2", qos);
      this->novaTelRawImuPublisher2_ = this->create_publisher<novatel_oem7_msgs::msg::RAWIMU>("novatel_bottom/rawimu", qos);
      this->novaTelRawImuXPublisher2_ = this->create_publisher<sensor_msgs::msg::Imu>("novatel_bottom/rawimux", qos);

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
        this->simClockTime.clock = rclcpp::Time(this->sec, this->nsec);
        this->simClockTimePublisher_->publish(this->simClockTime);
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Use Wall Clock (system clock).");
        this->updateVESIVehicleInputs_ = this->create_wall_timer(10ms, std::bind(&AsmSocketCanBridgeNode::vesiCallback, this));
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
                   << "publishSimulationState" << ","
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
    double scaled_value = (static_cast<double>(physical_value) - signal_information.offset) / signal_information.factor;
    uint32_t raw_value = static_cast<uint32_t>(std::round(scaled_value));

    if (signal_information.is_signed) {
        raw_value &= ((1u << signal_information.length) - 1);
    }

    for (int i = 0; i < signal_information.length; ++i) {
        int bitIndex = signal_information.endian ? (signal_information.start_bit + i)
                                                : (signal_information.start_bit - i);
        int byteIndex = bitIndex / 8;
        int bitInByte = signal_information.endian ? (bitIndex % 8)
                                                  : (7 - (bitIndex % 8));

        uint8_t bitVal = (raw_value >> i) & 0x01;
        data[byteIndex] &= ~(1 << bitInByte);       // Clear bit
        data[byteIndex] |= (bitVal << bitInByte);   // Set bit
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

    // Throttle command (%)
    this->feedbackCmd.vehicle_inputs.throttle_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.throttle_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_throttle_cmd = 0;

    // # Brake pressure command (kPa)
    this->feedbackCmd.vehicle_inputs.brake_cmd_front = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_rear = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_bias_switch = 0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_brake_cmd = 0;

    // # Steering motor angle command (degrees)
    this->feedbackCmd.vehicle_inputs.steering_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.steering_cmd_count = 0;
    this->feedbackCmd.vehicle_inputs.enable_steering_cmd = 0;

    // # Gear command
    this->feedbackCmd.vehicle_inputs.gear_cmd = 1;
    this->feedbackCmd.vehicle_inputs.enable_gear_cmd = 0;
    
    // # Race Control command
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
          if (!findMessage(in_frame.can_id)) {
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
          if (!findMessage(in_frame.can_id)) {
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
          if (!findMessage(in_frame.can_id)) {
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
          if (!findMessage(in_frame.can_id)) {
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
          if (!findMessage(in_frame.can_id)) {
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
          if (!findMessage(in_frame.can_id)) {
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
          if (!findMessage(in_frame.can_id)) {
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
          if (!findMessage(in_frame.can_id)) {
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
                                         struct can_frame frame)
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
    message_lookup_.reserve(can_message_info.size());
    message_signal_lookup_.reserve(can_message_info.size());

    for (auto &message : can_message_info) {
      message_lookup_.emplace(message.id, &message);
      auto &signal_map = message_signal_lookup_[message.id];
      signal_map.reserve(message.signals.size());
      for (const auto &signal : message.signals) {
        signal_map.emplace(signal.name, &signal);
      }
    }
  }

  const Message* AsmSocketCanBridgeNode::findMessage(uint32_t message_id) const
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
    // Configure simulated clock
    simClockTime.clock = rclcpp::Time(this->sec,this->nsec);
    this->simClockTimePublisher_->publish(simClockTime);
  }

  void AsmSocketCanBridgeNode::simTimeIncreaseCallback(const std_msgs::msg::UInt16 & msg)
  {
    for (uint16_t timeIncreaseStep = 0; timeIncreaseStep <  msg.data; timeIncreaseStep++)
    {
      vesiCallback();
    }
    simClockTimeCallback();
  }

  void AsmSocketCanBridgeNode::vesiCallback()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "vesiCallback");

    if (this->enableTimeRecord)
    {
      this->timeEndVESICallBackNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      this->measured_vesi_times.push_back((int64_t(this->timeEndVESICallBackNanosec) - int64_t(this->timeStartVESICallBackNanosec)) / 1000000.0);

      if (this->timeStartNanosec != 0 && this->timeStartVESICallBackNanosec != 0 && this->measured_vesi_times.size() == 5)
      {
        // write measurments to csv log file
        this->myfile.open(std::string(this->pathTimeRecord) + "/duration_recording.csv", std::ios_base::app);
        this->myfile << std::to_string(measured_vesi_times[0]) << ","
                     << std::to_string(measured_vesi_times[1]) << ","
                     << std::to_string(measured_vesi_times[2]) << ","
                     << std::to_string(measured_vesi_times[3]) << ","
                     << std::to_string(measured_vesi_times[4]) << "\n";
        this->myfile.close();
      }

      this->measured_vesi_times.clear();
      this->timeStartVESICallBackNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }

    try
    {

      if (this->maneuverStarted == true)
      {
        if (this->enableTimeRecord){
          this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        }
        AsmSocketCanBridgeNode::sendVehicleFeedbackToSimulation();
        if (this->enableTimeRecord){
          this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
          this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
        }

        if (this->enableTimeRecord){
          this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        }
        this->api.requestCustomData(&canbus_raw_buffer_);
        if (this->enableTimeRecord){
          this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
          this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
        }

        if(this->simModeEnabled)
        {
          this->simTotalMsec += 1;
          this->sec = this->simTotalMsec / 1000;
          this->nsec = (this->simTotalMsec % 1000) * 1000000;
        }
        else
        {
          this->simTotalMsec += 10;
        }
      }
      else
      {
        this->api.requestCustomData(&canbus_raw_buffer_);
      }

      if (this->enableTimeRecord){
        this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      }
      constexpr auto required_canbus_size = sizeof(ASMBus);
      if (canbus_raw_buffer_.size() >= required_canbus_size)
      {
        std::memcpy(&canBusStorage_, canbus_raw_buffer_.data(), required_canbus_size);
        this->canBus = &canBusStorage_;
        if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState == 3 && this->maneuverStarted == false)
        {
          this->maneuverStarted = true;
          RCLCPP_INFO(get_logger(), "Maneuver started. Data will be published.");
        } else if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState != 3 && this->maneuverStarted == true)
        {
          RCLCPP_INFO(get_logger(), "Maneuver stopped. System will be reset.");
          initializeFeedback();
          std_msgs::msg::Bool resetMsg;
          resetMsg.data = true;
          this->resetCommandPublisher_->publish(resetMsg);
        }
        this->vesiDataAvailabe = true;
      }
      else 
      {
        this->canBus = nullptr;
        this->vesiDataAvailabe = false;
        if (canbus_raw_buffer_.empty())
        {
          if (this->verbosePrinting) {
            RCLCPP_WARN(get_logger(), "No Custom Data available.");
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *this->get_clock(),
                                 5000,
                                 "No Custom Data available.");
          }
        }
        else
        {
          RCLCPP_ERROR(get_logger(),
                       "Custom data buffer size (%zu) smaller than ASMBus (%zu); ignoring frame.",
                       canbus_raw_buffer_.size(),
                       required_canbus_size);
        }
      }
      if (this->enableTimeRecord){
        this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
      }

    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to request data from ASM: %s", e.what());
    }

    if (this->enableTimeRecord){
      this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    AsmSocketCanBridgeNode::publishSimulationState();
    if (this->enableTimeRecord){
      this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
    }
  }

  void AsmSocketCanBridgeNode::publishSimulationState()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishSimulationState");

    if (!this->canBus) {
      RCLCPP_ERROR(get_logger(), "canBus pointer is null.");
      return;
    }

    try
    {
      if (this->vesiDataAvailabe == true && this->simTotalMsec != 0)
      {
        if(this->simTotalMsec % (this->pubIntervalRaceControlData) == 0) {
          AsmSocketCanBridgeNode::publishBaseToCarSummary();
          AsmSocketCanBridgeNode::publishMarelliReport();
          AsmSocketCanBridgeNode::publishMiscRCReport();
        }

        if(this->simTotalMsec % (this->pubIntervalVehicleData) == 0) {
          AsmSocketCanBridgeNode::publishSteeringReport();
          AsmSocketCanBridgeNode::publishBrakeReport();
          AsmSocketCanBridgeNode::publishAcceleratorReport();
          AsmSocketCanBridgeNode::publishWheelReport();
          AsmSocketCanBridgeNode::publishMiscReport();
          AsmSocketCanBridgeNode::publishDiagnosticReport();
          AsmSocketCanBridgeNode::publishVectorIndependentSigMsg();
        }

        if(this->simTotalMsec % (this->pubIntervalPowertrainData) == 0)
          AsmSocketCanBridgeNode::publishPtReport();
        
        if(this->simTotalMsec % (this->pubIntervalVectorNavData) == 0)
          AsmSocketCanBridgeNode::publishVectorNavData();

        if(this->simTotalMsec % (this->pubIntervalNovatelData) == 0)
        {            
          AsmSocketCanBridgeNode::publishNovatelReport();
          AsmSocketCanBridgeNode::publishNovatelData(1);
          AsmSocketCanBridgeNode::publishNovatelData(2);
        }
        if(this->simTotalMsec % (this->pubIntervalFoxgloveMap) == 0)
          AsmSocketCanBridgeNode::publishFoxgloveMap(0);

        if (this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count > 10) {
          RCLCPP_ERROR(get_logger(), "Unreasonable fellow_count value: %f -> GroundTruthArray and fellows in FoxgloveMap will not be published", this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);
        }
        else if (this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count == 0)
        {
          if (this->verbosePrinting)
            RCLCPP_INFO(get_logger(), "No fellows included in the scenario. No GroundTruthArray will be published and FoxgloveMap will be published only for the ego");
        }
        else {
          if (this->verbosePrinting)
            RCLCPP_INFO(get_logger(), "Reasonable fellow_count value: %f", this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);
          if(this->simTotalMsec % (this->pubIntervalGroundTruthArray) == 0)
            AsmSocketCanBridgeNode::publishGroundTruthArray();
          if(this->simTotalMsec % (this->pubIntervalFoxgloveMap) == 0)
            for (size_t fellowID = 1; fellowID <= std::min(3, (int)this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count); fellowID++)
              AsmSocketCanBridgeNode::publishFoxgloveMap(fellowID);
        }

        this->vesiDataAvailabe = false;
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Publishing of data failed: %s", e.what());
    }
  }

  void AsmSocketCanBridgeNode::publishFoxgloveMap(uint8_t fellowID)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "publishFoxgloveMap fellowID: %d", fellowID);

    // Best Pos
    auto foxgloveMap = sensor_msgs::msg::NavSatFix();

    foxgloveMap.status.status = -1;
    foxgloveMap.status.service = 1;

    if (fellowID == 0)
      {
        foxgloveMap.latitude = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat;
        foxgloveMap.longitude = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon;
        foxgloveMap.altitude = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.hgt;
        this->foxgloveMapPublisher_ = this->foxgloveMapPublisher0_;
      }
    else if (fellowID == 1)
      {
        foxgloveMap.latitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[0];
        foxgloveMap.longitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[0];
        foxgloveMap.altitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[0];
        this->foxgloveMapPublisher_ = this->foxgloveMapPublisher1_;
      }
    else if (fellowID == 2)
      {
        foxgloveMap.latitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[1];
        foxgloveMap.longitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[1];
        foxgloveMap.altitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[1];
        this->foxgloveMapPublisher_ = this->foxgloveMapPublisher2_;
      }
    else if (fellowID == 3)
      {
        foxgloveMap.latitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[2];
        foxgloveMap.longitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[2];
        foxgloveMap.altitude = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[2];
        this->foxgloveMapPublisher_ = this->foxgloveMapPublisher3_;
      }
    else
      {
        RCLCPP_ERROR(get_logger(), "Unknown Fellow ID. Only three Fellows are supported.");
      }
    

    foxgloveMap.position_covariance_type = 0;

    foxgloveMap.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      foxgloveMap.header.stamp.sec = this->sec;
      foxgloveMap.header.stamp.nanosec = this->nsec;
    }
    else
    {
      foxgloveMap.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      foxgloveMap.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (foxgloveMap.header.stamp.sec*1000000000);
    }
  
    this->foxgloveMapPublisher_->publish(foxgloveMap);
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

  void AsmSocketCanBridgeNode::publishBaseToCarSummary()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishBaseToCarSummary");
    
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "publishBaseToCarSummary"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "base_to_car_heartbeat")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.base_to_car_heartbeat);
          if (current_signal.name == "track_flag")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.track_flag);
          if (current_signal.name == "veh_flag")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.veh_flag);
          if (current_signal.name == "veh_rank")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.veh_rank);
          if (current_signal.name == "lap_count")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.lap_count);
          if (current_signal.name == "lap_distance")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.lap_distance);
          if (current_signal.name == "round_target_speed")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.round_target_speed);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::base_to_car_summary");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishMarelliReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishMarelliReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "marelli_report_1"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "marelli_track_flag")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.track_flag);
          if (current_signal.name == "marelli_vehicle_flag")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.veh_flag);
          if (current_signal.name == "marelli_sector_flag")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.track_flag);
          if (current_signal.name == "marelli_rc_base_sync_check")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, true);
          if (current_signal.name == "marelli_rc_lte_rssi")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 255);
        }
        break;
      }
    }

    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::marelli_report_1");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "marelli_report_2"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "marelli_gps_lat")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat);
          if (current_signal.name == "marelli_gps_long")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon);
        }
        break;
      }
    }

    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::marelli_report_2");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishMiscRCReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishMarelliReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "base_to_car_timing"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "laps")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.lap_count);
          if (current_signal.name == "lap_time")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.lap_time);
          if (current_signal.name == "time_stamp")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.time_stamp);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::base_to_car_timing");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "rest_of_field"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "comp_veh_num")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "comp_rank")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "comp_veh_flag")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "comp_laps_count")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "comp_lap_distance")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "comp_speed")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::rest_of_field");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishPtReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishPtReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "pt_report_1"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "throttle_position")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.throttle_position);
          if (current_signal.name == "current_gear")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.current_gear);
          if (current_signal.name == "engine_speed_rpm")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_rpm);
          if (current_signal.name == "vehicle_speed_kmph")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.vehicle_speed_kmph);
          if (current_signal.name == "engine_run_switch")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_run_switch_status);
          if (current_signal.name == "engine_state")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_on_status);
          if (current_signal.name == "gear_shift_status")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.gear_shift_status);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::pt_report_1");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "pt_report_2"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "fuel_pressure_kPa")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.fuel_pressure);
          if (current_signal.name == "engine_oil_pressure_kPa")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_oil_pressure);
          if (current_signal.name == "coolant_temperature")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_coolant_temperature);
          if (current_signal.name == "transmission_temperature")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_oil_temperature);
          if (current_signal.name == "transmission_pressure_kPa")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_oil_pressure);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::pt_report_2");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "pt_report_3"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "engine_oil_temperature")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_oil_temperature);
          if (current_signal.name == "torque_wheels")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.torque_wheels_nm);
          if (current_signal.name == "driver_traction_aim_swicth_fbk")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "driver_traction_range_switch_fbk")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "push2pass_status")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.push2pass_status);
          if (current_signal.name == "push2pass_budget_s")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.push2pass_budget_s);
          if (current_signal.name == "push2pass_active_app_limit")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->asm_bus_var.race_control_var.push2pass_active_app_limit);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::pt_report_3");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishSteeringReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishSteeringReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "steering_report"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "steering_motor_fdbk_counter")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "primary_steering_angular_rate")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "commanded_steering_rate")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::steering_report");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "steering_report_extd"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "average_steering_ang_fdbk")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle);
          if (current_signal.name == "primary_steering_angle_fbk")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle);
          if (current_signal.name == "secondary_steering_ang_fdbk")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::steering_report_extd");

      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "steering_report_extd_2"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "motor_duty_cycle_cmd")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "motor_duty_cycle_fbk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "motor_current_fbk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sbw_ecu_voltage")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sbw_ecu_temp")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sbw_error_code")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sbw_motor_torque_estimate")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::steering_report_extd_2");

      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "steering_report_extd_3"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "steering_p_contribution")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "steering_i_contribution")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "steering_d_contribution")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::steering_report_extd_3");

      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishBrakeReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishBrakeReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "brake_pressure_report"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "brk_pressure_fdbk_counter")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "brake_pressure_fdbk_rear")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rear_brake_pressure);
          if (current_signal.name == "brake_pressure_fdbk_front")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.front_brake_pressure);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::brake_pressure_report");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "brake_report_extd"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "F_brk_pos_cmd")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "F_brk_pos_fbk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "R_brk_pos_cmd")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "R_brk_pos_fbk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::brake_report_extd");

      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "brake_report_extd_2"){
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals){
          if (current_signal.name == "f_brake_act_force")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "r_brake_act_force")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::brake_report_extd_2");

      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishAcceleratorReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishAcceleratorReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "accelerator_report") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "acc_pedal_fdbk_counter")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "acc_pedal_fdbk")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.accel_pedal_output);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::accelerator_report");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishWheelReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishWheelReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RR_1") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RR_Tire_Temp_01")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_02")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_03")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_04")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RR_1");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RR_2") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RR_Tire_Temp_05")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_06")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_07")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_08")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RR_2");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RR_3") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RR_Tire_Temp_09")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_10")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_11")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_12")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RR_3");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RR_4") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RR_Tire_Temp_13")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_14")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_15")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
          if (current_signal.name == "RR_Tire_Temp_16")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RR_4");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RL_1") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RL_Tire_Temp_01")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_02")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_03")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_04")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RL_1");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RL_2") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RL_Tire_Temp_05")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_06")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_07")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_08")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RL_2");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RL_3") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RL_Tire_Temp_09")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_10")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_11")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_12")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RL_3");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_RL_4") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RL_Tire_Temp_13")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_14")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_15")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
          if (current_signal.name == "RL_Tire_Temp_16")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_RL_4");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FR_1") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FR_Tire_Temp_01")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_02")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_03")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_04")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FR_1");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FR_2") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FR_Tire_Temp_05")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_06")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_07")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_08")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FR_2");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FR_3") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FR_Tire_Temp_09")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_10")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_11")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_12")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FR_3");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FR_4") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FR_Tire_Temp_13")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_14")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_15")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
          if (current_signal.name == "FR_Tire_Temp_16")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FR_4");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FL_1") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FL_Tire_Temp_01")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_02")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_03")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_04")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FL_1");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FL_2") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FL_Tire_Temp_05")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_06")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_07")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_08")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FL_2");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FL_3") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FL_Tire_Temp_09")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_10")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_11")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_12")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FL_3");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Temp_FL_4") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FL_Tire_Temp_13")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_14")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_15")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
          if (current_signal.name == "FL_Tire_Temp_16")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Temp_FL_4");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Pressure_RR") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RR_Tire_Pressure_Gauge")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_pressure_gauge);
          if (current_signal.name == "RR_Tire_Pressure")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_pressure);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Pressure_RR");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Pressure_RL") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "RL_Tire_Pressure_Gauge")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_pressure_gauge);
          if (current_signal.name == "RL_Tire_Pressure")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_pressure);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Pressure_RL");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Pressure_FR") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FR_Tire_Pressure_Gauge")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_pressure_gauge);
          if (current_signal.name == "FR_Tire_Pressure")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_pressure);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Pressure_FR");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
    for (const auto& current_message : can_message_info) {
      if (current_message.name == "Tire_Pressure_FL") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "FL_Tire_Pressure_Gauge")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_pressure_gauge);
          if (current_signal.name == "FL_Tire_Pressure")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_pressure);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::Tire_Pressure_FL");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "wheel_strain_gauge") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "wheel_strain_gauge_RR")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_wheel_load);
          if (current_signal.name == "wheel_strain_gauge_RL")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_wheel_load);
          if (current_signal.name == "wheel_strain_gauge_FR")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_wheel_load);
          if (current_signal.name == "wheel_strain_gauge_FL")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_wheel_load);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::wheel_strain_gauge");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "wheel_potentiometer_data") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "wheel_potentiometer_RR")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_damper_linear_potentiometer);
          if (current_signal.name == "wheel_potentiometer_RL")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_damper_linear_potentiometer);
          if (current_signal.name == "wheel_potentiometer_FR")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_damper_linear_potentiometer);
          if (current_signal.name == "wheel_potentiometer_FL")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_damper_linear_potentiometer);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::wheel_potentiometer_data");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "wheel_speed_report") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "wheel_speed_RR")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_right);
          if (current_signal.name == "wheel_speed_RL")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_left);
          if (current_signal.name == "wheel_speed_FR")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_right);
          if (current_signal.name == "wheel_speed_FL")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_left);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::wheel_speed_report");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishMiscReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishMiscReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "misc_report") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "battery_voltage")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.battery_voltage);
          if (current_signal.name == "safety_switch_state")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.safety_switch_state);
          if (current_signal.name == "mode_switch_state")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.mode_switch_state);
          if (current_signal.name == "sys_state")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.sys_state);
          if (current_signal.name == "raptor_rolling_counter")
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::misc_report");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishDiagnosticReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishDiagnosticReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "diagnostic_report") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "sd_system_warning")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sd_system_failure")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sd_brake_warning1")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sd_brake_warning2")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sd_brake_warning3")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sd_steer_warning1")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sd_steer_warning2")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "sd_steer_warning3")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "motec_warning")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "est1_oos_front_brk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "est2_oos_rear_brk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "est3_low_eng_speed")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "est4_sd_comms_loss")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "est5_motec_comms_loss")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "est6_sd_ebrake")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "adlink_hb_lost")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "rc_lost")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::diagnostic_report");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishVectorIndependentSigMsg()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishVectorNav");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "VECTOR__INDEPENDENT_SIG_MSG") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "ang_heading")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_heading_ins);
          if (current_signal.name == "pos_y")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.y);
          if (current_signal.name == "pos_x")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.x);
          if (current_signal.name == "yaw_rate")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.angularrate_var.z);
          if (current_signal.name == "velocity_long")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.x);
          if (current_signal.name == "velocity_lat")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.y);
          if (current_signal.name == "motor_angle")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle);
          if (current_signal.name == "acceleration")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.accel_var.x);
          if (current_signal.name == "rc_base_sync_check")
            this->insertBits(can_out_frame.data, current_signal, true);
          if (current_signal.name == "rc_lte_rssi")
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "duty_cycle_fbk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "duty_cycle_dmd")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
          if (current_signal.name == "steering_motor_ang_avg_fdbk")
            // TODO: Check the exact priority and meaning of the field and if required add value from ASM
            this->insertBits(can_out_frame.data, current_signal, 0);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::VECTOR__INDEPENDENT_SIG_MSG");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishNovatelReport()
  {
    if (this->sentMessagePrinting)
      RCLCPP_INFO(get_logger(), "publishNovatelReport");

    for (const auto& current_message : can_message_info) {
      if (current_message.name == "novatel_report") {
        can_out_frame.can_id = current_message.id;
        can_out_frame.can_dlc = current_message.dlc;
        std::memset(can_out_frame.data, 0, can_out_frame.can_dlc);
        for (const auto& current_signal : current_message.signals) {
          if (current_signal.name == "novatel_lat")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lat);
          if (current_signal.name == "novatel_long")
            this->insertBits(can_out_frame.data, current_signal, this->canBus->sim_interface_var.nova_tel_pwr_pak1_var.best_pos_var.lon);
        }
        break;
      }
    }
    if (this->sentMessagePrinting) {
      RCLCPP_INFO(get_logger(), "can_out::novatel_report");
      RCLCPP_INFO(get_logger(), "send: 0x%03X [%d] ",can_out_frame.can_id, can_out_frame.can_dlc);
      for (int i = 0; i < can_out_frame.can_dlc; i++)
        RCLCPP_INFO(get_logger(), "send: %02X ",can_out_frame.data[i]);
    }
    can_write(this->can_socket, can_out_frame);
  }

  void AsmSocketCanBridgeNode::publishVectorNavData()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "publishVectorNavData");

    auto attitudeGroup = vectornav_msgs::msg::AttitudeGroup();
    auto commonGroup = vectornav_msgs::msg::CommonGroup();
    auto imuGroup = vectornav_msgs::msg::ImuGroup();
    auto gpsGroup = vectornav_msgs::msg::GpsGroup();
    auto insGroup = vectornav_msgs::msg::InsGroup();
    auto timeGroup = vectornav_msgs::msg::TimeGroup();

    attitudeGroup.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      attitudeGroup.header.stamp.sec = this->sec;
      attitudeGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      attitudeGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      attitudeGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (attitudeGroup.header.stamp.sec*1000000000);
    }

    attitudeGroup.vpestatus.attitude_quality = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.attitude_quality;
    attitudeGroup.vpestatus.gyro_saturation = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.gyro_saturation;
    attitudeGroup.vpestatus.gyro_saturation_recovery = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.gyro_saturation_recovery;
    attitudeGroup.vpestatus.mag_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.mag_disturbance;
    attitudeGroup.vpestatus.mag_saturation = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.mag_saturation;
    attitudeGroup.vpestatus.acc_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.acc_disturbance;
    attitudeGroup.vpestatus.acc_saturation = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.acc_saturation;
    attitudeGroup.vpestatus.known_mag_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.known_mag_disturbance;
    attitudeGroup.vpestatus.known_accel_disturbance = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.vpestatus_var.known_accel_disturbance;

    attitudeGroup.yawpitchroll.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.yawpitchroll_var.x; 
    attitudeGroup.yawpitchroll.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.yawpitchroll_var.y; 
    attitudeGroup.yawpitchroll.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.yawpitchroll_var.z;

    attitudeGroup.quaternion.w = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.w;
    attitudeGroup.quaternion.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.x;
    attitudeGroup.quaternion.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.y;
    attitudeGroup.quaternion.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.quaternion_var.z;
    
    attitudeGroup.dcm[0] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[0];
    attitudeGroup.dcm[1] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[1];
    attitudeGroup.dcm[2] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[2];
    attitudeGroup.dcm[3] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[3];
    attitudeGroup.dcm[4] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[4];
    attitudeGroup.dcm[5] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[5];
    attitudeGroup.dcm[6] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[6];
    attitudeGroup.dcm[7] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[7];
    attitudeGroup.dcm[8] = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.dcm[8];
    
    attitudeGroup.magned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.magned_var.x;
    attitudeGroup.magned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.magned_var.y;
    attitudeGroup.magned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.magned_var.z;

    attitudeGroup.accelned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.accelned_var.x;
    attitudeGroup.accelned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.accelned_var.y;
    attitudeGroup.accelned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.accelned_var.z;

    attitudeGroup.linearaccelbody.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelbody_var.x;
    attitudeGroup.linearaccelbody.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelbody_var.y;
    attitudeGroup.linearaccelbody.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelbody_var.z;

    attitudeGroup.linearaccelned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelned_var.x;
    attitudeGroup.linearaccelned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelned_var.y;
    attitudeGroup.linearaccelned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.linearaccelned_var.z;
    
    attitudeGroup.ypru.x = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.ypru_var.x;
    attitudeGroup.ypru.y = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.ypru_var.y;
    attitudeGroup.ypru.z = this->canBus->sim_interface_var.vector_nav_vn1_var.attitude_group_var.ypru_var.z;

    this->verctorNavAttitudeGroupPublisher_->publish(attitudeGroup);

    commonGroup.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      commonGroup.header.stamp.sec = this->sec;
      commonGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      commonGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      commonGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (commonGroup.header.stamp.sec*1000000000);
    }

    commonGroup.timestartup = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timestartup;
    commonGroup.timegps = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timegps;
    commonGroup.timesyncin = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timesyncin;

    commonGroup.yawpitchroll.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.yawpitchroll_var.x;
    commonGroup.yawpitchroll.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.yawpitchroll_var.y;
    commonGroup.yawpitchroll.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.yawpitchroll_var.z;

    commonGroup.quaternion.w = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.w;
    commonGroup.quaternion.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.x;
    commonGroup.quaternion.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.y;
    commonGroup.quaternion.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.quaternion_var.z;

    commonGroup.angularrate.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.angularrate_var.x;
    commonGroup.angularrate.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.angularrate_var.y;
    commonGroup.angularrate.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.angularrate_var.z;

    commonGroup.position.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.x;
    commonGroup.position.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.y;
    commonGroup.position.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.position_var.z;

    commonGroup.velocity.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.x;
    commonGroup.velocity.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.y;
    commonGroup.velocity.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.velocity_var.z;

    commonGroup.accel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.accel_var.x;
    commonGroup.accel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.accel_var.y;
    commonGroup.accel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.accel_var.z;

    commonGroup.imu_accel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_accel_var.x;
    commonGroup.imu_accel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_accel_var.y;
    commonGroup.imu_accel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_accel_var.z;

    commonGroup.imu_rate.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_rate_var.x;
    commonGroup.imu_rate.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_rate_var.y;
    commonGroup.imu_rate.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.imu_rate_var.z;

    commonGroup.magpres_mag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_mag_var.x;
    commonGroup.magpres_mag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_mag_var.y;
    commonGroup.magpres_mag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_mag_var.z;

    commonGroup.magpres_temp = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_temp;
    commonGroup.magpres_pres = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.magpres_pres;
    commonGroup.deltatheta_dtime = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtime;

    commonGroup.deltatheta_dtheta.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtheta_var.x;
    commonGroup.deltatheta_dtheta.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtheta_var.y;
    commonGroup.deltatheta_dtheta.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dtheta_var.z;

    commonGroup.deltatheta_dvel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dvel_var.x;
    commonGroup.deltatheta_dvel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dvel_var.y;
    commonGroup.deltatheta_dvel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.deltatheta_dvel_var.z;

    commonGroup.insstatus.gps_fix = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_fix;
    commonGroup.insstatus.time_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.time_error;
    commonGroup.insstatus.imu_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.imu_error;
    commonGroup.insstatus.mag_pres_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.mag_pres_error;
    commonGroup.insstatus.gps_error = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_error;
    commonGroup.insstatus.gps_heading_ins = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_heading_ins;
    commonGroup.insstatus.gps_compass = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.insstatus_var.gps_compass;

    commonGroup.syncincnt = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.syncincnt;
    commonGroup.timegpspps = this->canBus->sim_interface_var.vector_nav_vn1_var.common_group_var.timegpspps;

    this->verctorNavCommonGroupPublisher_->publish(commonGroup);

    imuGroup.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      imuGroup.header.stamp.sec = this->sec;
      imuGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      imuGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      imuGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (imuGroup.header.stamp.sec*1000000000);
    }

    imuGroup.imustatus = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.imustatus;

    imuGroup.uncompmag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.x;
    imuGroup.uncompmag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.y;
    imuGroup.uncompmag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.z;

    imuGroup.uncompmag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.x;
    imuGroup.uncompmag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.y;
    imuGroup.uncompmag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompmag_var.z;

    imuGroup.uncompaccel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompaccel_var.x;
    imuGroup.uncompaccel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompaccel_var.y;
    imuGroup.uncompaccel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompaccel_var.z;

    imuGroup.uncompgyro.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompgyro_var.x;
    imuGroup.uncompgyro.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompgyro_var.y;
    imuGroup.uncompgyro.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.uncompgyro_var.z;

    imuGroup.temp = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.temp;
    imuGroup.pres = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.pres;
    imuGroup.deltatheta_time = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_time;

    imuGroup.deltatheta_dtheta.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_dtheta_var.x;
    imuGroup.deltatheta_dtheta.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_dtheta_var.y;
    imuGroup.deltatheta_dtheta.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltatheta_dtheta_var.z;

    imuGroup.deltavel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltavel_var.x;
    imuGroup.deltavel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltavel_var.y;
    imuGroup.deltavel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.deltavel_var.z;

    imuGroup.mag.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.mag_var.x;
    imuGroup.mag.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.mag_var.y;
    imuGroup.mag.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.mag_var.z;

    imuGroup.accel.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.accel_var.x;
    imuGroup.accel.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.accel_var.y;
    imuGroup.accel.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.accel_var.z;

    imuGroup.angularrate.x = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.angularrate_var.x;
    imuGroup.angularrate.y = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.angularrate_var.y;
    imuGroup.angularrate.z = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.angularrate_var.z;

    imuGroup.sensat = this->canBus->sim_interface_var.vector_nav_vn1_var.imu_group_var.sensat;

    this->verctorNavImuGroupPublisher_->publish(imuGroup);
    

    for(int i=0;i<2;i++)
    {
      gps_group currentGPS;
      if (i == 0)
      {
        currentGPS = this->canBus->sim_interface_var.vector_nav_vn1_var.gps_group1_var;
        this->verctorNavGpsGroupPublisher = this->verctorNavGpsGroupLeftPublisher_;
        }
      else if (i == 1)
      {
        currentGPS = this->canBus->sim_interface_var.vector_nav_vn1_var.gps_group2_var;
        this->verctorNavGpsGroupPublisher = this->verctorNavGpsGroupRightPublisher_;
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Only two Vectornav GPS antennas are supported.");
      }

      gpsGroup.header.frame_id = "world";

      if(this->simModeEnabled)
      {
        gpsGroup.header.stamp.sec = this->sec;
        gpsGroup.header.stamp.nanosec = this->nsec;
      }
      else
      {
        gpsGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        gpsGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (gpsGroup.header.stamp.sec*1000000000);
      }


      gpsGroup.utc.year = currentGPS.utc_var.year;
      gpsGroup.utc.month = currentGPS.utc_var.month;
      gpsGroup.utc.day = currentGPS.utc_var.day;
      gpsGroup.utc.hour = currentGPS.utc_var.hour;
      gpsGroup.utc.min = currentGPS.utc_var.min;
      gpsGroup.utc.sec = currentGPS.utc_var.sec;
      gpsGroup.utc.ms = currentGPS.utc_var.ms;

      gpsGroup.tow = currentGPS.tow;
      gpsGroup.week = currentGPS.week;
      gpsGroup.numsats = currentGPS.numsats;
      gpsGroup.fix = currentGPS.fix;

      gpsGroup.poslla.x = currentGPS.poslla_var.x;
      gpsGroup.poslla.y = currentGPS.poslla_var.y;
      gpsGroup.poslla.z = currentGPS.poslla_var.z;

      gpsGroup.posecef.x = currentGPS.posecef_var.x;
      gpsGroup.posecef.y = currentGPS.posecef_var.y;
      gpsGroup.posecef.z = currentGPS.posecef_var.z;

      gpsGroup.velned.x = currentGPS.velned_var.x;
      gpsGroup.velned.y = currentGPS.velned_var.y;
      gpsGroup.velned.z = currentGPS.velned_var.z;

      gpsGroup.velecef.x = currentGPS.velecef_var.x;
      gpsGroup.velecef.y = currentGPS.velecef_var.y;
      gpsGroup.velecef.z = currentGPS.velecef_var.z;

      gpsGroup.posu.x = currentGPS.posu_var.x;
      gpsGroup.posu.y = currentGPS.posu_var.y;
      gpsGroup.posu.z = currentGPS.posu_var.z;

      gpsGroup.velu = currentGPS.velu;
      gpsGroup.timeu = currentGPS.timeu;
      gpsGroup.timeinfo_status = currentGPS.timeinfo_status;
      gpsGroup.timeinfo_leapseconds = currentGPS.timeinfo_leapseconds;

      gpsGroup.dop.g = currentGPS.dop_var.g;
      gpsGroup.dop.p = currentGPS.dop_var.p;
      gpsGroup.dop.t = currentGPS.dop_var.t;
      gpsGroup.dop.v = currentGPS.dop_var.v;
      gpsGroup.dop.h = currentGPS.dop_var.h;
      gpsGroup.dop.n = currentGPS.dop_var.n;
      gpsGroup.dop.e = currentGPS.dop_var.e;

      this->verctorNavGpsGroupPublisher->publish(gpsGroup);
    }

    insGroup.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      insGroup.header.stamp.sec = this->sec;
      insGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      insGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      insGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (insGroup.header.stamp.sec*1000000000);
    }

    insGroup.insstatus.gps_fix = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_fix;
    insGroup.insstatus.time_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.time_error;
    insGroup.insstatus.imu_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.imu_error;
    insGroup.insstatus.mag_pres_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.mag_pres_error;
    insGroup.insstatus.gps_error = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_error;
    insGroup.insstatus.gps_heading_ins = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_heading_ins;
    insGroup.insstatus.gps_compass = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.insstatus_var.gps_compass;

    insGroup.poslla.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.x;
    insGroup.poslla.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.y;
    insGroup.poslla.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.z;

    insGroup.posecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posecef_var.x;
    insGroup.posecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posecef_var.y;
    insGroup.posecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posecef_var.z;

    insGroup.velbody.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velbody_var.x;
    insGroup.velbody.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velbody_var.y;
    insGroup.velbody.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velbody_var.z;

    insGroup.velned.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velned_var.x;
    insGroup.velned.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velned_var.y;
    insGroup.velned.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velned_var.z;

    insGroup.velecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velecef_var.x;
    insGroup.velecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velecef_var.y;
    insGroup.velecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velecef_var.z;

    insGroup.magecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.magecef_var.x;
    insGroup.magecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.magecef_var.y;
    insGroup.magecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.magecef_var.z;

    insGroup.accelecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.accelecef_var.x;
    insGroup.accelecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.accelecef_var.y;
    insGroup.accelecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.accelecef_var.z;

    insGroup.linearaccelecef.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.linearaccelecef_var.x;
    insGroup.linearaccelecef.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.linearaccelecef_var.y;
    insGroup.linearaccelecef.z = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.linearaccelecef_var.z;

    insGroup.posu = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.posu_var;
    insGroup.velu = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.velu;

    this->verctorNavInsGroupPublisher_->publish(insGroup);

    timeGroup.header.frame_id = "";
    
    if(this->simModeEnabled)
    {
      timeGroup.header.stamp.sec = this->sec;
      timeGroup.header.stamp.nanosec = this->nsec;
    }
    else
    {
      timeGroup.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      timeGroup.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (timeGroup.header.stamp.sec*1000000000);
    }

    timeGroup.timestartup = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestartup;
    timeGroup.timegps = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timegps;
    timeGroup.gpstow = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.gpstow;
    timeGroup.gpsweek = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.gpsweek;
    timeGroup.timesyncin = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timesyncin;
    timeGroup.timegpspps = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timegpspps;

    timeGroup.timeutc.year = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.year;
    timeGroup.timeutc.month = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.month;
    timeGroup.timeutc.day = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.day;
    timeGroup.timeutc.hour = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.hour;
    timeGroup.timeutc.min = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.min;
    timeGroup.timeutc.sec = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.sec;
    timeGroup.timeutc.ms = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timeutc_var.ms;

    timeGroup.syncincnt = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.syncincnt;
    timeGroup.syncoutcnt = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.syncoutcnt;

    timeGroup.timestatus.time_ok = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestatus_var.time_ok;
    timeGroup.timestatus.date_ok = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestatus_var.date_ok;
    timeGroup.timestatus.utctime_ok = this->canBus->sim_interface_var.vector_nav_vn1_var.time_group_var.timestatus_var.utctime_ok;

    this->verctorNavTimeGroupPublisher_->publish(timeGroup);
  }

  void AsmSocketCanBridgeNode::publishNovatelData(uint8_t novatelID)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "publishNovatelData");
    
    nova_tel_pwr_pak currentNovatel;
    
    if (novatelID == 1)
    {
      currentNovatel = this->canBus->sim_interface_var.nova_tel_pwr_pak1_var;
      this->novaTelBestPosPublisher = this->novaTelBestPosPublisher1_;
      this->novaTelBestGNSSPosPublisher = this->novaTelBestGNSSPosPublisher1_;
      this->novaTelBestVelPublisher = this->novaTelBestVelPublisher1_;
      this->novaTelBestGNSSVelPublisher = this->novaTelBestGNSSVelPublisher1_;
      this->novaTelInspvaPublisher = this->novaTelInspvaPublisher1_;
      this->novaTelHeading2Publisher = this->novaTelHeading2Publisher1_;
      this->novaTelRawImuPublisher = this->novaTelRawImuPublisher1_;
      this->novaTelRawImuXPublisher = this->novaTelRawImuXPublisher1_;
      }
    else if (novatelID == 2)
    {
      currentNovatel = this->canBus->sim_interface_var.nova_tel_pwr_pak2_var;
      this->novaTelBestPosPublisher = this->novaTelBestPosPublisher2_;
      this->novaTelBestGNSSPosPublisher = this->novaTelBestGNSSPosPublisher2_;
      this->novaTelBestVelPublisher = this->novaTelBestVelPublisher2_;
      this->novaTelBestGNSSVelPublisher = this->novaTelBestGNSSVelPublisher2_;
      this->novaTelInspvaPublisher = this->novaTelInspvaPublisher2_;
      this->novaTelHeading2Publisher = this->novaTelHeading2Publisher2_;
      this->novaTelRawImuPublisher = this->novaTelRawImuPublisher2_;
      this->novaTelRawImuXPublisher = this->novaTelRawImuXPublisher2_;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
    }
    
    // Best Pos
    auto bestPos = novatel_oem7_msgs::msg::BESTPOS();

    bestPos.nov_header.message_name = currentNovatel.best_pos_var.nov_header_var.message_name[0];
    bestPos.nov_header.message_id = currentNovatel.best_pos_var.nov_header_var.message_id;
    bestPos.nov_header.message_type = currentNovatel.best_pos_var.nov_header_var.message_type;
    bestPos.nov_header.sequence_number = currentNovatel.best_pos_var.nov_header_var.sequence_number;
    bestPos.nov_header.time_status = currentNovatel.best_pos_var.nov_header_var.time_status;
    bestPos.nov_header.gps_week_number = currentNovatel.best_pos_var.nov_header_var.gps_week_number;
    bestPos.nov_header.gps_week_milliseconds = currentNovatel.best_pos_var.nov_header_var.gps_week_milliseconds;
    bestPos.nov_header.idle_time = currentNovatel.best_pos_var.nov_header_var.idle_time;

    bestPos.sol_status.status = currentNovatel.best_pos_var.sol_status;

    bestPos.pos_type.type = currentNovatel.best_pos_var.pos_type;
    
    bestPos.lat = currentNovatel.best_pos_var.lat;
    bestPos.lon = currentNovatel.best_pos_var.lon;
    bestPos.hgt = currentNovatel.best_pos_var.hgt;
    bestPos.undulation = currentNovatel.best_pos_var.undulation;
    bestPos.datum_id = currentNovatel.best_pos_var.datum_id;
    bestPos.lat_stdev = currentNovatel.best_pos_var.lat_stdev;
    bestPos.lon_stdev = currentNovatel.best_pos_var.lon_stdev;
    bestPos.hgt_stdev = currentNovatel.best_pos_var.hgt_stdev;

    bestPos.stn_id[0] = currentNovatel.best_pos_var.stn_id[0];
    bestPos.stn_id[1] = currentNovatel.best_pos_var.stn_id[1];
    bestPos.stn_id[2] = currentNovatel.best_pos_var.stn_id[2];
    bestPos.stn_id[3] = currentNovatel.best_pos_var.stn_id[3];

    bestPos.diff_age = currentNovatel.best_pos_var.diff_age;
    bestPos.sol_age = currentNovatel.best_pos_var.sol_age;
    bestPos.num_svs = currentNovatel.best_pos_var.num_svs;
    bestPos.num_sol_svs = currentNovatel.best_pos_var.num_sol_svs;
    bestPos.num_sol_l1_svs = currentNovatel.best_pos_var.num_sol_l1_svs;
    bestPos.num_sol_multi_svs = currentNovatel.best_pos_var.num_sol_multi_svs;
    bestPos.reserved = currentNovatel.best_pos_var.reserved;

    bestPos.ext_sol_stat.status = currentNovatel.best_pos_var.ext_sol_stat;

    bestPos.galileo_beidou_sig_mask = currentNovatel.best_pos_var.galileo_beidou_sig_mask;
    bestPos.gps_glonass_sig_mask = currentNovatel.best_pos_var.gps_glonass_sig_mask;

    bestPos.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      bestPos.header.stamp.sec = this->sec;
      bestPos.header.stamp.nanosec = this->nsec;
    }
    else
    {
      bestPos.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      bestPos.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (bestPos.header.stamp.sec*1000000000);
    }

    this->novaTelBestPosPublisher->publish(bestPos);
    this->novaTelBestGNSSPosPublisher->publish(bestPos);
    
    // Best Vel
    auto bestVel = novatel_oem7_msgs::msg::BESTVEL();

    bestVel.nov_header.message_name = currentNovatel.best_vel_var.nov_header_var.message_name[0];
    bestVel.nov_header.message_id = currentNovatel.best_vel_var.nov_header_var.message_id;
    bestVel.nov_header.message_type = currentNovatel.best_vel_var.nov_header_var.message_type;
    bestVel.nov_header.sequence_number = currentNovatel.best_vel_var.nov_header_var.sequence_number;
    bestVel.nov_header.time_status = currentNovatel.best_vel_var.nov_header_var.time_status;
    bestVel.nov_header.gps_week_number = currentNovatel.best_vel_var.nov_header_var.gps_week_number;
    bestVel.nov_header.gps_week_milliseconds = currentNovatel.best_vel_var.nov_header_var.gps_week_milliseconds;
    bestVel.nov_header.idle_time = currentNovatel.best_vel_var.nov_header_var.idle_time;

    bestVel.sol_status.status = currentNovatel.best_vel_var.sol_status;

    bestVel.vel_type.type = currentNovatel.best_vel_var.vel_type;
    
    bestVel.latency = currentNovatel.best_vel_var.latency;
    bestVel.diff_age = currentNovatel.best_vel_var.diff_age;
    bestVel.hor_speed = currentNovatel.best_vel_var.hor_speed;
    bestVel.trk_gnd = currentNovatel.best_vel_var.trk_gnd;
    bestVel.ver_speed = currentNovatel.best_vel_var.ver_speed;
    bestVel.reserved = currentNovatel.best_vel_var.reserved;

    bestVel.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      bestVel.header.stamp.sec = this->sec;
      bestVel.header.stamp.nanosec = this->nsec;
    }
    else
    {
      bestVel.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      bestVel.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (bestVel.header.stamp.sec*1000000000);
    }

    this->novaTelBestVelPublisher->publish(bestVel);
    this->novaTelBestGNSSVelPublisher->publish(bestVel);

    // Inspva
    auto inspva = novatel_oem7_msgs::msg::INSPVA();

    inspva.nov_header.message_name = currentNovatel.inspava_var.nov_header_var.message_name[0];
    inspva.nov_header.message_id = currentNovatel.inspava_var.nov_header_var.message_id;
    inspva.nov_header.message_type = currentNovatel.inspava_var.nov_header_var.message_type;
    inspva.nov_header.sequence_number = currentNovatel.inspava_var.nov_header_var.sequence_number;
    inspva.nov_header.time_status = currentNovatel.inspava_var.nov_header_var.time_status;
    inspva.nov_header.gps_week_number = currentNovatel.inspava_var.nov_header_var.gps_week_number;
    inspva.nov_header.gps_week_milliseconds = currentNovatel.inspava_var.nov_header_var.gps_week_milliseconds;
    inspva.nov_header.idle_time = currentNovatel.inspava_var.nov_header_var.idle_time;

    inspva.latitude = currentNovatel.inspava_var.latitude;
    inspva.longitude = currentNovatel.inspava_var.longitude;
    inspva.height = currentNovatel.inspava_var.height;
    inspva.north_velocity = currentNovatel.inspava_var.north_velocity;
    inspva.east_velocity = currentNovatel.inspava_var.east_velocity;
    inspva.up_velocity = currentNovatel.inspava_var.up_velocity;
    inspva.roll = currentNovatel.inspava_var.roll;
    inspva.pitch = currentNovatel.inspava_var.pitch;
    inspva.azimuth = currentNovatel.inspava_var.azimuth;

    inspva.status.status = currentNovatel.inspava_var.status_var.status_var;

    inspva.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      inspva.header.stamp.sec = this->sec;
      inspva.header.stamp.nanosec = this->nsec;
    }
    else
    {
      inspva.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      inspva.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (inspva.header.stamp.sec*1000000000);
    }

    this->novaTelInspvaPublisher->publish(inspva);

    // Heading 2
    auto heading2 = novatel_oem7_msgs::msg::HEADING2();

    heading2.nov_header.message_name = currentNovatel.heading_2_var.nov_header_var.message_name[0];
    heading2.nov_header.message_id = currentNovatel.heading_2_var.nov_header_var.message_id;
    heading2.nov_header.message_type = currentNovatel.heading_2_var.nov_header_var.message_type;
    heading2.nov_header.sequence_number = currentNovatel.heading_2_var.nov_header_var.sequence_number;
    heading2.nov_header.time_status = currentNovatel.heading_2_var.nov_header_var.time_status;
    heading2.nov_header.gps_week_number = currentNovatel.heading_2_var.nov_header_var.gps_week_number;
    heading2.nov_header.gps_week_milliseconds = currentNovatel.heading_2_var.nov_header_var.gps_week_milliseconds;
    heading2.nov_header.idle_time = currentNovatel.heading_2_var.nov_header_var.idle_time;

    heading2.sol_status.status = currentNovatel.heading_2_var.sol_status;

    heading2.pos_type.type = currentNovatel.heading_2_var.pos_type;

    heading2.length = currentNovatel.heading_2_var.length;
    heading2.heading = currentNovatel.heading_2_var.heading;
    heading2.pitch = currentNovatel.heading_2_var.pitch;
    heading2.reserved = currentNovatel.heading_2_var.reserved;
    heading2.heading_stdev = currentNovatel.heading_2_var.heading_stdev;
    heading2.pitch_stdev = currentNovatel.heading_2_var.pitch_stdev;
    heading2.rover_stn_id[0] = currentNovatel.heading_2_var.rover_stn_id[0];
    heading2.rover_stn_id[1] = currentNovatel.heading_2_var.rover_stn_id[1];
    heading2.rover_stn_id[2] = currentNovatel.heading_2_var.rover_stn_id[2];
    heading2.rover_stn_id[3] = currentNovatel.heading_2_var.rover_stn_id[3];
    heading2.master_stn_id[0] = currentNovatel.heading_2_var.master_stn_id[0];
    heading2.master_stn_id[1] = currentNovatel.heading_2_var.master_stn_id[1];
    heading2.master_stn_id[2] = currentNovatel.heading_2_var.master_stn_id[2];
    heading2.master_stn_id[3] = currentNovatel.heading_2_var.master_stn_id[3];
    heading2.num_sv_tracked = currentNovatel.heading_2_var.num_sv_tracked;
    heading2.num_sv_in_sol = currentNovatel.heading_2_var.num_sv_in_sol;
    heading2.num_sv_obs = currentNovatel.heading_2_var.num_sv_obs;
    heading2.num_sv_multi = currentNovatel.heading_2_var.num_sv_multi;
    heading2.sol_source.source = currentNovatel.heading_2_var.sol_source;
    heading2.ext_sol_status.status = currentNovatel.heading_2_var.ext_sol_status;
    heading2.galileo_beidou_sig_mask = currentNovatel.heading_2_var.galileo_beidou_sig_mask;
    heading2.gps_glonass_sig_mask = currentNovatel.heading_2_var.gps_glonass_sig_mask;

    heading2.header.frame_id = "world";

    if(this->simModeEnabled)
    {
      heading2.header.stamp.sec = this->sec;
      heading2.header.stamp.nanosec = this->nsec;
    }
    else
    {
      heading2.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      heading2.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (heading2.header.stamp.sec*1000000000);
    }

    this->novaTelHeading2Publisher->publish(heading2);

    // Raw IMU
    auto rawImu = novatel_oem7_msgs::msg::RAWIMU();

    rawImu.nov_header.message_name = currentNovatel.raw_imu_var.nov_header_var.message_name[0];
    rawImu.nov_header.message_id = currentNovatel.raw_imu_var.nov_header_var.message_id;
    rawImu.nov_header.message_type = currentNovatel.raw_imu_var.nov_header_var.message_type;
    rawImu.nov_header.sequence_number = currentNovatel.raw_imu_var.nov_header_var.sequence_number;
    rawImu.nov_header.time_status = currentNovatel.raw_imu_var.nov_header_var.time_status;
    rawImu.nov_header.gps_week_number = currentNovatel.raw_imu_var.nov_header_var.gps_week_number;
    rawImu.nov_header.gps_week_milliseconds = currentNovatel.raw_imu_var.nov_header_var.gps_week_milliseconds;
    rawImu.nov_header.idle_time = currentNovatel.raw_imu_var.nov_header_var.idle_time;

    rawImu.gnss_week = currentNovatel.raw_imu_var.gnss_week;
    rawImu.gnss_seconds = currentNovatel.raw_imu_var.gnss_seconds;
    rawImu.status = currentNovatel.raw_imu_var.status_var;

    rawImu.linear_acceleration.x = currentNovatel.raw_imu_var.linear_acceleration_var.x;
    rawImu.linear_acceleration.y = currentNovatel.raw_imu_var.linear_acceleration_var.y;
    rawImu.linear_acceleration.z = currentNovatel.raw_imu_var.linear_acceleration_var.z;

    rawImu.angular_velocity.x = currentNovatel.raw_imu_var.angular_velocity_var.x;
    rawImu.angular_velocity.y = currentNovatel.raw_imu_var.angular_velocity_var.y;
    rawImu.angular_velocity.z = currentNovatel.raw_imu_var.angular_velocity_var.z;

    rawImu.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      rawImu.header.stamp.sec = this->sec;
      rawImu.header.stamp.nanosec = this->nsec;
    }
    else
    {
      rawImu.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      rawImu.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (rawImu.header.stamp.sec*1000000000);
    }

    this->novaTelRawImuPublisher->publish(rawImu);

    // Raw IMUX
    auto rawImuX = sensor_msgs::msg::Imu();

    rawImuX.orientation.x = 0;
    rawImuX.orientation.y = 0;
    rawImuX.orientation.z = 0;
    rawImuX.orientation.w = 0;
    for (size_t i = 0; i < 9; i++) {rawImuX.orientation_covariance[i] = -1;}
    
    rawImuX.angular_velocity.x = currentNovatel.raw_imu_var.angular_velocity_var.x;
    rawImuX.angular_velocity.y = currentNovatel.raw_imu_var.angular_velocity_var.y;
    rawImuX.angular_velocity.z = currentNovatel.raw_imu_var.angular_velocity_var.z;
    for (size_t i = 0; i < 9; i++) {rawImuX.angular_velocity_covariance[i] = 0;}

    rawImuX.linear_acceleration.x = currentNovatel.raw_imu_var.linear_acceleration_var.x;
    rawImuX.linear_acceleration.y = currentNovatel.raw_imu_var.linear_acceleration_var.y;
    rawImuX.linear_acceleration.z = currentNovatel.raw_imu_var.linear_acceleration_var.z;
    for (size_t i = 0; i < 9; i++) {rawImuX.linear_acceleration_covariance[i] = 0;}

    rawImuX.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      rawImuX.header.stamp.sec = this->sec;
      rawImuX.header.stamp.nanosec = this->nsec;
    }
    else
    {
      rawImuX.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      rawImuX.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (rawImuX.header.stamp.sec*1000000000);
    }

    this->novaTelRawImuXPublisher->publish(rawImuX);
  }

  void AsmSocketCanBridgeNode::publishGroundTruthArray()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(get_logger(), "publishGroundTruthArray");

    auto groundTruthArray = autonoma_msgs::msg::GroundTruthArray();

    groundTruthArray.vehicles.resize(this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);

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
      RCLCPP_INFO(get_logger(), "publishGroundTruthArray1");

    bool groundTruthArrayFilled = false;

    for (size_t vehicleID = 0; vehicleID < this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count; vehicleID++)
    {
      RCLCPP_INFO(get_logger(), "publishGroundTruthArray2");
      groundTruthArray.vehicles[vehicleID].header.frame_id = groundTruthArray.header.frame_id;
      groundTruthArray.vehicles[vehicleID].header.stamp.sec = groundTruthArray.header.stamp.sec;
      groundTruthArray.vehicles[vehicleID].header.stamp.nanosec = groundTruthArray.header.stamp.nanosec;
      RCLCPP_INFO(get_logger(), "publishGroundTruthArray3");

      groundTruthArray.vehicles[vehicleID].car_num = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.car_num[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].lat = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[vehicleID];
      groundTruthArray.vehicles[vehicleID].lon = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[vehicleID];
      groundTruthArray.vehicles[vehicleID].hgt = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].vx = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vx[vehicleID];
      groundTruthArray.vehicles[vehicleID].vy = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vy[vehicleID];
      groundTruthArray.vehicles[vehicleID].vz = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vz[vehicleID];
      RCLCPP_INFO(get_logger(), "publishGroundTruthArray4");
      
      groundTruthArray.vehicles[vehicleID].yaw = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.yaw[vehicleID];
      groundTruthArray.vehicles[vehicleID].pitch = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.pitch[vehicleID];
      groundTruthArray.vehicles[vehicleID].roll = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.roll[vehicleID];
      RCLCPP_INFO(get_logger(), "publishGroundTruthArray5");
      
      groundTruthArray.vehicles[vehicleID].del_x = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_x[vehicleID];
      groundTruthArray.vehicles[vehicleID].del_y = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_y[vehicleID];
      groundTruthArray.vehicles[vehicleID].del_z = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_z[vehicleID];
      groundTruthArrayFilled = true;
    }
      RCLCPP_INFO(get_logger(), "publishGroundTruthArray6");
    
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
    
    RCLCPP_INFO(get_logger(), "publishGroundTruthArray7");
    this->groundTruthArrayPublisher_->publish(groundTruthArray);
    RCLCPP_INFO(get_logger(), "publishGroundTruthArray8");
  }

} // namespace asm_socketcan_bridge


int main(int argc, char * argv[])
{
  rclcpp::Node::SharedPtr AsmSocketCanBridgeNodePtr;
  try
  {
    rclcpp::init(argc, argv);

    AsmSocketCanBridgeNodePtr = std::make_shared<asm_socketcan_bridge::AsmSocketCanBridgeNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
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
