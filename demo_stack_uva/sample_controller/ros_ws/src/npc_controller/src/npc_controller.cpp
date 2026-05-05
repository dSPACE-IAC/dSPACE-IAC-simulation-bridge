#include <filesystem>
#include <stdlib.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "npc_controller.hpp"
#include <rclcpp/qos.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <libalglib/interpolation.h>

namespace {
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
  constexpr int32_t kCanWordBytes = 8;

  int32_t convert_to_mt_bit_ordering(uint32_t bit, uint32_t dlc = static_cast<uint32_t>(kCanWordBytes))
  {
    const int32_t message_bit_length = static_cast<int32_t>(dlc * 8U);
    const int32_t row = static_cast<int32_t>(bit / 8U);
    const int32_t offset = static_cast<int32_t>(bit % 8U);
    return (message_bit_length - (row + 1) * 8) + offset;
  }

  int32_t unpack_signal_bits(const uint8_t *data, const Signal &signal_information)
  {
    if (signal_information.length == 0) {
      return 0;
    }

    const bool little_endian = signal_information.endian != 0U;
    const int32_t length = static_cast<int32_t>(signal_information.length);

    int32_t start_bit = static_cast<int32_t>(signal_information.start_bit);
    if (little_endian) {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit);
    } else {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit) - (length - 1);
    }

    const int32_t bit = start_bit % 8;
    const bool is_exactly_byte = ((bit + length) % 8) == 0;
    const uint32_t num_bytes =
      static_cast<uint32_t>((is_exactly_byte ? 0 : 1) + ((bit + length) / 8));

    int32_t byte_index = kCanWordBytes - (start_bit / 8) - 1;
    int32_t bits_remaining = length;
    int32_t mask_shift = bit;
    int32_t right_shift = 0;

    uint32_t unsigned_result = 0;
    for (uint32_t i = 0; i < num_bytes; ++i) {
      if (byte_index < 0 || byte_index >= kCanWordBytes) {
        return 0;
      }

      int32_t mask = 0xFF;
      if (bits_remaining < 8) {
        mask >>= (8 - bits_remaining);
      }
      mask <<= mask_shift;

      const int32_t extracted_byte = (data[byte_index] & mask) >> mask_shift;
      unsigned_result |=
        static_cast<uint32_t>(extracted_byte) << (8 * static_cast<int32_t>(i) - right_shift);

      if (!little_endian) {
        if ((byte_index % kCanWordBytes) == 0) {
          byte_index += 2 * kCanWordBytes - 1;
        } else {
          --byte_index;
        }
      } else {
        ++byte_index;
      }

      bits_remaining -= (8 - mask_shift);
      right_shift += mask_shift;
      mask_shift = 0;
    }

    if (signal_information.is_signed) {
      const int32_t sign_index = length - 1;
      if (sign_index >= 0 && sign_index < 32 &&
        (unsigned_result & (1U << sign_index)) != 0U)
      {
        if (length < 32) {
          unsigned_result |= (0xFFFFFFFFU << length);
        }
      }
      return static_cast<int32_t>(unsigned_result);
    }

    return static_cast<int32_t>(unsigned_result);
  }

  void pack_signal_bits(uint8_t *data, const Signal &signal_information, uint64_t raw_value)
  {
    if (signal_information.length == 0) {
      return;
    }

    const bool little_endian = signal_information.endian != 0U;
    const int32_t length = static_cast<int32_t>(signal_information.length);

    int32_t start_bit = static_cast<int32_t>(signal_information.start_bit);
    if (little_endian) {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit);
    } else {
      start_bit = convert_to_mt_bit_ordering(signal_information.start_bit) - (length - 1);
    }

    const int32_t bit = start_bit % 8;
    const bool is_exactly_byte = ((bit + length) % 8) == 0;
    const uint32_t num_bytes =
      static_cast<uint32_t>((is_exactly_byte ? 0 : 1) + ((bit + length) / 8));

    int32_t byte_index = kCanWordBytes - (start_bit / 8) - 1;
    int32_t bits_remaining = length;
    int32_t mask_shift = bit;
    int32_t right_shift = 0;

    for (uint32_t i = 0; i < num_bytes; ++i) {
      if (byte_index < 0 || byte_index >= kCanWordBytes) {
        return;
      }

      uint8_t mask = 0xFF;
      if (bits_remaining < 8) {
        mask >>= (8 - bits_remaining);
      }
      mask = static_cast<uint8_t>(mask << mask_shift);

      const uint64_t extracted_byte =
        (raw_value >> (8 * static_cast<int32_t>(i) - right_shift)) & 0xFFULL;

      data[byte_index] = static_cast<uint8_t>(data[byte_index] & ~mask);
      data[byte_index] |= static_cast<uint8_t>((extracted_byte << mask_shift) & mask);

      if (!little_endian) {
        if ((byte_index % kCanWordBytes) == 0) {
          byte_index += 2 * kCanWordBytes - 1;
        } else {
          --byte_index;
        }
      } else {
        ++byte_index;
      }

      bits_remaining -= (8 - mask_shift);
      right_shift += mask_shift;
      mask_shift = 0;
    }
  }

}  // namespace


namespace controller
{
    ControllerNode::ControllerNode(const rclcpp::NodeOptions &options)
        : Node("npc_controller", options)
    {

        const auto history_size{declare_parameter("qos_history", 1)};
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(history_size), rmw_qos_profile_iac);
        const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(history_size), rmw_qos_profile_sim_clock);

        if (std::getenv("SIM_CLOCK_MODE"))
        {
            if (std::string(std::getenv("SIM_CLOCK_MODE")) == "true")
                this->simModeEnabled = true;
            else
                this->simModeEnabled = false;
        }
        else
        {
            this->simModeEnabled = false;
        }
        if (std::getenv("CAR_NUM")){
            veh_num = std::stoi(std::getenv("CAR_NUM"));
        } else {
            veh_num = 42;
        }

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
        this->publish_ros_all = this->declare_parameter<bool>(
        "logging.publish_ros_all",
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

        this->useRaptorDbwNode = this->declare_parameter<bool>(
            "connection.useRaptorDbwNode",
            false);
        if (this->useRaptorDbwNode) {
            RCLCPP_INFO(this->get_logger(), "Raptor DBW node is used. Direct CAN communication is disabled");
        } else {
            RCLCPP_INFO(this->get_logger(), "Direct CAN communication is enabled");
        }
        this->disableStateMachine = this->declare_parameter<bool>(
            "disableStateMachine",
            false);
        if (this->disableStateMachine) {
            RCLCPP_INFO(this->get_logger(), "Statemachine disabled. System will directly proceed to ct_state 8.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Statemachine enabled. System will wait for red flag to be initiated.");
        }
            
        // Initialize subscribers.
        // bestpos_sub_ = create_subscription<novatel_oem7_msgs::msg::BESTPOS>("/novatel_bottom/bestpos", qos, std::bind(&ControllerNode::bestpos_callback, this, std::placeholders::_1));
        bestpos_sub_ = create_subscription<novatel_oem7_msgs::msg::BESTPOS>("bestpos", qos, std::bind(&ControllerNode::bestpos_callback, this, std::placeholders::_1));
        if (this->useRaptorDbwNode) {
            wheel_speed_sub_ = create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>("wheel_speed_report", qos, std::bind(&ControllerNode::wheel_speed_callback_ros_msg, this, std::placeholders::_1));
            pt_report_sub_ = create_subscription<npc_controller_msgs::msg::PtReport>("pt_report", qos, std::bind(&ControllerNode::receivePtReport_ros_msg, this, std::placeholders::_1));
            sys_state_sub_ = this->create_subscription<npc_controller_msgs::msg::MiscReport>("raptor_dbw_interface/misc_report_do", 1, std::bind(&ControllerNode::receiveSysState_ros_msg, this, std::placeholders::_1));
            flags_sub_ = this->create_subscription<npc_controller_msgs::msg::RcToCt>("raptor_dbw_interface/rc_to_ct", 1, std::bind(&ControllerNode::receiveFlags_ros_msg, this, std::placeholders::_1));
        }
        ct_input_sub_ = this->create_subscription<std_msgs::msg::Int32>("ct_input", qos, std::bind(&ControllerNode::receiveCtInput, this, std::placeholders::_1));

        if(this->simModeEnabled)
            simClockTime_ = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", sim_qos, std::bind(&ControllerNode::simClockTimeCallback, this, std::placeholders::_1));

        // Load Track Paths
        std::string track_name = declare_parameter("track_name", "");
        std::filesystem::path share_dir = std::filesystem::path(ament_index_cpp::get_package_share_directory("npc_controller"));
        std::filesystem::path map_dir = share_dir /
                                        std::filesystem::path("maps") /
                                        std::filesystem::path(track_name);
        std::filesystem::path pit_line_dir =
            map_dir / map_dir / std::filesystem::path("pit_lane.csv");
        pit_line_ = load_path(pit_line_dir.string());
        std::filesystem::path center_line_dir =
            map_dir / map_dir / std::filesystem::path("center_line.csv");
        center_line_ = load_path(center_line_dir.string());
        path_loaded = true;

        current_path_ = &pit_line_;
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
        register_timer("pure_pursuit_timer", [this]() { this->pure_pursuit(); });
        register_timer("long_control_timer", [this]() { this->long_control(); });
        register_timer("control_timer", [this]() { this->lateral_control(); });
        register_timer("state_machine_timer", [this]() { this->state_machine(); });

        // Set time increase step to 10 ms
        sim_time_increase_msg_.data = 10;

        // Initialize publishers.
        vehicle_cmd_pub_ = this->create_publisher<autonoma_msgs::msg::VehicleInputs>("vehicle_inputs", qos);
        ct_report_pub_ = this->create_publisher<autonoma_msgs::msg::ToRaptor>("to_raptor", qos);
        sim_time_increase_pub_ = this->create_publisher<std_msgs::msg::UInt16>("sim_time_increase", sim_qos);
        steering_cmd_pub_ = create_publisher<raptor_dbw_msgs::msg::SteeringCmd>("steering_cmd", 1);
        gear_cmd_pub_ = create_publisher<std_msgs::msg::UInt8>("gear_cmd", 1);
        throttle_cmd_pub_ = create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>("accelerator_pedal_cmd", 1);
        brake_cmd_pub_ = create_publisher<raptor_dbw_msgs::msg::BrakeCmd>("brake_cmd", 1);
        npc_ct_report_pub_ = this->create_publisher<npc_controller_msgs::msg::CtReport>("raptor_dbw_interface/ct_report",10);

        // Debug Publishers
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
        target_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_point", qos);
        base_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("base_point", qos);
        debug_pub_ = this->create_publisher<npc_controller_msgs::msg::NPCDebug>("debug", qos);

        if(this->simModeEnabled)
        {
            prev_time_ = double(this->sec) + double(this->nsec) * 1e-9;
            prev_vel_time_ = double(this->sec) + double(this->nsec) * 1e-9;
            prev_acc_time_ = double(this->sec) + double(this->nsec) * 1e-9;
        }
        else
        {       
            prev_time_ = this->now().seconds() + this->now().nanoseconds() * 1e-9;
            prev_vel_time_ = this->now().seconds() + this->now().nanoseconds() * 1e-9;
            prev_acc_time_ = this->now().seconds() + this->now().nanoseconds() * 1e-9;
        }

        // Declare Parameters
        declare_parameter("vehicle.wheelbase", 2.971);
        declare_parameter("vehicle.steering_ratio", 15.0);
        declare_parameter("vehicle.min_steer", -0.2793);
        declare_parameter("vehicle.max_steer", 0.2793);
        declare_parameter("vehicle.min_lookahead_dist", 15.0);
        declare_parameter("vehicle.max_lookahead_dist", 45.0);
        declare_parameter("vehicle.lookahead_gain", 0.8);

        declare_parameter("vehicle.min_throttle", 0.0);
        declare_parameter("vehicle.max_throttle", 100.0);
        declare_parameter("vehicle.min_brake", 0.0);
        declare_parameter("vehicle.max_brake", 5500.0);
        declare_parameter("vehicle.max_acc", 5.5);
        declare_parameter("vehicle.min_acc", -6.0);
        declare_parameter("vehicle.vel_kp", 0.2);
        declare_parameter("vehicle.vel_ki", 0.005);
        declare_parameter("vehicle.vel_kd", 0.08);
        declare_parameter("vehicle.throttle_kp", 0.03);
        declare_parameter("vehicle.throttle_ki", 0.01);
        declare_parameter("vehicle.throttle_kd", 0.05);
        declare_parameter("vehicle.braking_kp", 0.002);
        declare_parameter("vehicle.braking_ki", 0.0001);
        declare_parameter("vehicle.braking_kd", 0.0001);

        declare_parameter("lap_state_machine.yellow_flag_box_exit", 18.0);
        declare_parameter("lap_state_machine.yellow_flag_pit_row", 30.0);
        declare_parameter("lap_state_machine.yellow_flag_pit_lane", 30.0);
        declare_parameter("lap_state_machine.yellow_flag_on_track", 60.0);
        declare_parameter("lap_state_machine.green_flag", 60.0);
        declare_parameter("lap_state_machine.black_flag", 50.0);
        declare_parameter("lap_state_machine.pit_transition_out", 1055.3165683121185);
        declare_parameter("lap_state_machine.pit_transition_in", 1093.7091990052456);
        declare_parameter("lap_state_machine.pit_entry_dec", 993.7091990052456);
        declare_parameter("lap_state_machine.pit_inc_speed", 160.21269895019626);
        declare_parameter("lap_state_machine.pit_dec_speed", 2003.823017429067);
        declare_parameter("lap_state_machine.pit_stop", 115.29526969674424);

        // Load Parameters
        wheelbase_ = get_parameter("vehicle.wheelbase").as_double();
        steering_ratio_ = get_parameter("vehicle.steering_ratio").as_double();
        min_steer_ = get_parameter("vehicle.min_steer").as_double();
        max_steer_ = get_parameter("vehicle.max_steer").as_double();
        min_lookahead_dist_ = get_parameter("vehicle.min_lookahead_dist").as_double();
        max_lookahead_dist_ = get_parameter("vehicle.max_lookahead_dist").as_double();
        lookahead_gain_ = get_parameter("vehicle.lookahead_gain").as_double();

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

        // Initialize Lap State Machine Parameters
        LapStateSpeeds speeds;
        speeds.yellow_flag_box_exit = get_parameter("lap_state_machine.yellow_flag_box_exit").as_double();
        speeds.yellow_flag_pit_row = get_parameter("lap_state_machine.yellow_flag_pit_row").as_double();
        speeds.yellow_flag_pit_lane = get_parameter("lap_state_machine.yellow_flag_pit_lane").as_double();
        speeds.yellow_flag_on_track = get_parameter("lap_state_machine.yellow_flag_on_track").as_double();
        speeds.green_flag = get_parameter("lap_state_machine.green_flag").as_double();
        speeds.black_flag = get_parameter("lap_state_machine.black_flag").as_double();
        LapStateLocs locs;
        locs.pit_transition_out_loc = get_parameter("lap_state_machine.pit_transition_out").as_double();
        locs.pit_transition_in_loc = get_parameter("lap_state_machine.pit_transition_in").as_double();
        locs.pit_entry_dec_loc = get_parameter("lap_state_machine.pit_entry_dec").as_double();
        locs.pit_inc_speed_loc = get_parameter("lap_state_machine.pit_inc_speed").as_double();
        locs.pit_dec_speed_loc = get_parameter("lap_state_machine.pit_dec_speed").as_double();
        locs.pit_stop = get_parameter("lap_state_machine.pit_stop").as_double();

        lap_state_inputs_.speeds = speeds;
        lap_state_inputs_.locs = locs;

        if (!this->useRaptorDbwNode) {
            const int16_t default_max_retries = 1;
            const int64_t raw_max_retries = this->declare_parameter<int64_t>("connection.max_retries", static_cast<int64_t>(default_max_retries));
            max_retries = sanitize_retry_value(raw_max_retries, default_max_retries, this->get_logger(), "Maximum connection retries");
            RCLCPP_INFO(this->get_logger(), "Maximum connection retries: %d", static_cast<int>(max_retries));

            auto pkg_share = ament_index_cpp::get_package_share_directory("npc_controller");
            can1_dbc_path = this->declare_parameter<std::string>("can.can1_dbc_path", pkg_share + "/config/CAN1-INDY-V26.dbc");
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
        }
    }

    // general utilities
    int ControllerNode::open_socket(const std::string &iface)
    {
        RCLCPP_INFO(get_logger(), "Socket open...");
        int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        RCLCPP_INFO(get_logger(), "CAN socket opened fd=%d", sock);
        if (sock < 0) {
            RCLCPP_ERROR(get_logger(),
                        "socket() failed for interface %s: %s",
                        iface.c_str(), strerror(errno));
            return -1;
        }

        struct ifreq ifr {};
        strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(get_logger(),
                        "ioctl(SIOCGIFINDEX) failed for %s: %s",
                        iface.c_str(), strerror(errno));
            close(sock);
            return -1;
        }

        RCLCPP_INFO(get_logger(), "Socket bind...");
        struct sockaddr_can addr {};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(),
                        "bind() failed for %s: %s",
                        iface.c_str(), strerror(errno));
            close(sock);
            return -1;
        }

        RCLCPP_INFO(get_logger(), "CAN socket bound fd=%d", sock);
        return sock;
    }
    
    void ControllerNode::buildMessageLookup()
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

    const Message* ControllerNode::findMessageByID(uint32_t message_id) const
    {
        const auto iter = message_lookup_.find(message_id);
        if (iter == message_lookup_.end()) {
        return nullptr;
        }
        return iter->second;
    }

    const ControllerNode::SignalLookupMap* ControllerNode::findSignalLookup(uint32_t message_id) const
    {
        const auto iter = message_signal_lookup_.find(message_id);
        if (iter == message_signal_lookup_.end()) {
        return nullptr;
        }
        return &iter->second;
    }

    const Signal* ControllerNode::findSignal(uint32_t message_id, std::string_view signal_name) const
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

    const Message* ControllerNode::findMessageByName(std::string_view message_name) const
    {
        const auto iter = message_name_lookup_.find(message_name);
        if (iter == message_name_lookup_.end()) {
        return nullptr;
        }
        return iter->second;
    }


    // can read
    void ControllerNode::can_reader_loop(int sock, const std::string &bus_id)
    {
        RCLCPP_INFO(this->get_logger(), "Can reader loop started for %s", bus_id.c_str());
        struct can_frame in_frame{};
        while (rclcpp::ok() && !stop_reader_.load()) {
            if (this->receivedMessagePrinting) RCLCPP_INFO(this->get_logger(), "Can reader loop...");

            RCLCPP_DEBUG(get_logger(), "About to read CAN fd=%d", sock);
            int nbytes = read(sock, &in_frame, sizeof(in_frame));
            if (nbytes < 0) {
                if (stop_reader_.load()) break;

                if (errno == EINTR) continue;
                RCLCPP_ERROR(get_logger(), "CAN read failed: %s (sock=%d)", strerror(errno), sock);
                
                if (errno == EBADF) {
                    RCLCPP_ERROR(get_logger(), "Socket became invalid (EBADF). Exiting reader loop.");
                    break;
                }
                continue;
            }
            if (nbytes == 0) {
                continue;
            }

            if (this->receivedMessagePrinting)
                RCLCPP_INFO(this->get_logger(), "received: 0x%03X [%d] ",in_frame.can_id, in_frame.can_dlc);
            for (int i = 0; i < in_frame.can_dlc; i++) {
                if (this->receivedMessagePrinting)
                RCLCPP_INFO(this->get_logger(), "received: %02X ",in_frame.data[i]);
            }

            const auto warn_missing_metadata = [&](uint32_t message_id) {
                if (this->verbosePrinting) {
                RCLCPP_WARN(this->get_logger(),
                            "No metadata for CAN ID %u; dropping frame.",
                            message_id);
                } else {
                RCLCPP_WARN_ONCE(this->get_logger(),
                                "No metadata for CAN ID %u; dropping frame.",
                                message_id);
                }
            };

            switch (in_frame.can_id) {
                case 1300: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "wheel_speed_report");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("wheel_speed_RL")) {
                        this->ws_rear_left =static_cast<float>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("wheel_speed_FR")) {
                        this->ws_front_right =static_cast<float>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("wheel_speed_FL")) {
                        this->ws_front_left =static_cast<float>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("wheel_speed_RR")) {
                        this->ws_rear_right =static_cast<float>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "wheel_speed_RL: %f  wheel_speed_FR: %f  wheel_speed_FL: %f  wheel_speed_RR: %f",
                                    this->ws_rear_left,
                                    this->ws_front_right,
                                    this->ws_front_left,
                                    this->ws_rear_right);
                    }
                    this->wheel_speed_callback();
                    break;
                }
                case 1340: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "pt_report_1");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("throttle_position")) {
                        this->throttle_position =
                        static_cast<float>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("current_gear")) {
                        this->current_gear =
                        static_cast<int8_t>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("engine_speed_rpm")) {
                        this->engine_rpm =
                        static_cast<float>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "throttle_position: %f  current_gear: %d  engine_speed_rpm: %f",
                                    this->throttle_position,
                                    this->current_gear,
                                    this->engine_rpm);
                    }
                    this->receivePtReport();
                    break;
                }
                case 1250: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "marelli_report_1");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("marelli_track_flag")) {
                        this->track_flag =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (const auto value = get_scaled("marelli_vehicle_flag")) {
                        this->veh_flag =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "marelli_track_flag: %d  marelli_vehicle_flag: %d",
                                    this->track_flag,
                                    this->veh_flag);
                    }
                    this->receiveFlags();
                    break;
                }
                case 1200: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "base_to_car_summary");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("round_target_speed")) {
                        this->round_target_speed =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(),
                                    "round_target_speed: %d",
                                    this->round_target_speed);
                    }
                    this->receiveFlags();
                    break;
                }
                case 1304: {
                    if (this->receivedMessagePrinting)
                        RCLCPP_INFO(this->get_logger(), "misc_report");
                    std::lock_guard<std::mutex> lock(feedback_mutex_);
                    if (!findMessageByID(in_frame.can_id)) {
                        warn_missing_metadata(in_frame.can_id);
                        break;
                    }
                    const auto get_scaled = [&](std::string_view name) {
                        return extractSignalScaled(in_frame.can_id, name, in_frame.data);
                    };
                    if (const auto value = get_scaled("sys_state")) {
                        this->sys_state =
                        static_cast<uint8_t>(std::floor(*value));
                    }
                    if (this->receivedDecodedMessagePrinting) {
                        RCLCPP_INFO(this->get_logger(), "sys_state: %d", this->sys_state);
                    }
                    this->receiveFlags();
                    break;
                }
            }
        }
    }

    int32_t ControllerNode::extractBits(const uint8_t* data, Signal signal_information) const
    {
        return unpack_signal_bits(data, signal_information);
    }

    std::optional<double>
    ControllerNode::extractSignalScaled(uint32_t message_id, std::string_view signal_name, const uint8_t* data) const
    {
        const auto *signal = findSignal(message_id, signal_name);
        if (!signal) {
        return std::nullopt;
        }
        const auto raw = extractBits(data, *signal);
        return static_cast<double>(raw) * signal->factor + signal->offset;
    }


    // Callbacks
    void ControllerNode::simClockTimeCallback(const rosgraph_msgs::msg::Clock &msg)
    {
        this->sec = msg.clock.sec;
        this->nsec = msg.clock.nanosec;
        if (msg.clock.sec != 0 && msg.clock.nanosec != 0)
        {
            pure_pursuit();
            long_control();
            lateral_control();
            state_machine();
        }
        sim_time_increase_pub_->publish(sim_time_increase_msg_);
    }

    void ControllerNode::bestpos_callback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
    {
        double lat = msg->lat;
        double lon = msg->lon;
        double height = msg->hgt;
        double x, y, z;
        gps_map_.Forward(lat, lon, height, x, y, z);

        // Calculate Heading from Position Difference
        double dx = x - prev_x_;
        double dy = y - prev_y_;
        vehicle_state_.yaw = std::atan2(dy, dx);
        prev_x_ = x;
        prev_y_ = y;

        // Convert from Antenna to Rear Axle of Vehicle
        vehicle_state_.x = x - 3.175 * std::cos(vehicle_state_.yaw);
        vehicle_state_.y = y - 3.175 * std::sin(vehicle_state_.yaw);
        vehicle_state_.z = z;

        previous_state_ = vehicle_state_;
        RCLCPP_WARN_ONCE(get_logger(),"position_received");
        position_received = true;

        // Publish Odometry
        odometry_msg_.header.stamp = this->now();
        odometry_msg_.header.frame_id = "map";
        odometry_msg_.child_frame_id = "base_link";
        odometry_msg_.pose.pose.position.x = vehicle_state_.x;
        odometry_msg_.pose.pose.position.y = vehicle_state_.y;
        odometry_msg_.pose.pose.position.z = vehicle_state_.z;

        double cy = std::cos(vehicle_state_.yaw * 0.5);
        double sy = std::sin(vehicle_state_.yaw * 0.5);

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

        double current_time;
        if(this->simModeEnabled) {current_time = double(this->sec) + double(this->nsec) * 1e-9;}
        else {current_time = this->now().seconds() + this->now().nanoseconds() * 1e-9;}

        double dt = current_time - prev_time_;
        double accel = vel_filter_.processSample((avg_ws - previous_state_.vx) / dt);
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
        double fl = msg->front_left;
        double fr = msg->front_right;
        double rl = msg->rear_left;
        double rr = msg->rear_right;
        double avg_ws = (fl + fr + rl + rr) / 4.0 / 3.6;

        double current_time;
        if(this->simModeEnabled) {current_time = double(this->sec) + double(this->nsec) * 1e-9;}
        else {current_time = this->now().seconds() + this->now().nanoseconds() * 1e-9;}

        double dt = current_time - prev_time_;
        double accel = vel_filter_.processSample((avg_ws - previous_state_.vx) / dt);
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
        track_flag_ = int2tf(msg->track_flag);
        vehicle_flag_ = int2vf(msg->veh_flag);
        target_speed_ = msg->target_speed;
    }

    void ControllerNode::receiveSysState_ros_msg(const npc_controller_msgs::msg::MiscReport::SharedPtr msg)
    {
        sys_state_ = int2sys(msg->sys_state);
    }

    void ControllerNode::receivePtReport_ros_msg(const npc_controller_msgs::msg::PtReport::SharedPtr msg) {
        current_gear_ = msg->current_gear;
        engine_speed_ = msg->engine_rpm;
        engine_running_ = bool( msg->engine_rpm > 500 );
        reported_throttle_ = msg->throttle_position;
        engine_braking_decel = ((engine_speed_ > 1300 && vehicle_state_.vx > 5.0 && reported_throttle_ < 5.0) ? (30 * GEAR_RATIOS[current_gear_] * FINAL_DRIVE_RATIO) : 0.0);
    }

    // can send    
    void ControllerNode::can_write(int sock, const struct can_frame &frame)
    {
        if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(frame)) {
        perror("Write");
        return;
        }
    }

    void ControllerNode::finalizeCanMessage(const PreparedCanMessage &message)
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

    std::optional<ControllerNode::PreparedCanMessage>
    ControllerNode::prepareCanMessage(std::string_view message_name)
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

    template <typename T>
    void ControllerNode::insertBits(uint8_t* data, Signal signal_information, T physical_value)
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

        pack_signal_bits(data, signal_information, raw_value);
    }

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

        vehicle_cmd_msg_.brake_cmd_front = brake_cmd_front;
        vehicle_cmd_msg_.brake_cmd_rear = brake_cmd_rear;
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
        debug_msg_.output_throttle = vehicle_state_.throttle;
        debug_msg_.output_brake = vehicle_state_.brake;
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
        min_steer_ = get_parameter("vehicle.min_steer").as_double();
        max_steer_ = get_parameter("vehicle.max_steer").as_double();
        min_lookahead_dist_ = get_parameter("vehicle.min_lookahead_dist").as_double();
        max_lookahead_dist_ = get_parameter("vehicle.max_lookahead_dist").as_double();
        lookahead_gain_ = get_parameter("vehicle.lookahead_gain").as_double();

        // Saturate and Translate Wheel Angle to Steering Wheel Angle
        double steering_angle = std::max(min_steer_, std::min(pure_pursuit_steering_angle, max_steer_));
        double steering_cmd = steering_angle * (180.0 / M_PI) * steering_ratio_;
        if(this->sys_state!=9){steering_cmd = 0;}
        vehicle_cmd_msg_.steering_cmd = steering_cmd;
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

    void ControllerNode::state_machine()
    {
        if (!position_received || !wheel_speed_received || !path_loaded)
        {
            // RCLCPP_INFO(this->get_logger(),"position_received: %d  wheel_speed_received: %d  path_loaded: %d",
            //                                 position_received,
            //                                 wheel_speed_received,
            //                                 path_loaded);
            return;
        }
        // Calculate Path Base Projections
        PathPoint current_position;
        current_position.x = vehicle_state_.x;
        current_position.y = vehicle_state_.y;
        current_position.z = vehicle_state_.z;
        current_position.yaw = vehicle_state_.yaw;

        // TODO: Adjust for multiple static lines
        int center_bp = calculate_base_projections(center_line_, current_position);
        center_line_s_ = center_line_.points[center_bp].s;
        int pit_bp = calculate_base_projections(pit_line_, current_position);
        pit_line_s_ = pit_line_.points[pit_bp].s;

        // Process Drive by Wire State Machine
        ct_state_ = dbw_state_machine(ct_state_, track_flag_, vehicle_flag_, sys_state_, estop_, ct_input_, vehicle_state_.vx, disableStateMachine);

        // Process Lap State Machine
        lap_state_inputs_.ct_state = ct_state_;
        lap_state_inputs_.vehicle_flag = vehicle_flag_;
        lap_state_inputs_.track_flag = track_flag_;
        lap_state_inputs_.target_speed = target_speed_;
        lap_state_inputs_.current_speed = vehicle_state_.vx;
        lap_state_inputs_.center_line_s = center_line_s_;
        lap_state_inputs_.pit_lane_s = pit_line_s_;

        lap_state_machine_.transition(lap_state_inputs_, lap_state_outputs_);

        desired_velocity_ = lap_state_outputs_.des_vel / 2.23694; // Convert to from MPH to m/s
        if (lap_state_outputs_.path == "pits")
        {
            current_path_ = &pit_line_;
        }
        else
        {
            current_path_ = &center_line_;
        }

        // Publish CT Report
        ct_report_msg_.ct_state = static_cast<int>(ct_state_);
        ct_report_msg_.header.stamp = this->now();

        ct_report_msg_.track_cond_ack = static_cast<int>(track_flag_);
        ct_report_msg_.veh_sig_ack = static_cast<int>(vehicle_flag_);
        ct_report_msg_.rolling_counter = ct_counter_;
        ct_report_msg_.veh_num = veh_num;
        push2pass_counter = (push2pass_counter + 1) % 500;
        if (push2pass_counter == 42)
        {
            if (current_push2pass_request)
            {
                current_push2pass_request = false;
            }
            else
            {
                current_push2pass_request = true;
            }
            // current_push2pass_request = true;
        }
        
        ct_report_msg_.push2pass_request = current_push2pass_request;
        ct_report_pub_->publish(ct_report_msg_);
        ct_counter_ = (ct_counter_ + 1) % 8;

        if (!this->useRaptorDbwNode) {
            publishCanMessage("ct_report", [&](PreparedCanMessage &message) {
                auto assign = [&](std::string_view signal_name, auto value) {
                    if (const auto *signal = findSignal(message.metadata->id, signal_name)) {
                        insertBits(message.frame.data, *signal, value);
                    }
                };
                assign("track_cond_ack", static_cast<int>(track_flag_));
                assign("veh_sig_ack", static_cast<int>(vehicle_flag_));
                assign("ct_state", static_cast<int>(ct_state_));
                assign("ct_state_rolling_counter", ct_counter_);
                assign("veh_num", veh_num);
            });
        } else if (this->useRaptorDbwNode || this->publish_ros_all) {
            // Publish CT Report
            npc_ct_report_msg_.ct_state = static_cast<int>(ct_state_);
            npc_ct_report_msg_.track_flag_ack = static_cast<int>(track_flag_);
            npc_ct_report_msg_.veh_flag_ack = static_cast<int>(vehicle_flag_);
            npc_ct_report_msg_.target_speed = target_speed_;
            npc_ct_report_msg_.rolling_counter = ct_counter_;
            npc_ct_report_pub_->publish(npc_ct_report_msg_);
        }

        // Populate Debug Message
        debug_msg_.ct_state = static_cast<int>(ct_state_);
        debug_msg_.track_flag = static_cast<int>(track_flag_);
        debug_msg_.vehicle_flag = static_cast<int>(vehicle_flag_);
        debug_msg_.sys_state = static_cast<int>(sys_state_);
        debug_msg_.track_loc = static_cast<int>(lap_state_inputs_.track_loc);
        debug_msg_.lap_state = static_cast<int>(lap_state_inputs_.lap_state);
        debug_msg_.center_s = center_line_s_;
        debug_msg_.pit_s = pit_line_s_;
    }

    // control logic
    void ControllerNode::pure_pursuit()
    {
        /**
         * @brief This function computes the steering angle using the pure pursuit algorithm
         * @return The steering angle in radians
         */

        // Check to see if we have enough localization data.
        if (!wheel_speed_received || !position_received || !path_loaded) {return;}

        PathPoint current_position;
        current_position.x = vehicle_state_.x;
        current_position.y = vehicle_state_.y;
        current_position.z = vehicle_state_.z;
        current_position.yaw = vehicle_state_.yaw;

        int start_index = calculate_base_projections(*current_path_, current_position);

        double current_velocity = vehicle_state_.vx;
        double lookahead = std::max(min_lookahead_dist_, std::min(lookahead_gain_ * current_velocity, max_lookahead_dist_));

        PathPoint target_position = pure_pursuit_target_point(*current_path_, start_index, current_position, lookahead);

        double pursuit_vector_dx = target_position.x - current_position.x;
        double pursuit_vector_dy = target_position.y - current_position.y;

        double alpha = atan2(pursuit_vector_dy, pursuit_vector_dx) - vehicle_state_.yaw;
        double delta = atan((2 * wheelbase_ * sin(alpha)) / lookahead);

        // Update the steering angle.
        pure_pursuit_steering_angle = delta;

        // Visualize the pure pursuit base and target points
        base_point_msg_.header.stamp = this->now();
        base_point_msg_.header.frame_id = "map";
        base_point_msg_.point.x = current_path_->points[start_index].x;
        base_point_msg_.point.y = current_path_->points[start_index].y;

        target_point_msg_.header.stamp = this->now();
        target_point_msg_.header.frame_id = "map";
        target_point_msg_.point.x = target_position.x;
        target_point_msg_.point.y = target_position.y;

        base_point_pub_->publish(base_point_msg_);
        target_point_pub_->publish(target_point_msg_);
    }

    int ControllerNode::calculate_base_projections(const Path &path, const PathPoint &current_position)
    {
        // Calculate start index. Find the closest point to us.
        int start_index = 0;
        double best_distance = 1000000;
        for (unsigned int i = 0; i < path.points.size(); i++)
        {
            double dx = path.points[i].x - current_position.x;
            double dy = path.points[i].y - current_position.y;
            double distance = sqrt(dx * dx + dy * dy);
            if (distance < best_distance)
            {
                best_distance = distance;
                start_index = i;
            }
        }
        return start_index;
    }

    double ControllerNode::calc_acceleration(double set_point)
    {
        // removes oldest instant vel error from vector
        if (vel_error_arr_.size() >= 10)
        {
            vel_integral_error_ -= vel_error_arr_[0];
            vel_error_arr_.erase(vel_error_arr_.begin());
        }

        // Calculate Acceleration Required using PID
        vel_integral_error_ += prev_vel_error_ / 10.0;
        vel_error_arr_.push_back(prev_vel_error_ / 10.0);

        double vel_error = set_point - vehicle_state_.vx;
        double dt;
        if(this->simModeEnabled)
            dt = double(this->sec) + double(this->nsec) * 1e-9 - prev_vel_time_;
        else
            dt = this->now().seconds() + this->now().nanoseconds() * 1e-9 - prev_vel_time_;
        double vel_derivative_error_ = (vel_error - prev_vel_error_) / dt;

        double des_accel = (vel_kp_ * vel_error + vel_ki_ * vel_integral_error_ + vel_kd_ * vel_derivative_error_);
        prev_vel_error_ = vel_error;
        if(this->simModeEnabled)
            prev_vel_time_ = double(this->sec) + double(this->nsec) * 1e-9;
        else
            prev_vel_time_ = this->now().seconds() + this->now().nanoseconds() * 1e-9;
        // removes oldest instant vel error from vector
        if (acc_error_arr_.size() >= 10)
        {
            acc_integral_error_ -= acc_error_arr_[0];
            acc_error_arr_.erase(acc_error_arr_.begin());
        }

        // Calculate Throttle using PID
        acc_integral_error_ += prev_acc_error_ / double(10);
        acc_error_arr_.push_back(prev_acc_error_ / double(10));

        acc_error_ = des_accel - vehicle_state_.ax;
        if(this->simModeEnabled)
            dt = double(this->sec) + double(this->nsec) * 1e-9 - prev_acc_time_;
        else
            dt = this->now().seconds() + this->now().nanoseconds() * 1e-9 - prev_acc_time_;

        acc_derivative_error_ = (acc_error_ - prev_acc_error_) / dt;

        prev_acc_error_ = acc_error_;

        if(this->simModeEnabled)
            prev_acc_time_ = double(this->sec) + double(this->nsec) * 1e-9;
        else
            prev_acc_time_ = this->now().seconds() + this->now().nanoseconds() * 1e-9;

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

    void ControllerNode::calc_throttle(double desired_acceleration)
    {

        // Calculate Deadband
        double db = -non_brake_decel_;
        if (vehicle_state_.brake > 250.0)
        {
            vehicle_state_.throttle = 0.0;
        }
        else if (desired_acceleration > db || (desired_acceleration > 0.1 && vehicle_state_.vx < 5.0))
        {
            double delta_throttle = (throttle_kp_ * acc_error_ + throttle_ki_ * acc_integral_error_ + throttle_kd_ * acc_derivative_error_);
            vehicle_state_.throttle += delta_throttle;
        }
        else
        {
            vehicle_state_.throttle = 0.0;
        }
    }

    void ControllerNode::calc_brake(double desired_acceleration)
    {

        // Calculate Deadband
        double db = -non_brake_decel_ + 0.05;
        double brake_setpoint = (acc_error_ - db);
        debug_msg_.brake_deadband = db;
        if (vehicle_state_.vx < 1.0 && desired_acceleration < 0.0)
        {
            vehicle_state_.brake = 0.0;
        }
        else if (desired_acceleration < db)
        {

            vehicle_state_.brake = -(desired_acceleration - db) * VEHICLE_MASS_KG * REAR_WHEEL_RAD;
            vehicle_state_.brake += -(brake_kp_ * brake_setpoint + brake_ki_ * acc_integral_error_ + brake_kd_ * acc_derivative_error_) * this->REAR_WHEEL_RAD * this->VEHICLE_MASS_KG;
        }
        else
        {
            vehicle_state_.brake = 0.0;
        }
    }

    PathPoint ControllerNode::pure_pursuit_target_point(const Path &path, int start_index, const PathPoint &position, double lookahead) const
    {
        int min_ind = 0;
        double min_diff = 1000000.0;
        // todo: make better w/ interpolation between indices
        // the path.points.size()/2 is to make sure we don't accidentally look behind ourselves
        for (unsigned int i = start_index; i < path.points.size() / 2 + start_index; i++)
        {
            double dx = (path.points[i % path.points.size()].x - position.x);
            double dy = (path.points[i % path.points.size()].y - position.y);
            // Finds the point that is along the circle at a distance `lookahead` away.
            double diff = abs(lookahead - sqrt(dx * dx + dy * dy));
            if (diff < min_diff)
            {
                min_diff = diff;
                min_ind = i;
            }
        }
        return path.points[min_ind % path.points.size()];
    }

    void ControllerNode::receivePtReport()
    {
        current_gear_ = this->current_gear;
        engine_speed_ = this->engine_rpm;
        engine_running_ = bool(this->engine_rpm > 500);
        reported_throttle_ = this->throttle_position;
        engine_braking_decel = ((engine_speed_ > 1300 && vehicle_state_.vx > 5.0 && reported_throttle_ < 5.0) ? (30 * GEAR_RATIOS[current_gear_] * FINAL_DRIVE_RATIO) : 0.0);
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

    // Helper Functions
    Path ControllerNode::load_path(std::string filename)
    {
        std::vector<std::vector<double>> path_data = read_csv(filename, 4, ",");

        alglib::spline1dinterpolant x_interpolator, y_interpolator;

        // Calculate Arc Length
        std::vector<double> arc_length;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
        arc_length.push_back(0.0);
        x.push_back(path_data[0][0]);
        y.push_back(path_data[0][1]);

        for (unsigned int i = 1; i < path_data[0].size(); i++)
        {
            x.push_back(path_data[0][i]);
            y.push_back(path_data[1][i]);
            double dx = path_data[0][i] - path_data[0][i - 1];
            double dy = path_data[1][i] - path_data[1][i - 1];
            double ds = std::sqrt(dx * dx + dy * dy);
            arc_length.push_back(arc_length[i - 1] + ds);
        }

        // Generate Splines
        alglib::real_1d_array xs, ys, s;
        xs.setcontent(x.size(), x.data());
        ys.setcontent(y.size(), y.data());
        s.setcontent(arc_length.size(), arc_length.data());

        alglib::spline1dbuildlinear(s, xs, x_interpolator);
        alglib::spline1dbuildlinear(s, ys, y_interpolator);

        double xp, yp, dxp, dyp, ddxp, ddyp;
        std::vector<double> s_sample;
        double track_length = arc_length[arc_length.size() - 1];
        Path path;
        for (double i = 0; i < track_length; i++)
        {
            alglib::spline1ddiff(x_interpolator, i, xp, dxp, ddxp);
            alglib::spline1ddiff(y_interpolator, i, yp, dyp, ddyp);
            PathPoint point;
            point.s = i;
            point.x = xp;
            point.y = yp;
            point.z = 0.0;
            point.yaw = std::atan2(dyp, dxp);
            path.points.push_back(point);
        }

        return path;
    }


} // namespace controller

RCLCPP_COMPONENTS_REGISTER_NODE(controller::ControllerNode)
