#include <cstdlib>
#include <filesystem>
#include <thread>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "npc_controller.hpp"
#include "iac_sim_time/sim_clock_mode.hpp"
#include <rclcpp/qos.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

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
}  // namespace

namespace controller
{
    ControllerNode::ControllerNode(const rclcpp::NodeOptions &options)
        : Node("npc_controller", options)
    {
        const auto history_size{declare_parameter("qos_history", 1)};
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(history_size), rmw_qos_profile_iac);
        const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(history_size), rmw_qos_profile_sim_clock);

        bool use_sim_time = false;
        if (!this->get_parameter("use_sim_time", use_sim_time))
        {
            use_sim_time = false;
        }

        if (const char *sim_clock_mode = std::getenv("SIM_CLOCK_MODE"))
        {
            const auto environment_setting = iac_sim_time::parse_sim_clock_mode(sim_clock_mode);
            if (environment_setting.has_value())
            {
                use_sim_time = environment_setting.value();
                this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
                RCLCPP_INFO(this->get_logger(),
                            "SIM_CLOCK_MODE environment override: %s",
                            use_sim_time ? "true" : "false");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "Ignoring invalid SIM_CLOCK_MODE value '%s'; using use_sim_time parameter",
                            sim_clock_mode);
            }
        }

        this->simModeEnabled = use_sim_time;
        RCLCPP_INFO(this->get_logger(),
                    "Simulation clock mode %s",
                    this->simModeEnabled ? "enabled" : "disabled");

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
        std::string track_name = declare_parameter("track_name", "ims");

        // Configure geodetic origin for global GPS -> local XY conversion.
        const double gps_origin_lat = this->declare_parameter<double>(
            "gps_origin.lat", 39.7947350319205384);
        const double gps_origin_lon = this->declare_parameter<double>(
            "gps_origin.lon", -86.2352425671970906);
        const double gps_origin_hgt = this->declare_parameter<double>(
            "gps_origin.hgt", 224.1435846661534015);
        gps_map_.Reset(gps_origin_lat, gps_origin_lon, gps_origin_hgt);
        RCLCPP_INFO(this->get_logger(),
                    "GPS local origin set to lat=%.10f lon=%.10f hgt=%.3f",
                    gps_origin_lat,
                    gps_origin_lon,
                    gps_origin_hgt);

        std::filesystem::path share_dir = std::filesystem::path(ament_index_cpp::get_package_share_directory("npc_controller"));
        std::filesystem::path map_dir = share_dir /
                                        std::filesystem::path("maps") /
                                        std::filesystem::path(track_name);
        std::filesystem::path center_line_dir = map_dir / std::filesystem::path("center_line.csv");
        std::filesystem::path pit_line_dir = map_dir / std::filesystem::path("pit_lane.csv");
        const std::filesystem::path optimal_line_dir = map_dir / std::filesystem::path("optimal_line.csv");

        if (!std::filesystem::exists(center_line_dir) && std::filesystem::exists(optimal_line_dir)) {
            RCLCPP_WARN(this->get_logger(),
                        "center_line.csv missing for track '%s'; falling back to optimal_line.csv",
                        track_name.c_str());
            center_line_dir = optimal_line_dir;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Loading track '%s' from map dir: %s",
                    track_name.c_str(),
                    map_dir.string().c_str());
        RCLCPP_INFO(this->get_logger(), "Center line file: %s", center_line_dir.string().c_str());
        center_line_ = load_path(center_line_dir.string());

        if (std::filesystem::exists(optimal_line_dir)) {
            RCLCPP_INFO(this->get_logger(), "Optimal line file: %s", optimal_line_dir.string().c_str());
            optimal_line_ = load_path(optimal_line_dir.string());
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "optimal_line.csv missing for track '%s'; reusing center line for optimal-line diagnostics",
                        track_name.c_str());
            optimal_line_ = center_line_;
        }

        if (!std::filesystem::exists(pit_line_dir)) {
            RCLCPP_WARN(this->get_logger(),
                        "pit_lane.csv missing for track '%s'; reusing center line as pit line",
                        track_name.c_str());
            pit_line_ = center_line_;
        } else {
            RCLCPP_INFO(this->get_logger(), "Pit lane file: %s", pit_line_dir.string().c_str());
            pit_line_ = load_path(pit_line_dir.string());
        }
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
        // In deterministic sim mode the control functions are invoked once per /clock tick
        // from simClockTimeCallback, so periodic timers must NOT be created here (they would
        // double-invoke the control loop). Wall mode keeps the periodic timers unchanged.
        if (!this->simModeEnabled) {
            register_timer("pure_pursuit_timer", [this]() { this->pure_pursuit(); });
            register_timer("long_control_timer", [this]() { this->long_control(); });
            register_timer("control_timer", [this]() { this->lateral_control(); });
            register_timer("state_machine_timer", [this]() { this->state_machine(); });
        }

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
        declare_parameter("vehicle.steering_cmd_sign", 1.0);
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
        steering_cmd_sign_ = get_parameter("vehicle.steering_cmd_sign").as_double();
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

        // Load speed profile if available (optional parameter)
        speed_profile_.waypoints.clear();
        try {
            // Try to load speed profile waypoints from nested YAML structure
            std::vector<double> profile_s_values;
            std::vector<double> profile_speeds;

            int waypoint_idx = 0;
            while (true) {
                std::string s_param = "lap_state_machine.speed_profile.waypoints." + std::to_string(waypoint_idx) + ".s";
                std::string v_param = "lap_state_machine.speed_profile.waypoints." + std::to_string(waypoint_idx) + ".speed";

                if (!this->has_parameter(s_param) || !this->has_parameter(v_param)) {
                    break;
                }

                try {
                    double s_val = this->declare_parameter<double>(s_param, 0.0);
                    double v_val = this->declare_parameter<double>(v_param, 0.0);

                    SpeedProfileWaypoint wp;
                    wp.s = s_val;
                    wp.speed = v_val;
                    speed_profile_.waypoints.push_back(wp);

                    waypoint_idx++;
                } catch (...) {
                    break;
                }
            }

            if (!speed_profile_.waypoints.empty()) {
                RCLCPP_INFO(this->get_logger(), "Loaded speed profile with %zu waypoints", speed_profile_.waypoints.size());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "No speed profile waypoints specified");
            }
        } catch (...) {
            RCLCPP_DEBUG(this->get_logger(), "No speed profile configuration found");
        }
        lap_state_inputs_.speed_profile = speed_profile_;

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

    ControllerNode::~ControllerNode()
    {
        if (simModeEnabled) {
            RCLCPP_INFO(
                get_logger(),
                "SIM_OBS controller summary=1 clock_received=%llu control_invocations=%llu "
                "zero_clock_messages=%llu handshakes_sent=%llu last_clock_sec=%u "
                "last_clock_nanosec=%u",
                static_cast<unsigned long long>(sim_clock_messages_received_.load()),
                static_cast<unsigned long long>(sim_control_invocations_.load()),
                static_cast<unsigned long long>(sim_zero_clock_messages_.load()),
                static_cast<unsigned long long>(sim_handshakes_sent_.load()),
                sec,
                nsec);
        }
    }

} // namespace controller

RCLCPP_COMPONENTS_REGISTER_NODE(controller::ControllerNode)