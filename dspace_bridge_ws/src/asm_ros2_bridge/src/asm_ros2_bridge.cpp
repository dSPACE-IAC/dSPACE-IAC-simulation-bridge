#include "asm_ros2_bridge.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace asm_ros2_bridge {

  AsmRos2BridgeNode::AsmRos2BridgeNode() : Node("asm_ros2_bridge_node")
  {
    if (std::getenv("VESI_IP")){
      this->api.setSimManagerHost(std::getenv("VESI_IP"));
      RCLCPP_INFO(this->get_logger(), "Set SimManager Host IP to: %s", std::getenv("VESI_IP"));
    } else {
      this->api.setSimManagerHost("127.0.0.1");
      RCLCPP_INFO(this->get_logger(), "Set SimManager Host IP to: 127.0.0.1 (default)");
    }
    
    if (std::getenv("ASM_IP")){
      this->api.setASMHost(std::getenv("ASM_IP"));
      RCLCPP_INFO(this->get_logger(), "Set ASM Host IP to: %s", std::getenv("ASM_IP"));
    } else {
      this->api.setASMHost("127.0.0.1");
      RCLCPP_INFO(this->get_logger(), "Set ASM Host IP to: 127.0.0.1 (default)");
    }
       
    if (std::getenv("SIMMANAGER_PORT")){
      this->api.setSimManagerPort(std::stoi(std::getenv("SIMMANAGER_PORT")));
      RCLCPP_INFO(this->get_logger(), "Set SimManager Host Port to: %s", std::getenv("SIMMANAGER_PORT"));
    } else {
      this->api.setSimManagerPort(12345);
      RCLCPP_INFO(this->get_logger(), "Set SimManager Host Port to: 12345 (default)");
    }

    RCLCPP_INFO(this->get_logger(), "Set Publish intervals");
    if (std::getenv("PUB_ITV_RACE_CONTROL_DATA")){
      this->pubIntervalRaceControlData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_RACE_CONTROL_DATA"))));
    } else {
      this->pubIntervalRaceControlData = 10;
    }
    if (std::getenv("PUB_ITV_VEHICLE_DATA")){
      this->pubIntervalVehicleData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_VEHICLE_DATA"))));
    } else {
      this->pubIntervalVehicleData = 10;
    }
    if (std::getenv("PUB_ITV_POWER_TRAIN_DATA")){
      this->pubIntervalPowertrainData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_POWER_TRAIN_DATA"))));
    } else {
      this->pubIntervalPowertrainData = 10;
    }
    if (std::getenv("PUB_ITV_GROUND_TRUTH_ARRAY")){
      this->pubIntervalGroundTruthArray = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_GROUND_TRUTH_ARRAY"))));
    } else {
      this->pubIntervalGroundTruthArray = 10;
    }
    if (std::getenv("PUB_ITV_VECTOR_NAV_DATA")){
      this->pubIntervalVectorNavData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_VECTOR_NAV_DATA"))));
    } else {
      this->pubIntervalVectorNavData = 10;
    }
    if (std::getenv("PUB_ITV_NOVATEL_DATA")){
      this->pubIntervalNovatelData = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_NOVATEL_DATA"))));
    } else {
      this->pubIntervalNovatelData = 10;
    }
    if (std::getenv("PUB_ITV_FOXGLOVE_MAP")){
      this->pubIntervalFoxgloveMap = static_cast<uint32_t>(std::stoul(std::string(std::getenv("PUB_ITV_FOXGLOVE_MAP"))));
    } else {
      this->pubIntervalFoxgloveMap = 10;
    }

    if (std::getenv("PATH_LOG")){
      this->pathTimeRecord = std::string(std::getenv("PATH_LOG"));
    } else {
      this->pathTimeRecord = "/root/record_log";
    }
    if (std::getenv("ENABLE_LOG")){
      if (std::string(std::getenv("ENABLE_LOG")) == "true"){
        this->enableTimeRecord = true;
      } else {
        this->enableTimeRecord = false;
      }
    } else {
      this->enableTimeRecord = false;
    }

    RCLCPP_INFO(this->get_logger(), "Set Custom Data required to: true");
    this->api.setCustomDataRequired(true);

    RCLCPP_INFO(this->get_logger(), "Trying to connect to V-ESI at SimManager IP and Port.");
    RCLCPP_INFO(this->get_logger(), "Trying to connect to ASM at ASM IP with CustomDataInterface set to: true");
    bool vesiConnection = false;
    int16_t retries = 1;
    int16_t max_retries = 1;

    if (std::getenv("VERBOSE_PRINT") && (std::string(std::getenv("VERBOSE_PRINT")) == "true"))
    {
      this->verbosePrinting = true;
    }

    if (std::getenv("SIM_CLOCK_MODE") && (std::string(std::getenv("SIM_CLOCK_MODE")) == "true"))
    {
        this->simModeEnabled = true;
    }

    while (vesiConnection == false)
    {
      try
      {
        std::list<uint16_t> providedControlDataIDs{22222};
        api.setProvidedControlDataIDs(providedControlDataIDs, true);

        this->api.connect();
        RCLCPP_INFO(this->get_logger(), "V-ESI connection configured.");
        vesiConnection = true;
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure V-ESI: %s", e.what());
        if (retries < max_retries)
        {
          RCLCPP_INFO(this->get_logger(), "Failed for the %d time. Try again.", retries);
          retries++;
          rclcpp::sleep_for(1s);
        }
        else
        {
          RCLCPP_FATAL(get_logger(), "Failed to often to initialize V-ESI connection; exiting.");
          rclcpp::shutdown();
          return;
        }
      }
    }
    
    try
    {
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_iac);
      const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sim_clock);
      this->raceControlDataPublisher_ = this->create_publisher<autonoma_msgs::msg::RaceControl>("race_control", qos);
      this->push2PassFeedbackPublisher_ = this->create_publisher<autonoma_msgs::msg::Push2Pass>("push_2_pass", qos);
      this->vehicleDataPublisher_ = this->create_publisher<autonoma_msgs::msg::VehicleData>("vehicle_data", qos);
      this->powertrainDataPublisher_ = this->create_publisher<autonoma_msgs::msg::PowertrainData>("powertrain_data", qos);
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

      this->receiveVehicleCommands_ = this->create_subscription<autonoma_msgs::msg::VehicleInputs>("vehicle_inputs", qos, std::bind(&AsmRos2BridgeNode::subscribeVehicleCommandsCallback, this, _1));
      this->receiveRaptorCommands_ = this->create_subscription<autonoma_msgs::msg::ToRaptor>("to_raptor", qos, std::bind(&AsmRos2BridgeNode::subscribeRaptorCommandsCallback, this, _1));

      this->useCustomRaceControlSource_ = this->create_subscription<std_msgs::msg::Bool>("use_custom_race_control", qos, std::bind(&AsmRos2BridgeNode::switchRaceControlSourceCallback, this, _1));
      initializeFeedback();

      if(this->simModeEnabled)
      {
        RCLCPP_INFO(this->get_logger(), "Use Simulated Clock.");
        this->simClockTimePublisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", sim_qos);      
        this->simTimeIncrease_ = this->create_subscription<std_msgs::msg::UInt16>("sim_time_increase", sim_qos, std::bind(&AsmRos2BridgeNode::simTimeIncreaseCallback, this, _1));
        vesiCallback();
        this->simClockTime.clock = rclcpp::Time(this->sec, this->nsec);
        this->simClockTimePublisher_->publish(this->simClockTime);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Use Wall Clock (system clock).");
        this->updateVESIVehicleInputs_ = this->create_wall_timer(10ms, std::bind(&AsmRos2BridgeNode::vesiCallback, this));
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create object for ASM-ROS2-Bridge node: %s", e.what());
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
      RCLCPP_INFO(this->get_logger(), "Log created under path: %s", this->pathTimeRecord.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Setup done.");
  }

  void AsmRos2BridgeNode::initializeFeedback()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "initializeFeedback");

    this->maneuverStarted = false;

    // Throttle command (%)
    this->feedbackCmd.vehicle_inputs.throttle_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.throttle_cmd_count = 0;

    // # Brake pressure command (kPa)
    this->feedbackCmd.vehicle_inputs.brake_cmd_front = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_rear = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_bias_switch = 0.0;
    this->feedbackCmd.vehicle_inputs.brake_cmd_count = 0;

    // # Steering motor angle command (degrees)
    this->feedbackCmd.vehicle_inputs.steering_cmd = 0.0;
    this->feedbackCmd.vehicle_inputs.steering_cmd_count = 0;

    // # Gear command
    this->feedbackCmd.vehicle_inputs.gear_cmd = 1;
    
    // # Race Control command
    this->feedbackCmd.to_raptor.track_cond_ack = 0;
    this->feedbackCmd.to_raptor.veh_sig_ack = 0;
    this->feedbackCmd.to_raptor.ct_state = 0;
    this->feedbackCmd.to_raptor.rolling_counter = 0;
    this->feedbackCmd.to_raptor.veh_num = 255;

    this->feedbackCmd.to_raptor.push2pass_request = false;
  }

  void AsmRos2BridgeNode::simClockTimeCallback()
  {
    // Configure simulated clock
    simClockTime.clock = rclcpp::Time(this->sec,this->nsec);
    this->simClockTimePublisher_->publish(simClockTime);
  }

  void AsmRos2BridgeNode::simTimeIncreaseCallback(const std_msgs::msg::UInt16 & msg)
  {
      for (uint16_t timeIncreaseStep = 0; timeIncreaseStep <  msg.data; timeIncreaseStep++)
      {
        vesiCallback();
      }
      simClockTimeCallback();
  }

  void AsmRos2BridgeNode::vesiCallback()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "vesiCallback");

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
      std::vector<uint8_t> canbus_raw;

      if (this->maneuverStarted == true)
      {
        if (this->enableTimeRecord){
          this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        }
        AsmRos2BridgeNode::sendVehicleFeedbackToSimulation();
        if (this->enableTimeRecord){
          this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
          this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
        }

        if (this->enableTimeRecord){
          this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        }
        this->api.requestCustomData(&canbus_raw);
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
        this->api.requestCustomData(&canbus_raw);
      }

      if (this->enableTimeRecord){
        this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      }
      if (canbus_raw.empty()==false)
      {
        this->canBus = reinterpret_cast<ASMBus*>(canbus_raw.data());
        if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState == 3 && this->maneuverStarted == false)
        {
          this->maneuverStarted = true;
          RCLCPP_INFO(this->get_logger(), "Maneuver started. Data will be published");
        } else if (this->canBus->asm_bus_var.environment.maneuver.maneuverScheduler.info.maneuverState != 3 && this->maneuverStarted == true)
        {
          RCLCPP_INFO(this->get_logger(), "aneuver stopped. System will be reset");
          initializeFeedback();
          std_msgs::msg::Bool resetMsg;
          resetMsg.data = true;
          this->resetCommandPublisher_->publish(resetMsg);
        }
        this->vesiDataAvailabe = true;
      }
      else {RCLCPP_WARN(this->get_logger(), "No Custom Data available");}
      if (this->enableTimeRecord){
        this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
      }

    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to request data from ASM: %s", e.what());
    }

    if (this->enableTimeRecord){
      this->timeStartNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
    AsmRos2BridgeNode::publishSimulationState();
    if (this->enableTimeRecord){
      this->timeEndNanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      this->measured_vesi_times.push_back((int64_t(this->timeEndNanosec) - int64_t(this->timeStartNanosec)) / 1000000.0);
    }
  }

  void AsmRos2BridgeNode::publishSimulationState()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishSimulationState");

    try
    {
      if (this->vesiDataAvailabe == true && this->simTotalMsec != 0)
      {
        if(this->simTotalMsec % (this->pubIntervalRaceControlData) == 0)
            AsmRos2BridgeNode::publishRaceControlData();
            AsmRos2BridgeNode::publishPush2PassFeedback();

        if(this->simTotalMsec % (this->pubIntervalVehicleData) == 0)
            AsmRos2BridgeNode::publishVehicleData();

        if(this->simTotalMsec % (this->pubIntervalPowertrainData) == 0)
            AsmRos2BridgeNode::publishPowertrainData();

        if(this->simTotalMsec % (this->pubIntervalVectorNavData) == 0)
            AsmRos2BridgeNode::publishVectorNavData();

        if(this->simTotalMsec % (this->pubIntervalNovatelData) == 0)
        {            
            AsmRos2BridgeNode::publishNovatelData(1);
            AsmRos2BridgeNode::publishNovatelData(2);
        }

        if(this->simTotalMsec % (this->pubIntervalFoxgloveMap) == 0)
          AsmRos2BridgeNode::publishFoxgloveMap(0);

        if (this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count > 10) {
          RCLCPP_ERROR(this->get_logger(), "Unreasonable fellow_count value: %f -> GroundTruthArray and fellows in FoxgloveMap will not be published", this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);
        }
        else if (this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count == 0)
        {
          if (this->verbosePrinting)
            RCLCPP_INFO(this->get_logger(), "No fellows included in the scenario. No GroundTruthArray will be published and FoxgloveMap will be published only for the ego");
        }
        else {
          if (this->verbosePrinting)
            RCLCPP_INFO(this->get_logger(), "Reasonable fellow_count value: %f", this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);
          if(this->simTotalMsec % (this->pubIntervalGroundTruthArray) == 0)
            AsmRos2BridgeNode::publishGroundTruthArray();
          if(this->simTotalMsec % (this->pubIntervalFoxgloveMap) == 0)
            for (size_t fellowID = 1; fellowID <= std::min(3, (int)this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count); fellowID++)
              AsmRos2BridgeNode::publishFoxgloveMap(fellowID);
        }

        this->vesiDataAvailabe = false;
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Publishing of data failed: %s", e.what());
    }
  }

  void AsmRos2BridgeNode::publishFoxgloveMap(uint8_t fellowID)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishFoxgloveMap");

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
        RCLCPP_ERROR(this->get_logger(), "Unknown Fellow ID. Only three Fellows are supported.");
      }
    

    foxgloveMap.position_covariance_type = 0;

    // Header
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

  void AsmRos2BridgeNode::sendVehicleFeedbackToSimulation()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "sendVehicleFeedbackToSimulation");
    if (this->raptorDataAvailabe == false && this->stackRaptorConnectionWarningSent == false)
    {
      RCLCPP_WARN(this->get_logger(), "Did not receive to_raptor message. This might lead to unexpected behavior of the RaceControl e.g. setting of flags and P2P is not available. Check that your stack is alive.");
      this->stackRaptorConnectionWarningSent = true;
    }
    else if (this->raptorDataAvailabe == true && this->stackRaptorConnectionWarningSent == true)
    {
      RCLCPP_INFO(this->get_logger(), "to_raptor message received.");
      this->stackRaptorConnectionWarningSent = false;
    }

    if (this->feedbackDataAvailabe == false && this->stackFeedbackConnectionWarningSent == false)
    {
      RCLCPP_WARN(this->get_logger(), "Did not receive vehicle_inputs message. The vehicle might move in an unexpected way. Check that your stack is alive.");
      this->stackFeedbackConnectionWarningSent = true;
    }
    else if (this->feedbackDataAvailabe == true && this->stackFeedbackConnectionWarningSent == true)
    {
      RCLCPP_INFO(this->get_logger(), "vehicle_inputs message received.");
      this->stackFeedbackConnectionWarningSent = false;
    }

    this->api.sendControlData(22222,std::addressof(this->feedbackCmd),sizeof(this->feedbackCmd));
    if(this->simModeEnabled)
      this->api.increaseSimulationTime(0.001);
    else
      this->api.increaseSimulationTime(0.01);
  }

  void AsmRos2BridgeNode::subscribeVehicleCommandsCallback(const autonoma_msgs::msg::VehicleInputs & msg)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "subscribeVehicleCommandsCallback");

    // Throttle command (%)
    this->feedbackCmd.vehicle_inputs.throttle_cmd = msg.throttle_cmd;

    this->feedbackCmd.vehicle_inputs.throttle_cmd_count = msg.throttle_cmd_count;
    this->feedbackCmd.vehicle_inputs.enable_throttle_cmd = 1;

    // # Brake pressure command (kPa)
    this->feedbackCmd.vehicle_inputs.brake_cmd_front = msg.brake_cmd_front;
    this->feedbackCmd.vehicle_inputs.brake_cmd_rear = msg.brake_cmd_rear;
    this->feedbackCmd.vehicle_inputs.brake_bias_switch = msg.brake_bias_switch;
    this->feedbackCmd.vehicle_inputs.brake_cmd_count = msg.brake_cmd_count;
    this->feedbackCmd.vehicle_inputs.enable_brake_cmd = 1;

    // # Steering motor angle command (degrees)
    this->feedbackCmd.vehicle_inputs.steering_cmd = msg.steering_cmd;
    this->feedbackCmd.vehicle_inputs.steering_cmd_count = msg.steering_cmd_count;
    this->feedbackCmd.vehicle_inputs.enable_steering_cmd = 1;

    // # Gear command
    this->feedbackCmd.vehicle_inputs.gear_cmd = msg.gear_cmd;
    this->feedbackCmd.vehicle_inputs.enable_gear_cmd = 1;

    this->feedbackDataAvailabe = true;
  }

  void AsmRos2BridgeNode::subscribeRaptorCommandsCallback(const autonoma_msgs::msg::ToRaptor & msg)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "subscribeRaptorCommandsCallback");

    // time 5 end "stack_answer_time_toraptor"

    this->feedbackCmd.to_raptor.track_cond_ack = msg.track_cond_ack;
    this->feedbackCmd.to_raptor.veh_sig_ack = msg.veh_sig_ack;
    this->feedbackCmd.to_raptor.ct_state = msg.ct_state;
    this->feedbackCmd.to_raptor.rolling_counter = msg.rolling_counter;
    if (msg.veh_num != 0 && msg.veh_num != 255)
    {
      this->feedbackCmd.to_raptor.veh_num = msg.veh_num;
    }
    else
    {
      if (this->numberWarningSent == false)
      {
        RCLCPP_ERROR(this->get_logger(), "Vehicle number should not be 0 or 255. Will be replaced by 254. Change this configuration to enable multi car racing.");
        this->numberWarningSent = true;
      }
      this->feedbackCmd.to_raptor.veh_num = 254;
    }

    this->feedbackCmd.to_raptor.push2pass_request = msg.push2pass_request;
    
    this->raptorDataAvailabe = true;
  }

  void AsmRos2BridgeNode::switchRaceControlSourceCallback(const std_msgs::msg::Bool & msg)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "switchRaceControlSourceCallback");

    this->useCustomRaceControl = msg.data;
  }

  void AsmRos2BridgeNode::publishRaceControlData()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishRaceControlData");

    auto raceControlData = autonoma_msgs::msg::RaceControl();

    // RaceControl Message
    if (this->maneuverStarted)
    {
      raceControlData.base_to_car_heartbeat = this->canBus->asm_bus_var.race_control_var.base_to_car_heartbeat;
      // raceControlData.track_flag = this->canBus->asm_bus_var.race_control_var.track_flag;
      if (this->canBus->asm_bus_var.race_control_var.track_flag == 4)
      {
        raceControlData.track_flag = 4;
      }
      else
      {
        raceControlData.track_flag = this->canBus->asm_bus_var.race_control_var.track_flag;
      }
      raceControlData.veh_flag = this->canBus->asm_bus_var.race_control_var.veh_flag;
      raceControlData.sys_state = this->canBus->asm_bus_var.race_control_var.sys_state;
      raceControlData.veh_rank = this->canBus->asm_bus_var.race_control_var.veh_rank;
      raceControlData.lap_count = this->canBus->asm_bus_var.race_control_var.lap_count;
      raceControlData.lap_distance = this->canBus->asm_bus_var.race_control_var.lap_distance;
      raceControlData.round_target_speed = this->canBus->asm_bus_var.race_control_var.round_target_speed;

      raceControlData.laps = this->canBus->asm_bus_var.race_control_var.laps;
      raceControlData.lap_time = this->canBus->asm_bus_var.race_control_var.lap_time;
      raceControlData.time_stamp = this->canBus->asm_bus_var.race_control_var.time_stamp;
      raceControlData.rc_base_sync_check = true;
    }
    else
    {
      raceControlData.base_to_car_heartbeat = this->canBus->asm_bus_var.race_control_var.base_to_car_heartbeat;
      raceControlData.track_flag = 255;
      raceControlData.veh_flag = 1;
      raceControlData.sys_state = 255;
      raceControlData.veh_rank = 1;
      raceControlData.lap_count = 0;
      raceControlData.lap_distance = 0;
      raceControlData.round_target_speed = 0;

      raceControlData.laps = 0;
      raceControlData.lap_time = 0;
      raceControlData.rc_base_sync_check = false;
      raceControlData.time_stamp = this->canBus->asm_bus_var.race_control_var.time_stamp;
    }
    
    // raceControlData.rc_base_sync_check = true;
    
    // Header
    raceControlData.header.frame_id = "";

    if(this->simModeEnabled)
    {
      raceControlData.header.stamp.sec = this->sec;
      raceControlData.header.stamp.nanosec = this->nsec;
    }
    else
    {
      raceControlData.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      raceControlData.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (raceControlData.header.stamp.sec*1000000000);
    }

    if (this->useCustomRaceControl == false)
    {
      this->raceControlDataPublisher_->publish(raceControlData);
    }
  }

  void AsmRos2BridgeNode::publishPush2PassFeedback()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishPush2PassFeedback");

    auto p2pData = autonoma_msgs::msg::Push2Pass();

    // Push2Pass Message

    p2pData.push2pass_status = this->canBus->asm_bus_var.race_control_var.push2pass_status;
    p2pData.push2pass_budget_s = this->canBus->asm_bus_var.race_control_var.push2pass_budget_s;
    p2pData.push2pass_active_app_limit = this->canBus->asm_bus_var.race_control_var.push2pass_active_app_limit;

    // Header
    p2pData.header.frame_id = "";

    if(this->simModeEnabled)
    {
      p2pData.header.stamp.sec = this->sec;
      p2pData.header.stamp.nanosec = this->nsec;
    }
    else
    {
      p2pData.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      p2pData.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (p2pData.header.stamp.sec*1000000000);
    }

    this->push2PassFeedbackPublisher_->publish(p2pData);
  }

  void AsmRos2BridgeNode::publishVehicleData()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishVehicleData");

    auto vehicleData = autonoma_msgs::msg::VehicleData();

    // Vehicle Data Message
    // Tire data
    vehicleData.fl_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_temperature;
    vehicleData.fl_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_damper_linear_potentiometer;
    vehicleData.fl_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_pressure;
    vehicleData.fl_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_tire_pressure_gauge;
    vehicleData.fl_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_wheel_load;

    vehicleData.fr_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_temperature;
    vehicleData.fr_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_damper_linear_potentiometer;
    vehicleData.fr_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_pressure;
    vehicleData.fr_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_tire_pressure_gauge;
    vehicleData.fr_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_wheel_load;

    vehicleData.rl_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_temperature;
    vehicleData.rl_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_damper_linear_potentiometer;
    vehicleData.rl_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_pressure;
    vehicleData.rl_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_tire_pressure_gauge;
    vehicleData.rl_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_wheel_load;

    vehicleData.rr_tire_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_temperature;
    vehicleData.rr_damper_linear_potentiometer = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_damper_linear_potentiometer;
    vehicleData.rr_tire_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_pressure;
    vehicleData.rr_tire_pressure_gauge = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_tire_pressure_gauge;
    vehicleData.rr_wheel_load = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_wheel_load;

    // Brake temps
    vehicleData.fl_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fl_brake_temp;
    vehicleData.fr_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.fr_brake_temp;
    vehicleData.rl_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rl_brake_temp;
    vehicleData.rr_brake_temp = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rr_brake_temp;

    // Misc data
    vehicleData.battery_voltage = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.battery_voltage;
    vehicleData.safety_switch_state = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.safety_switch_state;
    vehicleData.mode_switch_state = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.mode_switch_state;
    vehicleData.sys_state = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.sys_state;

    // Accel pedal report 
    vehicleData.accel_pedal_input = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.accel_pedal_input;
    vehicleData.accel_pedal_output = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.accel_pedal_output;

    // Brake report
    vehicleData.front_brake_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.front_brake_pressure;
    vehicleData.rear_brake_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.rear_brake_pressure;

    // Steering report
    vehicleData.steering_wheel_angle = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle;
    vehicleData.steering_wheel_angle_cmd = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_angle_cmd;
    vehicleData.steering_wheel_torque = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.steering_wheel_torque;

    // Wheel speeds (kph)
    vehicleData.ws_front_left = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_left;
    vehicleData.ws_front_right = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_front_right;
    vehicleData.ws_rear_left = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_left;
    vehicleData.ws_rear_right = this->canBus->sim_interface_var.vehicle_sensors_var.vehicle_data_var.ws_rear_right;

    // Header
    vehicleData.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      vehicleData.header.stamp.sec = this->sec;
      vehicleData.header.stamp.nanosec = this->nsec;
    }
    else
    {
      vehicleData.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      vehicleData.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (vehicleData.header.stamp.sec*1000000000);
    }

    this->vehicleDataPublisher_->publish(vehicleData);
  }

  void AsmRos2BridgeNode::publishPowertrainData()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishPowertrainData");
    
    auto powertrainData = autonoma_msgs::msg::PowertrainData();

    // Powertrain Data Message
    // Data
    powertrainData.map_sensor = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.map_sensor;
    powertrainData.lambda_sensor = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.lambda_sensor;
    
    powertrainData.fuel_level = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.fuel_level;
    powertrainData.fuel_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.fuel_pressure;
    
    powertrainData.engine_oil_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_oil_pressure;
    powertrainData.engine_oil_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_oil_temperature;
    powertrainData.engine_coolant_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_coolant_temperature;
    powertrainData.engine_coolant_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_coolant_pressure;
    
    powertrainData.engine_rpm = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_rpm;
    powertrainData.engine_on_status = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_on_status;
    powertrainData.engine_run_switch_status = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.engine_run_switch_status;
    powertrainData.throttle_position = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.throttle_position;
    
    powertrainData.current_gear = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.current_gear;
    powertrainData.gear_shift_status = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.gear_shift_status;

    powertrainData.transmission_oil_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_oil_pressure;
    powertrainData.transmission_accumulator_pressure = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_accumulator_pressure;
    powertrainData.transmission_oil_temperature = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.transmission_oil_temperature;
    powertrainData.vehicle_speed_kmph = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.vehicle_speed_kmph;
    powertrainData.torque_wheels_nm = this->canBus->sim_interface_var.vehicle_sensors_var.power_train_data_var.torque_wheels_nm;

    // Header
    powertrainData.header.frame_id = "ego";

    if(this->simModeEnabled)
    {
      powertrainData.header.stamp.sec = this->sec;
      powertrainData.header.stamp.nanosec = this->nsec;
    }
    else
    {
      powertrainData.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
      powertrainData.header.stamp.nanosec = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - (powertrainData.header.stamp.sec*1000000000);
    }

    this->powertrainDataPublisher_->publish(powertrainData);
  }

  void AsmRos2BridgeNode::publishGroundTruthArray()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray");

    if (this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count > 10) {
      RCLCPP_ERROR(this->get_logger(), "Unreasonable fellow_count value: %f", this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);
      return;
    }
    else if (this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count == 0)
    {
      if (this->verbosePrinting)
        RCLCPP_INFO(this->get_logger(), "No fellows included in the scenario. No GroundTruthArray will be published");
      return;
    }
    else {
      if (this->verbosePrinting)
        RCLCPP_INFO(this->get_logger(), "Reasonable fellow_count value: %f", this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);
    }

    auto groundTruthArray = autonoma_msgs::msg::GroundTruthArray();

    groundTruthArray.vehicles.resize(this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count);

    // Header
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
    bool groundTruthArrayFilled = false;

    for (size_t vehicleID = 0; vehicleID < this->canBus->sim_interface_var.vehicle_sensors_var.fellow_count; vehicleID++)
    {
      groundTruthArray.vehicles[vehicleID].header.frame_id = groundTruthArray.header.frame_id;
      groundTruthArray.vehicles[vehicleID].header.stamp.sec = groundTruthArray.header.stamp.sec;
      groundTruthArray.vehicles[vehicleID].header.stamp.nanosec = groundTruthArray.header.stamp.nanosec;

      groundTruthArray.vehicles[vehicleID].car_num = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.car_num[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].lat = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lat[vehicleID];
      groundTruthArray.vehicles[vehicleID].lon = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.lon[vehicleID];
      groundTruthArray.vehicles[vehicleID].hgt = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.hgt[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].vx = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vx[vehicleID];
      groundTruthArray.vehicles[vehicleID].vy = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vy[vehicleID];
      groundTruthArray.vehicles[vehicleID].vz = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.vz[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].yaw = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.yaw[vehicleID];
      groundTruthArray.vehicles[vehicleID].pitch = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.pitch[vehicleID];
      groundTruthArray.vehicles[vehicleID].roll = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.roll[vehicleID];
      
      groundTruthArray.vehicles[vehicleID].del_x = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_x[vehicleID];
      groundTruthArray.vehicles[vehicleID].del_y = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_y[vehicleID];
      groundTruthArray.vehicles[vehicleID].del_z = this->canBus->sim_interface_var.vehicle_sensors_var.ground_truth_var.del_z[vehicleID];
      groundTruthArrayFilled = true;
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
    
    this->groundTruthArrayPublisher_->publish(groundTruthArray);
  }

  void AsmRos2BridgeNode::publishVectorNavData()
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishVectorNavData");

    auto attitudeGroup = vectornav_msgs::msg::AttitudeGroup();
    auto commonGroup = vectornav_msgs::msg::CommonGroup();
    auto imuGroup = vectornav_msgs::msg::ImuGroup();
    auto gpsGroup = vectornav_msgs::msg::GpsGroup();
    auto insGroup = vectornav_msgs::msg::InsGroup();
    auto timeGroup = vectornav_msgs::msg::TimeGroup();

    // Header
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

    // Header
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

    // Header
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
        RCLCPP_ERROR(this->get_logger(), "Only two Vectornav GPS antennas are supported.");
      }
      
      // Header
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
    
    // Header
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

    insGroup.poslla.x = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.y; // switched x <-> y based on comment of team
    insGroup.poslla.y = this->canBus->sim_interface_var.vector_nav_vn1_var.ins_group_var.poslla_var.x;
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
    
    // Header
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

  void AsmRos2BridgeNode::publishNovatelData(uint8_t novatelID)
  {
    if (this->verbosePrinting)
      RCLCPP_INFO(this->get_logger(), "publishNovatelData");
    
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
        RCLCPP_ERROR(this->get_logger(), "Unknown ID of Novatel Device. Only two Novatels are supported.");
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

    // Header
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

    // Header
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

    // Header
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
    
    // Header
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

    // Header
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

    // Header
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
} // namespace asm_ros2_bridge


int main(int argc, char * argv[])
{
  rclcpp::Node::SharedPtr AsmRos2BridgeNodePtr;
  try
  {
    rclcpp::init(argc, argv);
    
    AsmRos2BridgeNodePtr = std::make_shared<asm_ros2_bridge::AsmRos2BridgeNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(AsmRos2BridgeNodePtr);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception& e)
  {
    if (AsmRos2BridgeNodePtr) {
      RCLCPP_ERROR(AsmRos2BridgeNodePtr->get_logger(), "Failed to initialize ASM-ROS2-Bridge node: %s", e.what());
    } else {
      std::cerr << "Failed to initialize ASM-ROS2-Bridge node: " << e.what() << std::endl;
    }
    return 1;
  }
  
}
