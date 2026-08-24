#include "asm_socketcan_bridge.h"

namespace {
constexpr std::size_t kNovatelTopIndex = 0;
constexpr std::size_t kNovatelBottomIndex = 1;
constexpr std::size_t kVectorNavGpsLeftIndex = 0;
constexpr std::size_t kVectorNavGpsRightIndex = 1;
}  // namespace

namespace asm_socketcan_bridge {
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
        foxgloveMap.latitude = bus.nova_tel_pwr_pak1_var.best_pos_var.lat;
        foxgloveMap.longitude = bus.nova_tel_pwr_pak1_var.best_pos_var.lon;
        foxgloveMap.altitude = bus.nova_tel_pwr_pak1_var.best_pos_var.hgt;
        populated = true;
        return;
      }
      if (fellowID == 1) {
        foxgloveMap.latitude = bus.ground_truth_var.lat[0];
        foxgloveMap.longitude = bus.ground_truth_var.lon[0];
        foxgloveMap.altitude = bus.ground_truth_var.hgt[0];
        populated = true;
        return;
      }
      if (fellowID == 2) {
        foxgloveMap.latitude = bus.ground_truth_var.lat[1];
        foxgloveMap.longitude = bus.ground_truth_var.lon[1];
        foxgloveMap.altitude = bus.ground_truth_var.hgt[1];
        populated = true;
        return;
      }
      if (fellowID == 3) {
        foxgloveMap.latitude = bus.ground_truth_var.lat[2];
        foxgloveMap.longitude = bus.ground_truth_var.lon[2];
        foxgloveMap.altitude = bus.ground_truth_var.hgt[2];
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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

  void AsmSocketCanBridgeNode::publish_vectornav_attitude_group()
  {
    vectornav_msgs::msg::AttitudeGroup attitudeGroup;
    setHeader(attitudeGroup.header, "world");
    bool populated = false;
    if (!withCanBusShared([&](const ASMBus &bus) {
      const auto &source = bus.vector_nav_vn1_var.attitude_group_var;
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
      const auto &source = bus.vector_nav_vn1_var.common_group_var;
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
      const auto &source = bus.vector_nav_vn1_var.imu_group_var;
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
      populateGpsGroupMessage(gpsGroup, bus.vector_nav_vn1_var.gps_group1_var);
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
      populateGpsGroupMessage(gpsGroup, bus.vector_nav_vn1_var.gps_group2_var);
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
      const auto &source = bus.vector_nav_vn1_var.ins_group_var;
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
      const auto &source = bus.vector_nav_vn1_var.time_group_var;
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
      groundTruthArray.header.stamp.sec = this->simTime_.seconds();
      groundTruthArray.header.stamp.nanosec = this->simTime_.nanoseconds();
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
      const auto fellow_count_raw = bus.ground_truth_var.fellow_count;
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

        vehicle.car_num = bus.ground_truth_var.car_num[vehicleID];
        vehicle.lat = bus.ground_truth_var.lat[vehicleID];
        vehicle.lon = bus.ground_truth_var.lon[vehicleID];
        vehicle.hgt = bus.ground_truth_var.hgt[vehicleID];
        vehicle.vx = bus.ground_truth_var.vx[vehicleID];
        vehicle.vy = bus.ground_truth_var.vy[vehicleID];
        vehicle.vz = bus.ground_truth_var.vz[vehicleID];
        if (this->verbosePrinting) {
          RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 4");
        }
        vehicle.yaw = bus.ground_truth_var.yaw[vehicleID];
        vehicle.pitch = bus.ground_truth_var.pitch[vehicleID];
        vehicle.roll = bus.ground_truth_var.roll[vehicleID];
        if (this->verbosePrinting) {
          RCLCPP_INFO(this->get_logger(), "publishGroundTruthArray Checkpoint 5");
        }
        vehicle.del_x = bus.ground_truth_var.del_x[vehicleID];
        vehicle.del_y = bus.ground_truth_var.del_y[vehicleID];
        vehicle.del_z = bus.ground_truth_var.del_z[vehicleID];
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


  // can messages
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
      const auto *data = index == kNovatelTopIndex ? &bus.nova_tel_pwr_pak1_var
                                                   : &bus.nova_tel_pwr_pak2_var;
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



} // namespace asm_socketcan_bridge
