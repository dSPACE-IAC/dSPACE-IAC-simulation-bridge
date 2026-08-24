#include "asm_socketcan_bridge.h"

#include <chrono>
#include <string>
#include <string_view>

namespace asm_socketcan_bridge {

  void AsmSocketCanBridgeNode::setHeader(std_msgs::msg::Header &header, std::string_view frame_id) const
  {
    header.frame_id = std::string(frame_id);
    if (simModeEnabled) {
      header.stamp.sec = this->simTime_.seconds();
      header.stamp.nanosec = this->simTime_.nanoseconds();
      return;
    }
    const auto now = std::chrono::system_clock::now();
    const auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now).time_since_epoch().count();
    const auto nsecs_total = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();
    header.stamp.sec = static_cast<int32_t>(secs);
    header.stamp.nanosec = static_cast<uint32_t>(nsecs_total - (secs * 1000000000LL));
  }

  void AsmSocketCanBridgeNode::populateBestPosMessage(novatel_oem7_msgs::msg::BESTPOS &message, const nova_tel_pwr_pak &data) const
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

  void AsmSocketCanBridgeNode::populateBestVelMessage(novatel_oem7_msgs::msg::BESTVEL &message, const nova_tel_pwr_pak &data) const
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

  void AsmSocketCanBridgeNode::populateInspvaMessage(novatel_oem7_msgs::msg::INSPVA &message, const nova_tel_pwr_pak &data) const
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

  void AsmSocketCanBridgeNode::populateHeading2Message(novatel_oem7_msgs::msg::HEADING2 &message, const nova_tel_pwr_pak &data) const
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

  void AsmSocketCanBridgeNode::populateRawImuMessage(novatel_oem7_msgs::msg::RAWIMU &message, const nova_tel_pwr_pak &data) const
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

  void AsmSocketCanBridgeNode::populateRawImuXMessage(sensor_msgs::msg::Imu &message, const nova_tel_pwr_pak &data) const
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

  void AsmSocketCanBridgeNode::populateGpsGroupMessage(vectornav_msgs::msg::GpsGroup &message, const gps_group &source)
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

} // namespace asm_socketcan_bridge
