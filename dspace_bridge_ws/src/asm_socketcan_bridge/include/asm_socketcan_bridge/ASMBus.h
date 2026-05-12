#ifndef ASMBUS_H
#define ASMBUS_H

#include "rtwtypes.h"
#include "multiword_types.h"

#pragma pack(push, 1)
typedef struct 
{
  real32_T fl_tire_temperature;
  real32_T fl_tire_pressure;
  real32_T fl_tire_pressure_gauge;
  real_T fl_wheel_load;
} front_left_tire;
typedef struct 
{
  real32_T fr_tire_temperature;
  real32_T fr_tire_pressure;
  real32_T fr_tire_pressure_gauge;
  real_T fr_wheel_load;
} front_right_tire;
typedef struct 
{
  real32_T rl_tire_temperature;
  real32_T rl_tire_pressure;
  real32_T rl_tire_pressure_gauge;
  real_T rl_wheel_load;
} rear_left_tire;
typedef struct 
{
  real32_T rr_tire_temperature;
  real32_T rr_tire_pressure;
  real32_T rr_tire_pressure_gauge;
  real_T rr_wheel_load;
} rear_right;
typedef struct 
{
  front_left_tire front_left_tire_var;
  front_right_tire front_right_tire_var;
  rear_left_tire rear_left_tire_var;
  rear_right rear_right_var;
} tires;
typedef struct 
{
  real_T fl_damper_linear_potentiometer;
  real_T fr_damper_linear_potentiometer;
  real_T rl_damper_linear_potentiometer;
  real_T rr_damper_linear_potentiometer;
} suspension;
typedef struct 
{
  real_T front_brake_pressure;
  real_T rear_brake_pressure;
  real32_T fl_brake_temp;
  real32_T fr_brake_temp;
  real32_T rl_brake_temp;
  real32_T rr_brake_temp;
  real32_T f_brk_pos_cmd;
  real32_T f_brk_pos_fbk;
  real32_T r_brk_pos_cmd;
  real32_T r_brk_pos_fbk;
  real32_T f_brake_act_force;
  real32_T r_brake_act_force;
} brake;
typedef struct 
{
  real32_T accel_pedal_input;
  real32_T accel_pedal_output;
} accelerator;
typedef struct 
{
  real32_T ws_front_left;
  real32_T ws_front_right;
  real32_T ws_rear_left;
  real32_T ws_rear_right;
} wheel_speed;
typedef struct 
{
  real32_T battery_voltage;
  uint8_T safety_switch_state;
  boolean_T mode_switch_state;
  uint8_T sys_state;
} misc_report;
typedef struct 
{
  real32_T steering_wheel_angle_cmd;
  real_T steering_wheel_angle;
  real_T steering_wheel_torque;
  real_T primary_steering_angular_rate;
  real_T commanded_steering_rate;
  real_T motor_duty_cycle_cmd;
  real_T motor_duty_cycle_fbk;
  real_T motor_current_fbk;
  real_T sbw_motor_torque_estimate;
  real_T steering_p_contribution;
  real_T steering_i_contribution;
  real_T steering_d_contribution;
} steering;
typedef struct 
{
  uint8_T attitude_quality;
  boolean_T gyro_saturation;
  boolean_T gyro_saturation_recovery;
  uint8_T mag_disturbance;
  boolean_T mag_saturation;
  uint8_T acc_disturbance;
  boolean_T acc_saturation;
  boolean_T known_mag_disturbance;
  boolean_T known_accel_disturbance;
} vpestatus;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} yawpitchroll;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
  real_T w;
} quaternion;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} magned;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} accelned;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} linearaccelbody;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} linearaccelned;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} ypru;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} angularrate;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} position;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} velocity;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} accel;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} imu_accel;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} imu_rate;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} magpres_mag;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} deltatheta_dtheta;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} deltatheta_dvel;
typedef struct 
{
  uint8_T mode;
  boolean_T gps_fix;
  boolean_T time_error;
  boolean_T imu_error;
  boolean_T mag_pres_error;
  boolean_T gps_error;
  boolean_T gps_heading_ins;
  boolean_T gps_compass;
} insstatus;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} uncompmag;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} uncompaccel;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} uncompgyro;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} deltavel;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} mag;
typedef struct 
{
  uint8_T year;
  uint8_T month;
  uint8_T day;
  uint8_T hour;
  uint8_T min;
  uint8_T sec;
  uint16_T ms;
} utc;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} poslla;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} posecef;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} velned;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} velecef;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} posu;
typedef struct 
{
  real32_T g;
  real32_T p;
  real32_T t;
  real32_T v;
  real32_T h;
  real32_T n;
  real32_T e;
} dop;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} slBus1_poslla;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} velbody;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} magecef;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} accelecef;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} linearaccelecef;
typedef struct 
{
  uint8_T year;
  uint8_T month;
  uint8_T day;
  uint8_T hour;
  uint8_T min;
  uint8_T sec;
  uint16_T ms;
} timeutc;
typedef struct 
{
  boolean_T time_ok;
  boolean_T date_ok;
  boolean_T utctime_ok;
} timestatus;
typedef struct 
{
  uint16_T oEM7MSGTYPE_LOG;
  uint8_T message_name[2];
  uint16_T message_id;
  uint8_T message_type;
  uint32_T sequence_number;
  uint8_T time_status;
  uint16_T gps_week_number;
  uint32_T gps_week_milliseconds;
  uint8_T idle_time;
} nov_header;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} linear_acceleration;
typedef struct 
{
  real_T x;
  real_T y;
  real_T z;
} angular_velocity;
typedef struct 
{
  uint32_T iNS_INACTIVE;
  uint32_T iNS_ALIGNING;
  uint32_T iNS_HIGH_VARIANCE;
  uint32_T iNS_SOLUTION_GOOD;
  uint32_T iNS_SOLUTION_FREE;
  uint32_T iNS_ALIGNMENT_COMPLETE;
  uint32_T dETERMINING_ORIENTATION;
  uint32_T wAITING_INITIAL_POS;
  uint32_T wAITING_AZIMUTH;
  uint32_T iNITIALIZING_BIASES;
  uint32_T mOTION_DETECT;
  uint32_T status_var;
} status;
typedef struct 
{
  real32_T map_sensor;
  real32_T lambda_sensor;
  real32_T fuel_level;
  real32_T fuel_pressure;
  real32_T engine_oil_pressure;
  real32_T engine_oil_temperature;
  real32_T engine_coolant_temperature;
  real32_T engine_coolant_pressure;
  real_T engine_rpm;
  real_T engine_on_status;
  real_T engine_run_switch_status;
  real_T throttle_position;
  real_T current_gear;
  real_T gear_shift_status;
  real32_T transmission_oil_pressure;
  real32_T transmission_accumulator_pressure;
  real32_T transmission_oil_temperature;
  real_T vehicle_speed_kmph;
  real_T torque_wheels_nm;
  real_T driver_traction_aim_switch_fbk;
  real_T driver_traction_range_switch_fbk;
  real_T boost_aim_psi;
  real_T boost_press_psi;
  real_T intake_manifold_press_kPa;
  real_T intake_air_temp_degC;
} power_train_data;
typedef struct 
{
  tires tires_var;
  suspension suspension_var;
  brake brake_var;
  accelerator accelerator_var;
  wheel_speed wheel_speed_var;
  misc_report misc_report_var;
  steering steering_var;
} vehicle_data;
typedef struct 
{
  vpestatus vpestatus_var;
  yawpitchroll yawpitchroll_var;
  quaternion quaternion_var;
  real32_T dcm[9];
  magned magned_var;
  accelned accelned_var;
  linearaccelbody linearaccelbody_var;
  linearaccelned linearaccelned_var;
  ypru ypru_var;
} attitude_group;
typedef struct 
{
  uint64_T timestartup;
  uint64_T timegps;
  uint64_T timesyncin;
  yawpitchroll yawpitchroll_var;
  quaternion quaternion_var;
  angularrate angularrate_var;
  position position_var;
  velocity velocity_var;
  accel accel_var;
  imu_accel imu_accel_var;
  imu_rate imu_rate_var;
  magpres_mag magpres_mag_var;
  real32_T magpres_temp;
  real32_T magpres_pres;
  real32_T deltatheta_dtime;
  deltatheta_dtheta deltatheta_dtheta_var;
  deltatheta_dvel deltatheta_dvel_var;
  insstatus insstatus_var;
  uint32_T syncincnt;
  uint16_T timegpspps;
} common_group;
typedef struct 
{
  uint16_T imustatus;
  uncompmag uncompmag_var;
  uncompaccel uncompaccel_var;
  uncompgyro uncompgyro_var;
  real32_T temp;
  real32_T pres;
  real32_T deltatheta_time;
  deltatheta_dtheta deltatheta_dtheta_var;
  deltavel deltavel_var;
  mag mag_var;
  accel accel_var;
  angularrate angularrate_var;
  uint16_T sensat;
} imu_group;
typedef struct 
{
  utc utc_var;
  uint64_T tow;
  uint16_T week;
  uint8_T numsats;
  uint8_T fix;
  poslla poslla_var;
  posecef posecef_var;
  velned velned_var;
  velecef velecef_var;
  posu posu_var;
  real32_T velu;
  uint32_T timeu;
  uint8_T timeinfo_status;
  int8_T timeinfo_leapseconds;
  dop dop_var;
} gps_group;

typedef struct 
{
  insstatus insstatus_var;
  slBus1_poslla poslla_var;
  posecef posecef_var;
  velbody velbody_var;
  velned velned_var;
  velecef velecef_var;
  magecef magecef_var;
  accelecef accelecef_var;
  linearaccelecef linearaccelecef_var;
  real32_T posu_var;
  real32_T velu;
} ins_group;
typedef struct 
{
  uint64_T timestartup;
  uint64_T timegps;
  uint64_T gpstow;
  uint16_T gpsweek;
  uint64_T timesyncin;
  uint64_T timegpspps;
  timeutc timeutc_var;
  uint32_T syncincnt;
  uint32_T syncoutcnt;
  timestatus timestatus_var;
} time_group;
typedef struct 
{
  nov_header nov_header_var;
  uint32_T sol_status;
  uint32_T pos_type;
  real_T lat;
  real_T lon;
  real_T hgt;
  real32_T undulation;
  uint32_T datum_id;
  real32_T lat_stdev;
  real32_T lon_stdev;
  real32_T hgt_stdev;
  int8_T stn_id[4];
  real32_T diff_age;
  real32_T sol_age;
  uint8_T num_svs;
  uint8_T num_sol_svs;
  uint8_T num_sol_l1_svs;
  uint8_T num_sol_multi_svs;
  uint8_T reserved;
  uint8_T ext_sol_stat;
  uint8_T galileo_beidou_sig_mask;
  uint8_T gps_glonass_sig_mask;
} best_pos;
typedef struct 
{
  nov_header nov_header_var;
  uint32_T sol_status;
  uint32_T vel_type;
  real32_T latency;
  real32_T diff_age;
  real_T hor_speed;
  real_T trk_gnd;
  real_T ver_speed;
  real32_T reserved;
} best_vel;
typedef struct 
{
  nov_header nov_header_var;
  uint32_T sol_status;
  uint32_T pos_type;
  real32_T length;
  real_T heading;
  real_T pitch;
  real32_T reserved;
  real32_T heading_stdev;
  real32_T pitch_stdev;
  int8_T rover_stn_id[4];
  int8_T master_stn_id[4];
  uint8_T num_sv_tracked;
  uint8_T num_sv_in_sol;
  uint8_T num_sv_obs;
  uint8_T num_sv_multi;
  uint8_T sol_source;
  uint8_T ext_sol_status;
  uint8_T galileo_beidou_sig_mask;
  uint8_T gps_glonass_sig_mask;
} heading_2;
typedef struct 
{
  nov_header nov_header_var;
  uint32_T gnss_week;
  real_T gnss_seconds;
  uint32_T status_var;
  linear_acceleration linear_acceleration_var;
  angular_velocity angular_velocity_var;
} raw_imu;
typedef struct 
{
  nov_header nov_header_var;
  real_T latitude;
  real_T longitude;
  real_T height;
  real_T north_velocity;
  real_T east_velocity;
  real_T up_velocity;
  real_T roll;
  real_T pitch;
  real_T azimuth;
  status status_var;
} inspava;
typedef struct 
{
  real_T maneuverTime_s;
  real_T maneuverState;
} ManeuverInfo;
typedef struct 
{
  uint8_T base_to_car_heartbeat;
  uint8_T track_flag;
  uint8_T veh_flag;
  uint8_T veh_rank;
  uint8_T lap_count;
  real32_T lap_distance;
  uint8_T round_target_speed;
  uint8_T laps;
  real32_T lap_time;
  real32_T time_stamp;
  uint8_T sys_state;
  uint8_T push2pass_status;
  uint16_T push2pass_budget_s;
  uint8_T push2pass_active_app_limit;
  uint8_T marelli_sector_flag;
  uint8_T marelli_rc_base_sync_check;
  uint8_T target_speed_multi_car_race;
} race_control;
typedef struct 
{
  real_T fellow_count;
  real_T car_num[30];
  real_T lat[30];
  real_T lon[30];
  real_T hgt[30];
  real_T vx[30];
  real_T vy[30];
  real_T vz[30];
  real_T yaw[30];
  real_T pitch[30];
  real_T roll[30];
  real_T del_x[30];
  real_T del_y[30];
  real_T del_z[30];
} ground_truth;
typedef struct 
{
  power_train_data power_train_data_var;
  vehicle_data vehicle_data_var;
} vehicle_sensors;
typedef struct 
{
  attitude_group attitude_group_var;
  common_group common_group_var;
  imu_group imu_group_var;
  gps_group gps_group1_var;
  gps_group gps_group2_var;
  ins_group ins_group_var;
  time_group time_group_var;
} vector_nav_vn1;
typedef struct 
{
  best_pos best_pos_var;
  best_vel best_vel_var;
  heading_2 heading_2_var;
  raw_imu raw_imu_var;
  inspava inspava_var;
} nova_tel_pwr_pak;

typedef struct 
{
  ManeuverInfo maneuverInfo;
  race_control race_control_var;
  ground_truth ground_truth_var;
  vehicle_sensors vehicle_sensors_var;
  vector_nav_vn1 vector_nav_vn1_var;
  nova_tel_pwr_pak nova_tel_pwr_pak1_var;
  nova_tel_pwr_pak nova_tel_pwr_pak2_var;
} ASMBus;
#pragma pack(pop)

#endif /* ASMBUS_H */