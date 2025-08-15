#ifndef RTW_HEADER_ASMBus_h_
#define RTW_HEADER_ASMBus_h_
#include "rtwtypes.h"
#include "ASMBus.h"
#include "multiword_types.h"
#pragma pack(push, 1) 
typedef struct 
{
  real_T angle_Alpha_FL_Wheel_deg;
  real_T angle_Beta_FL_Wheel_deg;
  real_T angle_Gamma_FL_Wheel_deg;
} Angle_FL_Wheel_Alpha_Beta_Gamma_deg;
typedef struct 
{
  real_T angle_Alpha_FR_Wheel_deg;
  real_T angle_Beta_FR_Wheel_deg;
  real_T angle_Gamma_FR_Wheel_deg;
} Angle_FR_Wheel_Alpha_Beta_Gamma_deg;
typedef struct 
{
  real_T angle_Alpha_RL_Wheel_deg;
  real_T angle_Beta_RL_Wheel_deg;
  real_T angle_Gamma_RL_Wheel_deg;
} Angle_RL_Wheel_Alpha_Beta_Gamma_deg;
typedef struct 
{
  real_T angle_Alpha_RR_Wheel_deg;
  real_T angle_Beta_RR_Wheel_deg;
  real_T angle_Gamma_RR_Wheel_deg;
} Angle_RR_Wheel_Alpha_Beta_Gamma_deg;
typedef struct 
{
  real_T a_x_Vehicle_CoG_m_s2;
  real_T a_y_Vehicle_CoG_m_s2;
  real_T a_z_Vehicle_CoG_m_s2;
} a_Vehicle_CoG_x_y_z_m_s2;
typedef struct 
{
  real_T w_x_Vehicle_CoG_dt_rad_s2;
  real_T w_y_Vehicle_CoG_dt_rad_s2;
  real_T w_z_Vehicle_CoG_dt_rad_s2;
} w_Vehicle_CoG_dt_x_y_z_rad_s2;
typedef struct 
{
  real_T v_x_Vehicle_CoG_km_h;
  real_T v_y_Vehicle_CoG_km_h;
  real_T v_z_Vehicle_CoG_km_h;
} v_Vehicle_CoG_x_y_z_km_h;
typedef struct 
{
  real_T rollRate_Vehicle_CoG_deg_s;
  real_T pitchRate_Vehicle_CoG_deg_s;
  real_T yawRate_Vehicle_CoG_deg_s;
} w_Vehicle_CoG_x_y_z_deg_s;
typedef struct 
{
  real_T v_FL_Abs_WheelBody_x_y_z_km_h[3];
  real_T v_FR_Abs_WheelBody_x_y_z_km_h[3];
  real_T v_RL_Abs_WheelBody_x_y_z_km_h[3];
  real_T v_RR_Abs_WheelBody_x_y_z_km_h[3];
} WheelBodyVelocity;
typedef struct 
{
  real_T w_FL_Abs_WheelBody_x_y_z_rad_s[3];
  real_T w_FR_Abs_WheelBody_x_y_z_rad_s[3];
  real_T w_RL_Abs_WheelBody_x_y_z_rad_s[3];
  real_T w_RR_Abs_WheelBody_x_y_z_rad_s[3];
} WheelBodyAngularVelocity;
typedef struct 
{
  real_T w_FL_Abs_Wheel_x_y_z_rad_s[3];
  real_T w_FR_Abs_Wheel_x_y_z_rad_s[3];
  real_T w_RL_Abs_Wheel_x_y_z_rad_s[3];
  real_T w_RR_Abs_Wheel_x_y_z_rad_s[3];
} WheelAngularVelocity;
typedef struct 
{
  real_T pos_x_Vehicle_CoorSys_E_m;
  real_T pos_y_Vehicle_CoorSys_E_m;
  real_T pos_z_Vehicle_CoorSys_E_m;
} Pos_Vehicle_CoorSys_E_x_y_z_m;
typedef struct 
{
  real_T angle_Roll_Vehicle_CoorSys_E_deg;
  real_T angle_Pitch_Vehicle_CoorSys_E_deg;
  real_T angle_Yaw_Vehicle_CoorSys_E_deg;
} Angle_Vehicle_CoorSys_E_x_y_z_deg;
typedef struct 
{
  real_T f_x_FL_Tire_CoorSys_V_N;
  real_T f_y_FL_Tire_CoorSys_V_N;
  real_T f_z_FL_Tire_CoorSys_V_N;
} F_FL_Tire_CoorSys_V_x_y_z_N;
typedef struct 
{
  real_T trq_x_FL_Tire_CoorSys_V_Nm;
  real_T trq_y_FL_Tire_CoorSys_V_Nm;
  real_T trq_z_FL_Tire_CoorSys_V_Nm;
} Trq_FL_Tire_CoorSys_V_x_y_z_Nm;
typedef struct 
{
  real_T f_x_FR_Tire_CoorSys_V_N;
  real_T f_y_FR_Tire_CoorSys_V_N;
  real_T f_z_FR_Tire_CoorSys_V_N;
} F_FR_Tire_CoorSys_V_x_y_z_N;
typedef struct 
{
  real_T trq_x_FR_Tire_CoorSys_V_Nm;
  real_T trq_y_FR_Tire_CoorSys_V_Nm;
  real_T trq_z_FR_Tire_CoorSys_V_Nm;
} Trq_FR_Tire_CoorSys_V_x_y_z_Nm;
typedef struct 
{
  real_T f_x_RL_Tire_CoorSys_V_N;
  real_T f_y_RL_Tire_CoorSys_V_N;
  real_T f_z_RL_Tire_CoorSys_V_N;
} F_RL_Tire_CoorSys_V_x_y_z_N;
typedef struct 
{
  real_T trq_x_RL_Tire_CoorSys_V_Nm;
  real_T trq_y_RL_Tire_CoorSys_V_Nm;
  real_T trq_z_RL_Tire_CoorSys_V_Nm;
} Trq_RL_Tire_CoorSys_V_x_y_z_Nm;
typedef struct 
{
  real_T f_x_RR_Tire_CoorSys_V_N;
  real_T f_y_RR_Tire_CoorSys_V_N;
  real_T f_z_RR_Tire_CoorSys_V_N;
} F_RR_Tire_CoorSys_V_x_y_z_N;
typedef struct 
{
  real_T trq_x_RR_Tire_CoorSys_V_Nm;
  real_T trq_y_RR_Tire_CoorSys_V_Nm;
  real_T trq_z_RR_Tire_CoorSys_V_Nm;
} Trq_RR_Tire_CoorSys_V_x_y_z_Nm;
typedef struct 
{
  real_T pos_FL_Wheel_x_y_z_m[3];
  real_T pos_FL_Wheel_dq1_Front_x_y_z[3];
  real_T pos_FL_Wheel_dq2_Front_x_y_z[3];
  real_T pos_FL_Wheel_dDispl_SteeringRod_x_y_z[3];
} Position_Left;
typedef struct 
{
  Angle_FL_Wheel_Alpha_Beta_Gamma_deg angle_FL_Wheel_Alpha_Beta_Gamma_deg;
  real_T w_Rel_FL_Wheel_dq1_Front_dt_x_y_z[3];
  real_T w_Rel_FL_Wheel_dq2_Front_dt_x_y_z[3];
  real_T w_Rel_FL_Wheel_dDispl_SteeringRod_dt_x_y_z_rad_m[3];
  real_T unitVec_y_Wheel_FL_CoorSys_V_x_y_z[3];
} Orientation_Left;
typedef struct 
{
  real_T displ_Spring_FL_m;
  real_T displ_Spring_FL_dq1_Front;
  real_T displ_Spring_FL_dq2_Front;
} Spring_Left;
typedef struct 
{
  real_T displ_Damper_FL_dt_m_s;
  real_T displ_Damper_FL_dq1_Front;
  real_T displ_Damper_FL_dq2_Front;
  real_T displ_Damper_FL_m;
} Damper_Left;
typedef struct 
{
  real_T pos_FR_Wheel_x_y_z_m[3];
  real_T pos_FR_Wheel_dq1_Front_x_y_z[3];
  real_T pos_FR_Wheel_dq2_Front_x_y_z[3];
  real_T pos_FR_Wheel_dDispl_SteeringRod_x_y_z[3];
} Position_Right;
typedef struct 
{
  Angle_FR_Wheel_Alpha_Beta_Gamma_deg angle_FR_Wheel_Alpha_Beta_Gamma_deg;
  real_T w_Rel_FR_Wheel_dq1_Front_x_y_z[3];
  real_T w_Rel_FR_Wheel_dq2_Front_x_y_z[3];
  real_T w_Rel_FR_Wheel_dDispl_SteeringRod_dt_x_y_z_rad_m[3];
  real_T unitVec_y_Wheel_FR_CoorSys_V_x_y_z[3];
} Orientation_Right;
typedef struct 
{
  real_T displ_Spring_FR_m;
  real_T displ_Spring_FR_dq1_Front;
  real_T displ_Spring_FR_dq2_Front;
} Spring_Right;
typedef struct 
{
  real_T displ_Damper_FR_dt_m_s;
  real_T displ_Damper_FR_dq1_Front;
  real_T displ_Damper_FR_dq2_Front;
  real_T displ_Damper_FR_m;
} Damper_Right;
typedef struct 
{
  real_T angle_Toe_FL_Wheel_CoorSys_V_deg;
  real_T angle_Camber_FL_Wheel_CoorSys_V_deg;
} Angle_FL_Wheel_Toe_Camber_deg;
typedef struct 
{
  real_T angle_Toe_FR_Wheel_CoorSys_V_deg;
  real_T angle_Camber_FR_Wheel_CoorSys_V_deg;
} Angle_FR_Wheel_Toe_Camber_deg;
typedef struct 
{
  real_T pos_RL_Wheel_x_y_z_m[3];
  real_T pos_RL_Wheel_dq1_Rear_x_y_z[3];
  real_T pos_RL_Wheel_dq2_Rear_x_y_z[3];
  real_T pos_RL_Wheel_dDispl_SteeringRod_x_y_z[3];
} slBus24_Position_Left;
typedef struct 
{
  Angle_RL_Wheel_Alpha_Beta_Gamma_deg angle_RL_Wheel_Alpha_Beta_Gamma_deg;
  real_T w_Rel_RL_Wheel_dq1_Rear_dt_x_y_z[3];
  real_T w_Rel_RL_Wheel_dq2_Rear_dt_x_y_z[3];
  real_T w_Rel_RL_Wheel_dDispl_SteeringRod_dt_x_y_z_rad_m[3];
  real_T unitVec_y_Wheel_RL_CoorSys_V_x_y_z[3];
} slBus25_Orientation_Left;
typedef struct 
{
  real_T displ_Spring_RL_m;
  real_T displ_Spring_RL_dq1_Rear;
  real_T displ_Spring_RL_dq2_Rear;
} slBus26_Spring_Left;
typedef struct 
{
  real_T displ_Damper_RL_dt_m_s;
  real_T displ_Damper_RL_dq1_Rear;
  real_T displ_Damper_RL_dq2_Rear;
  real_T displ_Damper_RL_m;
} slBus27_Damper_Left;
typedef struct 
{
  real_T pos_RR_Wheel_x_y_z_m[3];
  real_T pos_RR_Wheel_dq1_Rear_x_y_z[3];
  real_T pos_RR_Wheel_dq2_Rear_x_y_z[3];
  real_T pos_RR_Wheel_dDispl_SteeringRod_x_y_z[3];
} slBus29_Position_Right;
typedef struct 
{
  Angle_RR_Wheel_Alpha_Beta_Gamma_deg angle_RR_Wheel_Alpha_Beta_Gamma_deg;
  real_T w_Rel_RR_Wheel_dq1_Rear_x_y_z[3];
  real_T w_Rel_RR_Wheel_dq2_Rear_x_y_z[3];
  real_T w_Rel_RR_Wheel_dDispl_SteeringRod_dt_x_y_z_rad_m[3];
  real_T unitVec_y_Wheel_RR_CoorSys_V_x_y_z[3];
} slBus30_Orientation_Right;
typedef struct 
{
  real_T displ_Spring_RR_m;
  real_T displ_Spring_RR_dq1_Rear;
  real_T displ_Spring_RR_dq2_Rear;
} slBus31_Spring_Right;
typedef struct 
{
  real_T displ_Damper_RR_dt_m_s;
  real_T displ_Damper_RR_dq1_Rear;
  real_T displ_Damper_RR_dq2_Rear;
  real_T displ_Damper_RR_m;
} slBus32_Damper_Right;
typedef struct 
{
  real_T angle_Toe_RL_Wheel_CoorSys_V_deg;
  real_T angle_Camber_RL_Wheel_CoorSys_V_deg;
} Angle_RL_Wheel_Toe_Camber_deg;
typedef struct 
{
  real_T angle_Toe_RR_Wheel_CoorSys_V_deg;
  real_T angle_Camber_RR_Wheel_CoorSys_V_deg;
} Angle_RR_Wheel_Toe_Camber_deg;
typedef struct 
{
  a_Vehicle_CoG_x_y_z_m_s2 a_Vehicle_CoG_x_y_z_m_s2_var;
  real_T a_Vehicle_Cog_withG_x_y_z_m_s2[3];
} Linear;
typedef struct 
{
  w_Vehicle_CoG_dt_x_y_z_rad_s2 w_Vehicle_CoG_dt_x_y_z_rad_s2_var;
} Angular;
typedef struct 
{
  real_T a_FL_Wheel_x_y_z_m_s2[3];
  real_T a_FR_Wheel_x_y_z_m_s2[3];
  real_T a_RL_Wheel_x_y_z_m_s2[3];
  real_T a_RR_Wheel_x_y_z_m_s2[3];
} slBus3_Linear;
typedef struct 
{
  v_Vehicle_CoG_x_y_z_km_h v_Vehicle_CoG_x_y_z_km_h_var;
  real_T v_Total_Vehicle_km_h;
  real_T angle_SideSlip_Vehicle_deg;
} slBus4_Linear;
typedef struct 
{
  w_Vehicle_CoG_x_y_z_deg_s w_Vehicle_CoG_x_y_z_deg_s_var;
} slBus5_Angular;
typedef struct 
{
  WheelBodyVelocity wheelBodyVelocity;
} slBus6_Linear;
typedef struct 
{
  WheelBodyAngularVelocity wheelBodyAngularVelocity;
  WheelAngularVelocity wheelAngularVelocity;
} slBus7_Angular;
typedef struct 
{
  real_T z_FL_Wheel_dt_m_s;
  real_T z_FR_Wheel_dt_m_s;
  real_T z_RL_Wheel_dt_m_s;
  real_T z_RR_Wheel_dt_m_s;
} z_Wheel_dt_FL_FR_RL_RR_m_s;
typedef struct 
{
  Pos_Vehicle_CoorSys_E_x_y_z_m pos_Vehicle_CoorSys_E_x_y_z_m;
  real_T pos_Vehicle_CoorSys_E_m[3];
  real_T s_Vehicle_m;
} slBus9_Linear;
typedef struct 
{
  Angle_Vehicle_CoorSys_E_x_y_z_deg angle_Vehicle_CoorSys_E_x_y_z_deg;
  real_T angle_V_Alpha_Beta_Gamma_deg[3];
} slBus10_Angular;
typedef struct 
{
  real_T pos_FL_Wheels_CoorSys_E_x_y_z_m[3];
  real_T pos_FR_Wheels_CoorSys_E_x_y_z_m[3];
  real_T pos_RL_Wheels_CoorSys_E_x_y_z_m[3];
  real_T pos_RR_Wheels_CoorSys_E_x_y_z_m[3];
} slBus12_Linear;
typedef struct 
{
  real_T r_FL_Tire_m;
  real_T pos_CP_FL_CoorSys_E_x_y_m[2];
} ContactPoint;
typedef struct 
{
  F_FL_Tire_CoorSys_V_x_y_z_N f_FL_Tire_CoorSys_V_x_y_z_N;
  Trq_FL_Tire_CoorSys_V_x_y_z_Nm trq_FL_Tire_CoorSys_V_x_y_z_Nm;
} TireForcesAndTorques;
typedef struct 
{
  real_T v_Circumf_Dyn_FL_Wheel_km_h;
  real_T slip_x_FL;
  real_T slip_y_FL;
  real_T r_Dyn_FL_Tire_m;
  real_T corneringStiffness_FL_N;
  real_T coef_RollingRes_FL;
  real_T trq_x_FL_CoorSys_CP_Nm;
  real_T trq_y_FL_CoorSys_W_Nm;
  real_T trq_z_FL_CoorSys_CP_Nm;
  real_T f_x_Dyn_FL_Tire_CoorSys_CP_N;
  real_T f_y_Dyn_FL_Tire_CoorSys_CP_N;
  real_T f_z_Dyn_FL_Tire_CoorSys_CP_N;
} ActiveTire;
typedef struct 
{
  real_T r_FR_Tire_m;
  real_T pos_CP_FR_CoorSys_E_x_y_m[2];
} slBus14_ContactPoint;
typedef struct 
{
  F_FR_Tire_CoorSys_V_x_y_z_N f_FR_Tire_CoorSys_V_x_y_z_N;
  Trq_FR_Tire_CoorSys_V_x_y_z_Nm trq_FR_Tire_CoorSys_V_x_y_z_Nm;
} slBus15_TireForcesAndTorques;
typedef struct 
{
  real_T v_Circumf_Dyn_FR_Wheel_km_h;
  real_T slip_x_FR;
  real_T slip_y_FR;
  real_T r_Dyn_FR_Tire_m;
  real_T corneringStiffness_FR_N;
  real_T coef_RollingRes_FR;
  real_T trq_x_FR_CoorSys_CP_Nm;
  real_T trq_y_FR_CoorSys_W_Nm;
  real_T trq_z_FR_CoorSys_CP_Nm;
  real_T f_x_Dyn_FR_Tire_CoorSys_CP_N;
  real_T f_y_Dyn_FR_Tire_CoorSys_CP_N;
  real_T f_z_Dyn_FR_Tire_CoorSys_CP_N;
} slBus16_ActiveTire;
typedef struct 
{
  real_T r_RL_Tire_m;
  real_T pos_CP_RL_CoorSys_E_x_y_m[2];
} slBus17_ContactPoint;
typedef struct 
{
  F_RL_Tire_CoorSys_V_x_y_z_N f_RL_Tire_CoorSys_V_x_y_z_N;
  Trq_RL_Tire_CoorSys_V_x_y_z_Nm trq_RL_Tire_CoorSys_V_x_y_z_Nm;
} slBus18_TireForcesAndTorques;
typedef struct 
{
  real_T v_Circumf_Dyn_RL_Wheel_km_h;
  real_T slip_x_RL;
  real_T slip_y_RL;
  real_T r_Dyn_RL_Tire_m;
  real_T corneringStiffness_RL_N;
  real_T coef_RollingRes_RL;
  real_T trq_x_RL_CoorSys_CP_Nm;
  real_T trq_y_RL_CoorSys_W_Nm;
  real_T trq_z_RL_CoorSys_CP_Nm;
  real_T f_x_Dyn_RL_Tire_CoorSys_CP_N;
  real_T f_y_Dyn_RL_Tire_CoorSys_CP_N;
  real_T f_z_Dyn_RL_Tire_CoorSys_CP_N;
} slBus19_ActiveTire;
typedef struct 
{
  real_T r_RR_Tire_m;
  real_T pos_CP_RR_CoorSys_E_x_y_m[2];
} slBus20_ContactPoint;
typedef struct 
{
  F_RR_Tire_CoorSys_V_x_y_z_N f_RR_Tire_CoorSys_V_x_y_z_N;
  Trq_RR_Tire_CoorSys_V_x_y_z_Nm trq_RR_Tire_CoorSys_V_x_y_z_Nm;
} slBus21_TireForcesAndTorques;
typedef struct 
{
  real_T v_Circumf_Dyn_RR_Wheel_km_h;
  real_T slip_x_RR;
  real_T slip_y_RR;
  real_T r_Dyn_RR_Tire_m;
  real_T corneringStiffness_RR_N;
  real_T coef_RollingRes_RR;
  real_T trq_x_RR_CoorSys_CP_Nm;
  real_T trq_y_RR_CoorSys_W_Nm;
  real_T trq_z_RR_CoorSys_CP_Nm;
  real_T f_x_Dyn_RR_Tire_CoorSys_CP_N;
  real_T f_y_Dyn_RR_Tire_CoorSys_CP_N;
  real_T f_z_Dyn_RR_Tire_CoorSys_CP_N;
} slBus22_ActiveTire;
typedef struct 
{
  real_T p_FL_Brake_bar;
  real_T p_FR_Brake_bar;
  real_T p_RL_Brake_bar;
  real_T p_RR_Brake_bar;
} Pressure;
typedef struct 
{
  real_T f_OutputPoint_N;
  real_T p_OutputPoint_bar;
  real_T pos_BrakePedal_OutputPoint_perc;
  real_T f_Max_Pedal_N;
  real_T p_Max_Booster_bar;
} OutputPoint;
typedef struct 
{
  Position_Left position_Left;
  Orientation_Left orientation_Left;
  Spring_Left spring_Left;
  Damper_Left damper_Left;
} Kinematics_Left;
typedef struct 
{
  Position_Right position_Right;
  Orientation_Right orientation_Right;
  Spring_Right spring_Right;
  Damper_Right damper_Right;
} Kinematics_Right;
typedef struct 
{
  real_T displ_Stabilizer_m;
  real_T displ_Stabilizer_FL_m;
  real_T displ_Stabilizer_FR_m;
  real_T displ_Stabilizer_dq1_Front;
  real_T displ_Stabilizer_dq2_Front;
} Stabilizer;
typedef struct 
{
  real_T displ_x_Wheel_F_x_Left_mm;
  real_T displ_x_Wheel_F_x_Right_mm;
  real_T displ_y_Wheel_F_x_Left_mm;
  real_T displ_y_Wheel_F_x_Right_mm;
  real_T angle_Alpha_Wheel_F_x_Left_deg;
  real_T angle_Alpha_Wheel_F_x_Right_deg;
  real_T angle_Gamma_Wheel_F_x_Left_deg;
  real_T angle_Gamma_Wheel_F_x_Right_deg;
} ComplianceLongForce;
typedef struct 
{
  real_T displ_x_Wheel_F_y_Left_mm;
  real_T displ_x_Wheel_F_y_Right_mm;
  real_T displ_y_Wheel_F_y_Left_mm;
  real_T displ_y_Wheel_F_y_Right_mm;
  real_T angle_Alpha_Wheel_F_y_Left_deg;
  real_T angle_Alpha_Wheel_F_y_Right_deg;
  real_T angle_Gamma_Wheel_F_y_Left_deg;
  real_T angle_Gamma_Wheel_F_y_Right_deg;
} ComplianceLatForce;
typedef struct 
{
  real_T displ_x_Wheel_Trq_z_Left_mm;
  real_T displ_x_Wheel_Trq_z_Right_mm;
  real_T displ_y_Wheel_Trq_z_Left_mm;
  real_T displ_y_Wheel_Trq_z_Right_mm;
  real_T angle_Alpha_Wheel_Trq_z_Left_deg;
  real_T angle_Alpha_Wheel_Trq_z_Right_deg;
  real_T angle_Gamma_Wheel_Trq_z_Left_deg;
  real_T angle_Gamma_Wheel_Trq_z_Right_deg;
} ComplianceAligningTorque;
typedef struct 
{
  real_T displ_x_Wheel_Trq_x_Left_mm;
  real_T displ_x_Wheel_Trq_x_Right_mm;
  real_T displ_y_Wheel_Trq_x_Left_mm;
  real_T displ_y_Wheel_Trq_x_Right_mm;
  real_T angle_Alpha_Wheel_Trq_x_Left_deg;
  real_T angle_Alpha_Wheel_Trq_x_Right_deg;
  real_T angle_Gamma_Wheel_Trq_x_Left_deg;
  real_T angle_Gamma_Wheel_Trq_x_Right_deg;
} ComplianceLongTorque;
typedef struct 
{
  real_T pos_FL_Wheel_x_y_z_m[3];
  real_T pos_FR_Wheel_x_y_z_m[3];
} Pos_Wheel_FL_FR_x_y_z_m;
typedef struct 
{
  real_T unitVec_y_FL_Wheel_CoorSys_V_x_y_z[3];
  real_T unitVec_y_FR_Wheel_CoorSys_V_x_y_z[3];
} UnitVec_y_Wheel_CoorSys_V_FL_FR_x_y_z_;
typedef struct 
{
  Angle_FL_Wheel_Toe_Camber_deg angle_FL_Wheel_Toe_Camber_deg;
  Angle_FR_Wheel_Toe_Camber_deg angle_FR_Wheel_Toe_Camber_deg;
} Angle_Wheel_FL_FR_Toe_Camber_deg;
typedef struct 
{
  slBus24_Position_Left position_Left;
  slBus25_Orientation_Left orientation_Left;
  slBus26_Spring_Left spring_Left;
  slBus27_Damper_Left damper_Left;
} slBus28_Kinematics_Left;
typedef struct 
{
  slBus29_Position_Right position_Right;
  slBus30_Orientation_Right orientation_Right;
  slBus31_Spring_Right spring_Right;
  slBus32_Damper_Right damper_Right;
} slBus33_Kinematics_Right;
typedef struct 
{
  real_T displ_Stabilizer_m;
  real_T displ_Stabilizer_RL_m;
  real_T displ_Stabilizer_RR_m;
  real_T displ_Stabilizer_dq1_Rear;
  real_T displ_Stabilizer_dq2_Rear;
} slBus34_Stabilizer;
typedef struct 
{
  real_T pos_RL_Wheel_x_y_z_m[3];
  real_T pos_RR_Wheel_x_y_z_m[3];
} Pos_Wheel_RL_RR_x_y_z_m;
typedef struct 
{
  real_T unitVec_y_RL_Wheel_CoorSys_V_x_y_z[3];
  real_T unitVec_y_RR_Wheel_CoorSys_V_x_y_z[3];
} UnitVec_y_Wheel_CoorSys_V_RL_RR_x_y_z_;
typedef struct 
{
  Angle_RL_Wheel_Toe_Camber_deg angle_RL_Wheel_Toe_Camber_deg;
  Angle_RR_Wheel_Toe_Camber_deg angle_RR_Wheel_Toe_Camber_deg;
} Angle_Wheel_RL_RR_Toe_Camber_deg;
typedef struct 
{
  real_T sfunc_State;
  real_T data_Valid_0_1;
  real_T flag_Sensor_Init_Ready_0_1;
} slBus1_Info;
typedef struct 
{
  real_T collision_Flag_0NoCollision_1Collision;
  real_T collision_Flag_Held_0NoCollision_1Collision;
  real_T object_ID[5];
  real_T time_Collision_s[5];
  real_T pos_Collision_Point_x_y_m[10];
  real_T v_Collision_CoG_Vehicle_x_y_m_s[10];
  real_T v_Collision_CoG_Vehicle_Total_m_s[5];
  real_T v_Rel_Collision_Point_x_y_m_s[10];
  real_T v_Rel_Collision_Point_Total_m_s[5];
} SensorOutput;
typedef struct 
{
  Linear linear;
  Angular angular;
} VehicleCoG;
typedef struct 
{
  slBus3_Linear linear;
} Wheels;
typedef struct 
{
  slBus4_Linear linear;
  slBus5_Angular angular;
} Vehicle;
typedef struct 
{
  slBus6_Linear linear;
  slBus7_Angular angular;
} slBus8_Wheels;
typedef struct 
{
  real_T v_Vehicle_x_y_z_km_h[3];
  real_T w_Vehicle_x_y_z_deg_s[3];
  z_Wheel_dt_FL_FR_RL_RR_m_s z_Wheel_dt_FL_FR_RL_RR_m_s_var;
} Generalized;
typedef struct 
{
  slBus9_Linear linear;
  slBus10_Angular angular;
} slBus11_Vehicle;
typedef struct 
{
  slBus12_Linear linear;
} slBus13_Wheels;
typedef struct 
{
  real_T g_F_Vehicle_x_y_z_N[3];
  real_T g_Trq_Vehicle_x_y_z_N[3];
  real_T g_F_z_Wheel_FL_FR_RL_RR_N[4];
} GeneralizedForcesAndTorques;
typedef struct 
{
  real_T v_Vehicle_dt_x_y_z_m_s2[3];
  real_T w_Vehicle_dt_x_y_z_rad_s2[3];
  real_T q_Wheel_dt2_q1_Front_q2_Front_q1_Rear_q2_Rear[4];
} GeneralizedVelocitiesDerivative;
typedef struct 
{
  real_T a_Angular_Sensor1_CoorSys_V_x_y_z_rad_s2[3];
  real_T a_Total_Sensor1_CoorSys_V_x_y_z_m_s2[3];
  real_T a_Sensor1_CoorSys_V_x_y_z_m_s2[3];
  real_T v_Sensor1_CoorSys_V_x_y_z_m_s[3];
  real_T w_Sensor1_CoorSys_V_x_y_z_rad_s[3];
  real_T a_Angular_Sensor1_CoorSys_VH_x_y_z_rad_s2[3];
  real_T a_Total_Sensor1_CoorSys_VH_x_y_z_m_s2[3];
  real_T a_Sensor1_CoorSys_VH_x_y_z_m_s2[3];
  real_T v_Sensor1_CoorSys_VH_x_y_z_m_s[3];
  real_T w_Sensor1_CoorSys_VH_x_y_z_rad_s[3];
} Sensor1;
typedef struct 
{
  real_T a_Angular_Sensor2_CoorSys_V_x_y_z_rad_s2[3];
  real_T a_Total_Sensor2_CoorSys_V_x_y_z_m_s2[3];
  real_T a_Sensor2_CoorSys_V_x_y_z_m_s2[3];
  real_T v_Sensor2_CoorSys_V_x_y_z_m_s[3];
  real_T w_Sensor2_CoorSys_V_x_y_z_rad_s[3];
  real_T a_Angular_Sensor2_CoorSys_VH_x_y_z_rad_s2[3];
  real_T a_Total_Sensor2_CoorSys_VH_x_y_z_m_s2[3];
  real_T a_Sensor2_CoorSys_VH_x_y_z_m_s2[3];
  real_T v_Sensor2_CoorSys_VH_x_y_z_m_s[3];
  real_T w_Sensor2_CoorSys_VH_x_y_z_rad_s[3];
} Sensor2;
typedef struct 
{
  real_T a_Angular_Sensor3_CoorSys_V_x_y_z_rad_s2[3];
  real_T a_Total_Sensor3_CoorSys_V_x_y_z_m_s2[3];
  real_T a_Sensor3_CoorSys_V_x_y_z_m_s2[3];
  real_T v_Sensor3_CoorSys_V_x_y_z_m_s[3];
  real_T w_Sensor3_CoorSys_V_x_y_z_rad_s[3];
  real_T a_Angular_Sensor3_CoorSys_VH_x_y_z_rad_s2[3];
  real_T a_Total_Sensor3_CoorSys_VH_x_y_z_m_s2[3];
  real_T a_Sensor3_CoorSys_VH_x_y_z_m_s2[3];
  real_T v_Sensor3_CoorSys_VH_x_y_z_m_s[3];
  real_T w_Sensor3_CoorSys_VH_x_y_z_rad_s[3];
} Sensor3;
typedef struct 
{
  real_T a_Angular_Sensor4_CoorSys_V_x_y_z_rad_s2[3];
  real_T a_Total_Sensor4_CoorSys_V_x_y_z_m_s2[3];
  real_T a_Sensor4_CoorSys_V_x_y_z_m_s2[3];
  real_T v_Sensor4_CoorSys_V_x_y_z_m_s[3];
  real_T w_Sensor4_CoorSys_V_x_y_z_rad_s[3];
  real_T a_Angular_Sensor4_CoorSys_VH_x_y_z_rad_s2[3];
  real_T a_Total_Sensor4_CoorSys_VH_x_y_z_m_s2[3];
  real_T a_Sensor4_CoorSys_VH_x_y_z_m_s2[3];
  real_T v_Sensor4_CoorSys_VH_x_y_z_m_s[3];
  real_T w_Sensor4_CoorSys_VH_x_y_z_rad_s[3];
} Sensor4;
typedef struct 
{
  real_T f_Vehicle_Gravitational_x_y_z_N[3];
  real_T f_FL_Gravitational_x_y_z_N[3];
  real_T f_FR_Gravitational_x_y_z_N[3];
  real_T f_RL_Gravitational_x_y_z_N[3];
  real_T f_RR_Gravitational_x_y_z_N[3];
} GravitationalForces;
typedef struct 
{
  real_T f_Vehicle_Gyroscopic_x_y_z_N[3];
  real_T f_FL_Gyroscopic_x_y_z_N[3];
  real_T f_FR_Gyroscopic_x_y_z_N[3];
  real_T f_RL_Gyroscopic_x_y_z_N[3];
  real_T f_RR_Gyroscopic_x_y_z_N[3];
} GyroscopicForces;
typedef struct 
{
  real_T trq_Vehicle_Gyroscope_x_y_z_Nm[3];
  real_T trq_FL_Gyroscope_x_y_z_Nm[3];
  real_T trq_FR_Gyroscope_x_y_z_Nm[3];
  real_T trq_RL_Gyroscope_x_y_z_Nm[3];
  real_T trq_RR_Gyroscope_x_y_z_Nm[3];
} GyroscopicTorques;
typedef struct 
{
  real_T a_Vehicle_CoG_Residual_x_y_z_m_s2[3];
  real_T a_FL_Residual_x_y_z_m_s2[3];
  real_T a_FR_Residual_x_y_z_m_s2[3];
  real_T a_RL_Residual_x_y_z_m_s2[3];
  real_T a_RR_Residual_x_y_z_m_s2[3];
} ResidualAccelerations;
typedef struct 
{
  ContactPoint contactPoint;
  TireForcesAndTorques tireForcesAndTorques;
  ActiveTire activeTire;
} Tire_FL;
typedef struct 
{
  slBus14_ContactPoint contactPoint;
  slBus15_TireForcesAndTorques tireForcesAndTorques;
  slBus16_ActiveTire activeTire;
} Tire_FR;
typedef struct 
{
  slBus17_ContactPoint contactPoint;
  slBus18_TireForcesAndTorques tireForcesAndTorques;
  slBus19_ActiveTire activeTire;
} Tire_RL;
typedef struct 
{
  slBus20_ContactPoint contactPoint;
  slBus21_TireForcesAndTorques tireForcesAndTorques;
  slBus22_ActiveTire activeTire;
} Tire_RR;
typedef struct 
{
  real_T sw_Tire_Model_1TMEasy_2MF;
} TireModelSwitch;
typedef struct 
{
  real_T p_bar;
  real_T p_max_bar;
  real_T enable_p_Brake_Desired_0_1;
  real_T p_Brake_Desired_bar;
} ActiveBrakeCylinder;
typedef struct 
{
  real_T p_bar;
  real_T p_max_bar;
} BrakeCylinderLinear;
typedef struct 
{
  real_T displ_Piston_mm;
  real_T f_N;
  real_T p_bar;
  real_T p_max_bar;
} BrakeCylinderPhysical;
typedef struct 
{
  Pressure pressure;
} BrakingCircuit;
typedef struct 
{
  real_T trq_FL_Brake_Nm;
  real_T trq_FR_Brake_Nm;
  real_T trq_RL_Brake_Nm;
  real_T trq_RR_Brake_Nm;
} Torque;
typedef struct 
{
  real_T trq_Max_Brake_Nm;
  real_T trq_Max_Brake_FL_FR_RL_RR_Nm[4];
  real_T trq_FL_max_Brake_Nm;
  real_T trq_FR_max_Brake_Nm;
  real_T trq_RL_max_Brake_Nm;
  real_T trq_RR_max_Brake_Nm;
} MaximumTorque;
typedef struct 
{
  real_T const_BrakeDiscFactor_FL_m3;
  real_T const_BrakeDiscFactor_FR_m3;
  real_T const_BrakeDiscFactor_RL_m3;
  real_T const_BrakeDiscFactor_RR_m3;
} Const_BrakeDiscFactor;
typedef struct 
{
  real_T p_In_MainfoldValve_bar;
  real_T p_Out_MainfoldValve_bar;
  real_T t_In_MainfoldValve_degC;
  real_T m_dot_ManifoldValve_kg_s;
  real_T m_dot_Out_ManifoldValve_kg_s;
  real_T m_dot_In_ManifoldValve_kg_s;
} ManifoldValve;
typedef struct 
{
  real_T m_dot_In_1_VacuumChamber_kg_s;
  real_T m_dot_In_2_VacuumChamber_kg_s;
  real_T m_dot_Out_2_VacuumChamber_kg_s;
  real_T t_VacuumChamber_degC;
  real_T p_VacuumChamber_bar;
  real_T v_VacuumChamber_m3;
} VacuumChamber;
typedef struct 
{
  real_T valveState_1ChamberCon_2AmbientCon_3CloseAll;
  real_T p_In_BellValve_bar;
  real_T p_Out_BellValve_bar;
  real_T t_In_BellValve_degC;
  real_T m_dot_BellValve_kg_s;
  real_T ctrl_BellValve_0_1;
} BellValve;
typedef struct 
{
  real_T m_dot_In_RearChamber_kg_s;
  real_T m_dot_Out_RearChamber_kg_s;
  real_T t_RearChamber_degC;
  real_T p_RearChamber_bar;
  real_T v_RearChamber_m3;
} RearChamber;
typedef struct 
{
  real_T f_Pedal_N;
  real_T f_Spring_N;
  real_T f_RearChamber_N;
  real_T f_VacuumChamber_N;
  real_T f_PushRod_N;
  real_T x_m;
  real_T v_delta_m3;
  real_T a_Diaphragm_m2;
  OutputPoint outputPoint;
} PushRod;
typedef struct 
{
  real_T p_Setpoint_bar;
  real_T p_MasterBrakeCyl_bar;
  real_T p_delta_bar;
  real_T valveState_1ChamberCon_2AmbientCon;
  real_T ctrl_A_BellValve_0_1;
  real_T f_Pedal_N;
} ControllerBellValve;
typedef struct 
{
  Kinematics_Left kinematics_Left;
  Kinematics_Right kinematics_Right;
  Stabilizer stabilizer;
  real_T susKinFrontModel;
} SuspensionKinematicsFront;
typedef struct 
{
  real_T angle_Comp_FR_Wheel_dt_x_y_z_m[3];
  real_T angle_Comp_FL_Wheel_dt_x_y_z_m[3];
  real_T angle_Comp_FR_Wheel_x_y_z_rad[3];
  real_T angle_Comp_FL_Wheel_x_y_z_rad[3];
  real_T displ_Comp_FR_Wheel_dt_x_y_z_m[3];
  real_T displ_Comp_FL_Wheel_dt_x_y_z_m[3];
  real_T displ_Comp_FR_Wheel_x_y_z_m[3];
  real_T displ_Comp_FL_Wheel_x_y_z_m[3];
  ComplianceLongForce complianceLongForce;
  ComplianceLatForce complianceLatForce;
  ComplianceAligningTorque complianceAligningTorque;
  ComplianceLongTorque complianceLongTorque;
} SuspensionComplianceFront;
typedef struct 
{
  real_T f_FL_Spring_N;
  real_T f_FL_Spring_dDispl_FL_Spring_N_m;
  real_T f_FL_Damper_N;
  real_T f_FL_Damper_dDispl_FL_Damper_dt_Ns_m;
  real_T f_Stabilizer_N;
  real_T f_Stabilizer_dDispl_Stabilizer_N_m;
  real_T f_FR_Spring_N;
  real_T f_FR_Spring_dDispl_FR_Spring_N_m;
  real_T f_FR_Damper_N;
  real_T f_FR_Damper_dDispl_FR_Damper_dt_Ns_m;
} SuspensionForcesFront;
typedef struct 
{
  Pos_Wheel_FL_FR_x_y_z_m pos_Wheel_FL_FR_x_y_z_m;
  UnitVec_y_Wheel_CoorSys_V_FL_FR_x_y_z_ unitVec_y_Wheel_CoorSys_V_FL_FR_x_y_z_;
  Angle_Wheel_FL_FR_Toe_Camber_deg angle_Wheel_FL_FR_Toe_Camber_deg;
} RelativePositionFront;
typedef struct 
{
  slBus28_Kinematics_Left kinematics_Left;
  slBus33_Kinematics_Right kinematics_Right;
  slBus34_Stabilizer stabilizer;
  real_T susKinRearModel;
} SuspensionKinematicsRear;
typedef struct 
{
  real_T angle_Comp_RR_Wheel_dt_x_y_z_m[3];
  real_T angle_Comp_RL_Wheel_dt_x_y_z_m[3];
  real_T angle_Comp_RR_Wheel_x_y_z_rad[3];
  real_T angle_Comp_RL_Wheel_x_y_z_rad[3];
  real_T displ_Comp_RR_Wheel_dt_x_y_z_m[3];
  real_T displ_Comp_RL_Wheel_dt_x_y_z_m[3];
  real_T displ_Comp_RR_Wheel_x_y_z_m[3];
  real_T displ_Comp_RL_Wheel_x_y_z_m[3];
  ComplianceLongForce complianceLongForce;
  ComplianceLatForce complianceLatForce;
  ComplianceAligningTorque complianceAligningTorque;
  ComplianceLongTorque complianceLongTorque;
} SuspensionComplianceRear;
typedef struct 
{
  real_T f_RL_Spring_N;
  real_T f_RL_Spring_dDispl_RL_Spring_N_m;
  real_T f_RL_Damper_N;
  real_T f_RL_Damper_dDispl_RL_Damper_dt_Ns_m;
  real_T f_Stabilizer_N;
  real_T f_Stabilizer_dDispl_Stabilizer_N_m;
  real_T f_RR_Spring_N;
  real_T f_RR_Spring_dDispl_RR_Spring_N_m;
  real_T f_RR_Damper_N;
  real_T f_RR_Damper_dDispl_RR_Damper_dt_Ns_m;
} SuspensionForcesRear;
typedef struct 
{
  Pos_Wheel_RL_RR_x_y_z_m pos_Wheel_RL_RR_x_y_z_m;
  UnitVec_y_Wheel_CoorSys_V_RL_RR_x_y_z_ unitVec_y_Wheel_CoorSys_V_RL_RR_x_y_z_;
  Angle_Wheel_RL_RR_Toe_Camber_deg angle_Wheel_RL_RR_Toe_Camber_deg;
} RelativePositionRear;
typedef struct 
{
  real_T trq_Fric_Clutch_Nm;
  real_T trq_TorsionSpring_Clutch_Nm;
  real_T omega_Out_ClutchPlate_rad_s;
} Clutch;
typedef struct 
{
  real_T i_unsync_Trm;
  real_T i_Trm;
  real_T trq_Out_Trm_Nm;
  real_T omega_In_Trm_rad_s;
  real_T inertia_Trm_kgm2;
  real_T omega_Out_Trm_rad_s;
} GearBox;
typedef struct 
{
  real_T trq_Fric_Clutch_Nm;
  real_T trq_TorsionSpring_Clutch_Nm;
  real_T omega_Out_ClutchPlate_rad_s;
} LockUpClutch;
typedef struct 
{
  real_T trq_Pump_Nm;
  real_T trq_Turbine_Nm;
} TorqueConverter;
typedef struct 
{
  real_T reset_VehicleStates_0_1;
  real_T roadControl;
  real_T get_VehPos_Trigger;
  real_T keep_VehiclePosition_0_1;
} MdlControl;
typedef struct 
{
  real_T s_Vehicle_Init_m;
  real_T d_Vehicle_Init_m;
  real_T laneIdx_Vehicle_Init;
  real_T pos_z_Vehicle_Rel_Init_m;
  real_T angle_Yaw_Vehicle_Rel_Init_deg;
  real_T v_Vehicle_Init_km_h;
  real_T route_Id_Init;
  real_T route_Direction_Init;
} InitialVehStates;
typedef struct 
{
  real_T mode_AccBr_1Stim_2Driver;
  real_T mode_GearClutch_1Stim_2Driver_3OpenClutch_4RefGear;
  real_T mode_Steer_1Stim_2Driver_3Fix;
  real_T mode_Road_1NoRoad_2Straight_3Circle_4Road;
  real_T latDriver_1LatCtrl1_2LatCtrl2;
  real_T mode_LatCtrl_1Pos_2Yaw;
  real_T mode_AdaptVelocityToRoad_1Off_2On;
  real_T mode_ForceToRoad_1Off_2On;
  real_T mode_SelectorLever_1Stim_2Driver;
  real_T mode_CurvatureCalculation_1Road_2Driver;
  real_T mode_TrafficDriver_1Off_2On;
  real_T mode_RelativeLane_0CenterLane_1PreferredLane;
} ModeSignals;
typedef struct 
{
  real_T pos_AccPedal_Maneuver_perc;
  real_T pos_BrakePedal_Maneuver_perc;
  real_T pos_ClutchPedal_Maneuver_perc;
  real_T gear_Maneuver;
  real_T angle_SteeringWheel_Maneuver_deg;
  real_T v_Vehicle_Ref_Maneuver_m_s;
  real_T d_Maneuver_m;
  real_T laneIdx_Maneuver;
  real_T dist_Def_Maneuver[3];
  real_T v_Vehicle_Ref_Preview_Maneuver_m_s;
  real_T v_Vehicle_Ref_Preview_State_0Invalid_1CurrentSeg_2NextSeg;
  real_T sw_SteeringMode_Maneuver_1Angle_2Trq;
  real_T trq_Steering_Maneuver_Nm;
  real_T selectorLever_Maneuver;
  real_T dist_Def_Refline_Maneuver[4];
} RefSignals;
typedef struct 
{
  real_T a_x_Max_m_s2;
  real_T a_x_Brake_Threshold_m_s2;
  real_T a_y_Max_m_s2;
  real_T curv_Road_Circle_1_m;
  real_T gear_Max;
  real_T width_Deadzone_LatCtrl_m;
  real_T v_Lat_m_s;
  real_T object_Type_ID;
  real_T driver_Type_Index;
} Parameters;
typedef struct 
{
  real_T user1;
  real_T user2;
  real_T user3;
  real_T user4;
  real_T user5;
  real_T user6;
  real_T user7;
  real_T user8;
  real_T user9;
  real_T user10;
  real_T user11;
  real_T user12;
  real_T user13;
  real_T user14;
  real_T user15;
  real_T user16;
  real_T user17;
  real_T user18;
  real_T user19;
  real_T user20;
  real_T user21;
  real_T user22;
  real_T user23;
  real_T user24;
  real_T user25;
  real_T user26;
  real_T user27;
  real_T user28;
  real_T user29;
  real_T user30;
} UserSignals;
typedef struct 
{
  real_T currentSegment;
  real_T t_local_s;
  real_T s_local_m;
  real_T maneuverTime_s;
  real_T maneuverState;
  real_T errorFlag;
  real_T currentSequence;
} Info;
typedef struct 
{
  slBus1_Info info;
  SensorOutput sensorOutput;
} Collision_Sensor_1;
typedef struct 
{
  real_T pos_x_Fellows_m[30];
  real_T pos_y_Fellows_m[30];
  real_T pos_z_Fellows_m[30];
} Pos_Fellows_MainPnt_CoorSys_E_x_y_z_m;
typedef struct 
{
  real_T angle_Roll_Fellows_deg[30];
  real_T angle_Pitch_Fellows_deg[30];
  real_T angle_Yaw_Fellows_deg[30];
  real_T angle_Wheel_Fellows_deg[30];
} Angle_Fellows_x_y_z_deg;
typedef struct 
{
  real_T distance_xy_m[35];
  real_T distance_z_m[35];
} Dist_Response_Traffic;
typedef struct 
{
  real_T enable_p_Brake_Desired_0_1;
  real_T p_Brake_Desired_bar;
} DesiredBrakePressure;
typedef struct 
{
  VehicleCoG vehicleCoG;
  Wheels wheels;
} Accelerations;
typedef struct 
{
  Vehicle vehicle;
  slBus8_Wheels wheels;
  Generalized generalized;
} Velocities;
typedef struct 
{
  slBus11_Vehicle vehicle;
  slBus13_Wheels wheels;
} Positions;
typedef struct 
{
  GeneralizedForcesAndTorques generalizedForcesAndTorques;
  GeneralizedVelocitiesDerivative generalizedVelocitiesDerivative;
} VehicleEquationOfMotion;
typedef struct 
{
  real_T a_Gravity_CoorSys_V_x_y_z_m_s2[3];
} GravityAcceleration;
typedef struct 
{
  Sensor1 sensor1;
  Sensor2 sensor2;
  Sensor3 sensor3;
  Sensor4 sensor4;
} SensorMotion;
typedef struct 
{
  real_T m_TotalVehicle_kg;
  real_T posVec_CoG_TotalVehicle_x_y_z_m[3];
  real_T pos_x_CoG_Vehicle_Total_m;
  real_T inertia_z_Vehicle_Total_kgm2;
  real_T m_TotalV1_kg;
  real_T posVec_CoG_TotalV1_x_y_z_m[3];
} VehicleMassAndAdditionalLoads;
typedef struct 
{
  GravitationalForces gravitationalForces;
  GyroscopicForces gyroscopicForces;
  GyroscopicTorques gyroscopicTorques;
  ResidualAccelerations residualAccelerations;
} MassForcesAndTorques;
typedef struct 
{
  Tire_FL tire_FL;
  Tire_FR tire_FR;
  Tire_RL tire_RL;
  Tire_RR tire_RR;
  TireModelSwitch tireModelSwitch;
} Tire_allSignals;
typedef struct 
{
  real_T f_x_Aero_CoorSys_V_N;
  real_T f_y_Aero_CoorSys_V_N;
  real_T f_z_Aero_CoorSys_V_N;
} Forces;
typedef struct 
{
  real_T trq_x_Aero_CoorSys_V_Nm;
  real_T trq_y_Aero_CoorSys_V_Nm;
  real_T trq_z_Aero_CoorSys_V_Nm;
} Torques;
typedef struct 
{
  real_T cw_x;
} Coefficients;
typedef struct 
{
  real_T w_Wheel_FL_CoorSys_V_x_y_z_rad_s[3];
  real_T w_Wheel_FR_CoorSys_V_x_y_z_rad_s[3];
  real_T w_Wheel_RL_CoorSys_V_x_y_z_rad_s[3];
  real_T w_Wheel_RR_CoorSys_V_x_y_z_rad_s[3];
} WheelSpeed_CoorSys_V;
typedef struct 
{
  real_T trq_Shaft_FL_CoorSys_V_x_y_z_Nm[3];
  real_T trq_Shaft_FR_CoorSys_V_x_y_z_Nm[3];
  real_T trq_Shaft_RL_CoorSys_V_x_y_z_Nm[3];
  real_T trq_Shaft_RR_CoorSys_V_x_y_z_Nm[3];
} ShaftsTorques_CoorSys_V;
typedef struct 
{
  real_T trq_Tire_Whee_FL_Nm;
  real_T trq_Tire_Whee_FR_Nm;
  real_T trq_Tire_Whee_RL_Nm;
  real_T trq_Tire_Whee_RR_Nm;
} TireWheelTorques;
typedef struct 
{
  real_T omega_FL_Wheel_rad_s;
  real_T omega_FR_Wheel_rad_s;
  real_T omega_RL_Wheel_rad_s;
  real_T omega_RR_Wheel_rad_s;
} WheelSpeed;
typedef struct 
{
  real_T omega_Wheel_FL_FR_RL_RR_rad_s[4];
  real_T omega_Mean_Wheel_rad_s;
} WheelSpeed2;
typedef struct 
{
  real_T angle_Rot_FL_Wheel_deg;
  real_T angle_Rot_FR_Wheel_deg;
  real_T angle_Rot_RL_Wheel_deg;
  real_T angle_Rot_RR_Wheel_deg;
} RotationAngle;
typedef struct 
{
  real_T g_F_Steering_SteeringRod_N;
  real_T trq_Steering_Nm;
  real_T trq_PowerSteering_Nm;
  real_T trqSpring_SteeringColumn_Nm;
  real_T trq_PowerSteeringDynamics_Nm;
  real_T trqSpring_PowerSteeringSystem_Nm;
} ForcesAndTorques;
typedef struct 
{
  real_T g_FTrq_Tire_Steering_N;
  real_T g_Trq_FL_Steering_N;
  real_T g_Trq_FR_Steering_N;
  real_T g_F_FL_Steering_N;
  real_T g_F_FR_Steering_N;
} slBus23_GeneralizedForcesAndTorques;
typedef struct 
{
  real_T angle_EPSSystem_deg;
  real_T angle_EPSSystem_dt_deg_s;
  real_T angle_LowerSteeringColumn_deg;
  real_T angle_LowerSteeringColumn_dt_deg_s;
  real_T angle_UpperSteeringColumn_deg;
  real_T angle_UpperSteeringColumn_dt_deg_s;
  real_T angle_SteeringWheel_deg;
  real_T angle_SteeringWheel_dt_deg_s;
  real_T displ_Steering_m;
  real_T displ_Steering_dt_m_s;
  real_T displ_SteeringRod_dt2_m_s2;
  real_T angle_SteeringGear_deg;
  real_T angle_SteeringGear_dt_deg_s;
  real_T angle_EPSPinion_rad;
  real_T angle_EPSPinion_dt_rad_s;
  real_T i_Steering_rad_m;
  real_T frictionForce_SteeringRod_N;
  real_T i_Steering_Wheel2Rod_rad_m;
} StatesSteeringSystem;
typedef struct 
{
  real_T trqSensor_Steering_Nm;
  real_T trq_Driver_Nm;
  real_T g_torque_SteeringWheel_Nm;
  real_T trqFriction_SteeringWheel_Nm;
  real_T trq_SteeringWheel_Nm;
} SteeringWheelTorque;
typedef struct 
{
  ActiveBrakeCylinder activeBrakeCylinder;
  BrakeCylinderLinear brakeCylinderLinear;
  BrakeCylinderPhysical brakeCylinderPhysical;
} MasterBrakeCylinder;
typedef struct 
{
  BrakingCircuit brakingCircuit;
  real_T brakeHydraulicsModel;
} BrakeHydraulics;
typedef struct 
{
  Torque torque;
  MaximumTorque maximumTorque;
  Pressure pressure;
  Const_BrakeDiscFactor const_BrakeDiscFactor;
} BrakeDisc;
typedef struct 
{
  ManifoldValve manifoldValve;
  VacuumChamber vacuumChamber;
  BellValve bellValve;
  RearChamber rearChamber;
  PushRod pushRod;
  ControllerBellValve controllerBellValve;
} VacuumServo;
typedef struct 
{
  real_T pos_AccPedal_perc;
  real_T p_Manifold_bar;
} Manifold;
typedef struct 
{
  SuspensionKinematicsFront suspensionKinematicsFront;
  SuspensionComplianceFront suspensionComplianceFront;
  SuspensionForcesFront suspensionForcesFront;
  RelativePositionFront relativePositionFront;
} SuspensionFront;
typedef struct 
{
  SuspensionKinematicsRear suspensionKinematicsRear;
  SuspensionComplianceRear suspensionComplianceRear;
  SuspensionForcesRear suspensionForcesRear;
  RelativePositionRear relativePositionRear;
} SuspensionRear;
typedef struct 
{
  real_T eta_Eff_Engine_0_1;
  real_T eta_Ind_Engine_0_1;
  real_T eta_Mechanical_Engine_0_1;
} Efficiency;
typedef struct 
{
  real_T cO2_Emission_kg_h;
  real_T cO2_Emission_Total_kg;
} CO2_Emission;
typedef struct 
{
  Clutch clutch;
  GearBox gearBox;
} Manual;
typedef struct 
{
  LockUpClutch lockUpClutch;
  GearBox gearBox;
  TorqueConverter torqueConverter;
} Automatic;
typedef struct 
{
  real_T i_unsync_Trm;
  real_T i_Trm;
  real_T trq_Out_Trm_Nm;
  real_T omega_In_Trm_rad_s;
  real_T inertia_Trm_kgm2;
  real_T omega_Out_Trm_rad_s;
  real_T gear;
} slBus39_OutputSignals;
typedef struct 
{
  real_T omega_Out_Front_CentralDiff_rad_s;
  real_T omega_Out_Rear_CentralDiff_rad_s;
  real_T omega_In_CentralDiff_rad_s;
  real_T trq_Lock_Nm;
  real_T omega_Cage_rad_s;
  real_T const_i_MainReductionGear;
  real_T const_TrqDistribution;
} CentralDifferential;
typedef struct 
{
  real_T omega_Out_Lef_FrontDifferentialt_rad_s;
  real_T omega_Out_Right_FrontDifferential_rad_s;
  real_T omega_In_CentralDiff_FrontDifferential_rad_s;
  real_T trq_Lock_FrontDifferential_Nm;
  real_T omega_Cage_rad_s;
  real_T const_i_MainReductionGear;
  real_T const_TrqDistribution;
} FrontDifferential;
typedef struct 
{
  real_T omega_Out_Lef_RearDifferentialt_rad_s;
  real_T omega_Out_Right_RearDifferential_rad_s;
  real_T omega_In_CentralDiff_RearDifferential_rad_s;
  real_T trq_Lock_RearDifferential_Nm;
  real_T omega_Cage_rad_s;
  real_T const_i_MainReductionGear;
  real_T const_TrqDistribution;
} RearDifferential;
typedef struct 
{
  real_T trq_TGCL4WD_Shaft_Nm;
  real_T omega_Delta_TGCL4WD_Shaft_rad_s;
} Shaft_TGCL4WD;
typedef struct 
{
  real_T trq_CF_Shaft_Nm;
  real_T omega_Delta_CF_Shaft_rad_s;
} Shaft_CF;
typedef struct 
{
  real_T trq_CR_Shaft_Nm;
  real_T omega_Delta_CR_Shaft_rad_s;
} Shaft_CR;
typedef struct 
{
  real_T trq_In_Transfer_Gearbox_Nm;
  real_T omega_Out_Transfer_Gearbox_rad_s;
} TransferGearbox;
typedef struct 
{
  real_T trq_Lock_Clutch_4WD_Nm;
  real_T omega_Clutch_4WD_In_rad_s;
  real_T omega_Clutch_4WD_Out_rad_s;
} Clutch_4WD;
typedef struct 
{
  real_T trq_FL_Shaft_Nm;
  real_T omega_Delta_FL_Shaft_rad_s;
} Shaft_FL;
typedef struct 
{
  real_T trq_FR_Shaft_Nm;
  real_T omega_Delta_FR_Shaft_rad_s;
} Shaft_FR;
typedef struct 
{
  real_T trq_RL_Shaft_Nm;
  real_T omega_Delta_RL_Shaft_rad_s;
} Shaft_RL;
typedef struct 
{
  real_T trq_RR_Shaft_Nm;
  real_T omega_Delta_RR_Shaft_rad_s;
} Shaft_RR;
typedef struct 
{
  MdlControl mdlControl;
  InitialVehStates initialVehStates;
  ModeSignals modeSignals;
  RefSignals refSignals;
  Parameters parameters;
  UserSignals userSignals;
  Info info;
} ManeuverScheduler;
typedef struct 
{
  real_T externalSegmentEnd_Mode;
  real_T externalSegmentEnd_Value;
  real_T aSM_Vehicle_TurnLights;
  real_T eCU_Commands;
  real_T mnv_DataStream_5;
  real_T mnv_DataStream_6;
  real_T mnv_DataStream_7;
  real_T mnv_DataStream_8;
} UserSignals_Customized;
typedef struct 
{
  real_T sw_StartButton_0Off_1On;
  real_T mode_StartButtonState_1Stim_2SoftECU;
  real_T state_StartButton_1PowerOff_0Acc_1PowerOn_2StarterOn;
} StartButton;
typedef struct 
{
  real_T sw_ParkingBrake_0Off_1On;
} ParkingBrake;
typedef struct 
{
  Collision_Sensor_1 collision_Sensor_1;
} Collision_Sensor;
typedef struct 
{
  real_T a_Fellows_Ref_Scheduler_m_s2[30];
  real_T userSignals_Fellows[60];
  real_T d_Total_Fellows_m[30];
  real_T laneIdx_Total_Fellows[30];
  real_T v_Fellows_km_h[30];
  Pos_Fellows_MainPnt_CoorSys_E_x_y_z_m pos_Fellows_MainPnt_CoorSys_E_x_y_z_m;
  Angle_Fellows_x_y_z_deg angle_Fellows_x_y_z_deg;
  real_T angle_Wheel_Fellows_deg[30];
  real_T v_Fellows_x_y_z_km_h[90];
  real_T w_Fellows_x_y_z_deg_s[90];
  real_T dist_Refline_Response_m;
  real_T a_x_Vehicle_Ref_m_s2;
  real_T flag_FellowUsed_0_1[30];
  real_T dist_Def_Trf[105];
  real_T route_Id_Fellows_Ref_Scheduler[30];
  real_T direction_Fellows_Ref_Scheduler_1_1[30];
  real_T v_Lat_Fellows_Ref_Scheduler_m_s[30];
  real_T forceToRoad_Fellows_Ref_Scheduler_0_1[30];
  real_T relativeLane_Mode_Fellows_0CenterLane_1PreferredLane[30];
  real_T laneIdx_Fellows_Ref_Scheduler[30];
  real_T v_Fellows_Ref_Scheduler_km_h[30];
  real_T d_Fellows_Ref_Scheduler_m[30];
  real_T s_Fellows_Ref_Scheduler_m[30];
} Traffic_Signals_toEnvironment;
typedef struct 
{
  real_T d_Total_Fellows_m[30];
  real_T laneIdx_Total_Fellows[30];
  real_T s_Fellows_m[30];
  real_T v_Fellows_km_h[30];
} FellowSignalLabels;
typedef struct 
{
  real_T pos_Vehicle_Init_CoorSys_E_x_y_z_m[3];
  real_T angle_Vehicle_Init_CoorSys_E_x_y_z_rad[3];
} Road_Init;
typedef struct 
{
  real_T s_Veh_Road_m;
  real_T s_Total_Veh_Road_m;
  real_T d_Total_Veh_Road_m;
  real_T s_Veh_Road_Trj_m;
  real_T laneIdx_Total_Veh_Road;
  real_T curv_Veh_Road_1_m;
  real_T pos_x_Ref_Veh_Road_m;
  real_T pos_y_Ref_Veh_Road_m;
  real_T angle_Yaw_Veh_Road_deg;
  real_T slope_Veh_Road_perc;
  real_T v_Wind_Coor_Sys_E_x_y_z_m_s[3];
  real_T slope_Lateral_Veh_Road_deg;
} Road_ASMVeh;
typedef struct 
{
  real_T pos_z_CP_m[4];
  real_T fric_Coeff_CP[4];
  real_T unitVec_z_CP_x_y_z[12];
  real_T sw_Tire_Parameter_Set_CP_1_2_3_4[4];
} Road_ContactPoints;
typedef struct 
{
  real_T pos_x_Fellows_Road_m[30];
  real_T pos_y_Fellows_Road_m[30];
  real_T pos_z_Fellows_Road_m[30];
  real_T angle_Yaw_Fellows_Road_deg[30];
  real_T angle_Pitch_Fellows_Road_deg[30];
  real_T angle_Roll_Fellows_Road_deg[30];
  real_T curv_Fellows_Road_1_m[30];
  real_T d_Total_Fellows_Road_m[30];
  real_T laneIdx_Total_Fellows_Road[30];
  real_T roadSegment_Fellows[30];
  real_T laneSection_Fellows[30];
  real_T speed_Limit_Fellows[30];
  real_T network_Item_Type_Fellows[30];
  real_T network_Item_Id_Fellows[30];
  real_T d_Fellows_RefLine_m[30];
  real_T routeLength_Fellows_m[30];
  real_T flag_RouteClosed_Fellows_0_1[30];
  Dist_Response_Traffic dist_Response_Traffic;
  real_T angle_Yaw_Fellows_RoadRefline_deg[30];
} Road_TrafficFellows;
typedef struct 
{
  real_T gPS_Method;
  real_T const_Offset_Lat_deg;
  real_T const_Offset_Long_deg;
  real_T user4;
  real_T user5;
  real_T user6;
  real_T user7;
  real_T user8;
  real_T user9;
  real_T user10;
} slBus2_UserSignals_Customized;
typedef struct 
{
  real_T longitude_deg;
  real_T latitude_deg;
  real_T heading_deg;
} Pos_Vehicle_GPS_Long_Lat_deg;
typedef struct 
{
  real_T t_Ambient_degC;
  real_T p_Ambient_Pa;
} Ambient;
typedef struct 
{
  DesiredBrakePressure desiredBrakePressure;
} SoftECU_Brake;
typedef struct 
{
  real_T trq_PowerSteering_Nm;
} SoftECU_PowerSteering;
typedef struct 
{
  Accelerations accelerations;
  Velocities velocities;
  Positions positions;
  VehicleEquationOfMotion vehicleEquationOfMotion;
  GravityAcceleration gravityAcceleration;
  SensorMotion sensorMotion;
  VehicleMassAndAdditionalLoads vehicleMassAndAdditionalLoads;
  MassForcesAndTorques massForcesAndTorques;
} VehicleMovement;
typedef struct 
{
  Tire_allSignals tire_allSignals;
  real_T r_Dyn_Tire_FL_FR_RL_RR_m[4];
  real_T r_Dyn_Tire_Mean_m;
  real_T coef_RollingRes_FL_FR_RL_RR[4];
  real_T coef_RollingRes_Mean;
  real_T corneringStiffness_Tire_Front_N_rad;
  real_T corneringStiffness_Tire_Rear_N_rad;
  real_T corneringStiffness_FL_FR_RL_RR_N[4];
} Tire;
typedef struct 
{
  Forces forces;
  Torques torques;
  Coefficients coefficients;
} Aerodynamics;
typedef struct 
{
  WheelSpeed_CoorSys_V wheelSpeed_CoorSys_V;
  ShaftsTorques_CoorSys_V shaftsTorques_CoorSys_V;
  TireWheelTorques tireWheelTorques;
  WheelSpeed wheelSpeed;
  WheelSpeed2 wheelSpeed2;
  RotationAngle rotationAngle;
} Wheel;
typedef struct 
{
  real_T displ_Steering_m;
  real_T displ_Steering_dt_m_s;
  real_T displ_SteeringRod_dt2_m_s2;
  real_T m_Steering_kg;
  ForcesAndTorques forcesAndTorques;
  slBus23_GeneralizedForcesAndTorques generalizedForcesAndTorques;
  real_T i_Steering_rad_m;
  real_T i_Steering_Mean;
  real_T angle_SteeringGear_deg;
  real_T angle_SteeringWheel_deg;
  real_T angle_SteeringGear_dt_deg_s;
  real_T angle_SteeringWheel_dt_deg_s;
  real_T const_Stiffness_SteeringColumn_Nm_rad;
  StatesSteeringSystem statesSteeringSystem;
  real_T frictionTrq_SteeringSystem_Nm;
  SteeringWheelTorque steeringWheelTorque;
} Steering;
typedef struct 
{
  MasterBrakeCylinder masterBrakeCylinder;
  BrakeHydraulics brakeHydraulics;
  BrakeDisc brakeDisc;
  VacuumServo vacuumServo;
  Manifold manifold;
} Brake;
typedef struct 
{
  SuspensionFront suspensionFront;
  SuspensionRear suspensionRear;
} Suspension;
typedef struct 
{
  real_T f_Driving_Res_N;
} DrivingResitances;
typedef struct 
{
  real_T sw_Engine_Dynamics_Fast_0_1;
  boolean_T enable_Trq_Request_Fast_0_1;
  real_T trq_Request_Fast_Nm;
} ESP_Fast_Torque_Set;
typedef struct 
{
  real_T trq_Driver_Ind_Des_Nm;
  real_T trq_Ind_Set_Nm;
  real_T enable_Trq_Ind_Set_0_1;
} Shift_Torque_Set;
typedef struct 
{
  real_T pos_AccPedal_Request_perc;
  real_T enable_Trq_Request_Fast_0_1;
  real_T enable_Trq_Request_Extern1_0_1;
  real_T enable_Trq_Request_Extern2_0_1;
  real_T enable_Trq_Request_Extern3_0_1;
  real_T trq_Request_Fast_Nm;
  real_T trq_Request_Extern1_Nm;
  real_T trq_Request_Extern2_Nm;
  real_T trq_Request_Extern3_Nm;
  real_T pos_AccPedal_Request_Fast_perc;
  real_T pos_AccPedal_Request_Extern1_perc;
  real_T pos_AccPedal_Request_Extern2_perc;
  real_T pos_AccPedal_Request_Extern3_perc;
  real_T pos_AccPedal_Request_Driver_perc;
} Torque_Intervention;
typedef struct 
{
  real_T n_Engine_Idle_Set_rpm;
  real_T pos_AccPedal_Ctrl_perc;
} IdleSpeedControl;
typedef struct 
{
  real_T state_Engine_0_1_2_3_4;
  real_T sw_Starter_0Off_1On;
} EngineOperation;
typedef struct 
{
  real_T timeConst_Engine_Dynamics_s;
  real_T trq_Eff_Engine_Nm;
  real_T trq_Driver_Des_Nm;
  real_T trq_FullLoad_Engine_Nm;
  real_T trq_Fric_Engine_Nm;
  real_T trq_Ind_Engine_Nm;
  real_T trq_Driver_Ind_Des_Nm;
  real_T trq_Engine_Ind_Max_Nm;
} Engine;
typedef struct 
{
  real_T mdot_Fuel_g_h;
  real_T b_e_g_kW_h;
  Efficiency efficiency;
  CO2_Emission cO2_Emission;
  real_T fuel_Consumption_l_h;
  real_T fuel_Consumption_Total_l;
} FuelConsumption;
typedef struct 
{
  real_T t_Preview_s;
  real_T pos_AccPedal_perc;
  real_T pos_BrakePedal_perc;
} LongCtrl;
typedef struct 
{
  real_T pos_AccPedal_perc;
  real_T pos_BrakePedal_perc;
  real_T pos_ClutchPedal_perc;
  real_T gear;
  real_T selectorLever_3T_2P_1R_0N_1D;
} GearShifter;
typedef struct 
{
  real_T s_Preview_Lat_m;
  real_T angle_Steering_rad;
  real_T angle_SteeringWheel_deg;
} LatCtrl1;
typedef struct 
{
  real_T angle_SteeringWheel_deg;
  real_T angle_Steering_deg;
  real_T s_Preview_Lat_m;
  real_T e_Yaw_deg;
  real_T e_Lat_m;
  real_T e_Lat_Preview_m[7];
} LatCtrl2;
typedef struct 
{
  real_T e_Lat_m;
  real_T e_Lat_Abs_m;
  real_T e_Lat_Total_m;
} PositionError;
typedef struct 
{
  real_T v_x_Ref_m_s;
  real_T v_x_Ref_km_h;
  real_T v_x_Ref_Preview_m_s;
  real_T v_x_Ref_Preview_km_h;
  real_T a_x_Ref_m_s2;
  real_T a_y_Ref_m_s2;
  real_T s_Preview_vRef_m;
} SpeedProfiler;
typedef struct 
{
  real_T curv_Vehicle_1_m;
  real_T curv_Preview_1_m[10];
} Curvature;
typedef struct 
{
  real_T ctrl_Lockup_Clutch_0_1;
} Lockup_Clutch_Control;
typedef struct 
{
  real_T ctrl_Parking_Pawl_0_1;
} Shift_Lock_Control;
typedef struct 
{
  real_T gear_Requested;
} Shift_Strategy;
typedef struct 
{
  real_T gear_Requested;
} Tip_Shift_Control;
typedef struct 
{
  real_T factor_Trq_Engine_Red_perc;
  real_T gear;
} Torque_Intervention_Control;
typedef struct 
{
  real_T n_Engine_rad_s;
  real_T n_Engine_rpm;
  real_T trq_Mass_Mod_Nm;
  real_T const_Inertia_Engine_kgm2;
  real_T trq_Eff_Crank_Nm;
} Crankshaft;
typedef struct 
{
  real_T trq_Starter_Nm;
  real_T inertia_Starter_kgm2;
} Starter;
typedef struct 
{
  real_T trq_TestBench_Nm;
  real_T inertia_TestBench_kgm2;
} TestBench;
typedef struct 
{
  Manual manual;
  Automatic automatic;
  slBus39_OutputSignals outputSignals;
} Transmission;
typedef struct 
{
  real_T i_FrontDrive;
  real_T i_RearDrive;
  real_T trqDistri_Front_Rear_0_1;
  real_T i_FinalDrive;
  CentralDifferential centralDifferential;
  FrontDifferential frontDifferential;
  RearDifferential rearDifferential;
  Shaft_TGCL4WD shaft_TGCL4WD;
  Shaft_CF shaft_CF;
  Shaft_CR shaft_CR;
  TransferGearbox transferGearbox;
  Clutch_4WD clutch_4WD;
  Shaft_FL shaft_FL;
  Shaft_FR shaft_FR;
  Shaft_RL shaft_RL;
  Shaft_RR shaft_RR;
  real_T trq_Out_Front_Diff_Nm;
  real_T trq_Shaft_FL_FR_RL_RR_Nm[4];
} FinalDriveAssembly;
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
  ManeuverScheduler maneuverScheduler;
  UserSignals_Customized userSignals_Customized;
  StartButton startButton;
  real_T sw_TipShift_1Down_0Off_1Up;
  ParkingBrake parkingBrake;
} Maneuver;
typedef struct 
{
  Collision_Sensor collision_Sensor;
} Sensors;
typedef struct 
{
  Traffic_Signals_toEnvironment traffic_Signals_toEnvironment;
  FellowSignalLabels fellowSignalLabels;
} Traffic;
typedef struct 
{
  Road_Init road_Init;
  Road_ASMVeh road_ASMVeh;
  Road_ContactPoints road_ContactPoints;
  Road_TrafficFellows road_TrafficFellows;
  slBus2_UserSignals_Customized userSignals_Customized;
  Pos_Vehicle_GPS_Long_Lat_deg pos_Vehicle_GPS_Long_Lat_deg;
  Ambient ambient;
} Road;
typedef struct 
{
  SoftECU_Brake softECU_Brake;
  SoftECU_PowerSteering softECU_PowerSteering;
} Control;
typedef struct 
{
  VehicleMovement vehicleMovement;
  Tire tire;
  Aerodynamics aerodynamics;
  Wheel wheel;
  Steering steering;
  Brake brake;
  Suspension suspension;
  DrivingResitances drivingResitances;
} Plant;
typedef struct 
{
  ESP_Fast_Torque_Set eSP_Fast_Torque_Set;
  Shift_Torque_Set shift_Torque_Set;
  Torque_Intervention torque_Intervention;
  IdleSpeedControl idleSpeedControl;
  EngineOperation engineOperation;
} slBus35_Control;
typedef struct 
{
  Engine engine;
  FuelConsumption fuelConsumption;
} slBus36_Plant;
typedef struct 
{
  LongCtrl longCtrl;
  GearShifter gearShifter;
} LongitudinalController;
typedef struct 
{
  LatCtrl1 latCtrl1;
  LatCtrl2 latCtrl2;
  PositionError positionError;
} LateralController;
typedef struct 
{
  SpeedProfiler speedProfiler;
  Curvature curvature;
} ReferenceVelocity;
typedef struct 
{
  real_T pos_AccPedal_perc;
  real_T pos_BrakePedal_perc;
  real_T pos_ClutchPedal_perc;
  real_T gear;
  real_T selectorLever_3T_2P_1R_0N_1D;
  real_T s_Preview_vRef_m[15];
  real_T s_Preview_LatCtrl_m[10];
  real_T s_Preview_m[25];
  real_T angle_SteeringWheel_deg;
} OutputSignals;
typedef struct 
{
  Lockup_Clutch_Control lockup_Clutch_Control;
  Shift_Lock_Control shift_Lock_Control;
  Shift_Strategy shift_Strategy;
  Tip_Shift_Control tip_Shift_Control;
  Torque_Intervention_Control torque_Intervention_Control;
} slBus38_Control;
typedef struct 
{
  Crankshaft crankshaft;
  Starter starter;
  TestBench testBench;
  Transmission transmission;
  FinalDriveAssembly finalDriveAssembly;
} slBus40_Plant;
typedef struct 
{
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
} power_train_data;
typedef struct 
{
  real32_T fl_tire_temperature;
  real_T fl_damper_linear_potentiometer;
  real32_T fl_tire_pressure;
  real32_T fl_tire_pressure_gauge;
  real_T fl_wheel_load;
  real32_T fr_tire_temperature;
  real_T fr_damper_linear_potentiometer;
  real32_T fr_tire_pressure;
  real32_T fr_tire_pressure_gauge;
  real_T fr_wheel_load;
  real32_T rl_tire_temperature;
  real_T rl_damper_linear_potentiometer;
  real32_T rl_tire_pressure;
  real32_T rl_tire_pressure_gauge;
  real_T rl_wheel_load;
  real32_T rr_tire_temperature;
  real_T rr_damper_linear_potentiometer;
  real32_T rr_tire_pressure;
  real32_T rr_tire_pressure_gauge;
  real_T rr_wheel_load;
  real32_T fl_brake_temp;
  real32_T fr_brake_temp;
  real32_T rl_brake_temp;
  real32_T rr_brake_temp;
  real32_T battery_voltage;
  uint8_T safety_switch_state;
  boolean_T mode_switch_state;
  uint8_T sys_state;
  real32_T accel_pedal_input;
  real32_T accel_pedal_output;
  real_T front_brake_pressure;
  real_T rear_brake_pressure;
  real_T steering_wheel_angle;
  real32_T steering_wheel_angle_cmd;
  real_T steering_wheel_torque;
  real32_T ws_front_left;
  real32_T ws_front_right;
  real32_T ws_rear_left;
  real32_T ws_rear_right;
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
  poslla poslla_var;
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
} race_control;
typedef struct 
{
  Maneuver maneuver;
  Sensors sensors;
  Traffic traffic;
  Road road;
} Environment;
typedef struct 
{
  Control control;
  Plant plant;
} VehicleDynamics;
typedef struct 
{
  slBus35_Control control;
  slBus36_Plant plant;
} slBus37_Engine;
typedef struct 
{
  LongitudinalController longitudinalController;
  LateralController lateralController;
  ReferenceVelocity referenceVelocity;
  OutputSignals outputSignals;
} Driver;
typedef struct 
{
  slBus38_Control control;
  slBus40_Plant plant;
} DriveTrain;
typedef struct 
{
  real_T fellow_count;
  ground_truth ground_truth_var;
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
  race_control race_control_var;
  Environment environment;
  VehicleDynamics vehicleDynamics;
  slBus37_Engine engine;
  Driver driver;
  DriveTrain driveTrain;
} asm_bus;
typedef struct 
{
  vehicle_sensors vehicle_sensors_var;
  vector_nav_vn1 vector_nav_vn1_var;
  nova_tel_pwr_pak nova_tel_pwr_pak1_var;
  nova_tel_pwr_pak nova_tel_pwr_pak2_var;
} sim_interface;
typedef struct 
{
  asm_bus asm_bus_var;
  sim_interface sim_interface_var;
} ASMBus;

#pragma pack(pop)
#endif                                 /* RTW_HEADER_ASMBus_h_ */
