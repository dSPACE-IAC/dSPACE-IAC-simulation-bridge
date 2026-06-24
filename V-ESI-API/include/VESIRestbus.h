#ifndef VESIRESTBUS_H
#define VESIRESTBUS_H


#pragma pack(push, 1)
namespace VESIRestbus
{
    // VehicleDynamics
    typedef struct
    {
        double Angle_SteeringWheel;
        double Angle_SteeringWheel_dt;

    } Steering;
    typedef struct
    {
        double a_x_Vehicle_CoG;
        double a_y_Vehicle_CoG;
        double a_z_Vehicle_CoG;

        double w_x_Vehicle_CoG_dt;
        double w_y_Vehicle_CoG_dt;
        double w_z_Vehicle_CoG_dt;

        double v_x_Vehicle_CoG;
        double v_y_Vehicle_CoG;
        double v_z_Vehicle_CoG;

        double v_Total_Vehicle;

        double RollRate_Vehicle_CoG;
        double PitchRate_Vehicle_CoG;
        double YawRate_Vehicle_CoG;

        double Pos_x_Vehicle_CoorSys_E;
        double Pos_y_Vehicle_CoorSys_E;
        double Pos_z_Vehicle_CoorSys_E;

        double Angle_Roll_Vehicle_CoorSys_E;
        double Angle_Pitch_Vehicle_CoorSys_E;
        double Angle_Yaw_Vehicle_CoorSys_E;

    } VehicleMovement;

    typedef struct
    {
        double omega_FL_Wheel;
        double omega_FR_Wheel;
        double omega_RL_Wheel;
        double omega_RR_Wheel;

        double Angle_Rot_FL_Wheel;
        double Angle_Rot_FR_Wheel;
        double Angle_Rot_RL_Wheel;
        double Angle_Rot_RR_Wheel;

    } Wheel;
    typedef struct
    {
        double ActiveBrakeCylinderPressure;
    } Brake;

    typedef struct
    {
        Steering steering;
        VehicleMovement vehiclemovement;
        Wheel wheel;
        Brake brake;

    } VehicleDynamics;

    // BasicEngine
    typedef struct
    {
        double Trq_Eff_Engine;
        double Trq_Driver_Des;
        double Trq_Fric_Engine;
        double Trq_Ind_Engine;
        double Trq_FullLoad_Engine;
    } BasicEngine;


    // Drivetrain
    typedef struct
    {
        double gear;
        double omega_In_Trm;
        double omega_Out_Trm;
        double Trq_Out_Trm;
        double n_Engine_rpm;
        double Trq_Eff_Crank;

    } DriveTrain;


    // Environment
    typedef struct
    {
        double ManeuverTime;
        double ManeuverState;
        double Vehicle_Turnlights;

    } Maneuver;

    typedef struct
    {
        Maneuver maneuver;

    } Environment;
    // VESI-Restbus-Signals
    typedef struct
    {
        VehicleDynamics vehicledynamics;
        BasicEngine basicengine;
        DriveTrain drivetrain;
        Environment environment;

    } VESIRestbus;
}
#pragma pack(pop)

#endif
