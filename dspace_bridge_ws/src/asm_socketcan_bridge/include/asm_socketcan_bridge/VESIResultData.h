#include <stdint.h>
#ifndef VESI_SUT_FEEDBACK_STRUCT_H
#define VESI_SUT_FEEDBACK_STRUCT_H
#include "rtwtypes.h"
#pragma pack(push, 1)



typedef struct
{
    // Throttle command (%)
    double throttle_cmd;
    uint8_t throttle_cmd_count;
    uint8_t enable_throttle_cmd;

    // Brake pressure command (kPa)
    uint16_t brake_cmd_front;
    uint16_t brake_cmd_rear;
    uint8_t brake_bias_switch;
    uint8_t brake_cmd_count;
    uint8_t enable_brake_cmd;

    // Steering motor angle command (degrees)
    int16_t steering_cmd;
    uint8_t steering_cmd_count;
    uint8_t enable_steering_cmd;

    uint8_t drive_steering_FF_cntrl_switch;
    float driver_steering_FF_cmd;

    uint8_t drive_steering_gain_cntrl_switch;
    float driver_steering_P_cmd;
    float driver_steering_I_cmd;
    float driver_steering_D_cmd;

    // Traction control commands
    uint8_t driver_traction_aim_switch;
    uint8_t driver_traction_range_switch;


    // Gear command
    uint8_t gear_cmd;
    uint8_t enable_gear_cmd;
}
VehicleInputs;

typedef struct
{
    uint16_t track_cond_ack; // track flag
    uint8_t veh_sig_ack; // vehicle flag
    uint8_t marelli_sector_flag_ack;

    uint16_t ct_state;
    uint8_t rolling_counter;
    uint8_t veh_num;

    uint8_t push2pass_switch;
    uint8_t push2pass_request;

}
ToRaptor;

typedef struct
{
    VehicleInputs vehicle_inputs;
    ToRaptor to_raptor;
}
VESIResultData;

#pragma pack(pop)

#endif