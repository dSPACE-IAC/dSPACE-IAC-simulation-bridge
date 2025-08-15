#include <stdint.h>
#ifndef VESI_SUT_FEEDBACK_STRUCT_H
#define VESI_SUT_FEEDBACK_STRUCT_H
#include "rtwtypes.h"
#pragma pack(push, 1)



typedef struct
{
    // Throttle command (%)
    float throttle_cmd;
    uint8_t throttle_cmd_count;
    uint8_t enable_throttle_cmd;

    // Brake pressure command (kPa)
    float brake_cmd_front;
    float brake_cmd_rear;
    uint8_t brake_bias_switch;
    uint8_t brake_cmd_count;
    uint8_t enable_brake_cmd;

    // Steering motor angle command (degrees)
    float steering_cmd;
    uint8_t steering_cmd_count;
    uint8_t enable_steering_cmd;

    // Gear command
    uint8_t gear_cmd;
    uint8_t enable_gear_cmd;
}
VehicleInputs;

typedef struct
{
    uint8_t track_cond_ack; // track flag
    uint8_t veh_sig_ack; // vehicle flag
    uint8_t ct_state;
    uint8_t rolling_counter;
    uint8_t veh_num;

    boolean_T push2pass_request;
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



















