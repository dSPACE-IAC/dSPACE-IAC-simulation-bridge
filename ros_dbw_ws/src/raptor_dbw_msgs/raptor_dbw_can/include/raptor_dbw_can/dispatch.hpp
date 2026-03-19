// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RAPTOR_DBW_CAN__DISPATCH_HPP_
#define RAPTOR_DBW_CAN__DISPATCH_HPP_

#include <stdint.h>

namespace raptor_dbw_can
{
typedef enum
{
  VIN_MUX_VIN0  = 0x00,
  VIN_MUX_VIN1  = 0x01,
  VIN_MUX_VIN2  = 0x02,
} VinMux;

typedef enum
{
  WHEEL_SPEED_MUX0  = 0x00,
  WHEEL_SPEED_MUX1  = 0x01,
  WHEEL_SPEED_MUX2  = 0x02,
} WheelSpeedMux;

#undef BUILD_ASSERT

enum
{
  ID_WHEEL_SPEED_REPORT_DO      = 0x0514,
  ID_BRAKE_PRESSURE_REPORT_DO   = 0x0515,
  ID_ACCELERATOR_REPORT_DO      = 0x0516,
  ID_STEERING_REPORT_DO         = 0x0517,
  ID_MISC_REPORT_DO             = 0x0518,
  ID_STEERING_REPORT_EXTD       = 0x0520,
  ID_RC_TO_CT                   = 0x04B0,
  ID_BASE_TO_CAR_TIMING         = 0x04B8,
  ID_TIRE_TEMP_FL_1             = 0x052C,
  ID_TIRE_TEMP_FL_2             = 0x052D,
  ID_TIRE_TEMP_FL_3             = 0x052E,
  ID_TIRE_TEMP_FL_4             = 0x052F,
  ID_TIRE_TEMP_FR_1             = 0x0530,
  ID_TIRE_TEMP_FR_2             = 0x0531,
  ID_TIRE_TEMP_FR_3             = 0x0532,
  ID_TIRE_TEMP_FR_4             = 0x0533,
  ID_TIRE_TEMP_RL_1             = 0x0534,
  ID_TIRE_TEMP_RL_2             = 0x0535,
  ID_TIRE_TEMP_RL_3             = 0x0536,
  ID_TIRE_TEMP_RL_4             = 0x0537,
  ID_TIRE_TEMP_RR_1             = 0x0538,
  ID_TIRE_TEMP_RR_2             = 0x0539,
  ID_TIRE_TEMP_RR_3             = 0x053A,
  ID_TIRE_TEMP_RR_4             = 0x053B,
  ID_WHEEL_STRAIN_GAUGE         = 0x051E,
  ID_WHEEL_POTENTIOMETER        = 0x051F,
  ID_TIRE_PRESSURE_FL           = 0x0528,
  ID_TIRE_PRESSURE_FR           = 0x0529,
  ID_TIRE_PRESSURE_RL           = 0x052A,
  ID_TIRE_PRESSURE_RR           = 0x052B,
  ID_VEL_ACC_HIL                = 0x05DD,
  ID_POSITION_HEADING_HIL       = 0x05DC,
  ID_PT_REPORT_1                = 0x053C,
  ID_PT_REPORT_2                = 0x053D,
  ID_PT_REPORT_3                = 0x053F,
  ID_DIAGNOSTIC_REPORT          = 0x053E,
  ID_MYLAPS_REPORT_1            = 0x00E4,
  ID_MYLAPS_REPORT_2            = 0x00E2,
  // MOTEC REPORT
  ID_M1_GENERAL_1               = 0x0640,
  ID_M1_GENERAL_2               = 0x0641,
  ID_M1_GENERAL_3               = 0x0642,
  ID_M1_GENERAL_4               = 0x0644,
  ID_M1_GENERAL_5               = 0x0645,
  ID_M1_GENERAL_6               = 0x0648,
  ID_M1_GENERAL_7               = 0x0649,
  ID_M1_GENERAL_8               = 0x064C,
  ID_M1_GENERAL_9               = 0x064D,
  ID_M1_GENERAL_10              = 0x064F,
  ID_M1_GENERAL_11              = 0x0650,
  ID_M1_GENERAL_12              = 0x0651,
  ID_M1_GENERAL_13              = 0x0653,
  ID_M1_GENERAL_14              = 0x0659,
  ID_M1_GENERAL_15              = 0x0630,
  ID_M1_GENERAL_16              = 0x0631,
  ID_M1_GENERAL_17              = 0x0668,
  ID_M1_GENERAL_18              = 0x0018,
  // CAN3 MESSAGES - SPACEDRIVE
  ID_SHUTDOWN_SEQ               = 0x0160,
  ID_REQ_BRAKE_POS              = 0x0021,
  ID_REQ_STEER_POS              = 0x0020,
  ID_STAT_BRK_3                 = 0x0085,
  ID_STAT_BRK_2                 = 0x0083,
  ID_STAT_BRK_1                 = 0x0081,
  ID_STAT_STR_3                 = 0x0084,
  ID_STAT_STR_2                 = 0x0082,
  ID_STAT_STR_1                 = 0x0080,
  ID_ACT_ERR_BRAKE_3            = 0x0065,
  ID_ACT_ERR_STEER_3            = 0x0064,
  ID_ACT_ERR_BRAKE_2            = 0x0063,
  ID_ACT_ERR_STEER_2            = 0x0062,
  ID_ACT_ERR_BRAKE_1            = 0x0061,
  ID_ACT_ERR_STEER_1            = 0x0060,
  ID_CALB_STEER_POS_3           = 0x0124,
  ID_CALB_STEER_POS_2           = 0x0122,
  ID_CALB_STEER_POS_1           = 0x0120,
  ID_CALB_BRAKE_POS_3           = 0x0125,
  ID_CALB_BRAKE_POS_2           = 0x0123,
  ID_CALB_BRAKE_POS_1           = 0x0121,
  // CAN3 MESSAGES - TTPMS
  ID_RR_TTPMS_6                 = 0x061D,
  ID_RR_TTPMS_5                 = 0x061C,
  ID_RR_TTPMS_4                 = 0x061B,
  ID_RR_TTPMS_3                 = 0x061A,
  ID_RR_TTPMS_2                 = 0x0619,
  ID_RR_TTPMS_1                 = 0x0618,
  ID_LR_TTPMS_6                 = 0x0617,
  ID_LR_TTPMS_5                 = 0x0616,
  ID_LR_TTPMS_4                 = 0x0615,
  ID_LR_TTPMS_3                 = 0x0614,
  ID_LR_TTPMS_2                 = 0x0613,
  ID_LR_TTPMS_1                 = 0x0612,
  ID_RF_TTPMS_6                 = 0x0611,
  ID_RF_TTPMS_5                 = 0x0610,
  ID_RF_TTPMS_4                 = 0x060F,
  ID_RF_TTPMS_3                 = 0x060E,
  ID_RF_TTPMS_2                 = 0x060D,
  ID_RF_TTPMS_1                 = 0x060C,
  ID_LF_TTPMS_6                 = 0x060B,
  ID_LF_TTPMS_5                 = 0x060A,
  ID_LF_TTPMS_4                 = 0x0609,
  ID_LF_TTPMS_3                 = 0x0608,
  ID_LF_TTPMS_2                 = 0x0607,
  ID_LF_TTPMS_1                 = 0x0606

};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DISPATCH_HPP_
