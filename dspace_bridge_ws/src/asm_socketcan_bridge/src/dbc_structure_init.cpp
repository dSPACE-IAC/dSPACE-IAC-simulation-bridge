#include "dbc_structure.h"

#include <cstdlib>

std::vector<Message> initialize_messages() {
    size_t message_count = 53;
    std::vector<Message> messages(message_count);

    messages[0].id = 3221225472;
    messages[0].dlc = 0;
    messages[0].name = "VECTOR__INDEPENDENT_SIG_MSG";
    messages[0].transmitter = "Vector__XXX";
    messages[0].signal_count = 13;
    messages[0].signals.resize(messages[0].signal_count);
    messages[0].signals[0] = Signal{ "ang_heading", 40, 16, 1, true, 0.01f, 0.0f, -327.68f, 327.67f, "rad" };
    messages[0].signals[1] = Signal{ "pos_y", 20, 20, 1, true, 0.01f, 0.0f, -5242.88f, 5242.87f, "m" };
    messages[0].signals[2] = Signal{ "pos_x", 0, 20, 1, true, 0.01f, 0.0f, -5242.88f, 5242.87f, "m" };
    messages[0].signals[3] = Signal{ "yaw_rate", 36, 12, 1, true, 0.025f, 0.0f, -51.2f, 51.175f, "rps" };
    messages[0].signals[4] = Signal{ "velocity_long", 24, 12, 1, true, 0.05f, 0.0f, -102.4f, 102.35f, "mps" };
    messages[0].signals[5] = Signal{ "velocity_lat", 12, 12, 1, true, 0.05f, 0.0f, -102.4f, 102.35f, "mps" };
    messages[0].signals[6] = Signal{ "motor_angle", 48, 12, 1, true, 0.5f, 0.0f, -255.0f, 255.0f, "rad" };
    messages[0].signals[7] = Signal{ "acceleration", 0, 12, 1, true, 0.025f, 0.0f, -51.2f, 51.175f, "mps2" };
    messages[0].signals[8] = Signal{ "rc_base_sync_check", 0, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[0].signals[9] = Signal{ "rc_lte_rssi", 1, 8, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "dBm" };
    messages[0].signals[10] = Signal{ "duty_cycle_fbk", 48, 8, 1, true, 1.0f, 0.0f, -100.0f, 100.0f, "%" };
    messages[0].signals[11] = Signal{ "duty_cycle_dmd", 40, 8, 1, true, 1.0f, 0.0f, -100.0f, 100.0f, "%" };
    messages[0].signals[12] = Signal{ "steering_motor_ang_avg_fdbk", 0, 11, 1, true, 0.5f, 0.0f, -1024.0f, 1023.0f, "deg" };

    messages[1].id = 1312;
    messages[1].dlc = 6;
    messages[1].name = "steering_report_extd";
    messages[1].transmitter = "Vector__XXX";
    messages[1].signal_count = 3;
    messages[1].signals.resize(messages[1].signal_count);
    messages[1].signals[0] = Signal{ "average_steering_ang_fdbk", 32, 11, 1, true, 0.5f, 0.0f, -1024.0f, 1023.0f, "deg" };
    messages[1].signals[1] = Signal{ "secondary_steering_ang_fdbk", 16, 11, 1, true, 0.5f, 0.0f, -1024.0f, 1023.0f, "deg" };
    messages[1].signals[2] = Signal{ "primary_steering_angle_fbk", 0, 11, 1, true, 0.5f, 0.0f, -1024.0f, 1023.0f, "deg" };

    messages[2].id = 1339;
    messages[2].dlc = 8;
    messages[2].name = "Tire_Temp_RR_4";
    messages[2].transmitter = "Vector__XXX";
    messages[2].signal_count = 4;
    messages[2].signals.resize(messages[2].signal_count);
    messages[2].signals[0] = Signal{ "RR_Tire_Temp_16", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[2].signals[1] = Signal{ "RR_Tire_Temp_15", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[2].signals[2] = Signal{ "RR_Tire_Temp_14", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[2].signals[3] = Signal{ "RR_Tire_Temp_13", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[3].id = 1338;
    messages[3].dlc = 8;
    messages[3].name = "Tire_Temp_RR_3";
    messages[3].transmitter = "Vector__XXX";
    messages[3].signal_count = 4;
    messages[3].signals.resize(messages[3].signal_count);
    messages[3].signals[0] = Signal{ "RR_Tire_Temp_12", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[3].signals[1] = Signal{ "RR_Tire_Temp_11", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[3].signals[2] = Signal{ "RR_Tire_Temp_10", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[3].signals[3] = Signal{ "RR_Tire_Temp_09", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[4].id = 1337;
    messages[4].dlc = 8;
    messages[4].name = "Tire_Temp_RR_2";
    messages[4].transmitter = "Vector__XXX";
    messages[4].signal_count = 4;
    messages[4].signals.resize(messages[4].signal_count);
    messages[4].signals[0] = Signal{ "RR_Tire_Temp_08", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[4].signals[1] = Signal{ "RR_Tire_Temp_07", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[4].signals[2] = Signal{ "RR_Tire_Temp_06", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[4].signals[3] = Signal{ "RR_Tire_Temp_05", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[5].id = 1336;
    messages[5].dlc = 8;
    messages[5].name = "Tire_Temp_RR_1";
    messages[5].transmitter = "Vector__XXX";
    messages[5].signal_count = 4;
    messages[5].signals.resize(messages[5].signal_count);
    messages[5].signals[0] = Signal{ "RR_Tire_Temp_04", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[5].signals[1] = Signal{ "RR_Tire_Temp_03", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[5].signals[2] = Signal{ "RR_Tire_Temp_02", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[5].signals[3] = Signal{ "RR_Tire_Temp_01", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[6].id = 1335;
    messages[6].dlc = 8;
    messages[6].name = "Tire_Temp_RL_4";
    messages[6].transmitter = "Vector__XXX";
    messages[6].signal_count = 4;
    messages[6].signals.resize(messages[6].signal_count);
    messages[6].signals[0] = Signal{ "RL_Tire_Temp_16", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[6].signals[1] = Signal{ "RL_Tire_Temp_15", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[6].signals[2] = Signal{ "RL_Tire_Temp_14", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[6].signals[3] = Signal{ "RL_Tire_Temp_13", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[7].id = 1334;
    messages[7].dlc = 8;
    messages[7].name = "Tire_Temp_RL_3";
    messages[7].transmitter = "Vector__XXX";
    messages[7].signal_count = 4;
    messages[7].signals.resize(messages[7].signal_count);
    messages[7].signals[0] = Signal{ "RL_Tire_Temp_12", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[7].signals[1] = Signal{ "RL_Tire_Temp_11", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[7].signals[2] = Signal{ "RL_Tire_Temp_10", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[7].signals[3] = Signal{ "RL_Tire_Temp_09", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[8].id = 1333;
    messages[8].dlc = 8;
    messages[8].name = "Tire_Temp_RL_2";
    messages[8].transmitter = "Vector__XXX";
    messages[8].signal_count = 4;
    messages[8].signals.resize(messages[8].signal_count);
    messages[8].signals[0] = Signal{ "RL_Tire_Temp_08", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[8].signals[1] = Signal{ "RL_Tire_Temp_07", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[8].signals[2] = Signal{ "RL_Tire_Temp_06", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[8].signals[3] = Signal{ "RL_Tire_Temp_05", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[9].id = 1332;
    messages[9].dlc = 8;
    messages[9].name = "Tire_Temp_RL_1";
    messages[9].transmitter = "Vector__XXX";
    messages[9].signal_count = 4;
    messages[9].signals.resize(messages[9].signal_count);
    messages[9].signals[0] = Signal{ "RL_Tire_Temp_04", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[9].signals[1] = Signal{ "RL_Tire_Temp_03", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[9].signals[2] = Signal{ "RL_Tire_Temp_02", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[9].signals[3] = Signal{ "RL_Tire_Temp_01", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[10].id = 1331;
    messages[10].dlc = 8;
    messages[10].name = "Tire_Temp_FR_4";
    messages[10].transmitter = "Vector__XXX";
    messages[10].signal_count = 4;
    messages[10].signals.resize(messages[10].signal_count);
    messages[10].signals[0] = Signal{ "FR_Tire_Temp_16", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[10].signals[1] = Signal{ "FR_Tire_Temp_15", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[10].signals[2] = Signal{ "FR_Tire_Temp_14", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[10].signals[3] = Signal{ "FR_Tire_Temp_13", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[11].id = 1330;
    messages[11].dlc = 8;
    messages[11].name = "Tire_Temp_FR_3";
    messages[11].transmitter = "Vector__XXX";
    messages[11].signal_count = 4;
    messages[11].signals.resize(messages[11].signal_count);
    messages[11].signals[0] = Signal{ "FR_Tire_Temp_12", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[11].signals[1] = Signal{ "FR_Tire_Temp_11", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[11].signals[2] = Signal{ "FR_Tire_Temp_10", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[11].signals[3] = Signal{ "FR_Tire_Temp_09", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[12].id = 1329;
    messages[12].dlc = 8;
    messages[12].name = "Tire_Temp_FR_2";
    messages[12].transmitter = "Vector__XXX";
    messages[12].signal_count = 4;
    messages[12].signals.resize(messages[12].signal_count);
    messages[12].signals[0] = Signal{ "FR_Tire_Temp_08", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[12].signals[1] = Signal{ "FR_Tire_Temp_07", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[12].signals[2] = Signal{ "FR_Tire_Temp_06", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[12].signals[3] = Signal{ "FR_Tire_Temp_05", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[13].id = 1328;
    messages[13].dlc = 8;
    messages[13].name = "Tire_Temp_FR_1";
    messages[13].transmitter = "Vector__XXX";
    messages[13].signal_count = 4;
    messages[13].signals.resize(messages[13].signal_count);
    messages[13].signals[0] = Signal{ "FR_Tire_Temp_04", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[13].signals[1] = Signal{ "FR_Tire_Temp_03", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[13].signals[2] = Signal{ "FR_Tire_Temp_02", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[13].signals[3] = Signal{ "FR_Tire_Temp_01", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[14].id = 1327;
    messages[14].dlc = 8;
    messages[14].name = "Tire_Temp_FL_4";
    messages[14].transmitter = "Vector__XXX";
    messages[14].signal_count = 4;
    messages[14].signals.resize(messages[14].signal_count);
    messages[14].signals[0] = Signal{ "FL_Tire_Temp_16", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[14].signals[1] = Signal{ "FL_Tire_Temp_15", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[14].signals[2] = Signal{ "FL_Tire_Temp_14", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[14].signals[3] = Signal{ "FL_Tire_Temp_13", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[15].id = 1326;
    messages[15].dlc = 8;
    messages[15].name = "Tire_Temp_FL_3";
    messages[15].transmitter = "Vector__XXX";
    messages[15].signal_count = 4;
    messages[15].signals.resize(messages[15].signal_count);
    messages[15].signals[0] = Signal{ "FL_Tire_Temp_12", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[15].signals[1] = Signal{ "FL_Tire_Temp_11", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[15].signals[2] = Signal{ "FL_Tire_Temp_10", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[15].signals[3] = Signal{ "FL_Tire_Temp_09", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[16].id = 1325;
    messages[16].dlc = 8;
    messages[16].name = "Tire_Temp_FL_2";
    messages[16].transmitter = "Vector__XXX";
    messages[16].signal_count = 4;
    messages[16].signals.resize(messages[16].signal_count);
    messages[16].signals[0] = Signal{ "FL_Tire_Temp_08", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[16].signals[1] = Signal{ "FL_Tire_Temp_07", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[16].signals[2] = Signal{ "FL_Tire_Temp_06", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[16].signals[3] = Signal{ "FL_Tire_Temp_05", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[17].id = 1324;
    messages[17].dlc = 8;
    messages[17].name = "Tire_Temp_FL_1";
    messages[17].transmitter = "Vector__XXX";
    messages[17].signal_count = 4;
    messages[17].signals.resize(messages[17].signal_count);
    messages[17].signals[0] = Signal{ "FL_Tire_Temp_04", 55, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[17].signals[1] = Signal{ "FL_Tire_Temp_03", 39, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[17].signals[2] = Signal{ "FL_Tire_Temp_02", 23, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };
    messages[17].signals[3] = Signal{ "FL_Tire_Temp_01", 7, 16, 0, false, 0.1f, -100.0f, -100.0f, 6453.5f, "degC" };

    messages[18].id = 1323;
    messages[18].dlc = 4;
    messages[18].name = "Tire_Pressure_RR";
    messages[18].transmitter = "Vector__XXX";
    messages[18].signal_count = 2;
    messages[18].signals.resize(messages[18].signal_count);
    messages[18].signals[0] = Signal{ "RR_Tire_Pressure_Gauge", 23, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };
    messages[18].signals[1] = Signal{ "RR_Tire_Pressure", 7, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };

    messages[19].id = 1322;
    messages[19].dlc = 4;
    messages[19].name = "Tire_Pressure_RL";
    messages[19].transmitter = "Vector__XXX";
    messages[19].signal_count = 2;
    messages[19].signals.resize(messages[19].signal_count);
    messages[19].signals[0] = Signal{ "RL_Tire_Pressure_Gauge", 23, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };
    messages[19].signals[1] = Signal{ "RL_Tire_Pressure", 7, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };

    messages[20].id = 1321;
    messages[20].dlc = 4;
    messages[20].name = "Tire_Pressure_FR";
    messages[20].transmitter = "Vector__XXX";
    messages[20].signal_count = 2;
    messages[20].signals.resize(messages[20].signal_count);
    messages[20].signals[0] = Signal{ "FR_Tire_Pressure_Gauge", 23, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };
    messages[20].signals[1] = Signal{ "FR_Tire_Pressure", 7, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };

    messages[21].id = 1320;
    messages[21].dlc = 4;
    messages[21].name = "Tire_Pressure_FL";
    messages[21].transmitter = "Vector__XXX";
    messages[21].signal_count = 2;
    messages[21].signals.resize(messages[21].signal_count);
    messages[21].signals[0] = Signal{ "FL_Tire_Pressure_Gauge", 23, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };
    messages[21].signals[1] = Signal{ "FL_Tire_Pressure", 7, 16, 0, false, 1.0f, 0.0f, 0.0f, 65535.0f, "mbar" };

    messages[22].id = 1403;
    messages[22].dlc = 8;
    messages[22].name = "gear_shift_cmd";
    messages[22].transmitter = "Vector__XXX";
    messages[22].signal_count = 1;
    messages[22].signals.resize(messages[22].signal_count);
    messages[22].signals[0] = Signal{ "desired_gear", 0, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[23].id = 1404;
    messages[23].dlc = 8;
    messages[23].name = "ct_report";
    messages[23].transmitter = "Vector__XXX";
    messages[23].signal_count = 5;
    messages[23].signals.resize(messages[23].signal_count);
    messages[23].signals[0] = Signal{ "track_cond_ack", 8, 16, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[23].signals[1] = Signal{ "veh_sig_ack", 0, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[23].signals[2] = Signal{ "ct_state", 24, 16, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[23].signals[3] = Signal{ "ct_state_rolling_counter", 40, 8, 1, false, 1.0f, 0.0f, 0.0f, 9.0f, "" };
    messages[23].signals[4] = Signal{ "veh_num", 48, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[24].id = 1402;
    messages[24].dlc = 8;
    messages[24].name = "steering_cmd";
    messages[24].transmitter = "Vector__XXX";
    messages[24].signal_count = 5;
    messages[24].signals.resize(messages[24].signal_count);
    messages[24].signals[0] = Signal{ "steering_motor_cmd_counter", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 255.0f, "" };
    messages[24].signals[1] = Signal{ "steering_motor_ang_cmd", 0, 11, 1, true, 0.5f, 0.0f, -1024.0f, 1023.0f, "deg" };
    messages[24].signals[2] = Signal{ "driver_steering_P_cmd", 24, 8, 1, false, 0.1f, 0.0f, 0.0f, 0.0f, "" };
    messages[24].signals[3] = Signal{ "driver_steering_I_cmd", 32, 8, 1, false, 0.1f, 0.0f, 0.0f, 0.0f, "" };
    messages[24].signals[4] = Signal{ "driver_steering_D_cmd", 40, 8, 1, false, 0.01f, 0.0f, 0.0f, 0.0f, "" };

    messages[25].id = 1401;
    messages[25].dlc = 8;
    messages[25].name = "accelerator_cmd";
    messages[25].transmitter = "Vector__XXX";
    messages[25].signal_count = 2;
    messages[25].signals.resize(messages[25].signal_count);
    messages[25].signals[0] = Signal{ "acc_pedal_cmd_counter", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 255.0f, "" };
    messages[25].signals[1] = Signal{ "acc_pedal_cmd", 0, 16, 1, true, 0.01f, 0.0f, -150.0f, 655.35f, "%" };

    messages[26].id = 1400;
    messages[26].dlc = 8;
    messages[26].name = "brake_pressure_cmd";
    messages[26].transmitter = "Vector__XXX";
    messages[26].signal_count = 3;
    messages[26].signals.resize(messages[26].signal_count);
    messages[26].signals[0] = Signal{ "brk_pressure_cmd_counter", 40, 8, 1, false, 1.0f, 0.0f, 0.0f, 255.0f, "" };
    messages[26].signals[1] = Signal{ "F_brake_pressure_cmd", 0, 13, 1, false, 1.0f, 0.0f, 0.0f, 8191.0f, "" };
    messages[26].signals[2] = Signal{ "R_brake_pressure_cmd", 16, 13, 1, false, 1.0f, 0.0f, 0.0f, 8191.0f, "" };

    messages[27].id = 1311;
    messages[27].dlc = 8;
    messages[27].name = "wheel_potentiometer_data";
    messages[27].transmitter = "Vector__XXX";
    messages[27].signal_count = 4;
    messages[27].signals.resize(messages[27].signal_count);
    messages[27].signals[0] = Signal{ "wheel_potentiometer_RR", 32, 16, 1, false, 0.01f, 0.0f, 0.0f, 0.0f, "" };
    messages[27].signals[1] = Signal{ "wheel_potentiometer_RL", 48, 16, 1, false, 0.01f, 0.0f, 0.0f, 0.0f, "" };
    messages[27].signals[2] = Signal{ "wheel_potentiometer_FR", 16, 16, 1, false, 0.01f, 0.0f, 0.0f, 0.0f, "" };
    messages[27].signals[3] = Signal{ "wheel_potentiometer_FL", 0, 16, 1, false, 0.01f, 0.0f, 0.0f, 0.0f, "" };

    messages[28].id = 1310;
    messages[28].dlc = 8;
    messages[28].name = "wheel_strain_gauge";
    messages[28].transmitter = "Vector__XXX";
    messages[28].signal_count = 4;
    messages[28].signals.resize(messages[28].signal_count);
    messages[28].signals[0] = Signal{ "wheel_strain_gauge_RR", 48, 12, 1, false, 10.0f, 0.0f, 0.0f, 8190.0f, "" };
    messages[28].signals[1] = Signal{ "wheel_strain_gauge_RL", 32, 12, 1, false, 10.0f, 0.0f, 0.0f, 8190.0f, "" };
    messages[28].signals[2] = Signal{ "wheel_strain_gauge_FR", 16, 12, 1, false, 10.0f, 0.0f, 0.0f, 8190.0f, "" };
    messages[28].signals[3] = Signal{ "wheel_strain_gauge_FL", 0, 12, 1, false, 10.0f, 0.0f, 0.0f, 8190.0f, "" };

    messages[29].id = 1304;
    messages[29].dlc = 5;
    messages[29].name = "misc_report";
    messages[29].transmitter = "Vector__XXX";
    messages[29].signal_count = 5;
    messages[29].signals.resize(messages[29].signal_count);
    messages[29].signals[0] = Signal{ "battery_voltage", 0, 8, 1, false, 0.1f, 0.0f, 0.0f, 0.0f, "V" };
    messages[29].signals[1] = Signal{ "safety_switch_state", 32, 3, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[29].signals[2] = Signal{ "mode_switch_state", 16, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[29].signals[3] = Signal{ "sys_state", 8, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[29].signals[4] = Signal{ "raptor_rolling_counter", 19, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[30].id = 1303;
    messages[30].dlc = 8;
    messages[30].name = "steering_report";
    messages[30].transmitter = "Vector__XXX";
    messages[30].signal_count = 3;
    messages[30].signals.resize(messages[30].signal_count);
    messages[30].signals[0] = Signal{ "steering_motor_fdbk_counter", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 255.0f, "" };
    messages[30].signals[1] = Signal{ "primary_steering_angular_rate", 0, 16, 1, true, 0.1f, 0.0f, 0.0f, 0.0f, "deg/s" };
    messages[30].signals[2] = Signal{ "commanded_steering_rate", 24, 16, 1, true, 0.1f, 0.0f, 0.0f, 0.0f, "deg/s" };

    messages[31].id = 1302;
    messages[31].dlc = 3;
    messages[31].name = "accelerator_report";
    messages[31].transmitter = "Vector__XXX";
    messages[31].signal_count = 2;
    messages[31].signals.resize(messages[31].signal_count);
    messages[31].signals[0] = Signal{ "acc_pedal_fdbk_counter", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 255.0f, "" };
    messages[31].signals[1] = Signal{ "acc_pedal_fdbk", 0, 16, 1, false, 0.1f, 0.0f, 0.0f, 65535.0f, "%" };

    messages[32].id = 1301;
    messages[32].dlc = 5;
    messages[32].name = "brake_pressure_report";
    messages[32].transmitter = "Vector__XXX";
    messages[32].signal_count = 3;
    messages[32].signals.resize(messages[32].signal_count);
    messages[32].signals[0] = Signal{ "brk_pressure_fdbk_counter", 32, 8, 1, false, 1.0f, 0.0f, 0.0f, 255.0f, "" };
    messages[32].signals[1] = Signal{ "brake_pressure_fdbk_rear", 16, 13, 1, false, 1.0f, 0.0f, 0.0f, 8191.0f, "kPa" };
    messages[32].signals[2] = Signal{ "brake_pressure_fdbk_front", 0, 13, 1, false, 1.0f, 0.0f, 0.0f, 8191.0f, "kPa" };

    messages[33].id = 1300;
    messages[33].dlc = 8;
    messages[33].name = "wheel_speed_report";
    messages[33].transmitter = "Vector__XXX";
    messages[33].signal_count = 4;
    messages[33].signals.resize(messages[33].signal_count);
    messages[33].signals[0] = Signal{ "wheel_speed_RL", 32, 16, 1, false, 0.1f, 0.0f, 0.0f, 340.0f, "kmph" };
    messages[33].signals[1] = Signal{ "wheel_speed_FR", 16, 16, 1, false, 0.1f, 0.0f, 0.0f, 340.0f, "kmph" };
    messages[33].signals[2] = Signal{ "wheel_speed_FL", 0, 16, 1, false, 0.1f, 0.0f, 0.0f, 340.0f, "kmph" };
    messages[33].signals[3] = Signal{ "wheel_speed_RR", 48, 16, 1, false, 0.1f, 0.0f, 0.0f, 340.0f, "kmph" };

    messages[34].id = 1340;
    messages[34].dlc = 8;
    messages[34].name = "pt_report_1";
    messages[34].transmitter = "Vector__XXX";
    messages[34].signal_count = 7;
    messages[34].signals.resize(messages[34].signal_count);
    messages[34].signals[0] = Signal{ "throttle_position", 5, 16, 0, false, 0.1f, 0.0f, 0.0f, 100.0f, "%" };
    messages[34].signals[1] = Signal{ "current_gear", 21, 8, 0, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[34].signals[2] = Signal{ "engine_speed_rpm", 29, 16, 0, false, 1.0f, 0.0f, 0.0f, 16000.0f, "rpm" };
    messages[34].signals[3] = Signal{ "vehicle_speed_kmph", 45, 16, 0, false, 0.1f, 0.0f, 0.0f, 6553.5f, "kmph" };
    messages[34].signals[4] = Signal{ "engine_run_switch", 7, 1, 0, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[34].signals[5] = Signal{ "engine_state", 6, 1, 0, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[34].signals[6] = Signal{ "gear_shift_status", 61, 3, 0, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[35].id = 1341;
    messages[35].dlc = 8;
    messages[35].name = "pt_report_2";
    messages[35].transmitter = "Vector__XXX";
    messages[35].signal_count = 5;
    messages[35].signals.resize(messages[35].signal_count);
    messages[35].signals[0] = Signal{ "fuel_pressure_kPa", 7, 16, 0, false, 0.1f, 0.0f, 0.0f, 1000.0f, "kPa" };
    messages[35].signals[1] = Signal{ "engine_oil_pressure_kPa", 23, 16, 0, false, 0.1f, 0.0f, 0.0f, 0.0f, "kPa" };
    messages[35].signals[2] = Signal{ "coolant_temperature", 39, 8, 0, false, 1.0f, 0.0f, 0.0f, 255.0f, "C" };
    messages[35].signals[3] = Signal{ "transmission_temperature", 47, 8, 0, false, 1.0f, 0.0f, 0.0f, 255.0f, "C" };
    messages[35].signals[4] = Signal{ "transmission_pressure_kPa", 55, 16, 0, false, 1.0f, 0.0f, 0.0f, 0.0f, "kPa" };

    messages[36].id = 1342;
    messages[36].dlc = 8;
    messages[36].name = "diagnostic_report";
    messages[36].transmitter = "Vector__XXX";
    messages[36].signal_count = 17;
    messages[36].signals.resize(messages[36].signal_count);
    messages[36].signals[0] = Signal{ "sd_system_warning", 0, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[1] = Signal{ "sd_system_failure", 1, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[2] = Signal{ "sd_brake_warning1", 2, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[3] = Signal{ "sd_brake_warning2", 3, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[4] = Signal{ "sd_brake_warning3", 4, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[5] = Signal{ "sd_steer_warning1", 5, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[6] = Signal{ "sd_steer_warning2", 6, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[7] = Signal{ "sd_steer_warning3", 7, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[8] = Signal{ "motec_warning", 8, 8, 1, false, 1.0f, 0.0f, 0.0f, 31.0f, "" };
    messages[36].signals[9] = Signal{ "est1_oos_front_brk", 16, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[10] = Signal{ "est2_oos_rear_brk", 17, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[11] = Signal{ "est3_low_eng_speed", 18, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[12] = Signal{ "est4_sd_comms_loss", 19, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[13] = Signal{ "est5_motec_comms_loss", 20, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[14] = Signal{ "est6_sd_ebrake", 21, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[15] = Signal{ "adlink_hb_lost", 22, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[36].signals[16] = Signal{ "rc_lost", 23, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[37].id = 1200;
    messages[37].dlc = 8;
    messages[37].name = "base_to_car_summary";
    messages[37].transmitter = "Vector__XXX";
    messages[37].signal_count = 7;
    messages[37].signals.resize(messages[37].signal_count);
    messages[37].signals[0] = Signal{ "base_to_car_heartbeat", 0, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[37].signals[1] = Signal{ "track_flag", 4, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[37].signals[2] = Signal{ "veh_flag", 8, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[37].signals[3] = Signal{ "veh_rank", 12, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[37].signals[4] = Signal{ "lap_count", 24, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[37].signals[5] = Signal{ "lap_distance", 32, 16, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "m" };
    messages[37].signals[6] = Signal{ "round_target_speed", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "mph" };

    messages[38].id = 1208;
    messages[38].dlc = 8;
    messages[38].name = "base_to_car_timing";
    messages[38].transmitter = "Vector__XXX";
    messages[38].signal_count = 3;
    messages[38].signals.resize(messages[38].signal_count);
    messages[38].signals[0] = Signal{ "laps", 0, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[38].signals[1] = Signal{ "lap_time", 8, 24, 1, false, 0.1f, 0.0f, 0.0f, 0.0f, "ms" };
    messages[38].signals[2] = Signal{ "time_stamp", 32, 32, 1, false, 0.1f, 0.0f, 0.0f, 0.0f, "ms" };

    messages[39].id = 1209;
    messages[39].dlc = 8;
    messages[39].name = "rest_of_field";
    messages[39].transmitter = "Vector__XXX";
    messages[39].signal_count = 6;
    messages[39].signals.resize(messages[39].signal_count);
    messages[39].signals[0] = Signal{ "comp_veh_num", 0, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[39].signals[1] = Signal{ "comp_rank", 12, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[39].signals[2] = Signal{ "comp_veh_flag", 8, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[39].signals[3] = Signal{ "comp_laps_count", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[39].signals[4] = Signal{ "comp_lap_distance", 24, 16, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "m" };
    messages[39].signals[5] = Signal{ "comp_speed", 48, 16, 1, false, 0.1f, 0.0f, 0.0f, 0.0f, "kmph" };

    messages[40].id = 1343;
    messages[40].dlc = 8;
    messages[40].name = "pt_report_3";
    messages[40].transmitter = "Vector__XXX";
    messages[40].signal_count = 7;
    messages[40].signals.resize(messages[40].signal_count);
    messages[40].signals[0] = Signal{ "engine_oil_temperature", 0, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "degC" };
    messages[40].signals[1] = Signal{ "torque_wheels", 8, 16, 1, true, 0.1f, 0.0f, 0.0f, 0.0f, "Nm" };
    messages[40].signals[2] = Signal{ "driver_traction_aim_swicth_fbk", 24, 8, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[40].signals[3] = Signal{ "driver_traction_range_switch_fbk", 32, 8, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[40].signals[4] = Signal{ "push2pass_status", 40, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[40].signals[5] = Signal{ "push2pass_budget_s", 44, 10, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "s" };
    messages[40].signals[6] = Signal{ "push2pass_active_app_limit", 54, 7, 1, false, 1.0f, 0.0f, 0.0f, 100.0f, "%" };

    messages[41].id = 228;
    messages[41].dlc = 8;
    messages[41].name = "mylaps_report_1";
    messages[41].transmitter = "Vector__XXX";
    messages[41].signal_count = 2;
    messages[41].signals.resize(messages[41].signal_count);
    messages[41].signals[0] = Signal{ "mylaps_speed", 0, 16, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "kmph" };
    messages[41].signals[1] = Signal{ "mylaps_heading", 16, 16, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "degrees" };

    messages[42].id = 226;
    messages[42].dlc = 8;
    messages[42].name = "mylaps_report_2";
    messages[42].transmitter = "Vector__XXX";
    messages[42].signal_count = 2;
    messages[42].signals.resize(messages[42].signal_count);
    messages[42].signals[0] = Signal{ "Latitude", 0, 32, 1, true, 1e-07f, 0.0f, 0.0f, 0.0f, "" };
    messages[42].signals[1] = Signal{ "Longitude", 32, 32, 1, true, 1e-07f, 0.0f, 0.0f, 0.0f, "" };

    messages[43].id = 1406;
    messages[43].dlc = 8;
    messages[43].name = "dash_switches_cmd";
    messages[43].transmitter = "Vector__XXX";
    messages[43].signal_count = 5;
    messages[43].signals.resize(messages[43].signal_count);
    messages[43].signals[0] = Signal{ "driver_traction_aim_switch", 0, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[43].signals[1] = Signal{ "driver_traction_range_switch", 4, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[43].signals[2] = Signal{ "brake_bias_aim_switch", 8, 4, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[43].signals[3] = Signal{ "drive_steering_gain_cntrl_switch", 12, 1, 1, false, 1.0f, 0.0f, 0.0f, 1.0f, "" };
    messages[43].signals[4] = Signal{ "push2pass_switch", 13, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[44].id = 1450;
    messages[44].dlc = 8;
    messages[44].name = "ct_vehicle_acc_feedback";
    messages[44].transmitter = "Vector__XXX";
    messages[44].signal_count = 3;
    messages[44].signals.resize(messages[44].signal_count);
    messages[44].signals[0] = Signal{ "long_ct_vehicle_acc_fbk", 0, 16, 1, true, 0.01f, 0.0f, -327.68f, 327.67f, "G" };
    messages[44].signals[1] = Signal{ "lat_ct_vehicle_acc_fbk", 16, 16, 1, true, 0.01f, 0.0f, -327.68f, 327.67f, "G" };
    messages[44].signals[2] = Signal{ "vertical_ct_vehicle_acc_fbk", 32, 16, 1, true, 0.01f, 0.0f, -327.68f, 327.67f, "G" };

    messages[45].id = 1250;
    messages[45].dlc = 8;
    messages[45].name = "marelli_report_1";
    messages[45].transmitter = "Vector__XXX";
    messages[45].signal_count = 5;
    messages[45].signals.resize(messages[45].signal_count);
    messages[45].signals[0] = Signal{ "marelli_track_flag", 0, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[45].signals[1] = Signal{ "marelli_vehicle_flag", 8, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[45].signals[2] = Signal{ "marelli_sector_flag", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[45].signals[3] = Signal{ "marelli_rc_base_sync_check", 24, 1, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[45].signals[4] = Signal{ "marelli_rc_lte_rssi", 25, 8, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[46].id = 1251;
    messages[46].dlc = 8;
    messages[46].name = "marelli_report_2";
    messages[46].transmitter = "Vector__XXX";
    messages[46].signal_count = 2;
    messages[46].signals.resize(messages[46].signal_count);
    messages[46].signals[0] = Signal{ "marelli_gps_lat", 0, 32, 1, true, 1e-07f, 0.0f, 0.0f, 0.0f, "�" };
    messages[46].signals[1] = Signal{ "marelli_gps_long", 32, 32, 1, true, 1e-07f, 0.0f, 0.0f, 0.0f, "�" };

    messages[47].id = 1405;
    messages[47].dlc = 8;
    messages[47].name = "ct_report_2";
    messages[47].transmitter = "Vector__XXX";
    messages[47].signal_count = 3;
    messages[47].signals.resize(messages[47].signal_count);
    messages[47].signals[0] = Signal{ "marelli_track_flag_ack", 0, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[47].signals[1] = Signal{ "marelli_vehicle_flag_ack", 8, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };
    messages[47].signals[2] = Signal{ "marelli_sector_flag_ack", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 0.0f, "" };

    messages[48].id = 1313;
    messages[48].dlc = 7;
    messages[48].name = "steering_report_extd_2";
    messages[48].transmitter = "Vector__XXX";
    messages[48].signal_count = 7;
    messages[48].signals.resize(messages[48].signal_count);
    messages[48].signals[0] = Signal{ "motor_duty_cycle_cmd", 0, 8, 1, true, 1.0f, 0.0f, -100.0f, 100.0f, "%" };
    messages[48].signals[1] = Signal{ "motor_duty_cycle_fbk", 8, 8, 1, true, 1.0f, 0.0f, -100.0f, 100.0f, "%" };
    messages[48].signals[2] = Signal{ "motor_current_fbk", 16, 8, 1, false, 1.0f, 0.0f, 0.0f, 128.0f, "A" };
    messages[48].signals[3] = Signal{ "sbw_ecu_voltage", 24, 8, 1, false, 0.1f, 0.0f, 0.0f, 25.5f, "V" };
    messages[48].signals[4] = Signal{ "sbw_ecu_temp", 32, 8, 1, false, 1.0f, 0.0f, 0.0f, 130.0f, "degC" };
    messages[48].signals[5] = Signal{ "sbw_error_code", 40, 8, 1, false, 1.0f, 0.0f, 0.0f, 120.0f, "" };
    messages[48].signals[6] = Signal{ "sbw_motor_torque_estimate", 48, 8, 1, false, 1.0f, 0.0f, 0.0f, 110.0f, "Nm" };

    messages[49].id = 1314;
    messages[49].dlc = 8;
    messages[49].name = "brake_report_extd";
    messages[49].transmitter = "Vector__XXX";
    messages[49].signal_count = 4;
    messages[49].signals.resize(messages[49].signal_count);
    messages[49].signals[0] = Signal{ "F_brk_pos_cmd", 0, 16, 1, false, -0.354f, 7500.0f, 0.0f, 15000.0f, "" };
    messages[49].signals[1] = Signal{ "F_brk_pos_fbk", 16, 16, 1, false, 1.0f, 0.0f, 0.0f, 15000.0f, "" };
    messages[49].signals[2] = Signal{ "R_brk_pos_cmd", 32, 16, 1, false, -0.354f, 7500.0f, 0.0f, 15000.0f, "" };
    messages[49].signals[3] = Signal{ "R_brk_pos_fbk", 48, 16, 1, false, 1.0f, 0.0f, 0.0f, 15000.0f, "" };

    messages[50].id = 1315;
    messages[50].dlc = 4;
    messages[50].name = "brake_report_extd_2";
    messages[50].transmitter = "Vector__XXX";
    messages[50].signal_count = 2;
    messages[50].signals.resize(messages[50].signal_count);
    messages[50].signals[0] = Signal{ "f_brake_act_force", 0, 12, 1, false, 1.0f, 0.0f, 0.0f, 3000.0f, "N" };
    messages[50].signals[1] = Signal{ "r_brake_act_force", 16, 12, 1, false, 1.0f, 0.0f, 0.0f, 3000.0f, "N" };

    messages[51].id = 1350;
    messages[51].dlc = 8;
    messages[51].name = "novatel_report";
    messages[51].transmitter = "Vector__XXX";
    messages[51].signal_count = 2;
    messages[51].signals.resize(messages[51].signal_count);
    messages[51].signals[0] = Signal{ "novatel_lat", 0, 32, 1, true, 1e-07f, 0.0f, 0.0f, 0.0f, "deg" };
    messages[51].signals[1] = Signal{ "novatel_long", 32, 32, 1, true, 1e-07f, 0.0f, 0.0f, 0.0f, "deg" };

    messages[52].id = 1316;
    messages[52].dlc = 4;
    messages[52].name = "steering_report_extd_3";
    messages[52].transmitter = "Vector__XXX";
    messages[52].signal_count = 3;
    messages[52].signals.resize(messages[52].signal_count);
    messages[52].signals[0] = Signal{ "steering_p_contribution", 0, 10, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "%" };
    messages[52].signals[1] = Signal{ "steering_i_contribution", 10, 10, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "%" };
    messages[52].signals[2] = Signal{ "steering_d_contribution", 20, 10, 1, true, 1.0f, 0.0f, 0.0f, 0.0f, "%" };

    return messages;
}
