#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <cstring>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <vector>
#include <fstream>
#include <algorithm>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <unistd.h>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <initializer_list>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>

#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/header.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"

#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/rawimu.hpp"

#include "foxglove_msgs/msg/scene_update.hpp"

#include "autonoma_msgs/msg/ground_truth_array.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "iac_qos.h"

#include "VESIAPI.h"
#include "VESIResultData.h"

#include "ASMBus.h"
#include "RaceControlInterface.h"
#include "dbc_structure.h"

namespace asm_socketcan_bridge
{
    class AsmSocketCanBridgeNode : public rclcpp::Node
    {

    public:
        AsmSocketCanBridgeNode();
        ~AsmSocketCanBridgeNode() override;

    private:
        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr foxgloveMapPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr foxgloveMapPublisher0_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr foxgloveMapPublisher1_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr foxgloveMapPublisher2_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr foxgloveMapPublisher3_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resetCommandPublisher_;
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr simClockTimePublisher_;

        rclcpp::Publisher<vectornav_msgs::msg::CommonGroup>::SharedPtr verctorNavCommonGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::AttitudeGroup>::SharedPtr verctorNavAttitudeGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::ImuGroup>::SharedPtr verctorNavImuGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::InsGroup>::SharedPtr verctorNavInsGroupPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr verctorNavGpsGroupLeftPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr verctorNavGpsGroupRightPublisher_;
        rclcpp::Publisher<vectornav_msgs::msg::TimeGroup>::SharedPtr verctorNavTimeGroupPublisher_;



        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestPosPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestGNSSPosPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestVelPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestGNSSVelPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::INSPVA>::SharedPtr novaTelInspvaPublisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::HEADING2>::SharedPtr novaTelHeading2Publisher1_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr novaTelRawImuPublisher1_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelRawImuXPublisher1_;

        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestPosPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestGNSSPosPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestVelPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestGNSSVelPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::INSPVA>::SharedPtr novaTelInspvaPublisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::HEADING2>::SharedPtr novaTelHeading2Publisher2_;
        rclcpp::Publisher<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr novaTelRawImuPublisher2_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelRawImuXPublisher2_;

        // Timer
        rclcpp::TimerBase::SharedPtr vesiAcquisitionTimer_;
        rclcpp::TimerBase::SharedPtr updateVESIVehicleInputs_;

        // Subsciber
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr useCustomRaceControlSource_;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr simTimeIncrease_;

        // reader threads
        std::thread reader_thread1;
        std::thread reader_thread2;

        // writer threads
        std::thread writer_thread1;
        std::thread writer_thread2;

        // Parameter
        bool maneuverStarted = false;
        bool vesiDataAvailabe = false;
        bool feedbackDataAvailabe = false;
        bool raptorDataAvailabe = false;
        bool useCustomRaceControl = false;

        bool verbosePrinting = false;
        bool receivedMessagePrinting = false;
        bool receivedDecodedMessagePrinting = false;
        bool sentMessagePrinting = false;

        bool simModeEnabled = false;
        bool numberWarningSent = false;
        bool stackFeedbackConnectionWarningSent = false;
        uint8_t prestart_rolling_counter;

        int16_t max_retries;

        // DBC paths
        std::string can1_dbc_path;

        // socketcan interface names
        std::string can_iface;

        // Execution duration logging
        double duration_send_feedback_ms = 0.0;
        double duration_request_data_ms = 0.0;
        double duration_cast_ms = 0.0;
        double duration_callback_interval_ms = 0.0;
        int64_t last_callback_start_ns = 0;
        std::ofstream myfile;
        std::string pathTimeRecord;
        bool enableTimeRecord;
        bool metrics_ready = false;

        // Simulated clock
        uint32_t nsec = 0;
        uint32_t sec = 0;
        uint64_t simTotalMsec = 0;
        rosgraph_msgs::msg::Clock simClockTime;
        rclcpp::TimerBase::SharedPtr updateSimClock_;

        // Custom publish frequencies
        uint32_t pubIntervalRaceControlData;
        uint32_t pubIntervalVehicleData;
        uint32_t pubIntervalPowertrainData;
        uint32_t pubIntervalGroundTruthArray;
        uint32_t pubIntervalVectorNavData;
        uint32_t pubIntervalNovatelData;
        uint32_t pubIntervalFoxgloveMap;

        // Custom Structures
        VESIResultData feedbackCmds;
        VESIAPI api;
        ASMBus canBusStorage_{};
        ASMBus *canBus = nullptr;
        VESIResultData feedbackCmd;

        void initializeFeedback();

        // Callbacks
        void vesiCallback();
        void simClockTimeCallback();
        void initialSimClockPublish();
        void sendVehicleFeedbackToSimulation();
        void subscribeVehicleCommandsCallback();
        void subscribeRaptorCommandsCallback();
        void switchRaceControlSourceCallback(const std_msgs::msg::Bool &msg);
        void simTimeIncreaseCallback(const std_msgs::msg::UInt16 &msg);

        // Publishing functions
        void publish_map2d_ego_position();
        void publish_map2d_fellow1_position();
        void publish_map2d_fellow2_position();
        void publish_map2d_fellow3_position();
        void publishFoxgloveSceneUpdate();
        void publishSimulationState();
        void publish_base_to_car_summary();
        void publish_marelli_report_1();
        void publish_marelli_report_2();
        void publish_base_to_car_timing();
        void publish_rest_of_field();
        void publish_pt_report_1();
        void publish_pt_report_2();
        void publish_pt_report_3();
        void publish_steering_report();
        void publish_steering_report_extd();
        void publish_steering_report_extd_2();
        void publish_steering_report_extd_3();
        void publish_brake_pressure_report();
        void publish_brake_report_extd();
        void publish_brake_report_extd_2();
        void publish_accelerator_report();
        void publish_Tire_Temp_RR_1();
        void publish_Tire_Temp_RR_2();
        void publish_Tire_Temp_RR_3();
        void publish_Tire_Temp_RR_4();
        void publish_Tire_Temp_RL_1();
        void publish_Tire_Temp_RL_2();
        void publish_Tire_Temp_RL_3();
        void publish_Tire_Temp_RL_4();
        void publish_Tire_Temp_FR_1();
        void publish_Tire_Temp_FR_2();
        void publish_Tire_Temp_FR_3();
        void publish_Tire_Temp_FR_4();
        void publish_Tire_Temp_FL_1();
        void publish_Tire_Temp_FL_2();
        void publish_Tire_Temp_FL_3();
        void publish_Tire_Temp_FL_4();
        void publish_Tire_Pressure_RR();
        void publish_Tire_Pressure_RL();
        void publish_Tire_Pressure_FR();
        void publish_Tire_Pressure_FL();
        void publish_wheel_strain_gauge();
        void publish_wheel_potentiometer_data();
        void publish_wheel_speed_report();
        void publish_misc_report();
        void publish_diagnostic_report();
        void publish_VECTOR__INDEPENDENT_SIG_MSG();
        void publish_novatel_report();
        void publish_vectornav_attitude_group();
        void publish_vectornav_common_group();
        void publish_vectornav_imu_group();
        void publish_vectornav_gps_group_left();
        void publish_vectornav_gps_group_right();
        void publish_vectornav_ins_group();
        void publish_vectornav_time_group();
        void publish_novatel_bestpos(uint8_t novatel_id);
        void publish_novatel_bestgnsspos(uint8_t novatel_id);
        void publish_novatel_bestvel(uint8_t novatel_id);
        void publish_novatel_bestgnssvel(uint8_t novatel_id);
        void publish_novatel_inspva(uint8_t novatel_id);
        void publish_novatel_heading2(uint8_t novatel_id);
        void publish_novatel_rawimu(uint8_t novatel_id);
        void publish_novatel_rawimux(uint8_t novatel_id);

        rclcpp::Publisher<autonoma_msgs::msg::GroundTruthArray>::SharedPtr groundTruthArrayPublisher_;
        void publishGroundTruthArray();

        // socket helpers
        int open_socket(const std::string &iface);
        void can_reader_loop(int sock, const std::string &bus_id);
        void can_write(int sock, const struct can_frame &frame);
        template <typename T>
        void insertBits(uint8_t* data, Signal signal_information, T physical_value);
        int32_t extractBits(const uint8_t* data, Signal signal_information) const;

        struct PreparedCanMessage {
            struct can_frame frame;
            const Message* metadata;
        };

        int can_socket = -1;
        std::vector<Message> can_message_info;
        using SignalLookupMap = std::unordered_map<std::string_view, const Signal*>;
        std::unordered_map<uint32_t, const Message*> message_lookup_;
        std::unordered_map<uint32_t, SignalLookupMap> message_signal_lookup_;
        std::unordered_map<std::string_view, const Message*> message_name_lookup_;
        void buildMessageLookup();
        const Message* findMessageByID(uint32_t message_id) const;
        const Signal* findSignal(uint32_t message_id, std::string_view signal_name) const;
        const SignalLookupMap* findSignalLookup(uint32_t message_id) const;
        const Message* findMessageByName(std::string_view message_name) const;
        std::optional<PreparedCanMessage> prepareCanMessage(std::string_view message_name);
        void finalizeCanMessage(const PreparedCanMessage &message);
        template <typename Populator>
        bool publishCanMessage(std::string_view message_name, Populator &&populate)
        {
            auto prepared = prepareCanMessage(message_name);
            if (!prepared) {
                return false;
            }
            {
                std::shared_lock<std::shared_mutex> lock(can_bus_mutex_);
                if (!this->canBus) {
                    RCLCPP_ERROR(get_logger(), "canBus pointer is null.");
                    return false;
                }
                std::forward<Populator>(populate)(*prepared, *this->canBus);
            }
            finalizeCanMessage(*prepared);
            return true;
        }
        template <typename Func>
        bool withCanBusShared(Func &&func) const
        {
            std::shared_lock<std::shared_mutex> lock(can_bus_mutex_);
            if (!this->canBus) {
                RCLCPP_ERROR(get_logger(), "canBus pointer is null.");
                return false;
            }
            std::forward<Func>(func)(*this->canBus);
            return true;
        }
        void setHeader(std_msgs::msg::Header &header, std::string_view frame_id) const;
        void populateGpsGroupMessage(vectornav_msgs::msg::GpsGroup &message, const gps_group &source);
        void populateBestPosMessage(novatel_oem7_msgs::msg::BESTPOS &message, const nova_tel_pwr_pak &data) const;
        void populateBestVelMessage(novatel_oem7_msgs::msg::BESTVEL &message, const nova_tel_pwr_pak &data) const;
        void populateInspvaMessage(novatel_oem7_msgs::msg::INSPVA &message, const nova_tel_pwr_pak &data) const;
        void populateHeading2Message(novatel_oem7_msgs::msg::HEADING2 &message, const nova_tel_pwr_pak &data) const;
        void populateRawImuMessage(novatel_oem7_msgs::msg::RAWIMU &message, const nova_tel_pwr_pak &data) const;
        void populateRawImuXMessage(sensor_msgs::msg::Imu &message, const nova_tel_pwr_pak &data) const;
        void publishFoxgloveMapEntry(uint8_t fellowID);
        std::optional<double> extractSignalScaled(uint32_t message_id, std::string_view signal_name, const uint8_t* data) const;

        std::vector<uint8_t> canbus_raw_buffer_;
        std::mutex feedback_mutex_;
        mutable std::shared_mutex can_bus_mutex_;
        std::mutex metrics_mutex_;
        std::mutex can_socket_mutex_;
        std::atomic<bool> stop_reader_{false};
    };
} // namespace asm_socketcan_bridge
