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
#include <atomic>
#include <unistd.h>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <optional>
#include <string_view>
#include <unordered_map>

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

        rclcpp::Publisher<vectornav_msgs::msg::GpsGroup>::SharedPtr verctorNavGpsGroupPublisher;

        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestPosPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr novaTelBestGNSSPosPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestVelPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr novaTelBestGNSSVelPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::INSPVA>::SharedPtr novaTelInspvaPublisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::HEADING2>::SharedPtr novaTelHeading2Publisher;
        rclcpp::Publisher<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr novaTelRawImuPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr novaTelRawImuXPublisher;

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
        std::vector <double> measured_vesi_times;
        int64_t timeStartNanosec = 0;
        int64_t timeEndNanosec = 0;
        int64_t timeStartVESICallBackNanosec = 0;
        int64_t timeEndVESICallBackNanosec = 0;
        std::ofstream myfile;
        std::string pathTimeRecord;
        bool enableTimeRecord;

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
        void publishFoxgloveMap(uint8_t fellowID);
        void publishFoxgloveSceneUpdate();
        void publishSimulationState();
        void publishBaseToCarSummary();
        void publishMarelliReport();
        void publishMiscRCReport();
        void publishPtReport();
        void publishSteeringReport();
        void publishBrakeReport();
        void publishAcceleratorReport();
        void publishWheelReport();
        void publishMiscReport();
        void publishDiagnosticReport();
        void publishVectorIndependentSigMsg();
        void publishNovatelReport();
        void publishVectorNavData();
        void publishNovatelData(uint8_t novatelID);

        rclcpp::Publisher<autonoma_msgs::msg::GroundTruthArray>::SharedPtr groundTruthArrayPublisher_;
        void publishGroundTruthArray();

        // socket helpers
        int open_socket(const std::string &iface);
        void can_reader_loop(int sock, const std::string &bus_id);
        void can_write(int sock, struct can_frame frame);
        template <typename T>
        void insertBits(uint8_t* data, Signal signal_information, T physical_value);
        int32_t extractBits(const uint8_t* data, Signal signal_information) const;

        int can_socket = -1;
        struct can_frame can_out_frame;
        std::vector<Message> can_message_info;
        using SignalLookupMap = std::unordered_map<std::string_view, const Signal*>;
        std::unordered_map<uint32_t, const Message*> message_lookup_;
        std::unordered_map<uint32_t, SignalLookupMap> message_signal_lookup_;
        void buildMessageLookup();
        const Message* findMessage(uint32_t message_id) const;
        const Signal* findSignal(uint32_t message_id, std::string_view signal_name) const;
        const SignalLookupMap* findSignalLookup(uint32_t message_id) const;
        std::optional<double> extractSignalScaled(uint32_t message_id, std::string_view signal_name, const uint8_t* data) const;

        std::vector<uint8_t> canbus_raw_buffer_;
        std::mutex feedback_mutex_;
        std::atomic<bool> stop_reader_{false};
    };
} // namespace asm_socketcan_bridge

