#include <chrono>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <vector>
#include <fstream>
#include <algorithm>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "iac_qos.h"
#include "VESIAPI.h"
#include "AtomicSensorInformation.h"
#include "CameraDataDeserializerDefault.h"
#include "CameraDataDeserializerTypes.h"



namespace aurelion_ros2_bridge
{
    class AurelionRos2BridgeNode : public rclcpp::Node
    {

    public:
        AurelionRos2BridgeNode();
        ~AurelionRos2BridgeNode() = default;

    private:

        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarDataPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radarDataPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cameraDataPublisher_;

        // Sensors publishing functions
        VESIAPI sensorApi;
        void connectToSensorApi(int16_t max_retries);
        void publishLidarData(uint8_t sensorId);
        void publishRadarData(uint8_t sensorId);
        void publishCameraData(std::unique_ptr<CameraDataDeserializerDefault>& deserializer, uint8_t sensorId, int cameraImageType);

        // Parameters
        int MAX_SENSOR_RESULT_PRINTOUTS = 3;
        bool verbosePrinting = false;
        std::string sensorType;
        uint8_t sensorId;
        std::string rosTopic;
        int cameraImageType;


    };

} // namespace aurelion_ros2_bridge
