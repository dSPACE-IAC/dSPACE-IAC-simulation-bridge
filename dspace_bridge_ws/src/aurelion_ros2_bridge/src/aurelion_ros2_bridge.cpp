#include "aurelion_ros2_bridge.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace aurelion_ros2_bridge {

  AurelionRos2BridgeNode::AurelionRos2BridgeNode() : Node("aurelion_ros2_bridge_node")
  {

    //-----------------------------Configuration--------------------------------
    if(std::getenv("VESI_SENSOR_IP")){
      this->sensorApi.setSimManagerHost(std::getenv("VESI_SENSOR_IP"));
      std::cout << "Set V-ESI sensor IP to: "<< std::getenv("VESI_SENSOR_IP") << std::endl;
    } else {
      this->sensorApi.setSimManagerHost("127.0.0.1");
      std::cout << "Set V-ESI sensor IP to: 127.0.0.1 (default)" << std::endl;
    }
  
    if(std::getenv("VESI_SENSOR_PORT")){
      this->sensorApi.setSimManagerPort(std::stoi(std::getenv("VESI_SENSOR_PORT")));
      std::cout << "Set V-ESI sensor Port to: "<< std::getenv("VESI_SENSOR_PORT") << std::endl;
    } else {
      this->sensorApi.setSimManagerPort(12345);
      std::cout << "Set V-ESI sensor Port to: 12345 (default)"<< std::endl;
    }

    if(std::getenv("SENSOR_TYPE")){
      this->sensorType = std::string(std::getenv("SENSOR_TYPE"));
      std::cout << "Sensor type: " << this->sensorType << std::endl;
    }

    if(std::getenv("SENSOR_ID")){
      this->sensorId = static_cast<uint8_t>(std::stoi(std::getenv("SENSOR_ID")));
      std::cout << "Sensor Id: " << static_cast<int>(this->sensorId) << std::endl;
    }

    if(std::getenv("ROS_TOPIC")){
      this->rosTopic = std::string(std::getenv("ROS_TOPIC"));
      std::cout << "ROS topic: "<< std::getenv("ROS_TOPIC") << std::endl;
    } else {
      this->rosTopic = "sensor_data";
      std::cout << "Warning: No ROS topic name specified"<< std::endl;
    }

    if(std::getenv("VERBOSE_PRINT") && (std::string(std::getenv("VERBOSE_PRINT")) == "true"))
    {
      this->verbosePrinting = true;
    }

    //-----------------------------Connect to V-ESI--------------------------------

    std::list<uint8_t> requiredSensorIds{this->sensorId};
    this->sensorApi.setRequiredSensorIDs(requiredSensorIds);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_iac);

    int16_t max_retries = 10;
    AurelionRos2BridgeNode::connectToSensorApi(max_retries);
    std::cout << "Setup done." << '\n';

    //-----------------------------Sensors--------------------------------

    if(this->sensorType == "LIDAR" || this->sensorType ==  "Lidar" || this->sensorType == "lidar")
    {
      this->lidarDataPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->rosTopic, qos);
      while(rclcpp::ok())
      {
        //auto start = std::chrono::high_resolution_clock::now();
        try
        {
           AurelionRos2BridgeNode::publishLidarData(this->sensorId);
        }
        catch(const std::exception& e)
        {
          std::cerr << "Publishing of lidar data failed: " << e.what() << '\n';
          rclcpp::sleep_for(5s);
          AurelionRos2BridgeNode::connectToSensorApi(max_retries);
        }
        //auto end = std::chrono::high_resolution_clock::now();
        //std::chrono::duration<double, std::milli> duration = end - start;
        //std::cout << "Function execution time: " << duration.count() << " ms" << std::endl;
      };
    }

    if(this->sensorType == "RADAR" || this->sensorType == "Radar" || this->sensorType == "radar")
    {
      this->radarDataPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->rosTopic, qos);
      while(rclcpp::ok())
      {
        try
        {
          AurelionRos2BridgeNode::publishRadarData(this->sensorId);
        }
        catch(const std::exception& e)
        {
          std::cerr << "Publishing of radar data failed: " << e.what() << '\n';
          rclcpp::sleep_for(5s);
          AurelionRos2BridgeNode::connectToSensorApi(max_retries);
        }
      };
    }

    if(this->sensorType == "CAMERA" || this->sensorType == "Camera" || this->sensorType == "camera")
    {
      this->cameraImageType = 0;
      this->cameraDataPublisher_ = this->create_publisher<sensor_msgs::msg::Image>(this->rosTopic, qos);
      std::unique_ptr<CameraDataDeserializerDefault> cameraDeserializer(new CameraDataDeserializerDefault);
      cameraDeserializer->Initialize();
      while(rclcpp::ok())
      {
        try
        {
          AurelionRos2BridgeNode::publishCameraData(cameraDeserializer, this->sensorId, this->cameraImageType);
        }
        catch(const std::exception& e)
        {
          std::cerr << "Publishing of camera data failed: " << e.what() << '\n';
          rclcpp::sleep_for(5s);
          AurelionRos2BridgeNode::connectToSensorApi(max_retries);
        }
      };
    }
  }

  void AurelionRos2BridgeNode::connectToSensorApi(int16_t max_retries)
  {

    std::cout << "Trying to connect to sensor V-ESI..." << std::endl;
    bool vesiSensorConnection = false;
    int16_t retries = 1;
    while (vesiSensorConnection == false)
    {
      try
      {
        this->sensorApi.connect();
        std::cout << "V-ESI connection configured." << std::endl;
        vesiSensorConnection = true;
      }
      catch(const std::exception& e)
      {
        std::cerr << "Failed to configure V-ESI: " << e.what() << std::endl;
        if (retries < max_retries)
        {
          std::cout << "Failed for the " << retries << ". time. Try again." << std::endl;
          retries++;
          rclcpp::sleep_for(5s);
        }
        else
        {
          std::cout << "Failed too often. Exit." << std::endl;
          throw e;
        }
      }
    }
  }

  void AurelionRos2BridgeNode::publishLidarData(uint8_t sensorId)
  {

    // Create instance of sensor storage class and request data
    AtomicSensorInformation sensorData;
    this->sensorApi.requestSensorData(this->sensorId, &sensorData);

    // Write data into lidar frame
    CompositeLidarFrame lidarFrame;
    sensorData.getSensorData(&lidarFrame);

    // Print some data points
    if(this->verbosePrinting)
    {
      std::cout << "\tRequested lidar sensor with ID " << unsigned(sensorId) << " at maneuver time: " << sensorData.getSensorHeader().m_uiTimeStamp << "\xE6s" << std::endl;
      for (size_t i = 0; i < lidarFrame.LidarPoints.size() && i < this->MAX_SENSOR_RESULT_PRINTOUTS; i++)
      {
          const PointCloudLidarPoint& point = lidarFrame.LidarPoints[i];
          std::cout << "\t\tP[" << i << "] azimuth=" << point.Azimuth << " elevation=" << point.Elevation << " distance=" << point.Distance << std::endl;
      }
    }

    // Fill lidar data message
    auto lidarData = sensor_msgs::msg::PointCloud2();
    lidarData.header.stamp = this->now();
    lidarData.header.frame_id = "Lidar data";
    lidarData.height = 1;
    lidarData.width = lidarFrame.LidarPoints.size(); // Number of lidar points
    lidarData.is_dense = true;
    lidarData.is_bigendian = false;
    lidarData.point_step = 7 * sizeof(float); // azimuth, elevation, depth, x, y, z, reflectivity
    lidarData.row_step = lidarData.point_step * lidarData.width;
    lidarData.data.resize(lidarData.row_step * lidarData.height);

    // Define fields
    sensor_msgs::PointCloud2Modifier modifier(lidarData);
    modifier.setPointCloud2Fields(
      7,
      "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32,
      "elevation", 1, sensor_msgs::msg::PointField::FLOAT32,
      "depth", 1, sensor_msgs::msg::PointField::FLOAT32,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "reflectance", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    // Fill ROS2::PointCloud2 message
    sensor_msgs::PointCloud2Iterator<float> iter_azi(lidarData, "azimuth");
    sensor_msgs::PointCloud2Iterator<float> iter_ele(lidarData, "elevation");
    sensor_msgs::PointCloud2Iterator<float> iter_dep(lidarData, "depth");
    sensor_msgs::PointCloud2Iterator<float> iter_x(lidarData, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(lidarData, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(lidarData, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_refl(lidarData, "reflectance");

    //std::cout << "Number of Points: " << lidarData.width << std::endl;

    // Convert to Cartesian coordinates
    for (size_t i = 0; i < lidarData.width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_azi, ++iter_ele, ++iter_dep, ++iter_refl)
    {
        const PointCloudLidarPoint& point = lidarFrame.LidarPoints[i];
        float azimuthRad   = point.Azimuth * M_PI / 180.0;
        float elevationRad = point.Elevation * M_PI / 180.0;
        *iter_azi = static_cast<float>(point.Azimuth);
        *iter_ele = static_cast<float>(point.Elevation);
        *iter_dep = static_cast<float>(point.Distance);
        *iter_x = static_cast<float>(point.Distance * cos(elevationRad) * cos(azimuthRad));
        *iter_y = static_cast<float>(point.Distance * cos(elevationRad) * sin(azimuthRad) * (-1));
        *iter_z = static_cast<float>(point.Distance * sin(elevationRad));
        *iter_refl = static_cast<float>(point.Reflectivity);
    }

    // Publish ROS2::PointCloud2 message
    this->lidarDataPublisher_->publish(lidarData);

  }

  void AurelionRos2BridgeNode::publishRadarData(uint8_t sensorId)
  {

    // Create instance of sensor storage class and request data
    AtomicSensorInformation sensorData;
    this->sensorApi.requestSensorData(sensorId, &sensorData);

    // Write data into detections
    std::vector<Detection> detections;
    sensorData.getSensorData(&detections);

    // Print some data points
    if(this->verbosePrinting)
    {
      std::cout << "\tRequested radar sensor with ID " << unsigned(sensorId) << " at maneuver time: " << sensorData.getSensorHeader().m_uiTimeStamp << "\xE6s" << std::endl;
      for (size_t i = 0; i < detections.size() && i < this->MAX_SENSOR_RESULT_PRINTOUTS; i++)
      {
          const Detection& detection = detections[i];
          std::cout << "\t\tDetection [" << i << "]: azimuth=" << detection.AzimuthAngle << " elevation=" << detection.ElevationAngle << " radial_distance=" << detection.RadialDistance
                    << " radial_velocity=" << detection.RadialVelocity << std::endl;
      }
    }

    // Fill radar data message
    auto radarData = sensor_msgs::msg::PointCloud2();
    radarData.header.stamp = this->now();
    radarData.header.frame_id = "Radar data";
    radarData.height = 1;
    radarData.width = detections.size(); // Number of radar points
    radarData.is_dense = true;
    radarData.is_bigendian = false;
    radarData.point_step = 10 * sizeof(float); // azimuth, elevation, depth, x, y, z, reflectivity, x, y, z, radial velocity, existence probability, SNR, RCS
    radarData.row_step = radarData.point_step * radarData.width;
    radarData.data.resize(radarData.row_step * radarData.height);

    // Define fields
    sensor_msgs::PointCloud2Modifier modifier(radarData);
    modifier.setPointCloud2Fields(
      10,
      "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32,
      "elevation", 1, sensor_msgs::msg::PointField::FLOAT32,
      "depth", 1, sensor_msgs::msg::PointField::FLOAT32,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "radial_velocity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "radar_cross_section", 1, sensor_msgs::msg::PointField::FLOAT32,
      "signal_to_noise_ratio", 1, sensor_msgs::msg::PointField::FLOAT32,
      "existence_probability", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    // Fill ROS2::PointCloud2 message
    sensor_msgs::PointCloud2Iterator<float> iter_azi(radarData, "azimuth");
    sensor_msgs::PointCloud2Iterator<float> iter_ele(radarData, "elevation");
    sensor_msgs::PointCloud2Iterator<float> iter_dep(radarData, "depth");
    sensor_msgs::PointCloud2Iterator<float> iter_x(radarData, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(radarData, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(radarData, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_radVel(radarData, "radial_velocity");
    sensor_msgs::PointCloud2Iterator<float> iter_RCS(radarData, "radar_cross_section");
    sensor_msgs::PointCloud2Iterator<float> iter_SNR(radarData, "signal_to_noise_ratio");
    sensor_msgs::PointCloud2Iterator<float> iter_exProb(radarData, "existence_probability");

    // Convert to Cartesian coordinates
    for (size_t i = 0; i < radarData.width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_azi, ++iter_ele, ++iter_dep, ++iter_radVel, ++iter_RCS, ++iter_SNR, ++iter_exProb)
    {
        const Detection& detection = detections[i];
        *iter_azi = static_cast<float>(detection.AzimuthAngle);
        *iter_ele = static_cast<float>(detection.ElevationAngle);
        *iter_dep = static_cast<float>(detection.RadialDistance);
        *iter_x = static_cast<float>(detection.RadialDistance * cos(detection.ElevationAngle) * cos(detection.AzimuthAngle));
        *iter_y = static_cast<float>(detection.RadialDistance * cos(detection.ElevationAngle) * sin(detection.AzimuthAngle));
        *iter_z = static_cast<float>(detection.RadialDistance * sin(detection.ElevationAngle));
        *iter_radVel = static_cast<float>(detection.RadialVelocity);
        *iter_RCS = static_cast<float>(detection.RadarCrossSection);
        *iter_SNR = static_cast<float>(detection.SNR);
        *iter_exProb = static_cast<float>(detection.ExistanceProbability);
    }

    // Publish ROS2::PointCloud2 message
    this->radarDataPublisher_->publish(radarData);

  }

  void AurelionRos2BridgeNode::publishCameraData(std::unique_ptr<CameraDataDeserializerDefault>& deserializer, uint8_t sensorId, int cameraImageType)
  {
    
    // Set camera image type
    CameraImageType requestedCameraImageType = static_cast<CameraImageType>(cameraImageType);

    // Create instance of sensor storage class and request data
    AtomicSensorInformation sensorData;
    sensorApi.requestSensorData(sensorId, &sensorData);

    // Write data into image
    const CameraImage* requestedImage = nullptr;
    sensorData.getSensorData(&requestedImage, deserializer.get(), requestedCameraImageType);

    // Print some data points
    if(this->verbosePrinting)
    {
      std::cout << "\tRequested camera sensor with ID " << unsigned(sensorId) << " at maneuver time: " << sensorData.getSensorHeader().m_uiTimeStamp << "\xE6s" << std::endl;
      std::cout << "\t\twidth=" << requestedImage->Width << " height=" << requestedImage->Height << " row_pitch=" << requestedImage->RowPitch << " stride=" << requestedImage->Stride << std::endl;
    }

    // Fill ROS2::Image message
    auto cameraData = sensor_msgs::msg::Image();
    cameraData.header.stamp = this->now();
    cameraData.header.frame_id = "Camera data";
    cameraData.height = requestedImage->Height;
    cameraData.width = requestedImage->Width;  
    cameraData.encoding = "rgb8"; 
    cameraData.step = cameraData.width * 3; // RGB
    cameraData.data.resize(cameraData.height * cameraData.step);
    cameraData.data.assign(requestedImage->Data, requestedImage->Data + (cameraData.height * cameraData.step));

    // Publish ROS2::Image message
    this->cameraDataPublisher_->publish(cameraData);

  }

} // namespace aurelion_ros2_bridge


int main(int argc, char *argv[])
{
  try
  {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr AurelionRos2BridgeNodePtr = std::make_shared<aurelion_ros2_bridge::AurelionRos2BridgeNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(AurelionRos2BridgeNodePtr);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception& e)
  {
    std::cerr << "Failed to initialize Aurelion-ROS2-Bridge node: " << e.what() << '\n';
  }
  
}
