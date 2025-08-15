#pragma once

#ifdef _WIN32
    #if defined(VESILIBRARY_EXPORT)
        #define VESILIBRARYEXPORT   __declspec(dllexport)
    #else
        #define VESILIBRARYEXPORT   __declspec(dllimport)
    #endif
#else
    #if defined(VESILIBRARY_EXPORT)
        #define VESILIBRARYEXPORT __attribute__((visibility("default")))
    #else
        #define VESILIBRARYEXPORT
    #endif
#endif

#include <stdint.h>
#include <string>
#include <cstring>
#include <vector>
#include "DsTCompositeHeader.h"

#include "PP_Radar_PropagationPaths_DeserializerDataTypes.h"
#include "PP_Radar_Detections_DeserializerDataTypes.h"
#include "PP_Radar_RawDataGen_DeserializerDataTypes.h"
#include "PP_Lidar_PointCloud_DeserializerDataTypes.h"
#include "PP_USS_PropagationPaths_DeserializerDataTypes.h"
#include "CameraDataDeserializerBase.h"
#include "CameraDataDeserializerTypes.h"

using namespace dSPACE::PPLidarPointCloudDeserializer;
using namespace dSPACE::PPRadarPropagationPathsDeserializer;
using namespace dSPACE::PPRadarDetectionsDeserializer;
using namespace dSPACE::PPRadarRawDataGenDeserializer;
using namespace dSPACE::PPUltrasonicPropagationPathsDeserializer;
using namespace CameraDataDeserializers;



const uint16_t GENERAL_HEADER_SIZE = sizeof(DsTCompositeGeneralHeader);
const uint16_t SENSOR_HEADER_SIZE = sizeof(DsTCompositeSensorData);


/*! \class AtomicSensorInformation
\brief Storage class for sensor data

Stores the sensor data that has been requested by the V-ESI API as well as additional meta-information.
*/
class VESILIBRARYEXPORT AtomicSensorInformation
{
    public:
        AtomicSensorInformation();
        ~AtomicSensorInformation();
        //! Copy constructor to perform a deep copy of the stored data
        AtomicSensorInformation(const AtomicSensorInformation& other);
        //!Assignment operator to perform a correct assignment of the stored data
        AtomicSensorInformation& operator=(const AtomicSensorInformation& other);
        //! Function to get the general header of the sensor information.
        /*!
        \returns DsTCompositeGeneralHeader, the general sensor header that holds relevant meta information concerning all simulated sensors (e.g. timestamp)
        \sa DsTCompositeGeneralHeader
        */
        DsTCompositeGeneralHeader getGeneralHeader();
        //! Function to get the specific sensor header of the sensor information.
        /*!
        \returns DsTCompositeSensorData, the sensor specific header that holds relevant meta information of the specific sensor (e.g. sensor type)
        \sa DsTCompositeSensorData
        */
        DsTCompositeSensorData getSensorHeader();
        //! Function to get the size of the stored sensor data.
        /*!
        \returns Size of sensor data in bytes
        */
        uint64_t getSensorDataSize();
        //! Function to get the stored sensor data as byte array.
        /*!
        \param sensor_data Reference to a pointer that points to the stored sensor data
        */
        void getSensorData(uint8_t** sensor_data);
        //! Function to get the stored sensor data as a certain camera image. Throws if the requested sensor data is not of type: Camera
		/*!
		\param sensor_data Reference to a pointer that points to the requested camera sensor data
        \param camera_data_deserializer Reference to the camera deserializer that should be used
		\param image_type The type of the image that should be returned
		*/
        void getSensorData(const CameraImage** camera_image, CameraDataDeserializerBase* camera_data_deserializer, CameraImageType image_type);
        //! Function to get the stored sensor data. Throws if the requested sensor data is not of type: Camera
        /*!
        \param sensor_data Reference to a pointer that points to the requested camera sensor data
        \param camera_data_deserializer Reference to the camera deserializer that should be used
        \param image_type The type of the image that should be returned
        */
        void getSensorData(const CameraData** camera_data, CameraDataDeserializerBase* camera_data_deserializer, CameraDataType data_type);
        //! Function to get the stored sensor data as radar paths. Throws if the requested sensor data is not of type: Radar
        /*!
        \param sensor_data Reference to a struct that stores requested radar data
        \param rxAntennaCount Number of RX-antennas of the configured sensor
        \sa DefaultPostProcessor_Deserializer::DsCCompositumRadarFrame
        */
        void getSensorData(CompositeRadarFrame* sensor_data);
        //! Function to get the stored sensor data as radar detection list. The corresponding radar-postprocessing has to be selected within MotionDesk. Throws if the requested sensor data is not of type: Radar
        /*!
        \param sensor_data Vector that stores requested radar detecions
        \sa PP_RadarDetectionsList_Deserializer::Detection
		*/
        void getSensorData(std::vector<Detection>* sensor_data);
		//! Function to get the stored sensor data as raw radar adc samples. The corresponding radar-postprocessing has to be selected within MotionDesk. Throws if the requested sensor data is not of type: Radar
		/*!
		\param sensor_data Vector that stores requested radar raw adcsamples
		\sa PP_RadarDetectionsList_Deserializer::Detection
		*/
		void getSensorData(std::vector<AdcSample>* sensor_data);

        //! Function to get the stored sensor data lidar as lidar point cloud. Throws if the requested sensor data is not of type: Lidar
        /*!
        \param sensor_data Reference to a struct that stores requested lidar data
        \param virtualSensorIndex Index of the virtual lidar sensor that is supposed to be requested. (If there is only one virtual lidar sensor, virtualSensorIndex=0 is to be used)
        \sa Lidar_DefaultPostProcessor_Deserializer::DsCCompositeLidarFrame
        */
        void getSensorData(CompositeLidarFrame* sensor_data);
        //! Function to get the stored sensor data as ultrasonic structure. Throws if the requested sensor data is not of type: Ultrasonic
        /*!
        \param sensor_data Reference to a struct that stores requested ultrasonic data
        */
        void getSensorData(CompositeUltrasonicFrame* sensor_data);



        DsTCompositeGeneralHeader* general_header;
        DsTCompositeSensorData* sensor_header;
        uint64_t sensor_data_size;
        uint8_t* sensor_data;

};