#pragma once

#include <cstdint>

#pragma pack(push, sensorHeader)
#pragma pack(1)
/*! \struct DsTCompositeGeneralHeader
\brief This structure holds relevant meta information of the sensor simulation. This information is common to all simulated sensors
*/
struct DsTCompositeGeneralHeader
{
    uint8_t     m_byErr1[3];        // fixed: (0,0,0)
    uint8_t     m_byErr2[3];        // fixed: (127,127,127)
    uint8_t     m_byErr3[3];        // fixed: (255,255,255)
    uint16_t    m_Version;          // version of the header structure
    uint8_t     m_byNumSensors;     // number of sensors which did fit into the composite window
    uint64_t    m_uiTimeStamp;      // simulation time in µs
    uint64_t    m_byRollCounter;    // rolling counter i.e. for latency measurements
    uint16_t    m_Padding;          // padding
};
/*! \struct DsTCompositeSensorData
\brief This structure holds relevant meta information of a specific sensor.
*/
struct DsTCompositeSensorData
{
    uint8_t     m_byType;           // type of the sensor DsESensorType
    uint8_t     m_byPosition;       // stores the sensor ID
    uint8_t     m_byInputFormat;    // reserved
    uint64_t    m_byOutputFormat;   // output format of the sensor MDLCommonInterfaces::LaserSensorOutputMode, MDLCommonInterfaces::CameraSensorOutputMode
    uint64_t    m_uiTimeStamp;      // simulation time in µs, might differ from DsTCompositeGeneralHeader::m_uiTimeStamp
    uint16_t    m_uiFrameRate;      // reserved
    uint16_t    m_uiWidth;          // width of the sensor in px
    uint16_t    m_uiHeight;         // height of the sensor in px
    uint16_t    m_uiX;              // X position of the sensor inside the composite window
    uint16_t    m_uiY;              // Y position of the sensor inside the composite window (origin is at the bottom left)
    uint8_t     m_Padding;          // padding
};
/*! \struct DsTCompositeHeaderEnd
\brief This structure holds additional information of the sensor simulation.
*/
struct DsTCompositeHeaderEnd
{
    uint32_t    m_crc;              // Cyclic Redundancy Check (CRC) of CompositeGeneralHeader and all CompositeSensorData
    uint16_t    m_Padding;          // padding
};

#pragma pack(pop, sensorHeader)