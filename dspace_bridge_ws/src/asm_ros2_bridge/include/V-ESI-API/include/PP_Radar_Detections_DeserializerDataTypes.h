#pragma once

#include "PP_Radar_Detections_Base_Detection.h"
#include "SensorBaseHeader.h"
#include "DsHostDeviceMacro.h"

namespace dSPACE	//NOLINT
{
	namespace PPRadarDetectionsDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABCDEF20;

		//ver 4: Deserializer unification
		//ver 5: Instance IDs
		static constexpr uint8_t Version = 5;

		struct Detection : public PPRadarDetectionsBase::Detection
		{
			//just use default fields.
		};


		struct RadarDetectionsHeader : public DeserializerBase::SensorBaseHeader
		{
			uint16_t NumDetections{};
			uint64_t ManeuverTimeUs{}; //Only used for udp

			// TODO: Unify
			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSize()
			{
				return sizeof(DeserializerBase::SensorBaseHeader) + sizeof(uint16_t) + sizeof(uint64_t);
			}

			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSizeUdp()
			{
				return sizeof(DeserializerBase::SensorBaseHeader) + sizeof(uint16_t) + sizeof(uint64_t) - sizeof(uint32_t);
			}

			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSizeTexture()
			{
				return sizeof(DeserializerBase::SensorBaseHeader) + sizeof(uint16_t);
			}
		};
		//directly after the header, the UDP packet contains numDetections x PP_RadarDetectionsList_Base::Detection struct, 
		//as defined in "PP_RadarDetectionsList_Base_Detection.h"
		//there is no additional padding between header and Detections, as well as between Detections.
		//the packet size is dynamic and dependent on numDetections, but can not exceed 65507 bytes (maximum of one UDP packet)

	}
}