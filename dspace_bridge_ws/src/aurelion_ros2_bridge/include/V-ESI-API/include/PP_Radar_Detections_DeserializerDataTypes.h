#pragma once

#include "PP_Radar_Detections_Base_Detection.h"
#include "OptixSensorBaseHeader.h"
#include "DsHostDeviceMacro.h"

namespace dSPACE	//NOLINT
{
	namespace PPRadarDetectionsDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABCDEF20;

		//ver 4: Deserializer unification
		//ver 5: Instance IDs
		//ver 6: Simulation time + version handling
		static constexpr DeserializerBase::DeserializerVersion Version =
		{
			6,	// Major
			0	// Minor
		};

		struct Detection : public PPRadarDetectionsBase::Detection
		{
			//just use default fields.
		};

#pragma pack(push, 1)
		struct RadarDetectionsHeader : public DeserializerBase::OptixSensorBaseHeader
		{
			uint16_t NumDetections{};
		};
#pragma pack(pop)
	}
}