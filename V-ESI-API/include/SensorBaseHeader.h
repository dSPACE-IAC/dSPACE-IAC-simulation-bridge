#pragma once

#include <cstdint>

namespace dSPACE	// NOLINT
{
	namespace DeserializerBase
	{

#pragma pack(push, 1) 
		struct DeserializerVersion
		{
			uint16_t Major;
			uint16_t Minor;
		};

		struct SensorBaseHeader
		{
			uint32_t FormatIdentifier;	// Unique deserializer identifier

			DeserializerVersion Version;			// Version to match serialized data to correct deserializer.

			double SimulationTime;		// Simulation time in s
		};
#pragma pack(pop)
	}
}