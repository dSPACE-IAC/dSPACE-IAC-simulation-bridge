#pragma once

#include <cstdint>

namespace dSPACE	// NOLINT
{
	namespace DeserializerBase
	{
#pragma pack(push, 1) 
		struct SensorBaseHeader
		{
			uint32_t FormatIdentifier;	// Unique deserializer identifier
			uint8_t Version;			// Version to match serialized data to correct deserializer.
			uint32_t DataLength;		// Accumulated data length of sensor header + data + potential offsets (used for checking data completeness)
		};
#pragma pack(pop)
	}
}