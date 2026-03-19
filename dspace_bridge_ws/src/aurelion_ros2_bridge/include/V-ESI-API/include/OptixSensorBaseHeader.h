#pragma once

#include <cstdint>
#include "SensorBaseHeader.h"

namespace dSPACE
{
	namespace DeserializerBase
	{
#pragma pack(push, 1)
		struct OptixSensorBaseHeader : SensorBaseHeader{
			uint32_t DataLength;
		};
#pragma pack(pop)
	}
}
