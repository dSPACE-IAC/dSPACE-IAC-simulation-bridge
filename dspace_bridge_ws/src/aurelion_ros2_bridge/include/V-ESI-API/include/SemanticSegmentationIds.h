#pragma once

#include <cstdint>
#include "DsHostDeviceMacro.h"

using instanceId_t = uint32_t;
//unfortunately we need to use a 32bit value here (instead of 16, which would be enough to encode > 65k classes), 
//because the concept of AndreS encodes colors as class ids (and not numbers starting from 1)
//if the concept changes, we may be able to switch to a uint16_t to safe triangle gpu memory.
using classId_t = uint32_t;

enum SpecialSemanticSegmentationIds : uint32_t
{
	UNKNOWN = 0,	// objects with no id
	FOG = 1,		// detections with this segmentationID are not due to an actual object, but due to fog (currently only occurs in lidar)
};

struct SemanticSegmentationIds
{
	instanceId_t InstanceID = SpecialSemanticSegmentationIds::UNKNOWN;
	classId_t ClassID = SpecialSemanticSegmentationIds::UNKNOWN;

	DS_HOSTDEVICE bool operator==(const SemanticSegmentationIds& other) const
	{
		return 
			InstanceID == other.InstanceID && 
			ClassID == other.ClassID;
	}
};


