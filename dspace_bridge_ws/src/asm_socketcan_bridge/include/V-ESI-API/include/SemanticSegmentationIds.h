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
	UNKNOWN = 0,		// objects with no id
};

struct SemanticSegmentationIds
{
	instanceId_t InstanceID;
	classId_t ClassID;

	//we cant use a ctor or variable inline assignment due to optix, so we have to use an "init" function to set defaults...
	DS_HOSTDEVICE void init()
	{
		InstanceID = SpecialSemanticSegmentationIds::UNKNOWN;
		ClassID = SpecialSemanticSegmentationIds::UNKNOWN;
	}

	DS_HOSTDEVICE bool operator==(const SemanticSegmentationIds& other) const
	{
		return 
			InstanceID == other.InstanceID && 
			ClassID == other.ClassID;
	}
};


