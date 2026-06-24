#pragma once

#include <cstdint>
#include <vector>

#include "SemanticSegmentationIds.h"
#include "OptixSensorBaseHeader.h"

namespace dSPACE	//NOLINT
{
	namespace PPLidarPointCloudDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xBCDEF20;

		//ver 4: Deserializer unification
		//ver 5: Version handling
		//ver 5.1: Fix header parsing
		static constexpr DeserializerBase::DeserializerVersion Version =
		{
			5,	// Major
			1	// Minor
		};

#pragma pack(push, 1)
		struct PointCloudLidarPoint
		{
			float Normal[3];			// normal vector of surface hit, normalized
			float Azimuth;				// in degree, clockwise (!)
			float Elevation;			// in degree, pointing up
			float Distance;				// in m
			float RelativeVelocity;	// in m/s, projected in ray direction
			float Reflectivity;			// in [0, 1]
			float OpticalPower;		// rescaled to [0,1]. Multiplication with transmit power yields receive power.
			float TimeOffset;			// in ms, offset from simulation time

			SemanticSegmentationIds GroundTruthData; //Semantic Segmentation IDs of this point.
			uint32_t RayID;
			uint16_t MaterialID;
		};
#pragma pack(pop)

#pragma pack(push, 1)
		struct PointCloudOutputModes
		{
			bool Normal;
			bool Azimuth;
			bool Elevation;
			bool Distance;
			bool RelativeVelocity;
			bool Reflectivity;
			bool OpticalPower;
			bool TimeOffset;

			bool MaterialID;
			bool ClassID;
			bool InstanceID;
			bool RayID;

			size_t GetLidarPointSize() const
			{
				size_t size = 0;

				size += this->Normal ? sizeof(PointCloudLidarPoint::Normal) : 0;
				size += this->Azimuth ? sizeof(PointCloudLidarPoint::Azimuth) : 0;
				size += this->Elevation ? sizeof(PointCloudLidarPoint::Elevation) : 0;
				size += this->Distance ? sizeof(PointCloudLidarPoint::Distance) : 0;
				size += this->TimeOffset ? sizeof(PointCloudLidarPoint::TimeOffset) : 0;
				size += this->RelativeVelocity ? sizeof(PointCloudLidarPoint::RelativeVelocity) : 0;
				size += this->Reflectivity ? sizeof(PointCloudLidarPoint::Reflectivity) : 0;
				size += this->OpticalPower ? sizeof(PointCloudLidarPoint::OpticalPower) : 0;
				size += this->MaterialID ? sizeof(PointCloudLidarPoint::MaterialID) : 0;
				size += this->ClassID ? sizeof(PointCloudLidarPoint::GroundTruthData.ClassID) : 0;
				size += this->InstanceID ? sizeof(PointCloudLidarPoint::GroundTruthData.InstanceID) : 0;
				size += this->RayID ? sizeof(PointCloudLidarPoint::RayID) : 0;

				return size;
			}
		};
#pragma pack(pop)

#pragma pack(push, 1) 
		struct LidarPointCloudHeader : DeserializerBase::OptixSensorBaseHeader
		{
			uint32_t Checksum;								// CRC32 checksum if enabled, otherwise zero.
			PointCloudOutputModes Output;					// Defines what kind of information is valid to read per point.
			uint32_t NumPoints;								// Number of points within frame.
			uint32_t MaxNumOfPoints;						// Maximal possible number of points under current configuration (max number of rays and returns per ray). 
			
			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSize()
			{
				return sizeof(LidarPointCloudHeader);
			}
		};
#pragma pack(pop)

		struct CompositeLidarFrame
		{
			std::vector<PointCloudLidarPoint> LidarPoints;
		};
	}
}