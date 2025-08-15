#pragma once

#include <cstdint>
#include <vector>
#include <ostream>
#include <iomanip>

#include "OptixMaterial.h"
#include "SemanticSegmentationIds.h"
#include "OptixSensorBaseHeader.h"

namespace dSPACE	// NOLINT
{
	namespace PPUltrasonicPropagationPathsDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABBBBB11;

		//ver 1: initial version
		static constexpr DeserializerBase::DeserializerVersion Version =
		{
			1,	// Major
			0	// Minor
		};

#pragma pack(push, 1) //pack structs to enable fast std::memcpy from img data without padding risk
		struct UltrasonicPropagationPathHeader : DeserializerBase::OptixSensorBaseHeader
		{
			uint32_t NumReceivers;

			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSize()
			{
				return sizeof(UltrasonicPropagationPathHeader);
			}
		};

		// Vector of three floats.
		struct Vec3f
		{
			float X, Y, Z;

			friend std::ostream& operator<<(std::ostream& o, const Vec3f& d)
			{
				o << '[' << d.X << ',' << d.Y << ',' << d.Z << ']';
				return o;
			}
		};

		// Stores a single interaction point of an ultrasonic path.
		struct CompositeUltrasonicFrameReceivedDataPathHop
		{
			Vec3f Position; //XYZ-Position of this hop in the coordinate system of the receiving RX - Sensor of the ray path, in meters. (Note: Unlike raytracer output, these are cartesian coordinates and not spherical coordinates)
			materialId_t MaterialId; //The material ID of the hit surface for this hop
			SemanticSegmentationIds GroundTruthData; //Semantic Segmentation IDs of this hop.

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrameReceivedDataPathHop& d)
			{
				o << "\t\t\tPosition: " << d.Position << '\n';
				o << "\t\t\tMaterial ID: " << d.MaterialId << '\n';
				o << "\t\t\tClass ID: " << d.GroundTruthData.ClassID << '\n';
				o << "\t\t\tInstance ID: " << d.GroundTruthData.InstanceID << '\n';

				return o;
			}
		};
#pragma pack(pop)

		// Stores a single ultrasonic path, including all interaction points.
		struct CompositeUltrasonicFrameReceivedDataPath
		{
			std::vector<CompositeUltrasonicFrameReceivedDataPathHop> Hops;	// a list of all reflection points (hops) of this propagation path. Does not include emitter and receiver.
			float SoundIntensityWattPerSquaremeter;							// the sound intensity in Watt per Squaremeter
			float PathLengthMeter;											// the total path length in meter
			uint8_t EmitterId;												// the id of the emitter of the path
			uint8_t ReceiverId;												// the id of the receiver of the path

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrameReceivedDataPath& d)
			{
				o << "\t\tSource Emitter Id: " << unsigned(d.EmitterId) << '\n';
				o << "\t\tReceiving Receiver Id: " << unsigned(d.ReceiverId) << '\n';
				o << "\t\tSound Intensity: " << d.SoundIntensityWattPerSquaremeter << "W/m2\n";
				o << "\t\tPath Length: " << d.PathLengthMeter << "m\n";
				o << "\t\tHop Count: " << d.Hops.size() << '\n';
				for (size_t i = 0; i < d.Hops.size(); ++i)
				{
					o << "\t\tData Of Hop " << i << '\n';
					o << d.Hops[i] << '\n';
				}
				return o;
			}
		};

		// Stores all ultrasonic data received by a specific receiver.
		struct CompositeUltrasonicFrameReceivedData
		{
			std::vector<CompositeUltrasonicFrameReceivedDataPath> Paths;

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrameReceivedData& d)
			{
				o << "\tPath Count: " << d.Paths.size() << '\n';
				for (size_t i = 0; i < d.Paths.size(); ++i)
				{
					o << "\tData Of Path " << i << '\n';
					o << d.Paths[i] << '\n';
				}
				return o;
			}
		};

		// Stores an entire ultrasonic data frame.
		struct CompositeUltrasonicFrame
		{
			std::vector<CompositeUltrasonicFrameReceivedData> ReceivedData;

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrame& d)
			{
				o << "Receiver Count: " << d.ReceivedData.size() << '\n';

				for (size_t i = 0; i < d.ReceivedData.size(); ++i)
				{
					o << "Data Of Receiver " << i << '\n';
					o << d.ReceivedData[i] << '\n';
				}
				return o;
			}
		};
	};
}