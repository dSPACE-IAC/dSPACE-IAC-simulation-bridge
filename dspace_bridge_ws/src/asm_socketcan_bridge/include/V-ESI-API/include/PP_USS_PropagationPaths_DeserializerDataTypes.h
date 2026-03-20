#pragma once

#include <cstdint>
#include <vector>
#include <ostream>
#include <iomanip>

#include "OptixMaterial.h"
#include "SemanticSegmentationIds.h"

namespace dSPACE	// NOLINT
{
	namespace PPUltrasonicPropagationPathsDeserializer
	{
		static const uint32_t FormatIdentifier = 0xABBBBB11;
		//version history:
		//1: initial version
		static const uint8_t Version = 1;

#pragma pack(push, 1) //pack structs to enable fast std::memcpy from img data without padding risk
		struct UltrasonicPropagationPathHeader
		{
			uint32_t FormatIdentifier;	// Unique deserializer identifier
			uint8_t Version;			// Version to match serialized data to correct deserializer.
			uint32_t DataLength;
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

		// A single interaction point of a ultrasonic path.
		struct CompositeUltrasonicFrameReceiverDataPathHop
		{
			Vec3f Position; //XYZ-Position of this hop in the coordinate system of the receiving RX - Sensor of the ray path, in meters. (Note: Unlike raytracer output, these are cartesian coordinates and not spherical coordinates)
			materialId_t MaterialId; //The material ID of the hit surface for this hop
			SemanticSegmentationIds GroundTruthData; //Semantic Segmentation IDs of this hop.

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrameReceiverDataPathHop& d)
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
		struct CompositeUltrasonicFrameReceiverDataPath
		{
			std::vector<CompositeUltrasonicFrameReceiverDataPathHop> Hops; //A list of all reflection points (hops) of this propagation path. Does not include TX- and RX-Sensor. 
			float SoundIntensityWattPerSquaremeter;	// the sound intensity in Watt per Squaremeter
			float Length; //see raytracer channel impulse response output.
			uint8_t SourceEmitterId; //see raytracer channel impulse response output.

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrameReceiverDataPath& d)
			{
				o << "\t\tSound Intensity in Watt per Squaremeter: " << d.SoundIntensityWattPerSquaremeter << '\n';
				o << "\t\tPath Length: " << d.Length << '\n';
				o << "\t\tHop Count: " << d.Hops.size() << '\n';
				for (size_t i = 0; i < d.Hops.size(); ++i)
				{
					o << "\t\tHop Data " << i << '\n';
					o << d.Hops[i] << '\n';
				}
				return o;
			}
		};

		// Stores all ultrasonic data received by a specific RX-Sensor.
		struct CompositeUltrasonicFrameReceiverData
		{
			std::vector<CompositeUltrasonicFrameReceiverDataPath> Paths;

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrameReceiverData& d)
			{
				o << "\tPath Count: " << d.Paths.size() << '\n';
				for (size_t i = 0; i < d.Paths.size(); ++i)
				{
					o << "\tPath Data " << i << '\n';
					o << d.Paths[i] << '\n';
				}
				return o;
			}
		};

		// Stores an entire ultrasonic data frame.
		struct CompositeUltrasonicFrame
		{
			std::vector<CompositeUltrasonicFrameReceiverData> ReceiverData;

			friend std::ostream& operator<<(std::ostream& o, const CompositeUltrasonicFrame& d)
			{
				o << "Receiver Count: " << d.ReceiverData.size() << '\n';

				for (size_t i = 0; i < d.ReceiverData.size(); ++i)
				{
					o << "Receiver Data " << i << '\n';
					o << d.ReceiverData[i] << '\n';
				}
				return o;
			}
		};
	};
}