#pragma once

#include <cstdint>
#include <vector>
#include <ostream>
#include <iomanip>

#include "OptixMaterial.h"
#include "SemanticSegmentationIds.h"
#include "SensorBaseHeader.h"

namespace dSPACE	// NOLINT
{
	namespace PPRadarPropagationPathsDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABCDEF22;

		//ver 8: Deserializer unification
		static constexpr uint8_t Version = 8;

#pragma pack(push, 1) //pack structs to enable fast std::memcpy from img data without padding risk
		struct RadarPropagationPathHeader
		{
			uint32_t FormatIdentifier;	// Unique deserializer identifier
			uint8_t Version;			// Version to match serialized data to correct deserializer.
			uint32_t DataLength;
			uint32_t NumRxas;

			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSize()
			{
				return sizeof(RadarPropagationPathHeader);
			}
		};

		// Vector of three floats.
		struct Vec3f
		{
			float X, Y, Z;

			friend std::ostream& operator<<(std::ostream& Os, const Vec3f& Vec)
			{
				Os << '[' << Vec.X << ',' << Vec.Y << ',' << Vec.Z << ']';
				return Os;
			}
		};

		// A single interaction point of a radar path.
		struct CompositeRadarFrameRxaDataPathHop
		{
			Vec3f Position; //XYZ-Position of this hop in the coordinate system of the receiving RX - Antenna of the ray path, in meters. (Note: Unlike raytracer output, these are cartesian coordinates and not spherical coordinates)
			Vec3f Velocity; //XYZ-Velocity of this hop relative to the receiving antenna in meters/second. Please note that the values are calculated for the antenna coordinate system.
			materialId_t MaterialId; //The material ID of the hit surface for this hop
			SemanticSegmentationIds GroundTruthData; //Semantic Segmentation IDs of this hop.

			friend std::ostream& operator<<(std::ostream& Os, const CompositeRadarFrameRxaDataPathHop& Hop)
			{
				Os << "\t\t\tPosition: " << Hop.Position << '\n';
				Os << "\t\t\tVelocity: " << Hop.Velocity << '\n';
				Os << "\t\t\tMaterial ID: " << Hop.MaterialId << '\n';
				Os << "\t\t\tClass ID: " << Hop.GroundTruthData.ClassID << '\n';
				Os << "\t\t\tInstance ID: " << Hop.GroundTruthData.InstanceID << '\n';

				return Os;
			}
		};
#pragma pack(pop)

		struct ComplexFloat
		{
			float Real, Imag;

			friend std::ostream& operator<<(std::ostream& Os, const ComplexFloat& Value)
			{
				Os << Value.Real << "+" << Value.Imag << "i";
				return Os;
			}
		};

		struct ComplexEField
		{
			ComplexFloat X, Y, Z;

			friend std::ostream& operator<<(std::ostream& Os, const ComplexEField& Efield)
			{
				Os << "(";
				Os << Efield.X;
				Os << " | ";
				Os << Efield.Y;
				Os << " | ";
				Os << Efield.Z;
				Os << ")";

				return Os;
			}
		};

		struct EFieldsFromExcitation
		{
			ComplexEField EFieldFromHExcitation, EFieldFromVExcitation;

			friend std::ostream& operator<<(std::ostream& Os, const EFieldsFromExcitation& Efields)
			{
				Os << "[";
				Os << Efields.EFieldFromHExcitation;
				Os << " - ";
				Os << Efields.EFieldFromVExcitation;
				Os << "]";
				return Os;
			}

		};

		// Stores a single radar path, including all interaction points.
		struct CompositeRadarFrameRxaDataPath
		{
			std::vector<CompositeRadarFrameRxaDataPathHop> Hops; //A list of all reflection points (hops) of this propagation path. Does not include TX- and RX-Antenna. 
			EFieldsFromExcitation EFields;
			float Length; //see raytracer channel impulse response output.
			float DopplerSpeed; //see raytracer channel impulse response output.
			uint8_t SourceTxAntennaId; //see raytracer channel impulse response output.

			friend std::ostream& operator<<(std::ostream& Os, const CompositeRadarFrameRxaDataPath& Path)
			{
				Os << "\t\teFields: " << Path.EFields << '\n';
				Os << "\t\tPath Length: " << Path.Length << '\n';
				Os << "\t\tTotal Doppler Speed: " << Path.DopplerSpeed << '\n';
				Os << "\t\tHop Count: " << Path.Hops.size() << '\n';
				for (size_t i = 0; i < Path.Hops.size(); ++i)
				{
					Os << "\t\tHop Data " << i << '\n';
					Os << Path.Hops[i] << '\n';
				}
				return Os;
			}
		};

		// Stores all radar data received by a specific RX-Antenna.
		struct CompositeRadarFrameRxaData
		{
			std::vector<CompositeRadarFrameRxaDataPath> Paths;

			friend std::ostream& operator<<(std::ostream& Os, const CompositeRadarFrameRxaData& Data)
			{
				Os << "\tPath Count: " << Data.Paths.size() << '\n';
				for (size_t i = 0; i < Data.Paths.size(); ++i)
				{
					Os << "\tPath Data " << i << '\n';
					Os << Data.Paths[i] << '\n';
				}
				return Os;
			}
		};

		// Stores an entire radar data frame.
		struct CompositeRadarFrame
		{
			std::vector<CompositeRadarFrameRxaData> RxaData;

			friend std::ostream& operator<<(std::ostream& Os, const CompositeRadarFrame& Frame)
			{
				Os << "RXA Count: " << Frame.RxaData.size() << '\n';

				for (size_t i = 0; i < Frame.RxaData.size(); ++i)
				{
					Os << "RXA Data " << i << '\n';
					Os << Frame.RxaData[i] << '\n';
				}
				return Os;
			}
		};
	}
}
