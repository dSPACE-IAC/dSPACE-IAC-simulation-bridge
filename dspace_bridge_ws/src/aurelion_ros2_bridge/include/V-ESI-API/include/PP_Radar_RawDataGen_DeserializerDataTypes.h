#pragma once

#include <cstdint>
#include <iostream>
#include "OptixSensorBaseHeader.h"
#include "DsHostDeviceMacro.h"

namespace dSPACE	// NOLINT
{
	namespace PPRadarRawDataGenDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABCDEF21;

		//ver 3: Deserializer unification
		//ver 4: Version handling
		static constexpr DeserializerBase::DeserializerVersion Version =
		{
			4,	// Major
			0	// Minor
		};

		struct AdcSample
		{
			float Real = 0.0f;
			float Imag = 0.0f;

			//for unknown reason, clang needs ctors for initializing this type
			inline AdcSample() = default;
			inline AdcSample(float Real, float Imag) : Real(Real), Imag(Imag) {}

			friend std::ostream& operator<<(std::ostream& Os, const AdcSample& Sample)
			{
				Os << Sample.Real << "+" << Sample.Imag << "i";
				return Os;
			}
		};

#pragma pack(push, 1)
		struct RawDataConfig
		{
			uint32_t NumRxas = 0;
			uint32_t NumChirps = 0;
			uint32_t NumSamples = 0;
			bool ReadComplexSamples = true;
		};

		struct RadarRawDataGenHeader : public DeserializerBase::OptixSensorBaseHeader
		{
			RawDataConfig Conf;

			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSize()
			{
				return sizeof(DeserializerBase::OptixSensorBaseHeader) + sizeof(RawDataConfig);
			}
		};
#pragma pack(pop)
	}
}