#pragma once

#include <vector>
#include <cstdint>
#include <iostream>
#include "SensorBaseHeader.h"
#include "DsHostDeviceMacro.h"

namespace dSPACE	// NOLINT
{
	namespace PPRadarRawDataGenDeserializer
	{
		static const uint32_t FormatIdentifier = 0xABCDEF21;

		//ver 3: Deserializer unification
		static const uint8_t Version = 3;

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
#pragma pack(pop)

		struct RadarRawDataGenHeader : public DeserializerBase::SensorBaseHeader
		{
			RawDataConfig Conf;

			DS_HOSTDEVICE static inline constexpr size_t GetHeaderSize()
			{
				return sizeof(DeserializerBase::SensorBaseHeader) + sizeof(RawDataConfig);
			}
		};
	}
}