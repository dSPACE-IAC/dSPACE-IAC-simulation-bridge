#pragma once

#include <cstdint>
#include <ostream>
#include <cmath>

#include "DsHostDeviceMacro.h"
#include "SemanticSegmentationIds.h"


namespace PPRadarDetectionsBase
{
	struct ComplexFloat
	{
		float Real, Imag;
	};

	struct ComplexFloat3
	{
		ComplexFloat X,Y,Z;

		DS_HOSTDEVICE inline ComplexFloat3 operator*(const float floatVar) const
		{
			return { {X.Real * floatVar, X.Imag * floatVar},
					 {Y.Real * floatVar, Y.Imag * floatVar},
					 {Z.Real * floatVar, Z.Imag * floatVar} };
		}
		
		DS_HOSTDEVICE inline float amplitude() const
		{
			return sqrt(
				X.Real * X.Real + X.Imag * X.Imag + 
				Y.Real * Y.Real + Y.Imag * Y.Imag + 
				Z.Real * Z.Real + Z.Imag * Z.Imag) ;
		}
	};

	struct Detection
	{
		float RadialDistance = 0.f; //[m] Distance to the measured detection (one way) relative to sensor. Affected by accuracy and resolution in config.
		float RadialDistanceStdDev = 0.f; //[m] Standard deviation of distance measurement. Influenced by accuracy.distance in the config

		float AzimuthAngle = 0.f; //[rad] Azimuth angle of detection relative to sensor. Positive Y coordinate will produce positive angle value. Affected by accuracy and resolution in config.
		float AzimuthAngleStdDev = 0.f; //[rad] Standard deviation of azimuth measurement. Influenced by accuracy.azimuth in the config

		float ElevationAngle = 0.f; //[rad] Elevation angle of detection relative to sensor. Positive Z coordinate will produce positive angle value. Affected by accuracy and resolution in config.
		float ElevationAngleStdDev = 0.f; //[rad] Standard deviation of elevation measurement. Influenced by accuracy.elevation in the config

		float ExistanceProbability = 0.f; //[0..1] Existence probability of the detection. Is lower for detections based on multipath propagation.

		float RadialVelocity = 0.f; //[m/s] Radial velocity to the measured detection relative to sensor. Affected by accuracy and resolution in config.
		float RadialVelocityStdDev = 0.f; //[m/s] Standard deviation of velocity measurement. Influenced by accuracy.velocity in the config

		float SNR = 0.f; //Signal to Noise Ratio in dB. Influenced by detection signal strength, as well as sensor sending power, wavelength, temperature and receiver bandwith.
		float RadarCrossSection = 0.f; //[m^2] Calculated RCS of the detection. Calculation is based on the received e field together with sensor polarization. For the calculation, the path loss discarded (So unline SNR, RCS is independent of distance to the detection). Also, influences of antenna gain are compensated.
		ComplexFloat3 RawEField{}; //Complex raw E-Field of the detection. Influenced by sending power, distance and reflection characteristics of target.

		classId_t ClassID; //The class id of the detection. If the detection contains multiple rays, the id from the strongest ray is selected.
		instanceId_t InstanceID; //The instance id of the detection. If the detection contains multiple rays, the id from the strongest ray is selected.

		friend std::ostream &operator<<(std::ostream &o, const Detection &)
		{
			o << "NYI";
			return o;
		}
	};
}