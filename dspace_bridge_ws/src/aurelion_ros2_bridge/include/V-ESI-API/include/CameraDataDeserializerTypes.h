// Copyright 2021 dSPACE GmbH. All rights reserved.

#pragma once

#include <cinttypes>

#ifdef CAMERA_DESERIALIZER_EXPORT
	#ifdef _WIN32
		#define SENSOR_DESERIALIZER_API __declspec(dllexport)
	#else
		#define SENSOR_DESERIALIZER_API __attribute__((visibility("default")))
	#endif
#else
	#ifdef _WIN32
		#define SENSOR_DESERIALIZER_API __declspec(dllimport)
	#else
		#define SENSOR_DESERIALIZER_API
	#endif
#endif

namespace CameraDataDeserializers
{
	enum class CameraImageType : uint8_t
	{
		CIT_COLOR,		// use this to get the sRGB, raw12 or raw24 images
		CIT_CLASSID,	// use this to get the class ID image
		CIT_INSTANCEID,	// use this to get the instance ID image
		CIT_VELOCITY,	// use this to get the velocity image
		CIT_DEPTH		// use this to get the depth image
	};

	enum class CameraDataType : uint8_t
	{
		CDT_OBJECT_BOUNDS // use this to get the object bounds
	};

	/**
	 * Represents the serialized camera data.
	 */
	struct CameraBuffer
	{
		const uint8_t* Data;	// buffer containing the camera data
		uint32_t Size;			// size of the data buffer
		uint32_t RowPitch;		// size (in bytes) of a single row of the camera image
		uint64_t Flags;			// combination of flags describing the contents of the data
	};

	/**
	 * Represents a camera image (the result of the deserialization).
	 */
	struct CameraImage
	{
		const uint8_t* Data;	// buffer containing the image data
		uint32_t Width;			// width of the image in pixels
		uint32_t Height;		// height of the image in pixels
		uint32_t Stride;		// size (in bytes) of a pixel
		uint32_t RowPitch;		// size (in bytes) of a image row
	};

	/**
	 * Represents a buffer containing camera data (the result of the deserialization).
	 */
	struct CameraData
	{
		const uint8_t* Data;	// buffer containing the image data
		uint32_t Size;			// buffer size (in number of elements of size Stride)
		uint32_t Stride;		// size of a single element in the data array
	};

	/**
	 * Contains properties of the camera frame.
	 */
	struct CameraFrameContext
	{
		float Exposure;				// the exposure factor of the current frame.
		float InvDepthZToWorldZ[2];	// components for converting the inverse depth (NDC) to world space depth. Use as 1 / (invDepthZ * InvDepthZToWorldZ[0] - InvDepthZToWorldZ[1]).
	};

	/**
	 * Represents the bounding information of a single object.
	 */
	struct ObjectBoundsDesc
	{
		uint32_t ClassID;			// the class ID of the object
		uint32_t InstanceID;		// the instance ID of the object
		uint32_t NumVisiblePixels;	// number of visible (unoccluded) pixels
		uint32_t MinBounds[2];		// the minimum bounds of the object
		uint32_t MaxBounds[2];		// the maximum bound of the object
	};
}