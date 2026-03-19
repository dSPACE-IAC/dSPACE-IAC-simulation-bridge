// Copyright 2021 dSPACE GmbH. All rights reserved.

#pragma once

#include "CameraDataDeserializerTypes.h"

namespace CameraDataDeserializers
{
	class SENSOR_DESERIALIZER_API CameraDataDeserializerBase
	{
	public:
		/**
		 * The ctor.
		 */
		CameraDataDeserializerBase();

		/**
		 * The dtor.
		 */
		virtual ~CameraDataDeserializerBase() = default;

		/**
		 * Should be called before any other method.
		 * @returns false on error and true on success.
		 */
		virtual bool Initialize() = 0;

		/**
		 * Should be called before class is destroyed.
		 * @returns false on error and true on success.
		 */
		virtual bool Destroy() = 0;

		/**
		 * Deserializes the camera data.
		 * Do not delete or modify the data while the memory area is being shared between the deserializer and other components.
		 * @param Data struct describing and containing the serialized camera data
		 * @returns false on error and true on success.
		 */
		virtual bool Deserialize(const CameraBuffer* Data) = 0;

		/**
		 * Returns a camera image.
		 * @param Type the type of the image.
		 * @returns the camera image
		 */
		virtual const CameraImage& GetImage(CameraImageType Type) const = 0;

		/**
		 * Returns camera data.
		 * @param Type of the data to return.
		 * @returns the camera data.
		 */
		virtual const CameraData& GetData(CameraDataType Type) const = 0;

	protected:
		const CameraBuffer* SerializedData;	// the last deserialized camera data
	};
}