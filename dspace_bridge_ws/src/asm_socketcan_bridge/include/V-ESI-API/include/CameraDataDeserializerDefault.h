// Copyright 2021 dSPACE GmbH. All rights reserved.

#pragma once

#include "CameraDataDeserializerBase.h"

namespace CameraDataDeserializers
{
	/**
	 * This deserializer is able to deserialize the output of a standard camera sensor.
	 */
	class SENSOR_DESERIALIZER_API CameraDataDeserializerDefault : public CameraDataDeserializerBase
	{
	public:
		/**
		 * The ctor.
		 */
		CameraDataDeserializerDefault();

		/**
		 * The dtor.
		 */
		virtual ~CameraDataDeserializerDefault() = default;

		/**
		 * Call this method before calling any other methods. This will initialize the deserializer.
		 * @returns false on error and true on success.
		 */
		virtual bool Initialize() override;

		/**
		 * Call this method before the class instance is destroyed.
		 * @returns false on error and true on success.
		 */
		virtual bool Destroy() override;

		/**
		 * Deserializes the camera data.
		 * Do not delete or modify the data while the memory area is being shared between the deserializer and other components.
		 * @param Data struct describing and containing the serialized camera data
		 * @returns false on error and true on success.
		 */
		virtual bool Deserialize(const CameraBuffer* Data) override;

		/**
		 * Returns a camera image.
		 * @param Type the type of the image.
		 * @returns the camera image.
		 */
		virtual const CameraImage& GetImage(CameraImageType Type) const override;

		/**
		 * Returns camera data.
		 * @param Type of the data to return.
		 * @returns the camera data.
		 */
		virtual const CameraData& GetData(CameraDataType Type) const override;

		/**
		 * Returns the state of the current frame. The state can be used to decode the output images like color or depth.
		 * @returns the camera frame state.
		 */
		const CameraFrameContext& GetFrameContext() const;

	protected:
		CameraFrameContext FrameContext;

		CameraImage ColorImage;
		CameraImage ClassIDImage;
		CameraImage InstanceIDImage;
		CameraImage VelocityImage;
		CameraImage DepthImage;

		CameraData ObjectBounds;
	};
}