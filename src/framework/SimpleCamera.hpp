/////////////////////////////////////////////////////////////////////////////////////////
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
// expressly authorized by NVIDIA. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA CORPORATION & AFFILIATES.
//
// SPDX-FileCopyrightText: Copyright (c) 2015-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef COMMON_SIMPLECAMERA_HPP_
#define COMMON_SIMPLECAMERA_HPP_

// Driveworks
#include <dw/core/context/Context.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>

// C++ Std
#include <memory>
#include <vector>
#include <type_traits>
#include <chrono>
#include <thread>

// Common
#include <framework/Checks.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/CameraFramePipeline.hpp>

namespace dw_samples
{
namespace common
{

/**
* Simple class to get images from a camera. It supports streaming and converting (once, in that order)
* so the returned image is in the expected format. It returns the generic dwImageGeneric type that points
* to an underlying concrete dwImageXXX struct. The real type matches the type requested in the output properties.
* The class uses CameraFramePipeline for the processing.
* The accepted protocols are:
* camera.gmsl
* camera.virtual
* camera.usb
*
* Usage:
* \code
* SimpleCamera camera(propsOut, sensorParams, sal, ctx);
*
* for(dwImageGeneric *img = camera.readFrame(); img!=nullptr; img=camera.readFrame())
* {
*   // Do things with img
* }
*
* \endcode
*
* NOTE: for tutorials and best practices about Camera sensors, please see sensors/camera samples
*/
class SimpleCamera
{
public:
    /// empty constructor
    SimpleCamera(dwSALHandle_t sal, dwContextHandle_t ctx);

    /// creates a simple camera that outputs a frame with the properties of the camera image
    SimpleCamera(const dwSensorParams& params, dwSALHandle_t sal, dwContextHandle_t ctx,
                 dwCameraOutputType outputType = DW_CAMERA_OUTPUT_NATIVE_PROCESSED);
    /**
     * creates a simple camera and also sets up image streamer and format converter to output a
     * converted image, with properties different from the properties of the camera image
    **/
    SimpleCamera(const dwImageProperties& outputProperties, const dwSensorParams& params, dwSALHandle_t sal,
                 dwContextHandle_t ctx, dwCameraOutputType outputType = DW_CAMERA_OUTPUT_NATIVE_PROCESSED);

    virtual ~SimpleCamera();

    /**
     * sets up streamer and converter to be used when acquiring a frame if outputProperties are different
     * from input properties
    **/
    void setOutputProperties(const dwImageProperties& outputProperties);

    dwSensorHandle_t getSensorHandle() { return m_sensor; }
    dwCameraFrameHandle_t getFrameHandle() { return m_pendingFrame; }

    const dwCameraProperties& getCameraProperties() const { return m_cameraProperties; }
    const dwImageProperties& getImageProperties() const { return m_framePipeline->getImageProperties(); }
    virtual const dwImageProperties& getOutputProperties() const { return m_framePipeline->getOutputProperties(); }

    static dwTime_t constexpr DEFAULT_READ_FRAME_TIMEOUT = 3000000;
    virtual dwImageHandle_t readFrame(dwTime_t timeout = DEFAULT_READ_FRAME_TIMEOUT);
    void getFrameTimestamp(dwTime_t* timestamp);

    /// Seeking functionality
    bool enableSeeking(size_t& frameCount, dwTime_t& startTimestamp, dwTime_t& endTimestamp);
    void seekToTime(dwTime_t timestamp);
    void seekToFrame(size_t frameIdx);
    void getCurrFrameIdx(size_t* frameIdx);

    /// Releases the frame returned by readFrame. Calling this is optional.
    void releaseFrame();

    void resetCamera();

    /// Enables conversion and streaming directly to GL for each frame read
    /// After this is enabled, getFrameGL() will return the GL image for the last read frame.
    void enableGLOutput();

    bool isGLOutputEnabled() const { return m_streamerGL != nullptr; }

    /// Returns the frame converted to RGBA format of the same type as the input image
    /// Only valid when GL output has been enabled
    dwImageHandle_t getFrameRgba() const { return m_framePipeline->getFrameRgba(); }

    /// Returns the frame converted to RGBA format as a GL frame
    /// Only valid when GL output has been enabled
    dwImageHandle_t getFrameRgbaGL();

protected:
    void createSensor(dwImageProperties& imageProps, const dwSensorParams& params, dwCameraOutputType outputType);

    dwContextHandle_t m_ctx;
    dwSALHandle_t m_sal;

    dwSensorHandle_t m_sensor;
    dwCameraProperties m_cameraProperties;

    dwCameraFrameHandle_t m_pendingFrame;

    std::unique_ptr<CameraFramePipeline> m_framePipeline;

    std::unique_ptr<SimpleImageStreamerGL<>> m_streamerGL;
    dwImageHandle_t m_imageRgbaGL = nullptr;

    bool m_started;
};
}
}

#endif
