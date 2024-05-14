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
// SPDX-FileCopyrightText: Copyright (c) 2015-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef COMMON_CAMERAFRAMEPIPELINE_HPP_
#define COMMON_CAMERAFRAMEPIPELINE_HPP_

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

namespace dw_samples
{
namespace common
{

//-------------------------------------------------------------------------------
/**
* Class to process camera frame. Supports streaming and converting
* (once, in that order) so the returned image is in the expected format.
* The real type matches the type requested in the output properties.
*/
class CameraFramePipeline
{
public:
    CameraFramePipeline(const dwImageProperties& inputImageProperties,
                        dwCameraOutputType outputType,
                        dwContextHandle_t ctx,
                        cudaStream_t cudaStream = cudaStream_t{cudaStreamDefault});

    virtual ~CameraFramePipeline();

    /// Set the optional cuda stream to execute image conversions
    virtual void setCUDAStream(cudaStream_t cudaStream);
    /// Return the cuda stream used to execute image conversions
    virtual cudaStream_t getCUDAStream() const;

    /// Sets up output properties. Initializes streamers and converters
    /// if required.
    virtual void setOutputProperties(const dwImageProperties& outputImageProperties);

    /// Performs single camera frame processing. When exit, the camera frame is fully copied (cuda stream sync)
    virtual void processFrame(dwCameraFrameHandle_t cameraFrame);

    /// Acquire the latest processed frame. Only valid when GL
    /// output is enabled.
    dwImageHandle_t getFrameRgba();
    dwImageHandle_t getFrame();

    bool isRgbaOutputEnabled() const { return m_enableRgba; }

    /// Enables conversion to rgba format
    void enableRgbaOutput();

    const dwImageProperties& getImageProperties() const { return m_inputImgProps; }
    virtual const dwImageProperties& getOutputProperties() const { return m_outputImgProps; }
    dwImageProperties getRgbaOutputProperties() const;

    /// virtual method indicating softISP is present, always false for SimpleCamera
    virtual bool isSoftISPEnabled() const { return false; }

    virtual void reset();

protected:
    dwContextHandle_t m_ctx;

    dwCameraOutputType m_outputType;
    dwImageProperties m_inputImgProps;
    dwImageProperties m_outputImgProps;
    dwImageProperties m_outputRgbaProps;

    std::unique_ptr<SimpleImageStreamer<>> m_streamer;
    dwImageHandle_t m_converter = nullptr;

    dwImageHandle_t m_image     = nullptr;
    dwImageHandle_t m_imageRgba = nullptr;
    bool m_enableRgba;

    cudaStream_t m_cudaStream;
};
}
}

#endif
