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

#include "CameraFramePipeline.hpp"

#include <framework/Log.hpp>

namespace dw_samples
{
namespace common
{

///////////////////////////////////////////////////////////////////////////////
/// CameraFramePipeline
///////////////////////////////////////////////////////////////////////////////

CameraFramePipeline::CameraFramePipeline(const dwImageProperties& inputImageProperties,
                                         dwCameraOutputType outputType,
                                         dwContextHandle_t ctx,
                                         cudaStream_t cudaStream)
    : m_ctx(ctx)
    , m_outputType(outputType)
    , m_inputImgProps(inputImageProperties)
    , m_outputImgProps(inputImageProperties)
    , m_enableRgba(false)
    , m_cudaStream(cudaStream)
{
}

CameraFramePipeline::~CameraFramePipeline()
{
    if (m_converter != DW_NULL_HANDLE)
        dwImage_destroy(m_converter);
}

void CameraFramePipeline::setCUDAStream(cudaStream_t cudaStream)
{
    m_cudaStream = cudaStream;
}

cudaStream_t CameraFramePipeline::getCUDAStream() const
{
    return m_cudaStream;
}

void CameraFramePipeline::setOutputProperties(const dwImageProperties& outputImageProperties)
{
    m_outputImgProps        = outputImageProperties;
    m_outputImgProps.width  = m_inputImgProps.width;
    m_outputImgProps.height = m_inputImgProps.height;

    if (m_inputImgProps.type != m_outputImgProps.type)
    {
        m_streamer.reset(new SimpleImageStreamer<>(m_inputImgProps, m_outputImgProps.type, 60000, m_ctx));
    }

    if (m_inputImgProps.format != m_outputImgProps.format)
    {
        dwImage_create(&m_converter, m_outputImgProps, m_ctx);
    }
}

void CameraFramePipeline::enableRgbaOutput()
{
    m_outputRgbaProps        = getOutputProperties();
    m_outputRgbaProps.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    // we hardcode here that the output rgba image should be DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8
    m_outputRgbaProps.type = DW_IMAGE_CUDA;
    m_enableRgba           = true;
}

dwImageProperties CameraFramePipeline::getRgbaOutputProperties() const
{
    return m_outputRgbaProps;
}

dwImageHandle_t CameraFramePipeline::getFrame()
{
    return m_image;
}

dwImageHandle_t CameraFramePipeline::getFrameRgba()
{
    if (!isRgbaOutputEnabled())
    {
        logWarn("CameraFramePipeline: RGBA output is not enabled. Did you forget to call enableRgbaOutput()?\n");
    }
    return m_imageRgba;
}

void CameraFramePipeline::processFrame(dwCameraFrameHandle_t cameraFrame)
{
    dwImageHandle_t img;
    CHECK_DW_ERROR(dwSensorCamera_getImage(&img, m_outputType, cameraFrame));

    m_image = img;
    if (m_streamer)
    {
        m_image = m_streamer->post(img);
    }

    if (m_converter)
    {
        dwStatus res = dwImage_copyConvertAsync(m_converter, m_image, m_cudaStream, m_ctx);
        if (res != DW_SUCCESS)
        {
            logError("CameraFramePipeline: error converting image: %s\n", dwGetStatusName(res));
        }

        cudaError_t const error = cudaStreamSynchronize(m_cudaStream);
        if (error != cudaSuccess)
        {
            logError("CameraFramePipeline: cudaStreamSynchronize, failed to synchronize");
        }

        m_image = m_converter;
    }

    // Rgba & OpenGL
    // If SoftIsp pipeline is enabled it needs to be processed first
    if (!isSoftISPEnabled())
    {
        if (isRgbaOutputEnabled())
        {
            CHECK_DW_ERROR(dwSensorCamera_getImage(&img, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, cameraFrame));
            m_imageRgba = img;
        }
    }
}

void CameraFramePipeline::reset()
{
    if (m_streamer)
    {
        m_streamer->release();
    }
}
}
}
