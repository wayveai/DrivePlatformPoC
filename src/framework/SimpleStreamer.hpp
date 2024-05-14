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
// SPDX-FileCopyrightText: Copyright (c) 2017-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_COMMON_SIMPLESTREAMER_HPP_
#define SAMPLES_COMMON_SIMPLESTREAMER_HPP_

#include <dw/interop/streamer/ImageStreamer.h>
#include <dwvisualization/interop/ImageStreamer.h>
#include <framework/Checks.hpp>

namespace dw_samples
{
namespace common
{

template <typename T>
inline T getTyped(dwImageHandle_t img);

template <>
inline dwImageHandle_t getTyped<dwImageHandle_t>(dwImageHandle_t img)
{
    return img;
}

template <>
inline dwImageCPU* getTyped<dwImageCPU*>(dwImageHandle_t img)
{
    dwImageCPU* imgCPU;
    dwImage_getCPU(&imgCPU, img);
    return imgCPU;
}

template <>
inline dwImageCUDA* getTyped<dwImageCUDA*>(dwImageHandle_t img)
{
    dwImageCUDA* imgCUDA;
    dwImage_getCUDA(&imgCUDA, img);
    return imgCUDA;
}

template <>
inline dwImageGL* getTyped<dwImageGL*>(dwImageHandle_t img)
{
    dwImageGL* imgGL;
    dwImage_getGL(&imgGL, img);
    return imgGL;
}

#ifdef VIBRANTE
template <>
inline dwImageNvMedia* getTyped<dwImageNvMedia*>(dwImageHandle_t img)
{
    dwImageNvMedia* imgNvMedia;
    dwImage_getNvMedia(&imgNvMedia, img);
    return imgNvMedia;
}
#endif

/**
 * A wrapper for the streamer classes. It sacrifices performance to provide a very simple interface.
 * Posting an image blocks and directly returns the image.
 *
 * Usage:
 * \code
 * SimpleImageStreamer streamer(propsIn, DW_IMAGE_CUDA, 66000, ctx);
 *
 * dwImageCUDA inputImg = getImgFromSomewhere();
 *
 * dwImageGL *outputImg = streamer.post(&inputImg);
 * ...do GL stuff...
 * streamer.release();
 *
 * \endcode
 *
 * NOTE: we strongly encourage to use the real dwImageStreamer, please see the samples in Image for a
 *       complete tutorial
 */
template <typename T = dwImageHandle_t>
class SimpleImageStreamer
{
public:
    SimpleImageStreamer(const dwImageProperties& imageProps, dwImageType typeOut, dwTime_t timeout, dwContextHandle_t ctx)
        : m_timeout(timeout)
        , m_pendingReturn(nullptr)
    {
        if (typeOut == DW_IMAGE_GL)
        {
            throw std::runtime_error("Cannot use SimpleImageStreamer for GL images, use SimpleImageStreamerGL instead");
        }
        CHECK_DW_ERROR(dwImageStreamer_initialize(&m_streamer, &imageProps, typeOut, ctx));
    }

    ~SimpleImageStreamer()
    {
#ifndef DW_USE_NVMEDIA_DRIVE
        if (m_pendingReturn)
            release();
#endif
        dwImageStreamer_release(m_streamer);
    }

    /// Posts the input image, blocks until the output image is available, returns the output image.

    typename std::conditional<std::is_same<T, dwImageHandle_t>::value, T, T*>::type post(dwImageHandle_t imgS)
    {
        if (m_pendingReturn)
            release();

        CHECK_DW_ERROR(dwImageStreamer_producerSend(imgS, m_streamer));
        CHECK_DW_ERROR(dwImageStreamer_consumerReceive(&m_pendingReturn, m_timeout, m_streamer));

        if (!m_pendingReturn)
            throw std::runtime_error("Cannot receive image");

        return getTyped<typename std::conditional<std::is_same<T, dwImageHandle_t>::value, T, T*>::type>(m_pendingReturn);
    }

    /// Returns the previously received image to the real streamer.
    /// This method is optional. Either post() or the destructor will also return the image.
    void release()
    {
        if (m_pendingReturn)
        {
            CHECK_DW_ERROR(dwImageStreamer_consumerReturn(&m_pendingReturn, m_streamer));

            m_pendingReturn = nullptr;

            CHECK_DW_ERROR(dwImageStreamer_producerReturn(nullptr, m_timeout, m_streamer));
        }
    }

private:
    dwImageStreamerHandle_t m_streamer;
    dwTime_t m_timeout;

    dwImageHandle_t m_pendingReturn;
};

/// Similar to SimpleImageStreamer but streams to GL
/// It is a different class because the streaming methods to call are different.
template <typename T = dwImageHandle_t>
class SimpleImageStreamerGL
{
public:
    SimpleImageStreamerGL(const dwImageProperties& imageProps,
                          dwTime_t timeout,
                          dwContextHandle_t ctx,
                          cudaStream_t cudaStream = cudaStreamDefault)
        : m_timeout(timeout)
        , m_pendingReturn(nullptr)
    {
        CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_streamer, &imageProps, DW_IMAGE_GL, ctx));
        if (imageProps.type == DW_IMAGE_CUDA)
        {
            setCudaStream(cudaStream);
        }
    }

    ~SimpleImageStreamerGL()
    {
#ifndef DW_USE_NVMEDIA_DRIVE
        if (m_pendingReturn)
            release();
#endif
        CHECK_DW_ERROR_NOTHROW(dwImageStreamerGL_release(m_streamer));
    }

    cudaStream_t getCudaStream() const
    {
        cudaStream_t cudaStream{};
        CHECK_DW_ERROR(dwImageStreamerGL_getCUDAStream(&cudaStream, m_streamer));
        return cudaStream;
    }

    void setCudaStream(cudaStream_t cudaStream)
    {
        CHECK_DW_ERROR(dwImageStreamerGL_setCUDAStream(cudaStream, m_streamer));
    }

    using TReturn = typename std::conditional<std::is_same<T, dwImageHandle_t>::value, T, T*>::type;

    /// Posts the input image, blocks until the output image is available, returns the output image.
    TReturn post(dwImageHandle_t imgS)
    {
        if (m_pendingReturn)
            release();

        CHECK_DW_ERROR(dwImageStreamerGL_producerSend(imgS, m_streamer));
        CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&m_pendingReturn, m_timeout, m_streamer));

        if (!m_pendingReturn)
            throw std::runtime_error("Cannot receive image");

        return getTyped<TReturn>(m_pendingReturn);
    }

    /// The last image returned by post()
    TReturn getLast()
    {
        return getTyped<TReturn>(m_pendingReturn);
    }

    /// Returns the previously received image to the real streamer.
    /// This method is optional. Either post() or the destructor will also return the image.
    void release()
    {
        if (m_pendingReturn)
        {
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&m_pendingReturn, m_streamer));

            m_pendingReturn = nullptr;

            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, m_timeout, m_streamer));
        }
    }

private:
    dwImageStreamerHandle_t m_streamer;
    dwTime_t m_timeout;

    dwImageHandle_t m_pendingReturn;
};

} // namespace common
} // namespace dw_samples

#endif
