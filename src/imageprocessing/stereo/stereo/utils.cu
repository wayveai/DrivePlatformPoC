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

#include "utils.hpp"
#include <cuda.h>
#include <stdexcept>

uint32_t iDivUp(uint32_t a, uint32_t b)
{
    return ((a % b) != 0U) ? ((a / b) + 1U) : (a / b);
}

__global__ void mixDispConfKernel(uint8_t* colorOut, size_t pitchColor, uint8_t* confidence,
                                  size_t pitchConfidence, uint32_t width, uint32_t height,
                                  bool showInvalid)
{
    const uint32_t tidx = blockDim.x * blockIdx.x + threadIdx.x;
    const uint32_t tidy = blockDim.y * blockIdx.y + threadIdx.y;

    if (tidx >= width || tidy >= height)
        return;

    uint8_t val = confidence[tidy * pitchConfidence + tidx];
    if (val > 0)
        val = 1;
    if (showInvalid && (confidence[tidy * pitchConfidence + tidx] == 1))
        val = 255;

    colorOut[tidy * pitchColor + 4 * tidx + 0] = (val == 255) ? val : val * colorOut[tidy * pitchColor + 4 * tidx + 0];
    colorOut[tidy * pitchColor + 4 * tidx + 1] = (val == 255) ? val : val * colorOut[tidy * pitchColor + 4 * tidx + 1];
    colorOut[tidy * pitchColor + 4 * tidx + 2] = (val == 255) ? val : val * colorOut[tidy * pitchColor + 4 * tidx + 2];
}

void mixDispConf(dwImageCUDA* colorOut, dwImageCUDA confidence, bool showInvalid)
{
    dim3 numThreads = dim3(32, 4, 1);

    mixDispConfKernel<<<dim3(iDivUp(confidence.prop.width, numThreads.x),
                             iDivUp(confidence.prop.height, numThreads.y)),
                        numThreads>>>(reinterpret_cast<uint8_t*>(colorOut->dptr[0]),
                                      colorOut->pitch[0],
                                      reinterpret_cast<uint8_t*>(confidence.dptr[0]),
                                      confidence.pitch[0],
                                      confidence.prop.width,
                                      confidence.prop.height,
                                      showInvalid);

    if (cudaGetLastError() != cudaSuccess)
    {
        throw std::runtime_error("Failed to mix disparity and confidence");
    }
}

__host__ __device__ float32_t interpolate(float32_t val, float32_t y0, float32_t x0, float32_t y1, float32_t x1)
{
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

__host__ __device__ float32_t base(float32_t val)
{
    if (val <= -0.75f)
        return 0.f;
    else if (val <= -0.25f)
        return interpolate(val, 0.0f, -0.75f, 1.0f, -0.25f);
    else if (val <= 0.25f)
        return 1.0f;
    else if (val <= 0.75f)
        return interpolate(val, 1.0f, 0.25f, 0.0f, 0.75f);
    else
        return 0.0f;
}

__host__ __device__ dwVector4f fromNormGrayToColor(float32_t normGray)
{
    dwVector4f color;
    color.x = base(normGray - 0.5f);
    color.y = base(normGray);
    color.z = base(normGray + 0.5f);
    color.w = 1.f;

    return color;
}

__global__ void colorCodeKernel(uint8_t* color, size_t pitchOut, uint8_t* gray, size_t pitchIn,
                                uint32_t width, uint32_t height, float32_t scaleFactor, float32_t maxDistance, float32_t depthFactor)
{
    const uint32_t tidx = blockDim.x * blockIdx.x + threadIdx.x;
    const uint32_t tidy = blockDim.y * blockIdx.y + threadIdx.y;

    if (tidx >= width || tidy >= height)
        return;

    // Set 0-disparity to 1-disparity to avoid division by 0 in coloring
    uint32_t disp   = gray[tidy * pitchIn + tidx] == 0 ? 1 : gray[tidy * pitchIn + tidx] * scaleFactor;
    float32_t depth = depthFactor / (disp * maxDistance);

    // Crop relative depths larger than maxDistance
    depth = depth > 1.f ? 1.f : depth;

    // normalize from [0, 1] to [-1, 1] (invert scale to render red as closer blue as further)
    float32_t normGray = -2 * depth + 1.f;

    dwVector4f c                              = fromNormGrayToColor(normGray);
    color[tidy * pitchOut * 1 + 4 * tidx + 0] = c.x * 255.f;
    color[tidy * pitchOut * 1 + 4 * tidx + 1] = c.y * 255.f;
    color[tidy * pitchOut * 1 + 4 * tidx + 2] = c.z * 255.f;
    color[tidy * pitchOut * 1 + 4 * tidx + 3] = c.w * 255.f;
}

void colorCode(dwImageCUDA* colorOut, dwImageCUDA disparity, float32_t scaleFactor, float32_t maxDistance, float32_t depthFactor)
{

    dim3 numThreads = dim3(32, 4, 1);
    colorCodeKernel<<<dim3(iDivUp(disparity.prop.width, numThreads.x),
                           iDivUp(disparity.prop.height, numThreads.y)),
                      numThreads>>>(reinterpret_cast<uint8_t*>(colorOut->dptr[0]), colorOut->pitch[0],
                                    reinterpret_cast<uint8_t*>(disparity.dptr[0]), disparity.pitch[0],
                                    disparity.prop.width, disparity.prop.height, scaleFactor, maxDistance, depthFactor);

    if (cudaGetLastError() != cudaSuccess)
    {
        throw std::runtime_error("Failed to color code");
    }
}

__global__ void createAnaglyphRKernel(uint8_t* color, size_t pitchOut, uint8_t* grayLeft, uint8_t* grayRight,
                                      size_t pitchIn, uint32_t width, uint32_t height)
{
    const uint32_t tidx = blockDim.x * blockIdx.x + threadIdx.x;
    const uint32_t tidy = blockDim.y * blockIdx.y + threadIdx.y;

    if (tidx >= width || tidy >= height)
        return;

    color[tidy * pitchOut + 4 * tidx + 0] = grayLeft[tidy * pitchIn + tidx];
    color[tidy * pitchOut + 4 * tidx + 1] = 0.0f;
    color[tidy * pitchOut + 4 * tidx + 2] = grayRight[tidy * pitchIn + tidx];
    color[tidy * pitchOut + 4 * tidx + 3] = 255;
}

__global__ void createAnaglyphRGBAKernel(uint8_t* color, size_t pitchOut, uint8_t* grayLeft, uint8_t* grayRight,
                                         size_t pitchIn, uint32_t width, uint32_t height)
{
    const uint32_t tidx = blockDim.x * blockIdx.x + threadIdx.x;
    const uint32_t tidy = blockDim.y * blockIdx.y + threadIdx.y;

    if (tidx >= width || tidy >= height)
        return;

    float32_t leftR                       = static_cast<float32_t>(grayLeft[tidy * pitchIn + 4 * tidx + 0]) / 255.0f;
    float32_t leftG                       = static_cast<float32_t>(grayLeft[tidy * pitchIn + 4 * tidx + 1]) / 255.0f;
    float32_t leftB                       = static_cast<float32_t>(grayLeft[tidy * pitchIn + 4 * tidx + 2]) / 255.0f;
    float32_t rightR                      = static_cast<float32_t>(grayRight[tidy * pitchIn + 4 * tidx + 0]) / 255.0f;
    float32_t rightG                      = static_cast<float32_t>(grayRight[tidy * pitchIn + 4 * tidx + 1]) / 255.0f;
    float32_t rightB                      = static_cast<float32_t>(grayRight[tidy * pitchIn + 4 * tidx + 2]) / 255.0f;
    float32_t anaR                        = leftR * 0.33f + leftG * 0.33f + leftB * 0.33f;
    float32_t anaG                        = 0.0f;
    float32_t anaB                        = rightR * 0.33f + rightG * 0.33f + rightB * 0.33f;
    color[tidy * pitchOut + 4 * tidx + 0] = (anaR * 255.0f) > 255.0f ? 255 : static_cast<uint8_t>(anaR * 255.0f);
    color[tidy * pitchOut + 4 * tidx + 1] = (anaG * 255.0f) > 255.0f ? 255 : static_cast<uint8_t>(anaG * 255.0f);
    color[tidy * pitchOut + 4 * tidx + 2] = (anaB * 255.0f) > 255.0f ? 255 : static_cast<uint8_t>(anaB * 255.0f);
    color[tidy * pitchOut + 4 * tidx + 3] = 255;
}

void createAnaglyph(dwImageCUDA anaglyph, dwImageCUDA inputLeft, dwImageCUDA inputRight)
{

    dim3 numThreads = dim3(32, 4, 1);
    if ((inputLeft.prop.format == DW_IMAGE_FORMAT_R_UINT8) || (inputLeft.prop.format == DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR))
    {
        createAnaglyphRKernel<<<dim3(iDivUp(inputLeft.prop.width, numThreads.x),
                                     iDivUp(inputLeft.prop.height, numThreads.y)),
                                numThreads>>>(reinterpret_cast<uint8_t*>(anaglyph.dptr[0]), anaglyph.pitch[0],
                                              reinterpret_cast<uint8_t*>(inputLeft.dptr[0]),
                                              reinterpret_cast<uint8_t*>(inputRight.dptr[0]),
                                              inputLeft.pitch[0], inputLeft.prop.width, inputLeft.prop.height);
    }
    else
    {
        createAnaglyphRGBAKernel<<<dim3(iDivUp(inputLeft.prop.width, numThreads.x),
                                        iDivUp(inputLeft.prop.height, numThreads.y)),
                                   numThreads>>>(reinterpret_cast<uint8_t*>(anaglyph.dptr[0]), anaglyph.pitch[0],
                                                 reinterpret_cast<uint8_t*>(inputLeft.dptr[0]),
                                                 reinterpret_cast<uint8_t*>(inputRight.dptr[0]),
                                                 inputLeft.pitch[0], inputLeft.prop.width, inputLeft.prop.height);
    }

    if (cudaGetLastError() != cudaSuccess)
    {
        throw std::runtime_error("Failed to create anaglyph");
    }
}
