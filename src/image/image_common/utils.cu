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
// SPDX-FileCopyrightText: Copyright (c) 2015-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
// clang-format off
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// clang-format on
__global__ void kernel(uint8_t* image, size_t pitch, const uint32_t width, const uint32_t height,
                       const uint32_t val, const dwImageFormat format)
{
    const uint32_t tidx = blockDim.x * blockIdx.x + threadIdx.x;
    const uint32_t tidy = blockDim.y * blockIdx.y + threadIdx.y;

    if (tidx >= width || tidy >= height)
        return;

    if (format == DW_IMAGE_FORMAT_RGB_UINT8)
    {
        image[tidy * pitch + 3 * tidx + 0] = (tidx + val) % 255;
        image[tidy * pitch + 3 * tidx + 1] = (tidy + val) % 255;
        image[tidy * pitch + 3 * tidx + 2] = (tidx + tidy + 2 * val) % 255;
    }
    else
    {
        image[tidy * pitch + 4 * tidx + 0] = (tidx + val) % 255;
        image[tidy * pitch + 4 * tidx + 1] = (tidy + val) % 255;
        image[tidy * pitch + 4 * tidx + 2] = (tidx + tidy + 2 * val) % 255;
        image[tidy * pitch + 4 * tidx + 3] = 255;
    }
}

uint32_t iDivUp(const uint32_t a, const uint32_t b)
{
    return ((a % b) != 0U) ? ((a / b) + 1U) : (a / b);
}

void generateImage(dwImageHandle_t image, const uint32_t val)
{
    dwImageProperties prop;
    dwImage_getProperties(&prop, image);
    if (prop.format != DW_IMAGE_FORMAT_RGB_UINT8 && prop.format != DW_IMAGE_FORMAT_RGBA_UINT8)
    {
        throw std::runtime_error("unsupported format in image samples generateImage");
    }

    if (prop.type == DW_IMAGE_CUDA)
    {
        dwImageCUDA* imgCUDA;
        dwImage_getCUDA(&imgCUDA, image);
        dim3 numThreads = dim3(32, 4, 1);
        kernel<<<dim3(iDivUp(prop.width, numThreads.x),
                      iDivUp(prop.height, numThreads.y)),
                 numThreads>>>(static_cast<uint8_t*>(imgCUDA->dptr[0]), imgCUDA->pitch[0], prop.width,
                               prop.height, val, prop.format);
#ifdef VIBRANTE
#if VIBRANTE_PDK_DECIMAL < 6000400
    }
    else if (prop.type == DW_IMAGE_NVMEDIA)
    {
        dwImageNvMedia* imgNvMedia;
        dwImage_getNvMedia(&imgNvMedia, image);

        NvMediaImageSurfaceMap mapping;
        NvMediaStatus nvmStatus = NvMediaImageLock(imgNvMedia->img, NVMEDIA_IMAGE_ACCESS_WRITE, &mapping);
        if (nvmStatus != NVMEDIA_STATUS_OK)
        {
            throw std::runtime_error("error in generateImage");
        }

        uint8_t* ptr   = reinterpret_cast<uint8_t*>(mapping.surface[0].mapping);
        uint32_t pitch = mapping.surface[0].pitch;

        for (uint32_t i = 0; i < prop.height; ++i)
        {
            for (uint32_t j = 0; j < prop.width; ++j)
            {
                ptr[i * pitch + j * 4 + 0] = (j + val) % 255;
                ptr[i * pitch + j * 4 + 1] = (i + val) % 255;
                ptr[i * pitch + j * 4 + 2] = (i + j + 2 * val) % 255;
                ptr[i * pitch + j * 4 + 3] = 255;
            }
        }

        NvMediaImageUnlock(imgNvMedia->img);
#endif
#endif
    }
    else
    {
        throw std::runtime_error("unsupported type in image samples generateImage");
    }
}
// clang-format on
#pragma GCC diagnostic pop
// clang-format off
