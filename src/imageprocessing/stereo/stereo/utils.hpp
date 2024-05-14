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

#ifndef SAMPLES_STEREO_SAMPLEUTILS_HPP_
#define SAMPLES_STEREO_SAMPLEUTILS_HPP_

#include <dw/core/base/Types.h>
#include <dw/image/Image.h>

void mixDispConf(dwImageCUDA* colorOut, dwImageCUDA confidence, bool showInvalid);

void colorCode(dwImageCUDA* colorOut, dwImageCUDA disparity, float32_t scaleFactor, float32_t maxDistance, float32_t depthFactor);

void createAnaglyph(dwImageCUDA anaglyph, dwImageCUDA inputLeft, dwImageCUDA inputRight);

__host__ __device__ float32_t interpolate(float32_t val, float32_t y0, float32_t x0, float32_t y1, float32_t x1);

__host__ __device__ float32_t base(float32_t val);

__host__ __device__ dwVector4f fromNormGrayToColor(float32_t normGray);

#endif
