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
// SPDX-FileCopyrightText: Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_FRAMEWORK_CHECKS_EXT_HPP_
#define SAMPLES_FRAMEWORK_CHECKS_EXT_HPP_

#include "Checks.hpp"
#include <dwvisualization/gl/GL.h>
#include <cuda_runtime.h>

#include <iostream>
#include <string>
#include <stdexcept>

//------------------------------------------------------------------------------
// print GL error with location
inline void getGLError(int line, const char* file, const char* function)
{
    GLenum error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::cerr << file << " in function " << function << " in line " << line << ": glError: 0x"
                  << std::hex << error << std::dec << std::endl;
        exit(1);
    }
}

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

// macro to easily check for GL errors
#define CHECK_GL_ERROR() getGLError(__LINE__, __FILE__, __PRETTY_FUNCTION__);

#endif // SAMPLES_FRAMEWORK_CHECKS_EXT_HPP_
