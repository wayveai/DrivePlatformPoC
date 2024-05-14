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
// SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_COMMON_WINDOW_LINUX_EGL_HPP_
#define SAMPLES_COMMON_WINDOW_LINUX_EGL_HPP_

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "WindowEGL.hpp"

/**
 * @brief The EGLDisplay class for Linux platform
 */
class WindowLinuxEGL : public WindowEGL
{
public:
    WindowLinuxEGL(int width, int height, bool offscreen, int samples);
    virtual ~WindowLinuxEGL();

private:
    bool initDrm(const char* name, EGLAttrib* layerAttribs, int& dispWidth, int& dispHeight);

    PFNEGLSTREAMCONSUMEROUTPUTEXTPROC eglStreamConsumerOutputEXT = nullptr;
    PFNEGLGETOUTPUTLAYERSEXTPROC eglGetOutputLayersEXT           = nullptr;
};

#endif // SAMPLES_COMMON_WINDOW_LINUX_EGL_HPP_
