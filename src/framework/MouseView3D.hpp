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

#ifndef SAMPLES_COMMON_MOUSEVIEW3D_HPP_
#define SAMPLES_COMMON_MOUSEVIEW3D_HPP_

#include <dw/core/base/Types.h>

class MouseView3D
{
public:
    MouseView3D();

    //4x4 matrix in col-major format
    const dwMatrix4f* getModelView() const
    {
        return &m_modelView;
    }

    //4x4 matrix in col-major format
    const dwMatrix4f* getProjection() const
    {
        return &m_projection;
    }

    const float32_t* getEye() const
    {
        return m_eye;
    }

    void setWindowAspect(float32_t aspect);

    void setFov(float32_t fovRads)
    {
        m_fovRads = fovRads;
    }

    void mouseDown(int button, float32_t x, float32_t y);
    void mouseUp(int button, float32_t x, float32_t y);
    void mouseMove(float32_t x, float32_t y);
    void mouseWheel(float32_t dx, float32_t dy);

    void setCenter(float32_t x, float32_t y, float32_t z);
    void setRadiusFromCenter(float32_t zoom);
    void setAngleFromCenter(float32_t yaw, float32_t pitch);
    const float32_t* getCenter() const
    {
        return m_center;
    }

private:
    dwMatrix4f m_modelView;
    dwMatrix4f m_projection;

    float32_t m_windowAspect; // width/height
    float32_t m_fovRads;
    float32_t m_center[3];
    float32_t m_up[3];
    float32_t m_eye[3];

    float32_t m_zNear;
    float32_t m_zFar;

    // MOUSE NAVIGATION VARIABLES
    float32_t m_startAngles[2];
    float32_t m_startCenter[3];

    float32_t m_radius;
    float32_t m_angles[2];

    bool m_mouseLeft;
    bool m_mouseRight;
    float32_t m_currentPos[2];

    void updateEye();
    void updateMatrices();
};

#endif // SAMPLES_COMMON_MOUSEVIEW3D_HPP_
