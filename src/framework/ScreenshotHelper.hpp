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

#ifndef COMMON_SCREENSHOT_HPP_
#define COMMON_SCREENSHOT_HPP_

// Common
#include <framework/Checks.hpp>

// Driveworks
#include <dw/sensors/Sensors.h>
#include <dwvisualization/interop/ImageStreamer.h>
#include <dwvisualization/image/FrameCapture.h>

#include <string>
#include <lodepng.h>

namespace dw_samples
{
namespace common
{

class ScreenshotHelper
{
public:
    ScreenshotHelper(dwContextHandle_t ctx, dwSALHandle_t sal, uint32_t width, uint32_t height, std::string path);
    virtual ~ScreenshotHelper();

    // Set a flag to take a screenshot when processScreenshotTrig is called.
    // Useful for calling screenshot from different thread
    void triggerScreenshot(const std::string filename = "");

    // If the flag is set, take a screenshot. This allows the screenshot to be called from a different thread.
    void processScreenshotTrig();

    // Take a screenshot. This must be called from a thread with access to GL context.
    void takeScreenshot();

private:
    dwImageStreamerHandle_t m_streamer;
    dwFrameCaptureHandle_t m_frameCapture;

    dwImageHandle_t m_imageGL;

    uint32_t m_screenshotCount{0};

    dwRect m_roi;

    std::string m_pathName;

    std::string m_filename;

    // internal flag to trigger a screenshot. Used by triggerScreenshot and processScreenshotTrig.
    bool m_screenshotTrigger;
};
} // namespace common
} // namespace dw_samples

#endif
