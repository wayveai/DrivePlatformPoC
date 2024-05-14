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
// SPDX-FileCopyrightText: Copyright (c) 2020-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef DRIVEWORKSSAMPLE_SEEK_HPP_
#define DRIVEWORKSSAMPLE_SEEK_HPP_

#include <framework/SimpleCamera.hpp>
#include <dwvisualization/core/RenderEngine.h>
#include <framework/ChecksExt.hpp>
#include "DriveWorksSample.hpp"
#include <framework/WindowGLFW.hpp>

namespace dw_samples
{
namespace common
{

//-------------------------------------------------------------------------------
/**
* Driveworks sample base class that adds frame seeking functionality
* Features:
* - Seeking forward and backwards in video with left and right arrow keys
* - Change how many frames to seek at a time with the up and down arrow keys
* - Seek back to the beginning of the video with the home key
*
* How to use:
* Inherited classes need to override virtual derived*(derivedInit, derivedProcess etc.) functions
* to complete implementation
*
* Implementation:
* to allow child classes to direclty inherit DriveWorksSampleSeek without calling extra functions
* from the parent class the virtual functions from the base DriveWorksSample class
* onInitialize, onProcess etc. are overrided in this class and the inheritance chain is passed sidways
* to the derived functions(derivedInit, derivedProcess etc...)
*
* Dependencies: derivedinit must initialize the camera and renderEngine object
* in order to initialize seeking properly
*/
class DriveWorksSampleSeek : public DriveWorksSample
{
public:
    DriveWorksSampleSeek(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    virtual ~DriveWorksSampleSeek() = default;

    bool onInitialize() override;
    void onProcess() override;
    void onRender() override;
    // to enable pressing keys down for continuous seeking
    void onKeyRepeat(int key, int scancode, int mods) override;
    void onKeyDown(int key, int /*scancode*/, int /*mods*/) override;

protected:
    // seeking specific functionality
    virtual void seekInit();
    virtual void seekRender();
    virtual void seekProcess();
    void seekOnKeyRepeat(int key, int scancode, int mods);
    void seekOnKeyDown(int key);
    // virtual functions that need to be overwritten by sample app class implementation
    virtual bool derivedInit() { return true; }
    virtual void derivedProcess() {}
    virtual void derivedRender() {}
    virtual void derivedOnKeyRepeat(int key, int scancode, int mods)
    {
        (void)key;
        (void)scancode;
        (void)mods;
    }
    virtual void derivedOnKeyDown(int key) { (void)key; }
    // visualziation helper function to display frame info on screen
    void displayFrameInfo();
    // boilerplate code to grab next available image frame from camera
    virtual void getNextFrame(dwImageHandle_t* nextFrameOut, dwImageGL** nextFrameGL);

    dwContextHandle_t m_context = DW_NULL_HANDLE;
    dwImageGL* m_imgGl;
    std::unique_ptr<SimpleCamera> m_camera;
    std::unique_ptr<SimpleImageStreamerGL<>> m_streamerCUDA2GL;
    dwImageHandle_t m_imageRGBA;
    dwImageHandle_t m_rcbImage            = nullptr;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;

    uint32_t m_imageWidth  = 0;
    uint32_t m_imageHeight = 0;

    // seeking video variables
    static const int32_t TIMEOUT         = 1000;
    const uint32_t MIN_FRAME_SKIP_NUMBER = 1;
    const uint32_t MAX_FRAME_SKIP_NUMBER = 10000;
    bool m_seekingEnabled                = true; // default on
    int64_t m_seekFrame                  = 0;
    int64_t m_curFrame                   = -1;
    uint32_t m_frameSkipNumber           = 10;
    size_t m_totFrameCount               = 0;
    dwTime_t m_videoStartTimestamp       = 0;
    dwTime_t m_videoEndTimestamp         = 0;
    // check whether we need to force processing of a frame when the video is paused
    bool m_forceProcessFrame = false;
    bool m_displayFrameNum   = false;

    bool m_enterSeekToFrameMode{false};
    uint32_t m_seekToFrame{0};

    // start and end frame index
    // for single camera sample, they are simply 0 and (numFrames - 1) respectively
    // for multiple cameras sample they are (startFrameIdx in the main camera) and (endFrameIdx in the main camera) respetively
    int64_t m_videoStartFrameIdx{};
    int64_t m_videoEndFrameIdx{};
};

} // namespace common
} // namespace dw_samples

#endif // DRIVEWORKSSAMPLE_SEEK_HPP_
