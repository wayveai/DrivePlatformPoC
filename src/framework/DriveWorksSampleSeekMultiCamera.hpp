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
// SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef DRIVEWORKSSAMPLE_SEEK_MULTICAMERA_HPP_
#define DRIVEWORKSSAMPLE_SEEK_MULTICAMERA_HPP_

#include <samples/framework/DriveWorksSampleSeek.hpp>

namespace dw_samples
{
namespace common
{

static constexpr uint32_t MAX_NUM_CAMERAS = 12;

//-------------------------------------------------------------------------------
/**
* Driveworks sample base class that adds frame seeking functionality for multiple camera inputs
* Features:
* - Features that DriveWorksSampleSeek class provides
* - Input videos are not guaranteed to start at the same timestamp
* - However, all the frames from the different videos are synchronously captured at the same timestamp
* - Only the overlapping sections of all the videos are played in the sample app
*
* How to use:
* Inherited classes need to override virtual derived*(derivedInit, derivedProcess etc.) functions
* to complete implementation
*
* Implementation:
* to allow child classes to direclty inherit DriveWorksSampleSeekMultiCamera without calling extra functions
* from the parent class the virtual functions from the base DriveWorksSampleSeekMultiCamera class
* onInitialize, onProcess etc. are overrided in this class and the inheritance chain is passed sidways
* to the derived functions(derivedInit, derivedProcess etc...)
*
* Dependencies: derivedinit must initialize the camera and renderEngine object
* in order to initialize seeking properly
*/

class DriveWorksSampleSeekMultiCamera : public DriveWorksSampleSeek
{
public:
    DriveWorksSampleSeekMultiCamera(const ProgramArguments& args)
        : DriveWorksSampleSeek(args)
        , m_restartCameras(true) {}

    virtual ~DriveWorksSampleSeekMultiCamera() = default;

protected:
    static constexpr uint32_t MAIN_CAMERA_INDEX = 0;

    // seeking specific functionality
    void seekInit() override;
    void seekRender() override;
    void seekProcess() override;

    bool getNextFrameSingleCamera(dwImageHandle_t* nextFrameOut, dwImageGL** nextFrameGL, uint32_t cameraIdx);

    dwImageGL* m_imgGl[MAX_NUM_CAMERAS]{};
    uint32_t m_numCameras{};
    std::unique_ptr<SimpleCamera> m_cameras[MAX_NUM_CAMERAS];
    std::unique_ptr<SimpleImageStreamerGL<>> m_streamerCUDA2GL[MAX_NUM_CAMERAS];

    dwImageHandle_t m_imageRGBA[MAX_NUM_CAMERAS]{};
    dwImageHandle_t m_rcbImage[MAX_NUM_CAMERAS]{};

    size_t m_numFramesPerCamera[MAX_NUM_CAMERAS]     = {};
    dwTime_t m_videoStartTimestamps[MAX_NUM_CAMERAS] = {};
    dwTime_t m_videoEndTimestamps[MAX_NUM_CAMERAS]   = {};

    dwTime_t m_videoStartTimestamp = std::numeric_limits<dwTime_t>::min();
    dwTime_t m_videoEndTimestamp   = std::numeric_limits<dwTime_t>::max();

    // the flag to indicate whether we should restart all the cameras synchronously
    bool m_restartCameras{true};

    // frame difference of each camera w.r.t m_videoStartFrameIdx
    int64_t m_frameOffset[MAX_NUM_CAMERAS] = {};
};

} // namespace common
} // namespace dw_samples

#endif // DRIVEWORKSSAMPLE_SEEK_MULTICAMERA_HPP_
