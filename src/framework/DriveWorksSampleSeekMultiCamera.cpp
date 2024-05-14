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

#include <samples/framework/DriveWorksSampleSeekMultiCamera.hpp>

namespace dw_samples
{
namespace common
{

//------------------------------------------------------------------------
void DriveWorksSampleSeekMultiCamera::seekRender()
{
    // If video is paused and seeking occurs, reprocess the frame for detections
    if (m_seekingEnabled && m_forceProcessFrame)
    {
        onProcess();
        m_forceProcessFrame = false;
    }
}

//------------------------------------------------------------------------
void DriveWorksSampleSeekMultiCamera::seekProcess()
{
    if (m_restartCameras)
    {
        // all the frames from the different videos are synchronously captured at the same timestamp
        // only the overlapping section of all the videos are played in the sample app
        // each camera is reset to the start timestamp of the overlapping section
        // and the current frame is set the start frameIdxJ
        for (uint32_t i = 0; i < m_numCameras; ++i)
        {
            m_cameras[i]->seekToTime(m_videoStartTimestamp);
            m_curFrame  = m_videoStartFrameIdx;
            m_seekFrame = m_curFrame;
        }
        m_restartCameras = false;
    }

    // Get next frame for master camera
    for (uint32_t i = 0; i < m_numCameras; ++i)
    {
        // ensure master camera is called first
        bool res = getNextFrameSingleCamera(static_cast<dwImageHandle_t*>(&m_rcbImage[i]), &m_imgGl[i], i);
        if (res == false)
        {
            std::this_thread::yield();
            onReset();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            m_restartCameras = true;
        }
    }
    m_curFrame = m_seekFrame;
    m_curFrame++;
    m_seekFrame = m_curFrame;
}

void DriveWorksSampleSeekMultiCamera::seekInit()
{
    for (uint32_t i = 0; i < m_numCameras; ++i)
    {
        dwImageProperties displayProperties = m_cameras[i]->getOutputProperties();
        displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;

        CHECK_DW_ERROR(dwImage_create(&m_imageRGBA[i], displayProperties, m_context));
        m_streamerCUDA2GL[i].reset(new SimpleImageStreamerGL<>(displayProperties, TIMEOUT, m_context));

        if (!m_cameras[i]->enableSeeking(m_numFramesPerCamera[i], m_videoStartTimestamps[i], m_videoEndTimestamps[i]))
        {
            logError("Seeking could not be enabled, seeking  functionality is disabled.");
            m_seekingEnabled = false;
        }
        // find the start timestamp and end timestamp of the overlapping section
        m_videoStartTimestamp = std::max(m_videoStartTimestamp, m_videoStartTimestamps[i]);
        m_videoEndTimestamp   = std::min(m_videoEndTimestamp, m_videoEndTimestamps[i]);
    }

    if (m_videoStartTimestamp >= m_videoEndTimestamp)
    {
        logError("No overlapping section among all the input cameras");
        stop();
    }

    // find the start frameIdx and end frameIdx of the main camera
    size_t videoStartFrameIdx{};
    m_cameras[MAIN_CAMERA_INDEX]->seekToTime(m_videoStartTimestamp);
    m_cameras[MAIN_CAMERA_INDEX]->getCurrFrameIdx(&videoStartFrameIdx);
    m_videoStartFrameIdx = static_cast<int64_t>(videoStartFrameIdx);

    size_t videoEndFrameIdx{};
    m_cameras[MAIN_CAMERA_INDEX]->seekToTime(m_videoEndTimestamp);
    m_cameras[MAIN_CAMERA_INDEX]->getCurrFrameIdx(&videoEndFrameIdx);
    m_videoEndFrameIdx = static_cast<int64_t>(videoEndFrameIdx);

    m_totFrameCount = m_videoEndFrameIdx - m_videoStartFrameIdx + 1;
    std::cout << "Number of frames: " << m_totFrameCount << std::endl;

    // get the frame offset for each camera w.r.t the the main camera frame index
    for (uint32_t i = 0; i < m_numCameras; ++i)
    {
        // get the
        size_t currVideoStartFrameIdx{};
        m_cameras[i]->seekToTime(m_videoStartTimestamp);
        m_cameras[i]->getCurrFrameIdx(&currVideoStartFrameIdx);
        m_frameOffset[i] = static_cast<int64_t>(currVideoStartFrameIdx) - m_videoStartFrameIdx;
        std::cout << "Camera " << i << ": " << m_frameOffset[i] << std::endl;
    }

    m_curFrame  = m_videoStartFrameIdx;
    m_seekFrame = m_videoStartFrameIdx;

    std::cout << "\n\n\n--- USAGE GUIDE ---\n";
    std::cout << "Space       : play / pause\n";
    std::cout << "Right arrow : seek forward\n";
    std::cout << "Left arrow  : seek backward\n";
    std::cout << "Up arrow    : increase seek delta\n";
    std::cout << "Down arrow  : decrease seek delta\n";
    std::cout << "Home        : sequence start\n";
    std::cout << "F           : seek to frame\n";
    std::cout << "V           : display frame num on screen\n";
    std::cout << "D           : show dense depth if applicable,\n";
    std::cout << "               [S | X | Z | C] for fisheye camera on/off toggling\n";
    std::cout << "               (front, rear, left, right respectively)\n";
    std::cout << "--------------------------------\n\n";
}

bool DriveWorksSampleSeekMultiCamera::getNextFrameSingleCamera(dwImageHandle_t* nextFrameOut, dwImageGL** nextFrameGL, uint32_t cameraIdx)
{
    if (m_seekingEnabled && (m_seekFrame != m_curFrame))
    {
        m_cameras[cameraIdx]->seekToFrame(m_seekFrame + m_frameOffset[cameraIdx]);
    }

    dwImageHandle_t nextFrame = m_cameras[cameraIdx]->readFrame();
    if (nextFrame == nullptr)
    {
        return false;
    }

    *nextFrameOut = nextFrame;
    CHECK_DW_ERROR(dwImage_copyConvert(m_imageRGBA[cameraIdx], nextFrame, m_context));
    dwImageHandle_t frameGL = m_streamerCUDA2GL[cameraIdx]->post(m_imageRGBA[cameraIdx]);
    dwImage_getGL(nextFrameGL, frameGL);
    return true;
}

} // namespace common
} // namespace dw_samples
