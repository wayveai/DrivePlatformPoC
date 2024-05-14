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
// SPDX-FileCopyrightText: Copyright (c) 2020-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "DriveWorksSampleSeek.hpp"

namespace dw_samples
{
namespace common
{
bool DriveWorksSampleSeek::onInitialize()
{
    bool success = derivedInit();
    seekInit();
    return success;
}

void DriveWorksSampleSeek::onProcess()
{
    seekProcess();
    derivedProcess();
}

/**
 * @brief onRender Called in the main loop even when paused.
 * The method is not executed for non-window applications.
 */
void DriveWorksSampleSeek::onRender()
{
    seekRender();
    derivedRender();
    if (m_displayFrameNum)
    {
        displayFrameInfo();
    }
    renderutils::renderFPS(m_renderEngine, getCurrentFPS());
}

void DriveWorksSampleSeek::seekRender()
{
    // If video is paused and seeking occurs, reprocess the frame for detections
    if (m_seekingEnabled && m_forceProcessFrame)
    {
        onProcess();
        m_forceProcessFrame = false;
    }
}

void DriveWorksSampleSeek::seekProcess()
{
    // read from camera
    DriveWorksSampleSeek::getNextFrame(static_cast<dwImageHandle_t*>(&m_rcbImage), &m_imgGl);
    std::this_thread::yield();
    while (m_rcbImage == nullptr)
    {
        onReset();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DriveWorksSampleSeek::getNextFrame(static_cast<dwImageHandle_t*>(&m_rcbImage), &m_imgGl);
    }
}

void DriveWorksSampleSeek::seekInit()
{
    dwImageProperties displayProperties = m_camera->getOutputProperties();
    displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;

    CHECK_DW_ERROR(dwImage_create(&m_imageRGBA, displayProperties, m_context));
    m_streamerCUDA2GL.reset(new SimpleImageStreamerGL<>(displayProperties, TIMEOUT, m_context));

    if (!m_camera->enableSeeking(m_totFrameCount, m_videoStartTimestamp, m_videoEndTimestamp))
    {
        logError("Seeking could not be enabled, seeking  functionality is disabled.");
        m_seekingEnabled = false;
    }

    m_videoStartFrameIdx = 0;
    m_videoEndFrameIdx   = static_cast<int64_t>(m_totFrameCount - 1);

    std::cout << "\n\n\n--- USAGE GUIDE ---\n";
    std::cout << "Space       : play / pause\n";
    std::cout << "Right arrow : seek forward\n";
    std::cout << "Left arrow  : seek backward\n";
    std::cout << "Up arrow    : increase seek delta\n";
    std::cout << "Down arrow  : decrease seek delta\n";
    std::cout << "Home        : sequence start\n";
    std::cout << "F           : seek to frame\n";
    std::cout << "V           : display frame num on screen\n";
    std::cout << "--------------------------------\n\n";
}

void DriveWorksSampleSeek::displayFrameInfo()
{
    char frameSkipStr[50];
    sprintf(frameSkipStr, "Frame skip step: %d", m_frameSkipNumber);
    char frameInfoStr[100];
    sprintf(frameInfoStr, "Frame %ld/%ld (%0.1f%%)", m_seekFrame, m_totFrameCount, 100.0f * float(m_seekFrame) / float(m_totFrameCount));
    const float32_t offsetFromPreviousText = 0.015f;                          // arbitrary number picked to not overlap with FPS rendering
    const float32_t offsetToNotStackText   = offsetFromPreviousText + 0.015f; // offset to not collide text rendering
    CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
    dwVector2f frameSkipPosition{0.0f, 1.0f - offsetFromPreviousText};
    dwVector2f frameInfoPosition{0.0f, 1.0f - offsetToNotStackText};
    CHECK_DW_ERROR(dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_12, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_renderText2D(frameSkipStr, frameSkipPosition, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_renderText2D(frameInfoStr, frameInfoPosition, m_renderEngine));
}

void DriveWorksSampleSeek::getNextFrame(dwImageHandle_t* nextFrameOut, dwImageGL** nextFrameGL)
{
    if (m_seekingEnabled && (m_seekFrame != m_curFrame))
    {
        m_camera->seekToFrame(m_seekFrame);
        m_curFrame = m_seekFrame;
    }
    else if (!m_seekingEnabled || static_cast<size_t>(m_curFrame) != m_totFrameCount)
    {
        m_curFrame++;
    }

    dwImageHandle_t nextFrame = m_camera->readFrame();
    if (nextFrame == nullptr)
    {
        if (m_seekingEnabled)
        {
            m_seekFrame = m_curFrame;
        }
        else
        {
            stop();
            m_camera->resetCamera();
        }
    }
    else
    {
        *nextFrameOut = nextFrame;
        CHECK_DW_ERROR(dwImage_copyConvert(m_imageRGBA, nextFrame, m_context));
        dwImageHandle_t frameGL = m_streamerCUDA2GL->post(m_imageRGBA);
        dwImage_getGL(nextFrameGL, frameGL);
    }
    m_seekFrame = m_curFrame;
}

void DriveWorksSampleSeek::onKeyRepeat(int key, int scancode, int mods)
{
    seekOnKeyRepeat(key, scancode, mods);
    derivedOnKeyRepeat(key, scancode, mods);
}

void DriveWorksSampleSeek::onKeyDown(int key, int /*scancode*/, int /*mods*/)
{
    seekOnKeyDown(key);
    derivedOnKeyDown(key);
}

// to enable pressing keys down for continuous seeking
void DriveWorksSampleSeek::seekOnKeyRepeat(int key, int scancode, int mods)
{
    (void)scancode;
    (void)mods;
    seekOnKeyDown(key);
}

void DriveWorksSampleSeek::seekOnKeyDown(int key)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (m_seekingEnabled)
    {
        bool seek{false};
        bool update_skip{false};
        if (m_enterSeekToFrameMode)
        {
            if (key < GLFW_KEY_0 or key > GLFW_KEY_9)
            {
                m_seekFrame            = m_seekToFrame;
                m_enterSeekToFrameMode = false;
                seek                   = true;
            }
            else
            {
                m_seekToFrame = 10 * m_seekToFrame + (key - GLFW_KEY_0);
                std::cout << "Seek to frame: " << m_seekToFrame << std::endl;
            }
        }
        else
        {
            switch (key)
            {
            case GLFW_KEY_HOME:
            {
                m_seekFrame = m_videoStartFrameIdx;
                seek        = true;
                break;
            }
            case GLFW_KEY_LEFT:
            {
                m_seekFrame -= m_frameSkipNumber;
                seek = true;
                break;
            }
            case GLFW_KEY_RIGHT:
            {
                m_seekFrame += m_frameSkipNumber;
                seek = true;
                break;
            }
            case GLFW_KEY_UP:
            {
                m_frameSkipNumber = std::min(m_frameSkipNumber * 10, MAX_FRAME_SKIP_NUMBER);
                update_skip       = true;
                break;
            }
            case GLFW_KEY_DOWN:
            {
                m_frameSkipNumber = std::max(m_frameSkipNumber / 10, MIN_FRAME_SKIP_NUMBER);
                update_skip       = true;
                break;
            }
            case GLFW_KEY_F:
            {
                std::cout << "Enter frame to seek to.\n";
                m_enterSeekToFrameMode = true;
                m_seekToFrame          = m_videoStartFrameIdx;
                break;
            }
            case GLFW_KEY_V:
            {
                m_displayFrameNum = !m_displayFrameNum;
                break;
            }
            default:
                break;
            }
        }
        if (update_skip)
        {
            std::cout << "Current seek step " << m_frameSkipNumber << std::endl;
        }
        if (seek)
        {
            m_forceProcessFrame = isPaused();
            m_seekFrame         = std::max(m_seekFrame, m_videoStartFrameIdx);
            m_seekFrame         = std::min(m_seekFrame, m_videoEndFrameIdx);
        }
    }
}

} // namespace common
} // namespace dw_samples
