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

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <thread>

// SAMPLE framework
#include <framework/DriveWorksSample.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/SimpleCamera.hpp>
// TODO: deprecated
#include <framework/Checks.hpp>
#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif

#include <framework/SamplesDataPath.hpp>

// CORE
#include <dw/core/context/Context.h>
#include <dw/core/logger/Logger.h>
#include <dw/core/base/Version.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>

// IMAGE
#include <dw/interop/streamer/ImageStreamer.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Sample application
//------------------------------------------------------------------------------

class CameraSeekSample : public DriveWorksSample
{
public:
    CameraSeekSample(const ProgramArguments& args)
        : DriveWorksSample(args)
        , m_context(DW_NULL_HANDLE)
        , m_renderEngine(DW_NULL_HANDLE)
        , m_renderer(DW_NULL_HANDLE)
        , m_sal(DW_NULL_HANDLE)
        , m_startTimestamp(0)
        , m_endTimestamp(0)
        , m_frameCount(0)
        , m_seekMode(SEEKING_MODE_FRAME_EVENT)
        , m_frameEvent(0)
        , m_prevFrameEvent(0)
        , m_timestamp(0)
        , m_prevTimestamp(0)
        , m_forceProcessFrame(false)
    {
    }

    ~CameraSeekSample()
    {
    }

    typedef enum {
        SEEKING_MODE_FRAME_EVENT = 0,
        SEEKING_MODE_TIMESTAMP   = 1
    } SeekingMode;

private:
    dwContextHandle_t m_context;
    dwVisualizationContextHandle_t m_viz;
    dwRenderEngineHandle_t m_renderEngine;
    dwRendererHandle_t m_renderer;
    dwSALHandle_t m_sal;

    dwTime_t m_startTimestamp;
    dwTime_t m_endTimestamp;
    size_t m_frameCount;
    dwTime_t m_timeSkip     = 1000000;
    uint32_t m_minTimeSkip  = 100000;
    uint32_t m_maxTimeSkip  = 100 * 1000000L;
    uint32_t m_frameSkip    = 10;
    uint32_t m_minFrameSkip = 1;
    uint32_t m_maxFrameSkip = 1000;

    SeekingMode m_seekMode;
    size_t m_frameEvent;
    size_t m_prevFrameEvent;
    dwTime_t m_timestamp;
    dwTime_t m_prevTimestamp;

    bool m_forceProcessFrame;

    std::unique_ptr<SimpleCamera> m_camera;

public:
    bool onInitialize() override final
    {
        // Initialize DW context
        initializeDriveWorks(m_context);

        // Initialize renderer
        initRenderer();

        // Initialize SAL and camera
        initCamera();

        // Print usage information
        std::cout << "\n\n\n--- USAGE GUIDE ---\n";
        std::cout << "F/f         : switch to frame event seek mode (default)\n";
        std::cout << "T/t         : switch to timestamp seek mode\n";
        std::cout << "Space       : play / pause\n";
        std::cout << "Right arrow : seek forward\n";
        std::cout << "Left arrow  : seek backward\n";
        std::cout << "Up arrow    : increase seek delta\n";
        std::cout << "Down arrow  : decrease seek delta\n";
        std::cout << "--------------------------------\n\n";

        return true;
    }

    void onRelease() override final
    {
        // Release camera
        m_camera.reset();

        if (m_sal)
        {
            dwSAL_release(m_sal);
        }

        if (m_renderer)
        {
            dwRenderer_release(m_renderer);
        }

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    void onResizeWindow(int width, int height) override final
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, m_renderer);
    }

    // to enable pressing keys down for continuous seeking
    void onKeyRepeat(int key, int scancode, int mods) override final
    {
        onKeyDown(key, scancode, mods);
    }

    void onKeyDown(int key, int /*scancode*/, int /*mods*/) override final
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        switch (key)
        {
        case GLFW_KEY_T:
            if (m_seekMode != SEEKING_MODE_TIMESTAMP)
            {
                m_seekMode = SEEKING_MODE_TIMESTAMP;
                std::cout << "Switch to timestamp seek" << std::endl;
            }

            break;

        case GLFW_KEY_F:
            if (m_seekMode != SEEKING_MODE_FRAME_EVENT)
            {
                m_seekMode = SEEKING_MODE_FRAME_EVENT;
                std::cout << "Switch to frame event seek" << std::endl;
            }

            break;

        case GLFW_KEY_LEFT:
            if (m_seekMode == SEEKING_MODE_FRAME_EVENT)
            {
                if (m_frameEvent > m_frameSkip)
                    m_frameEvent -= m_frameSkip;
                else
                    m_frameEvent = 0;
            }
            else
            {
                if (m_timestamp > m_timeSkip + m_startTimestamp)
                    m_timestamp -= m_timeSkip;
                else
                    m_timestamp = m_startTimestamp;
            }

            m_forceProcessFrame = isPaused();

            break;

        case GLFW_KEY_RIGHT:
            if (m_seekMode == SEEKING_MODE_FRAME_EVENT)
            {
                if (m_frameEvent + m_frameSkip < m_frameCount)
                    m_frameEvent += m_frameSkip;
                else
                    m_frameEvent = 0;
            }
            else
            {
                if (m_timestamp + m_timeSkip < m_endTimestamp)
                    m_timestamp += m_timeSkip;
                else
                    m_timestamp = m_startTimestamp;
            }

            m_forceProcessFrame = isPaused();

            break;

        case GLFW_KEY_UP:
            if (m_seekMode == SEEKING_MODE_FRAME_EVENT)
            {
                if (m_frameSkip < m_maxFrameSkip)
                    m_frameSkip *= 10;
                else
                    std::cout << "MAX frame skip step (" << m_maxFrameSkip << ") reached!" << std::endl;
            }
            else
            {
                if (m_timeSkip < m_maxTimeSkip)
                    m_timeSkip *= 10;
                else
                    std::cout << "MAX timestamp skip step (" << m_maxTimeSkip << ") reached!" << std::endl;
            }

            break;

        case GLFW_KEY_DOWN:
            if (m_seekMode == SEEKING_MODE_FRAME_EVENT)
            {
                if (m_frameSkip > m_minFrameSkip)
                    m_frameSkip /= 10;
                else
                    std::cout << "MIN frame skip step (" << m_minFrameSkip << ") reached!" << std::endl;
            }
            else
            {
                if (m_timeSkip > m_minTimeSkip)
                    m_timeSkip /= 10;
                else
                    std::cout << "MIN timestamp skip step (" << m_minTimeSkip << ") reached!" << std::endl;
            }

            break;

        default:
            break;
        }
    }

    void onProcess() override final
    {
        // Seek if requested
        if (m_timestamp != m_prevTimestamp)
        {
            std::cout << "Seeking to timestamp: " << m_timestamp << std::endl;
            m_camera->seekToTime(m_timestamp);
        }
        else if (m_frameEvent != m_prevFrameEvent)
        {
            std::cout << "Seeking to frame event: " << m_frameEvent << std::endl;
            m_camera->seekToFrame(m_frameEvent);
        }

        dwImageHandle_t imageHandle = m_camera->readFrame();
        if (imageHandle == nullptr)
        {
            // End of stream reached, reseting sansor
            m_camera->resetCamera();
            m_frameEvent = 0;

            return;
        }

        dwImageHandle_t imageHandleGL = m_camera->getFrameRgbaGL();

        if (imageHandleGL != nullptr)
        {
            dwImage_getTimestamp(&m_timestamp, imageHandleGL);
            m_prevTimestamp = m_timestamp;

            m_frameEvent++;
            m_prevFrameEvent = m_frameEvent;
        }
    }

    void onRender()
    {
        // Processing one frame if seek requested on pause
        if (m_forceProcessFrame)
        {
            onProcess();
            m_forceProcessFrame = false;
        }

        // Render the frame if available
        dwImageHandle_t imageHandleGL = m_camera->getFrameRgbaGL();

        if (imageHandleGL != nullptr)
        {
            dwImageGL* imageGL;
            dwImage_getGL(&imageGL, imageHandleGL);
            CHECK_DW_ERROR(dwRenderer_renderTexture(imageGL->tex, imageGL->target, m_renderer));
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
        char tstep[50];
        sprintf(tstep, "Timestamp skip step: %ld (%0.1fs)", m_timeSkip, float(m_timeSkip) / 1000000.0f);
        char fstep[50];
        sprintf(fstep, "Frame skip step: %d", m_frameSkip);
        char mode[50];
        char loc[100];
        if (m_seekMode == SEEKING_MODE_FRAME_EVENT)
        {
            sprintf(mode, "Seek mode: FRAME");
            sprintf(loc, "Frame %ld/%ld (%0.1f%%)", m_frameEvent, m_frameCount, 100.0f * float(m_frameEvent) / float(m_frameCount));
        }
        else
        {
            sprintf(mode, "Seek mode: TIMESTAMP");
            sprintf(loc, "Timestamp %ld/%ld (%0.1f%%)", m_timestamp, m_endTimestamp, 100.0f * float(m_timestamp - m_startTimestamp) / float(m_endTimestamp - m_startTimestamp));
        }
        dwRenderer_renderText(10, getWindowHeight() - 20, tstep, m_renderer);
        dwRenderer_renderText(10, getWindowHeight() - 36, fstep, m_renderer);
        dwRenderer_renderText(10, getWindowHeight() - 52, mode, m_renderer);
        dwRenderer_renderText(10, getWindowHeight() - 68, loc, m_renderer);
    }

private:
    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks(dwContextHandle_t& context) const
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};

#ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
#endif

        CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
    }

    void initRenderer()
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        // init render engine with default params
        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        CHECK_DW_ERROR_MSG(dwRenderer_initialize(&m_renderer, m_viz),
                           "Cannot initialize Renderer, make sure GL context is available");
        dwRect rect;
        rect.width  = getWindowWidth();
        rect.height = getWindowHeight();
        rect.x      = 0;
        rect.y      = 0;

        dwRenderer_setRect(rect, m_renderer);
    }

    void initCamera()
    {
        CHECK_DW_ERROR_MSG(dwSAL_initialize(&m_sal, m_context),
                           "Cannot initialize SAL.");

        const std::string& paramStr = getArgs().parameterString();

        dwSensorParams params{};
        params.parameters = paramStr.c_str();
        params.protocol   = "camera.virtual";

        size_t pointPos = paramStr.find_last_of('.');
        if (pointPos == std::string::npos)
        {
            throw std::runtime_error("Unsupported video format. Expected raw/lraw/h264/h265/mp4");
        }
        std::string extension = paramStr.substr(pointPos + 1);

        m_camera.reset(new SimpleCamera(params, m_sal, m_context, DW_CAMERA_OUTPUT_NATIVE_PROCESSED));

        m_camera->enableGLOutput();

        if (!m_camera->enableSeeking(m_frameCount, m_startTimestamp, m_endTimestamp))
        {
            throw std::runtime_error("Could not enable seeking for camera.");
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    ProgramArguments arguments(argc, argv,
                               {ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str())});

    CameraSeekSample app(arguments);
    app.initializeWindow("Camera Seek Sample", 1280, 800, arguments.enabled("offscreen"));

    return app.run();
}
