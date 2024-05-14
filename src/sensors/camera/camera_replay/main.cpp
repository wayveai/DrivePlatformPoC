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
// SPDX-FileCopyrightText: Copyright (c) 2016-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <framework/DriveWorksSample.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Log.hpp>
#include <dw/core/base/Version.h>

#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>
#include <dwvisualization/interop/ImageStreamer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Camera replay samples
// The Video replay sample demonstrates H.264/H.265 playback using a hardware decoder and .RAW/.LRAW using
// the Tegra ISP
//
// The sample opens a window to play back the provided video file. The playback
// does not support any container formats (MP4 or similar); a pure H.264 stream is
// required.
//------------------------------------------------------------------------------
class CameraReplay : public DriveWorksSample
{
private:
    std::unique_ptr<ScreenshotHelper> m_screenshot;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                   = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer         = DW_NULL_HANDLE;
    dwSensorHandle_t m_camera             = DW_NULL_HANDLE;
    dwCameraFrameHandle_t m_frame         = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_image2GL    = DW_NULL_HANDLE;
    dwCameraProperties m_cameraProps      = {};
    dwImageProperties m_cameraImageProps  = {};

    dwImageHandle_t m_imageRGBA = DW_NULL_HANDLE;

    dwImageStreamerHandle_t m_streamerToCPUGrab = DW_NULL_HANDLE;
    bool m_frameGrab                            = false;

    enum RecordFileFormat
    {
        RECORD_FILE_FORMAT_H264,
        RECORD_FILE_FORMAT_H265,
        RECORD_FILE_FORMAT_MP4,
        RECORD_FILE_FORMAT_LRAW,
        RECORD_FILE_FORMAT_RAW,
        RECORD_FILE_FORMAT_POD,
        RECORD_FILE_FORMAT_UNSUPPORTED,
    };

    void getRecordFileFormat(RecordFileFormat& fileformat, std::string params)
    {
        if (params.compare("h264") == 0)
        {
            fileformat = RECORD_FILE_FORMAT_H264;
        }
        else if (params.compare("h265") == 0)
        {
            fileformat = RECORD_FILE_FORMAT_H265;
        }
        else if (params.compare("raw") == 0)
        {
            fileformat = RECORD_FILE_FORMAT_RAW;
        }
        else if (params.compare("lraw") == 0)
        {
            fileformat = RECORD_FILE_FORMAT_LRAW;
        }
        else if (params.compare("mp4") == 0)
        {
            fileformat = RECORD_FILE_FORMAT_MP4;
        }
        else if (params.compare("pod") == 0)
        {
            fileformat = RECORD_FILE_FORMAT_POD;
        }
        else
        {
            fileformat = RECORD_FILE_FORMAT_UNSUPPORTED;
        }
    }

    void setFileFormat(std::string params)
    {

        RecordFileFormat fileFormat;
        getRecordFileFormat(fileFormat, params);

        if (fileFormat == RECORD_FILE_FORMAT_H264)
        {
            params += ",format=h264";
        }
        else if (fileFormat == RECORD_FILE_FORMAT_H265)
        {
            params += ",format=h265";
        }
        else if (fileFormat == RECORD_FILE_FORMAT_RAW)
        {
            params += ",format=raw";
        }
        else if (fileFormat == RECORD_FILE_FORMAT_POD)
        {
            params += ",format=pod";
        }
        else if (fileFormat == RECORD_FILE_FORMAT_LRAW)
        {
            params += ",format=lraw";
        }
        else if (fileFormat == RECORD_FILE_FORMAT_MP4)
        {
            params += ",format=mp4";
        }
        else
        {
            std::cout << "setFileFormat: Incorrect format of sensor params \n";
        }
    }

public:
    CameraReplay(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);
            dwSAL_initialize(&m_sal, m_context);
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            dwVisualizationInitialize(&m_viz, m_context);

            // init render engine with default params
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

            CHECK_DW_ERROR_MSG(dwRenderer_initialize(&m_renderer, m_viz),
                               "Cannot initialize Renderer, maybe no GL context available?");
            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, m_renderer);
        }

        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            std::string file = "video=" + getArgument("video");
            dwSensorParams sensorParams{};
            sensorParams.protocol = "camera.virtual";
            if ((std::stoi(getArgument("newcodecstack")) > 0))
            {
                setFileFormat(file);
                file += ",newcodecstack=1";
                sensorParams.protocol = "camera.virtual.new";
                std::cout << "Using new codec stack." << std::endl;
            }

            sensorParams.parameters = file.c_str();
            CHECK_DW_ERROR_MSG(dwSAL_createSensor(&m_camera, sensorParams, m_sal),
                               "Cannot create virtual camera sensor, maybe wrong video file?");

            dwSensorCamera_getSensorProperties(&m_cameraProps, m_camera);
            printf("Camera image with %dx%d at %f FPS\n", m_cameraProps.resolution.x,
                   m_cameraProps.resolution.y, m_cameraProps.framerate);

            // we would like the application run as fast as the original video
            setProcessRate(m_cameraProps.framerate);
        }

        // -----------------------------
        // initialize streamer and software isp for raw video playback, if the video input is raw
        // -----------------------------

        dwImageProperties from{};

        CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&from, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera));

        CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_image2GL, &from, DW_IMAGE_GL, m_context));

        CHECK_DW_ERROR(dwImageStreamer_initialize(&m_streamerToCPUGrab, &from, DW_IMAGE_CPU, m_context));

        // -----------------------------
        // Start Sensors
        // -----------------------------
        CHECK_DW_ERROR(dwSensor_start(m_camera));

        m_screenshot.reset(new ScreenshotHelper(m_context, m_sal, getWindowWidth(), getWindowHeight(), "CameraReplay"));

        std::cout << "Initialization complete." << std::endl;
        return true;
    }

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

    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {

        if (m_frame)
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));

        dwSensor_reset(m_camera);
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if (m_frame)
            dwSensorCamera_returnFrame(&m_frame);

        // stop sensor
        dwSensor_stop(m_camera);

        m_screenshot.reset();

        if (m_streamerToCPUGrab)
        {
            dwImageStreamer_release(m_streamerToCPUGrab);
        }

        // release sensor
        dwSAL_releaseSensor(m_camera);

        // release renderer and streamer
        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        dwRenderer_release(m_renderer);
        dwImageStreamerGL_release(m_image2GL);

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        {
            dwSAL_release(m_sal);
            dwVisualizationRelease(m_viz);
            CHECK_DW_ERROR(dwRelease(m_context));
            CHECK_DW_ERROR(dwLogger_release());
        }
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, m_renderer);
    }

    void onKeyDown(int key, int scancode, int mods) override
    {
        (void)scancode;
        (void)mods;

        if (key == GLFW_KEY_S)
        {
            m_screenshot->triggerScreenshot();
        }

        if (key == GLFW_KEY_F)
        {
            m_frameGrab = true;
        }
    }

    void onProcess() override
    {
        // return the previous frame to camera
        if (m_frame)
        {
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));
        }

        // ---------------------------
        // grab frame from camera
        // ---------------------------
        uint32_t countFailure = 0;
        dwStatus status       = DW_NOT_READY;

        while ((status == DW_NOT_READY) || (status == DW_END_OF_STREAM) || (status == DW_TIME_OUT))
        {
            status = dwSensorCamera_readFrame(&m_frame, 600000, m_camera);
            countFailure++;
            if (countFailure == 1000000)
            {
                std::cout << "Camera virtual doesn't seem responsive, exit loop and stopping the sample" << std::endl;
                stop();
                return;
            }

            if (status == DW_END_OF_STREAM)
            {
                std::cout << "Video reached end of stream" << std::endl;
                CHECK_DW_ERROR(dwSensor_reset(m_camera));
            }
            else if ((status != DW_TIME_OUT) && (status != DW_NOT_READY))
            {
                CHECK_DW_ERROR(status);
            }
        }

        CHECK_DW_ERROR(dwSensorCamera_getImage(&m_imageRGBA, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_frame));
    }

    ///------------------------------------------------------------------------------
    /// Rendering
    ///     - push the RGBA cuda image through the streamer to convert it into GL
    ///     - render frame on screen
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        if (m_imageRGBA)
        {
            if (m_frameGrab)
            {
                frameGrab(m_imageRGBA);
                m_frameGrab = false;
            }

            CHECK_DW_ERROR(dwImageStreamerGL_producerSend(m_imageRGBA, m_image2GL));

            dwImageHandle_t frameGL;
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&frameGL, 33000, m_image2GL));

            dwImageGL* imageGL = nullptr;
            CHECK_DW_ERROR(dwImage_getGL(&imageGL, frameGL));

            char stime[64];
            sprintf(stime, "Frame time: %lu [us]", imageGL->timestamp_us);

            dwRenderer_renderTexture(imageGL->tex, imageGL->target, m_renderer);
            dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer);
            dwRenderer_renderText(10, 10, stime, m_renderer);

            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&frameGL, m_image2GL));
            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 33000, m_image2GL));
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());

        // screenshot if required
        m_screenshot->processScreenshotTrig();
    }

    ///------------------------------------------------------------------------------
    /// Image stream frame grabber.
    ///     - read from camera
    ///     - get an image with a useful format
    ///     - save image to file
    ///------------------------------------------------------------------------------
    void frameGrab(dwImageHandle_t frameCUDA)
    {
        dwTime_t timeout = 500000;

        // stream that image to the CPU domain
        CHECK_DW_ERROR(dwImageStreamer_producerSend(frameCUDA, m_streamerToCPUGrab));

        // receive the streamed image as a handle
        dwImageHandle_t frameCPU;
        CHECK_DW_ERROR(dwImageStreamer_consumerReceive(&frameCPU, timeout, m_streamerToCPUGrab));

        // get an image from the frame
        dwImageCPU* imgCPU;
        CHECK_DW_ERROR(dwImage_getCPU(&imgCPU, frameCPU));

        // write the image to a file
        char fname[128];
        dwTime_t timestamp;
        dwImage_getTimestamp(&timestamp, frameCPU);
        if (timestamp == 0)
        {
            static int32_t screenshotCount = 0;
            timestamp                      = screenshotCount++;
        }
        sprintf(fname, "framegrab_%s.png", std::to_string(timestamp).c_str());
        uint32_t error = lodepng_encode32_file(fname, imgCPU->data[0], imgCPU->prop.width, imgCPU->prop.height);
        std::cout << "Frame Grab saved to " << fname << " " << error << "\n";

        // reset frame grab flag
        // returned the consumed image
        CHECK_DW_ERROR(dwImageStreamer_consumerReturn(&frameCPU, m_streamerToCPUGrab));

        // notify the producer that the work is done
        CHECK_DW_ERROR(dwImageStreamer_producerReturn(nullptr, timeout, m_streamerToCPUGrab));
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("video", (SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
                              ProgramArguments::Option_t("newcodecstack", "0", "If this set to 1, New Codec Stack is used for decode/reading camera frames\n"),
                          },
                          "Camera replay sample which playback .h264/.h265/.mp4/.raw/.lraw video streams in a GL window.");

    // -------------------
    // initialize and start a window application
    CameraReplay app(args);

    app.initializeWindow("Camera Replay Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
