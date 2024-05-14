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
// SPDX-FileCopyrightText: Copyright (c) 2016-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <dw/core/base/Version.h>

#include <framework/DriveWorksSample.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Log.hpp>

#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>
#include <dwvisualization/interop/ImageStreamer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>
#include <dw/imageprocessing/geometry/imagetransformation/ImageTransformation.h>
#include <dw/imageprocessing/filtering/ImageFilter.h>
#include <dw/imageprocessing/filtering/Threshold.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Image Transformation sample
// Sample demonstrates the following Image Processing functions:
// - Image Resize
// - TBA...
//------------------------------------------------------------------------------
class ImageTransformationSample : public DriveWorksSample
{
private:
    std::unique_ptr<ScreenshotHelper> m_screenshot;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwContextHandle_t m_context             = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz    = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine   = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                     = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer           = DW_NULL_HANDLE;
    dwSensorHandle_t m_camera               = DW_NULL_HANDLE;
    dwCameraFrameHandle_t m_frame           = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_image2GL      = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_image2GLSmall = DW_NULL_HANDLE;
    dwCameraProperties m_cameraProps        = {};

    dwImageProperties m_imgProperties = {};

    dwImageHandle_t m_imageRGBA = DW_NULL_HANDLE;

    dwImageHandle_t imageSmall         = DW_NULL_HANDLE;
    dwImageHandle_t imageSmallRGBA     = DW_NULL_HANDLE;
    dwImageHandle_t imageSmallR        = DW_NULL_HANDLE;
    dwImageHandle_t imageSmallR_binary = DW_NULL_HANDLE;
    dwImageHandle_t imageBlur          = DW_NULL_HANDLE;

    dwThresholdParameters m_thresholdParams{};

    uint32_t m_tile[2];

    uint32_t m_winSize = 3;
    bool m_updateBox   = false;
    dwBox2D m_roi      = {0, 0, 0, 0};
    dwBox2Df m_roiTemp = {0, 0, 0, 0};

    dwImageTransformationHandle_t m_imageTransformationEngine = DW_NULL_HANDLE;
    dwImageProcessingInterpolation m_interpolation            = DW_IMAGEPROCESSING_INTERPOLATION_DEFAULT;
    dwThresholdMode m_thresholdMode                           = DW_THRESHOLD_MODE_OTSU;

    dwImageFilterHandle_t filter = DW_NULL_HANDLE;

    bool m_doThreshold              = false;
    dwThresholdHandle_t m_threshold = DW_NULL_HANDLE;

public:
    ImageTransformationSample(const ProgramArguments& args)
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

            initRenderer();
        }

        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            std::string file = "video=" + getArgument("video");

            dwSensorParams sensorParams{};
            sensorParams.protocol = "camera.virtual";

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

        dwSensorCamera_getImageProperties(&m_imgProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera);

        dwImageProperties fromSmall = m_imgProperties;
        m_roi.x                     = 200;
        m_roi.y                     = 200;
        m_roi.width                 = m_imgProperties.width - 500;
        m_roi.height                = m_imgProperties.height - 500;

        m_roiTemp.x      = m_roi.x;
        m_roiTemp.y      = m_roi.x;
        m_roiTemp.width  = m_roi.width;
        m_roiTemp.height = m_roi.height;

        fromSmall.width  = 600;
        fromSmall.height = 600;
        CHECK_DW_ERROR(dwImage_create(&imageSmall, fromSmall, m_context));

        dwSensorCamera_getImageProperties(&m_imgProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera);

        //fromSmall.format = DW_IMAGE_FORMAT_RGBA_UINT8;
        CHECK_DW_ERROR(dwImage_create(&imageSmallRGBA, fromSmall, m_context));
        fromSmall.format = DW_IMAGE_FORMAT_R_UINT8;
        CHECK_DW_ERROR(dwImage_create(&imageSmallR, fromSmall, m_context));
        CHECK_DW_ERROR(dwImage_create(&imageSmallR_binary, fromSmall, m_context));
        dwSensorCamera_getImageProperties(&m_imgProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera);
        CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_image2GL, &m_imgProperties, DW_IMAGE_GL, m_context));
        CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_image2GLSmall, &fromSmall, DW_IMAGE_GL, m_context));

        dwImageTransformationParameters params{false};
        CHECK_DW_ERROR(dwImageTransformation_initialize(&m_imageTransformationEngine, params, m_context));

        dwImageFilterConfig filterConfig = {};
        filterConfig.filterType          = DW_IMAGEFILTER_TYPE_RECURSIVE_GAUSSIAN_FILTER;
        filterConfig.imageWidth          = fromSmall.width;
        filterConfig.imageHeight         = fromSmall.height;
        filterConfig.order               = 0;
        filterConfig.sigma               = static_cast<float32_t>(m_winSize) / 2;

        CHECK_DW_ERROR(dwImageFilter_initialize(&filter, &filterConfig, m_context));
        dwSensorCamera_getImageProperties(&m_imgProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera);

        CHECK_DW_ERROR(dwImage_create(&imageBlur, fromSmall, m_context));

        m_thresholdParams.mode                 = m_thresholdMode;
        m_thresholdParams.maxVal               = 255;
        m_thresholdParams.behavior             = DW_THRESHOLD_BEHAVIOR_BINARY;
        m_thresholdParams.manualThresholdValue = 200;

        m_thresholdParams.thresholdingImage = imageBlur;
        CHECK_DW_ERROR(dwThreshold_initialize(&m_threshold, m_thresholdParams, m_context));
        // -----------------------------
        // Start Sensors
        // -----------------------------
        dwSensor_start(m_camera);

        m_screenshot.reset(new ScreenshotHelper(m_context, m_sal, getWindowWidth(), getWindowHeight(), "ImageTransformationSample"));
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

        if (m_imageTransformationEngine)
            dwImageTransformation_release(m_imageTransformationEngine);

        if (imageSmall)
            dwImage_destroy(imageSmall);

        if (imageSmallRGBA)
            dwImage_destroy(imageSmallRGBA);

        if (filter)
            dwImageFilter_release(filter);

        if (m_frame)
            dwSensorCamera_returnFrame(&m_frame);

        if (m_threshold)
            dwThreshold_release(m_threshold);

        // stop sensor
        dwSensor_stop(m_camera);

        // release sensor
        dwSAL_releaseSensor(m_camera);

        m_screenshot.reset();

        // release renderer and streamer
        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        dwImageStreamerGL_release(m_image2GL);
        dwImageStreamerGL_release(m_image2GLSmall);

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

    bool initRenderer()
    {
        // init render engine with default params
        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        dwRenderEngineTileState paramList[2];
        for (uint32_t i = 0; i < 2; ++i)
        {
            dwRenderEngine_initTileState(&paramList[i]);
            paramList[i].modelViewMatrix = DW_IDENTITY_MATRIX4F;
            paramList[i].font            = DW_RENDER_ENGINE_FONT_VERDANA_24;
        }

        dwRenderEngine_addTilesByCount(m_tile, 2, 2, paramList, m_renderEngine);

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        CHECK_DW_ERROR(dwRenderEngine_reset(m_renderEngine));
        dwRectf rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));
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
            if (m_interpolation == DW_IMAGEPROCESSING_INTERPOLATION_LINEAR)
                m_interpolation = DW_IMAGEPROCESSING_INTERPOLATION_DEFAULT;
            else
                m_interpolation = DW_IMAGEPROCESSING_INTERPOLATION_LINEAR;
        }

        if (key == GLFW_KEY_T)
        {
            if (m_doThreshold == false)
            {
                m_doThreshold   = true;
                m_thresholdMode = DW_THRESHOLD_MODE_SIMPLE;
                std::cout << "Switched threshold to simple" << std::endl;
            }
            else
            {

                if (m_thresholdMode == DW_THRESHOLD_MODE_OTSU)
                {
                    m_thresholdMode = DW_THRESHOLD_MODE_PER_PIXEL;

                    m_thresholdParams.thresholdingImage = imageBlur;
                    std::cout << "Switched threshold to per pixel" << std::endl;
                }
                else if (m_thresholdMode == DW_THRESHOLD_MODE_PER_PIXEL)
                {
                    m_doThreshold = false;
                    std::cout << "Switched threshold off" << std::endl;
                }
                else if (m_thresholdMode == DW_THRESHOLD_MODE_SIMPLE)
                {
                    m_thresholdMode = DW_THRESHOLD_MODE_OTSU;
                    std::cout << "Switched threshold to Otsu" << std::endl;
                }
            }

            m_thresholdParams.mode = m_thresholdMode;
        }

        if (key == GLFW_KEY_UP)
        {
            m_winSize += 2;
        }

        if (key == GLFW_KEY_DOWN)
        {
            m_winSize -= 2;
            if (m_winSize < 3)
                m_winSize = 3;
        }
    }

    void onProcess() override
    {
        // return the previous frame to camera
        if (m_frame)
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));

        // ---------------------------
        // grab frame from camera
        // ---------------------------

        dwStatus status = dwSensorCamera_readFrame(&m_frame, 60000, m_camera);
        while (status != DW_SUCCESS)
        {

            if (status == DW_END_OF_STREAM)
            {
                dwSensor_reset(m_camera);
                log("Video reached end of stream.\n");
            }
            else if (status != DW_TIME_OUT && status != DW_NOT_READY)
            {
                CHECK_DW_ERROR(status);
            }

            status = dwSensorCamera_readFrame(&m_frame, 60000, m_camera);
        }

        CHECK_DW_ERROR(dwSensorCamera_getImage(&m_imageRGBA, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_frame));

        dwImageTransformation_setBorderMode(DW_IMAGEPROCESSING_BORDER_MODE_ZERO, m_imageTransformationEngine);
        CHECK_DW_ERROR(dwImageTransformation_setInterpolationMode(m_interpolation, m_imageTransformationEngine));

        // a roi of the image is scaled and format converter in one call
        // convert to R and scale in a single call
        CHECK_DW_ERROR(dwImageTransformation_copySubImage(imageSmallR, m_imageRGBA, m_roi, m_imageTransformationEngine));

        if (m_thresholdMode == DW_THRESHOLD_MODE_PER_PIXEL)
        {
            CHECK_DW_ERROR(dwImageFilter_applyFilter(imageBlur, imageSmallR, filter));
        }

        CHECK_DW_ERROR(dwThreshold_setThresholdParameters(m_thresholdParams, m_threshold));

        if (m_doThreshold)
            CHECK_DW_ERROR(dwThreshold_applyThreshold(imageSmallR_binary, imageSmallR, m_threshold));

        CHECK_DW_ERROR(dwSensorCamera_getImage(&m_imageRGBA, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_frame));
    }

    virtual void onMouseDown(int button, float x, float y, int /* mods*/) override
    {
        if (button == 0)
        {
            m_updateBox = true;

            m_roiTemp.x      = x * m_imgProperties.width / (getWindowWidth() / 2);
            m_roiTemp.y      = y * m_imgProperties.height / getWindowHeight();
            m_roiTemp.width  = 0;
            m_roiTemp.height = 0;
        }
    }

    virtual void onMouseMove(float x, float y) override
    {
        if (!m_updateBox)
            return;

        int32_t fx = x * m_imgProperties.width / getWindowWidth();
        int32_t fy = y * m_imgProperties.height / getWindowHeight();

        fx *= 2;

        m_roiTemp.width  = abs(fx - m_roiTemp.x);
        m_roiTemp.height = abs(fy - m_roiTemp.y);
        if (m_roiTemp.x > fx)
            m_roiTemp.x = fx;
        if (m_roiTemp.y > fy)
            m_roiTemp.y = fy;

        // if the converted result will be 0, we can't use the ROI
        if ((int32_t)m_roiTemp.width != 0 && (int32_t)m_roiTemp.height != 0)
        {
            m_roi.x      = m_roiTemp.x;
            m_roi.y      = m_roiTemp.y;
            m_roi.width  = m_roiTemp.width;
            m_roi.height = m_roiTemp.height;
        }
    }

    virtual void onMouseUp(int button, float /* x*/, float /* y*/, int /* mods*/) override
    {
        if (button == 0)
        {
            m_updateBox = false;
        }
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
            CHECK_DW_ERROR(dwImageStreamerGL_producerSend(m_imageRGBA, m_image2GL));

            dwImageHandle_t frameGL;
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&frameGL, 33000, m_image2GL));

            dwImageGL* imageGL = nullptr;
            CHECK_DW_ERROR(dwImage_getGL(&imageGL, frameGL));

            CHECK_DW_ERROR(dwRenderEngine_setTile(m_tile[0], m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

            dwVector2f range{};
            range.x = m_imgProperties.width;
            range.y = m_imgProperties.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL, {0, 0, range.x, range.y}, m_renderEngine));

            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&frameGL, m_image2GL));
            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 33000, m_image2GL));

            CHECK_DW_ERROR(dwRenderEngine_setColor({1.f, 0.f, 0.f, 1.f}, m_renderEngine));

            CHECK_DW_ERROR(dwRenderEngine_render(
                DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D,
                &m_roiTemp, sizeof(dwBox2Df), 0,
                1, m_renderEngine));

            std::string tileString = std::to_string(m_imgProperties.width) + std::string("x") +
                                     std::to_string(m_imgProperties.height) + std::string(" Click and drag to select ROI");
            CHECK_DW_ERROR(dwRenderEngine_renderText2D(tileString.c_str(), {30, 50}, m_renderEngine));
        }

        if (imageSmallRGBA)
        {
            if (m_doThreshold)
            {
                CHECK_DW_ERROR(dwImageStreamerGL_producerSend(imageSmallR_binary, m_image2GLSmall));
            }
            else
            {
                CHECK_DW_ERROR(dwImageStreamerGL_producerSend(imageSmallR, m_image2GLSmall));
            }

            dwImageHandle_t frameGL;
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&frameGL, 33000, m_image2GLSmall));

            dwImageGL* imageGL = nullptr;
            CHECK_DW_ERROR(dwImage_getGL(&imageGL, frameGL));

            CHECK_DW_ERROR(dwRenderEngine_setTile(m_tile[1], m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

            dwVector2f range{};
            range.x = m_imgProperties.width;
            range.y = m_imgProperties.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            dwVector2f rangeim{};
            rangeim.x = imageGL->prop.width;
            rangeim.y = imageGL->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL, {0, 0, rangeim.x, rangeim.y}, m_renderEngine));

            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&frameGL, m_image2GLSmall));
            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 33000, m_image2GLSmall));

            std::string tileString = std::to_string(imageGL->prop.width) + std::string("x") +
                                     std::to_string(imageGL->prop.height);
            ;
            CHECK_DW_ERROR(dwRenderEngine_renderText2D(tileString.c_str(), {30, 30}, m_renderEngine));
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());

        // screenshot if required
        m_screenshot->processScreenshotTrig();
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
                          },
                          "imagetransformation sample.");

    // -------------------
    // initialize and start a window application
    ImageTransformationSample app(args);

    app.initializeWindow("Image Transformation sample", 1280, 800 / 2, args.enabled("offscreen"));

    return app.run();
}
