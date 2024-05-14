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
// SPDX-FileCopyrightText: Copyright (c) 2015-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <dw/core/base/Version.h>
#include <dw/core/context/Context.h>

#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/sensors/SensorSerializer.h>

#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>

#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/image/FrameCapture.h>

#include <lodepng.h>

using namespace dw_samples::common;

class CameraUSBSample : public DriveWorksSample
{
private:
    const uint32_t MAX_CAPTURE_MODES = 1024u;

    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                   = DW_NULL_HANDLE;
    dwSensorHandle_t m_camera             = DW_NULL_HANDLE;
    dwImageHandle_t m_cameraImage         = DW_NULL_HANDLE;
    dwImageHandle_t m_imageGL             = DW_NULL_HANDLE;

    dwImageStreamerHandle_t m_streamerGL = DW_NULL_HANDLE;
    dwCameraFrameHandle_t m_frame        = DW_NULL_HANDLE;

    // DW_IMAGE_CUDA for camera.usb
    dwImageType m_cameraImageType = DW_IMAGE_CUDA;

#ifndef VIBRANTE
    bool m_shouldRecord                     = false;
    dwSensorSerializerHandle_t m_serializer = DW_NULL_HANDLE;
    dwFrameCaptureHandle_t m_frameCapture   = DW_NULL_HANDLE;
#endif

    std::unique_ptr<ScreenshotHelper> m_screenshot;

public:
    CameraUSBSample(const ProgramArguments& args)
        : DriveWorksSample(args)
    {
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

    bool onInitialize() override
    {
        {
            initializeDriveWorks(m_context);
            CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));
        }

        dwImageProperties cameraImageProps{};
        {
            // initialize the sensor
            {
                dwSensorParams params{};
                params.protocol        = "camera.usb";
                std::string parameters = "device=" + getArgument("device");

                const std::string& modeParam = getArgument("mode");
                if (!modeParam.empty())
                {
                    parameters += ",mode=" + modeParam;
                }

                params.parameters = parameters.c_str();

                CHECK_DW_ERROR(dwSAL_createSensor(&m_camera, params, m_sal));
            }

            // Log available modes capture modes
            {
                uint32_t numModes = 0;
                CHECK_DW_ERROR(dwSensorCamera_getNumSupportedCaptureModes(&numModes, m_camera));

                if (numModes > 1)
                {
                    dwCameraProperties properties{};
                    CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&properties, m_camera));

                    for (uint32_t modeIdx = 0; modeIdx < numModes; ++modeIdx)
                    {
                        dwCameraProperties mode{};
                        CHECK_DW_ERROR(dwSensorCamera_getSupportedCaptureMode(&mode, modeIdx, m_camera));

                        const char* msgEnd = (mode.framerate == properties.framerate &&
                                              mode.resolution.x == properties.resolution.x &&
                                              mode.resolution.y == properties.resolution.y)
                                                 ? " fps (*)"
                                                 : " fps";

                        std::cout << "Mode " << modeIdx << ": " << mode.resolution.x << "x" << mode.resolution.y << " " << mode.framerate << msgEnd << std::endl;
                    }
                }
            }

            // Retrieve camera dimensions
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&cameraImageProps, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_camera));

            m_cameraImageType = cameraImageProps.type;

            if (m_cameraImageType == DW_IMAGE_CUDA)
            {
                cameraImageProps.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            }

            std::cout << "Camera image with " << cameraImageProps.width << "x" << cameraImageProps.height << std::endl;

            // sets the window size now that we know what the camera dimensions are.
            setWindowSize(cameraImageProps.width, cameraImageProps.height);

            // instantiation of an image streamer that can pass images to OpenGL.
            CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_streamerGL, &cameraImageProps, DW_IMAGE_GL, m_context));

            // start sensor
            CHECK_DW_ERROR(dwSensor_start(m_camera));
        }

#ifndef VIBRANTE
        {
            // Initialize the serializer
            m_shouldRecord = !getArgument("record-file").empty();
            if (m_shouldRecord)
            {
                dwSerializerParams params{};
                std::string parameterString = std::string("type=disk,file=") + getArgument("record-file");

                params.parameters = parameterString.c_str();
                CHECK_DW_ERROR(dwSensorSerializer_initialize(&m_serializer, &params, m_camera));

                dwFrameCaptureParams frameParams{};

                std::string paramsFrameCap = "type=disk";
                paramsFrameCap += ",format=h264";
                paramsFrameCap += ",file=" + getArgument("record-file");
                frameParams.params.parameters = paramsFrameCap.c_str();
                frameParams.mode              = DW_FRAMECAPTURE_MODE_SERIALIZE;
                dwImageProperties cameraImageProps;
                CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&cameraImageProps, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_camera));

                frameParams.width  = cameraImageProps.width;
                frameParams.height = cameraImageProps.height;

                CHECK_DW_ERROR(dwFrameCapture_initialize(&m_frameCapture, &frameParams, m_sal, m_context));
            }
        }
#endif

        {
            // init render engine with default params
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));
        }

        m_screenshot.reset(new ScreenshotHelper(m_context, m_sal, getWindowWidth(), getWindowHeight(), "CameraUSB"));

        return true;
    }

    void onProcess() override
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // webcams can deliver as low as 20 fps so we need a timeout high enough

        const dwStatus result = dwSensorCamera_readFrame(&m_frame, 50000, m_camera);
        if (DW_TIME_OUT == result)
        {
            return;
        }
        else if (DW_NOT_AVAILABLE == result)
        {
            std::cerr << "Camera is not running or not found" << std::endl;
            onRelease();
        }
        else if (DW_SUCCESS != result)
        {
            std::cerr << "Cannot get frame from the camera: " << dwGetStatusName(result) << std::endl;
            onRelease();
        }

        dwCameraOutputType outputType = m_cameraImageType == DW_IMAGE_CUDA ? DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8 : DW_CAMERA_OUTPUT_NATIVE_PROCESSED;
        CHECK_DW_ERROR(dwSensorCamera_getImage(&m_cameraImage, outputType, m_frame));

#ifndef VIBRANTE
        if (m_shouldRecord)
        {
            CHECK_DW_ERROR(dwFrameCapture_appendFrame(m_cameraImage, m_frameCapture));
        }
#endif
    }

    void onRender() override
    {
        if (m_cameraImage != nullptr)
        {
            // sends the camera image on the stream
            CHECK_DW_ERROR(dwImageStreamerGL_producerSend(m_cameraImage, m_streamerGL));

            // and waits for the GL image to come out.
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&m_imageGL, 30000, m_streamerGL));

            if (m_imageGL)
            {
                CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

                dwImageGL* frameGL = nullptr;
                CHECK_DW_ERROR(dwImage_getGL(&frameGL, m_imageGL));

                dwVector2f range{};
                range.x = frameGL->prop.width;
                range.y = frameGL->prop.height;
                CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderImage2D(frameGL, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));
            }

            // returning the GL image to its stream.
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&m_imageGL, m_streamerGL));

            // and waiting for the camera image to be returned to us.
            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 32000, m_streamerGL));
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());

        // return frame
        if (m_frame)
        {
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));
        }

        // screenshot if required
        m_screenshot->processScreenshotTrig();
    }

    void onRelease() override
    {
        m_screenshot.reset(nullptr);
#ifndef VIBRANTE
        if (m_serializer != DW_NULL_HANDLE)
            dwSensorSerializer_release(m_serializer);
#endif

        CHECK_DW_ERROR(dwSensor_stop(m_camera));

        if (m_streamerGL != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwImageStreamerGL_release(m_streamerGL));
        }

#ifndef VIBRANTE
        if (m_frameCapture)
        {
            dwFrameCapture_release(m_frameCapture);
        }
#endif
        CHECK_DW_ERROR(dwSAL_releaseSensor(m_camera));

        CHECK_DW_ERROR(dwSAL_release(m_sal));

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    void onResizeWindow(int width, int height) override
    {
        {
            CHECK_DW_ERROR(dwRenderEngine_reset(m_renderEngine));
            dwRectf rect;
            rect.width  = width;
            rect.height = height;
            rect.x      = 0;
            rect.y      = 0;
            CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));
        }
    }

    void onKeyUp(int key, int /*scancode*/, int /*mods*/) override
    {
        // take screenshot
        if (key == GLFW_KEY_S && m_screenshot)
            m_screenshot->triggerScreenshot();
    }
};

//#######################################################################################
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
#ifdef VIBRANTE_V5L
                              // on Vibrante systems the device for the camera is mostly 1
                              ProgramArguments::Option_t("device", "1"),
#else
                              // Only allow serialization on desktop
                              ProgramArguments::Option_t("record-file", ""),
                              ProgramArguments::Option_t("device", "0"),
#endif
                              ProgramArguments::Option_t("mode", "0")},
                          "Camera USB");

    CameraUSBSample app(args);

    app.initializeWindow("Camera USB Sample", 1280, 800, false);

    return app.run();
}
