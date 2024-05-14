/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2019-2022 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

// Samples
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/WindowGLFW.hpp>

// Core

#include <dw/core/logger/Logger.h>
#include <dw/core/base/VersionCurrent.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>

// Pyramid
#include <dw/imageprocessing/pyramid/Pyramid.h>
#ifdef DW_SDK_BUILD_PVA
#include <dw/imageprocessing/pyramid/PyramidPVA.h>
#include "cupva_host_wrapper.h"
#if VIBRANTE_PDK_DECIMAL >= 6000400
#include "cupva_cuda_wrapper.h"
#endif
#endif

// Renderer
#include <dwvisualization/core/RenderEngine.h>

using namespace dw_samples::common;

class PyramidModuleSample : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Driveworks Context, SAL and render engine
    // ------------------------------------------------
    dwContextHandle_t m_context          = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                  = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz = DW_NULL_HANDLE;

    dwCameraProperties m_cameraProps     = {};
    dwImageProperties m_cameraImageProps = {};

// ------------------------------------------------
// Pyramid
// ------------------------------------------------
#ifdef DW_SDK_BUILD_PVA
    cupvaStream_t m_cupvaStream;
    cudaStream_t m_stream;
    dwPyramidPVAHandle_t m_pyramidModule = DW_NULL_HANDLE;
    dwPyramidPVAParams m_imagePyramidParams{};
#endif
    dwPyramidImage m_pyramidOutput;

    // ------------------------------------------------
    // Renderer
    // ------------------------------------------------
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    std::unique_ptr<SimpleImageStreamerGL<>> streamerCUDA2GL;
    dwImageHandle_t m_imgGl;
    dwImageHandle_t frameGL;

    // ------------------------------------------------
    // Camera
    // ------------------------------------------------
    std::unique_ptr<SimpleCamera> m_camera;
    dwImageHandle_t inputImage;
    bool m_isRaw = false;

    // image width and height
    uint32_t m_imageWidth  = 0U;
    uint32_t m_imageHeight = 0U;

    /// The maximum number of output objects for a given bound output.
    static constexpr uint32_t MAX_OBJECT_OUTPUT_COUNT = 1000;

public:
    /// -----------------------------
    /// Initialize application
    /// -----------------------------
    PyramidModuleSample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize SDK, SAL, Sensors, Image Streamers, PyramidModule
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks SDK context, SAL and Visualization
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
            CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

#ifdef DW_SDK_BUILD_PVA
#if VIBRANTE_PDK_DECIMAL < 6000400
            if (cupvaStreamCreate(&m_cupvaStream, cupvaEngineType_t::CUPVA_PVA0, cupvaAffinityType_t::CUPVA_VPU_ANY) != cupvaError_t::ErrorNone)
            {
                throw std::runtime_error("Failed to create PVA stream");
            }
#endif
#endif

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};

#ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
            sdkParams.enablePVA  = true;
#endif

            CHECK_DW_ERROR(dwInitialize(&m_context, DW_VERSION, &sdkParams));
            CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
        }

        //------------------------------------------------------------------------------
        // initialize Renderer
        //------------------------------------------------------------------------------
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

            // Setup render engine
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            params.defaultTile.lineWidth = 0.2f;
            params.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_20;
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));
        }
        //------------------------------------------------------------------------------
        // initialize Sensors
        //------------------------------------------------------------------------------
        {
            dwSensorParams sensorParams{};
            {
                std::string parameterString;
#ifdef VIBRANTE
                if (getArgument("input-type").compare("camera") == 0)
                {
                    std::string cameraType = getArgument("camera-type");
                    parameterString        = "camera-type=" + cameraType;
                    parameterString += ",camera-group=" + getArgument("camera-group");
                    std::string cameraMask[4] = {"0001", "0010", "0100", "1000"};
                    uint32_t cameraIdx        = std::stoi(getArgument("camera-index"));
                    if (cameraIdx < 0 || cameraIdx > 3)
                    {
                        std::cerr << "Error: camera index must be 0, 1, 2 or 3" << std::endl;
                        return false;
                    }
                    parameterString += ",camera-mask=" + cameraMask[cameraIdx];
                    sensorParams.protocol   = "camera.gmsl";
                    sensorParams.parameters = parameterString.c_str();
                }
                else
#endif
                {

                    std::string videoFormat = getArgument("video");
                    parameterString         = "video=" + videoFormat;

                    sensorParams.protocol   = "camera.virtual";
                    sensorParams.parameters = parameterString.c_str();
                }
                {
                    m_camera.reset(new SimpleCamera(sensorParams, m_sal, m_context, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8));
                    m_isRaw = false;
                }
            }

            if (m_camera == nullptr)
            {
                logError("Camera could not be created\n");
                return false;
            }
            // get camera properties
            m_cameraProps = m_camera->getCameraProperties();

            std::cout << "Camera image with " << m_cameraProps.resolution.x << "x"
                      << m_cameraProps.resolution.y << " at "
                      << m_cameraProps.framerate << " FPS" << std::endl;

            dwImageProperties displayProperties = m_camera->getOutputProperties();

            m_imageWidth  = displayProperties.width;
            m_imageHeight = displayProperties.height;

            displayProperties.format = DW_IMAGE_FORMAT_R_UINT8;
            CHECK_DW_ERROR(dwImage_create(&inputImage, displayProperties, m_context));

            displayProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            streamerCUDA2GL.reset(new SimpleImageStreamerGL<>(displayProperties, 1000, m_context));
            CHECK_DW_ERROR(dwImage_create(&m_imgGl, displayProperties, m_context));
        }

        //------------------------------------------------------------------------------
        // initialize Pyramid Module
        //------------------------------------------------------------------------------
        {
#ifdef DW_SDK_BUILD_PVA
            CHECK_DW_ERROR(dwPyramidPVA_initDefaultParams(&m_imagePyramidParams));

            // Set Number of Levels
            uint32_t numOflvl = std::atoi(getArgument("pyrLevel").c_str());

            if (numOflvl != 5)
            {
                throw std::runtime_error("PVA can't run pyramid levels other than 5 due to size restrictions.");
            }

            {
                // Set pva
                uint32_t pvaNo = std::atoi(getArgument("pvaNo").c_str());
                if (pvaNo != 0)
                {
                    std::stringstream errorMsg;
                    errorMsg << "Unexpected PVA no. PVA no must be 0";
                    throw std::runtime_error(errorMsg.str());
                }
                m_imagePyramidParams.processorType = static_cast<dwProcessorType>(static_cast<uint32_t>(DW_PROCESSOR_TYPE_PVA_0) + pvaNo);
            }

            m_imagePyramidParams.vpuIndex    = 0;
            m_imagePyramidParams.imageWidth  = m_imageWidth;
            m_imagePyramidParams.imageHeight = m_imageHeight;
            m_imagePyramidParams.levelCount  = numOflvl;

            // Allocate and bind output pyramid
            dwImageProperties props = m_camera->getOutputProperties();
            props.format            = DW_IMAGE_FORMAT_RGBA_UINT8;
            dwTrivialDataType type;
            dwImage_getPixelType(&type, props.format);

#if VIBRANTE_PDK_DECIMAL >= 6000400
            cudaStreamCreate(&m_stream);
#endif

            CHECK_DW_ERROR(dwPyramid_create(&m_pyramidOutput,
                                            m_imagePyramidParams.levelCount,
                                            m_imagePyramidParams.imageWidth,
                                            m_imagePyramidParams.imageHeight,
                                            type,
                                            m_context));

            CHECK_DW_ERROR(dwPyramidPVA_initialize(&m_pyramidModule,
                                                   &m_imagePyramidParams,
                                                   &m_pyramidOutput,
#if VIBRANTE_PDK_DECIMAL >= 6000400
                                                   m_stream
#else
                                                   0
#endif
                                                   ,
                                                   m_context));
#if VIBRANTE_PDK_DECIMAL >= 6000400
            if (cupvaCudaCreateStream(&m_cupvaStream, m_stream, pvaEngineType::PVA_PVA0, pvaAffinityType::PVA_VPU_ANY) != pvaError::PVA_ERROR_NONE)
            {
                throw std::runtime_error("Failed to create PVA stream");
            }
#endif
            CHECK_DW_ERROR(dwPyramidPVA_setPVAStream(m_cupvaStream, m_pyramidModule));

#else
            throw std::runtime_error("PVA pyramid not supported on this platform.");
#endif
        }

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - collect sensor frame
    ///     - run Image pyramids
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        dwImageHandle_t rgbImagehandle = m_camera->readFrame();
        if (rgbImagehandle == nullptr)
        {
            m_camera->resetCamera();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return;
        }

        CHECK_DW_ERROR(dwImage_copyConvert(inputImage, rgbImagehandle, m_context));

#ifdef DW_SDK_BUILD_PVA
        CHECK_DW_ERROR(dwPyramidPVA_computePyramid(&m_pyramidOutput,
                                                   inputImage,
                                                   0,
                                                   m_pyramidModule));
#endif

        CHECK_DW_ERROR(dwImage_copyConvert(m_imgGl, rgbImagehandle, m_context));
    }

    ///------------------------------------------------------------------------------
    /// Render sample output on screen
    ///     - render video
    ///     - render lane markings
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        frameGL = streamerCUDA2GL->post(m_imgGl);

        if (!frameGL)
            return;

        dwVector2f range{};
        dwImageGL* imageGL;
        dwImage_getGL(&imageGL, frameGL);
        range.x = imageGL->prop.width;
        range.y = imageGL->prop.height;
        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));
    }

    ///------------------------------------------------------------------------------
    /// Free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
#ifdef DW_SDK_BUILD_PVA
        if (&m_pyramidOutput)
        {
            dwPyramid_destroy(m_pyramidOutput);
        }

        // Release Pyramid Image PVA
        if (m_pyramidModule)
        {
            CHECK_DW_ERROR(dwPyramidPVA_release(m_pyramidModule));
        }
#endif

        // Release render engine
        if (m_renderEngine)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        // release streamer
        if (m_imgGl != nullptr)
        {
            CHECK_DW_ERROR(dwImage_destroy(m_imgGl));
            streamerCUDA2GL.reset();
        }

        // release camera
        if (m_camera)
        {
            m_camera.reset();
        }

        CHECK_DW_ERROR(dwImage_destroy(inputImage));
        // Release SDK
        CHECK_DW_ERROR(dwSAL_release(m_sal));
        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());

#ifdef DW_SDK_BUILD_PVA
        cupvaStreamDestroy(m_cupvaStream);
#endif
    }

    ///------------------------------------------------------------------------------
    /// Reset detector
    ///------------------------------------------------------------------------------
    void onReset() override
    {
#ifdef DW_SDK_BUILD_PVA
        CHECK_DW_ERROR(dwPyramidPVA_reset(m_pyramidModule));
#endif
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
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
};

int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
#ifdef VIBRANTE
                              ProgramArguments::Option_t("camera-type", "ar0231-rccb-bae-sf3324", "camera gmsl type (see sample_sensors_info for all available camera types on this platform)"),
                              ProgramArguments::Option_t("camera-group", "a", "input port"),
                              ProgramArguments::Option_t("camera-index", "0", "camera index within the camera-group 0-3"),
                              ProgramArguments::Option_t("input-type", "video", "input type either video or camera"),
#endif
                              ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/stereo/left_1.h264").c_str(), "path to video"),
                              ProgramArguments::Option_t("pyrLevel", "5", "Number of levels in Image Pyramid (5)"),
                              ProgramArguments::Option_t("pvaNo", "0", "PVA engine to run corresponding stage on (0)")},
                          "Pyramid Module sample.");

    PyramidModuleSample app(args);

    app.initializeWindow("Image Pyramid PVA Sample", 1280, 800, args.enabled("offscreen"));
    if (!args.enabled("offscreen"))
        app.setProcessRate(30);

    return app.run();
}
