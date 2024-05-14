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
#include <framework/SampleFramework.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Checks.hpp>

#include <dwvisualization/core/RenderEngine.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>
#include <dw/rig/Rig.h>
#include <dw/imageprocessing/colorcorrection/ColorCorrection.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Color Correction
// The color correction sample demonstrates H.264 playback with color
// correction. The sample opens a window to play back the 4 provided
// video files and corrects their color based on a selected master camera
// index. Color correction can be activated or deactivated by pressing the C key.
//------------------------------------------------------------------------------
class ColorCorrection : public DriveWorksSample
{
private:
    /// @brief color correct types supported by the module
    typedef enum {
        /**
         * No color correction, original image
         */
        DW_COLOR_CORRECT_NONE = 0,

        /**
         * global correction using all the reprojected topview
         */
        DW_COLOR_CORRECT_GLOBAL,
    } dwColorCorrectType;

    static const uint32_t CAMERA_COUNT = 4;

    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t context          = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t viz = DW_NULL_HANDLE;
    dwSALHandle_t sal                  = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwRenderEngineHandle_t renderEngine = DW_NULL_HANDLE;
    uint32_t tileId[CAMERA_COUNT]       = {};
    std::unique_ptr<SimpleCamera> cameras[CAMERA_COUNT];
    std::unique_ptr<SimpleImageStreamerGL<>> streamerCUDA2GL[CAMERA_COUNT];

    dwImageProperties cameraImageProps[CAMERA_COUNT] = {};
    dwImageHandle_t frameCUDArgba[CAMERA_COUNT]      = {DW_NULL_HANDLE};
    dwImageHandle_t frameGL[CAMERA_COUNT]            = {DW_NULL_HANDLE};

    dwColorCorrectHandle_t colorCorrect;
    uint32_t refIdx;
    dwColorCorrectType ccType = DW_COLOR_CORRECT_GLOBAL;
    float32_t factor;

public:
    ColorCorrection(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize Renderer, Sensors, Image Streamers and Tracker
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Get values from command line
        // -----------------------------------------
        refIdx              = std::stoi(getArgument("ref"));
        factor              = std::stof(getArgument("factor"));
        std::string rigPath = getArgument("rig");

        if (refIdx > 3)
        {
            throw std::runtime_error("--ref [n] expect to be within [0, 3]!");
        }

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            dwLogger_initialize(getConsoleLoggerCallback(true));
            dwLogger_setLogLevel(DW_LOG_VERBOSE);

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};

#ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
#endif

            CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
            dwSAL_initialize(&sal, context);
            dwVisualizationInitialize(&viz, context);
        }

        // -----------------------------
        // initialize sensors
        // -----------------------------
        float32_t framerate[CAMERA_COUNT] = {};
        {
            // create sensors
            for (uint32_t i = 0; i < CAMERA_COUNT; i++)
            {
                std::string params = "video=" + getArgument((std::string("video") + std::to_string(i + 1)).c_str());
                params += +",isp-mode=yuv420-uint8-planar"; // color correction cuda kernels supports only planar format; use the right isp mode
                dwSensorParams sensorParams{};
                sensorParams.protocol   = "camera.virtual";
                sensorParams.parameters = params.c_str();

                cameras[i].reset(new SimpleCamera(sensorParams, sal, context));
                dwImageProperties outputProps = cameras[i]->getOutputProperties();
                outputProps.type              = DW_IMAGE_CUDA;
                cameras[i]->setOutputProperties(outputProps);

                dwImageProperties displayProps = outputProps;
                displayProps.format            = DW_IMAGE_FORMAT_RGBA_UINT8;
                CHECK_DW_ERROR(dwImage_create(&frameCUDArgba[i], displayProps, context));
                streamerCUDA2GL[i].reset(new SimpleImageStreamerGL<>(
                    displayProps, 1000, context));
                cameraImageProps[i] = cameras[i]->getOutputProperties();

                dwCameraProperties cameraProps = cameras[i]->getCameraProperties();
                std::cout << "Camera image with " << cameraImageProps[i].width << "x"
                          << cameraImageProps[i].height << " at " << cameraProps.framerate
                          << " FPS" << std::endl;

                framerate[i] = cameraProps.framerate;
            }

            // make sure all inputs have the same format and the same framerate
            for (uint32_t i = 1; i < CAMERA_COUNT; i++)
            {
                if (framerate[i] != framerate[i - 1] ||
                    cameraImageProps[i].format != cameraImageProps[i - 1].format ||
                    cameraImageProps[i].width != cameraImageProps[i - 1].width ||
                    cameraImageProps[i].height != cameraImageProps[i - 1].height)
                    throw std::runtime_error("Unmatching input videos");
            }

            // we would like the application run as fast as the original video
            setProcessRate(framerate[0]);
        }

        // -----------------------------
        // initialize color correction module
        // -----------------------------
        {
            dwRigHandle_t rigConfig = DW_NULL_HANDLE;
            CHECK_DW_ERROR(dwRig_initializeFromFile(&rigConfig, context, rigPath.c_str()));

            dwColorCorrectParameters ccParams{};
            ccParams.cameraWidth  = cameraImageProps[0].width;
            ccParams.cameraHeight = cameraImageProps[0].height;
            CHECK_DW_ERROR(dwColorCorrect_initializeFromRig(&colorCorrect, &ccParams, rigConfig, context));
            CHECK_DW_ERROR(dwRig_release(rigConfig));
        }

        // -----------------------------
        // Initialize Render Engine
        // -----------------------------
        {
            // Render engine: setup default viewport and maximal size of the internal buffer
            dwRenderEngineParams params;
            dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight());
            CHECK_DW_ERROR(dwRenderEngine_initialize(&renderEngine, &params, viz));

            // Video tiles
            dwRenderEngineTileState tileStates[CAMERA_COUNT] = {};
            for (size_t k = 0; k < CAMERA_COUNT; k++)
            {
                tileStates[k]                       = params.defaultTile;
                tileStates[k].projectionMatrix      = DW_IDENTITY_MATRIX4F;
                tileStates[k].modelViewMatrix       = DW_IDENTITY_MATRIX4F;
                tileStates[k].layout.positionLayout = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
                tileStates[k].layout.sizeLayout     = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
                tileStates[k].layout.viewport       = {0, 0, getWindowWidth() / 2.f, getWindowHeight() / 2.f};
            }
            CHECK_DW_ERROR(dwRenderEngine_addTilesByCount(tileId, CAMERA_COUNT, 2, tileStates, renderEngine));
        }

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        for (uint32_t i = 0; i < CAMERA_COUNT; i++)
        {
            cameras[i].reset();
            streamerCUDA2GL[i].reset();
            CHECK_DW_ERROR(dwImage_destroy(frameCUDArgba[i]));
        }

        CHECK_DW_ERROR(dwRenderEngine_release(renderEngine));
        CHECK_DW_ERROR(dwColorCorrect_release(colorCorrect));

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        CHECK_DW_ERROR(dwSAL_release(sal));
        CHECK_DW_ERROR(dwVisualizationRelease(viz));
        CHECK_DW_ERROR(dwRelease(context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        for (uint32_t i = 0; i < CAMERA_COUNT; i++)
        {
            streamerCUDA2GL[i]->release();
            cameras[i]->resetCamera();
        }
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRectf rect;
        {
            rect.width  = width;
            rect.height = height;
            rect.x      = 0;
            rect.y      = 0;
            dwRenderEngine_setBounds(rect, renderEngine);
        }

        for (size_t k = 0; k < CAMERA_COUNT; k++)
        {
            dwRenderEngine_setTile(tileId[k], renderEngine);
            dwRenderEngine_setViewport({0, 0, rect.width / 2, rect.height / 2}, renderEngine);
        }
    }

    void onKeyDown(int key, int scancode, int mods) override
    {
        (void)scancode;
        (void)mods;

        if (key == GLFW_KEY_C)
        {
            switch (ccType)
            {
            case DW_COLOR_CORRECT_NONE: ccType   = DW_COLOR_CORRECT_GLOBAL; break;
            case DW_COLOR_CORRECT_GLOBAL: ccType = DW_COLOR_CORRECT_NONE; break;
            default: break;
            }
        }
    }

    void onProcess() override
    {
        // first add the ref view
        if (!runSingleCameraPipeline(refIdx, true, cameras[refIdx].get(),
                                     streamerCUDA2GL[refIdx].get(),
                                     &frameGL[refIdx], &frameCUDArgba[refIdx]))

        {
            return;
        }

        // do correction for the rest views
        for (uint32_t i = 0; i < CAMERA_COUNT; i++)
        {
            if (i == refIdx)
                continue;

            //Run sensor
            if (!runSingleCameraPipeline(i, false, cameras[i].get(),
                                         streamerCUDA2GL[i].get(),
                                         &frameGL[i], &frameCUDArgba[i]))
            {
                return;
            }
        }
    }

    void onRender() override
    {
        for (uint32_t i = 0; i < CAMERA_COUNT; i++)
        {
            if (frameGL[i])
            {
                CHECK_DW_ERROR(dwRenderEngine_setTile(tileId[i], renderEngine));

                dwVector2f range{};
                dwImageGL* imageGL;
                dwImage_getGL(&imageGL, frameGL[i]);
                range.x = imageGL->prop.width;
                range.y = imageGL->prop.height;
                CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL,
                                                            {0.0f, 0.0f, range.x, range.y}, renderEngine));
            }
        }
    }

    //------------------------------------------------------------------------------
    bool runSingleCameraPipeline(int curIdx, bool isRef,
                                 SimpleCamera* camera,
                                 SimpleImageStreamerGL<>* cuda2gl,
                                 dwImageHandle_t* glFrame,
                                 dwImageHandle_t* cudaFrameRgba)
    {
        dwImageHandle_t frameYUV = camera->readFrame();
        if (frameYUV == nullptr)
        {
            reset();
            return false;
        }

        if (ccType != DW_COLOR_CORRECT_NONE)
        {
            if (isRef)
            {
                CHECK_DW_ERROR(dwColorCorrect_setReferenceCameraView(frameYUV, curIdx, colorCorrect));
            }
            else
            {
                CHECK_DW_ERROR(dwColorCorrect_correctByReferenceView(frameYUV, curIdx, factor, colorCorrect));
            }
        }

        CHECK_DW_ERROR(dwImage_copyConvert(*cudaFrameRgba, frameYUV, context));
        *glFrame = cuda2gl->post(*cudaFrameRgba);

        return true;
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{

    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("video1",
                                                      (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
                           ProgramArguments::Option_t("video2",
                                                      (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_1.h264").c_str()),
                           ProgramArguments::Option_t("video3",
                                                      (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_2.h264").c_str()),
                           ProgramArguments::Option_t("video4",
                                                      (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_3.h264").c_str()),
                           ProgramArguments::Option_t("rig",
                                                      (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/rig.json").c_str()),
                           ProgramArguments::Option_t("ref", "2"),
                           ProgramArguments::Option_t("factor", "0.8")},
                          "ColorCorrection sample.");

    // -------------------
    // initialize and start a window application
    ColorCorrection app(args);

    app.initializeWindow("Color Correction Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
