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

#include <dw/core/base/Version.h>
#include <framework/DriveWorksSample.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/MathUtils.hpp>

#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>
#include <dw/imageprocessing/tracking/templatetracker/TemplateTracker.h>
#include <dwvisualization/core/RenderEngine.h>

#include <atomic>

using namespace dw_samples::common;

std::string getExt(const std::string& filename)
{
    return filename.substr(filename.find_last_of(".") + 1);
}

//------------------------------------------------------------------------------
// Camera template tracker
// The Camera template tracker sample demonstrates the template tracking
// capabilities of the dw_features module. It loads a video stream and
// reads the images sequentially. For each frame, it tracks templates from the
// previous frame. It doesn't detect new features, when there's no templates
// in the frame, the video replay will be paused automatically, you can use
// mouse to drag the boxes to track and press space to start replay/tracking.
//------------------------------------------------------------------------------
class CameraTemplateTracker : public DriveWorksSample
{
protected:
    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwContextHandle_t context          = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t viz = DW_NULL_HANDLE;
    dwSALHandle_t sal                  = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwRenderEngineHandle_t renderEngine = DW_NULL_HANDLE;
    dwCameraProperties cameraProps      = {};
    dwImageProperties imageProps        = {};

    dwImageHandle_t frameCudaRgba = DW_NULL_HANDLE;
    dwImageHandle_t frameGL       = DW_NULL_HANDLE;
    std::unique_ptr<SimpleImageStreamerGL<>> streamerCUDA2GL;
    std::unique_ptr<SimpleCamera> camera;

    // tracker handles
    uint32_t maxTemplateCount         = 100;
    dwTemplateArray templateCPU       = {};
    dwTemplateArray templateGPU       = {};
    dwTemplateTrackerHandle_t tracker = DW_NULL_HANDLE;
    dwPyramidImage currentPyramid     = {};
    dwPyramidImage previousPyramid    = {};

    std::vector<dwBox2Df> newBoxToTrack;
    std::vector<dwBox2Df> trackedBoxes;
    std::atomic<bool> updateNewBox;

    bool isPVA = false;

public:
    CameraTemplateTracker(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize Render Engine, Sensors, Image Streamers and Tracker
    /// -----------------------------
    bool onInitialize() override
    {
        uint32_t trackMode    = std::stoi(getArgument("trackMode"));
        uint32_t pyramidLevel = std::stoi(getArgument("pyramidLevel"));
        if (pyramidLevel == 0)
        {
            std::cout << "pyramidLevel must be greater than zero." << std::endl;
            exit(-1);
        }

        isPVA                = std::atoi(getArgument("pva").c_str()) == 1;
        uint32_t pvaEngineNo = std::atoi(getArgument("pvaEngineNo").c_str());

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
            CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};

#ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
            // initialize the sample with PVA enabled.
            sdkParams.enablePVA = true;
#endif

            CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
            CHECK_DW_ERROR(dwSAL_initialize(&sal, context));
        }

        // -----------------------------
        // Initialize Render Engine
        // -----------------------------
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&viz, context));

            // Render engine: setup default viewport and maximal size of the internal buffer
            dwRenderEngineParams params{};
            params.bufferSize = 20000;
            params.bounds     = {0, 0,
                             static_cast<float32_t>(getWindowWidth()),
                             static_cast<float32_t>(getWindowHeight())};

            CHECK_DW_ERROR(dwRenderEngine_initTileState(&params.defaultTile));
            params.defaultTile.layout.viewport = params.bounds;
            params.defaultTile.lineWidth       = 2.f;
            params.defaultTile.font            = DW_RENDER_ENGINE_FONT_VERDANA_16;
            CHECK_DW_ERROR(dwRenderEngine_initialize(&renderEngine, &params, viz));
        }

        // -----------------------------
        // initialize sensors and streamers
        // -----------------------------
        initSensor();

        // -----------------------------
        // Initialize template tracker
        // -----------------------------
        {
            dwProcessorType processorType = DW_PROCESSOR_TYPE_GPU;
            (void)pvaEngineNo;
            if (isPVA)
            {
                throw std::runtime_error("pvaTemplateTracker is not supported");
            }

            CHECK_DW_ERROR(dwTemplateArray_createNew(&templateCPU, maxTemplateCount,
                                                     DW_MEMORY_TYPE_CPU, nullptr, context));
            CHECK_DW_ERROR(dwTemplateArray_createNew(&templateGPU, maxTemplateCount,
                                                     DW_MEMORY_TYPE_CUDA, nullptr, context));

            dwTemplateTrackerParameters params = {};
            CHECK_DW_ERROR(dwTemplateTracker_initDefaultParams(&params));
            params.algorithm        = static_cast<dwTemplateTrackerAlgorithm>(trackMode);
            params.validWidth       = params.maxTemplateSize;
            params.validHeight      = params.maxTemplateSize;
            params.imageWidth       = imageProps.width;
            params.imageHeight      = imageProps.height;
            params.maxPyramidLevel  = pyramidLevel;
            params.maxTemplateCount = maxTemplateCount;
            params.processorType    = processorType;
            CHECK_DW_ERROR(dwTemplateTracker_initialize(&tracker, &params, 0, context));

            dwTrivialDataType pxlType;
            dwImage_getPixelType(&pxlType, imageProps.format);
            CHECK_DW_ERROR(dwPyramid_create(&currentPyramid, pyramidLevel,
                                            imageProps.width, imageProps.height,
                                            pxlType, context));

            CHECK_DW_ERROR(dwPyramid_create(&previousPyramid, pyramidLevel,
                                            imageProps.width, imageProps.height,
                                            pxlType, context));
        }

        // -----------------------------
        // Add some features at the beginning
        // -----------------------------
        addNewBoxToTrack();

        return true;
    }

    virtual void initSensor()
    {
        std::string videoPath = getArgument("video");
        std::string file      = "video=" + videoPath;

        dwSensorParams sensorParams{};
        sensorParams.protocol   = "camera.virtual";
        sensorParams.parameters = file.c_str();

        std::string ext = getExt(getArgument("video"));

        camera.reset(new SimpleCamera(sensorParams, sal, context));
        dwImageProperties outputProps = camera->getOutputProperties();
        outputProps.type              = DW_IMAGE_CUDA;
        outputProps.format            = DW_IMAGE_FORMAT_RGB_FLOAT16_PLANAR;
        camera->setOutputProperties(outputProps);

        dwImageProperties displayProps = camera->getOutputProperties();
        displayProps.format            = DW_IMAGE_FORMAT_RGBA_UINT8;

        CHECK_DW_ERROR(dwImage_create(&frameCudaRgba, displayProps, context));

        streamerCUDA2GL.reset(new SimpleImageStreamerGL<>(displayProps, 1000, context));

        cameraProps = camera->getCameraProperties();
        imageProps  = camera->getOutputProperties();
        printf("Camera image with %dx%d at %f FPS\n", imageProps.width,
               imageProps.height, cameraProps.framerate);

        // we would like the application run as fast as the original video
        setProcessRate(cameraProps.framerate);
    }

    virtual void onMouseDown(int button, float x, float y, int /* mods*/) override
    {
        if (button == 0)
        {
            updateNewBox = true;
            newBoxToTrack.push_back(dwBox2Df{});
            newBoxToTrack.back().x = x * imageProps.width / getWindowWidth();
            newBoxToTrack.back().y = y * imageProps.height / getWindowHeight();
        }
    }

    virtual void onMouseMove(float x, float y) override
    {
        if (!updateNewBox)
            return;

        int32_t fx    = x * imageProps.width / getWindowWidth();
        int32_t fy    = y * imageProps.height / getWindowHeight();
        dwBox2Df& box = newBoxToTrack.back();
        box.width     = abs(fx - box.x);
        box.height    = abs(fy - box.y);
        if (box.x > fx)
            box.x = fx;
        if (box.y > fy)
            box.y = fy;
    }

    virtual void onMouseUp(int button, float /* x*/, float /* y*/, int /* mods*/) override
    {
        if (button == 0)
        {
            if (updateNewBox)
            {
                updateNewBox = false;

                // Discard boxes that are too small
                if (newBoxToTrack.back().width < 3 || newBoxToTrack.back().height < 3)
                    newBoxToTrack.pop_back();

                auto& newBox = newBoxToTrack.back();
                std::cout << "New feature added: (" << newBox.x << "," << newBox.y << ") size=(" << newBox.width << "," << newBox.height << ")" << std::endl;
            }
        }
    }

    void addNewBoxToTrack()
    {
        newBoxToTrack.push_back({319, 188, 25, 35});
        newBoxToTrack.push_back({270, 227, 44, 56});
        newBoxToTrack.push_back({651, 275, 46, 36});
        newBoxToTrack.push_back({714, 279, 25, 42});
        newBoxToTrack.push_back({745, 231, 76, 38});
        newBoxToTrack.push_back({1075, 222, 60, 47});
        newBoxToTrack.push_back({131, 274, 36, 93});
        newBoxToTrack.push_back({255, 378, 26, 29});
        newBoxToTrack.push_back({603, 398, 50, 36});
        newBoxToTrack.push_back({726, 402, 74, 39});
        newBoxToTrack.push_back({518, 334, 27, 38});

        updateNewBox = false;
    }
    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        newBoxToTrack.clear();
        trackedBoxes.clear();
        camera->resetCamera();
        CHECK_DW_ERROR(dwTemplateArray_reset(&templateGPU, cudaStream_t(0)));
        CHECK_DW_ERROR(dwTemplateTracker_reset(tracker));

        addNewBoxToTrack();
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        // stop sensor
        camera.reset();

        // release feature tracker
        CHECK_DW_ERROR(dwTemplateArray_destroy(templateCPU));
        CHECK_DW_ERROR(dwTemplateArray_destroy(templateGPU));
        CHECK_DW_ERROR(dwTemplateTracker_release(tracker));

        CHECK_DW_ERROR(dwPyramid_destroy(currentPyramid));
        CHECK_DW_ERROR(dwPyramid_destroy(previousPyramid));

        // release render engine
        if (renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(renderEngine));
        }

        // release streamer
        CHECK_DW_ERROR(dwImage_destroy(frameCudaRgba));
        streamerCUDA2GL.reset();

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        {
            CHECK_DW_ERROR(dwSAL_release(sal));
            CHECK_DW_ERROR(dwVisualizationRelease(viz));
            CHECK_DW_ERROR(dwRelease(context));
            CHECK_DW_ERROR(dwLogger_release());
        }
    }

    ///------------------------------------------------------------------------------
    /// Change render engine properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRectf rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0.f;
        rect.y      = 0.f;

        CHECK_DW_ERROR(dwRenderEngine_reset(renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, renderEngine));
    }

    void onRender() override
    {
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        CHECK_DW_ERROR(dwRenderEngine_setTile(0, renderEngine));

        if (frameGL)
        {
            dwVector2f range{};
            dwImageGL* imageGL;
            dwImage_getGL(&imageGL, frameGL);
            range.x = imageGL->prop.width;
            range.y = imageGL->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL,
                                                        {0.f, 0.f, range.x, range.y}, renderEngine));
        }

        if (isPaused())
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor({1.f, 0.f, 0.f, 1.f}, renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderText2D(
                "Drag the mouse to add boxes for tracking, press space to start tracking",
                {32.f, 32.f}, renderEngine));

            if (!newBoxToTrack.empty())
                CHECK_DW_ERROR(dwRenderEngine_render(
                    DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D,
                    newBoxToTrack.data(), sizeof(dwBox2Df), 0,
                    newBoxToTrack.size(), renderEngine));
        }

        if (!trackedBoxes.empty())
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor({0.f, 1.f, 0.f, 1.f}, renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_render(
                DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D,
                trackedBoxes.data(), sizeof(dwBox2Df), 0,
                trackedBoxes.size(), renderEngine));
        }

        renderutils::renderFPS(renderEngine, getCurrentFPS());
    }

    bool prepareImage(dwImageCUDA** frameCUDARcb)
    {
        dwImageHandle_t frameRcb = camera->readFrame();
        if (frameRcb == nullptr)
        {
            reset();
            return false;
        }
        else
        {
            CHECK_DW_ERROR(dwImage_getCUDA(frameCUDARcb, frameRcb));
            CHECK_DW_ERROR(dwImage_copyConvert(frameCudaRgba, frameRcb, context));
            frameGL = streamerCUDA2GL->post(frameCudaRgba);
        }

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - grab a frame from the camera
    ///     - convert frame to RGB
    ///     - push frame through the streamer to convert it into GL
    ///     - track the features in the frame
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        ProfileCUDASection s(getProfilerCUDA(), "ProcessFrame");

        // ---------------------------
        //  grab frame from camera, convert to RGB and push to GL
        // ---------------------------
        dwImageCUDA* frameCudaRcb = nullptr;
        if (!prepareImage(&frameCudaRcb))
            return;

        // ---------------------------
        // track the features in the frame
        // ---------------------------
        trackFrame(frameCudaRcb);

        // ---------------------------
        // Add new features
        // ---------------------------
        if (!newBoxToTrack.empty())
        {
            addNewFeaturesToTracker();
        }
        else if (*templateCPU.templateCount == 0)
        {
            pause();
        }
    }

    void trackFrame(const dwImageCUDA* plane)
    {
        ProfileCUDASection s(getProfilerCUDA(), "trackFrame");

        std::swap(currentPyramid, previousPyramid);
        {
            ProfileCUDASection s(getProfilerCUDA(), "computePyramid");
            CHECK_CUDA_ERROR(dwImageFilter_computePyramid(&currentPyramid, plane,
                                                          0, context));
        }

        if (getFrameIndex() > 0)
        {
            ProfileCUDASection s(getProfilerCUDA(), "trackCall");
            CHECK_DW_ERROR(dwTemplateTracker_trackPyramid(
                &templateGPU, &currentPyramid,
                &previousPyramid, tracker));
        }

        {
            //Get tracked feature info to CPU
            ProfileCUDASection s(getProfilerCUDA(), "downloadToCPU");
            CHECK_DW_ERROR(dwTemplateArray_copyAsync(&templateCPU, &templateGPU, 0));
        }

        trackedBoxes.resize(*templateCPU.templateCount);
        for (uint32_t i = 0; i < *templateCPU.templateCount; i++)
        {
            trackedBoxes[i] = templateCPU.bboxes[i];
        }
    }

    void addNewFeaturesToTracker()
    {
        // Avoid adding features while user is drawing
        if (updateNewBox)
            return;

        ProfileCUDASection s(getProfilerCUDA(), "AddFeature");
        if (newBoxToTrack.empty())
            return;

        uint32_t& nNew = *templateCPU.templateCount;
        for (const dwBox2Df& box : newBoxToTrack)
        {
            templateCPU.bboxes[nNew]   = box;
            templateCPU.statuses[nNew] = DW_FEATURE2D_STATUS_DETECTED;
            nNew++;

            if (nNew >= maxTemplateCount)
            {
                std::cout << "Too much features, will truncate the number to " << maxTemplateCount << std::endl;
                break;
            }
        }

        {
            ProfileCUDASection s(getProfilerCUDA(), "uploadToGPU");
            CHECK_DW_ERROR(dwTemplateArray_copyAsync(&templateGPU, &templateCPU, 0));
        }

        newBoxToTrack.clear();
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
                           ProgramArguments::Option_t("trackMode", "0"),
                           ProgramArguments::Option_t("pyramidLevel", "1"),
                           ProgramArguments::Option_t("pva", "0"),
                           ProgramArguments::Option_t("pvaEngineNo", "0")},
                          "Camera template tracker sample which tracks user defined templates and playback the results in a GL window.");

    // -------------------
    // initialize and start a window application
    CameraTemplateTracker app(args);

    app.initializeWindow("Camera template tracker", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
