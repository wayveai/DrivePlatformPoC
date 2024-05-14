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
#include <framework/WindowGLFW.hpp>
#include <framework/MathUtils.hpp>

#include <dwvisualization/core/RenderEngine.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>
#include <dw/imageprocessing/featuredetector/FeatureDetector.h>
#include <dw/imageprocessing/tracking/featuretracker/FeatureTracker.h>
#include <dw/imageprocessing/sfm/SFM.h>
#include <dw/sensors/sensormanager/SensorManager.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>
#include <dw/egomotion/base/Egomotion.h>

#include <framework/Mat4.hpp>

#include <atomic>
#include <unistd.h>
#include <cstring>
#include <array>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// SfM sample
// Demonstrates the usage of feature tracking, 3d triangulation, and car pose estimation
// using four cameras.
//------------------------------------------------------------------------------
class SfmSample : public DriveWorksSample
{
private:
    static constexpr size_t CAMERA_COUNT = 4;

    // ------------------------------------------------
    // SAL
    // ------------------------------------------------
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t sal                     = DW_NULL_HANDLE;

    dwSensorManagerHandle_t sensorManager = DW_NULL_HANDLE;
    dwRigHandle_t rigConfig               = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------

    // Egomotion
    dwVehicle const* vehicle;
    dwEgomotionHandle_t egomotion           = DW_NULL_HANDLE;
    dwVehicleIOHandle_t vehicleIO           = DW_NULL_HANDLE;
    dwTime_t lastEgoUpdate                  = 0;
    dwTransformation3f egoRig2World         = DW_IDENTITY_TRANSFORMATION3F;
    dwTransformation3f previousEgoRig2World = DW_IDENTITY_TRANSFORMATION3F;

    // Feature tracker
    uint32_t maxFeatureCount          = 2000;
    const size_t FEATURE_HISTORY_SIZE = 60;

    // Reconstructor
    dwReconstructorHandle_t reconstructor = DW_NULL_HANDLE;

    // Per-camera data
    struct CameraData
    {
        // Sensor
        dwSensorHandle_t camera            = DW_NULL_HANDLE;
        dwImageStreamerHandle_t image2GL   = DW_NULL_HANDLE;
        dwCameraProperties cameraProps     = {};
        dwImageProperties cameraImageProps = {};
        dwCameraFrameHandle_t frame        = DW_NULL_HANDLE;

        // Image
        dwImageHandle_t frameCudaRgba = DW_NULL_HANDLE; // Only used if the camera produces CUDA images

        dwImageHandle_t frameCudaYuv = DW_NULL_HANDLE; // This is the image used for processing. Could be same as frameCameraCudaYuv or coming from nvm2cuda streamer

        dwImageHandle_t frameGL = DW_NULL_HANDLE;

        // Features
        dwPyramidImage pyramidCurrent  = {};
        dwPyramidImage pyramidPrevious = {};

        dwFeature2DDetectorHandle_t detector = DW_NULL_HANDLE;
        dwFeature2DTrackerHandle_t tracker   = DW_NULL_HANDLE;

        // These point into the buffers of g_featureList
        dwFeatureHistoryArray featureHistoryCPU = {};
        dwFeatureHistoryArray featureHistoryGPU = {};
        dwFeatureArray featureDetectedGPU       = {};

        // Predicted points
        dwVector2f* d_predictedFeatureLocations = {nullptr};
        std::unique_ptr<dwVector2f[]> predictedFeatureLocations;

        // Projected points
        std::unique_ptr<dwVector2f[]> projectedLocations;
        dwVector2f* d_projectedLocations = {nullptr};

        // Reconstruction data
        dwVector4f* d_worldPoints = nullptr;
        std::unique_ptr<dwVector4f[]> worldPoints;

        // Render
        uint32_t tileId;
    };
    std::array<CameraData, CAMERA_COUNT> cameras;

    // Arrays containing pointers for all cameras
    uint32_t* d_allFeatureCount[CAMERA_COUNT]             = {nullptr};
    dwFeature2DStatus* d_allFeatureStatuses[CAMERA_COUNT] = {nullptr};
    dwVector4f* d_allWorldPoints[CAMERA_COUNT]            = {nullptr};
    dwVector2f* d_allProjections[CAMERA_COUNT]            = {nullptr};

    dwTransformation3f currentRig2World = DW_IDENTITY_TRANSFORMATION3F;

    // Pose rendering
    dwVector4f poseColor = DW_RENDERER_COLOR_DARKGREEN;
    std::vector<dwVector3f> positionsCAN;
    std::vector<dwVector3f> positionsRefined;
    std::vector<dwVector3f> positionsDifference;

    // UI control
    bool doPoseRefinement    = true;
    bool doFeaturePrediction = true;
    uint32_t worldTileId     = 0;

    const uint32_t MAX_FRAME_COUNT = 2000;
    uint32_t maxRenderBufferCount;

public:
    SfmSample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

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

    /// -----------------------------
    /// Initialize Renderer, Sensors, Image Streamers and Tracker
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Get values from command line
        // -----------------------------------------
        maxFeatureCount      = std::stoi(getArgument("maxFeatureCount"));
        maxRenderBufferCount = std::max(MAX_FRAME_COUNT, maxFeatureCount);

        uint32_t useHalf                = std::stoi(getArgument("useHalf"));
        uint32_t enableAdaptiveWindow   = std::stoi(getArgument("enableAdaptiveWindow"));
        float32_t displacementThreshold = std::stof(getArgument("displacementThreshold"));

        dwFeature2DTrackerAlgorithm trackMode =
            (getArgument("trackMode") == "0" ? DW_FEATURE2D_TRACKER_ALGORITHM_STD : DW_FEATURE2D_TRACKER_ALGORITHM_EX_FAST);
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);
            CHECK_DW_ERROR(dwSAL_initialize(&sal, m_context));
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));
        }

        // -----------------------------
        // initialize sensors
        // -----------------------------
        {

            // Need to change directories because the rig.json file contains
            // relative paths to the sensors
            int res;
            res = chdir(getArgument("baseDir").c_str());
            (void)res;
            CHECK_DW_ERROR(dwRig_initializeFromFile(&rigConfig, m_context, getArgument("rig").c_str()));

            // Sensor manager
            dwSensorManagerParams sensorManagerParams    = {};
            sensorManagerParams.singleVirtualCameraGroup = true; // we are expecting all cameras to be provided with a single SM event
            CHECK_DW_ERROR(dwSensorManager_initializeFromRigWithParams(&sensorManager, rigConfig, &sensorManagerParams, 10, sal));

            // Add can sensor

            for (size_t k = 0; k < CAMERA_COUNT; k++)
            {
                auto& camera = cameras[k];

                uint32_t cameraIndex{};
                CHECK_DW_ERROR(dwSensorManager_getSensorIndex(&cameraIndex, DW_SENSOR_CAMERA, k, sensorManager));
                CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&camera.camera, cameraIndex, sensorManager));
                CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&camera.cameraProps, camera.camera));
                CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&camera.cameraImageProps, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera.camera));

                // Pyramids
                CHECK_DW_ERROR(dwPyramid_create(&camera.pyramidCurrent, 3, camera.cameraImageProps.width,
                                                camera.cameraImageProps.height, DW_TYPE_UINT8, m_context));
                CHECK_DW_ERROR(dwPyramid_create(&camera.pyramidPrevious, 3, camera.cameraImageProps.width,
                                                camera.cameraImageProps.height, DW_TYPE_UINT8, m_context));

                // Feature list
                CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&camera.featureHistoryCPU, maxFeatureCount,
                                                               FEATURE_HISTORY_SIZE, DW_MEMORY_TYPE_CPU, nullptr, m_context));
                CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&camera.featureHistoryGPU, maxFeatureCount,
                                                               FEATURE_HISTORY_SIZE, DW_MEMORY_TYPE_CUDA, nullptr, m_context));
                CHECK_DW_ERROR(dwFeatureArray_createNew(&camera.featureDetectedGPU, maxFeatureCount,
                                                        DW_MEMORY_TYPE_CUDA, nullptr, m_context));

                // -----------------------------
                // Feature tracker
                // -----------------------------
                dwFeature2DDetectorConfig detectorConfig = {};
                dwFeature2DDetector_initDefaultParams(&detectorConfig);
                detectorConfig.type            = DW_FEATURE2D_DETECTOR_TYPE_STD;
                detectorConfig.imageWidth      = camera.cameraImageProps.width;
                detectorConfig.imageHeight     = camera.cameraImageProps.height;
                detectorConfig.maxFeatureCount = maxFeatureCount;
                CHECK_DW_ERROR(dwFeature2DDetector_initialize(&camera.detector, &detectorConfig, cudaStream_t(0), m_context));

                dwFeature2DTrackerConfig trackerConfig = {};
                dwFeature2DTracker_initDefaultParams(&trackerConfig);
                trackerConfig.algorithm                  = trackMode;
                trackerConfig.imageWidth                 = camera.cameraImageProps.width;
                trackerConfig.imageHeight                = camera.cameraImageProps.height;
                trackerConfig.maxFeatureCount            = maxFeatureCount;
                trackerConfig.historyCapacity            = FEATURE_HISTORY_SIZE;
                trackerConfig.pyramidLevelCount          = camera.pyramidCurrent.levelCount;
                trackerConfig.numLevelTranslationOnly    = trackerConfig.pyramidLevelCount - 1;
                trackerConfig.detectorType               = detectorConfig.type;
                trackerConfig.useHalf                    = useHalf;
                trackerConfig.enableAdaptiveWindowSizeLK = enableAdaptiveWindow;
                trackerConfig.displacementThreshold      = displacementThreshold;
                trackerConfig.enableSparseOutput         = 1;
                CHECK_DW_ERROR(dwFeature2DTracker_initialize(&camera.tracker, &trackerConfig, cudaStream_t(0), m_context));

                CHECK_CUDA_ERROR(cudaMalloc(&camera.d_predictedFeatureLocations, maxFeatureCount * sizeof(dwVector2f)));
                CHECK_CUDA_ERROR(cudaMalloc(&camera.d_worldPoints, maxFeatureCount * sizeof(dwVector4f)));
                CHECK_CUDA_ERROR(cudaMalloc(&camera.d_projectedLocations, maxFeatureCount * sizeof(dwVector2f)));
                camera.predictedFeatureLocations.reset(new dwVector2f[maxFeatureCount]);
                camera.worldPoints.reset(new dwVector4f[maxFeatureCount]);
                camera.projectedLocations.reset(new dwVector2f[maxFeatureCount]);

                CHECK_CUDA_ERROR(cudaMemset(camera.d_worldPoints, 0, maxFeatureCount * sizeof(dwVector4f)));

                d_allFeatureCount[k]    = camera.featureHistoryGPU.featureCount;
                d_allFeatureStatuses[k] = camera.featureHistoryGPU.statuses;
                d_allProjections[k]     = camera.d_projectedLocations;
                d_allWorldPoints[k]     = camera.d_worldPoints;

                // Streamer pipeline
                dwImageProperties displayImageProps{};
                CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&displayImageProps, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, camera.camera));

                // image2GL is either from NvMedia or CUDA, depending on camera
                CHECK_DW_ERROR(dwImageStreamerGL_initialize(&camera.image2GL,
                                                            &displayImageProps,
                                                            DW_IMAGE_GL,
                                                            m_context));
            }

            // we would like the application run as fast as the original video
            setProcessRate(cameras[0].cameraProps.framerate);
        }

        // -----------------------------
        // Egomotion
        // -----------------------------
        CHECK_DW_ERROR(dwRig_getVehicle(&vehicle, rigConfig));

        dwEgomotionParameters egoparams{};
        egoparams.vehicle         = *vehicle;
        egoparams.motionModel     = DW_EGOMOTION_ODOMETRY;
        egoparams.automaticUpdate = true;
        CHECK_DW_ERROR(dwEgomotion_initialize(&egomotion, &egoparams, m_context));

        //VehicleIO
        CHECK_DW_ERROR(dwVehicleIO_initializeFromRig(&vehicleIO, rigConfig, m_context));

        // -----------------------------
        // Reconstructor
        // -----------------------------
        dwReconstructorConfig reconstructorConfig;
        dwReconstructor_initConfig(&reconstructorConfig);
        reconstructorConfig.maxFeatureCount = maxFeatureCount;
        reconstructorConfig.rig             = rigConfig;

        CHECK_DW_ERROR(dwReconstructor_initialize(&reconstructor, &reconstructorConfig, cudaStream_t(0), m_context));

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            float32_t videoAspect     = static_cast<float32_t>(cameras[0].cameraImageProps.width) / cameras[0].cameraImageProps.height;
            float32_t videoTileHeight = getWindowHeight() / CAMERA_COUNT;
            float32_t videoTileWidth  = videoTileHeight * videoAspect;

            dwRenderEngineParams params;
            dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight());
            // This should be the size of the positions because they grow as the program
            // continues to run.
            params.bufferSize = maxRenderBufferCount * sizeof(dwVector4f);

            // 1st tile (default) - 3d world rendering
            {
                dwRenderEngine_initTileState(&params.defaultTile);
                params.defaultTile.layout.positionType    = DW_RENDER_ENGINE_TILE_POSITION_TYPE_TOP_LEFT;
                params.defaultTile.layout.positionLayout  = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
                params.defaultTile.layout.sizeLayout      = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
                params.defaultTile.layout.viewport.x      = videoTileWidth;
                params.defaultTile.layout.viewport.y      = 0;
                params.defaultTile.layout.viewport.width  = getWindowWidth() - videoTileWidth;
                params.defaultTile.layout.viewport.height = getWindowHeight();
            }

            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

            // Video tiles
            for (size_t k = 0; k < CAMERA_COUNT; k++)
            {
                dwRenderEngineTileState tileParams = params.defaultTile;
                tileParams.projectionMatrix        = DW_IDENTITY_MATRIX4F;
                tileParams.modelViewMatrix         = DW_IDENTITY_MATRIX4F;
                tileParams.layout.positionLayout   = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
                tileParams.layout.sizeLayout       = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
                tileParams.layout.viewport         = {0.f,
                                              k * videoTileHeight,
                                              videoTileWidth,
                                              videoTileHeight};
                tileParams.layout.positionType = DW_RENDER_ENGINE_TILE_POSITION_TYPE_TOP_LEFT;

                CHECK_DW_ERROR(dwRenderEngine_addTile(&cameras[k].tileId, &tileParams, m_renderEngine));
            }
        }

        // -----------------------------
        // Start Sensors
        // -----------------------------
        CHECK_DW_ERROR(dwSensorManager_start(sensorManager));

        return true;
    }

    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        dwSensorManager_stop(sensorManager);
        dwSensorManager_reset(sensorManager);
        dwSensorManager_start(sensorManager);
        dwEgomotion_reset(egomotion);
        dwVehicleIO_reset(vehicleIO);
        for (auto& camera : cameras)
        {
            dwFeatureHistoryArray_reset(&camera.featureHistoryGPU, cudaStream_t(0));
            dwFeatureArray_reset(&camera.featureDetectedGPU, cudaStream_t(0));

            dwFeature2DDetector_reset(camera.detector);
            dwFeature2DTracker_reset(camera.tracker);
        }

        dwReconstructor_reset(reconstructor);

        positionsCAN.clear();
        positionsRefined.clear();
        positionsDifference.clear();
        currentRig2World = DW_IDENTITY_TRANSFORMATION3F;
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        // stop sensor
        dwSensorManager_stop(sensorManager);

        dwSensorManager_release(sensorManager);
        dwRig_release(rigConfig);

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        dwEgomotion_release(egomotion);
        dwVehicleIO_release(vehicleIO);

        dwReconstructor_release(reconstructor);

        // Per-camera data
        for (auto& camera : cameras)
        {
            releaseFrame(camera);

            dwImageStreamerGL_release(camera.image2GL);
            dwPyramid_destroy(camera.pyramidCurrent);
            dwPyramid_destroy(camera.pyramidPrevious);

            dwFeature2DDetector_release(camera.detector);
            dwFeature2DTracker_release(camera.tracker);

            dwFeatureHistoryArray_destroy(camera.featureHistoryCPU);
            dwFeatureHistoryArray_destroy(camera.featureHistoryGPU);
            dwFeatureArray_destroy(camera.featureDetectedGPU);

            cudaFree(camera.d_predictedFeatureLocations);
            cudaFree(camera.d_projectedLocations);
            cudaFree(camera.d_worldPoints);
        };

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        dwSAL_release(sal);
        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    void onKeyDown(int key, int /* scancode*/, int /*mods*/) override
    {
        if (key == GLFW_KEY_Q)
        {
            if (worldTileId == 0)
                worldTileId = 4;
            else
                worldTileId--;
        }
        else if (key == GLFW_KEY_V)
        {
            doPoseRefinement = !doPoseRefinement;
        }
        else if (key == GLFW_KEY_F)
        {
            doFeaturePrediction = !doFeaturePrediction;
        }
    }
    void onMouseDown(int button, float x, float y, int mods) override
    {
        getMouseView().mouseDown(button, x, y);
        (void)mods;
    }
    void onMouseUp(int button, float x, float y, int mods) override
    {
        getMouseView().mouseUp(button, x, y);
        (void)mods;
    }
    void onMouseMove(float x, float y) override
    {
        getMouseView().mouseMove(x, y);
    }
    void onMouseWheel(float x, float y) override
    {
        getMouseView().mouseWheel(x, y);
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        {
            //dwRenderEngine_reset(m_renderEngine);
            dwRectf rect;
            rect.width  = width;
            rect.height = height;
            rect.x      = 0;
            rect.y      = 0;
            dwRenderEngine_setBounds(rect, m_renderEngine);
        }

        float32_t videoAspect     = static_cast<float32_t>(cameras[0].cameraImageProps.width) / cameras[0].cameraImageProps.height;
        float32_t videoTileHeight = getWindowHeight() / CAMERA_COUNT;
        float32_t videoTileWidth  = videoTileHeight * videoAspect;

        dwRenderEngine_setTile(0, m_renderEngine);
        dwRenderEngine_setViewport({videoTileWidth, 0, static_cast<float32_t>(width - videoTileWidth), static_cast<float32_t>(height)}, m_renderEngine);

        for (size_t k = 0; k < CAMERA_COUNT; k++)
        {
            dwRenderEngine_setTile(k + 1, m_renderEngine);
            dwRenderEngine_setViewport({0, k * videoTileHeight, videoTileWidth, videoTileHeight}, m_renderEngine);
        }
    }

    void checkBufferSizeAgainst(size_t size)
    {
        if (size > maxRenderBufferCount)
        {
            std::cerr << "Cannot render point count over "
                      << maxRenderBufferCount
                      << ". Try increasing MAX_FRAME_COUNT or maxFeatureCount."
                      << std::endl;
            throw std::runtime_error("Requested render size greater than size of render buffer.");
        }
    }

    ///------------------------------------------------------------------------------
    /// Render the window
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        if (isOffscreen())
            return;

        glClearColor(0.0, 0.0, 0.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (size_t tileId = 0; tileId < 5; tileId++)
        {
            dwRenderEngine_setTile(tileId, m_renderEngine);

            if (tileId == worldTileId)
                render3D();
            else
            {
                size_t cameraIdx;
                if (tileId > worldTileId)
                    cameraIdx = tileId - worldTileId - 1;
                else
                    cameraIdx = tileId + 4 - worldTileId;

                renderCamera(cameras[cameraIdx]);
            }
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
        CHECK_GL_ERROR();
    }

    void render3D()
    {
        getMouseView().setWindowAspect(static_cast<float32_t>(getWindowWidth()) / getWindowHeight());

        dwMatrix4f pi;
        Mat4_IsoInv(pi.array, currentRig2World.array);

        dwMatrix4f t;
        Mat4_AxB(t.array, getMouseView().getModelView()->array, pi.array);

        dwRenderEngine_setProjection(getMouseView().getProjection(), m_renderEngine);
        dwRenderEngine_setModelView(&t, m_renderEngine);

        dwRenderEngine_setColor(DW_RENDERER_COLOR_DARKGREY, m_renderEngine);
        dwRenderEngine_renderPlanarGrid3D({-300, -300, 600, 600}, 5, 5, &DW_IDENTITY_MATRIX4F, m_renderEngine);

        checkBufferSizeAgainst(positionsCAN.size());
        checkBufferSizeAgainst(positionsRefined.size());
        checkBufferSizeAgainst(positionsDifference.size());

        // Render poses
        dwRenderEngine_setColor(DW_RENDERER_COLOR_DARKGREEN, m_renderEngine);
        dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D, positionsDifference.data(), sizeof(dwVector3f), 0, positionsDifference.size() / 2, m_renderEngine);

        dwRenderEngine_setColor(DW_RENDERER_COLOR_DARKGREEN, m_renderEngine);
        dwRenderEngine_setPointSize(2.0f, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D, positionsCAN.data(), sizeof(dwVector3f), 0, positionsCAN.size(), m_renderEngine);

        dwRenderEngine_setColor(poseColor, m_renderEngine);
        dwRenderEngine_setLineWidth(2.0f, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_3D, positionsRefined.data(), sizeof(dwVector3f), 0, positionsRefined.size(), m_renderEngine);

        // Render points
        dwVector4f colors[4] = {DW_RENDERER_COLOR_RED,
                                DW_RENDERER_COLOR_GREEN,
                                DW_RENDERER_COLOR_BLUE,
                                DW_RENDERER_COLOR_YELLOW};
        dwRenderEngine_setPointSize(2.0f, m_renderEngine);
        for (size_t k = 0; k < CAMERA_COUNT; k++)
        {
            auto& camera = cameras[k];

            // Static buffers so allocation only happens for the first frame
            static std::vector<dwVector3f> pointsBuffer;

            pointsBuffer.clear();
            for (size_t i = 0; i < *camera.featureHistoryCPU.featureCount; i++)
            {
                auto& p = camera.worldPoints[i];
                if (p.w != 0.0f)
                    pointsBuffer.push_back(dwVector3f{p.x, p.y, p.z});
            }
            dwRenderEngine_setColor(colors[k], m_renderEngine);
            checkBufferSizeAgainst(pointsBuffer.size());
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D, pointsBuffer.data(), sizeof(dwVector3f), 0, pointsBuffer.size(), m_renderEngine);
        }
    }

    void renderCamera(CameraData& camera)
    {
        // Static buffers so allocation only happens for the first frame
        static std::vector<dwVector2f> featuresBuffer;
        static std::vector<dwVector2f> reprojectionsBuffer;
        static std::vector<dwVector2f> predictionPointsBuffer;
        static std::vector<dwVector2f> predictionLinesBuffer;

        if (!camera.frameGL)
            return;

        dwRenderEngine_setProjection(&DW_IDENTITY_MATRIX4F, m_renderEngine);
        dwRenderEngine_setModelView(&DW_IDENTITY_MATRIX4F, m_renderEngine);

        //////////////
        // Draw image
        dwVector2f range{};
        dwImageGL* frameGL;
        dwImage_getGL(&frameGL, camera.frameGL);
        range.x = frameGL->prop.width;
        range.y = frameGL->prop.height;
        dwRenderEngine_setCoordinateRange2D(range, m_renderEngine);
        dwRenderEngine_renderImage2D(frameGL, {0.0f, 0.0f, range.x, range.y}, m_renderEngine);

        //////////////
        // Draw features
        dwFeatureArray curFeatures{};
        dwFeatureHistoryArray_getCurrent(&curFeatures, &camera.featureHistoryCPU);
        const dwVector2f* locations = curFeatures.locations;

        featuresBuffer.clear();
        reprojectionsBuffer.clear();
        predictionPointsBuffer.clear();
        predictionLinesBuffer.clear();
        for (size_t i = 0; i < *curFeatures.featureCount; i++)
        {
            if (curFeatures.statuses[i] != DW_FEATURE2D_STATUS_TRACKED)
                continue;

            featuresBuffer.push_back(locations[i]);
            predictionPointsBuffer.push_back(camera.predictedFeatureLocations[i]);
            predictionLinesBuffer.push_back(camera.predictedFeatureLocations[i]);
            predictionLinesBuffer.push_back(locations[i]);

            if (camera.worldPoints[i].w != 0.0f)
                reprojectionsBuffer.push_back(camera.projectedLocations[i]);
        }

        dwRenderEngine_setPointSize(2.0f, m_renderEngine);
        dwRenderEngine_setColor(DW_RENDERER_COLOR_GREEN, m_renderEngine);
        checkBufferSizeAgainst(featuresBuffer.size() / 2);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D, featuresBuffer.data(), sizeof(dwVector2f), 0, featuresBuffer.size(), m_renderEngine);

        dwRenderEngine_setPointSize(4.0f, m_renderEngine);
        dwRenderEngine_setColor(DW_RENDERER_COLOR_DARKRED, m_renderEngine);
        checkBufferSizeAgainst(reprojectionsBuffer.size() / 2);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D, reprojectionsBuffer.data(), sizeof(dwVector2f), 0, reprojectionsBuffer.size(), m_renderEngine);

        dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
        dwRenderEngine_setColor(DW_RENDERER_COLOR_YELLOW, m_renderEngine);
        checkBufferSizeAgainst(predictionLinesBuffer.size() / 4);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_2D, predictionLinesBuffer.data(), sizeof(dwVector2f), 0, predictionLinesBuffer.size() / 2, m_renderEngine);
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

        for (auto& camera : cameras)
            releaseFrame(camera);

        while (!isAllCamerasReady())
        {
            const dwSensorEvent* event;
            {
                dwStatus status;
                status = dwSensorManager_acquireNextEvent(&event, 0, sensorManager);
                if (status == DW_END_OF_STREAM)
                {
                    reset();
                    return;
                }
            }

            // Process event
            switch (event->type)
            {
            case DW_SENSOR_CAMERA:
            {
                processCamera(*event);
                break;
            }
            case DW_SENSOR_CAN:
            {
                processCan(*event);
                break;
            }
            case DW_SENSOR_LIDAR:
            case DW_SENSOR_GPS:
            case DW_SENSOR_IMU:
            case DW_SENSOR_RADAR:
            case DW_SENSOR_TIME:
            case DW_SENSOR_ULTRASONIC:
            case DW_SENSOR_COUNT:
            case DW_SENSOR_DATA:
            default:
                throw std::runtime_error("Unexpected event type");
            }

            CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(event, sensorManager));
        }

        updatePose();

        getProfilerCUDA()->collectTimers();
    }

    void processCamera(const dwSensorEvent& event)
    {
        if (event.numCamFrames != CAMERA_COUNT)
            throw std::runtime_error("All cameras should come in simultaneously");

        for (size_t k = 0; k < event.numCamFrames; k++)
        {
            auto& camera = cameras[k];

            // Get CUDA & GL images

            CHECK_DW_ERROR(dwSensorCamera_getImage(&camera.frameCudaYuv, DW_CAMERA_OUTPUT_CUDA_YUV420_UINT8_SEMIPLANAR, event.camFrames[k]));
            CHECK_DW_ERROR(dwSensorCamera_getImage(&camera.frameCudaRgba, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, event.camFrames[k]));

            CHECK_DW_ERROR(dwImageStreamerGL_producerSend(camera.frameCudaRgba, camera.image2GL));
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&camera.frameGL, 30000, camera.image2GL));
        }
    }

    dwFeatureArray trackFrame(size_t cameraIdx, const dwTransformation3f& predictedRig2World)
    {
        auto& camera = cameras[cameraIdx];

        std::swap(camera.pyramidCurrent, camera.pyramidPrevious);

        if (doFeaturePrediction)
        {
            //Predict
            ProfileCUDASection s(getProfilerCUDA(), "Predict");

            CHECK_DW_ERROR(dwReconstructor_predictFeaturePosition(camera.d_predictedFeatureLocations,
                                                                  cameraIdx,
                                                                  &currentRig2World,
                                                                  &predictedRig2World,
                                                                  camera.featureDetectedGPU.featureCount,
                                                                  camera.featureDetectedGPU.statuses,
                                                                  camera.featureDetectedGPU.locations,
                                                                  camera.d_worldPoints,
                                                                  reconstructor));
        }
        else
        {
            CHECK_CUDA_ERROR(cudaMemcpyAsync(camera.d_predictedFeatureLocations,
                                             camera.featureDetectedGPU.locations,
                                             maxFeatureCount * sizeof(dwVector2f),
                                             cudaMemcpyDeviceToDevice,
                                             cudaStream_t(0)));
        }

        //Build pyramid
        {
            ProfileCUDASection s(getProfilerCUDA(), "Pyramid");

            dwImageCUDA* img;
            dwImage_getCUDA(&img, camera.frameCudaYuv);
            CHECK_DW_ERROR(dwImageFilter_computePyramid(&camera.pyramidCurrent, img, 0, m_context));
        }

        dwFeatureArray featuresTracked{};
        {
            ProfileCUDASection s(getProfilerCUDA(), "Tracking");
            CHECK_DW_ERROR(dwFeature2DTracker_trackFeatures(&camera.featureHistoryGPU, &featuresTracked,
                                                            nullptr, &camera.featureDetectedGPU,
                                                            camera.d_predictedFeatureLocations,
                                                            &camera.pyramidPrevious, &camera.pyramidCurrent,
                                                            camera.tracker));
        }

        return featuresTracked;
    }

    void copyDataFromGPU(CameraData& camera)
    {
        //Get tracked feature info to CPU
        ProfileCUDASection s(getProfilerCUDA(), "downloadToCPU");
        CHECK_DW_ERROR(dwFeatureHistoryArray_copyAsync(&camera.featureHistoryCPU, &camera.featureHistoryGPU, 0));

        CHECK_CUDA_ERROR(cudaMemcpyAsync(camera.predictedFeatureLocations.get(), camera.d_predictedFeatureLocations, maxFeatureCount * sizeof(dwVector2f), cudaMemcpyDeviceToHost, cudaStream_t(0)));
        CHECK_CUDA_ERROR(cudaMemcpyAsync(camera.projectedLocations.get(), camera.d_projectedLocations, maxFeatureCount * sizeof(dwVector2f), cudaMemcpyDeviceToHost, cudaStream_t(0)));
        CHECK_CUDA_ERROR(cudaMemcpyAsync(camera.worldPoints.get(), camera.d_worldPoints, maxFeatureCount * sizeof(dwVector4f), cudaMemcpyDeviceToHost, cudaStream_t(0)));
        cudaStreamSynchronize(cudaStream_t(0));
    }

    void processCan(const dwSensorEvent& event)
    {
        CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&event.canFrame, 0, vehicleIO));

        dwVehicleIOSafetyState vehicleIOSafeState{};
        dwVehicleIONonSafetyState vehicleIONonSafeState{};
        dwVehicleIOActuationFeedback vehicleIOActuationFeedbackState{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&vehicleIOSafeState, vehicleIO));
        CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&vehicleIONonSafeState, vehicleIO));
        CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&vehicleIOActuationFeedbackState, vehicleIO));
        CHECK_DW_ERROR(dwEgomotion_addVehicleIOState(&vehicleIOSafeState, &vehicleIONonSafeState, &vehicleIOActuationFeedbackState, egomotion));
    }

    bool isAllCamerasReady()
    {
        for (auto& camera : cameras)
        {
            if (!camera.frameCudaYuv)
                return false;
        }
        return true;
    }
    void updatePose()
    {
        dwTime_t now;
        CHECK_DW_ERROR(dwImage_getTimestamp(&now, cameras[0].frameCudaYuv));

        // update current absolute estimate of the pose using relative motion between now and last time
        {
            dwTransformation3f rigLast2rigNow;
            if (DW_SUCCESS == dwEgomotion_computeRelativeTransformation(&rigLast2rigNow, nullptr, lastEgoUpdate, now, egomotion))
            {
                // compute absolute pose given the relative motion between two last estimates
                dwTransformation3f rigNow2World;
                dwEgomotion_applyRelativeTransformation(&rigNow2World, &rigLast2rigNow, &egoRig2World);
                egoRig2World = rigNow2World;
            }
        }
        lastEgoUpdate = now;

        dwTransformation3f invPreviousEgoRig2World;
        Mat4_IsoInv(invPreviousEgoRig2World.array, previousEgoRig2World.array);

        dwTransformation3f egoLastRig2PredictedRig;
        Mat4_AxB(egoLastRig2PredictedRig.array, invPreviousEgoRig2World.array, egoRig2World.array);

        dwTransformation3f predictedRig2World;
        Mat4_AxB(predictedRig2World.array, currentRig2World.array, egoLastRig2PredictedRig.array);

        // Features
        const dwVector2f* d_trackedLocations[CAMERA_COUNT];
        for (size_t k = 0; k < CAMERA_COUNT; k++)
        {
            dwFeatureArray featuresTracked = trackFrame(k, predictedRig2World);
            d_trackedLocations[k]          = featuresTracked.locations;
        }

        // Refine
        if (doPoseRefinement)
        {
            poseColor = DW_RENDERER_COLOR_DARKRED;

            {
                ProfileCUDASection s(getProfilerCUDA(), "PoseEstimation");

                CHECK_DW_ERROR(dwReconstructor_estimatePoseAsync(&currentRig2World,
                                                                 &predictedRig2World,
                                                                 CAMERA_COUNT,
                                                                 d_allFeatureCount,
                                                                 d_allFeatureStatuses,
                                                                 d_trackedLocations,
                                                                 d_allWorldPoints,
                                                                 reconstructor));
            }

            //Blocking memcpy
            {
                ProfileCUDASection s(getProfilerCUDA(), "copyPose2CPU");
                CHECK_DW_ERROR(dwReconstructor_getEstimatedPose(&currentRig2World, reconstructor));
            }
        }
        else
        {
            poseColor        = DW_RENDERER_COLOR_DARKGREEN;
            currentRig2World = predictedRig2World;
        }

        // Store pose
        recordPose(predictedRig2World, currentRig2World);

        previousEgoRig2World = egoRig2World;

        //Update feature history
        {
            ProfileCUDASection s(getProfilerCUDA(), "History3D");
            int32_t currentPoseIdx;
            CHECK_DW_ERROR(dwReconstructor_updateHistory(&currentPoseIdx,
                                                         &currentRig2World,
                                                         CAMERA_COUNT,
                                                         d_allFeatureCount,
                                                         d_trackedLocations,
                                                         reconstructor));
        }

        for (size_t k = 0; k < CAMERA_COUNT; k++)
        {
            auto& camera = cameras[k];

            //Triangulate
            ProfileCUDASection s(getProfilerCUDA(), "Triangulation");
            CHECK_DW_ERROR(dwReconstructor_triangulateFeatures(
                camera.d_worldPoints,
                camera.featureHistoryGPU.statuses,
                camera.featureHistoryGPU.featureCount,
                k, reconstructor));
        }

        //Project back onto camera for display
        {
            ProfileCUDASection s(getProfilerCUDA(), "Reproject");
            CHECK_DW_ERROR(dwReconstructor_project(d_allProjections,
                                                   &currentRig2World,
                                                   d_allFeatureCount,
                                                   d_allWorldPoints,
                                                   reconstructor));
        }

        for (size_t k = 0; k < CAMERA_COUNT; k++)
        {
            copyDataFromGPU(cameras[k]);
            compactAndDetect(k);
        }
    }

    void releaseFrame(CameraData& camera)
    {
        if (camera.frameGL)
        {
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&camera.frameGL, camera.image2GL));
            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 33000, camera.image2GL));
        }

        camera.frameCudaYuv = nullptr;
        camera.frameGL      = nullptr;
        camera.frame        = nullptr;
    }

    void compactAndDetect(size_t cameraIdx)
    {
        ProfileCUDASection s(getProfilerCUDA(), "compactFeature");

        auto& camera = cameras[cameraIdx];

        // Determine which features to throw away
        {
            ProfileCUDASection s(getProfilerCUDA(), "compactFeatureHistory");
            CHECK_DW_ERROR(dwFeature2DTracker_compact(&camera.featureHistoryGPU, camera.tracker));
        }

        // Compact SFM data
        {
            ProfileCUDASection s(getProfilerCUDA(), "compactSFM");
            CHECK_DW_ERROR(dwReconstructor_compactFeatureHistory(
                cameraIdx, camera.featureHistoryGPU.featureCount,
                camera.featureHistoryGPU.newToOldMap, reconstructor));
            CHECK_DW_ERROR(dwReconstructor_compactWorldPoints(
                camera.d_worldPoints, camera.featureHistoryGPU.featureCount,
                camera.featureHistoryGPU.newToOldMap, reconstructor));
        }

        // Detect new features
        {
            ProfileCUDASection s(getProfilerCUDA(), "detectNewFeatures");
            dwFeatureArray lastTrackedFeatures{};
            dwFeatureHistoryArray_getCurrent(&lastTrackedFeatures, &camera.featureHistoryGPU);
            CHECK_DW_ERROR(dwFeature2DDetector_detectFromPyramid(
                &camera.featureDetectedGPU, &camera.pyramidCurrent,
                &lastTrackedFeatures, nullptr, camera.detector));
        }
    }

    //------------------------------------------------------------------------------
    void recordPose(const dwTransformation3f& poseCAN, const dwTransformation3f& poseRefined)
    {
        dwVector3f positionCAN     = {poseCAN.array[0 + 3 * 4], poseCAN.array[1 + 3 * 4], poseCAN.array[2 + 3 * 4]};
        dwVector3f positionRefined = {poseRefined.array[0 + 3 * 4], poseRefined.array[1 + 3 * 4], poseRefined.array[2 + 3 * 4]};

        positionsCAN.push_back(positionCAN);
        positionsRefined.push_back(positionRefined);
        positionsDifference.push_back(positionCAN);
        positionsDifference.push_back(positionRefined);
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{

    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("baseDir", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation").c_str()),
                           ProgramArguments::Option_t("rig", "rig.json"),
                           ProgramArguments::Option_t("maxFeatureCount", "2000"),
                           ProgramArguments::Option_t("trackMode", "0"),
                           ProgramArguments::Option_t("useHalf", "0"),
                           ProgramArguments::Option_t("displacementThreshold", "0.001"),
                           ProgramArguments::Option_t("enableAdaptiveWindow", "0")},
                          "SfM sample.");

    // -------------------
    // initialize and start a window application
    SfmSample app(args);

    app.initializeWindow("SfM Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
