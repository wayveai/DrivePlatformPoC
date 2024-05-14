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
// SPDX-FileCopyrightText: Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// SDK
#include <dw/core/base/Version.h>
#include <dw/calibration/engine/Engine.h>
#include <dw/sensors/sensormanager/SensorManager.h>
#include <dw/imageprocessing/featuredetector/FeatureDetector.h>
#include <dw/imageprocessing/tracking/featuretracker/FeatureTracker.h>

//Sample common files
#include <framework/DriveWorksSample.hpp>
#include <framework/Checks.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/MathUtils.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Mat4.hpp>

using namespace dw_samples::common;

class StereoCalibrationSample : public DriveWorksSample
{
private:
    //Modules
    dwContextHandle_t m_context                       = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz              = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine             = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                               = DW_NULL_HANDLE;
    dwRigHandle_t m_rig                               = DW_NULL_HANDLE;
    dwCameraModelHandle_t m_calibratedCameraLeft      = DW_NULL_HANDLE;
    dwCameraModelHandle_t m_calibratedCameraRight     = DW_NULL_HANDLE;
    dwCalibrationRoutineHandle_t m_calibrationRoutine = DW_NULL_HANDLE;
    dwCalibrationEngineHandle_t m_calibrationEngine   = DW_NULL_HANDLE;
    dwSensorManagerHandle_t m_sensorManager           = DW_NULL_HANDLE;

    // Calibration Info
    uint32_t m_vehicleSensorIdx     = std::numeric_limits<decltype(m_vehicleSensorIdx)>::max();
    uint32_t m_cameraLeftSensorIdx  = std::numeric_limits<decltype(m_cameraLeftSensorIdx)>::max();
    uint32_t m_cameraRightSensorIdx = std::numeric_limits<decltype(m_cameraRightSensorIdx)>::max();
    dwCalibrationStatus m_status;
    dwTransformation3f m_nominalLeft2Right;
    dwTransformation3f m_nominalRight2Left;
    dwTransformation3f m_calibratedLeft2Right;
    dwTransformation3f m_correctionR = {};
    dwVector3f m_correctionT         = {};
    dwVector3f m_correctionRPY       = {};

    //Camera Sensor
    std::unique_ptr<dw_samples::common::CameraFramePipeline> m_framePipelineLeft;
    std::unique_ptr<dw_samples::common::CameraFramePipeline> m_framePipelineRight;
    std::unique_ptr<dw_samples::common::SimpleImageStreamerGL<>> m_streamerGLLeft;
    std::unique_ptr<dw_samples::common::SimpleImageStreamerGL<>> m_streamerGLRight;
    dwVector2f m_cameraDimensions;
    dwImageHandle_t m_pendingFrame, m_pendingFrameLeft, m_pendingFrameRight;
    dwImageGL* m_pendingFrameGLLeft  = nullptr;
    dwImageGL* m_pendingFrameGLRight = nullptr;
    dwTime_t m_lastEventTime         = {}; // Time stamp of the first processed sensor event
    dwTime_t m_firstEventTime        = {}; // Time stamp of last processed sensor event
    dwTime_t m_offsetTime            = {}; // Global time offset to enable reset/looping (simulating a continuous dataset)

    static constexpr uint32_t NUM_VIDEOS = 2;
    uint32_t m_tiles[NUM_VIDEOS];

    // Feature Matcher
    static constexpr uint32_t HISTORY_CAPACITY     = 2;
    static constexpr uint32_t PYRAMID_LEVEL_COUNT  = 7;
    static constexpr uint32_t WINDOW_SIZE_LK       = 12;
    static constexpr uint32_t NUM_ITER_TRANSLATION = 50;

    uint32_t m_maxMatchesCount;
    dwFeature2DDetectorHandle_t m_featureDetector = DW_NULL_HANDLE;
    dwFeature2DTrackerHandle_t m_featureMatcher   = DW_NULL_HANDLE;
    dwFeatureHistoryArray m_matchesHistoryGPU     = {};
    dwFeatureArray m_matchesDetectedGPU           = {};
    dwPyramidImage m_pyramidPrevious              = {};
    dwPyramidImage m_pyramidCurrent               = {};

    //Rendering
    const dwVector4f m_colorBlue  = {32.0f / 255.0f, 72.0f / 255.0f, 230.0f / 255.0f, 1.0f};
    const dwVector4f m_colorGreen = {32.0f / 255.0f, 230.0f / 255.0f, 32.0f / 255.0f, 1.0f};
    const dwVector4f m_colorWhite = {230.0f / 255.0f, 230.0f / 255.0f, 230.0f / 255.0f, 1.0f};
    const dwVector4f m_colorRed   = {1.0, 32.0f / 255.0f, 32.0f / 255.0f, 1.0f};

    uint32_t m_pointBufferId, m_lineBufferId;
    bool m_leftPointActive = true;
    dwVector2f m_leftPoint;
    std::vector<dwVector2f> m_rightPointsNominal;
    std::vector<dwVector2f> m_rightPointsCalibrated;

    static constexpr uint32_t LINESPACING  = 20;
    static constexpr uint32_t BORDEROFFSET = 20;
    static constexpr uint32_t NUM_POINTS   = 1000;
    static constexpr float32_t MIN_DEPTH   = 0.1f;
    static constexpr float32_t MAX_DEPTH   = 500.f;

public:
    StereoCalibrationSample(const ProgramArguments& args)
        : DriveWorksSample(args), m_lastEventTime(0), m_firstEventTime(0), m_offsetTime(0)
    {
    }

    // -----------------------------------------------------------------------------
    // Initialize all modules
    bool onInitialize() override
    {
        initializeDriveWorks();

        initializeRig();

        initializeSensorManager();

        initializeCameraSensors();

        initializeFeatureMatcher();

        startSensorManager();

        startCalibration();

        if (getWindow())
        {
            initializeRenderer();
        }

        return true;
    }

    // -----------------------------------------------------------------------------
    // A point is rendered (red) in the top image while the nominal (blue) and estimated (green) epipolar lines on screen.
    // The correctness of the calibration can be verified by checking that the epipolar line passes through
    // the corresponding point in the bottom image.
    // Numerical information regarding calibration are printed in the bottom left corner
    void onRender() override
    {
        if (m_pendingFrameGLLeft)
        {
            // Set top tile
            CHECK_DW_ERROR(dwRenderEngine_setTile(m_tiles[0], m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

            dwVector2f range{};
            range.x = m_pendingFrameGLLeft->prop.width;
            range.y = m_pendingFrameGLLeft->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_pendingFrameGLLeft, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));

            // Render red point in the top image
            if (m_leftPointActive)
            {
                CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_pointBufferId,
                                                        DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D,
                                                        &m_leftPoint,
                                                        sizeof(dwVector2f), 0, 1,
                                                        m_renderEngine));

                CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorRed, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_pointBufferId, 1, m_renderEngine));
            }
        }

        if (m_pendingFrameGLRight)
        {
            // Set bottom tile
            CHECK_DW_ERROR(dwRenderEngine_setTile(m_tiles[1], m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

            dwVector2f range{};
            range.x = m_pendingFrameGLRight->prop.width;
            range.y = m_pendingFrameGLRight->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_pendingFrameGLRight, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));

            // Render nominal (blue) epipolar lines
            if (m_leftPointActive)
            {
                CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorBlue, m_renderEngine));

                CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_lineBufferId,
                                                        DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                        m_rightPointsNominal.data(),
                                                        sizeof(dwVector2f), 0, m_rightPointsNominal.size(),
                                                        m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_lineBufferId, m_rightPointsNominal.size(), m_renderEngine));

                CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_pointBufferId,
                                                        DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D,
                                                        &(m_rightPointsNominal[m_rightPointsNominal.size() - 1]),
                                                        sizeof(dwVector2f), 0, 1,
                                                        m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_pointBufferId, 1, m_renderEngine));

                // If calibration converged, render calibrated (green) epipolar lines
                if (m_status.state == dwCalibrationState::DW_CALIBRATION_STATE_ACCEPTED)
                {
                    computePoints(m_calibratedLeft2Right, m_rightPointsCalibrated);

                    CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorGreen, m_renderEngine));
                    CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_lineBufferId,
                                                            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                            m_rightPointsCalibrated.data(),
                                                            sizeof(dwVector2f), 0, m_rightPointsCalibrated.size(),
                                                            m_renderEngine));
                    CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_lineBufferId, m_rightPointsCalibrated.size(), m_renderEngine));

                    CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_pointBufferId,
                                                            DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D,
                                                            &(m_rightPointsCalibrated[m_rightPointsCalibrated.size() - 1]),
                                                            sizeof(dwVector2f), 0, 1,
                                                            m_renderEngine));
                    CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_pointBufferId, 1, m_renderEngine));
                }
            }
        }

        dwVector2f range = {static_cast<float32_t>(getWindowWidth()), static_cast<float32_t>(getWindowHeight())};

        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));

        //now render our matrices and text to the screen
        CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorBlue, m_renderEngine));

        std::stringstream ss;
        ss << "Rendering: Estimated (Green) + Nominal (Blue)\n"
              "Epipolar segments rendered until infinity at endpoint\n\n";
        ss << convertStatusToText() << "\n\n";

        ss << "Right camera position correction\n\n";
        ss << renderFloatsText(&m_correctionT.x, 3) << "\n\n";

        ss << "Right camera roll-pitch-yaw correction\n\n";
        ss << renderFloatsText(&m_correctionRPY.x, 3) << "\n\n";

        if (m_pendingFrameGLLeft && m_pendingFrameGLRight)
        {
            ss << "Timestamp: " << m_pendingFrameGLLeft->timestamp_us;
        }

        float32_t currentHeight = getWindowHeight() - 15 * LINESPACING;
        CHECK_DW_ERROR(dwRenderEngine_renderText2D(ss.str().c_str(), {BORDEROFFSET, currentHeight}, m_renderEngine));

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    // -----------------------------------------------------------------------------
    // Read the stereo frames and operate on them
    void onProcess() override
    {
        const dwSensorEvent* event = nullptr;
        m_pendingFrame = m_pendingFrameLeft = m_pendingFrameRight = nullptr;
        while (!m_pendingFrame)
        {
            dwCameraFrameHandle_t frames[2] = {DW_NULL_HANDLE, DW_NULL_HANDLE};
            dwStatus res                    = dwSensorManager_acquireNextEvent(&event, 1000, m_sensorManager);
            if (res == DW_SUCCESS)
            {
                switch (event->type)
                {
                case DW_SENSOR_CAMERA:
                    // Pass frames in left-right order
                    for (uint32_t i = 0; i < event->numCamFrames; ++i)
                    {
                        uint32_t idx = event->sensorIndices[i];
                        if (idx == m_cameraLeftSensorIdx)
                            frames[0] = event->camFrames[i];
                        else if (idx == m_cameraRightSensorIdx)
                            frames[1] = event->camFrames[i];
                    }
                    handleCameras(frames);

                    break;
                case DW_SENSOR_IMU:
                case DW_SENSOR_CAN:
                case DW_SENSOR_GPS:
                case DW_SENSOR_RADAR:
                case DW_SENSOR_TIME:
                case DW_SENSOR_LIDAR:
                case DW_SENSOR_DATA:
                case DW_SENSOR_COUNT:
                default:
                    break;
                }
            }
            else if (res == DW_END_OF_STREAM)
            {
                if (event != nullptr)
                {
                    CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(event, m_sensorManager));
                }

                pause();

                return;
            }

            if (!shouldRun())
            {
                if (event != nullptr)
                {
                    CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(event, m_sensorManager));
                }

                return;
            }
        }

        // Match features
        {
            resetMatcher();
            processImage(m_pendingFrameLeft);
            processImage(m_pendingFrameRight);

            // Feed features to calibration
            dwTime_t timestamp;
            CHECK_DW_ERROR(dwImage_getTimestamp(&timestamp, m_pendingFrameLeft));
            timestamp += m_offsetTime;
            CHECK_DW_ERROR(dwCalibrationEngine_addMatches(&m_matchesHistoryGPU, timestamp, m_cameraLeftSensorIdx, m_cameraRightSensorIdx, m_calibrationEngine));
            // Update timestamps
            if (!m_firstEventTime)
            {
                m_firstEventTime = timestamp;
            }
            m_lastEventTime = timestamp;
            // Compute calibration results
            computeResults();
        }

        // Stream for rendering
        dwImageHandle_t imgGL;
        if (m_streamerGLLeft && m_streamerGLRight)
        {
            if (m_pendingFrameLeft)
            {
                imgGL = m_streamerGLLeft->post(m_framePipelineLeft->getFrameRgba());
                CHECK_DW_ERROR(dwImage_getGL(&m_pendingFrameGLLeft, imgGL));
            }

            if (m_pendingFrameRight)
            {
                imgGL = m_streamerGLRight->post(m_framePipelineRight->getFrameRgba());
                CHECK_DW_ERROR(dwImage_getGL(&m_pendingFrameGLRight, imgGL));
            }
        }

        if (event != nullptr)
        {
            CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(event, m_sensorManager));
        }
    }

    // -----------------------------------------------------------------------------
    // Click on left image: left click assign point, right click delete point
    void onMouseDown(int32_t button, float32_t x, float32_t y, int32_t /*mods*/) override
    {
        // assign new point in left image by left click
        if (button == GLFW_MOUSE_BUTTON_1)
        {
            uint32_t selectedTile = 0;
            dwVector2f screenPos{x, y};
            dwVector2f screenSize{static_cast<float32_t>(getWindowWidth()), static_cast<float32_t>(getWindowHeight())};

            CHECK_DW_ERROR(dwRenderEngine_getTileByScreenCoordinates(&selectedTile, screenPos, screenSize, m_renderEngine));

            dwVector3f worldPos{};
            CHECK_DW_ERROR(dwRenderEngine_screenToWorld3D(&worldPos, screenPos, screenSize, m_renderEngine));

            if (selectedTile == m_tiles[0])
            {
                m_leftPointActive = true;
                m_leftPoint       = dwVector2f{m_pendingFrameGLLeft->prop.width * (worldPos.x + 1.f) / 2, m_pendingFrameGLLeft->prop.height * (1.f - worldPos.y) / 2};

                computePoints(m_nominalLeft2Right, m_rightPointsNominal);
            }
        }
        // delete point in left image by right click
        else if (button == GLFW_MOUSE_BUTTON_2)
        {
            m_leftPointActive = false;
        }
    }

    // -----------------------------------------------------------------------------
    // Resize window
    void onResizeWindow(int32_t width, int32_t height) override
    {
        CHECK_DW_ERROR(dwRenderEngine_reset(m_renderEngine));
        dwRectf rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));
    }

    // -----------------------------------------------------------------------------
    // Release all modules
    void onRelease() override
    {
        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_destroyBuffer(m_pointBufferId, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_destroyBuffer(m_lineBufferId, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        if (m_calibratedCameraLeft)
        {
            CHECK_DW_ERROR(dwCameraModel_release(m_calibratedCameraLeft));
        }

        if (m_calibratedCameraRight)
        {
            CHECK_DW_ERROR(dwCameraModel_release(m_calibratedCameraRight));
        }

        if (m_calibrationEngine)
        {
            CHECK_DW_ERROR(dwCalibrationEngine_stopCalibration(m_calibrationRoutine, m_calibrationEngine))
            CHECK_DW_ERROR(dwCalibrationEngine_release(m_calibrationEngine));
        }

        if (m_rig)
        {
            CHECK_DW_ERROR(dwRig_release(m_rig));
        }

        if (m_sensorManager)
        {
            CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
            CHECK_DW_ERROR(dwSensorManager_release(m_sensorManager));
        }

        if (m_sal)
        {
            CHECK_DW_ERROR(dwSAL_release(m_sal));
        }

        if (m_featureDetector)
        {
            CHECK_DW_ERROR(dwFeature2DDetector_release(m_featureDetector));
        }

        if (m_featureMatcher)
        {
            CHECK_DW_ERROR(dwFeature2DTracker_release(m_featureMatcher));
        }

        if (m_viz)
        {
            CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        }

        m_streamerGLLeft.reset();
        m_streamerGLRight.reset();
        m_framePipelineLeft.reset();
        m_framePipelineRight.reset();

        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    // -----------------------------------------------------------------------------
    // Reset egomotion, sensor manager, and gravity measurements
    void onReset() override
    {
        // reset sensor manager
        CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_reset(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));

        m_offsetTime += m_lastEventTime - m_firstEventTime;
        m_lastEventTime  = 0;
        m_firstEventTime = 0;
    }

private:
    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks()
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};

#ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
#endif

        CHECK_DW_ERROR(dwInitialize(&m_context, DW_VERSION, &sdkParams));
    }

    // -----------------------------------------------------------------------------
    // Initialize rig configuration
    // Initialize calibration engine
    void initializeRig()
    {
        //initialize our rig configuration module
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rig, m_context, getArgument("rig").c_str()));

        try
        {
            uint32_t sensorIdx = std::stol(getArgument("vehicle-sensor").c_str());
            // Use numeric idx
            // Start looking for CAN *first*, then DATA sensor
            if (dwRig_findSensorByTypeIndex(&m_vehicleSensorIdx, DW_SENSOR_CAN, sensorIdx, m_rig) != DW_SUCCESS)
            {
                // Try to find a general data sensor in the rig file if n
                CHECK_DW_ERROR_MSG(dwRig_findSensorByTypeIndex(&m_vehicleSensorIdx, DW_SENSOR_DATA, sensorIdx, m_rig),
                                   std::string{"Neither CAN nor DATA sensor found for sensor type index "} + std::to_string(sensorIdx));
            }
        }
        catch (const std::invalid_argument& /* string to integer conversion failed - use string as sensor name */)
        {
            // Determine camera sensor index using sensor name
            CHECK_DW_ERROR(dwRig_findSensorByName(&m_vehicleSensorIdx, getArgument("vehicle-sensor").c_str(), m_rig));
        }

        try
        {
            uint32_t sensorIdx = std::stol(getArgument("camera-sensor-left").c_str());
            // Determine camera sensor index using numeric sensor index
            CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&m_cameraLeftSensorIdx, DW_SENSOR_CAMERA, sensorIdx, m_rig));
        }
        catch (const std::invalid_argument& /* string to integer conversion failed - use string as sensor name */)
        {
            // Determine camera sensor index using sensor name
            CHECK_DW_ERROR(dwRig_findSensorByName(&m_cameraLeftSensorIdx, getArgument("camera-sensor-left").c_str(), m_rig));
        }

        try
        {
            uint32_t sensorIdx = std::stol(getArgument("camera-sensor-right").c_str());
            // Determine camera sensor index using numeric sensor index
            CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&m_cameraRightSensorIdx, DW_SENSOR_CAMERA, sensorIdx, m_rig));
        }
        catch (const std::invalid_argument& /* string to integer conversion failed - use string as sensor name */)
        {
            // Determine camera sensor index using sensor name
            CHECK_DW_ERROR(dwRig_findSensorByName(&m_cameraRightSensorIdx, getArgument("camera-sensor-right").c_str(), m_rig));
        }

        // initialize calibration engine
        CHECK_DW_ERROR(dwCalibrationEngine_initialize(&m_calibrationEngine, m_rig, m_context));
    }

    // -----------------------------------------------------------------------------
    // Initialize camera sensors
    void initializeCameraSensors()
    {
        // Left camera
        CHECK_DW_ERROR(dwCameraModel_initialize(&m_calibratedCameraLeft, m_cameraLeftSensorIdx, m_rig));

        dwSensorHandle_t cameraSensorLeft = DW_NULL_HANDLE;
        CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&cameraSensorLeft, m_cameraLeftSensorIdx, m_sensorManager));

        dwImageProperties imagePropertiesLeft{};
        CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imagePropertiesLeft, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraSensorLeft));

        m_framePipelineLeft.reset(new dw_samples::common::CameraFramePipeline(imagePropertiesLeft, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_context));

        dwImageProperties outputPropertiesLeft = imagePropertiesLeft;
        outputPropertiesLeft.format            = DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR;
        outputPropertiesLeft.type              = DW_IMAGE_CUDA;

        m_framePipelineLeft->setOutputProperties(outputPropertiesLeft);

        // check that intrinsics and sensor resolution match
        uint32_t width, height;
        dwCameraModel_getImageSize(&width, &height, m_calibratedCameraLeft);
        if (outputPropertiesLeft.width != width || outputPropertiesLeft.height != height)
            throw std::runtime_error(std::string("Left camera intrinsic and sensor resolutions mismatch"));

        // Right camera
        CHECK_DW_ERROR(dwCameraModel_initialize(&m_calibratedCameraRight, m_cameraRightSensorIdx, m_rig));

        dwSensorHandle_t cameraSensorRight = DW_NULL_HANDLE;
        CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&cameraSensorRight, m_cameraRightSensorIdx, m_sensorManager));

        dwImageProperties imagePropertiesRight{};
        CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imagePropertiesRight, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraSensorRight));

        m_framePipelineRight.reset(new dw_samples::common::CameraFramePipeline(imagePropertiesRight, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_context));

        dwImageProperties outputPropertiesRight = imagePropertiesRight;
        outputPropertiesRight.format            = DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR;
        outputPropertiesRight.type              = DW_IMAGE_CUDA;

        m_framePipelineRight->setOutputProperties(outputPropertiesRight);

        // check that intrinsics and sensor resolutions match
        CHECK_DW_ERROR(dwCameraModel_getImageSize(&width, &height, m_calibratedCameraRight));
        if (outputPropertiesRight.width != width || outputPropertiesRight.height != height)
            throw std::runtime_error(std::string("Right camera intrinsic and sensor resolutions mismatch"));

        // check that left and right resolutions match
        if (outputPropertiesLeft.width != outputPropertiesRight.width || outputPropertiesLeft.height != outputPropertiesRight.height)
            throw std::runtime_error(std::string("Left and Right camera resolutions mismatch"));

        m_cameraDimensions.x = outputPropertiesLeft.width;
        m_cameraDimensions.y = outputPropertiesLeft.height;
    }

    // -----------------------------------------------------------------------------
    // Initialize feature matcher
    void initializeFeatureMatcher()
    {
        // Initialize feature list
        m_maxMatchesCount = static_cast<uint32_t>(std::stoi(getArgument("matches-max-count")));

        CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&m_matchesHistoryGPU, m_maxMatchesCount, HISTORY_CAPACITY, DW_MEMORY_TYPE_CUDA, nullptr, m_context));
        CHECK_DW_ERROR(dwFeatureArray_createNew(&m_matchesDetectedGPU, m_maxMatchesCount, DW_MEMORY_TYPE_CUDA, nullptr, m_context));

        // Pyramids
        CHECK_DW_ERROR(dwPyramid_create(&m_pyramidPrevious, PYRAMID_LEVEL_COUNT, m_cameraDimensions.x, m_cameraDimensions.y, DW_TYPE_UINT8, m_context));
        CHECK_DW_ERROR(dwPyramid_create(&m_pyramidCurrent, PYRAMID_LEVEL_COUNT, m_cameraDimensions.x, m_cameraDimensions.y, DW_TYPE_UINT8, m_context));

        // Detector
        dwFeature2DDetectorConfig detectorConfig = {};
        CHECK_DW_ERROR(dwFeature2DDetector_initDefaultParams(&detectorConfig));
        detectorConfig.imageWidth      = m_cameraDimensions.x;
        detectorConfig.imageHeight     = m_cameraDimensions.y;
        detectorConfig.maxFeatureCount = m_maxMatchesCount;
        CHECK_DW_ERROR(dwFeature2DDetector_initialize(&m_featureDetector, &detectorConfig, cudaStream_t(0), m_context));

        // Matcher
        dwFeature2DTrackerConfig matcherConfig = {};
        CHECK_DW_ERROR(dwFeature2DTracker_initDefaultParams(&matcherConfig));
        matcherConfig.imageWidth             = m_cameraDimensions.x;
        matcherConfig.imageHeight            = m_cameraDimensions.y;
        matcherConfig.maxFeatureCount        = m_maxMatchesCount;
        matcherConfig.historyCapacity        = HISTORY_CAPACITY;
        matcherConfig.windowSizeLK           = WINDOW_SIZE_LK;
        matcherConfig.pyramidLevelCount      = PYRAMID_LEVEL_COUNT;
        matcherConfig.numIterTranslationOnly = NUM_ITER_TRANSLATION;
        CHECK_DW_ERROR(dwFeature2DTracker_initialize(&m_featureMatcher, &matcherConfig, cudaStream_t(0), m_context));
    }

    // -----------------------------------------------------------------------------
    // Initialize sensor manager
    void initializeSensorManager()
    {
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

        dwSensorManagerParams smParams{};

        smParams.enableSensors[0] = m_cameraLeftSensorIdx;
        smParams.numEnableSensors++;

        smParams.enableSensors[1] = m_cameraRightSensorIdx;
        smParams.numEnableSensors++;

        // initialize Driveworks SensorManager directly from rig file
        CHECK_DW_ERROR(dwSensorManager_initializeFromRigWithParams(&m_sensorManager, m_rig, &smParams, 1024, m_sal));
    }

    // -----------------------------------------------------------------------------
    // Initialize render engine
    void initializeRenderer()
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        m_framePipelineLeft->enableRgbaOutput();
        m_framePipelineRight->enableRgbaOutput();
        m_streamerGLLeft  = std::make_unique<dw_samples::common::SimpleImageStreamerGL<>>(m_framePipelineLeft->getRgbaOutputProperties(), 6000, m_context);
        m_streamerGLRight = std::make_unique<dw_samples::common::SimpleImageStreamerGL<>>(m_framePipelineRight->getRgbaOutputProperties(), 6000, m_context);

        // Setup render engine
        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        // Setup tiles for rendering
        dwRenderEngineTileState paramList[NUM_VIDEOS];
        for (uint32_t i = 0; i < NUM_VIDEOS; ++i)
        {
            CHECK_DW_ERROR(dwRenderEngine_initTileState(&paramList[i]));
            paramList[i].lineWidth = 1.0f;
            paramList[i].pointSize = 4.0f;
            paramList[i].font      = DW_RENDER_ENGINE_FONT_VERDANA_16;
        }
        dwRenderEngine_addTilesByCount(m_tiles, NUM_VIDEOS, 1, paramList, m_renderEngine);

        CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_pointBufferId,
                                                   DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D,
                                                   sizeof(dwVector2f),
                                                   0,
                                                   1,
                                                   m_renderEngine));

        CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_lineBufferId,
                                                   DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                   sizeof(dwVector2f),
                                                   0,
                                                   NUM_POINTS,
                                                   m_renderEngine));

        glDepthFunc(GL_ALWAYS);
    }

    // -----------------------------------------------------------------------------
    // Start sensor manager for handling cameras
    void startSensorManager()
    {
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));
    }

    // -----------------------------------------------------------------------------
    // Initialize stereo calibration engine
    // Start calibration
    void startCalibration()
    {
        // Initialize Stereo calibration engine
        dwCalibrationStereoParams stereoParams{};
        stereoParams.epipolar.maxMatchesCount = m_maxMatchesCount;
        CHECK_DW_ERROR(dwCalibrationEngine_initializeStereo(&m_calibrationRoutine, m_vehicleSensorIdx, m_cameraLeftSensorIdx, m_cameraRightSensorIdx, &stereoParams, cudaStream_t(0), m_calibrationEngine));

        // Get stereo nominal calibration to invert it
        CHECK_DW_ERROR(dwRig_getSensorToSensorTransformation(&m_nominalLeft2Right, m_cameraLeftSensorIdx, m_cameraRightSensorIdx, m_rig));
        Mat4_IsoInv(m_nominalRight2Left.array, m_nominalLeft2Right.array);

        // Start calibration
        CHECK_DW_ERROR(dwCalibrationEngine_startCalibration(m_calibrationRoutine, m_calibrationEngine))

        // Draw the first point in the middle of the left image and compute corresponding epipolar segment in the right image
        m_leftPoint = dwVector2f{m_cameraDimensions.x / 2, m_cameraDimensions.y / 2};
        computePoints(m_nominalLeft2Right, m_rightPointsNominal);
    }

    // -----------------------------------------------------------------------------
    // Get stereo images
    void handleCameras(dwCameraFrameHandle_t frames[2])
    {
        // Left frame
        if (frames[0] != DW_NULL_HANDLE)
        {
            m_framePipelineLeft->processFrame(frames[0]);
            m_pendingFrameLeft = m_framePipelineLeft->getFrame();
            m_pendingFrame     = m_pendingFrameLeft;
        }
        // Right frame
        if (frames[1] != DW_NULL_HANDLE)
        {
            m_framePipelineRight->processFrame(frames[1]);
            m_pendingFrameRight = m_framePipelineRight->getFrame();
            m_pendingFrame      = m_pendingFrameRight;
        }

        dwTime_t timestampLeft, timestampRight;
        CHECK_DW_ERROR(dwImage_getTimestamp(&timestampLeft, m_pendingFrameLeft));
        CHECK_DW_ERROR(dwImage_getTimestamp(&timestampRight, m_pendingFrameRight));

        if (timestampLeft != timestampRight)
        {
            throw std::runtime_error("Left and right timestamps must have the same value. Cameras are not synchronized.");
        }
    }

    // -----------------------------------------------------------------------------
    // Feature matcher must be reset for every pair of left-right frames to avoid matching over consecutive frames
    void resetMatcher()
    {
        CHECK_DW_ERROR(dwFeatureHistoryArray_reset(&m_matchesHistoryGPU, cudaStream_t(0)));
        CHECK_DW_ERROR(dwFeatureArray_reset(&m_matchesDetectedGPU, cudaStream_t(0)));

        CHECK_DW_ERROR(dwFeature2DDetector_reset(m_featureDetector));
        CHECK_DW_ERROR(dwFeature2DTracker_reset(m_featureMatcher));
    }

    // -----------------------------------------------------------------------------
    // Extract features and find matches
    void processImage(dwImageHandle_t imageYuv)
    {
        // Update pyramid
        dwImageCUDA* imageYuvCUDA;
        CHECK_DW_ERROR(dwImage_getCUDA(&imageYuvCUDA, imageYuv));

        std::swap(m_pyramidCurrent, m_pyramidPrevious);
        CHECK_DW_ERROR(dwImageFilter_computePyramid(&m_pyramidCurrent, imageYuvCUDA, cudaStream_t(0), m_context));

        // Match
        dwFeatureArray featurePredicted{};
        CHECK_DW_ERROR(dwFeature2DTracker_trackFeatures(&m_matchesHistoryGPU, &featurePredicted,
                                                        nullptr, &m_matchesDetectedGPU, nullptr,
                                                        &m_pyramidPrevious, &m_pyramidCurrent, m_featureMatcher));

        // Detect
        CHECK_DW_ERROR(dwFeature2DDetector_detectFromPyramid(&m_matchesDetectedGPU, &m_pyramidCurrent,
                                                             &featurePredicted, nullptr, m_featureDetector));
    }

    // -----------------------------------------------------------------------------
    // Compute calibration results and status
    void computeResults()
    {
        // Get stereo calibration transformation
        CHECK_DW_ERROR(dwCalibrationEngine_getSensorToSensorTransformation(&m_calibratedLeft2Right, m_cameraLeftSensorIdx, m_cameraRightSensorIdx, m_calibrationRoutine, m_calibrationEngine))

        // Compute correction for rotation and translation
        // Factorization: left2right = [correctionT] * left2RightNominal * [correctionR]
        Mat4_AxB(m_correctionR.array, m_nominalRight2Left.array, m_calibratedLeft2Right.array);
        m_correctionR.array[3 * 4] = m_correctionR.array[3 * 4 + 1] = m_correctionR.array[3 * 4 + 2] = 0.f;
        m_correctionT.x                                                                              = m_calibratedLeft2Right.array[3 * 4] - m_nominalLeft2Right.array[3 * 4];
        m_correctionT.y                                                                              = m_calibratedLeft2Right.array[3 * 4 + 1] - m_nominalLeft2Right.array[3 * 4 + 1];
        m_correctionT.z                                                                              = m_calibratedLeft2Right.array[3 * 4 + 2] - m_nominalLeft2Right.array[3 * 4 + 2];

        m_correctionRPY.x = RAD2DEG(std::atan2(m_correctionR.array[6], m_correctionR.array[10]));
        m_correctionRPY.y = RAD2DEG(std::atan2(-m_correctionR.array[2], std::sqrt(m_correctionR.array[0] * m_correctionR.array[0] + m_correctionR.array[1] * m_correctionR.array[1])));
        m_correctionRPY.z = RAD2DEG(std::atan2(m_correctionR.array[1], m_correctionR.array[0]));

        // Get stereo calibration status
        CHECK_DW_ERROR(dwCalibrationEngine_getCalibrationStatus(&m_status, m_calibrationRoutine, m_calibrationEngine));
    }

    // -----------------------------------------------------------------------------
    // Compute points belonging to epipolar line for rendering nominal/calibrated transformation
    void computePoints(const dwTransformation3f& left2right, std::vector<dwVector2f>& rightPoints)
    {
        rightPoints.clear();

        // We sample points at different depths and we reprojected them in the right image to identify the epipolar line
        float32_t max = MAX_DEPTH;
        float32_t min = MIN_DEPTH;
        float32_t ray[3];
        // Ray corresponding to point in the left image
        CHECK_DW_ERROR(dwCameraModel_pixel2Ray(&ray[0], &ray[1], &ray[2], m_leftPoint.x, m_leftPoint.y, m_calibratedCameraLeft));
        for (uint32_t di = 0; di < NUM_POINTS; ++di)
        {
            float32_t depth = (max - min) * (di / float32_t(NUM_POINTS - 1)) + min;
            // 3D Point in the left image along the ray at a given depth
            float32_t xa[3] = {ray[0] * depth, ray[1] * depth, ray[2] * depth};
            float32_t xb[3];
            // 3D point projected in the right camera coordinate frame
            Mat4_Axp(xb, left2right.array, xa);
            float32_t u, v;
            // 3D-2D reprojection in the right image coordinate frame
            CHECK_DW_ERROR(dwCameraModel_ray2Pixel(&u, &v, xb[0], xb[1], xb[2], m_calibratedCameraRight));
            rightPoints.push_back({u, v});
        }
    }

    // -----------------------------------------------------------------------------
    // Convert vector of floats to text
    std::string renderFloatsText(const float* floats, uint32_t numFloats)
    {
        std::stringstream ss;

        ss.precision(4);
        ss.setf(std::ios::fixed, std::ios::floatfield);

        for (uint32_t i = 0; i < numFloats; ++i)
        {
            ss << floats[i] << " ";
        }

        return ss.str();
    }

    // -----------------------------------------------------------------------------
    // Convert calibration status to text
    std::string convertStatusToText()
    {
        std::stringstream ss;

        const char* str = nullptr;
        CHECK_DW_ERROR(dwCalibrationState_toString(&str, m_status.state));
        ss << "State: " << str;

        ss.precision(2);
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss << " (" << 100.f * m_status.percentageComplete << "%)";

        return ss.str();
    }
};

//#######################################################################################
int main(int argc, const char** argv)
{
    const std::string samplePath = dw_samples::SamplesDataPath::get() + "/samples/recordings/stereo0/";

    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("path", samplePath.c_str(), "Data folder"),
                           ProgramArguments::Option_t("rig", (samplePath + "stereo_offset_rig.json").c_str(), "The input rig to use."),
                           ProgramArguments::Option_t("vehicle-sensor", "0", "If number, the index of the can/data sensor in the rig file (the index of n-th sensor of type CAN/DATA, not an absolute sensor index), else the name."),
                           ProgramArguments::Option_t("camera-sensor-left", "left", "If number, the index of the left camera sensor in the rig file, else the name."),
                           ProgramArguments::Option_t("camera-sensor-right", "right", "If number, the index of the right camera sensor in the rig file, else the name."),
                           ProgramArguments::Option_t("matches-max-count", "8000", "Max matches count for the 2d matcher")});

    StereoCalibrationSample app(args);

    if (args.get("offscreen") != "2")
    {
        app.initializeWindow("Stereo Calibration Sample", 1280, 800, args.get("offscreen") == "1");
    }

    return app.run();
}
