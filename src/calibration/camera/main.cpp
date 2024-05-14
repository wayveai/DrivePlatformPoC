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
// SPDX-FileCopyrightText: Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <dw/calibration/cameramodel/CameraModel.h>
#include <dw/calibration/engine/Engine.h>
#include <dw/calibration/engine/camera/CameraParams.h>
#include <dw/calibration/engine/common/CalibrationTypes.h>
#include <dw/calibration/engine/common/CalibrationTypes.h>
#include <dw/core/base/Version.h>
#include <dw/egomotion/base/Egomotion.h>
#include <dw/imageprocessing/featuredetector/FeatureDetector.h>
#include <dw/imageprocessing/tracking/featuretracker/FeatureTracker.h>
#include <dw/rig/Rig.h>
#include <dw/sensors/sensormanager/SensorManager.h>

// Sample common files
#include <framework/ChecksExt.hpp>
#include <framework/DriveWorksSample.hpp>
#include <framework/Mat4.hpp>
#include <framework/MathUtils.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/CameraFramePipeline.hpp>

#include <string>
#include <memory>

using namespace dw_samples::common;

class CameraCalibrationSample : public DriveWorksSample
{
private:
    // Modules
    dwContextHandle_t m_context                       = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz              = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                               = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfiguration                  = DW_NULL_HANDLE;
    dwEgomotionHandle_t m_egomotion                   = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO                   = DW_NULL_HANDLE;
    dwCalibrationRoutineHandle_t m_calibrationRoutine = DW_NULL_HANDLE;
    dwCalibrationEngineHandle_t m_calibrationEngine   = DW_NULL_HANDLE;

    // Calibration Info
    dwTransformation3f m_nominalSensor2Rig    = {};
    dwTransformation3f m_nominalRig2Sensor    = {};
    dwTransformation3f m_calculatedSensor2Rig = {};
    dwTransformation3f m_calculatedRig2Sensor = {};
    dwTransformation3f m_correctionR          = {};
    dwVector3f m_correctionT                  = {};

    uint32_t m_cameraIndex                   = std::numeric_limits<decltype(m_cameraIndex)>::max();
    uint32_t m_imuIndex                      = std::numeric_limits<decltype(m_imuIndex)>::max();
    uint32_t m_canIndex                      = std::numeric_limits<decltype(m_canIndex)>::max();
    dwCalibrationStatus m_status             = {};
    dwCameraModelHandle_t m_calibratedCamera = DW_NULL_HANDLE;

    // Camera properties
    dwCameraProperties m_cameraProperties{};

    // Camera Sensor
    std::unique_ptr<dw_samples::common::CameraFramePipeline> m_framePipeline = {};

    // Feature Tracker
    static constexpr uint32_t HISTORY_CAPACITY    = 60;
    uint32_t m_maxFeatureCount                    = {};
    dwFeature2DDetectorHandle_t m_featureDetector = DW_NULL_HANDLE;
    dwFeature2DTrackerHandle_t m_featureTracker   = DW_NULL_HANDLE;

    dwFeatureHistoryArray m_featureHistoryGPU = {};
    dwFeatureArray m_featureDetectedGPU       = {};

    dwPyramidImage m_pyramidPrevious = {};
    dwPyramidImage m_pyramidCurrent  = {};

    // Rendering
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    const dwVector4f m_colorBlue          = {32.0f / 255.0f, 72.0f / 255.0f, 230.0f / 255.0f, 1.0f};
    const dwVector4f m_colorGreen         = {32.0f / 255.0f, 230.0f / 255.0f, 32.0f / 255.0f, 1.0f};
    const dwVector4f m_colorWhite         = {230.0f / 255.0f, 230.0f / 255.0f, 230.0f / 255.0f, 1.0f};

    uint32_t m_nominalPitchRollRenderBufferId = {};
    std::vector<dwVector2f> m_nominalPitchRollPoints;
    uint32_t m_calculatedPitchRollRenderBufferId = {};
    std::vector<dwVector2f> m_calculatedPitchRollPoints;

    uint32_t m_nominalYawRenderBufferId           = {};
    std::vector<dwVector2f> m_nominalYawPoints    = {};
    uint32_t m_calculatedYawRenderBufferId        = {};
    std::vector<dwVector2f> m_calculatedYawPoints = {};

    dwVector2f m_scaledCameraDimensions = {};
    dwVector2f m_cameraDimensions       = {};
    dwVector3f m_cameraOpticalCenterDir = {};

    std::unique_ptr<dw_samples::common::SimpleImageStreamerGL<dwImageGL>> m_streamerGL = {};
    dwImageHandle_t m_currentImageRgba                                                 = DW_NULL_HANDLE;
    dwTime_t m_currentImageRgbaTimestamp                                               = {};
    dwSensorManagerHandle_t m_sensorManager                                            = DW_NULL_HANDLE;

    // Time stamp of last processed sensor event
    dwTime_t m_lastEventTime = {};
    // Time stamp of the first processed sensor event
    dwTime_t m_firstEventTime = {};
    // Global time offset to enable looping (simulating a continuous dataset)
    dwTime_t m_offsetTime = {};

    static constexpr uint32_t LINESPACING          = 20;
    static constexpr float32_t BORDEROFFSET        = 20;
    static constexpr uint32_t NUM_PITCHROLL_POINTS = 12;
    static constexpr uint32_t NUM_YAW_POINTS       = 12;

public:
    CameraCalibrationSample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    bool onInitialize() override
    {
        initializeDriveWorks();

        initializeModules();

        initializeSensorManager();

        initializeCameraSensor();

        initializeFeatureTracker();

        startSensorManager();

        startCalibration();

        if (getWindow())
        {
            initializeRenderer();
        }

        return true;
    }

    void onRender() override
    {
        CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        if (m_currentImageRgba)
        {
            dwImageGL const* imageGL = m_streamerGL->post(m_currentImageRgba);

            dwVector2f range{};
            range.x = imageGL->prop.width;
            range.y = imageGL->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));
        }

        dwVector2f range = {static_cast<float32_t>(getWindowWidth()),
                            static_cast<float32_t>(getWindowHeight())};
        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));

        CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorBlue, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_nominalPitchRollRenderBufferId,
                                                   m_nominalPitchRollPoints.size(),
                                                   m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_nominalYawRenderBufferId, m_nominalYawPoints.size(), m_renderEngine));

        // Render estimated calibration only if accepted
        if (m_status.state == dwCalibrationState::DW_CALIBRATION_STATE_ACCEPTED)
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorGreen, m_renderEngine));
            calculatePitchRollPoints(m_calculatedPitchRollPoints,
                                     m_calculatedRig2Sensor);
            CHECK_DW_ERROR(dwRenderEngine_setBuffer(
                m_calculatedPitchRollRenderBufferId,
                DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                m_calculatedPitchRollPoints.data(), sizeof(dwVector2f), 0,
                m_calculatedPitchRollPoints.size(), m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderBuffer(
                m_calculatedPitchRollRenderBufferId,
                m_calculatedPitchRollPoints.size(), m_renderEngine));

            calculateYawPoints(m_calculatedYawPoints, m_calculatedRig2Sensor);
            CHECK_DW_ERROR(dwRenderEngine_setBuffer(
                m_calculatedYawRenderBufferId,
                DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                m_calculatedYawPoints.data(), sizeof(dwVector2f), 0,
                m_calculatedYawPoints.size(), m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_calculatedYawRenderBufferId,
                                                       m_calculatedYawPoints.size(),
                                                       m_renderEngine));
        }

        // Render estimated transformations and text
        CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorWhite, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_20, m_renderEngine));

        std::stringstream ss;
        ss << "Rendering: Estimated (Green) + Nominal (Blue)\n\n";

        ss << convertStatusToText() << "\n\n";

        ss << "Camera position correction (rig coordinates)\n\n";

        ss << renderFloatsText(&m_correctionT.x, 3) << "\n\n";

        ss << "Camera rotation correction (camera coordinates)\n\n";

        ss << renderRotationAsText(m_correctionR) << "\n";

        // Query routine for enabled signals
        dwCalibrationSignal signals{};
        CHECK_DW_ERROR(dwCalibrationEngine_getSupportedSignals(&signals, m_calibrationRoutine, m_calibrationEngine))

        if (signals & DW_CALIBRATION_SIGNAL_POSE_ROLL)
        {
            ss << " roll ";
        }
        if (signals & DW_CALIBRATION_SIGNAL_POSE_PITCH)
        {
            ss << " pitch ";
        }
        if (signals & DW_CALIBRATION_SIGNAL_POSE_YAW)
        {
            ss << " yaw ";
        }
        if (signals & DW_CALIBRATION_SIGNAL_POSE_Z)
        {
            ss << " height ";
        }
        ss << "\n\n";

        if (m_currentImageRgba)
            ss << "Timestamp: " << m_currentImageRgbaTimestamp;

        float32_t currentHeight = getWindowHeight() - 32 * LINESPACING;
        CHECK_DW_ERROR(dwRenderEngine_renderText2D(ss.str().c_str(), {BORDEROFFSET, currentHeight}, m_renderEngine));
    }

    void onProcess() override
    {
        m_currentImageRgba = nullptr;

        // Loop over signals until an image is available
        while (!m_currentImageRgba)
        {
            const dwSensorEvent* acquiredEvent = nullptr;
            auto status                        = dwSensorManager_acquireNextEvent(&acquiredEvent, 0, m_sensorManager);

            if (status == DW_TIME_OUT)
            {
                return;
            }

            if (status != DW_SUCCESS)
            {
                if (status != DW_END_OF_STREAM)
                {
                    printf("Error reading sensor %s\n", dwGetStatusName(status));
                }
                else
                {
                    handleEndOfStream();
                    return;
                }
            }

            if (!m_firstEventTime)
            {
                m_firstEventTime = acquiredEvent->timestamp_us;
            }
            m_lastEventTime = acquiredEvent->timestamp_us;

            // handle signals
            switch (acquiredEvent->type)
            {
            case DW_SENSOR_CAN:
            {
                handleCAN(acquiredEvent->canFrame);
                break;
            }

            case DW_SENSOR_IMU:
            {
                handleIMU(acquiredEvent->imuFrame);
                break;
            }

            case DW_SENSOR_CAMERA:
            {
                handleCamera(acquiredEvent->camFrames[0]);
                break;
            }

            case DW_SENSOR_GPS:
            case DW_SENSOR_RADAR:
            case DW_SENSOR_TIME:
            case DW_SENSOR_LIDAR:
            case DW_SENSOR_DATA:
            case DW_SENSOR_COUNT:
            default:
                break;
            }

            CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(acquiredEvent, m_sensorManager));
        }
    }

    void onResizeWindow(int width, int height) override
    {
        dwRenderEngine_reset(m_renderEngine);
        dwRectf rect = {};
        rect.width   = width;
        rect.height  = height;
        rect.x       = 0;
        rect.y       = 0;
        CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));

        m_scaledCameraDimensions.x = static_cast<float32_t>(width) / m_cameraDimensions.x;

        m_scaledCameraDimensions.y = static_cast<float32_t>(height) / m_cameraDimensions.y;

        calculatePitchRollPoints(m_nominalPitchRollPoints, m_nominalRig2Sensor);
        CHECK_DW_ERROR(dwRenderEngine_setBuffer(
            m_nominalPitchRollRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
            m_nominalPitchRollPoints.data(), sizeof(dwVector2f), 0,
            m_nominalPitchRollPoints.size(), m_renderEngine));

        calculateYawPoints(m_nominalYawPoints, m_nominalRig2Sensor);
        CHECK_DW_ERROR(dwRenderEngine_setBuffer(
            m_nominalYawRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, m_nominalYawPoints.data(),
            sizeof(dwVector2f), 0, m_nominalYawPoints.size(), m_renderEngine));
    }

    void onRelease() override
    {
        if (m_renderEngine)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        {
            // Release Feature list
            CHECK_DW_ERROR(dwFeatureHistoryArray_destroy(m_featureHistoryGPU));
            CHECK_DW_ERROR(dwFeatureArray_destroy(m_featureDetectedGPU));

            // Release feature tracker
            CHECK_DW_ERROR(dwFeature2DDetector_release(m_featureDetector));
            CHECK_DW_ERROR(dwFeature2DTracker_release(m_featureTracker));

            // Release pyramid
            CHECK_DW_ERROR(dwPyramid_destroy(m_pyramidCurrent));
            CHECK_DW_ERROR(dwPyramid_destroy(m_pyramidPrevious));
        }

        m_framePipeline.reset();
        m_streamerGL.reset();

        if (m_calibratedCamera)
        {
            CHECK_DW_ERROR(dwCameraModel_release(m_calibratedCamera));
        }

        if (m_calibrationEngine)
        {
            CHECK_DW_ERROR(dwCalibrationEngine_stopCalibration(m_calibrationRoutine,
                                                               m_calibrationEngine))
            CHECK_DW_ERROR(dwCalibrationEngine_release(m_calibrationEngine));
        }

        if (m_egomotion)
        {
            CHECK_DW_ERROR(dwEgomotion_release(m_egomotion));
        }

        if (m_rigConfiguration)
        {
            CHECK_DW_ERROR(dwRig_release(m_rigConfiguration));
        }

        if (m_sensorManager)
        {
            CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
            CHECK_DW_ERROR(dwSensorManager_release(m_sensorManager));
        }

        if (m_vehicleIO)
        {
            CHECK_DW_ERROR(dwVehicleIO_release(m_vehicleIO));
        }

        if (m_sal)
        {
            CHECK_DW_ERROR(dwSAL_release(m_sal));
        }

        if (m_viz)
        {
            CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        }

        CHECK_DW_ERROR(dwRelease(m_context));
    }

    void onReset() override
    {
        CHECK_DW_ERROR(dwEgomotion_reset(m_egomotion));

        // reset feature tracking
        {
            CHECK_DW_ERROR(dwFeatureHistoryArray_reset(&m_featureHistoryGPU, cudaStream_t(0)));
            CHECK_DW_ERROR(dwFeatureArray_reset(&m_featureDetectedGPU, cudaStream_t(0)));

            CHECK_DW_ERROR(dwFeature2DDetector_reset(m_featureDetector));
            CHECK_DW_ERROR(dwFeature2DTracker_reset(m_featureTracker));
        }

        // reset sensor manager
        CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_reset(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));

        m_offsetTime += m_lastEventTime - m_firstEventTime;
        m_lastEventTime  = 0;
        m_firstEventTime = 0;
    }

private:
    // Initialize DW Logger and DW context
    void initializeDriveWorks()
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)))
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE))

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};

#ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
#endif

        CHECK_DW_ERROR(dwInitialize(&m_context, DW_VERSION, &sdkParams))
    }

    uint32_t getSensorIndex(dwSensorType sensorType, const std::string& sensorSelector) const
    {
        uint32_t sensorIndexInRig = 0;

        // Get sensor index
        try
        {
            uint32_t sensorIdx = std::stol(sensorSelector);

            // Determine sensor index in rig file using numeric sensor index
            CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&sensorIndexInRig, sensorType, sensorIdx, m_rigConfiguration));
        }
        catch (const std::invalid_argument& /* string to integer conversion failed - use string as sensor name */)
        {
            // Determine sensor index using sensor name
            CHECK_DW_ERROR(dwRig_findSensorByName(&sensorIndexInRig, sensorSelector.c_str(), m_rigConfiguration));
        }

        return sensorIndexInRig;
    }

    void initializeModules()
    {
        // Initialize rig configuration, get vehicle properties
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rigConfiguration, m_context, getArgument("rig").c_str()));

        // initialize egomotion
        {
            dwEgomotionParameters egomotionParameters{};
            CHECK_DW_ERROR(dwEgomotion_initParamsFromRig(&egomotionParameters, m_rigConfiguration,
                                                         "imu*", nullptr));
            egomotionParameters.motionModel      = DW_EGOMOTION_IMU_ODOMETRY;
            egomotionParameters.suspension.model = DW_EGOMOTION_SUSPENSION_TORSIONAL_SPRING_MODEL;
            egomotionParameters.automaticUpdate  = true;
            CHECK_DW_ERROR(dwEgomotion_initialize(&m_egomotion, &egomotionParameters, m_context));
        }

        // Initialize VehicleIO
        const dwVehicle* vehicle{};
        CHECK_DW_ERROR(dwRig_getVehicle(&vehicle, m_rigConfiguration));
        CHECK_DW_ERROR(dwVehicleIO_initialize(&m_vehicleIO, DW_VEHICLEIO_DATASPEED, vehicle, m_context));

        // Initialize the calibration engine
        CHECK_DW_ERROR(dwCalibrationEngine_initialize(&m_calibrationEngine, m_rigConfiguration, m_context));
    }

    // Initialize sensor manager
    void initializeSensorManager()
    {
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

        dwSensorManagerParams smParams{};

        m_cameraIndex                                       = getSensorIndex(DW_SENSOR_CAMERA, getArgument("camera-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_cameraIndex;

        m_imuIndex                                          = getSensorIndex(DW_SENSOR_IMU, getArgument("imu-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_imuIndex;

        m_canIndex                                          = getSensorIndex(DW_SENSOR_CAN, getArgument("can-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_canIndex;

        // Initialize DriveWorks SensorManager directly from rig file
        CHECK_DW_ERROR(dwSensorManager_initializeFromRigWithParams(&m_sensorManager, m_rigConfiguration, &smParams, 1024, m_sal));
    }

    // Initialize camera frame pipeline and camera model
    void initializeCameraSensor()
    {
        dwSensorHandle_t cameraSensor{};
        CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&cameraSensor, m_cameraIndex, m_sensorManager));

        // Get the camera parameters.
        CHECK_DW_ERROR_MSG(dwSensorCamera_getSensorProperties(&m_cameraProperties, cameraSensor), "Getting Camera properties failed.");

        dwImageProperties cameraImageProperties{};
        CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraSensor));

        // Use camera frame pipeline to transparently support encoded / raw cameras
        m_framePipeline = std::make_unique<dw_samples::common::CameraFramePipeline>(cameraImageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_context);

        dwImageProperties outputProperties = cameraImageProperties;
        outputProperties.format            = DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR;
        outputProperties.type              = DW_IMAGE_CUDA;
        m_framePipeline->setOutputProperties(outputProperties);
        m_framePipeline->enableRgbaOutput();

        // Initialize intrinsic camera model to get images sizes and check for consistency
        CHECK_DW_ERROR(dwCameraModel_initialize(&m_calibratedCamera, m_cameraIndex, m_rigConfiguration));
        m_cameraDimensions.x = cameraImageProperties.width;
        m_cameraDimensions.y = cameraImageProperties.height;

        m_scaledCameraDimensions.x = static_cast<float32_t>(getWindowWidth()) / m_cameraDimensions.x;
        m_scaledCameraDimensions.y = static_cast<float32_t>(getWindowHeight()) / m_cameraDimensions.y;

        uint32_t cameraModelWidth{};
        uint32_t cameraModelHeight{};
        CHECK_DW_ERROR(dwCameraModel_getImageSize(&cameraModelWidth, &cameraModelHeight, m_calibratedCamera));
        if (cameraModelWidth != cameraImageProperties.width || cameraModelHeight != cameraImageProperties.height)
        {
            throw std::runtime_error(std::string("Camera intrinsic and sensor resolutions missmatch"));
        }

        // Initialize GL streamer only if we are rendering something
        if (getWindow())
        {
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, cameraSensor));
            m_streamerGL = std::make_unique<dw_samples::common::SimpleImageStreamerGL<dwImageGL>>(cameraImageProperties, 10000, m_context);
        }
    }

    void initializeFeatureTracker()
    {
        // Initialize feature list
        m_maxFeatureCount = static_cast<uint32_t>(std::stoi(getArgument("feature-max-count")));

        CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&m_featureHistoryGPU, m_maxFeatureCount,
                                                       HISTORY_CAPACITY, DW_MEMORY_TYPE_CUDA, nullptr, m_context));
        CHECK_DW_ERROR(dwFeatureArray_createNew(&m_featureDetectedGPU, m_maxFeatureCount,
                                                DW_MEMORY_TYPE_CUDA, nullptr, m_context));

        // Detector
        dwFeature2DDetectorConfig detectorConfig = {};
        CHECK_DW_ERROR(dwFeature2DDetector_initDefaultParamsForCamera(&detectorConfig, &m_nominalSensor2Rig, m_calibratedCamera));
        detectorConfig.maxFeatureCount = m_maxFeatureCount;
        CHECK_DW_ERROR(dwFeature2DDetector_initialize(&m_featureDetector, &detectorConfig, cudaStream_t(0), m_context));

        // Tracker
        dwFeature2DTrackerConfig trackerConfig = {};
        CHECK_DW_ERROR(dwFeature2DTracker_initDefaultParamsForCamera(&trackerConfig, &m_nominalSensor2Rig, m_calibratedCamera));
        trackerConfig.maxFeatureCount = m_maxFeatureCount;
        trackerConfig.historyCapacity = HISTORY_CAPACITY;
        CHECK_DW_ERROR(dwFeature2DTracker_initialize(&m_featureTracker, &trackerConfig, cudaStream_t(0), m_context));

        // Pyramids
        CHECK_DW_ERROR(dwPyramid_create(&m_pyramidPrevious, trackerConfig.pyramidLevelCount,
                                        m_cameraDimensions.x, m_cameraDimensions.y,
                                        DW_TYPE_UINT8, m_context));
        CHECK_DW_ERROR(dwPyramid_create(&m_pyramidCurrent, trackerConfig.pyramidLevelCount,
                                        m_cameraDimensions.x, m_cameraDimensions.y,
                                        DW_TYPE_UINT8, m_context));
    }

    void initializeRenderer()
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(),
                                                        getWindowHeight()));
        params.defaultTile.lineWidth = 2.0f;
        params.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_16;
        params.maxBufferCount        = 4;
        CHECK_DW_ERROR(
            dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        CHECK_DW_ERROR(dwCameraModel_pixel2Ray(
            &m_cameraOpticalCenterDir.x, &m_cameraOpticalCenterDir.y,
            &m_cameraOpticalCenterDir.z, m_cameraDimensions.x / 2.f,
            m_cameraDimensions.y / 2.f, m_calibratedCamera));

        calculatePitchRollPoints(m_nominalPitchRollPoints, m_nominalRig2Sensor);
        CHECK_DW_ERROR(dwRenderEngine_createBuffer(
            &m_nominalPitchRollRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, sizeof(dwVector2f), 0,
            NUM_PITCHROLL_POINTS, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setBuffer(
            m_nominalPitchRollRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
            m_nominalPitchRollPoints.data(), sizeof(dwVector2f), 0,
            m_nominalPitchRollPoints.size(), m_renderEngine));

        calculateYawPoints(m_nominalYawPoints, m_nominalRig2Sensor);
        CHECK_DW_ERROR(dwRenderEngine_createBuffer(
            &m_nominalYawRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, sizeof(dwVector2f), 0,
            NUM_YAW_POINTS, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setBuffer(
            m_nominalYawRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, m_nominalYawPoints.data(),
            sizeof(dwVector2f), 0, m_nominalYawPoints.size(), m_renderEngine));

        CHECK_DW_ERROR(dwRenderEngine_createBuffer(
            &m_calculatedPitchRollRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, sizeof(dwVector2f), 0,
            NUM_PITCHROLL_POINTS, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_createBuffer(
            &m_calculatedYawRenderBufferId,
            DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, sizeof(dwVector2f), 0,
            NUM_YAW_POINTS, m_renderEngine));

        glDepthFunc(GL_ALWAYS);

        CHECK_GL_ERROR()
    }

    void startSensorManager()
    {
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));
    }

    void startCalibration()
    {
        // parse signals to calibrate
        dwCalibrationCameraSignal calibrationCameraSignals{};
        {
            const std::string signalsStr = getArgument("signals");
            if (signalsStr.find("default") != std::string::npos)
                calibrationCameraSignals = DW_CALIBRATION_CAMERA_SIGNAL_DEFAULT;
            else
            {
                if (signalsStr.find("pitchyaw") != std::string::npos)
                    calibrationCameraSignals = dwCalibrationCameraSignal(calibrationCameraSignals |
                                                                         DW_CALIBRATION_CAMERA_SIGNAL_PITCHYAW);
                if (signalsStr.find("roll") != std::string::npos)
                    calibrationCameraSignals = dwCalibrationCameraSignal(calibrationCameraSignals |
                                                                         DW_CALIBRATION_CAMERA_SIGNAL_ROLL);
                if (signalsStr.find("height") != std::string::npos)
                    calibrationCameraSignals = dwCalibrationCameraSignal(calibrationCameraSignals |
                                                                         DW_CALIBRATION_CAMERA_SIGNAL_HEIGHT);
            }
        }

        dwCalibrationCameraParams params{};
        params.method         = DW_CALIBRATION_CAMERA_METHOD_FEATURES;
        params.signals        = calibrationCameraSignals;
        params.fastAcceptance = getArgumentEnum("fast-acceptance",
                                                {"default", "enabled", "disabled"},
                                                {DW_CALIBRATION_FAST_ACCEPTANCE_DEFAULT,
                                                 DW_CALIBRATION_FAST_ACCEPTANCE_ENABLED,
                                                 DW_CALIBRATION_FAST_ACCEPTANCE_DISABLED});

        params.features.maxFeatureCount       = m_maxFeatureCount;
        params.features.maxFeatureHistorySize = HISTORY_CAPACITY;
        params.cameraProperties               = &m_cameraProperties;

        CHECK_DW_ERROR(dwCalibrationEngine_initializeCamera(
            &m_calibrationRoutine, m_cameraIndex, &params, m_egomotion, cudaStream_t(0),
            m_calibrationEngine));

        CHECK_DW_ERROR(dwRig_getNominalSensorToRigTransformation(
            &m_nominalSensor2Rig, m_cameraIndex, m_rigConfiguration))

        Mat4_IsoInv(m_nominalRig2Sensor.array, m_nominalSensor2Rig.array);

        CHECK_DW_ERROR(dwCalibrationEngine_startCalibration(m_calibrationRoutine, m_calibrationEngine))
    }

    void trackFeatures(dwImageCUDA const& imageCuda)
    {
        // Update pyramid
        std::swap(m_pyramidCurrent, m_pyramidPrevious);
        CHECK_DW_ERROR(dwImageFilter_computePyramid(&m_pyramidCurrent, &imageCuda,
                                                    cudaStream_t(0), m_context));

        // Track
        dwFeatureArray featurePredicted{};
        CHECK_DW_ERROR(dwFeature2DTracker_trackFeatures(
            &m_featureHistoryGPU, &featurePredicted, nullptr, &m_featureDetectedGPU,
            nullptr, &m_pyramidPrevious, &m_pyramidCurrent, m_featureTracker));

        // Detect
        CHECK_DW_ERROR(dwFeature2DDetector_detectFromPyramid(
            &m_featureDetectedGPU, &m_pyramidCurrent, &featurePredicted,
            nullptr, m_featureDetector));
    }

public:
    // Update
    void handleCAN(const dwCANMessage& canMsg)
    {
        // Sample is looping over the same data -> need to simulate continuous CAN data by adding an extra offset to the timestamp
        dwCANMessage msg = canMsg;
        msg.timestamp_us += m_offsetTime;

        // Only a single CAN bus in SensorManager -> reuse m_canIndex
        CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&msg, m_canIndex, m_vehicleIO));

        dwVehicleIOSafetyState vehicleIOSafeState{};
        dwVehicleIONonSafetyState vehicleIONonSafeState{};
        dwVehicleIOActuationFeedback vehicleIOActuationFeedbackState{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&vehicleIOSafeState, m_vehicleIO));
        CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&vehicleIONonSafeState, m_vehicleIO));
        CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&vehicleIOActuationFeedbackState, m_vehicleIO));
        CHECK_DW_ERROR(dwEgomotion_addVehicleIOState(&vehicleIOSafeState, &vehicleIONonSafeState, &vehicleIOActuationFeedbackState, m_egomotion));
    }

    void handleIMU(const dwIMUFrame& imuFrame)
    {
        // Sample is looping over the same data -> need to simulate continuous IMU data by adding an extra offset to the timestamp
        dwIMUFrame msgIMU = imuFrame;
        msgIMU.hostTimestamp += m_offsetTime;

        CHECK_DW_ERROR(dwEgomotion_addIMUMeasurement(&msgIMU, m_egomotion));
    }

    void handleCamera(const dwCameraFrameHandle_t& frame)
    {
        m_framePipeline->processFrame(frame);

        dwImageHandle_t image = m_framePipeline->getFrame();
        m_currentImageRgba    = m_framePipeline->getFrameRgba();

        dwImageCUDA* imageCuda;
        CHECK_DW_ERROR(dwImage_getCUDA(&imageCuda, image));
        CHECK_DW_ERROR(dwImage_getTimestamp(&m_currentImageRgbaTimestamp, image));

        {
            // Track features
            trackFeatures(*imageCuda);

            // Feed features to calibration
            // (sample is looping over the same data -> need to simulate continuous camera data by adding an extra offset to the timestamp)
            CHECK_DW_ERROR(dwCalibrationEngine_addFeatureDetections(
                m_maxFeatureCount, HISTORY_CAPACITY, m_featureHistoryGPU.featureCount,
                m_featureHistoryGPU.ages, m_featureHistoryGPU.locationHistory, m_featureHistoryGPU.statuses,
                m_featureHistoryGPU.currentTimeIdx,
                m_currentImageRgbaTimestamp + m_offsetTime, m_cameraIndex, m_calibrationEngine));
        }

        CHECK_DW_ERROR(dwCalibrationEngine_getSensorToRigTransformation(
            &m_calculatedSensor2Rig, m_calibrationRoutine, m_calibrationEngine))

        // Factorization: sensor2Rig = [correctionT] * sensor2RigNominal * [correctionR]
        Mat4_AxB(m_correctionR.array, m_nominalRig2Sensor.array, m_calculatedSensor2Rig.array);
        m_correctionR.array[3 * 4] = m_correctionR.array[3 * 4 + 1] = m_correctionR.array[3 * 4 + 2] = 0.f;

        m_correctionT.x = m_calculatedSensor2Rig.array[3 * 4] - m_nominalSensor2Rig.array[3 * 4];
        m_correctionT.y = m_calculatedSensor2Rig.array[3 * 4 + 1] - m_nominalSensor2Rig.array[3 * 4 + 1];
        m_correctionT.z = m_calculatedSensor2Rig.array[3 * 4 + 2] - m_nominalSensor2Rig.array[3 * 4 + 2];

        Mat4_IsoInv(m_calculatedRig2Sensor.array, m_calculatedSensor2Rig.array);

        CHECK_DW_ERROR(dwCalibrationEngine_getCalibrationStatus(&m_status, m_calibrationRoutine, m_calibrationEngine))
    }

    void handleEndOfStream() { onReset(); }

private:
    // Renderer
    std::string renderRotationAsText(const dwTransformation3f& transformation)
    {
        std::stringstream ss;

        ss.precision(4);
        ss.setf(std::ios::fixed, std::ios::floatfield);

        for (auto row = 0u; row < 3; ++row)
        {
            for (auto col = 0u; col < 3; ++col)
            {
                ss << transformation.array[row + col * 4] << " ";
            }
            ss << "\n";
        }

        return ss.str();
    }

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

    void calculatePitchRollPoints(std::vector<dwVector2f>& imagePoints,
                                  const dwTransformation3f& rig2Sensor)
    {
        imagePoints.clear();

        // Sample a "half circle" of directions in front of car in the xy ground
        // plane
        float32_t angleIncrement = M_PI / (NUM_PITCHROLL_POINTS - 1);
        float32_t angle          = 0.0f;

        for (auto i = 0u; i < NUM_PITCHROLL_POINTS; ++i, angle += angleIncrement)
        {
            float32_t rigDir[3];

            rigDir[0] = sin(angle);
            rigDir[1] = cos(angle);
            rigDir[2] = 0;

            float32_t cameraDir[3];

            Mat4_Rxp(cameraDir, rig2Sensor.array, rigDir);

            // discards points "behind" camera
            if (cameraDir[0] * m_cameraOpticalCenterDir.x +
                    cameraDir[1] * m_cameraOpticalCenterDir.y +
                    cameraDir[2] * m_cameraOpticalCenterDir.z >=
                0.1f)
            {

                float32_t u{0.0f}, v{0.0f};

                CHECK_DW_ERROR(dwCameraModel_ray2Pixel(&u, &v, cameraDir[0],
                                                       cameraDir[1], cameraDir[2],
                                                       m_calibratedCamera));

                u *= m_scaledCameraDimensions.x;
                v *= m_scaledCameraDimensions.y;

                imagePoints.push_back({u, v});
            }
        }
    }

    void calculateYawPoints(std::vector<dwVector2f>& imagePoints,
                            const dwTransformation3f& rig2Sensor)
    {
        // Sample points in front of car in the xy ground plane
        imagePoints.clear();

        float32_t offset = 0.f;

        for (auto i = 0u; i < NUM_YAW_POINTS; ++i, offset += offset + 1)
        {
            float32_t rigPoint[3];

            rigPoint[0] = offset;
            rigPoint[1] = 0.f;
            rigPoint[2] = 0.f;

            float32_t cameraPoint[3];

            Mat4_Axp(cameraPoint, rig2Sensor.array, rigPoint);

            // discards points "behind" camera
            if (cameraPoint[0] * m_cameraOpticalCenterDir.x +
                    cameraPoint[1] * m_cameraOpticalCenterDir.y +
                    cameraPoint[2] * m_cameraOpticalCenterDir.z >=
                0.1f)
            {
                float32_t u{0.0f}, v{0.0f};

                CHECK_DW_ERROR(
                    dwCameraModel_ray2Pixel(&u, &v, cameraPoint[0], cameraPoint[1],
                                            cameraPoint[2], m_calibratedCamera));

                u *= m_scaledCameraDimensions.x;
                v *= m_scaledCameraDimensions.y;

                imagePoints.push_back({u, v});
            }
        }
    }
};

//#######################################################################################
int32_t main(int32_t argc, const char** argv)
{
    const std::string samplePath = dw_samples::SamplesDataPath::get() + "/samples/recordings/suburb0/";

    // -------------------
    // define all arguments used by the application
    ProgramArguments args(
        argc, argv,
        {ProgramArguments::Option_t("rig", (samplePath + "rig.json").c_str(), "Path to rig configuration file"),
         ProgramArguments::Option_t("camera-sensor", "0", "The index or name of the camera sensor from the rig file to calibrate"),
         ProgramArguments::Option_t("can-sensor", "0", "The index or name of the CAN bus sensor from the rig-file to use"),
         ProgramArguments::Option_t("imu-sensor", "0", "The index or name of the IMU sensor in the rig-file to use"),
         ProgramArguments::Option_t("signals", "default", "'default', or a combination of ['pitchyaw','roll','height'] strings"),
         ProgramArguments::Option_t("fast-acceptance", "disabled", "'default', 'enabled', or 'disabled'"),
         ProgramArguments::Option_t("feature-max-count", "800", "Max feature count for the tracker")});

    CameraCalibrationSample app(args);

    if (args.get("offscreen") != "2")
    {
        app.initializeWindow("Camera Calibration Sample", 1280, 800, args.get("offscreen") == "1");
    }

    return app.run();
}
