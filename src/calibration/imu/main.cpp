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
#include <dw/core/base/Version.h>
#include <dw/rig/Rig.h>
#include <dw/egomotion/base/Egomotion.h>
#include <dw/calibration/engine/Engine.h>

// Sensor Input
#include <dw/sensors/sensormanager/SensorManager.h>
#include <dw/sensors/canbus/Interpreter.h>

// Rendering
#include <dwvisualization/core/RenderEngine.h>

// Sample common files
#include <framework/DriveWorksSample.hpp>
#include <framework/Checks.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/MathUtils.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Mat4.hpp>

using namespace dw_samples::common;

class IMUCalibrationSample : public DriveWorksSample
{
private:
    // Modules
    dwContextHandle_t m_context                       = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz              = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine             = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                               = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfiguration                  = DW_NULL_HANDLE;
    dwEgomotionHandle_t m_egomotion                   = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO                   = DW_NULL_HANDLE;
    dwCalibrationRoutineHandle_t m_calibrationRoutine = DW_NULL_HANDLE;
    dwCalibrationEngineHandle_t m_calibrationEngine   = DW_NULL_HANDLE;
    const dwVehicle* m_vehicle;

    dwImageProperties m_cameraOutputProperties{};

    // Calibration Info
    dwTransformation3f m_nominalIMU2Rig;
    dwTransformation3f m_nominalCamera2Rig;
    dwTransformation3f m_nominalRig2IMU;
    dwTransformation3f m_nominalRig2Camera;
    dwTransformation3f m_calculatedIMU2Rig;
    dwTransformation3f m_correctionR = {};
    dwVector3f m_correctionT         = {};
    dwVector3f m_correctionRPY       = {};

    uint32_t m_imuIndex    = std::numeric_limits<decltype(m_imuIndex)>::max();
    uint32_t m_cameraIndex = std::numeric_limits<decltype(m_cameraIndex)>::max();
    uint32_t m_canIndex    = std::numeric_limits<decltype(m_canIndex)>::max();
    dwCalibrationStatus m_status{};
    dwCameraModelHandle_t m_calibratedCamera = DW_NULL_HANDLE;

    // Data looping / timestamp adaptation
    dwTime_t m_loopDataOffsetTimestamp = 0;
    dwTime_t m_loopDataFirstTimestamp  = 0;
    dwTime_t m_loopDataLatestTimestamp = 0;

    // IMU up vector
    dwTime_t m_previousCameraTimeStamp    = 0;
    dwVector3f m_previousSpeed            = {0.0f, 0.0f, 0.0f};
    dwVector3f m_calculatedGravity        = {0.0f, 0.0f, 1.0f};
    dwVector3f m_nominalGravity           = {0.0f, 0.0f, 1.0f};
    static constexpr uint32_t FILTER_SIZE = 400;
    std::vector<float32_t> m_exponentialFilter;
    std::deque<dwVector3f> m_bufferAccCalculated;
    std::deque<dwVector3f> m_bufferAccNominal;
    static constexpr uint32_t WAIT_GRAVITY = 20;

    // Camera Sensor
    dwImageHandle_t m_imageRgba;
    dwImageHandle_t* m_currentImageRgba  = nullptr;
    dwTime_t m_currentImageRgbaTimestamp = {};

    // IMU
    dwIMUFrame m_imuFrame;

    // Rendering
    const dwVector4f m_colorBlue  = {32.0f / 255.0f, 72.0f / 255.0f, 230.0f / 255.0f, 1.0f};
    const dwVector4f m_colorGreen = {32.0f / 255.0f, 230.0f / 255.0f, 32.0f / 255.0f, 1.0f};
    const dwVector4f m_colorWhite = {230.0f / 255.0f, 230.0f / 255.0f, 230.0f / 255.0f, 1.0f};

    uint32_t m_nominalPitchRollRenderBufferId;
    std::vector<dwVector2f> m_nominalPitchRollPoints;
    uint32_t m_calculatedPitchRollRenderBufferId;
    std::vector<dwVector2f> m_calculatedPitchRollPoints;

    static constexpr uint32_t LINESPACING          = 20;
    static constexpr uint32_t BORDEROFFSET         = 20;
    static constexpr uint32_t NUM_PITCHROLL_POINTS = 12;

    dwVector2f m_scaledCameraDimensions;
    dwVector2f m_cameraDimensions;
    dwVector3f m_cameraOpticalCenterDir;

    dwSensorManagerHandle_t m_sensorManager = DW_NULL_HANDLE;

    std::unique_ptr<dw_samples::common::SimpleImageStreamerGL<dwImageGL>> m_streamerInput2GL;

public:
    IMUCalibrationSample(const ProgramArguments& args)
        : DriveWorksSample(args)
        , m_exponentialFilter(FILTER_SIZE)
    {
    }

    // -----------------------------------------------------------------------------
    // Initialize all modules
    bool onInitialize() override
    {
        initializeDriveWorks();

        // Initialize our rig configuration module
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rigConfiguration, m_context, getArgument("rig").c_str()));

        initializeSensorManager();

        initializeModules();

        initializeCameraSensor();

        initializeExponentialFilter();

        startSensorManager();

        startCalibration();

        if (getWindow())
        {
            initializeRenderer();
        }

        return true;
    }

    // -----------------------------------------------------------------------------
    // Render the nominal and estimated horizons on screen.
    // The horizon is used to visually represent the roll and pitch of an IMU calibration
    // The horizon rendered in blue represents the roll and pitch from nominal calibration
    // The horizon rendered in green represents the roll and pitch from calculated calibration
    // Numerical information regarding calibration are printed in the bottom left corner
    void onRender() override
    {
        CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        if (m_currentImageRgba)
        {
            dwImageGL const* imageGL = m_streamerInput2GL->post(*m_currentImageRgba);

            dwVector2f range{};
            range.x = imageGL->prop.width;
            range.y = imageGL->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));
        }

        dwVector2f range = {static_cast<float32_t>(getWindowWidth()), static_cast<float32_t>(getWindowHeight())};
        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));

        // Wait until calibration is accepted and calculated gravity can be filtered for noise
        if (m_status.state == DW_CALIBRATION_STATE_ACCEPTED && m_bufferAccCalculated.size() > WAIT_GRAVITY)
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorGreen, m_renderEngine));
            calculatePitchRollPoints(m_calculatedPitchRollPoints, m_nominalRig2Camera, m_calculatedGravity);
            CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_calculatedPitchRollRenderBufferId,
                                                    DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                    m_calculatedPitchRollPoints.data(),
                                                    sizeof(dwVector2f), 0, m_calculatedPitchRollPoints.size(),
                                                    m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_calculatedPitchRollRenderBufferId,
                                                       m_calculatedPitchRollPoints.size(),
                                                       m_renderEngine));
        }

        // Wait until nominal gravity can be filtered for noise
        if (m_bufferAccNominal.size() > WAIT_GRAVITY)
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorBlue, m_renderEngine));
            calculatePitchRollPoints(m_nominalPitchRollPoints, m_nominalRig2Camera, m_nominalGravity);
            CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_nominalPitchRollRenderBufferId,
                                                    DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                    m_nominalPitchRollPoints.data(),
                                                    sizeof(dwVector2f), 0, m_nominalPitchRollPoints.size(),
                                                    m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_nominalPitchRollRenderBufferId,
                                                       m_nominalPitchRollPoints.size(),
                                                       m_renderEngine));
        }

        CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_20, m_renderEngine));

        // Render matrices and text to the screen
        CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorWhite, m_renderEngine));

        std::stringstream ss;
        ss << "Rendering: Estimated (Green) + Nominal (Blue)\n\n";
        ss << convertStatusToText() << "\n\n";

        ss << "IMU position correction (rig coordinates)\n\n";
        ss << renderFloatsText(&m_correctionT.x, 3) << "\n\n";

        ss << "IMU roll-pitch-yaw correction (rig coordinates)\n\n";
        ss << renderFloatsText(&m_correctionRPY.x, 3) << "\n\n";

        ss << "IMU rotation correction (rig coordinates)\n\n";
        ss << renderRotationAsText(m_correctionR) << "\n";

        if (m_currentImageRgba)
        {
            ss << "Timestamp: " << m_currentImageRgbaTimestamp;
        }

        float32_t currentHeight = getWindowHeight() - 18 * LINESPACING;
        CHECK_DW_ERROR(dwRenderEngine_renderText2D(ss.str().c_str(), {BORDEROFFSET, currentHeight}, m_renderEngine));

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    // -----------------------------------------------------------------------------
    // Read the CAN, IMU and camera frames and operate on them
    void onProcess() override
    {
        m_currentImageRgba = nullptr;

        // Loop over signals until an image is available
        while (!m_currentImageRgba)
        {
            const dwSensorEvent* acquiredEvent = nullptr;
            auto status                        = dwSensorManager_acquireNextEvent(&acquiredEvent, 0, m_sensorManager);

            if (status == DW_TIME_OUT)
                return;

            if (status != DW_SUCCESS)
            {
                if (status != DW_END_OF_STREAM)
                    printf("Error reading sensor %s\n", dwGetStatusName(status));
                else
                {
                    handleEndOfStream();
                    break;
                }
            }

            // Handle signals
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

    // -----------------------------------------------------------------------------
    // Resize window
    void onResizeWindow(int width, int height) override
    {
        CHECK_DW_ERROR(dwRenderEngine_reset(m_renderEngine));
        dwRectf rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));

        m_scaledCameraDimensions.x = static_cast<float32_t>(width) / m_cameraDimensions.x;

        m_scaledCameraDimensions.y = static_cast<float32_t>(height) / m_cameraDimensions.y;

        calculatePitchRollPoints(m_nominalPitchRollPoints, m_nominalRig2Camera, m_nominalGravity);
        CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_nominalPitchRollRenderBufferId,
                                                DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                m_nominalPitchRollPoints.data(),
                                                sizeof(dwVector2f), 0, m_nominalPitchRollPoints.size(),
                                                m_renderEngine));
    }

    // -----------------------------------------------------------------------------
    // Release all modules
    void onRelease() override
    {
        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        m_streamerInput2GL.reset();

        if (m_calibratedCamera)
        {
            CHECK_DW_ERROR(dwCameraModel_release(m_calibratedCamera));
        }

        if (m_calibrationEngine)
        {
            CHECK_DW_ERROR(dwCalibrationEngine_stopCalibration(m_calibrationRoutine, m_calibrationEngine))
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
        CHECK_DW_ERROR(dwLogger_release());
    }

    // -----------------------------------------------------------------------------
    // Reset egomotion, sensor manager, and gravity measurements
    void onReset() override
    {
        // Restart on reset
        CHECK_DW_ERROR(dwEgomotion_reset(m_egomotion));

        // Reset sensor manager
        CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_reset(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));

        // Reset sample variables
        m_previousCameraTimeStamp = 0;
        m_previousSpeed           = dwVector3f{0.0f, 0.0f, 0.0f};

        m_loopDataOffsetTimestamp += m_loopDataLatestTimestamp - m_loopDataFirstTimestamp;
        m_loopDataFirstTimestamp = 0;
    }

private:
    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks()
    {
        // Initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // Initialize SDK context, using data folder
        dwContextParameters sdkParams = {};

#ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
#endif

        CHECK_DW_ERROR(dwInitialize(&m_context, DW_VERSION, &sdkParams));
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

    // -----------------------------------------------------------------------------
    // Initialize rig configuration
    // Initialize egomotion
    // Initialize calibration engine
    void initializeModules()
    {
        // Extract required parameters from the rig file
        dwTransformation3f imuToRig{};
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&imuToRig, m_imuIndex, m_rigConfiguration));

        m_vehicle = nullptr;
        CHECK_DW_ERROR(dwRig_getVehicle(&m_vehicle, m_rigConfiguration))

        // Initialize egomotion module
        dwEgomotionParameters params{};
        params.automaticUpdate = true;
        params.imu2rig         = imuToRig;
        params.vehicle         = *m_vehicle;
        params.motionModel     = DW_EGOMOTION_ODOMETRY;
        CHECK_DW_ERROR(dwEgomotion_initialize(&m_egomotion, &params, m_context))

        // Initialize VehicleIO
        CHECK_DW_ERROR(dwVehicleIO_initialize(&m_vehicleIO, DW_VEHICLEIO_DATASPEED, m_vehicle, m_context));

        // Finally initialize our calibration module
        CHECK_DW_ERROR(dwCalibrationEngine_initialize(&m_calibrationEngine, m_rigConfiguration, m_context))
    }

    // -----------------------------------------------------------------------------
    // Initialize camera model
    void initializeCameraSensor()
    {
        dwSensorHandle_t cameraSensor = DW_NULL_HANDLE;
        CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&cameraSensor, m_cameraIndex, m_sensorManager));

        CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&m_cameraOutputProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, cameraSensor));

        CHECK_DW_ERROR(dwImage_create(&m_imageRgba, m_cameraOutputProperties, m_context));

        CHECK_DW_ERROR(dwCameraModel_initialize(&m_calibratedCamera, 0, m_rigConfiguration));

        m_cameraDimensions.x = m_cameraOutputProperties.width;
        m_cameraDimensions.y = m_cameraOutputProperties.height;

        m_scaledCameraDimensions.x = static_cast<float32_t>(getWindowWidth()) / m_cameraDimensions.x;
        m_scaledCameraDimensions.y = static_cast<float32_t>(getWindowHeight()) / m_cameraDimensions.y;
    }

    // -----------------------------------------------------------------------------
    // Initialize sensor manager
    void initializeSensorManager()
    {
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

        dwSensorManagerParams smParams{};

        m_imuIndex                                          = getSensorIndex(DW_SENSOR_IMU, getArgument("imu-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_imuIndex;

        m_cameraIndex                                       = getSensorIndex(DW_SENSOR_CAMERA, getArgument("camera-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_cameraIndex;

        m_canIndex                                          = getSensorIndex(DW_SENSOR_CAN, getArgument("can-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_canIndex;

        // Initialize DriveWorks SensorManager directly from rig file
        dwSensorManager_initializeFromRigWithParams(&m_sensorManager, m_rigConfiguration, &smParams, 1024, m_sal);
    }

    // -----------------------------------------------------------------------------
    // Initialize exponential filter for gravity smoothing
    void initializeExponentialFilter()
    {
        // Initialize the exponential weighted moving average (EWMA) kernel
        m_exponentialFilter.resize(m_exponentialFilter.capacity());
        float32_t alpha                                     = 2.0f / (FILTER_SIZE + 1);
        m_exponentialFilter[m_exponentialFilter.size() - 1] = 1.0f;
        float32_t sum                                       = 1.0f;
        for (uint32_t i = FILTER_SIZE - 1; i >= 1; --i)
        {
            m_exponentialFilter[i - 1] = m_exponentialFilter[i] * (1.0f - alpha);
            sum += m_exponentialFilter[i - 1];
        }

        // Normalize it to have unit area
        for (uint32_t ii = 0; ii < FILTER_SIZE; ++ii)
        {
            m_exponentialFilter[ii] /= sum;
        }
    }

    // -----------------------------------------------------------------------------
    // Initialize render engine
    // Estimate camera optical center direction
    // Create buffer to draw the IMU-sensed horizon
    void initializeRenderer()
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        {
            dwImageProperties displayProperties = m_cameraOutputProperties;
            displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;
            m_streamerInput2GL                  = std::make_unique<SimpleImageStreamerGL<dwImageGL>>(displayProperties, 1000, m_context);
        }

        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        params.defaultTile.lineWidth = 2.0f;
        params.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_16;
        params.maxBufferCount        = 4;
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        CHECK_DW_ERROR(dwCameraModel_pixel2Ray(&m_cameraOpticalCenterDir.x,
                                               &m_cameraOpticalCenterDir.y,
                                               &m_cameraOpticalCenterDir.z,
                                               m_cameraDimensions.x / 2.f,
                                               m_cameraDimensions.y / 2.f,
                                               m_calibratedCamera));

        CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_nominalPitchRollRenderBufferId,
                                                   DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                   sizeof(dwVector3f),
                                                   0,
                                                   NUM_PITCHROLL_POINTS,
                                                   m_renderEngine));

        CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_calculatedPitchRollRenderBufferId,
                                                   DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D,
                                                   sizeof(dwVector3f),
                                                   0,
                                                   NUM_PITCHROLL_POINTS,
                                                   m_renderEngine));

        glDepthFunc(GL_ALWAYS);
    }

    // -----------------------------------------------------------------------------
    // Start sensor manager for handling camera, CAN, and IMU data
    void startSensorManager()
    {
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));
    }

    // -----------------------------------------------------------------------------
    // Initialize IMU calibration engine
    // Get nominal transformation for IMU and camera
    // Start calibration
    void startCalibration()
    {
        // Initialize IMU calibration engine
        dwCalibrationIMUParams imuParams{};
        CHECK_DW_ERROR(dwCalibrationEngine_initializeIMU(&m_calibrationRoutine, m_imuIndex, m_canIndex, &imuParams, m_calibrationEngine));

        // Get IMU nominal calibration to invert it
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&m_nominalIMU2Rig, m_imuIndex, m_rigConfiguration))
        Mat4_IsoInv(m_nominalRig2IMU.array, m_nominalIMU2Rig.array);

        // Get camera nominal calibration to invert it
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&m_nominalCamera2Rig, m_cameraIndex, m_rigConfiguration))
        Mat4_IsoInv(m_nominalRig2Camera.array, m_nominalCamera2Rig.array);

        // Start calibration
        CHECK_DW_ERROR(dwCalibrationEngine_startCalibration(m_calibrationRoutine, m_calibrationEngine))
    }

    // -----------------------------------------------------------------------------
    // Read CAN data, feed and update egomotion module
    void handleCAN(const dwCANMessage& canMsg)
    {
        // Sample is looping over the same data -> need to simulate continuous CAN data by adding an extra offset to the timestamp
        dwCANMessage msg = canMsg;
        msg.timestamp_us += m_loopDataOffsetTimestamp;
        if (!m_loopDataFirstTimestamp)
            m_loopDataFirstTimestamp = msg.timestamp_us;
        m_loopDataLatestTimestamp    = std::max(msg.timestamp_us, m_loopDataLatestTimestamp);

        // Only a single CAN bus in SensorManager -> reuse m_canIndex
        CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&msg, m_canIndex, m_vehicleIO));

        dwVehicleIONonSafetyState vehicleIONonSafetyState{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&vehicleIONonSafetyState, m_vehicleIO));

        dwVehicleIOActuationFeedback vehicleIOActuationFeedback{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&vehicleIOActuationFeedback, m_vehicleIO));

        CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIONonSafetyState(&vehicleIONonSafetyState, m_canIndex, m_calibrationEngine));
        CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIOActuationFeedback(&vehicleIOActuationFeedback, m_canIndex, m_calibrationEngine));

        dwVehicleIOSafetyState vehicleIOSafetyState{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&vehicleIOSafetyState, m_vehicleIO));

        CHECK_DW_ERROR(dwEgomotion_addVehicleIOState(&vehicleIOSafetyState, &vehicleIONonSafetyState, &vehicleIOActuationFeedback, m_egomotion));
    }

    // -----------------------------------------------------------------------------
    // Get IMU frame and then sensor2rig transformation to estimate the correction
    // If calibration has converged, the sample playback is slowed down
    void handleIMU(const dwIMUFrame& imuFrame)
    {
        // Sample is looping over the same data -> need to simulate continuous IMU data by adding an extra offset to the timestamp
        dwIMUFrame msg = imuFrame;
        msg.hostTimestamp += m_loopDataOffsetTimestamp;
        if (!m_loopDataFirstTimestamp)
            m_loopDataFirstTimestamp = msg.hostTimestamp;
        m_loopDataLatestTimestamp    = std::max(msg.hostTimestamp, m_loopDataLatestTimestamp);

        // Add IMU frame to calibration engine (only a single IMU registered with SensorManager)
        CHECK_DW_ERROR(dwCalibrationEngine_addIMUFrame(&msg, m_imuIndex, m_calibrationEngine));

        // Get IMU calibration transformation
        CHECK_DW_ERROR(dwCalibrationEngine_getSensorToRigTransformation(
            &m_calculatedIMU2Rig, m_calibrationRoutine, m_calibrationEngine))

        // Compute correction for rotation and translation
        // Factorization: sensor2Rig = [correctionT] * sensor2RigNominal * [correctionR]
        Mat4_AxB(m_correctionR.array, m_nominalRig2IMU.array, m_calculatedIMU2Rig.array);
        m_correctionR.array[3 * 4] = m_correctionR.array[3 * 4 + 1] = m_correctionR.array[3 * 4 + 2] = 0.f;
        m_correctionT.x                                                                              = m_calculatedIMU2Rig.array[3 * 4] - m_nominalIMU2Rig.array[3 * 4];
        m_correctionT.y                                                                              = m_calculatedIMU2Rig.array[3 * 4 + 1] - m_nominalIMU2Rig.array[3 * 4 + 1];
        m_correctionT.z                                                                              = m_calculatedIMU2Rig.array[3 * 4 + 2] - m_nominalIMU2Rig.array[3 * 4 + 2];

        m_correctionRPY.x = RAD2DEG(std::atan2(m_correctionR.array[6], m_correctionR.array[10]));
        m_correctionRPY.y = RAD2DEG(std::atan2(-m_correctionR.array[2], std::sqrt(m_correctionR.array[0] * m_correctionR.array[0] + m_correctionR.array[1] * m_correctionR.array[1])));
        m_correctionRPY.z = RAD2DEG(std::atan2(m_correctionR.array[1], m_correctionR.array[0]));

        // Get IMU calibration status
        CHECK_DW_ERROR(
            dwCalibrationEngine_getCalibrationStatus(&m_status, m_calibrationRoutine, m_calibrationEngine));

        // Slow down sample if calibration converged and gravity can be filtered for noise
        if (m_status.state == DW_CALIBRATION_STATE_ACCEPTED && m_bufferAccCalculated.size() > WAIT_GRAVITY)
            setProcessRate(60);
        else
            setProcessRate(1000);

        m_imuFrame = imuFrame;
    }

    // -----------------------------------------------------------------------------
    // Get camera image and estimate gravity
    void handleCamera(const dwCameraFrameHandle_t& frame)
    {
        dwImageHandle_t image;
        CHECK_DW_ERROR(dwSensorCamera_getImage(&image, DW_CAMERA_OUTPUT_CUDA_YUV420_UINT8_SEMIPLANAR, frame));

        CHECK_DW_ERROR(dwImage_copyConvert(m_imageRgba, image, m_context));
        m_currentImageRgba = &m_imageRgba;
        CHECK_DW_ERROR(dwImage_getTimestamp(&m_currentImageRgbaTimestamp, image));
        // Add loop offset to timestamp
        m_currentImageRgbaTimestamp += m_loopDataOffsetTimestamp;

        // Estimate gravity with nominal and calculated calibration transformation
        estimateGravity();
    }

    // -----------------------------------------------------------------------------
    // Reset in case of end of stream
    void handleEndOfStream()
    {
        reset();
    }

    // -----------------------------------------------------------------------------
    // Convert rotation matrix to text
    std::string renderRotationAsText(const dwTransformation3f& transformation)
    {
        std::stringstream ss;

        ss.precision(4);
        ss.setf(std::ios::fixed, std::ios::floatfield);

        for (auto row = 0u; row < 3; ++row)
        {

            for (auto col = 0u; col < 3; ++col)
                ss << transformation.array[row + col * 4] << " ";

            ss << "\n";
        }

        return ss.str();
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

    // -----------------------------------------------------------------------------
    // Gravity estimated from the IMU calibration module is used here to identify the horizontal plane.
    // Assumption: gravity is always perpendicular to the road surface
    // The estimated road plane is sampled for points in order to find rays from the camera
    void calculatePitchRollPoints(std::vector<dwVector2f>& imagePoints, const dwTransformation3f& rig2Camera, const dwVector3f gravity)
    {
        // Sample a "half circle" of directions in front of car in the xy ground plane
        imagePoints.clear();

        float32_t angleIncrement = M_PI / (NUM_PITCHROLL_POINTS - 1);
        float32_t angle          = 0.0f;
        for (auto i = 0u; i < NUM_PITCHROLL_POINTS; ++i, angle += angleIncrement)
        {
            float32_t rigDir[3];

            // Use the plane equation a(x-x0) + b(y-y0) + c(z-z0) = 0 to find points on the road surface,
            // where gravity = (a, b, c) and (x0, y0, z0) = (0, 0, 0) as the plane is assumed to pass through the rig origin
            // Points on the half circle lying on the plane orthogonal to gravity have coordinates (sin(a), cos(a), z)
            // where z solves the plane equation
            rigDir[0] = sin(angle);
            rigDir[1] = cos(angle);
            rigDir[2] = (-rigDir[0] * gravity.x - rigDir[1] * gravity.y) / gravity.z;

            // The ray in rig coordinates is transformed in camera coordinates
            float32_t cameraDir[3];
            Mat4_Rxp(cameraDir, rig2Camera.array, rigDir);

            // Discards points "behind" camera
            if (cameraDir[0] * m_cameraOpticalCenterDir.x +
                    cameraDir[1] * m_cameraOpticalCenterDir.y +
                    cameraDir[2] * m_cameraOpticalCenterDir.z >=
                0.1f)
            {

                float32_t u{0.0f}, v{0.0f};
                // The ray is projected back to the image plane, intersecting it into a single point
                CHECK_DW_ERROR(dwCameraModel_ray2Pixel(&u, &v,
                                                       cameraDir[0],
                                                       cameraDir[1],
                                                       cameraDir[2],
                                                       m_calibratedCamera));

                u *= m_scaledCameraDimensions.x;
                v *= m_scaledCameraDimensions.y;

                imagePoints.push_back({u, v});
            }
        }
    }

    // -----------------------------------------------------------------------------
    // Filter accelerometer data with exponential filter to remove noisy measurements
    void filterAccelerometer(dwVector3f& gravity, std::deque<dwVector3f>& bufferAcc)
    {
        // Add current gravity to buffer
        if (bufferAcc.size() == FILTER_SIZE)
        {
            bufferAcc.pop_front();
        }
        bufferAcc.push_back(gravity);

        // Apply exponential filter to accelerometer
        gravity = dwVector3f{0.0f, 0.0f, 0.0f};
        for (uint32_t i = 0; i < bufferAcc.size(); ++i)
        {
            gravity.x += m_exponentialFilter[i] * bufferAcc[i].x;
            gravity.y += m_exponentialFilter[i] * bufferAcc[i].y;
            gravity.z += m_exponentialFilter[i] * bufferAcc[i].z;
        }
    }

    // -----------------------------------------------------------------------------
    // Gravity is estimated from the IMU accelerometer measurement which is composed of three terms:
    // a = forward acceleration + centripetal acceleration (due to lateral motion) + gravity
    // Forward acceleration is estimated from the relative change in pose provided by the egomotion module
    // Centripetal acceleration is estimated as the cross product of speed and the angular velocity sensed by the gyroscope
    // Gravity is obtained by subtraction
    void estimateGravity()
    {
        // Find relative pose between current and previous time stamp
        dwTime_t currentCameraTimeStamp = m_currentImageRgbaTimestamp;
        dwTransformation3f pose;
        dwStatus status = dwEgomotion_computeRelativeTransformation(&pose, nullptr,
                                                                    currentCameraTimeStamp,
                                                                    m_previousCameraTimeStamp,
                                                                    m_egomotion);
        if (status != DW_SUCCESS)
        {
            m_previousCameraTimeStamp = currentCameraTimeStamp;
            return;
        }

        // Compute acceleration from EgoMotion to remove car acceleration from IMU reading
        float32_t dt                     = (currentCameraTimeStamp - m_previousCameraTimeStamp) / 1.0e6f; // elapsed time in [s]
        float32_t currentSpeed[3]        = {pose.array[12] / dt, pose.array[13] / dt, pose.array[14] / dt};
        float32_t currentAcceleration[3] = {(currentSpeed[0] - m_previousSpeed.x) / dt,
                                            (currentSpeed[1] - m_previousSpeed.y) / dt,
                                            (currentSpeed[2] - m_previousSpeed.z) / dt};

        // Compute centripetal acceleration to remove it from rig-transformed IMU acceleration
        float32_t gyroImu[3] = {static_cast<float32_t>(m_imuFrame.turnrate[0]), static_cast<float32_t>(m_imuFrame.turnrate[2]), static_cast<float32_t>(m_imuFrame.turnrate[2])};
        float32_t gyroRigNominal[3];
        Mat4_Rxp(gyroRigNominal, m_nominalIMU2Rig.array, gyroImu);
        float32_t centripetalAccelerationNominal[3];
        cross(centripetalAccelerationNominal, gyroRigNominal, currentSpeed);

        // Transform IMU acceleration in rig coordinate frame according to nominal calibration
        float32_t accImu[3] = {static_cast<float32_t>(m_imuFrame.acceleration[0]), static_cast<float32_t>(m_imuFrame.acceleration[1]), static_cast<float32_t>(m_imuFrame.acceleration[2])};
        float32_t accRigNominal[3];
        Mat4_Rxp(accRigNominal, m_nominalIMU2Rig.array, accImu);

        // Subtract car acceleration and centripetal acceleration from rig-transformed IMU acceleation
        removeExternalAcceleration(m_nominalGravity, accRigNominal, currentAcceleration, centripetalAccelerationNominal);

        // Apply exponential filter to nominal gravity to remove transient effects
        filterAccelerometer(m_nominalGravity, m_bufferAccNominal);

        // Transform IMU acceleration in rig coordinate frame according to calculated calibration
        if (m_status.state == DW_CALIBRATION_STATE_ACCEPTED)
        {
            float32_t gyroRigCalculated[3];
            Mat4_Rxp(gyroRigCalculated, m_calculatedIMU2Rig.array, gyroImu);
            float32_t centripetalAccelerationCalculated[3];
            cross(centripetalAccelerationCalculated, gyroRigCalculated, currentSpeed);

            float32_t accRigCalculated[3];
            Mat4_Rxp(accRigCalculated, m_calculatedIMU2Rig.array, accImu);

            // Subtract car acceleration and centripetal acceleration from rig-transformed IMU acceleation
            removeExternalAcceleration(m_calculatedGravity, accRigCalculated, currentAcceleration, centripetalAccelerationCalculated);

            // Apply exponential filter to calculated gravity to remove transient effects
            filterAccelerometer(m_calculatedGravity, m_bufferAccCalculated);
        }

        // Update previous data
        m_previousCameraTimeStamp = currentCameraTimeStamp;
        m_previousSpeed           = dwVector3f{currentSpeed[0], currentSpeed[1], currentSpeed[2]};
    }

    // -----------------------------------------------------------------------------
    // Gravity is obtained by subtracting forward acceleration and centripetal acceleration
    // from the total acceleration measured by the IMU
    void removeExternalAcceleration(dwVector3f& gravity,
                                    const float32_t* accelerometerRig,
                                    const float32_t* currentAcceleration,
                                    const float32_t* centripetalAcceleration)
    {
        gravity.x = accelerometerRig[0] - currentAcceleration[0] - centripetalAcceleration[0];
        gravity.y = accelerometerRig[1] - currentAcceleration[1] - centripetalAcceleration[1];
        gravity.z = accelerometerRig[2] - currentAcceleration[2] - centripetalAcceleration[2];
    }
};

//#######################################################################################
int main(int argc, const char** argv)
{
    const std::string samplePath = dw_samples::SamplesDataPath::get() + "/samples/recordings/suburb0/";

    // -------------------
    // Define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("rig", (samplePath + "imu_offset_rig.json").c_str(), "Path to rig configuration file"),
                              ProgramArguments::Option_t("camera-sensor", "0", "The index or name of the camera sensor from the rig file to use (only for visualization)"),
                              ProgramArguments::Option_t("can-sensor", "0", "The index or name of the CAN bus sensor from the rig-file to use"),
                              ProgramArguments::Option_t("imu-sensor", "0", "The index or name of the IMU sensor in the rig-file to calibrate"),
                          });

    IMUCalibrationSample app(args);

    if (args.get("offscreen") == "0" || args.get("offscreen") == "1")
    {
        app.initializeWindow("IMU Calibration Sample", 1280, 800, args.get("offscreen") == "1");
    }

    return app.run();
}
