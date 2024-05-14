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

// Driveworks sample framework
#include <framework/ChecksExt.hpp>
#include <framework/DriveWorksSample.hpp>
#include <framework/MathUtils.hpp>
#include <framework/Mat4.hpp>
#include <framework/SimpleStreamer.hpp>

// Core
#include <dw/core/logger/Logger.h>
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>

// Renderer
#include <dwvisualization/core/Renderer.h>

// Sensor manager
#include <dw/sensors/sensormanager/SensorManager.h>

// Rig configuration
#include <dw/rig/Rig.h>

// VehicleIO
#include <dw/control/vehicleio/VehicleIO.h>

// Radarmotion
#include <dw/egomotion/radar/DopplerMotionEstimator.h>

// Calibration
#include <dw/calibration/engine/Engine.h>

#include <fstream>
#include <memory>
#include <iostream>
#include <cstring>

using namespace dw_samples::common;

class RadarSelfCalibrationSample : public DriveWorksSample
{
    // ------------------------------------------------
    // Settings
    // ------------------------------------------------
    // Input file required for module to run
    std::string m_rigFile;
    std::string m_rigOutFile;

    uint32_t m_radarIndex  = std::numeric_limits<decltype(m_radarIndex)>::max();
    uint32_t m_canIndex    = std::numeric_limits<decltype(m_canIndex)>::max();
    uint32_t m_cameraIndex = std::numeric_limits<decltype(m_cameraIndex)>::max();

    // Contains relevant information for a particular sensor
    // which is read from rig configuration.
    struct SensorInfo
    {
        SensorInfo() = default;
        //! Reads information about one sensor from rig configuration
        uint32_t sensorId = 0;
        std::string parameters;
        std::string protocol;
        dwTransformation3f nominalSensorToRig = DW_IDENTITY_TRANSFORMATION3F;
        dwTransformation3f nominalRigToSensor = DW_IDENTITY_TRANSFORMATION3F;
        dwTransformation3f sensorToRig        = DW_IDENTITY_TRANSFORMATION3F;
        dwTransformation3f rigToSensor        = DW_IDENTITY_TRANSFORMATION3F;
        SensorInfo(dwRigHandle_t rigConfig, uint32_t sensorId)
        {
            this->sensorId = sensorId;
            CHECK_DW_ERROR_MSG(dwRig_getNominalSensorToRigTransformation(
                                   &this->nominalSensorToRig, sensorId, rigConfig),
                               "Error reading nominal sensor to rig transform.");
            Mat4_IsoInv(this->nominalRigToSensor.array, this->nominalSensorToRig.array);
            CHECK_DW_ERROR_MSG(
                dwRig_getSensorToRigTransformation(&this->sensorToRig, sensorId, rigConfig),
                "Error reading sensor to rig transform.");
            Mat4_IsoInv(this->nominalRigToSensor.array, this->nominalSensorToRig.array);
            Mat4_IsoInv(this->rigToSensor.array, this->sensorToRig.array);
            const char* parameters;
            CHECK_DW_ERROR_MSG(dwRig_getSensorParameter(&parameters, sensorId, rigConfig),
                               "Error reading sensor parameter from file.");
            this->parameters = parameters;
            const char* protocol;
            CHECK_DW_ERROR_MSG(dwRig_getSensorProtocol(&protocol, sensorId, rigConfig),
                               "Error reading sensor protocol from file.");
            this->protocol = protocol;
        }
    };

    struct RadarSensor : SensorInfo
    {
        RadarSensor() = default;
        RadarSensor(dwRigHandle_t rigConfig, uint32_t sensorId)
            : SensorInfo::SensorInfo(rigConfig, sensorId)
        {
        }
    };

    struct CANSensor : SensorInfo
    {
        CANSensor() = default;
        CANSensor(dwRigHandle_t rigConfig, uint32_t sensorId)
            : SensorInfo::SensorInfo(rigConfig, sensorId)
        {
        }
        std::string sOdometrySpeedFactor;
    };

    struct CAMERASensor : SensorInfo
    {
        CAMERASensor() = default;
        CAMERASensor(dwRigHandle_t rigConfig, uint32_t sensorId)
            : SensorInfo::SensorInfo(rigConfig, sensorId)
        {
        }

        ~CAMERASensor()
        {
            if (calibratedCamera)
                dwCameraModel_release(calibratedCamera);
        }

        dwCameraModelHandle_t calibratedCamera = DW_NULL_HANDLE;
    };

    //! Radar Sensor
    RadarSensor m_radar{};
    //! CAN Sensor
    CANSensor m_can{};
    //! CAMERA Sensor
    CAMERASensor m_camera{};
    //! Vehicle
    dwVehicle m_vehicle{};

    // ------------------------------------------------
    // Varous driveworks handles needed for sample
    // ------------------------------------------------
    dwContextHandle_t m_context                 = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz        = DW_NULL_HANDLE;
    dwSALHandle_t m_salHandle                   = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer               = DW_NULL_HANDLE;
    dwSensorManagerHandle_t m_sensorManager     = DW_NULL_HANDLE;
    dwCalibrationEngineHandle_t m_calibEngine   = DW_NULL_HANDLE;
    dwCalibrationRoutineHandle_t m_calibRoutine = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfig                   = DW_NULL_HANDLE;
    dwSensorHandle_t m_cameraSensor             = DW_NULL_HANDLE;
    dwRadarDopplerMotionHandle_t m_radarMotion  = DW_NULL_HANDLE;
    dwRenderBufferHandle_t m_pointCloud[DW_RADAR_RETURN_TYPE_COUNT]
                                       [DW_RADAR_RANGE_COUNT + 1];
    dwRenderBufferHandle_t m_radarPointCloudCameraCoordRenderBuffer = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO                                 = DW_NULL_HANDLE;

    // Updated radar extrinsics transformation
    dwTransformation3f m_updatedRadarToRig = DW_IDENTITY_TRANSFORMATION3F;
    // Inverse of the above
    dwTransformation3f m_updatedRigToRadar = DW_IDENTITY_TRANSFORMATION3F;
    // Calibration status
    dwCalibrationStatus m_status{};
    // Rotation correction computed by calibration
    dwTransformation3f m_correctionR = DW_IDENTITY_TRANSFORMATION3F;
    // Translation correction computed by calibration
    dwVector3f m_correctionT{};
    // default range type
    const dwRadarRange m_radarRangeType = DW_RADAR_RANGE_SHORT;
    // dwRadarScan
    dwRadarScan m_radarScan[DW_RADAR_RANGE_COUNT];
    // Point cloud buffer
    std::unique_ptr<uint8_t[]> m_pointCloudBuffer[DW_RADAR_RETURN_TYPE_COUNT]
                                                 [DW_RADAR_RANGE_COUNT];
    // How many radar packets of the current spin have been processed
    uint32_t m_radarDetectionCount = 0;
    // radar properties
    dwRadarProperties m_radarProperties{};

    // index of radar scan
    uint32_t m_scanIndex = 0;
    // Time stamp of last processed sensor event
    dwTime_t m_lastEventTime = {};
    // Time stamp of the first processed sensor event
    dwTime_t m_firstEventTime = {};
    // Global time offset to enable looping (simulating a continuous dataset)
    dwTime_t m_offsetTime = {};
    // radar detection roi
    dwRect m_radar2DRect;
    // image roi
    dwRect m_cameraRect;
    // text roi
    dwRect m_textRect;
    // Maximal number of points rendered
    static constexpr size_t m_maxNumPointsToRender = 20000;

    //------------------------------------------------
    // visualization specific
    //------------------------------------------------
    float32_t m_nearPlane;
    float32_t m_radius;
    float32_t m_vertAngle;
    float32_t m_horAngle;

    int32_t m_currentX;
    int32_t m_currentY;

    // for 3D Display
    dwMatrix4f m_modelview;
    dwMatrix4f m_projection;

    float32_t m_eye[3];
    float32_t m_center[3];
    float32_t m_up[3];
    const float32_t m_fovRads          = DEG2RAD(60.0f);
    static constexpr float32_t WORLD_Z = -2.5f;

    std::vector<dwVector2f> m_currentRadarPntsInCamera;
    std::vector<dwVector4f> m_currentRadarPointCloudCameraCoord;

    // Camera properties
    dwImageProperties m_cameraImageProperties{};
    std::unique_ptr<dw_samples::common::SimpleImageStreamerGL<>> m_streamerCUDA2GL;
    dwImageGL* m_currentGlFrame       = nullptr;
    dwImageHandle_t m_converterToRGBA = DW_NULL_HANDLE;

    static constexpr float32_t BORDEROFFSET = 20;

public:
    // initialize sample
    RadarSelfCalibrationSample(const ProgramArguments& args)
        : DriveWorksSample(args), m_lastEventTime(0), m_firstEventTime(0), m_offsetTime(0)
    {
        m_rigFile    = getArgument("rig");
        m_rigOutFile = getArgument("output-rig");
    }

    //#######################################################################################
    void initPointCloud()
    {
        //------------------------------------------
        // Get radar properties
        //------------------------------------------
        dwSensorHandle_t radarSensor;
        CHECK_DW_ERROR_MSG(dwSensorManager_getSensorHandle(&radarSensor, m_radarIndex, m_sensorManager), "Getting radar sensor handle failed");
        CHECK_DW_ERROR_MSG(dwSensorRadar_getProperties(&m_radarProperties, radarSensor), "Getting radar properties failed.");

        uint32_t maximumPoints = m_radarProperties.maxReturnsPerScan[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_LONG] * 10;

        m_pointCloudBuffer[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_LONG] =
            std::make_unique<uint8_t[]>(maximumPoints * sizeof(dwRadarDetection));

        maximumPoints = m_radarProperties.maxReturnsPerScan[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_SHORT] * 10;
        m_pointCloudBuffer[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_SHORT] =
            std::make_unique<uint8_t[]>(maximumPoints * sizeof(dwRadarDetection));
    }

    //#######################################################################################
    bool onInitialize() override
    {
        // -----------------------------
        // Initialize DriveWorks SDK context and SAL
        // -----------------------------
        {
            CHECK_DW_ERROR(dwLogger_initializeExtended(getConsoleLoggerExtendedCallback(true)));

            CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_INFO));

            dwContextParameters sdkParams = {};

#ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
#endif

            CHECK_DW_ERROR_MSG(dwInitialize(&m_context, DW_VERSION, &sdkParams),
                               "Cannot initialize DW SDK Context");
        }

        // -----------------------------------------
        // Initialize rig configuration module
        // -----------------------------------------
        {
            CHECK_DW_ERROR_MSG(
                dwRig_initializeFromFile(&m_rigConfig, m_context, m_rigFile.c_str()),
                "Error: Faile to initialize rig from file");
        }

        // -----------------------------------------
        // Initialize sensor manager for all sensors needed
        // -----------------------------------------
        {
            if (getWindow())
            {
                initRenderer();
            }
            initSensor();
            initPointCloud();
        }

        // -----------------------------------------
        // Initialize Radar Self-Calibration
        // -----------------------------------------
        {
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_initialize(&m_calibEngine, m_rigConfig, m_context),
                               "Error: Initialize calibration engine failed.");

            dwCalibrationRadarParams radarCalibrationParams{};
            enum OdometryPropertyEstimation
            {
                NO                          = 0,
                WHEEL_RADII                 = 1,
                VELOCITY_FACTOR             = 2,
                WHEEL_RADII_VELOCITY_FACTOR = 3
            };

            int32_t odometryPropertyEstimation = std::stoi(getArgument("calibrate-odometry-properties"));

            if (odometryPropertyEstimation == WHEEL_RADII_VELOCITY_FACTOR)
            {
                radarCalibrationParams.enableSpeedFactorEstimation = true;
                radarCalibrationParams.enableWheelRadiiEstimation  = true;
            }
            else
            {
                radarCalibrationParams.enableSpeedFactorEstimation = (odometryPropertyEstimation == OdometryPropertyEstimation::VELOCITY_FACTOR);
                radarCalibrationParams.enableWheelRadiiEstimation  = (odometryPropertyEstimation == OdometryPropertyEstimation::WHEEL_RADII);
            }
            radarCalibrationParams.radarProperties = &m_radarProperties;
            radarCalibrationParams.pitchMode       = DW_CALIBRATION_RADAR_PITCH_METHOD_NONE;
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_initializeRadar(&m_calibRoutine,
                                                                   m_radar.sensorId,
                                                                   m_can.sensorId,
                                                                   &radarCalibrationParams,
                                                                   m_calibEngine),
                               "Error: Initializing radar calibration failed.");

            dwRadarDopplerMotionParams radarMotionParams{};
            dwRadarDopplerMotion_getDefaultParams(&radarMotionParams);
            radarMotionParams.radarExtrinsics = m_radar.nominalSensorToRig;
            dwGenericVehicle genericVehicle{};
            CHECK_DW_ERROR(dwRig_getGenericVehicle(&genericVehicle, m_rigConfig));
            radarMotionParams.vehicle = genericVehicle;

            CHECK_DW_ERROR_MSG(
                dwRadarDopplerMotion_initialize(&m_radarMotion, &radarMotionParams, cudaStream_t(nullptr), m_context),
                "Fail to initialize RadarDopplerMotion");

            CHECK_DW_ERROR_MSG(
                dwCalibrationEngine_startCalibration(m_calibRoutine, m_calibEngine),
                "Error: Starting radar calibration failed.");
        }

        return true;
    }

    //#######################################################################################
    uint32_t getSensorIndex(dwSensorType sensorType, const std::string& sensorSelector) const
    {
        uint32_t sensorIndexInRig = 0;

        // Get sensor index
        try
        {
            uint32_t sensorIdx = std::stol(sensorSelector);

            // Determine sensor index in rig file using numeric sensor index
            CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&sensorIndexInRig, sensorType, sensorIdx, m_rigConfig));
        }
        catch (const std::invalid_argument& /* string to integer conversion failed - use string as sensor name */)
        {
            // Determine sensor index using sensor name
            CHECK_DW_ERROR(dwRig_findSensorByName(&sensorIndexInRig, sensorSelector.c_str(), m_rigConfig));
        }

        return sensorIndexInRig;
    }

    //#######################################################################################
    void initSensor()
    {
        //------------------------------------------
        // Get sensor indexes
        //------------------------------------------
        m_radarIndex  = getSensorIndex(DW_SENSOR_RADAR, getArgument("radar-sensor"));
        m_canIndex    = getSensorIndex(DW_SENSOR_CAN, getArgument("can-sensor"));
        m_cameraIndex = getSensorIndex(DW_SENSOR_CAMERA, getArgument("camera-sensor"));
        m_radar       = RadarSensor(m_rigConfig, m_radarIndex);
        m_can         = CANSensor(m_rigConfig, m_canIndex);
        m_camera      = CAMERASensor(m_rigConfig, m_cameraIndex);

        //------------------------------------------
        // Intitialize sensor manager
        //------------------------------------------
        dwSensorManagerParams smParams{};
        smParams.enableSensors[smParams.numEnableSensors++] = m_radarIndex;
        smParams.enableSensors[smParams.numEnableSensors++] = m_canIndex;
        smParams.enableSensors[smParams.numEnableSensors++] = m_cameraIndex;
        CHECK_DW_ERROR_MSG(dwSAL_initialize(&m_salHandle, m_context), "Error: Cannot initialize SAL");
        CHECK_DW_ERROR(dwSensorManager_initializeFromRigWithParams(&m_sensorManager, m_rigConfig, &smParams, 1024, m_salHandle));

        const dwVehicle* vehicle_;
        CHECK_DW_ERROR_MSG(dwRig_getVehicle(&vehicle_, m_rigConfig), "Error reading vehicle data");
        m_vehicle = *vehicle_;
        CHECK_DW_ERROR(dwVehicleIO_initialize(&m_vehicleIO, DW_VEHICLEIO_DATASPEED, &m_vehicle, m_context));

        //------------------------------------------
        // Camera sensor intitialization
        //------------------------------------------
        {
            // Create calibrated camera
            CHECK_DW_ERROR(dwCameraModel_initialize(&m_camera.calibratedCamera, m_cameraIndex, m_rigConfig));
            CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&m_cameraSensor, m_cameraIndex, m_sensorManager));
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&m_cameraImageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_cameraSensor));
#ifdef VIBRANTE
            m_cameraImageProperties.type = DW_IMAGE_NVMEDIA;
#else
            m_cameraImageProperties.type = DW_IMAGE_CUDA;
#endif
            m_cameraImageProperties.format = DW_IMAGE_FORMAT_RGB_FLOAT16;
            initCameraPipelineH264(m_cameraImageProperties);

            if (m_radarPointCloudCameraCoordRenderBuffer)
            {
                CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors(
                    static_cast<float32_t>(m_cameraImageProperties.width),
                    static_cast<float32_t>(m_cameraImageProperties.height),
                    m_radarPointCloudCameraCoordRenderBuffer));
            }
        }

        // -----------------------------
        // Start sensor manager
        // -----------------------------
        CHECK_DW_ERROR_MSG(dwSensorManager_start(m_sensorManager), "Error: Failed to start sensor manager");
    }

    //#######################################################################################
    void initCameraPipelineH264(const dwImageProperties& imageProperties)
    {
        dwImageProperties displayProperties = imageProperties;
        displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;
        CHECK_DW_ERROR_MSG(dwImage_create(&m_converterToRGBA, displayProperties, m_context),
                           "CREATE CAMERA IMAGE ERROR");
        if (getWindow())
        {
            m_streamerCUDA2GL = std::make_unique<dw_samples::common::SimpleImageStreamerGL<>>(
                displayProperties, 1000, m_context);
        }
    }

    //#########################################################################################
    bool processSensorEvent()
    {
        const dwSensorEvent* sensorEvent = nullptr;

        dwStatus status = dwSensorManager_acquireNextEvent(&sensorEvent, 0, m_sensorManager);
        if (status == DW_END_OF_STREAM)
        {
            return false;
        }
        CHECK_DW_ERROR_MSG(status, "Acquire next event failed");
        if (!m_firstEventTime)
        {
            m_firstEventTime = sensorEvent->timestamp_us;
        }
        m_lastEventTime = sensorEvent->timestamp_us;
        switch (sensorEvent->type)
        {
        case DW_SENSOR_CAN:
        {
            dwCANMessage msg = sensorEvent->canFrame;
            msg.timestamp_us += m_offsetTime;
            CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&msg, sensorEvent->sensorIndex, m_vehicleIO));

            dwVehicleIOSafetyState vehicleIOSafetyState{};
            dwVehicleIONonSafetyState vehicleIONonSafetyState{};
            dwVehicleIOActuationFeedback vehicleIOActuationFeedback{};
            CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&vehicleIOSafetyState, m_vehicleIO));
            CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&vehicleIONonSafetyState, m_vehicleIO));
            CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&vehicleIOActuationFeedback, m_vehicleIO));
            CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIONonSafetyState(&vehicleIONonSafetyState, m_can.sensorId, m_calibEngine));
            CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIOActuationFeedback(&vehicleIOActuationFeedback, m_can.sensorId, m_calibEngine));
            CHECK_DW_ERROR(dwRadarDopplerMotion_addVehicleIOState(&vehicleIOSafetyState, &vehicleIONonSafetyState, &vehicleIOActuationFeedback, m_radarMotion))
        }
        break;
        case DW_SENSOR_CAMERA:
        {
            dwImageHandle_t nextFrameCUDA = nullptr;
            CHECK_DW_ERROR_MSG(dwSensorCamera_getImage(&nextFrameCUDA,
                                                       DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                                       sensorEvent->camFrames[0]),
                               "GET IMAGE ERROR");
            CHECK_DW_ERROR_MSG(dwImage_copyConvert(m_converterToRGBA, nextFrameCUDA, m_context),
                               "Convert Image wrong");
            if (getWindow())
            {
                dwImageHandle_t frameGL = m_streamerCUDA2GL->post(m_converterToRGBA);
                CHECK_DW_ERROR(dwImage_getGL(&m_currentGlFrame, frameGL));
            }
        }
        break;
        case DW_SENSOR_RADAR:
        {
            m_radarScan[sensorEvent->radFrame->scanType.range] = *(sensorEvent->radFrame);
            const dwRadarScanType type                         = sensorEvent->radFrame->scanType;
            switch (type.returnType)
            {
            case DW_RADAR_RETURN_TYPE_DETECTION:
            {
                if (m_radarRangeType == type.range)
                {
                    m_radarDetectionCount = m_radarScan[type.range].numReturns;
                }

                if (m_radarDetectionCount > 0)
                {
                    std::memcpy(m_pointCloudBuffer[type.returnType][type.range].get(),
                                m_radarScan[type.range].data,
                                m_radarScan[type.range].numReturns * sizeof(dwRadarDetection));
                    m_radarScan[type.range].data =
                        reinterpret_cast<void*>(m_pointCloudBuffer[type.returnType][type.range].get());
                    handleRadarScan();
                }
            }
            break;
            case DW_RADAR_RETURN_TYPE_TRACK: break;
            case DW_RADAR_RETURN_TYPE_STATUS: break;
            case DW_RADAR_RETURN_TYPE_COUNT:
            default: std::cout << "Radar: Invalid point type received" << std::endl; break;
            }
        }
        break;
        case DW_SENSOR_IMU:
        case DW_SENSOR_GPS:
        case DW_SENSOR_TIME:
        case DW_SENSOR_LIDAR:
        case DW_SENSOR_DATA:
        case DW_SENSOR_COUNT:
        default: break;
        }

        CHECK_DW_ERROR_MSG(dwSensorManager_releaseAcquiredEvent(sensorEvent, m_sensorManager),
                           "Release sensor packet failed");

        return true;
    }
    //#########################################################################################.
    void handleRadarScan()
    {
        m_scanIndex++;
        dwRadarScan myscan = m_radarScan[m_radarRangeType];

        dwStatus status = dwRadarDopplerMotion_processAsync(&myscan, m_radarMotion);
        if (!status == DW_SUCCESS)
        {
            return;
        }

        dwRadarDopplerMotion bestPair;
        CHECK_DW_ERROR_MSG(dwRadarDopplerMotion_getMotion(&bestPair, m_radarMotion),
                           "dwRadarDopplerMotion: get estimated radar motion error!");
        bestPair.hostTimestamp_us += m_offsetTime;

        CHECK_DW_ERROR_MSG(dwCalibrationEngine_addRadarDopplerMotion(
                               &bestPair, m_radar.sensorId, m_calibEngine),
                           "dwRadarDopplerMotion: get estimated radar motion error!");
    }
    //#########################################################################################.
    bool acquireRadarScan()
    {
        m_radarDetectionCount = 0;
        while ((m_radarDetectionCount <= 0))
        {
            if (!processSensorEvent())
            {
                return false;
            }
        }
        return true;
    }

    //#########################################################################################.
    bool runLoop()
    {

        if (!acquireRadarScan())
        {
            return false;
        }
        // Get current calibration status and updated radar-to-rig calibration
        {
            CHECK_DW_ERROR_MSG(
                dwCalibrationEngine_getCalibrationStatus(&m_status, m_calibRoutine, m_calibEngine),
                "Error: Failed to get calibration status.");

            CHECK_DW_ERROR_MSG(dwCalibrationEngine_getSensorToRigTransformation(
                                   &m_updatedRadarToRig, m_calibRoutine, m_calibEngine),
                               "Error: Failed to get sensor to rig transformation from calibration engine.");

            // Calculate inverse of radarToRig
            Mat4_IsoInv(m_updatedRigToRadar.array, m_updatedRadarToRig.array);

            // Calculate rotation and translation deltas between nominal and updated calibration
            Mat4_AxB(m_correctionR.array,
                     m_updatedRadarToRig.array,
                     m_radar.nominalRigToSensor.array);
            m_correctionR.array[3 * 4] = m_correctionR.array[3 * 4 + 1] = m_correctionR.array[3 * 4 + 2] =
                0.f;
            m_correctionT.x = m_updatedRadarToRig.array[3 * 4 + 0] -
                              m_radar.nominalSensorToRig.array[3 * 4 + 0];
            m_correctionT.y = m_updatedRadarToRig.array[3 * 4 + 1] -
                              m_radar.nominalSensorToRig.array[3 * 4 + 1];
            m_correctionT.z = m_updatedRadarToRig.array[3 * 4 + 2] -
                              m_radar.nominalSensorToRig.array[3 * 4 + 2];
        }

        // get odometry speed factor calibration
        if (DW_CALIBRATION_STATE_ACCEPTED == m_status.state)
        {
            int32_t calib = std::stoi(getArgument("calibrate-odometry-properties"));

            if (calib & 2)
            {
                float32_t factor = 1.0f;
                if (DW_SUCCESS == dwCalibrationEngine_getOdometrySpeedFactor(&factor, m_calibRoutine, m_calibEngine))
                {
                    m_can.sOdometrySpeedFactor = std::to_string(factor);
                }
            }

            if (calib & 1)
            {
                for (auto w : {DW_VEHICLE_WHEEL_FRONT_LEFT, DW_VEHICLE_WHEEL_FRONT_RIGHT, DW_VEHICLE_WHEEL_REAR_LEFT, DW_VEHICLE_WHEEL_REAR_RIGHT})
                {
                    dwCalibrationEngine_getVehicleWheelRadius(&m_vehicle.wheelRadius[w], w, m_calibRoutine, m_calibEngine);
                }
            }
        }

        return true;
    }

    //#########################################################################################.
    void onProcess() override
    {
        if (isPaused())
            return;

        if (!runLoop())
        {
            stop();
            return;
        }
    }

    //#########################################################################################.
    void initRenderer()
    {
        initRenderDefaults();

        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));
        CHECK_DW_ERROR_MSG(dwRenderer_initialize(&m_renderer, m_viz),
                           "Cannot initialize Renderer, maybe no GL context available?");

        // Set some renderer defaults
        m_radar2DRect.width  = getWindowWidth() / 2;
        m_radar2DRect.height = getWindowHeight() / 2;
        m_radar2DRect.x      = getWindowWidth() / 2;
        m_radar2DRect.y      = 0;

        // set camera rectangle
        m_cameraRect.width  = getWindowWidth() / 2;
        m_cameraRect.height = getWindowHeight();
        m_cameraRect.x      = 0;
        m_cameraRect.y      = 0;

        prepareBufferRadarPoints();
        prepareBufferRadarInImage();

        m_textRect.x      = 0;
        m_textRect.y      = 0;
        m_textRect.width  = getWindowWidth() / 2;
        m_textRect.height = getWindowHeight();
    }

    //#########################################################################################.
    void prepareBufferRadarPoints()
    {
        // Prepare render buffer for point cloud
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32A32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        // Initialize render buffer for point cloud
        //  for (auto& buffer : m_pointCloud)
        {
            CHECK_DW_ERROR_MSG(
                dwRenderBuffer_initialize(&m_pointCloud[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_SHORT],
                                          layout,
                                          DW_RENDER_PRIM_POINTLIST,
                                          m_maxNumPointsToRender,
                                          m_viz),
                "Error: Initializing render buffer for point cloud failed.");

            CHECK_DW_ERROR_MSG(
                dwRenderBuffer_initialize(&m_pointCloud[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_LONG],
                                          layout,
                                          DW_RENDER_PRIM_POINTLIST,
                                          m_maxNumPointsToRender,
                                          m_viz),
                "Error: Initializing render buffer for point cloud failed.");
        }
    }

    //#########################################################################################.
    void prepareBufferRadarInImage()
    {
        dwRenderBufferVertexLayout layout;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        CHECK_DW_ERROR_MSG(dwRenderBuffer_initialize(&m_radarPointCloudCameraCoordRenderBuffer,
                                                     layout,
                                                     DW_RENDER_PRIM_POINTLIST,
                                                     m_maxNumPointsToRender,
                                                     m_viz),
                           "Error: Initializing render buffer for point cloud failed.");
    }

    //#########################################################################################
    void projectRadarPointsToCamera()
    {
        // transform radar points from radar frame to camera frame
        {
            m_currentRadarPointCloudCameraCoord.clear();

            dwTransformation3f const& radarToRig  = m_updatedRadarToRig;
            dwTransformation3f const& rigToCamera = m_camera.rigToSensor;

            dwTransformation3f const radarToCamera = rigToCamera * radarToRig;

            auto const* radarPoints = reinterpret_cast<dwRadarDetection const*>(
                m_radarScan[m_radarRangeType].data);
            for (size_t i = 0; i < m_radarDetectionCount; i++)
            {
                dwVector4f const radarPoint{
                    .x = radarPoints[i].x, .y = radarPoints[i].y, .z = 0.0, .w = 1.0f};
                m_currentRadarPointCloudCameraCoord.push_back(radarToCamera * radarPoint);
            }
        }

        // project points from camera domain to image domain
        {
            m_currentRadarPntsInCamera.clear();
            for (uint64_t j = 0; j < m_currentRadarPointCloudCameraCoord.size(); j++)
            {
                dwVector4f const currPoint = m_currentRadarPointCloudCameraCoord[j];
                if (currPoint.z < 0.0f)
                {
                    continue; // skip points behind camera
                }

                dwVector2f uv{};
                CHECK_DW_ERROR(dwCameraModel_ray2Pixel(
                    &uv.x, &uv.y, currPoint.x, currPoint.y, currPoint.z, m_camera.calibratedCamera));

                if (std::isnan(uv.x) || std::isnan(uv.y) || uv.x < 0.f || uv.y < 0.f ||
                    uv.x >= m_cameraImageProperties.width || uv.y >= m_cameraImageProperties.height)
                {
                    continue;
                }
                m_currentRadarPntsInCamera.push_back(uv);
            }
        }

        // update render buffer
        float32_t* buffer = nullptr;
        uint32_t maxVerts{};
        uint32_t stride{};
        CHECK_DW_ERROR_MSG(
            dwRenderBuffer_map(&buffer, &maxVerts, &stride, m_radarPointCloudCameraCoordRenderBuffer),
            "Error: Mapping render buffer failed.");
        uint32_t numVerts{};
        {
            for (size_t i = 0; i < m_currentRadarPntsInCamera.size(); i++)
            {
                if (numVerts >= maxVerts)
                    break;

                buffer[0] = m_currentRadarPntsInCamera[i].x;
                buffer[1] = m_currentRadarPntsInCamera[i].y;

                buffer += stride;
                numVerts++;
            }
        }
        CHECK_DW_ERROR_MSG(dwRenderBuffer_unmap(numVerts, m_radarPointCloudCameraCoordRenderBuffer),
                           "Error: Unmapping render buffer failed.");
    }

    void onReset() override
    {
        CHECK_DW_ERROR(dwVehicleIO_reset(m_vehicleIO));
        CHECK_DW_ERROR(dwRadarDopplerMotion_reset(m_radarMotion));

        // reset sensor manager
        CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_reset(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));

        m_offsetTime += m_lastEventTime - m_firstEventTime;
        m_lastEventTime  = 0;
        m_firstEventTime = 0;
    }

    //#######################################################################################
    void initRenderDefaults()
    {
        // initialize 3D view related variable
        m_radius    = 300;
        m_nearPlane = 180;
        m_vertAngle = DEG2RAD(89.9f); // bird view
        m_horAngle  = DEG2RAD(179.9f);

        m_currentX = -1;
        m_currentY = -1;

        // Initialize eye, up for bowl view
        m_eye[0] = m_radius * cos(m_vertAngle) * cos(m_horAngle);
        m_eye[1] = m_radius * cos(m_vertAngle) * sin(m_horAngle);
        m_eye[2] = m_radius * sin(m_vertAngle);

        m_up[0] = 0;
        m_up[1] = 0;
        m_up[2] = 1;

        m_center[0] = 0;
        m_center[1] = 0;
        m_center[2] = 0;
    }

    //#######################################################################################
    void onResizeWindow(int width, int height) override
    {
        m_radar2DRect.width  = width / 2;
        m_radar2DRect.height = height;
        m_radar2DRect.x      = width / 2;
        m_radar2DRect.y      = 0;

        m_cameraRect.width  = width / 2;
        m_cameraRect.height = height;
        m_cameraRect.x      = 0;
        m_cameraRect.y      = 0;
    }

    //#########################################################################################.
    static std::string renderRotationAsText(const dwTransformation3f& transformation)
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

    //#########################################################################################.
    static std::string renderFloatsText(const float* floats, uint32_t numFloats)
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

    //#########################################################################################.
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

    //#########################################################################################.
    void onRender() override
    {
        glDepthFunc(GL_ALWAYS);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        CHECK_GL_ERROR();

        auto const pointColor = m_status.state == dwCalibrationState::DW_CALIBRATION_STATE_ACCEPTED
                                    ? DW_RENDERER_COLOR_GREEN
                                    : DW_RENDERER_COLOR_RED;

        //-----------------------------------------
        // Render radar point cloud
        //-----------------------------------------
        accumulateCurrentPoints();

        /* set model and projection matrix for top-down view */
        lookAt(m_modelview.array, m_eye, m_center, m_up);
        ortho(
            m_projection.array, m_fovRads, 1.0f * m_radar2DRect.width / m_radar2DRect.height, m_nearPlane, 1000.0f);
        CHECK_DW_ERROR(dwRenderer_setPointSize(4.0f, m_renderer));
        CHECK_DW_ERROR(dwRenderer_setRect(m_radar2DRect, m_renderer));
        CHECK_DW_ERROR(dwRenderer_setModelView(&m_modelview, m_renderer));
        CHECK_DW_ERROR(dwRenderer_setProjection(&m_projection, m_renderer));
        CHECK_DW_ERROR(dwRenderer_setColor(pointColor, m_renderer));
        CHECK_DW_ERROR(dwRenderer_renderBuffer(
            m_pointCloud[DW_RADAR_RETURN_TYPE_DETECTION][m_radarRangeType], m_renderer));

        //---------------------------------------------
        // Render camera frame and overlay radar points on top
        //---------------------------------------------
        CHECK_DW_ERROR(dwRenderer_setRect(m_cameraRect, m_renderer));
        if (m_currentGlFrame)
        {
            CHECK_DW_ERROR(
                dwRenderer_renderTexture(m_currentGlFrame->tex, m_currentGlFrame->target, m_renderer));
        }

        // Detections
        projectRadarPointsToCamera();
        CHECK_DW_ERROR(dwRenderer_setPointSize(6.0f, m_renderer));
        CHECK_DW_ERROR(dwRenderer_setColor(pointColor, m_renderer));
        CHECK_DW_ERROR(dwRenderer_renderBuffer(m_radarPointCloudCameraCoordRenderBuffer, m_renderer));

        // render matrices and text to the screen
        std::stringstream ss;
        ss << "Rendering: accepted radar calibration is rendered in green\n\n";
        ss << "Radar scan index: " << m_scanIndex << "\n\n";
        ss << convertStatusToText() << "\n\n";
        ss << "Radar position correction:\n\n";
        ss << renderFloatsText(&m_correctionT.x, 3) << "\n\n";
        ss << "Radar rotation correction:\n\n";
        ss << renderRotationAsText(m_correctionR) << "\n";

        if (getArgument("calibrate-odometry-properties") != "0")
        {
            ss << "Radar corrected odometry speed factor: " << m_can.sOdometrySpeedFactor << "\n";
            ss << "      ";

            for (auto w : {DW_VEHICLE_WHEEL_FRONT_LEFT, DW_VEHICLE_WHEEL_FRONT_RIGHT, DW_VEHICLE_WHEEL_REAR_LEFT, DW_VEHICLE_WHEEL_REAR_RIGHT})
            {
                ss << "wheelRadius[" << w << "] = " << m_vehicle.wheelRadius[w] * 1000.f << "[mm], ";
            }
        }

        CHECK_DW_ERROR(dwRenderer_setRect(m_textRect, m_renderer));
        CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_DARKBLUE, m_renderer));
        CHECK_DW_ERROR(dwRenderer_setFont(DW_RENDER_FONT_VERDANA_20, m_renderer));
        float32_t currentHeight = getWindowHeight() - 2 * BORDEROFFSET;
        CHECK_DW_ERROR_MSG(dwRenderer_renderText(static_cast<int32_t>(BORDEROFFSET),
                                                 static_cast<int32_t>(currentHeight),
                                                 ss.str().c_str(), m_renderer),
                           "Error: Rendering text failed");
    }

    //#######################################################################################
    void accumulateCurrentPoints()
    {
        float32_t* map;
        uint32_t maxVerts, stride;

        {
            CHECK_DW_ERROR(
                dwRenderBuffer_map(&map,
                                   &maxVerts,
                                   &stride,
                                   m_pointCloud[DW_RADAR_RETURN_TYPE_DETECTION][m_radarRangeType]));

            for (size_t i = 0; i < m_radarScan[m_radarRangeType].numReturns; ++i)
            {
                auto* updatePoint = reinterpret_cast<dwRadarDetection const*>(
                    m_pointCloudBuffer[DW_RADAR_RETURN_TYPE_DETECTION][m_radarRangeType].get() +
                    i * sizeof(dwRadarDetection));

                dwVector4f pos{updatePoint->x, updatePoint->y, 0.f, 1.0f};
                pos                 = m_updatedRadarToRig * pos;
                map[i * stride + 0] = pos.x;
                map[i * stride + 1] = pos.y;

                map[i * stride + 2] = WORLD_Z;
                map[i * stride + 3] = 1.0f;

                map[i * stride + 4] = 0.5f;
                map[i * stride + 5] = 0.5f;
                map[i * stride + 6] = 0.5f;
                map[i * stride + 7] = 1.0f;
            }
            CHECK_DW_ERROR(
                dwRenderBuffer_unmap(m_radarScan[m_radarRangeType].numReturns,
                                     m_pointCloud[DW_RADAR_RETURN_TYPE_DETECTION][m_radarRangeType]));
        }
    }

    //#########################################################################################.
    void onRelease() override
    {
        if (m_status.state == DW_CALIBRATION_STATE_ACCEPTED && m_rigOutFile.size() != 0)
        {
            std::cout << "Saving updated rig configuration to file: " << m_rigOutFile << std::endl;

            CHECK_DW_ERROR_MSG(dwRig_setSensorToRigTransformation(
                                   &m_updatedRadarToRig, m_radar.sensorId, m_rigConfig),
                               "Error: setting updated sensor to rig transformation failed");

            int32_t calib = std::stoi(getArgument("calibrate-odometry-properties"));
            if (calib & 2)
            {
                if (m_can.sOdometrySpeedFactor.length())
                {
                    CHECK_DW_ERROR(dwRig_addOrSetSensorPropertyByName(m_can.sOdometrySpeedFactor.c_str(),
                                                                      "velocity_factor",
                                                                      m_can.sensorId,
                                                                      m_rigConfig));
                }
            }

            if (calib & 1)
            {
                CHECK_DW_ERROR(dwRig_setVehicle(&m_vehicle, m_rigConfig));
            }

            CHECK_DW_ERROR_MSG(dwRig_serializeToFile(m_rigOutFile.c_str(), m_rigConfig),
                               "Error: Serializing updated rig configuration to file failed.");
        }

        // -----------------------------------------
        // Stop sensor manager
        // -----------------------------------------
        if (m_sensorManager)
        {
            CHECK_DW_ERROR_MSG(dwSensorManager_stop(m_sensorManager),
                               "Error: Stopping sensor manager failed.");

            CHECK_DW_ERROR(dwSensorManager_release(m_sensorManager));
        }

        // -----------------------------------------
        // Release calibration engine
        // -----------------------------------------
        if (m_calibEngine)
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_release(m_calibEngine),
                               "Error: Releasing calibration engine failed.");

        // -----------------------------------------
        // Release rig configuration
        // -----------------------------------------
        if (m_rigConfig)
            CHECK_DW_ERROR_MSG(dwRig_release(m_rigConfig),
                               "Error: Releasing rig configuration failed.");

        // -----------------------------------------
        // Release vehicleIO handle
        // -----------------------------------------
        if (m_vehicleIO)
        {
            CHECK_DW_ERROR(dwVehicleIO_release(m_vehicleIO));
        }

        if (m_radarMotion)
        {
            CHECK_DW_ERROR(dwRadarDopplerMotion_release(m_radarMotion));
        }

        m_streamerCUDA2GL.reset();

        CHECK_DW_ERROR(dwImage_destroy(m_converterToRGBA));

        // -----------------------------------------
        // Release renderer and streamer
        // -----------------------------------------
        if (getWindow())
        {
            CHECK_DW_ERROR(dwRenderer_release(m_renderer));

            {
                CHECK_DW_ERROR_MSG(
                    dwRenderBuffer_release(m_pointCloud[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_SHORT]),
                    "Error: Releasing render buffer for point cloud failed");
                CHECK_DW_ERROR_MSG(
                    dwRenderBuffer_release(m_pointCloud[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_LONG]),
                    "Error: Releasing render buffer for point cloud failed");
                CHECK_DW_ERROR_MSG(
                    dwRenderBuffer_release(m_radarPointCloudCameraCoordRenderBuffer),
                    "Error: Releasing render buffer for point cloud failed");
            }

            CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        }

        // -----------------------------------------
        // Release DriveWorks handles, sdkContext and SAL
        // -----------------------------------------
        {
            CHECK_DW_ERROR_MSG(dwSAL_release(m_salHandle), "Error: Releasing SAL failed");
            CHECK_DW_ERROR(dwRelease(m_context));
            CHECK_DW_ERROR(dwLogger_release());
        }
    }
};

//------------------------------------------------------------------------------
int32_t main(int32_t argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    const std::string defRigFile =
        (dw_samples::SamplesDataPath::get() + "/samples/recordings/highway0/rig8Radars-wrong-yaw.json");
    using Option = ProgramArguments::Option_t;

    ProgramArguments args(
        argc,
        argv,
        {Option("rig", defRigFile.c_str(), "Path to input rig calibration file."),
         Option("output-rig", "", "Path where updated rig calibration file should be written."),
         Option("radar-sensor", "0", "The index or name of the lidar sensor from the rig file to calibrate"),
         Option("can-sensor", "0", "The index or name of the CAN bus sensor from the rig-file to use"),
         Option("camera-sensor", "0", "The index or name of the camera sensor from the rig file to calibrate"),
         Option("calibrate-odometry-properties",
                "1",
                "0: disabled, 1: will calibrate wheel radius, 2: will calibrate velocity_factor as CAN sensor property, 3: will do both")});

    // -------------------
    // initialize and start a window application
    // -------------------
    RadarSelfCalibrationSample app(args);

    if (args.get("offscreen") != "2")
    {
        app.initializeWindow("Radar Self Calibration Sample", 1024 * 2, 800, args.get("offscreen") == "1");
    }
    return app.run();
}
