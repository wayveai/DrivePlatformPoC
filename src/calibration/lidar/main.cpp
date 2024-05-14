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

// Driveworks sample includes
#include <dw/core/base/Version.h>
#include <framework/ChecksExt.hpp>
#include <framework/DriveWorksSample.hpp>
#include <framework/Mat4.hpp>
#include <framework/MathUtils.hpp>

// Renderer
#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>

// Point Cloud Processing
#include <dw/pointcloudprocessing/accumulator/PointCloudAccumulator.h>
#include <dw/pointcloudprocessing/icp/PointCloudICP.h>

// Egomotion
#include <dw/egomotion/base/Egomotion.h>

// Sensor manager and cand interpreter
#include <dw/sensors/sensormanager/SensorManager.h>
#include <dw/sensors/canbus/Interpreter.h>

// Rig configuration
#include <dw/rig/Rig.h>

// Lidar self-calibration
#include <dw/calibration/engine/Engine.h>

#include <array>
#include <algorithm>

using namespace dw_samples::common;

using std::cout;
using std::endl;

//------------------------------------------------------------------------------
// Lidar self-calibration sample
// The sample demonstrates how to use the Driveworks Lidar self-calibration API
// in combination with ego-motion and ICP
//------------------------------------------------------------------------------
class LidarSelfCalibrationSample : public DriveWorksSample
{
    // ------------------------------------------------
    // Types
    // ------------------------------------------------
    // A point used in DW Lidar module.
    typedef dwLidarPointXYZI dwPoint;
    // Point could data:
    typedef std::vector<dwPoint> PointCloud;

    // Contains all data pertaining to a particular lidar sweep
    struct LidarSweep
    {
        // Time stamp for sweep (average of first and last package added)
        dwTime_t timestamp;
        // Point cloud containing all valid points of sweep
        PointCloud points;
    };

    // Contains relevant information for a particular sensor
    // which is read from rig configuration.
    struct SensorInfo
    {
        uint32_t sensorId;
        std::string name;
        std::string parameters;
        std::string protocol;
        dwTransformation3f nominalSensorToRig;
        dwTransformation3f nominalRigToSensor;
    };

    // ------------------------------------------------
    // Settings
    // ------------------------------------------------
    // Input file required for module to run
    std::string m_rigFile;
    std::string m_rigOutFile;
    // Whether or not the app should run once through the data and then exit (rather than looping).
    bool m_runOnce;
    // Whether or not ego-motion pose is fed to lidar calibration (in addition to ICP pose)
    bool m_useEgoPose;
    // Whether or not print estimated transformation together with Lidar sweep number
    bool m_verbose;

    // ------------------------------------------------
    // Global Constants:
    // ------------------------------------------------
    // Maximal number of points rendered
    const uint32_t MAX_POINTS_RENDER = 64000;
    // Border on left side for rendering text [pixel]
    static constexpr float32_t BORDEROFFSET = 20;
    // Lidar accumulator filter window size
    static constexpr uint32_t FILTER_WINDOW_SIZE = 4;
    // By how much should vehicle contour be scaled for rendering
    static constexpr float32_t CONTOUR_SCALE = 3.0f;
    // CAN speed and steering factors
    static constexpr float32_t CAN_SPEED_FACTOR    = 0.277778f;    // kph -> m/s
    static constexpr float32_t CAN_STEERING_FACTOR = 0.001179277f; // steeringWheelAngle -> steeringAngle = deg2rad(a/14.8)

    // ------------------------------------------------
    // Application specific variables:
    // ------------------------------------------------
    // Counter for currently processed sweep
    uint32_t m_sweepNum;
    // Calibration status
    dwCalibrationStatus m_status{};
    // Rotation correction computed by calibration
    dwTransformation3f m_correctionR = DW_IDENTITY_TRANSFORMATION3F;
    // Translation correction computed by calibration
    dwVector3f m_correctionT{};

    // Time stamp of last processed sensor event
    dwTime_t m_lastEventTime;
    // Time stamp of the first processed sensor event
    dwTime_t m_firstEventTime;
    // Global time offset to enable looping (simulating a continuous dataset)
    dwTime_t m_offsetTime;
    // Time stamp of first lidar packet added per sweep
    dwTime_t m_firstLidarPacketTime;
    // Time stamp of last lidar packet added per sweep
    dwTime_t m_lastLidarPacketTime;
    // Time for which last event was added to egomotion
    dwTime_t m_lastEgoMotionAddedTime;
    // Time at which egomotion update() was last called
    dwTime_t m_lastEgoMotionUpdateTime;

    // Information for the different sensors used
    SensorInfo m_lidarInfo;
    SensorInfo m_imuInfo;
    SensorInfo m_canInfo;
    uint32_t m_lidarIndex = std::numeric_limits<decltype(m_lidarIndex)>::max();
    uint32_t m_imuIndex   = std::numeric_limits<decltype(m_imuIndex)>::max();
    uint32_t m_canIndex   = std::numeric_limits<decltype(m_canIndex)>::max();

    // Vehicle
    dwVehicle m_vehicle;

    // Properties of lidar sensor
    dwLidarProperties m_lidarProperties{};

    // Current delta pose in rig frame computed using Egomotion
    dwTransformation3f m_currentToPreviousRigEgo = DW_IDENTITY_TRANSFORMATION3F;
    // Updated lidar extrinsics transformation
    dwTransformation3f m_updatedLidarToRig = DW_IDENTITY_TRANSFORMATION3F;
    // Inverse of the above
    dwTransformation3f m_updatedRigToLidar = DW_IDENTITY_TRANSFORMATION3F;

    dwVector3f m_centerViewDiff{0, 0, 5};

    // We keep two lidar sweeps around (previous and current)
    LidarSweep m_current;
    LidarSweep m_previous;

    dwPointCloud m_currentGPU{};
    dwPointCloud m_previousGPU{};

    // Size of a depth map
    dwVector2ui m_depthMapSize{};

    // ------------------------------------------------
    // Varous driveworks handles needed for sample
    // ------------------------------------------------
    dwContextHandle_t m_context                   = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz          = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine         = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                           = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer                 = DW_NULL_HANDLE;
    dwPointCloudICPHandle_t m_icpHandle           = DW_NULL_HANDLE;
    dwEgomotionHandle_t m_egomotion               = DW_NULL_HANDLE;
    dwSensorManagerHandle_t m_sensorManager       = DW_NULL_HANDLE;
    dwPointCloudAccumulatorHandle_t m_accumulator = DW_NULL_HANDLE;
    dwCalibrationRoutineHandle_t m_calibRoutine   = DW_NULL_HANDLE;
    dwCalibrationEngineHandle_t m_calibEngine     = DW_NULL_HANDLE;
    dwRenderBufferHandle_t m_pointCloud           = DW_NULL_HANDLE;
    dwRenderBufferHandle_t m_contour              = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfig                     = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO               = DW_NULL_HANDLE;

public:
    // initialize sample
    LidarSelfCalibrationSample(const ProgramArguments& args)
        : DriveWorksSample(args), m_sweepNum(0), m_lastEventTime(0), m_firstEventTime(0), m_offsetTime(0), m_firstLidarPacketTime(0), m_lastLidarPacketTime(0), m_lastEgoMotionAddedTime(0), m_lastEgoMotionUpdateTime(0)
    {
        m_rigFile    = getArgument("rig");
        m_rigOutFile = getArgument("output-rig");
        m_runOnce    = getArgument("run-once") != "0";
        m_useEgoPose = getArgument("use-ego-pose") != "0";
        m_verbose    = getArgument("verbose") != "0";
    }

    /// -----------------------------
    /// Initialize sample application
    /// -----------------------------
    bool onInitialize() override
    {
        cout << std::setprecision(15);

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);

            CHECK_DW_ERROR_MSG(dwSAL_initialize(&m_sal, m_context),
                               "Error: Cannot initialize SAL");
        }

        // -----------------------------------------
        // Initialize Renderer
        // -----------------------------------------
        if (getWindow())
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

            // init render engine with default params
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, static_cast<uint32_t>(getWindowWidth()), static_cast<uint32_t>(getWindowHeight())));
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

            CHECK_DW_ERROR_MSG(dwRenderer_initialize(&m_renderer, m_viz),
                               "Error: Cannot initialize Renderer, maybe no GL context available?");
            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            CHECK_DW_ERROR_MSG(dwRenderer_setRect(rect, m_renderer),
                               "Error: Setting renderer rectangle failed.");

            CHECK_DW_ERROR_MSG(dwRenderer_setFont(DW_RENDER_FONT_VERDANA_24, m_renderer),
                               "Error: Setting renderer font failed.");

            prepareRenderBuffers();
        }

        // -----------------------------------------
        // Initialize rig configuration module
        // -----------------------------------------
        {
            CHECK_DW_ERROR_MSG(dwRig_initializeFromFile(&m_rigConfig, m_context, m_rigFile.c_str()),
                               "Error: Faile to initialize rig from file");

            m_lidarInfo = readSensorInformation(m_rigConfig, "lidar:*");
            m_imuInfo   = readSensorInformation(m_rigConfig, "imu");
            m_canInfo   = readSensorInformation(m_rigConfig, "can:vehicle");

            const dwVehicle* vehicle;
            CHECK_DW_ERROR_MSG(dwRig_getVehicle(&vehicle, m_rigConfig),
                               "Error: Failed to read vehicle data");
            m_vehicle = *vehicle;
        }

        // -----------------------------------------
        // Initialize sensor manager for all sensors needed
        // -----------------------------------------
        {
            initializeSensorManager();
        }

        // -----------------------------------------
        // Initialize lidar accumulator
        // -----------------------------------------
        {
            dwPointCloudAccumulatorParams params{};
            CHECK_DW_ERROR(dwPointCloudAccumulator_getDefaultParams(&params));
            params.minDistanceMeter         = 0.3F; // The minimum working distance for Hesai: points with depth smaller than that are blockage indicators.
            params.memoryType               = DW_MEMORY_TYPE_CUDA;
            params.filterWindowSize         = FILTER_WINDOW_SIZE;
            params.organized                = true;
            params.enableZeroCrossDetection = true;

            CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_initialize(&m_accumulator, &params, &m_lidarProperties, m_context),
                               "Error: Initializing point cloud accumulator failed.");

            m_currentGPU.type     = DW_MEMORY_TYPE_CUDA;
            m_currentGPU.format   = DW_POINTCLOUD_FORMAT_XYZI;
            m_currentGPU.capacity = m_lidarProperties.pointsPerSpin;

            CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_currentGPU));

            m_previousGPU.type     = DW_MEMORY_TYPE_CUDA;
            m_previousGPU.format   = DW_POINTCLOUD_FORMAT_XYZI;
            m_previousGPU.capacity = m_lidarProperties.pointsPerSpin;

            CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_previousGPU));
        }

        // -----------------------------------------
        // Initialize ICP module
        // -----------------------------------------
        {
            //We are using point cloud version for smaple,currently.
            dwPointCloudICPParams params{};
            CHECK_DW_ERROR(dwPointCloudICP_getDefaultParams(&params));
            params.icpType       = DW_POINT_CLOUD_ICP_TYPE_DEPTH_MAP;
            params.maxIterations = 12;

            CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_getSweepSize(&m_depthMapSize, m_accumulator),
                               "Errror: Getting accumulator sweep size failed.");

            params.depthmapSize = m_depthMapSize;
            params.maxPoints    = params.depthmapSize.x * params.depthmapSize.y;

            CHECK_DW_ERROR_MSG(dwPointCloudICP_initialize(&m_icpHandle, &params, m_context),
                               "Error: ICP Init failed");

            m_previous.points.resize(params.maxPoints);
            m_current.points.resize(params.maxPoints);
        }

        // -----------------------------
        // Initialize Ego Motion
        // -----------------------------
        {
            dwEgomotionParameters egomotionParameters{};

            egomotionParameters.imu2rig         = m_imuInfo.nominalSensorToRig;
            egomotionParameters.vehicle         = m_vehicle;
            egomotionParameters.motionModel     = DW_EGOMOTION_IMU_ODOMETRY;
            egomotionParameters.automaticUpdate = true;

            CHECK_DW_ERROR_MSG(dwEgomotion_initialize(&m_egomotion, &egomotionParameters, m_context),
                               "Error: Initialization of egomotion module failed.");
        }

        // -----------------------------------------
        // Initialize Lidar Self-Calibration
        // -----------------------------------------
        {
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_initialize(&m_calibEngine, m_rigConfig, m_context),
                               "Error: Initialize calibration engine failed.");

            dwCalibrationLidarParams lidarCalibrationParams{};
            lidarCalibrationParams.lidarProperties = &m_lidarProperties;
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_initializeLidar(&m_calibRoutine,
                                                                   m_lidarInfo.sensorId,
                                                                   m_canIndex,
                                                                   &lidarCalibrationParams,
                                                                   cudaStreamDefault,
                                                                   m_calibEngine),
                               "Error: Initializing lidar calibration failed.");

            CHECK_DW_ERROR_MSG(dwCalibrationEngine_startCalibration(m_calibRoutine, m_calibEngine),
                               "Error: Starting lidar calibration failed.");
        }

        // -----------------------------------------
        // Prepare render buffer for contour of vehicle in rig space XY plane
        // -----------------------------------------
        if (getWindow())
        {
            float32_t xMin = 0.5f * (m_vehicle.wheelbase - m_vehicle.length * CONTOUR_SCALE);
            float32_t xMax = 0.5f * (m_vehicle.wheelbase + m_vehicle.length * CONTOUR_SCALE);
            float32_t yMin = -0.5f * m_vehicle.width * CONTOUR_SCALE;
            float32_t yMax = 0.5f * m_vehicle.width * CONTOUR_SCALE;

            float32_t* map;
            uint32_t maxVerts;
            uint32_t stride;
            CHECK_DW_ERROR_MSG(dwRenderBuffer_map(&map, &maxVerts, &stride, m_contour),
                               "Error: Render buffer map failed.");

            dwVector3f* buffer = reinterpret_cast<dwVector3f*>(map);

            buffer[0].x = xMin;
            buffer[0].y = yMin;
            buffer[0].z = 0.f;
            buffer[1].x = xMax;
            buffer[1].y = yMin;
            buffer[1].z = 0.f;
            buffer[2].x = xMax;
            buffer[2].y = yMax;
            buffer[2].z = 0.f;
            buffer[3].x = xMin;
            buffer[3].y = yMax;
            buffer[3].z = 0.f;
            buffer[4]   = buffer[0];

            CHECK_DW_ERROR_MSG(dwRenderBuffer_unmap(5, m_contour),
                               "Error: Render buffer unmap failed");
        }

        // -----------------------------------------
        // initialize target point cloud
        // -----------------------------------------
        return getSweepGPU(&m_previousGPU);
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

    static SensorInfo readSensorInformation(dwRigHandle_t rigConfig,
                                            const std::string& sensorSearchPattern)
    {
        uint32_t sensorId;
        // Find sensor ID for lidar
        CHECK_DW_ERROR_MSG(dwRig_findSensorByName(&sensorId, sensorSearchPattern.c_str(), rigConfig),
                           "Error: Could not find sensor in rig configuration using search pattern: " + sensorSearchPattern);

        SensorInfo sensorInfo;
        sensorInfo.sensorId = sensorId;

        // Read extrinsics
        CHECK_DW_ERROR_MSG(dwRig_getNominalSensorToRigTransformation(&sensorInfo.nominalSensorToRig, sensorId, rigConfig),
                           "Error reading nominal sensor to rig transform.");
        Mat4_IsoInv(sensorInfo.nominalRigToSensor.array, sensorInfo.nominalSensorToRig.array);

        // Read sensor name
        const char* sensorName;
        CHECK_DW_ERROR_MSG(dwRig_getSensorName(&sensorName, sensorId, rigConfig),
                           "Error: Could not get sensor name");
        sensorInfo.name = sensorName;

        // Read parameter string
        const char* parameters;
        CHECK_DW_ERROR_MSG(dwRig_getSensorParameter(&parameters, sensorId, rigConfig),
                           "Error reading sensor parameter from file.");
        sensorInfo.parameters = parameters;

        // Read protocol string
        const char* protocol;
        CHECK_DW_ERROR_MSG(dwRig_getSensorProtocol(&protocol, sensorId, rigConfig),
                           "Error reading sensor protocol from file.");
        sensorInfo.protocol = protocol;

        cout << "Found sensor " << sensorInfo.name << " with sensor ID: " << sensorInfo.sensorId << endl;

        return sensorInfo;
    }

    static std::string getFullParameterString(const std::string& parameters, const std::string& directory)
    {
        if (parameters.find("file=") == 0)
        {
            return "file=" + directory + "/" + parameters.substr(5);
        }

        return parameters;
    }

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

    void initializeSensorManager()
    {
        dwSensorManagerParams smParams{};

        m_lidarIndex                                        = getSensorIndex(DW_SENSOR_LIDAR, getArgument("lidar-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_lidarIndex;

        m_imuIndex                                          = getSensorIndex(DW_SENSOR_IMU, getArgument("imu-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_imuIndex;

        m_canIndex                                          = getSensorIndex(DW_SENSOR_CAN, getArgument("can-sensor"));
        smParams.enableSensors[smParams.numEnableSensors++] = m_canIndex;

        // Initialize DriveWorks SensorManager directly from rig file
        CHECK_DW_ERROR(dwSensorManager_initializeFromRigWithParams(&m_sensorManager, m_rigConfig, &smParams, 1024, m_sal));

        dwSensorHandle_t lidarSensor;
        CHECK_DW_ERROR_MSG(dwSensorManager_getSensorHandle(&lidarSensor, m_lidarIndex, m_sensorManager),
                           "Error: Getting lidar sensor handle failed");

        CHECK_DW_ERROR_MSG(dwSensorLidar_getProperties(&m_lidarProperties, lidarSensor),
                           "Error: Getting lidar properties failed");

        CHECK_DW_ERROR(dwVehicleIO_initialize(&m_vehicleIO, DW_VEHICLEIO_DATASPEED, &m_vehicle, m_context));

        // -----------------------------
        // Start sensor manager
        // -----------------------------
        CHECK_DW_ERROR_MSG(dwSensorManager_start(m_sensorManager),
                           "Error: Failed to start sensor manager");
    }

    dwTransformation3f getEgomotionTransform(dwTime_t A, dwTime_t B)
    {
        dwTransformation3f poseAtoB;
        dwStatus status = dwEgomotion_computeRelativeTransformation(&poseAtoB, nullptr, A + m_offsetTime, B + m_offsetTime, m_egomotion);

        if (status == DW_NOT_AVAILABLE)
        {
            return DW_IDENTITY_TRANSFORMATION3F;
        }
        CHECK_DW_ERROR_MSG(status, "Error: getting egomotion transform failed.");
        return poseAtoB;
    }

    void stopSensorManager()
    {
        if (m_sensorManager)
        {
            CHECK_DW_ERROR_MSG(dwSensorManager_stop(m_sensorManager),
                               "Error: Stopping sensor manager failed.");

            dwSensorManager_release(m_sensorManager);
        }
        if (m_vehicleIO)
        {
            CHECK_DW_ERROR_MSG(dwVehicleIO_release(m_vehicleIO),
                               "Error: Releasing VehicleIO failed.");
        }
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if (m_status.state == DW_CALIBRATION_STATE_ACCEPTED && m_rigOutFile.size() != 0)
        {
            cout << "Saving updated rig configuration to file: " << m_rigOutFile << endl;

            CHECK_DW_ERROR_MSG(dwRig_setSensorToRigTransformation(&m_updatedLidarToRig, m_lidarInfo.sensorId, m_rigConfig),
                               "Error: setting updated sensor to rig transformation failed");
            CHECK_DW_ERROR_MSG(dwRig_serializeToFile(m_rigOutFile.c_str(), m_rigConfig),
                               "Error: Serializing updated rig configuration to file failed.");
        }

        // -----------------------------------------
        // Stop sensor manager
        // -----------------------------------------
        stopSensorManager();

        // -----------------------------------------
        // Release egomotion
        // -----------------------------------------
        if (m_egomotion)
        {
            CHECK_DW_ERROR_MSG(dwEgomotion_release(m_egomotion),
                               "Error: Releasing egomotion failed.");
        }

        // -----------------------------------------
        // Release icp
        // -----------------------------------------
        if (m_icpHandle)
        {
            CHECK_DW_ERROR_MSG(dwPointCloudICP_release(m_icpHandle),
                               "Error: Releasing ICP handle failed.");
        }

        // -----------------------------------------
        // Release gpu point clouds
        // -----------------------------------------
        {
            CHECK_DW_ERROR_MSG(dwPointCloud_destroyBuffer(&m_currentGPU),
                               "Error: Releasing gpu point cloud failed.");
            CHECK_DW_ERROR_MSG(dwPointCloud_destroyBuffer(&m_previousGPU),
                               "Error: Releasing accumulator gpu buffer failed.");
        }

        // -----------------------------------------
        // Release lidar accumulator
        // -----------------------------------------
        if (m_accumulator)
        {
            CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_release(m_accumulator),
                               "Error: Releasing Lidar accumulator failed.");
        }

        // -----------------------------------------
        // Release calibration engine
        // -----------------------------------------

        if (m_calibEngine)
        {
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_release(m_calibEngine),
                               "Error: Releasing calibration engine failed.");
        }

        // -----------------------------------------
        // Release rig configuration
        // -----------------------------------------
        if (m_rigConfig)
        {
            CHECK_DW_ERROR_MSG(dwRig_release(m_rigConfig),
                               "Error: Releasing rig configuration failed.");
        }

        // -----------------------------------------
        // Release renderer and streamer
        // -----------------------------------------
        {
            if (m_renderEngine)
                CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));

            if (m_renderer)
                CHECK_DW_ERROR_MSG(dwRenderer_release(m_renderer),
                                   "Error: Releasing renderer failed");
            if (m_pointCloud)
                CHECK_DW_ERROR_MSG(dwRenderBuffer_release(m_pointCloud),
                                   "Error: Releasing render buffer for point cloud failed");
            if (m_contour)
                CHECK_DW_ERROR_MSG(dwRenderBuffer_release(m_contour),
                                   "Error: Releasing render buffer for contour failed");
        }

        // -----------------------------------------
        // Release DriveWorks handles, sdkContext and SAL
        // -----------------------------------------
        {
            if (m_sal)
                CHECK_DW_ERROR_MSG(dwSAL_release(m_sal),
                                   "Error: Releasing SAL failed");

            if (m_viz)
            {
                CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
            }
            CHECK_DW_ERROR(dwRelease(m_context));
            CHECK_DW_ERROR(dwLogger_release());
        }
    }

    //------------------------------------------------------------------------------
    // This function accumulates target points and transformed source points for visualization
    // Source points are transformed using ego-motion pose and current lidar calibration estimate
    void accumulateCurrentPoints()
    {
        // update render buffer to keep latest points
        float32_t* map;
        uint32_t maxVerts;
        uint32_t stride;

        CHECK_DW_ERROR_MSG(dwRenderBuffer_map(&map, &maxVerts, &stride, m_pointCloud),
                           "Error: Mapping render buffer failed.");

        maxVerts = std::min(maxVerts, MAX_POINTS_RENDER);

        const float32_t vizRatio = static_cast<float32_t>(maxVerts) / (m_previousGPU.size + m_currentGPU.size);
        uint32_t numVerts        = 0;
        float32_t* buffer        = map;

        {
            // First add points from previous sweep
            for (const dwPoint& pointInLidarFrame : m_previous.points)
            {
                if (numVerts >= maxVerts)
                    break;

                if (float32_t(rand()) / RAND_MAX >= vizRatio)
                    continue;

                dwPoint pointInRigFrame;
                Mat4_Axp(
                    reinterpret_cast<float32_t*>(&pointInRigFrame),
                    m_updatedLidarToRig.array,
                    reinterpret_cast<const float32_t*>(&pointInLidarFrame));

                buffer[0] = pointInRigFrame.x;
                buffer[1] = pointInRigFrame.y;
                buffer[2] = pointInRigFrame.z;
                buffer[3] = DW_RENDERER_COLOR_RED.x;
                buffer[4] = DW_RENDERER_COLOR_RED.y;
                buffer[5] = DW_RENDERER_COLOR_RED.z;

                buffer += stride;
                numVerts++;
            }

            // Then add points from current sweep, tranaformed into the previous frame
            for (const dwPoint& pointInLidarFrame : m_current.points)
            {
                if (numVerts >= maxVerts)
                    break;

                if (static_cast<float32_t>(rand()) / RAND_MAX >= vizRatio)
                    continue;

                dwTransformation3f currentLidarToPreviousRig = m_currentToPreviousRigEgo * m_updatedLidarToRig;

                dwPoint pointInRigFrame;
                Mat4_Axp(
                    reinterpret_cast<float32_t*>(&pointInRigFrame),
                    currentLidarToPreviousRig.array,
                    reinterpret_cast<const float32_t*>(&pointInLidarFrame));

                buffer[0] = pointInRigFrame.x;
                buffer[1] = pointInRigFrame.y;
                buffer[2] = pointInRigFrame.z;
                buffer[3] = DW_RENDERER_COLOR_GREEN.x;
                buffer[4] = DW_RENDERER_COLOR_GREEN.y;
                buffer[5] = DW_RENDERER_COLOR_GREEN.z;

                buffer += stride;
                numVerts++;
            }
        }

        CHECK_DW_ERROR_MSG(dwRenderBuffer_unmap(numVerts, m_pointCloud),
                           "Error: Unmapping render buffer failed.");
    }

    bool processSensorEvent(bool& lidarPacketAdded, bool& hasNewLidarSweep)
    {
        const dwSensorEvent* sensorEvent = nullptr;
        dwStatus status                  = dwSensorManager_acquireNextEvent(&sensorEvent, 0, m_sensorManager);
        if (status == DW_END_OF_STREAM)
        {
            return false;
        }

        CHECK_DW_ERROR_MSG(status, "Error: Acquire next event failed");

        // Check if time-stamps are increasing
        if (sensorEvent->timestamp_us < m_lastEventTime)
        {
            std::cerr << "timestamp is not increasing" << std::endl;
            return false;
        }
        if (!m_firstEventTime)
        {
            m_firstEventTime = sensorEvent->timestamp_us;
        }
        m_lastEventTime = sensorEvent->timestamp_us;

        switch (sensorEvent->type)
        {
        case DW_SENSOR_LIDAR:
        {
            assert(sensorEvent->sensorTypeIndex == 0);

            // Add lidar packet to lidar accumulator
            CHECK_DW_ERROR(dwPointCloudAccumulator_addLidarPacket(sensorEvent->lidFrame, m_accumulator));
            CHECK_DW_ERROR(dwPointCloudAccumulator_isReady(&hasNewLidarSweep, m_accumulator));

            lidarPacketAdded      = true;
            m_lastLidarPacketTime = sensorEvent->timestamp_us;
            break;
        }
        case DW_SENSOR_IMU:
        {
            assert(sensorEvent->sensorTypeIndex == 0);

            // Add IMU packet to egomotion
            dwIMUFrame msgIMU = sensorEvent->imuFrame;
            msgIMU.hostTimestamp += m_offsetTime;

            CHECK_DW_ERROR_MSG(dwEgomotion_addIMUMeasurement(&msgIMU, m_egomotion),
                               "Error: Adding IMU to Egomotion failed");

            m_lastEgoMotionAddedTime = sensorEvent->timestamp_us;
            break;
        }
        case DW_SENSOR_CAN:
        {
            assert(sensorEvent->sensorTypeIndex == 0);

            // Sample is looping over the same data -> need to simulate continuous CAN data by adding an extra offset to the timestamp
            dwCANMessage msg = sensorEvent->canFrame;
            msg.timestamp_us += m_offsetTime;

            CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&msg, sensorEvent->sensorIndex, m_vehicleIO));
            dwVehicleIOSafetyState vehicleIOSafetyState{};
            CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&vehicleIOSafetyState, m_vehicleIO));

            dwVehicleIONonSafetyState vehicleIONonSafetyState{};
            CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&vehicleIONonSafetyState, m_vehicleIO));
            dwVehicleIOActuationFeedback vehicleIOActuationFeedback{};
            CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&vehicleIOActuationFeedback, m_vehicleIO));

            CHECK_DW_ERROR(dwEgomotion_addVehicleIOState(&vehicleIOSafetyState, &vehicleIONonSafetyState, &vehicleIOActuationFeedback, m_egomotion));
            m_lastEgoMotionAddedTime = sensorEvent->timestamp_us;

            CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIONonSafetyState(&vehicleIONonSafetyState, m_canIndex, m_calibEngine));
            CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIOActuationFeedback(&vehicleIOActuationFeedback, m_canIndex, m_calibEngine));

            break;
        }
        case DW_SENSOR_GPS:
        case DW_SENSOR_CAMERA:
        case DW_SENSOR_RADAR:
        case DW_SENSOR_TIME:
        case DW_SENSOR_DATA:
        case DW_SENSOR_COUNT:
        default:
            throw std::runtime_error("Error: Received incorrect sensor event. Only IMU, CAN and LIDAR are used.");
        }

        CHECK_DW_ERROR_MSG(dwSensorManager_releaseAcquiredEvent(sensorEvent, m_sensorManager),
                           "Error: Release sensor packet failed");

        return true;
    }

    //------------------------------------------------------------------------------
    // Fetch a GPU sweep from lidar.
    bool getSweepGPU(dwPointCloud* buffer)
    {
        m_firstLidarPacketTime = DW_TIMEOUT_INFINITE;

        while (true)
        {
            bool lidarPacketAdded = false;
            bool hasNewLidarSweep = false;
            if (!processSensorEvent(lidarPacketAdded, hasNewLidarSweep))
            {
                // End of stream reached
                return false;
            }

            if (!lidarPacketAdded)
                continue;

            // Remember timestamp of first lidar packet added
            if (m_firstLidarPacketTime == DW_TIMEOUT_INFINITE)
                m_firstLidarPacketTime = m_lastLidarPacketTime;

            if (!hasNewLidarSweep)
                continue;

            CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_bindOutput(buffer, m_accumulator),
                               "Error: Bind output GPU sweep buffer to accumulator failed");
            CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_process(m_accumulator),
                               "Error: Get lidar sweep from GPU accumulator failed.");

            buffer->timestamp = (m_firstLidarPacketTime + m_lastLidarPacketTime) / 2;
            return buffer->size != 0;
        }
    }

    //------------------------------------------------------------------------------
    // Main loop that fetchs sweeps, calculates ICP pose and feeds data to calibration engine.
    // We need to sample the points down to a smaller size without which ICP module will throw.
    bool runLoop()
    {
        // get next source points from the GPU
        if (!getSweepGPU(&m_currentGPU))
            return false;

        m_current.timestamp  = m_currentGPU.timestamp;
        m_previous.timestamp = m_previousGPU.timestamp;

        // Get ego-motion pose between current and previous lidar sweep
        m_currentToPreviousRigEgo = getEgomotionTransform(m_currentGPU.timestamp, m_previousGPU.timestamp);

        // Compute relative pose between sweeps using ICP and feed to lidar calibration engine
        {
            // Compute prior pose for ICP by transforming from rig frame to lidar frame
            dwTransformation3f currentToPreviousLidarEgo = m_lidarInfo.nominalRigToSensor * m_currentToPreviousRigEgo * m_lidarInfo.nominalSensorToRig;

            CHECK_DW_ERROR_MSG(dwPointCloudICP_bindInput(&m_currentGPU, &m_previousGPU, &currentToPreviousLidarEgo, m_icpHandle), "Error binding ICP input");

            // Run ICP to compute delta pose in lidar frame
            dwTransformation3f currentToPreviousLidarICP{};
            CHECK_DW_ERROR_MSG(dwPointCloudICP_bindOutput(&currentToPreviousLidarICP, m_icpHandle), "Error binding ICP output");

            if (dwPointCloudICP_process(m_icpHandle) == DW_SUCCESS)
            {
                // Get some stats about the ICP performance
                dwPointCloudICPResultStats icpResultStats{};
                CHECK_DW_ERROR_MSG(dwPointCloudICP_getLastResultStats(&icpResultStats, m_icpHandle), "Error: Failed to get ICP result stats");

                if (m_verbose)
                {
                    std::cout << "Adding current ICP pose to calibration engine:" << currentToPreviousLidarICP << endl;
                }

                if (m_useEgoPose)
                {
                    if (m_verbose)
                    {
                        cout << "Adding Ego-Motion pose to calibration engine:" << m_currentToPreviousRigEgo << endl;
                    }
                    CHECK_DW_ERROR_MSG(dwCalibrationEngine_addLidarPose(&currentToPreviousLidarICP, &m_currentToPreviousRigEgo,
                                                                        m_current.timestamp + m_offsetTime, m_previous.timestamp + m_offsetTime, m_lidarInfo.sensorId, m_calibEngine),
                                       "Error: Failed to add lidar pose to calibration engine.");
                }
                else
                {
                    CHECK_DW_ERROR_MSG(dwCalibrationEngine_addLidarPose(&currentToPreviousLidarICP, nullptr, m_current.timestamp + m_offsetTime, m_previous.timestamp + m_offsetTime,
                                                                        m_lidarInfo.sensorId, m_calibEngine),
                                       "Error: Failed to add lidar pose to calibration engine.");
                }
                m_previous.timestamp = m_current.timestamp;
            }
            else
                logError("Error: ICP Failed");
        }

        // transfer the cuda to host for rendering
        CHECK_CUDA_ERROR(cudaMemcpy(m_previous.points.data(), m_previousGPU.points,
                                    sizeof(dwLidarPointXYZI) * m_previousGPU.size,
                                    cudaMemcpyDeviceToHost));
        CHECK_CUDA_ERROR(cudaMemcpy(m_current.points.data(), m_currentGPU.points,
                                    sizeof(dwLidarPointXYZI) * m_currentGPU.size,
                                    cudaMemcpyDeviceToHost));

        // Add point cloud from current sweep to lidar calibration engine
        {
            if (m_verbose)
            {
                cout << "Adding " << m_current.points.size() << " Lidar points to calibration engine." << endl;
            }

            auto* data = static_cast<dwVector4f*>(m_currentGPU.points);
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_addLidarPointCloud(data, m_currentGPU.size,
                                                                      m_current.timestamp + m_offsetTime, m_lidarInfo.sensorId, m_calibEngine),
                               "Error: Failed to add lidar sweep to calibration engine.");
        }

        // Get current calibration status and updated lidar-to-rig calibration
        {
            CHECK_DW_ERROR_MSG(dwCalibrationEngine_getCalibrationStatus(&m_status, m_calibRoutine, m_calibEngine),
                               "Error: Failed to get calibration status.");

            CHECK_DW_ERROR_MSG(dwCalibrationEngine_getSensorToRigTransformation(
                                   &m_updatedLidarToRig, m_calibRoutine, m_calibEngine),
                               "Error: Failed to get sensor to rig transformation from calibration engine.");

            // Calculate inverse of lidarToRig
            Mat4_IsoInv(m_updatedRigToLidar.array, m_updatedLidarToRig.array);

            // Calculate rotation and translation deltas between nominal and updated calibration
            Mat4_AxB(m_correctionR.array, m_updatedLidarToRig.array, m_lidarInfo.nominalRigToSensor.array);
            m_correctionR.array[3 * 4] = m_correctionR.array[3 * 4 + 1] = m_correctionR.array[3 * 4 + 2] = 0.f;
            m_correctionT.x                                                                              = m_updatedLidarToRig.array[3 * 4 + 0] - m_lidarInfo.nominalSensorToRig.array[3 * 4 + 0];
            m_correctionT.y                                                                              = m_updatedLidarToRig.array[3 * 4 + 1] - m_lidarInfo.nominalSensorToRig.array[3 * 4 + 1];
            m_correctionT.z                                                                              = m_updatedLidarToRig.array[3 * 4 + 2] - m_lidarInfo.nominalSensorToRig.array[3 * 4 + 2];
        }

        if (getWindow())
        {
            accumulateCurrentPoints();
        }
        std::swap(m_previousGPU, m_currentGPU);
        return true;
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        if (isPaused())
            return;

        if (!runLoop())
        {
            if (!m_runOnce)
            {
                std::cout << "Restarting dataset from beginning." << endl;
                CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_reset(m_accumulator), "Error: Resetting lidar accumulator failed.");
                CHECK_DW_ERROR_MSG(dwEgomotion_reset(m_egomotion), "Error: Resetting egomotion failed.");
                stopSensorManager();
                initializeSensorManager();

                m_offsetTime += m_lastEventTime - m_firstEventTime;

                m_sweepNum                = 0;
                m_lastEventTime           = 0;
                m_firstEventTime          = 0;
                m_lastEgoMotionUpdateTime = 0;
                m_lastEgoMotionAddedTime  = 0;
                if (getSweepGPU(&m_previousGPU))
                {
                    return;
                }
                std::cerr << "Error: Restarting lidar capture failed." << endl;
            }

            stop();
            return;
        }

        cout << "\n----------Sweep: " << m_sweepNum++ << "----------\n";
    }

    ///------------------------------------------------------------------------------
    /// Init render buffers for point cloud and vehicle contour
    ///------------------------------------------------------------------------------
    void prepareRenderBuffers()
    {
        // RenderBuffer
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
        layout.colFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        // Initialize render buffer for point cloud
        CHECK_DW_ERROR_MSG(dwRenderBuffer_initialize(&m_pointCloud, layout, DW_RENDER_PRIM_POINTLIST, MAX_POINTS_RENDER, m_viz),
                           "Error: Initializing render buffer for point cloud failed.");

        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        // Initialize rende buffer for vehicle contour
        CHECK_DW_ERROR_MSG(dwRenderBuffer_initialize(&m_contour, layout, DW_RENDER_PRIM_LINELOOP, 5, m_viz),
                           "Error: Initializing render buffer for vehicle contour failed.");
    }

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

    ///------------------------------------------------------------------------------
    /// Render Lidar Point Cloud
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        glDepthFunc(GL_LESS);

        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // compute modelview by moving camera to the center of the current estimated position
        {
            float32_t center[3] = {0.f, 0.f, 0.f};

            if (!isPaused())
            {
                getMouseView().setCenter(center[0] + m_centerViewDiff.x, center[1] + m_centerViewDiff.y, center[2] + m_centerViewDiff.z);
            }
            else
            {
                // store current difference to center, to keep it when not paused
                m_centerViewDiff.x = getMouseView().getCenter()[0] - center[0];
                m_centerViewDiff.y = getMouseView().getCenter()[1] - center[1];
                m_centerViewDiff.z = getMouseView().getCenter()[2] - center[2];
            }
        }

        // 3D rendering
        CHECK_DW_ERROR_MSG(dwRenderer_setModelView(getMouseView().getModelView(), m_renderer),
                           "Error: Setting renderer model view failed.");
        CHECK_DW_ERROR_MSG(dwRenderer_setProjection(getMouseView().getProjection(), m_renderer),
                           "Error: Setting renderer projection failed.");

        // Point cloud
        CHECK_DW_ERROR_MSG(dwRenderer_renderBuffer(m_pointCloud, m_renderer),
                           "Error: Rendering render buffer failed.");

        // Vehicle contour
        CHECK_DW_ERROR_MSG(dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer),
                           "Error: Setting renderer color failed.");

        CHECK_DW_ERROR_MSG(dwRenderer_setLineWidth(3, m_renderer),
                           "Error: Setting renderer line width failed.");
        CHECK_DW_ERROR_MSG(dwRenderer_renderBuffer(m_contour, m_renderer),
                           "Error: Rendering render buffer failed.");

        // Now render our matrices and text to the screen
        CHECK_DW_ERROR_MSG(dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer),
                           "Error: Setting renderer color failed.");

        std::stringstream ss;
        ss << "Rendering: Previous sweep (Red) + Aligned current sweep (Green)\n\n";
        ss << "Vehicle contour on ground plane (White) indicates quality of calibration.\n\n";
        ss << convertStatusToText() << "\n\n";
        ss << "Lidar position correction\n\n";
        ss << renderFloatsText(&m_correctionT.x, 3) << "\n\n";
        ss << "Lidar rotation correction\n\n";
        ss << renderRotationAsText(m_correctionR) << "\n";
        float32_t currentHeight = getWindowHeight() - 2 * BORDEROFFSET;
        CHECK_DW_ERROR_MSG(dwRenderer_renderText(static_cast<int32_t>(BORDEROFFSET),
                                                 static_cast<int32_t>(currentHeight),
                                                 ss.str().c_str(), m_renderer),
                           "Error: Rendering text failed");

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }
};

//------------------------------------------------------------------------------
int32_t main(int32_t argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    const std::string defRigFile = (dw_samples::SamplesDataPath::get() + "/samples/lidar/rig_perturbed.json");
    typedef ProgramArguments::Option_t opt;

    ProgramArguments args(argc, argv,
                          {opt("rig", defRigFile.c_str(), "Path to input rig calibration file."),
                           opt("lidar-sensor", "0", "The index or name of the lidar sensor from the rig file to calibrate"),
                           opt("can-sensor", "0", "The index or name of the CAN bus sensor from the rig-file to use"),
                           opt("imu-sensor", "0", "The index or name of the IMU sensor in the rig-file to use"),
                           opt("output-rig", "", "Path where updated rig calibration file should be written."),
                           opt("verbose", "0", "verbose = 1 will print detailed estimation results."),
                           opt("run-once", "0", "Controls the app should run once through a dataset (rathe than looping)."),
                           opt("use-ego-pose", "1", "Whether or not ego-motion pose is fed to lidar calibration (in addition to ICP pose).")});

    // -------------------
    // initialize and start a window application
    LidarSelfCalibrationSample app(args);

    if (args.get("offscreen") != "2")
    {
        app.initializeWindow("Lidar Self Calibration Sample", 1600, 1200, args.get("offscreen") == "1");
    }

    return app.run();
}
