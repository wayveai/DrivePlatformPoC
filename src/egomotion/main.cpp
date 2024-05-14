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
// SPDX-FileCopyrightText: Copyright (c) 2017-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// Core
#include <dw/core/context/Context.h>
#include <dw/core/logger/Logger.h>
#include <dw/core/base/Version.h>

// VehicleIO
#include <dw/control/vehicleio/VehicleIO.h>

// Egomotion
#include <dw/egomotion/base/Egomotion.h>
#include <dw/egomotion/global/GlobalEgomotion.h>

// Rig
#include <dw/rig/Rig.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/imu/IMU.h>
#include <dw/sensors/gps/GPS.h>

// Renderer
#include <dwvisualization/core/Visualization.h>

//Sensor Manager
#include <dw/sensors/sensormanager/SensorManager.h>

// Sample Includes
#include <framework/DriveWorksSample.hpp>
#include <framework/Log.hpp>
#include <framework/SamplesDataPath.hpp>
#include <framework/MathUtils.hpp>
#include <framework/Mat4.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/WindowGLFW.hpp>

#include <condition_variable>
#include <thread>
#include <list>
#include <mutex>
#include <string>
#include <vector>

#include <stdio.h>
#include <string.h>

#include "TrajectoryLogger.hpp"

using namespace dw_samples::common;

///------------------------------------------------------------------------------
/// Egomotion sample
/// The sample demonstrates how to use dwEgomotion module to compute vehicle trajectory
/// It renders the trajectory in 3D together with accompanying video for visual inspection.
///------------------------------------------------------------------------------
class EgomotionSample : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t m_context                 = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine       = DW_NULL_HANDLE;
    dwEgomotionHandle_t m_egomotion             = DW_NULL_HANDLE;
    dwGlobalEgomotionHandle_t m_globalEgomotion = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                         = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO             = DW_NULL_HANDLE;
    dwSensorManagerHandle_t m_sensorManager     = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfig                   = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_vizCtx     = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Renderer related
    // ------------------------------------------------
    dwRendererHandle_t m_renderer = DW_NULL_HANDLE;
    bool m_shallRender            = false;
    uint32_t m_tileGrid           = 0;
    uint32_t m_tileVideo          = 1;
    uint32_t m_tileRollPlot       = 2;
    uint32_t m_tilePitchPlot      = 3;
    uint32_t m_tileAltitudePlot   = 4;

    enum RenderingMode
    {
        STICK_TO_VEHICLE,          // render such that vehicle is fixed, world rotates
        ON_VEHICLE_STICK_TO_WORLD, // render such that world is fixed, vehicle rotates
    } m_renderingMode = ON_VEHICLE_STICK_TO_WORLD;

    // ------------------------------------------------
    // Camera and video visualization
    // ------------------------------------------------
    dwImageHandle_t m_convertedImageRGBA = DW_NULL_HANDLE;
    std::unique_ptr<SimpleImageStreamerGL<>> m_streamerInput2GL;
    dwImageGL* m_currentGlFrame = nullptr;

    // ------------------------------------------------
    // current sensor states
    // ------------------------------------------------
    dwEgomotionParameters m_egomotionParameters{};
    dwGlobalEgomotionParameters m_globalEgomotionParameters{};

    dwIMUFrame m_currentIMUFrame = {};
    dwGPSFrame m_currentGPSFrame = {};
    uint32_t m_imuSensorIdx      = 0;
    uint32_t m_vehicleSensorIdx  = 0;
    uint32_t m_gpsSensorIdx      = 0;

    const dwSensorEvent* acquiredEvent = nullptr;

    // ------------------------------------------------
    // sample variables
    // ------------------------------------------------
    const dwTime_t POSE_SAMPLE_PERIOD = 100000; // sample frequency of the poses in the sample [usecs]
    const size_t MAX_BUFFER_POINTS    = 100000; // maximal number of points to keep in the motion history

    struct Pose
    {
        dwTime_t timestamp                 = 0;
        dwTransformation3f rig2world       = {};
        dwEgomotionUncertainty uncertainty = {};
        float32_t rpy[3]                   = {};
    };

    std::vector<Pose> m_poseHistory;

    dwQuaternionf m_orientationENU = DW_IDENTITY_QUATERNIONF; // enu orientation
    bool m_hasOrientationENU       = false;

    FILE* m_outputFile = nullptr;

    dwTime_t m_elapsedTime         = 0;
    dwTime_t m_lastSampleTimestamp = 0;
    dwTime_t m_firstTimestamp      = 0;

    // Trajectory logger
    TrajectoryLogger m_trajectoryLog;

public:
    /// -----------------------------
    /// Initialize application
    /// -----------------------------
    EgomotionSample(const ProgramArguments& args)
        : DriveWorksSample(args)
    {
        // output file which contains positions
        if (getArgument("output").length() > 0)
        {
            m_outputFile = fopen(getArgument("output").c_str(), "wt");
            log("Open file for write: %s\n", getArgument("output").c_str());
        }

        // Default to a zoomed-out view
        getMouseView().setRadiusFromCenter(25.0f);
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

    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers, Egomotion
    /// -----------------------------
    bool onInitialize() override
    {
        dwSensorManagerParams smParams{};
        dwSensorType vehicleSensorType{};

        // -----------------------------------------
        // Initialize DriveWorks SDK context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);

            CHECK_DW_ERROR(dwVisualizationInitialize(&m_vizCtx, m_context));
        }

        // -----------------------------------------
        // read Rig file to extract vehicle properties
        // -----------------------------------------
        {
            dwStatus ret = dwRig_initializeFromFile(&m_rigConfig, m_context, getArgument("rig").c_str());
            if (ret != DW_SUCCESS)
                throw std::runtime_error("Error reading rig config");

            std::string imuSensorName     = getArgument("imu-sensor-name");
            std::string vehicleSensorName = getArgument("vehicle-sensor-name");
            std::string gpsSensorName     = getArgument("gps-sensor-name");
            std::string cameraSensorName  = getArgument("camera-sensor-name");

            // extract sensor names from rig config
            {
                uint32_t cnt;
                dwRig_getSensorCount(&cnt, m_rigConfig);
                for (uint32_t i = 0; i < cnt; i++)
                {
                    dwSensorType type;
                    const char* name;
                    dwRig_getSensorType(&type, i, m_rigConfig);
                    dwRig_getSensorName(&name, i, m_rigConfig);

                    if (type == DW_SENSOR_IMU && imuSensorName.length() == 0)
                    {
                        imuSensorName = name;
                    }
                    if ((type == DW_SENSOR_CAN || type == DW_SENSOR_DATA) && vehicleSensorName.length() == 0)
                    {
                        vehicleSensorName = name;
                    }
                    if (type == DW_SENSOR_GPS && gpsSensorName.length() == 0)
                    {
                        gpsSensorName = name;
                    }
                    if (type == DW_SENSOR_CAMERA && cameraSensorName.length() == 0)
                    {
                        cameraSensorName = name;
                    }
                }
            }

            // get sensor Idx of the sensors we use
            CHECK_DW_ERROR_MSG(dwRig_findSensorByName(&m_imuSensorIdx, imuSensorName.c_str(), m_rigConfig), "Cannot find given IMU in the rig");
            CHECK_DW_ERROR_MSG(dwRig_findSensorByName(&m_vehicleSensorIdx, vehicleSensorName.c_str(), m_rigConfig), "Cannot find given vehicle sensor in the rig");

            smParams.enableSensors[smParams.numEnableSensors++] = m_vehicleSensorIdx;
            smParams.enableSensors[smParams.numEnableSensors++] = m_imuSensorIdx;

            const char* canSensorName = nullptr;

            CHECK_DW_ERROR_MSG(dwRig_getSensorType(&vehicleSensorType, m_vehicleSensorIdx, m_rigConfig), "Cannot find vehicle sensor type");

            if (vehicleSensorType == DW_SENSOR_CAN)
            {
                canSensorName = vehicleSensorName.c_str();
            }

            // read all settings:
            //  vehicle - properties of the steering system
            //  IMU - gyroscope bias if present
            //  CAN - velocity latency and factor
            dwEgomotion_initParamsFromRig(&m_egomotionParameters, m_rigConfig, imuSensorName.c_str(), canSensorName);

            if (dwRig_findSensorByName(&m_gpsSensorIdx, gpsSensorName.c_str(), m_rigConfig) == DW_SUCCESS)
            {
                //  GPS - gps antenna position
                dwGlobalEgomotion_initParamsFromRig(&m_globalEgomotionParameters, m_rigConfig, gpsSensorName.c_str());

                smParams.enableSensors[smParams.numEnableSensors++] = m_gpsSensorIdx;
            }
            else
            {
                logWarn("Could not find GPS sensor in rig file. Global egomotion estimates will not be available.\n");
            }

            uint32_t cameraSensorId = 0;
            if (dwRig_findSensorByName(&cameraSensorId, cameraSensorName.c_str(), m_rigConfig) == DW_SUCCESS)
            {
                smParams.enableSensors[smParams.numEnableSensors++] = cameraSensorId;
            }
            else
            {
                logWarn("Could not find camera sensor in rig file. No camera image will be shown.\n");
            }
        }

        //------------------------------------------------------------------------------
        // initialize Egomotion module
        //------------------------------------------------------------------------------
        {
            if (getArgument("mode") == "0")
                m_egomotionParameters.motionModel = DW_EGOMOTION_ODOMETRY;
            else if (getArgument("mode") == "1")
            {
                m_egomotionParameters.motionModel                = DW_EGOMOTION_IMU_ODOMETRY;
                m_egomotionParameters.estimateInitialOrientation = true;
            }
            else
            {
                logError("invalid mode %s\n", getArgument("mode").c_str());
                return false;
            }
            m_egomotionParameters.automaticUpdate      = true;
            auto speedType                             = std::stoi(getArgument("speed-measurement-type"));
            m_egomotionParameters.speedMeasurementType = dwEgomotionSpeedMeasurementType(speedType);

            if (getArgument("enable-suspension") == "1")
            {
                if (m_egomotionParameters.motionModel == DW_EGOMOTION_IMU_ODOMETRY)
                {
                    m_egomotionParameters.suspension.model = DW_EGOMOTION_SUSPENSION_TORSIONAL_SPRING_MODEL;
                }
                else
                {
                    logError("Error, suspension model requires Odometry+IMU (--mode=1)");
                    return false;
                }
            }

            dwStatus status = dwEgomotion_initialize(&m_egomotion, &m_egomotionParameters, m_context);
            if (status != DW_SUCCESS)
            {
                logError("Error dwEgomotion_initialize: %s\n", dwGetStatusName(status));
                return false;
            }
        }
        //------------------------------------------------------------------------------
        // initialize Global Egomotion module
        //------------------------------------------------------------------------------
        {
            dwStatus status = dwGlobalEgomotion_initialize(&m_globalEgomotion, &m_globalEgomotionParameters, m_context);
            if (status != DW_SUCCESS)
            {
                logError("Error dwGlobalEgomotion_initialize: %s\n", dwGetStatusName(status));
                return false;
            }
        }

        //------------------------------------------------------------------------------
        // initialize Sensors
        //------------------------------------------------------------------------------
        {
            CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

            // initialize Driveworks SensorManager from rig file
            dwStatus status = dwSensorManager_initializeFromRigWithParams(&m_sensorManager, m_rigConfig, &smParams, 16, m_sal);
            if (status != DW_SUCCESS)
            {
                logError("Error in initializing SensorManager: %s\n", dwGetStatusName(status));
                return false;
            }

            // Initialize VehicleIO
            {
                // Get vehicle sensor handle
                dwSensorHandle_t vehicleSensorHandle = DW_NULL_HANDLE;
                uint32_t sensorIndex                 = uint32_t(-1);

                CHECK_DW_ERROR(dwSensorManager_getSensorIndex(&sensorIndex, vehicleSensorType, 0, m_sensorManager));
                CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&vehicleSensorHandle, sensorIndex, m_sensorManager));
                CHECK_DW_ERROR(dwVehicleIO_initializeFromRig(&m_vehicleIO, m_rigConfig, m_context));
            }

            // create camera sensor
            uint32_t cnt;
            dwSensorManager_getNumSensors(&cnt, DW_SENSOR_CAMERA, m_sensorManager);

            if (cnt == 1)
            {
                uint32_t cameraSensorIndex{};
                CHECK_DW_ERROR(dwSensorManager_getSensorIndex(&cameraSensorIndex, DW_SENSOR_CAMERA, 0, m_sensorManager));
                dwSensorHandle_t cameraSensor = DW_NULL_HANDLE;
                CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&cameraSensor, cameraSensorIndex, m_sensorManager));

                dwCameraProperties cameraProperties{};
                dwImageProperties outputProperties{};

                CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&cameraProperties, cameraSensor));
                CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&outputProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraSensor));

#ifdef VIBRANTE
                outputProperties.type = DW_IMAGE_NVMEDIA;
#else
                outputProperties.type = DW_IMAGE_CUDA;
#endif
                outputProperties.format = DW_IMAGE_FORMAT_RGB_FLOAT16_PLANAR;

                std::cout << "Camera image with " << cameraProperties.resolution.x << "x"
                          << cameraProperties.resolution.y << " at " << cameraProperties.framerate << " FPS" << std::endl;

                dwImageProperties displayProperties = outputProperties;
                displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;
                CHECK_DW_ERROR(dwImage_create(&m_convertedImageRGBA, displayProperties, m_context));

                m_streamerInput2GL.reset(new SimpleImageStreamerGL<>(displayProperties, 1000, m_context));
            }

            // start sensor manager
            if (dwSensorManager_start(m_sensorManager) != DW_SUCCESS)
            {
                logError("SensorManager start failed\n");
                dwSensorManager_release(m_sensorManager);
                return false;
            }
        }

        //------------------------------------------------------------------------------
        // initializes rendering subpart
        // - the rendering module
        // - the render buffers
        // - projection and modelview matrices
        // - renderer settings
        // -----------------------------------------
        {
            CHECK_DW_ERROR(dwRenderer_initialize(&m_renderer, m_vizCtx));

            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, m_renderer);

            // Render engine: setup default viewport and maximal size of the internal buffer
            dwRenderEngineParams params{};
            params.bufferSize = sizeof(Pose) * MAX_BUFFER_POINTS;
            params.bounds     = {0, 0, static_cast<float32_t>(getWindowWidth()), static_cast<float32_t>(getWindowHeight())};

            // 1st tile (default) - 3d world rendering
            {
                dwRenderEngine_initTileState(&params.defaultTile);
                params.defaultTile.layout.viewport = params.bounds;
            }
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_vizCtx));

            dwRenderEngineTileState tileParams = params.defaultTile;
            tileParams.projectionMatrix        = DW_IDENTITY_MATRIX4F;
            tileParams.modelViewMatrix         = DW_IDENTITY_MATRIX4F;
            tileParams.layout.positionLayout   = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
            tileParams.layout.sizeLayout       = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;

            // 2nd tile - video input
            {
                tileParams.layout.viewport     = {0.f, 0.f, getWindowWidth() / 5.0f, getWindowHeight() / 5.0f};
                tileParams.layout.positionType = DW_RENDER_ENGINE_TILE_POSITION_TYPE_TOP_LEFT;

                dwRenderEngine_addTile(&m_tileVideo, &tileParams, m_renderEngine);
            }

            // plot tiles
            {
                const float32_t plotWidth  = getWindowWidth() / 4.0f;
                const float32_t plotHeight = getWindowHeight() / 4.0f;

                tileParams.layout.viewport     = {0.f, 2 * plotHeight, plotWidth, plotHeight};
                tileParams.layout.positionType = DW_RENDER_ENGINE_TILE_POSITION_TYPE_BOTTOM_RIGHT;

                dwRenderEngine_addTile(&m_tileRollPlot, &tileParams, m_renderEngine);

                tileParams.layout.viewport = {0.f, plotHeight, plotWidth, plotHeight};
                dwRenderEngine_addTile(&m_tilePitchPlot, &tileParams, m_renderEngine);

                tileParams.layout.viewport = {0.f, 0.f, plotWidth, plotHeight};
                dwRenderEngine_addTile(&m_tileAltitudePlot, &tileParams, m_renderEngine);
            }
        }

        //------------------------------------------------------------------------------
        // initialize trajectory logger
        //------------------------------------------------------------------------------
        {
            m_trajectoryLog.addTrajectory("GPS", TrajectoryLogger::Color::GREEN);
            m_trajectoryLog.addTrajectory("Egomotion", TrajectoryLogger::Color::RED);
        }
        return true;
    }

    ///------------------------------------------------------------------------------
    /// Free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if (acquiredEvent)
            dwSensorManager_releaseAcquiredEvent(acquiredEvent, m_sensorManager);

        // save path log to KML if desired
        if (getArgument("outputkml").length())
            m_trajectoryLog.writeKML(getArgument("outputkml"));

        // close open files
        if (m_outputFile)
            fclose(m_outputFile);

        if (m_convertedImageRGBA)
            dwImage_destroy(m_convertedImageRGBA);

        if (m_vehicleIO)
            dwVehicleIO_release(m_vehicleIO);

        dwSensorManager_stop(m_sensorManager);
        dwSensorManager_release(m_sensorManager);

        m_streamerInput2GL.reset();

        dwGlobalEgomotion_release(m_globalEgomotion);
        dwEgomotion_release(m_egomotion);
        dwRig_release(m_rigConfig);

        if (m_renderer)
        {
            dwRenderer_release(m_renderer);
        }

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        dwSAL_release(m_sal);

        CHECK_DW_ERROR(dwVisualizationRelease(m_vizCtx));
        CHECK_DW_ERROR(dwRelease(m_context));

        CHECK_DW_ERROR(dwLogger_release());
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        {
            dwRect rect;
            rect.width  = width;
            rect.height = height;
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, m_renderer);
        }

        {
            dwRenderEngine_reset(m_renderEngine);
            dwRectf rect;
            rect.width  = width;
            rect.height = height;
            rect.x      = 0;
            rect.y      = 0;
            dwRenderEngine_setBounds(rect, m_renderEngine);
        }

        log("window resized to %dx%d\n", width, height);
    }

    ///------------------------------------------------------------------------------
    /// Render 3D grid showing the world
    ///     - render local grid around the car
    ///     - render car local coordinate system
    ///------------------------------------------------------------------------------
    void render3DGrid()
    {
        if (m_poseHistory.empty())
            return;

        // activate default tile
        dwRenderEngine_setTile(m_tileGrid, m_renderEngine);

        const Pose& currentPose = m_poseHistory.back();
        auto currentRig2World   = currentPose.rig2world;

        // compute modelview by moving camera to the center of the current estimated position
        dwMatrix4f modelView;
        {
            if (m_renderingMode == RenderingMode::STICK_TO_VEHICLE)
            {
                dwTransformation3f world2rig;
                Mat4_IsoInv(world2rig.array, currentRig2World.array);
                Mat4_AxB(modelView.array, getMouseView().getModelView()->array, world2rig.array);
            }
            else if (m_renderingMode == RenderingMode::ON_VEHICLE_STICK_TO_WORLD)
            {
                float32_t center[3] = {currentRig2World.array[0 + 3 * 4],
                                       currentRig2World.array[1 + 3 * 4],
                                       currentRig2World.array[2 + 3 * 4]};

                getMouseView().setCenter(center[0], center[1], center[2]);
                Mat4_AxB(modelView.array, getMouseView().getModelView()->array, DW_IDENTITY_TRANSFORMATION3F.array);
            }
        }

        // 3D rendering
        dwRenderEngine_setModelView(&modelView, m_renderEngine);
        dwRenderEngine_setProjection(getMouseView().getProjection(), m_renderEngine);

        dwRenderEngine_setBackgroundColor({0.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);

        // render path
        dwRenderEngine_setColor({0.0f, 1.0f, 0.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setLineWidth(2.f, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_3D,
                              m_poseHistory.data(),
                              sizeof(Pose),
                              offsetof(Pose, rig2world) + 3 * 4 * sizeof(float32_t), // grabbing last column of rig2world
                              m_poseHistory.size(),
                              m_renderEngine);

        // render current orientation
        {
            dwRenderEngine_setLineWidth(2.0f, m_renderEngine);

            for (int i = 0; i < 3; i++)
            {
                float32_t localAxis[3] = {
                    i == 0 ? 1.f : 0.f,
                    i == 1 ? 1.f : 0.f,
                    i == 2 ? 1.f : 0.f};
                dwVector3f arrow[2];

                // origin
                arrow[0].x = currentRig2World.array[0 + 3 * 4];
                arrow[0].y = currentRig2World.array[1 + 3 * 4];
                arrow[0].z = currentRig2World.array[2 + 3 * 4];

                // axis
                Mat4_Axp(reinterpret_cast<float32_t*>(&arrow[1]), currentRig2World.array, localAxis);

                dwRenderEngine_setColor({localAxis[0], localAxis[1], localAxis[2], 1.0f}, m_renderEngine);
                dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ARROWS_3D,
                                      arrow,
                                      sizeof(dwVector3f) * 2,
                                      0,
                                      1,
                                      m_renderEngine);
            }
        }

        // if rotation in ENU frame is available, draw arrow towards north
        if (m_hasOrientationENU)
        {
            // local ("navigation") frame to ENU frame
            dwTransformation3f rig2Enu = rigidTransformation(m_orientationENU, {0.f, 0.f, 0.f});

            dwVector3f arrow[2];
            {
                dwVector3f arrowENU[2];
                dwVector3f arrowRig[2];

                arrowENU[0] = {0, 0, 0}; // start from origin
                arrowENU[1] = {0, 2, 0}; // pointing to north in ENU coordinate frame

                // map to rig coordinate frame
                Mat4_Rtxp(reinterpret_cast<float32_t*>(&arrowRig[0]), rig2Enu.array, reinterpret_cast<float32_t*>(&arrowENU[0]));
                Mat4_Rtxp(reinterpret_cast<float32_t*>(&arrowRig[1]), rig2Enu.array, reinterpret_cast<float32_t*>(&arrowENU[1]));

                // map to world space
                Mat4_Axp(reinterpret_cast<float32_t*>(&arrow[0]), currentRig2World.array, reinterpret_cast<float32_t*>(&arrowRig[0]));
                Mat4_Axp(reinterpret_cast<float32_t*>(&arrow[1]), currentRig2World.array, reinterpret_cast<float32_t*>(&arrowRig[1]));
            }

            const char* labels[] = {"GPS NORTH"};

            dwRenderEngine_setLineWidth(2.0f, m_renderEngine);
            dwRenderEngine_setColor({0.8f, 0.3f, 0.05f, 1.0f}, m_renderEngine);
            dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ARROWS_3D,
                                            arrow,
                                            sizeof(dwVector3f) * 2,
                                            0,
                                            labels,
                                            1,
                                            m_renderEngine);
        }

        {
            // render world grid - 10m squares
            dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 0.3f}, m_renderEngine);
            dwRenderEngine_setLineWidth(0.5f, m_renderEngine);
            dwRenderEngine_renderPlanarGrid3D({0, 0, 1100.0f, 1100.0f}, 10.f, 10.f, &DW_IDENTITY_MATRIX4F, m_renderEngine);

            // render world grid - 1m squares
            dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 0.1f}, m_renderEngine);
            dwRenderEngine_setLineWidth(0.25f, m_renderEngine);
            dwRenderEngine_renderPlanarGrid3D({0, 0, 1100.0f, 1100.0f}, 1.f, 1.f, &DW_IDENTITY_MATRIX4F, m_renderEngine);

            // render local grid - approximate dimensions of car
            dwMatrix4f modelView = dwMakeMatrix4f(currentRig2World);
            dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_setLineWidth(2.f, m_renderEngine);
            dwRenderEngine_renderPlanarGrid3D({0, 0, 2.1f, 1.0f}, 2.1f, 1.0f, &modelView, m_renderEngine);
        }
    }

    ///------------------------------------------------------------------------------
    /// Render information in text format
    ///------------------------------------------------------------------------------
    void renderText()
    {
        const dwVector2i origin = {10, 500};
        char sbuffer[128];

        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_20, m_renderer);

        // sample usage information
        {
            dwRenderer_renderText(origin.x, origin.y, "EGOMOTION MODULE SAMPLE", m_renderer);
            dwRenderer_renderText(origin.x, origin.y - 30, "F1 - camera on rig", m_renderer);
            dwRenderer_renderText(origin.x, origin.y - 50, "F2 - camera following rig in world", m_renderer);
            dwRenderer_renderText(origin.x, origin.y - 70, "SPACE - pause", m_renderer);
            dwRenderer_renderText(origin.x, origin.y - 85, "__________________________", m_renderer);
        }

        // Render currently selected mode
        {
            dwMotionModel motionModel;
            dwEgomotion_getMotionModel(&motionModel, m_egomotion);
            if (motionModel == DW_EGOMOTION_ODOMETRY)
                dwRenderer_renderText(origin.x, origin.y - 120, "Motion model: ODOMETRY", m_renderer);
            else if (motionModel == DW_EGOMOTION_IMU_ODOMETRY)
                dwRenderer_renderText(origin.x, origin.y - 120, "Motion model: ODOMETRY+IMU", m_renderer);
        }

        // Render currently selected measurement type
        {
            if (m_egomotionParameters.speedMeasurementType == DW_EGOMOTION_FRONT_SPEED)
                dwRenderer_renderText(origin.x, origin.y - 140, "Speed measurement: front linear speed along steering direction", m_renderer);
            else if (m_egomotionParameters.speedMeasurementType == DW_EGOMOTION_REAR_SPEED)
                dwRenderer_renderText(origin.x, origin.y - 140, "Speed measurement: rear linear speed along vehicle forward axis", m_renderer);
            else if (m_egomotionParameters.speedMeasurementType == DW_EGOMOTION_REAR_WHEEL_SPEED)
                dwRenderer_renderText(origin.x, origin.y - 140, "Speed measurement: rear wheel angular speed", m_renderer);
        }

        // Indicate if suspension model is active
        {
            if (m_egomotionParameters.suspension.model == DW_EGOMOTION_SUSPENSION_TORSIONAL_SPRING_MODEL)
                dwRenderer_renderText(origin.x, origin.y - 160, "Suspension modeling: enabled", m_renderer);
            else
                dwRenderer_renderText(origin.x, origin.y - 160, "Suspension modeling: disabled", m_renderer);
        }

        // render egomotion estimation information
        {
            dwEgomotionResult state{};
            dwEgomotion_getEstimation(&state, m_egomotion);

            dwEgomotionUncertainty uncertainty{};
            dwEgomotion_getUncertainty(&uncertainty, m_egomotion);

            // elapsed time
            sprintf(sbuffer, "Time: %.1f s", m_elapsedTime * 1e-6);
            dwRenderer_renderText(origin.x, origin.y - 195, sbuffer, m_renderer);

            static constexpr uint32_t VEL = DW_EGOMOTION_LIN_VEL_X;
            if ((state.validFlags & VEL) == VEL)
            {
                static float32_t oldSpeed    = std::numeric_limits<float32_t>::max();
                static float32_t olddVdt     = 0;
                static dwTime_t oldTimestamp = 0;

                float32_t speed = sqrt(state.linearVelocity[0] * state.linearVelocity[0] + state.linearVelocity[1] * state.linearVelocity[1]);
                float32_t dVdt  = 0;

                if (state.timestamp != oldTimestamp) // in case if we pause rendering, will reuse olddVdt
                {
                    dVdt         = (speed - oldSpeed) / (static_cast<float32_t>(state.timestamp - oldTimestamp) / 1000000.f);
                    oldTimestamp = state.timestamp;
                    oldSpeed     = speed;
                    olddVdt      = dVdt;
                }

                sprintf(sbuffer, "Speed: %.2f m/s (%.2f km/h), rate: %.2f m/s^2", speed, speed * 3.6, olddVdt);
                dwRenderer_renderText(origin.x, origin.y - 220, sbuffer, m_renderer);

                const auto printLinear = [this, &sbuffer](int32_t x, int32_t y, const char* name,
                                                          float32_t value, float32_t rate,
                                                          bool printLinear, bool printRate) {
                    int32_t len = 0;
                    len += sprintf(sbuffer, "%s: ", name);

                    if (printLinear)
                        len += sprintf(sbuffer + len, "%.2f m/s, ", value);

                    if (printRate)
                        len += sprintf(sbuffer + len, "rate: %.2f m/s^2", rate);

                    dwRenderer_renderText(x, y, sbuffer, m_renderer);
                };

                // V_x and Acc_x
                printLinear(origin.x, origin.y - 260, "V_x",
                            state.linearVelocity[0], state.linearAcceleration[0],
                            (state.validFlags & DW_EGOMOTION_LIN_VEL_X) != 0,
                            (state.validFlags & DW_EGOMOTION_LIN_ACC_X) != 0);
                // V_y and Acc_y
                printLinear(origin.x, origin.y - 280, "V_y",
                            state.linearVelocity[1], state.linearAcceleration[1],
                            (state.validFlags & DW_EGOMOTION_LIN_VEL_Y) != 0,
                            (state.validFlags & DW_EGOMOTION_LIN_ACC_Y) != 0);

                if (state.validFlags & DW_EGOMOTION_LIN_VEL_Z)
                {
                    // V_z and Acc_z
                    printLinear(origin.x, origin.y - 300, "V_z",
                                state.linearVelocity[2], state.linearAcceleration[2],
                                (state.validFlags & DW_EGOMOTION_LIN_VEL_Z) != 0,
                                (state.validFlags & DW_EGOMOTION_LIN_ACC_Z) != 0);
                }
            }
            else
                dwRenderer_renderText(origin.x, origin.y - 220, "Speed: not supported", m_renderer);

            const auto printAngle = [this, &sbuffer](int32_t x, int32_t y, const char* name,
                                                     float32_t value, float32_t std, float32_t rate,
                                                     bool printAngle, bool printStd, bool printRate) {
                int32_t len = 0;
                len += sprintf(sbuffer, "%s: ", name);

                if (printAngle)
                {
                    if (printStd)
                        len += sprintf(sbuffer + len, "%.2f +/- %.2f deg, ", RAD2DEG(value), RAD2DEG(std));
                    else
                        len += sprintf(sbuffer + len, "%.2f deg, ", RAD2DEG(value));
                }

                if (printRate)
                    len += sprintf(sbuffer + len, "rate: %.2f deg/s", RAD2DEG(rate));

                dwRenderer_renderText(x, y, sbuffer, m_renderer);
            };

            float32_t roll, pitch, yaw;
            quaternionToEulerAngles(state.rotation, roll, pitch, yaw);

            // Roll
            printAngle(origin.x, origin.y - 320, "Roll",
                       roll, std::sqrt(uncertainty.rotation.array[0]), state.angularVelocity[0],
                       (state.validFlags & DW_EGOMOTION_ROTATION) != 0,
                       (uncertainty.validFlags & DW_EGOMOTION_ROTATION) != 0,
                       (state.validFlags & DW_EGOMOTION_ANG_VEL_X) != 0);

            // Pitch
            printAngle(origin.x, origin.y - 340, "Pitch",
                       pitch, std::sqrt(uncertainty.rotation.array[3 + 1]), state.angularVelocity[1],
                       (state.validFlags & DW_EGOMOTION_ROTATION) != 0,
                       (uncertainty.validFlags & DW_EGOMOTION_ROTATION) != 0,
                       (state.validFlags & DW_EGOMOTION_ANG_VEL_Y) != 0);

            // Yaw
            printAngle(origin.x, origin.y - 360, "Yaw",
                       yaw, 0, state.angularVelocity[2],
                       (state.validFlags & DW_EGOMOTION_ROTATION) != 0,
                       false,
                       (state.validFlags & DW_EGOMOTION_ANG_VEL_Z) != 0);

            if ((state.validFlags & DW_EGOMOTION_ROTATION) != 0)
            {
                sprintf(sbuffer, "Rotation relative to starting pose (t=0)");
            }

            dwRenderer_renderText(origin.x, origin.y - 400, sbuffer, m_renderer);

            // query global pose
            dwGlobalEgomotionResult globalResult{};
            dwGlobalEgomotion_getEstimate(&globalResult, nullptr, m_globalEgomotion);

            if (globalResult.validPosition)
            {
                sprintf(sbuffer, "Longitude: %.5f deg", globalResult.position.lon);
                dwRenderer_renderText(origin.x, origin.y - 440, sbuffer, m_renderer);

                sprintf(sbuffer, "Latitude: %.5f deg", globalResult.position.lat);
                dwRenderer_renderText(origin.x, origin.y - 460, sbuffer, m_renderer);

                sprintf(sbuffer, "Altitude: %.2f m", globalResult.position.height);
                dwRenderer_renderText(origin.x, origin.y - 480, sbuffer, m_renderer);
            }
            else
            {
                sprintf(sbuffer, "GPS: not supported or not available");
                dwRenderer_renderText(origin.x, origin.y - 440, sbuffer, m_renderer);
            }
        }
    }

    ///------------------------------------------------------------------------------
    /// Render debug information in different tiles
    ///------------------------------------------------------------------------------
    void renderPlots()
    {
        // Render plots with the history of roll, pitch and altitude values
        if (!m_poseHistory.empty())
        {
            std::vector<dwVector2f> roll, rollUncertaintyPlus, rollUncertaintyMinus;
            std::vector<dwVector2f> pitch, pitchUncertaintyPlus, pitchUncertaintyMinus;
            std::vector<dwVector2f> altitude;

            float32_t negInf = -std::numeric_limits<float32_t>::infinity();
            float32_t posInf = std::numeric_limits<float32_t>::infinity();

            dwTime_t startTime = m_poseHistory.front().timestamp;
            dwTime_t lastTime  = m_poseHistory.back().timestamp;

            for (const auto& pose : m_poseHistory)
            {
                float32_t dt = float32_t((pose.timestamp - startTime) * 1e-6);

                // keep last 30sec for altitude
                if (lastTime - pose.timestamp < 240 * 1e6)
                {
                    altitude.push_back({dt, float32_t(pose.rig2world.array[2 + 3 * 4])});
                }

                // keep only last 5 sec for roll and pitch
                if (lastTime - pose.timestamp < 5 * 1e6)
                {
                    roll.push_back({dt, RAD2DEG(pose.rpy[0])});
                    pitch.push_back({dt, RAD2DEG(pose.rpy[1])});

                    if (pose.uncertainty.validFlags & DW_EGOMOTION_ROTATION)
                    {
                        rollUncertaintyPlus.push_back({dt, RAD2DEG(pose.rpy[0]) + RAD2DEG(std::sqrt(pose.uncertainty.rotation.array[0]))});
                        rollUncertaintyMinus.push_back({dt, RAD2DEG(pose.rpy[0]) - RAD2DEG(std::sqrt(pose.uncertainty.rotation.array[0]))});
                        pitchUncertaintyPlus.push_back({dt, RAD2DEG(pose.rpy[1]) + RAD2DEG(std::sqrt(pose.uncertainty.rotation.array[3 + 1]))});
                        pitchUncertaintyMinus.push_back({dt, RAD2DEG(pose.rpy[1]) - RAD2DEG(std::sqrt(pose.uncertainty.rotation.array[3 + 1]))});
                    }
                }
            }

            // Roll plot
            {
                dwRenderEnginePlotType types[]   = {DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP, DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP, DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP};
                const void* data[]               = {roll.data(), rollUncertaintyPlus.data(), rollUncertaintyMinus.data()};
                uint32_t strides[]               = {sizeof(dwVector2f), sizeof(dwVector2f), sizeof(dwVector2f)};
                uint32_t offsets[]               = {0, 0, 0};
                uint32_t counts[]                = {uint32_t(roll.size()), uint32_t(rollUncertaintyPlus.size()), uint32_t(rollUncertaintyMinus.size())};
                dwRenderEngineColorRGBA colors[] = {{1.0f, 0.0f, 0.0f, 1.0f},
                                                    {1.0f, 0.0f, 0.0f, 0.5f},
                                                    {1.0f, 0.0f, 0.0f, 0.5f}};
                float32_t widths[]   = {2.0f, 1.0f, 1.0f};
                const char* labels[] = {"roll", "", ""};

                dwRenderEngine_setTile(m_tileRollPlot, m_renderEngine);
                dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
                dwRenderEngine_renderPlots2D(types,
                                             data, strides, offsets, counts,
                                             colors, widths, labels,
                                             counts[1] > 0 ? 3 : 1,
                                             {negInf, -10.f, posInf, 10.f},
                                             {0.0f, 0.0f, 1.0f, 1.0f},
                                             {0.5f, 0.4f, 0.2f, 1.0f},
                                             1.f,
                                             "", " time", "[deg]",
                                             m_renderEngine);
            }

            // Pitch plot
            {
                dwRenderEnginePlotType types[]   = {DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP, DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP, DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP};
                const void* data[]               = {pitch.data(), pitchUncertaintyPlus.data(), pitchUncertaintyMinus.data()};
                uint32_t strides[]               = {sizeof(dwVector2f), sizeof(dwVector2f), sizeof(dwVector2f)};
                uint32_t offsets[]               = {0, 0, 0};
                uint32_t counts[]                = {uint32_t(pitch.size()), uint32_t(pitchUncertaintyPlus.size()), uint32_t(pitchUncertaintyMinus.size())};
                dwRenderEngineColorRGBA colors[] = {{0.0f, 1.0f, 0.0f, 1.0f},
                                                    {0.0f, 1.0f, 0.0f, 0.5f},
                                                    {0.0f, 1.0f, 0.0f, 0.5f}};
                float32_t widths[]   = {2.0f, 1.0f, 1.0f};
                const char* labels[] = {"pitch", "", ""};

                dwRenderEngine_setTile(m_tilePitchPlot, m_renderEngine);
                dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
                dwRenderEngine_renderPlots2D(types,
                                             data, strides, offsets, counts,
                                             colors, widths, labels,
                                             counts[1] > 0 ? 3 : 1,
                                             {negInf, -10.f, posInf, 10.f},
                                             {0.0f, 0.0f, 1.0f, 1.0f},
                                             {0.5f, 0.4f, 0.2f, 1.0f},
                                             1.f,
                                             "", " time", "[deg]",
                                             m_renderEngine);
            }

            // ALTITUDE PLOT
            {
                dwRenderEngine_setTile(m_tileAltitudePlot, m_renderEngine);
                dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
                dwRenderEngine_renderPlot2D(DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP,
                                            altitude.data(),
                                            sizeof(dwVector2f),
                                            0,
                                            altitude.size(),
                                            "Altitude",
                                            {negInf, altitude.back().y - 5.f, posInf, altitude.back().y + 5.f}, // plot a vertical range around last value of +-5m
                                            {0.0f, 0.0f, 1.0f, 1.0f},
                                            {0.5f, 0.4f, 0.2f, 1.0f},
                                            1.f,
                                            "", " time", "[m]",
                                            m_renderEngine);
            }
        }

        // render current motion estimates
        dwRenderEngine_setTile(m_tileGrid, m_renderEngine);
    }

    ///------------------------------------------------------------------------------
    /// Render sample output on screen
    ///     - render video
    ///     - render path
    ///     - render current estimate information
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        if (!isPaused() && !m_shallRender)
            return;

        m_shallRender = false;

        if (isOffscreen())
            return;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 3d rendering of grid
        render3DGrid();

        // video
        if (m_currentGlFrame)
        {
            dwRenderEngine_setTile(m_tileVideo, m_renderEngine);

            dwVector2f range{};
            range.x = m_currentGlFrame->prop.width;
            range.y = m_currentGlFrame->prop.height;
            dwRenderEngine_setCoordinateRange2D(range, m_renderEngine);
            dwRenderEngine_renderImage2D(m_currentGlFrame, {0.0f, 0.0f, range.x, range.y}, m_renderEngine);
        }

        renderText();
        renderPlots();

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - collect sensor data
    ///     - push data to egomotion
    ///     - update egomotion filter in certain interval
    ///     - extract latest filter state
    ///     - integrate relative poses to an absolute one
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        // we continue grabbing sensor data until we have a frame to render or an update to egomotion pose
        while (!m_shallRender && !isPaused())
        {
            auto status = dwSensorManager_acquireNextEvent(&acquiredEvent, 0, m_sensorManager);

            if (status == DW_TIME_OUT)
                return;

            if (status != DW_SUCCESS)
            {
                if (status != DW_END_OF_STREAM)
                    logError("Error reading sensor %s\n", dwGetStatusName(status));

                pause(); // pause after end of stream
                if (isOffscreen())
                    stop();

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return;
            }

            // indicator that we had a valid update of the pose
            dwTime_t timestamp = acquiredEvent->timestamp_us;

            if (m_firstTimestamp == 0)
            {
                m_firstTimestamp      = timestamp;
                m_lastSampleTimestamp = timestamp;
            }

            // parse and push the new measurement to the ego motion module
            switch (acquiredEvent->type)
            {

            // on new vehicle messages we parse the content and push to egomotion
            case DW_SENSOR_CAN:
            case DW_SENSOR_DATA:
            {
                // ignore any message which is not from the requested sensor
                if (acquiredEvent->sensorIndex != m_vehicleSensorIdx)
                    break;

                if (acquiredEvent->type == DW_SENSOR_CAN)
                {
                    CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&acquiredEvent->canFrame, 0, m_vehicleIO));
                }
                else
                {
                    CHECK_DW_ERROR(dwVehicleIO_consumeDataPacket(acquiredEvent->dataFrame, acquiredEvent->sensorIndex, m_vehicleIO));
                }

                dwVehicleIOSafetyState vehicleIOSafetyState{};
                dwVehicleIONonSafetyState vehicleIONonSafetyState{};
                dwVehicleIOActuationFeedback vehicleIOActuationFeedback{};
                CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&vehicleIOSafetyState, m_vehicleIO));
                CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&vehicleIONonSafetyState, m_vehicleIO));
                CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&vehicleIOActuationFeedback, m_vehicleIO));
                CHECK_DW_ERROR(dwEgomotion_addVehicleIOState(&vehicleIOSafetyState, &vehicleIONonSafetyState, &vehicleIOActuationFeedback, m_egomotion));
                break;
            }

            // on new IMU measurements push to egomotion
            case DW_SENSOR_IMU:
            {
                // ignore any IMU message which is not from the requested sensor
                if (acquiredEvent->sensorIndex != m_imuSensorIdx)
                    break;

                m_currentIMUFrame = acquiredEvent->imuFrame;

                if (m_egomotionParameters.motionModel != DW_EGOMOTION_ODOMETRY) // only supported for non-odometry mode
                {
                    dwEgomotion_addIMUMeasurement(&m_currentIMUFrame, m_egomotion);
                }
                break;
            }

            // same for GPS, pass any new data to egomotion
            // we do also store original data as reference
            case DW_SENSOR_GPS:
            {
                // ignore any GPS message which is not from the requested sensor
                if (acquiredEvent->sensorIndex != m_gpsSensorIdx)
                    break;

                m_currentGPSFrame = acquiredEvent->gpsFrame;

                // log path
                m_trajectoryLog.addWGS84("GPS", m_currentGPSFrame);

                CHECK_DW_ERROR(dwGlobalEgomotion_addGPSMeasurement(&m_currentGPSFrame, m_globalEgomotion));
                break;
            }

            // on new camera frames, we map them to GL for rendering on screen
            case DW_SENSOR_CAMERA:
            {
                dwImageHandle_t nextFrame;
                dwSensorCamera_getImage(&nextFrame, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, acquiredEvent->camFrames[0]);

                dwImage_copyConvert(m_convertedImageRGBA, nextFrame, m_context);
                dwImageHandle_t frameGL = m_streamerInput2GL->post(m_convertedImageRGBA);
                dwImage_getGL(&m_currentGlFrame, frameGL);

                break;
            }

            case DW_SENSOR_COUNT:
            case DW_SENSOR_LIDAR:
            case DW_SENSOR_RADAR:
            case DW_SENSOR_TIME:
            default: break;
            }

            // tell sensor manager that we are done processing the sensor event
            dwSensorManager_releaseAcquiredEvent(acquiredEvent, m_sensorManager);
            acquiredEvent = nullptr;

            // sample the actual pose at a camera rate (render it or output to the file)
            dwEgomotionResult estimate;
            dwEgomotionUncertainty uncertainty;
            if (dwEgomotion_getEstimation(&estimate, m_egomotion) == DW_SUCCESS &&
                dwEgomotion_getUncertainty(&uncertainty, m_egomotion) == DW_SUCCESS)
            {
                // update global egomotion
                dwGlobalEgomotion_addRelativeMotion(&estimate, &uncertainty, m_globalEgomotion);

                if (estimate.timestamp >= m_lastSampleTimestamp + POSE_SAMPLE_PERIOD)
                {
                    // extract relative pose between last and the current timestamp
                    dwTransformation3f rigLast2rigNow;

                    status = dwEgomotion_computeRelativeTransformation(&rigLast2rigNow, nullptr,
                                                                       m_lastSampleTimestamp, estimate.timestamp, m_egomotion);
                    if (status == DW_SUCCESS)
                    {
                        Pose pose{};

                        // store roll, pitch, yaw
                        quaternionToEulerAngles(estimate.rotation, pose.rpy[0], pose.rpy[1], pose.rpy[2]);

                        // pose from last frame
                        dwTransformation3f rigLast2world = DW_IDENTITY_TRANSFORMATION3F;

                        if (!m_poseHistory.empty())
                            rigLast2world = m_poseHistory.back().rig2world;
                        else if ((estimate.validFlags & DW_EGOMOTION_ROTATION) != 0)
                        {
                            // set initial rig2world to initial orientation estimate
                            // vehicle might not start on horizontal ground, therefore use
                            // initial pitch and roll estimates to orient car in the world
                            dwMatrix3f rot{};
                            getRotationMatrix(&rot, RAD2DEG(pose.rpy[0]), RAD2DEG(pose.rpy[1]), 0);
                            rotationToTransformMatrix(rigLast2world.array, rot.array);
                        }

                        // compute absolute pose given the relative motion between two last estimates
                        dwTransformation3f rigNow2World;
                        dwEgomotion_applyRelativeTransformation(&rigNow2World, &rigLast2rigNow, &rigLast2world);

                        pose.rig2world = rigNow2World;
                        pose.timestamp = estimate.timestamp;

                        // pop first entry. Keep in mind that this is not the most efficient way of doing so
                        if (m_poseHistory.size() > MAX_BUFFER_POINTS)
                        {
                            decltype(m_poseHistory) tmp;
                            tmp.assign(++m_poseHistory.begin(), m_poseHistory.end());
                            std::swap(tmp, m_poseHistory);
                        }

                        if (m_outputFile)
                            fprintf(m_outputFile, "%lu,%.2f,%.2f,%.2f\n", estimate.timestamp, rigNow2World.array[0 + 3 * 4], rigNow2World.array[1 + 3 * 4], rigNow2World.array[2 + 3 * 4]);

                        // if global orientation in ENU frame is available, store it
                        dwGlobalEgomotionResult absoluteEstimate{};
                        if (dwGlobalEgomotion_getEstimate(&absoluteEstimate, nullptr, m_globalEgomotion) == DW_SUCCESS &&
                            absoluteEstimate.timestamp == estimate.timestamp && absoluteEstimate.validOrientation)
                        {
                            m_orientationENU    = absoluteEstimate.orientation;
                            m_hasOrientationENU = true;

                            // Log motion in ENU space
                            if (m_trajectoryLog.size("Egomotion") == 0) // place initial egomotion point at current GPS point
                                m_trajectoryLog.addWGS84("Egomotion", m_currentGPSFrame);

                            m_trajectoryLog.addWGS84("Egomotion", absoluteEstimate.position);
                        }
                        else
                        {
                            m_hasOrientationENU = false;
                        }

                        // query pose uncertainty
                        dwEgomotion_getUncertainty(&pose.uncertainty, m_egomotion);

                        m_poseHistory.push_back(pose);

                        m_shallRender = true;
                    }
                    m_lastSampleTimestamp = estimate.timestamp;
                }
            }

            m_elapsedTime = timestamp - m_firstTimestamp;
        }
    }

    ///------------------------------------------------------------------------------
    /// Handle key press events
    ///------------------------------------------------------------------------------
    void onKeyDown(int key, int scancode, int mods) override
    {
        (void)scancode;
        (void)mods;

        if (key == GLFW_KEY_F1)
        {
            getMouseView().setCenter(0, 0, 0);
            m_renderingMode = RenderingMode::STICK_TO_VEHICLE;
        }

        if (key == GLFW_KEY_F2)
        {
            m_renderingMode = RenderingMode::ON_VEHICLE_STICK_TO_WORLD;
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char* argv[])
{
    const std::string samplePath = dw_samples::SamplesDataPath::get() + "/samples/recordings/cloverleaf/";

    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("vehicle-sensor-name", "", "[optional] Name of the vehicle sensor in rig file to be used as data input."),
                              ProgramArguments::Option_t("imu-sensor-name", "", "[optional] Name of the IMU sensor in rig file to be used as data input."),
                              ProgramArguments::Option_t("gps-sensor-name", "", "[optional] Name of the GPS sensor in rig file to be used as data input."),
                              ProgramArguments::Option_t("camera-sensor-name", "", "[optional] Name of the camera sensor in rig file to be used as data input."),

                              ProgramArguments::Option_t("rig", (samplePath + "rig-nominal-intrinsics.json").c_str(),
                                                         "Rig file containing all information about vehicle sensors and calibration."),

                              ProgramArguments::Option_t("output", "", "If specified, the odometry will be output to this file."),
                              ProgramArguments::Option_t("outputkml", "", "If specified, gps and estimated location will be output into this file"),
                              ProgramArguments::Option_t("mode", "1", "0=Ackerman motion, 1=IMU+Odometry+GPS"),
                              ProgramArguments::Option_t("speed-measurement-type", "1", "Speed measurement type, refer to dwEgomotionSpeedMeasurementType"),
                              ProgramArguments::Option_t("enable-suspension", "0", "If 1, enables egomotion suspension modeling (requires Odometry+IMU [--mode=1]), otherwise disabled."),

                          },
                          "DriveWorks egomotion sample");

    // -------------------
    // initialize and start a window application (with offscreen support if required)
    EgomotionSample app(args);

    app.initializeWindow("Egomotion Sample", 1920, 1080, args.enabled("offscreen"));

    if (!args.enabled("offscreen"))
        app.setProcessRate(240);
    return app.run();
}
