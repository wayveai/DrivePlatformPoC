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

#include <framework/DriveWorksSample.hpp>
#include <framework/MathUtils.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/MouseView3D.hpp>

#include <signal.h>
#include <string.h>

#include <iostream>
#include <chrono>

#include <thread>
#include <memory>
#include <vector>
#include <sstream>
#include <float.h>
#include <unistd.h>

#include <dwvisualization/core/RenderEngine.h>
#include <dw/sensors/Sensors.h>

// CORE
#include <dw/core/logger/Logger.h>
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>

// SAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/radar/Radar.h>

#include <sys/stat.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Radar replay sample
// The Radar replay sample demonstrates playback of radar sensor.
//
// The sample opens a window to play back the provided virtual sensor data file.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Help functions
//------------------------------------------------------------------------------
namespace
{
template <class T>
std::string toStr(const T& value)
{
    std::ostringstream s;
    s.precision(DBL_DIG);
    s << value;
    return s.str();
}
}

class RadarReplay : public DriveWorksSample
{
private:
    struct ColoredPoint3D
    {
        dwVector3f position;
        dwVector4f color;
    };

    // -----------------------------------------------------------
    // Constants for this radar replay which configure output view
    // -----------------------------------------------------------
    const float WORLD_Z = 0.0f;

    // -----------------------------------------------------------
    // GRID construction and GRID params
    // -----------------------------------------------------------
    const float WORLD_CIRCLE_DR_IN_METERS        = 5.0f;
    const float WORLD_GRID_RES_IN_METERS         = 10.0f;
    const float WORLD_GRID_SIZE_IN_METERS_WIDTH  = 220.0f;
    const float WORLD_GRID_SIZE_IN_METERS_HEIGHT = 150.0f;

    std::string m_outputDir = "";

    // ------------------------------------------------
    // SAL
    // ------------------------------------------------
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                   = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific. Variables for hold message text
    // ------------------------------------------------
    std::string m_message1;
    std::string m_message2;
    std::string m_message3;
    std::string m_message4;
    std::string m_message5;
    std::string m_message6;
    std::string m_message7;
    std::string m_message8;
    std::string m_message9;

    // -------------------------------------------------------
    // Sample specific. Sensors
    // -------------------------------------------------------
    dwSensorHandle_t m_radarSensor      = DW_NULL_HANDLE;
    dwRadarProperties m_radarProperties = {};

    // -------------------------------------------------------
    // Sample specific. Various buffer handles for rasterizing
    // -------------------------------------------------------

    uint32_t m_detectionPointCloud[DW_RADAR_RANGE_COUNT];
    uint32_t m_trackPointCloud[DW_RADAR_RANGE_COUNT];

    size_t m_accumulatedPoints[DW_RADAR_RETURN_TYPE_COUNT][DW_RADAR_RANGE_COUNT]{};

    dwRadarDetection* m_detectionData[DW_RADAR_RANGE_COUNT];
    dwRadarTrack* m_trackData[DW_RADAR_RANGE_COUNT];

    uint32_t m_textTile = 0;

    // -------------------------------------------------------
    // Sample specific. Various flags for various display modes
    // -------------------------------------------------------
    bool m_showGrid         = true;
    bool m_showText         = true;
    bool m_recordedRadar    = false;
    bool m_renderDetections = true;
    bool m_renderTracks     = true;
    bool m_isRunning        = true;

    // -------------------------------------------------------
    // Sample specific. To accumulate one radar spin
    // -------------------------------------------------------
    uint32_t m_numScansPerSpin;

    // -------------------------------------------------------
    // Sample specific.  For 3D Display
    // -------------------------------------------------------

    double m_freqMultiplier = 1.0; ///< Frequency muliplier to increase od decrease showed points during radar
                                   ///  scans. When value 1.0 points are reading in radar scan rate.
    dwTime_t m_lastTimestamp = 0;

    MouseView3D m_mouseView;

public:
    RadarReplay(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    void initializeInputDefaults()
    {
        getMouseView().setCenter(0.0f, 0.0f, 0.0f);
        getMouseView().setRadiusFromCenter(100.0f);
    }

    void onKeyDown(int key, int /*scancode*/, int /*mods*/) override
    {

        if (key == GLFW_KEY_R)
        {
            initializeInputDefaults();
            m_freqMultiplier = 1.0;
        }

        // change view mode of displayed grid
        if (key == GLFW_KEY_G)
            m_showGrid = !m_showGrid;

        // show hide text messages
        if (key == GLFW_KEY_F1)
            m_showText = !m_showText;

        // decrease frequency of displayed data
        if (key == GLFW_KEY_KP_SUBTRACT)
            m_freqMultiplier *= 0.5;

        // increase frequency of displayed data
        if (key == GLFW_KEY_KP_ADD)
            m_freqMultiplier *= 2;

        // Toggle visualization for detections
        if (key == GLFW_KEY_0)
            m_renderDetections = !m_renderDetections;

        // Toggle visualization for tracks
        if (key == GLFW_KEY_1)
            m_renderTracks = !m_renderTracks;
    }

    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers
    /// -----------------------------
    bool onInitialize() override
    {
        preparePointCloudDumps(getArgs());

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);
            dwSAL_initialize(&m_sal, m_context);
        }
        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            initializeSensor(getArgs());
        }
        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            initializeInputDefaults();
            initializeRenderer();
        }
        // -----------------------------
        // Start Sensors
        // -----------------------------
        //dwSensor_start(camera);
        CHECK_DW_ERROR(dwSensor_start(m_radarSensor));

        return true;
    }

    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        dwSensor_reset(m_radarSensor);
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        {
            m_isRunning = false;

            dwSensor_stop(m_radarSensor);
            dwSAL_releaseSensor(m_radarSensor);

            // release used objects in correct order
            for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; j++)
            {
                if (m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_DETECTION][j])
                {
                    dwRenderEngine_destroyBuffer(m_detectionPointCloud[j], m_renderEngine);
                    delete[] m_detectionData[j];
                }
                if (m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_TRACK][j])
                {
                    dwRenderEngine_destroyBuffer(m_trackPointCloud[j], m_renderEngine);
                    delete[] m_trackData[j];
                }
            }

            if (m_renderEngine != DW_NULL_HANDLE)
            {
                CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
            }
        }
        dwSAL_release(m_sal);

        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRectf bounds;
        bounds.width  = width;
        bounds.height = height;
        bounds.x      = 0;
        bounds.y      = 0;
        dwRenderEngine_setBounds(bounds, m_renderEngine);
    }

    ///------------------------------------------------------------------------------
    /// Rendering
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        // Render here
        renderFrame();
        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        // Process stuff
        computeSpin();
    }

protected:
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

    //#######################################################################################
    // Initialize Renderer
    //#######################################################################################
    void initializeRenderer()
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        // Set some renderer defaults
        dwRenderEngineParams reParams{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&reParams, getWindowWidth(), getWindowHeight()));
        reParams.maxBufferCount = 20;
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &reParams, m_viz));
        dwRenderEngineTileState textTile{};
        dwRenderEngine_initTileState(&textTile);
        textTile.layout.positionLayout  = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
        textTile.layout.sizeLayout      = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
        textTile.layout.positionType    = DW_RENDER_ENGINE_TILE_POSITION_TYPE_BOTTOM_LEFT;
        textTile.backgroundColor        = {0.1f, 0.1f, 0.1f, 1.0f};
        textTile.layout.viewport.width  = 450.0f;
        textTile.layout.viewport.height = 220.0f;
        textTile.layout.viewport.x      = 0.0f;
        textTile.layout.viewport.y      = 0.0f;

        textTile.color             = {1.0f, 1.0f, 1.0f, 1.0f};
        textTile.font              = DW_RENDER_ENGINE_FONT_VERDANA_16;
        textTile.coordinateRange2D = {textTile.layout.viewport.width, textTile.layout.viewport.height};

        CHECK_DW_ERROR(dwRenderEngine_addTile(&m_textTile, &textTile, m_renderEngine));

        {
            // Initialize Point Clouds
            for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; j++)
            {
                // Initialize point cloud for detections
                if (m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_DETECTION][j])
                {
                    uint32_t maximumPoints = m_radarProperties.maxReturnsPerScan[DW_RADAR_RETURN_TYPE_DETECTION][j];
                    CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_detectionPointCloud[j],
                                                               DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                                               sizeof(dwVector4f), 0,
                                                               2 * maximumPoints, m_renderEngine));
                    m_detectionData[j] = new dwRadarDetection[maximumPoints];
                }

                // Initialize point cloud for tracks
                if (m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_TRACK][j])
                {
                    uint32_t maximumPoints = m_radarProperties.maxReturnsPerScan[DW_RADAR_RETURN_TYPE_TRACK][j];

                    CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_trackPointCloud[j],
                                                               DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                                               sizeof(ColoredPoint3D), 0,
                                                               2 * maximumPoints, m_renderEngine));
                    m_trackData[j] = new dwRadarTrack[maximumPoints];
                }
            }
        }
    }

    void preparePointCloudDumps(ProgramArguments& arguments)
    {
        m_outputDir = arguments.get("output-dir");

        if (!m_outputDir.empty())
        {
            auto dirExist = [](const std::string& dir) -> bool {
                struct stat buffer;
                return (stat(dir.c_str(), &buffer) == 0);
            };

            if (dirExist(m_outputDir))
            {
                rmdir(m_outputDir.c_str());
            }

            mkdir(m_outputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

            if (chdir(m_outputDir.c_str()))
            {
                logError("Unable to change to output directory: %s\n", m_outputDir.c_str());
                exit(-1);
            }
        }
    }

    //#######################################################################################
    // Initialize Sensor
    //#######################################################################################
    void initializeSensor(ProgramArguments& arguments)
    {
        // create Radar interface
        m_radarSensor = DW_NULL_HANDLE;
        {
            dwSensorParams params{};

            std::string parameterString;
            std::string protocolString;

            if (strcmp(arguments.get("protocol").c_str(), "") != 0)
            {
                protocolString = arguments.get("protocol");

                if (protocolString == "radar.virtual")
                    m_recordedRadar = true;
                else
                    m_recordedRadar = false;
            }

            if (strcmp(arguments.get("params").c_str(), "") != 0)
                parameterString = arguments.get("params");

            if (protocolString.empty() || parameterString.empty())
            {
                std::cout << "INVALID PARAMETERS" << std::endl;
                exit(-1);
            }

            params.protocol   = protocolString.c_str();
            params.parameters = parameterString.c_str();

            m_numScansPerSpin = 0;
            if (dwSAL_createSensor(&m_radarSensor, params, m_sal) == DW_SUCCESS)
            {
                // Get Radar properties
                if (dwSensorRadar_getProperties(&m_radarProperties, m_radarSensor) == DW_SUCCESS)
                {
                    if (m_radarProperties.scansPerSecond != 0)
                    {
                        // Enable all supported scans
                        for (size_t i = 0; i < DW_RADAR_RETURN_TYPE_COUNT; i++)
                        {
                            for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; j++)
                            {
                                if (!m_radarProperties.supportedScanTypes[i][j])
                                    continue;

                                m_numScansPerSpin++;

                                dwRadarScanType type = {
                                    .returnType = static_cast<dwRadarReturnType>(i),
                                    .range      = static_cast<dwRadarRange>(j),
                                };

                                dwSensorRadar_toggleScanType(true, type, m_radarSensor);
                            }
                        }
                    }
                    else
                        throw std::runtime_error("In Radar Properties - packetsPerSecond is 0");
                }
                else
                    throw std::runtime_error("Could not read radar properties");
            }
            else
                throw std::runtime_error("Sensor Initialization Failed");
        }
    }

protected:
    void dumpRadarFrame(dwRadarDetection* data, uint32_t numPoints, dwTime_t timestamp)
    {
        const std::string radarFilename = std::to_string(timestamp) + ".bin";
        std::ofstream fout;
        fout.open(radarFilename, std::ios::binary | std::ios::out);
        fout.write(reinterpret_cast<char*>(data), numPoints * sizeof(dwRadarDetection));
        fout.close();
    }

    void renderFrame()
    {

        glDepthFunc(GL_LESS);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 3D rendering
        dwRenderEngine_setTile(0, m_renderEngine);
        dwRenderEngine_resetTile(m_renderEngine);
        dwRenderEngine_setLineWidth(2.0f, m_renderEngine);
        dwMatrix4f mat{};
        memcpy(mat.array, getMouseView().getModelView(), sizeof(dwTransformation3f));
        dwRenderEngine_setModelView(&mat, m_renderEngine);
        memcpy(mat.array, getMouseView().getProjection(), sizeof(dwTransformation3f));
        dwRenderEngine_setProjection(&mat, m_renderEngine);

        // Render grid
        if (m_showGrid)
        {

            dwRenderEngine_setColor({0.25f, 0.25f, 0.25f, 1.0f}, m_renderEngine);
            glClear(GL_DEPTH_BUFFER_BIT);
            dwRenderEngine_renderPlanarGrid3D({0.0f, 0.0f,
                                               WORLD_GRID_SIZE_IN_METERS_WIDTH * 0.25f,
                                               WORLD_GRID_SIZE_IN_METERS_HEIGHT * 0.25f},
                                              WORLD_GRID_RES_IN_METERS * 0.5f,
                                              WORLD_GRID_RES_IN_METERS * 0.5f,
                                              &DW_IDENTITY_MATRIX4F, m_renderEngine);
            glClear(GL_DEPTH_BUFFER_BIT);
            dwRenderEngine_setColor({0.0f, 1.0f, 0.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_renderEllipticalGrid3D({0.0f, 0.0f,
                                                   WORLD_GRID_SIZE_IN_METERS_WIDTH * 0.12f,
                                                   WORLD_GRID_SIZE_IN_METERS_HEIGHT * 0.12f},
                                                  WORLD_GRID_RES_IN_METERS * 0.5f,
                                                  WORLD_GRID_RES_IN_METERS * 0.5f,
                                                  &DW_IDENTITY_MATRIX4F, m_renderEngine);
            glClear(GL_DEPTH_BUFFER_BIT);
        }

        // Render worldspace axes
        {
            dwRenderEngine_setLineWidth(3.0f, m_renderEngine);
            dwVector3f axis[2]{};
            float32_t axisLength = 100.0f;
            axis[1]              = {axisLength, 0.0f, 0.0f};
            dwRenderEngine_setColor({1.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D,
                                  axis,
                                  sizeof(dwVector3f),
                                  0,
                                  1,
                                  m_renderEngine);
            axis[1] = {0.0f, axisLength, 0.0f};
            dwRenderEngine_setColor({0.0f, 1.0f, 0.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D,
                                  axis,
                                  sizeof(dwVector3f),
                                  0,
                                  1,
                                  m_renderEngine);
            axis[1] = {0.0f, 0.0f, axisLength};
            dwRenderEngine_setColor({0.0f, 0.0f, 1.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D,
                                  axis,
                                  sizeof(dwVector3f),
                                  0,
                                  1,
                                  m_renderEngine);
            glClear(GL_DEPTH_BUFFER_BIT);
        }

        if (m_renderDetections)
        {
            dwRenderEngine_setPointSize(5.0f, m_renderEngine);
            dwRenderEngine_setColor({1.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
            for (size_t i = 0; i < DW_RADAR_RANGE_COUNT; i++)
            {
                if (!m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_DETECTION][i])
                    continue;
                dwRenderEngine_renderBuffer(m_detectionPointCloud[i],
                                            m_accumulatedPoints[DW_RADAR_RETURN_TYPE_DETECTION][i], m_renderEngine);
            }
        }

        if (m_renderTracks)
        {
            dwRenderEngine_setPointSize(5.0f, m_renderEngine);
            dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_ATTRIBUTE_RGBA, 1.0, m_renderEngine);
            for (size_t i = 0; i < DW_RADAR_RANGE_COUNT; i++)
            {
                if (!m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_TRACK][i])
                    continue;
                dwRenderEngine_renderBuffer(
                    m_trackPointCloud[i], m_accumulatedPoints[DW_RADAR_RETURN_TYPE_TRACK][i], m_renderEngine);
            }
        }

        // Overlay text
        if (m_showText)
        {
            dwRenderEngine_setTile(m_textTile, m_renderEngine);
            dwRenderEngine_resetTile(m_renderEngine);

            float32_t yInc = 18.0f;
            float32_t y    = yInc;
            dwRenderEngine_renderText2D(m_message1.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;
            dwRenderEngine_renderText2D(m_message2.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;
            dwRenderEngine_renderText2D(m_message3.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;
            dwRenderEngine_renderText2D(m_message4.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;
            dwRenderEngine_renderText2D(m_message5.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;
            dwRenderEngine_renderText2D(m_message6.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;

            dwRenderEngine_setColor({1.0f, 0.75f, 0.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_renderText2D(m_message7.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;

            dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_renderText2D(m_message8.c_str(), {0.0f, y}, m_renderEngine);
            y += yInc;
            dwRenderEngine_renderText2D(m_message9.c_str(), {0.0f, y}, m_renderEngine);
        }
    }

    double updateFrequency()
    {
        return m_radarProperties.scansPerSecond * m_freqMultiplier;
    }

    bool isScanComplete(uint32_t numScans)
    {
        return numScans % m_numScansPerSpin == 0;
    }

    void computeSpin()
    {
        const dwRadarScan* nextPacket;
        static uint32_t packetCount = 0;
        static auto t_start         = std::chrono::high_resolution_clock::now();
        static auto t_end           = t_start;

        // Allow pausing for recoded replay
        if (m_recordedRadar && isPaused())
            return;

        // For recorded data thottling check how long to a full sping and match to the radar frequency
        if (m_recordedRadar)
        {
            t_end            = std::chrono::high_resolution_clock::now();
            double duration  = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            double sleepTime = 1000.0 / updateFrequency() - duration;
            // This ensures proper behavior and quick restart in cause of pauses.
            sleepTime = (sleepTime * 1000 > 0) ? sleepTime : 100;

            usleep(sleepTime * 1000);
        }

        // Empty the queue and append all points within the same spin.
        // Update render structures only when a full spin is done.
        dwStatus status = DW_SUCCESS;

        t_start = std::chrono::high_resolution_clock::now();

        bool scanComplete = false;
        while (!scanComplete && status == DW_SUCCESS)
        {
            status = dwSensorRadar_readScan(&nextPacket,
                                            1000000, m_radarSensor);
            if (status == DW_SUCCESS)
            {
                const dwRadarScanType& type = nextPacket->scanType;

                switch (type.returnType)
                {
                case DW_RADAR_RETURN_TYPE_DETECTION:
                    memcpy(m_detectionData[type.range],
                           nextPacket->data,
                           nextPacket->numReturns * sizeof(dwRadarDetection));
                    break;
                case DW_RADAR_RETURN_TYPE_TRACK:
                    memcpy(m_trackData[type.range],
                           nextPacket->data,
                           nextPacket->numReturns * sizeof(dwRadarTrack));
                    break;
                case DW_RADAR_RETURN_TYPE_STATUS:
                    break;
                case DW_RADAR_RETURN_TYPE_COUNT:
                default:
                    std::cout << "RadarReplay: Invalid point type received" << std::endl;
                    break;
                }
                m_accumulatedPoints[type.returnType][type.range] = nextPacket->numReturns;

                m_message1 = "Host timestamp " + toStr(nextPacket->hostTimestamp / 1000000ULL) + "(sec) / " + toStr(nextPacket->hostTimestamp) + "(microsecs)";
                m_message2 = "Sensor timestamp " + toStr(nextPacket->sensorTimestamp / 1000000ULL) + "(sec) / " + toStr(nextPacket->sensorTimestamp) + "(microsecs)";

                dwSensorRadar_returnScan(nextPacket, m_radarSensor);
                packetCount++;

                scanComplete = isScanComplete(packetCount);

                if (type.returnType == DW_RADAR_RETURN_TYPE_DETECTION && type.range == DW_RADAR_RANGE_SHORT)
                {
                    if (!m_outputDir.empty())
                    {
                        dumpRadarFrame(m_detectionData[type.range], nextPacket->numReturns, nextPacket->hostTimestamp);
                    }
                }
            }
        }

        if (status == DW_SAL_SENSOR_ERROR)
        {
            throw std::runtime_error("Unrecoverable sensor error occurred");
        }

        for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; j++)
        {
            size_t pointCount;

            pointCount = m_accumulatedPoints[DW_RADAR_RETURN_TYPE_DETECTION][j];
            if (pointCount > 0 && m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_DETECTION][j])
            {
                dwVector4f* map;

                // Map to the point cloud
                dwRenderEngine_mapBuffer(m_detectionPointCloud[j], reinterpret_cast<void**>(&map), 0,
                                         sizeof(dwVector4f) * pointCount,
                                         DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D, m_renderEngine);

                for (size_t k = 0; k < pointCount; k++)
                {
                    dwRadarDetection updatePoint = m_detectionData[j][k];
                    map[k]                       = dwVector4f{updatePoint.x, updatePoint.y, WORLD_Z, 0.0f};
                }
                dwRenderEngine_unmapBuffer(m_detectionPointCloud[j], DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                           m_renderEngine);
            }

            pointCount = m_accumulatedPoints[DW_RADAR_RETURN_TYPE_TRACK][j];
            if (pointCount > 0 && m_radarProperties.supportedScanTypes[DW_RADAR_RETURN_TYPE_TRACK][j])
            {
                ColoredPoint3D* map;

                // Map to the point cloud
                dwRenderEngine_mapBuffer(m_trackPointCloud[j], reinterpret_cast<void**>(&map), 0,
                                         sizeof(ColoredPoint3D) * pointCount,
                                         DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D, m_renderEngine);

                for (size_t k = 0; k < pointCount; k++)
                {
                    dwRadarTrack updatePoint = m_trackData[j][k];

                    map[k].position = dwVector3f{updatePoint.x, updatePoint.y, WORLD_Z};

                    switch (updatePoint.dynamicState)
                    {
                    case DW_RADAR_DYNAMIC_STATE_STATIONARY:
                        map[k].color = dwVector4f{0.1f, 0.1f, 0.9f, 0.0f};
                        break;
                    case DW_RADAR_DYNAMIC_STATE_MOVING:
                        map[k].color = dwVector4f{0.1f, 0.9f, 0.1f, 0.0f};
                        break;
                    case DW_RADAR_DYNAMIC_STATE_ONCOMING:
                        map[k].color = dwVector4f{0.9f, 0.9f, 0.1f, 0.0f};
                        break;
                    default:
                        map[k].color = dwVector4f{1.0f, 1.0f, 1.0f, 0.0f};
                        break;
                    }
                }
                dwRenderEngine_unmapBuffer(m_trackPointCloud[j], DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                           m_renderEngine);
            }
        }

        m_message3 = "Packets                     " + toStr(packetCount);
        m_message4 = "Application frequency for read Radar Scans (Hz)   " + toStr(updateFrequency());
        m_message6 = "Step in the rect grid: " + toStr(WORLD_GRID_RES_IN_METERS) + "m / radial grid: " + toStr(WORLD_CIRCLE_DR_IN_METERS) + "m";
        m_message7 = "Worldspace axes: Red-OX, Blue-OY, Green - OZ.)";
        m_message8 = "Press 'ESC' - to exit / 'R'-reset / 'G'- show/hide grid / 'F1' - show/hide this box / '+-' - increase/decrease reading scan rate ";
        m_message9 = "/ 'Toggle 0: Detections, 1: Tracks";

        // Grab properties, in case they were changed while running
        dwSensorRadar_getProperties(&m_radarProperties, m_radarSensor);

        // For recorded data, start over at the end of the file
        if (status == DW_END_OF_STREAM)
        {
            dwSensor_reset(m_radarSensor);
            packetCount = 0;
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    std::string dynamicsParams;
    dynamicsParams = "can-driver=can.socket"
                     ",can-params=device=can0"
                     ",rig-configuration=wwdc_rig.json"
                     ",radar-name=FL"
                     ",isReversed=false"
                     ",radome-damping=0.0";

    ProgramArguments arguments(argc, argv,
                               {
                                   ProgramArguments::Option_t("protocol", "radar.virtual"),
                                   ProgramArguments::Option_t("params", ("file=" + dw_samples::SamplesDataPath::get() + "/samples/sensors/radar/conti/ars_v4.bin").c_str()),
                                   ProgramArguments::Option_t("output-dir", ""),
                               },
                               "Radar replay sample which playback .bin video streams in a GL window.");

    // initialize and start a window application
    RadarReplay app(arguments);

    app.initializeWindow("Radar Replay Sample", 1024, 800, arguments.enabled("offscreen"));

    return app.run();
}
