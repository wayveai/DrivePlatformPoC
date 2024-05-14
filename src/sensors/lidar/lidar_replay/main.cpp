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

#include <dw/core/base/Version.h>
#include <framework/DriveWorksSample.hpp>
#include <dw/sensors/lidar/Lidar.h>

// Include all relevant DriveWorks modules

#include <sys/stat.h>
#include <unistd.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Template of a sample. Put some description what the sample does here
//------------------------------------------------------------------------------
class LidarReplaySample : public DriveWorksSample
{
private:
    std::string m_outputDir = "";

    dwContextHandle_t m_context = DW_NULL_HANDLE;
    dwSALHandle_t m_sal         = DW_NULL_HANDLE;

    dwSensorHandle_t m_lidarSensor = DW_NULL_HANDLE;
    dwLidarProperties m_lidarProperties{};
    bool m_recordedLidar = false;
    std::unique_ptr<float32_t[]> m_pointCloud;

    // Rendering
    dwVisualizationContextHandle_t m_visualizationContext = DW_NULL_HANDLE;

    dwRenderEngineColorByValueMode m_colorByValueMode = DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_XY;

    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    uint32_t m_gridBuffer                 = 0;
    uint32_t m_gridBufferPrimitiveCount   = 0;
    uint32_t m_pointCloudBuffer           = 0;
    uint32_t m_pointCloudBufferCapacity   = 0; // max storage
    uint32_t m_pointCloudBufferSize       = 0; // actual size

    std::string m_message1;
    std::string m_message2;
    std::string m_message3;
    std::string m_message4;
    std::string m_message5;
    std::string m_message6;
    std::string m_message7;

public:
    LidarReplaySample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    void initializeDriveWorks(dwContextHandle_t& context)
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

        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, context));

        std::string parameterString;
        std::string protocolString;
        dwSensorParams params{};
        if (strcmp(getArgument("protocol").c_str(), "") != 0)
        {
            protocolString  = getArgument("protocol");
            m_recordedLidar = (protocolString == "lidar.virtual") ? true : false;
        }

        if (strcmp(getArgument("params").c_str(), "") != 0)
            parameterString = getArgument("params");

        std::string showIntensity = getArgument("show-intensity");
        std::transform(showIntensity.begin(), showIntensity.end(), showIntensity.begin(), ::tolower);
        if (showIntensity.compare("true") == 0)
            m_colorByValueMode = DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_INTENSITY;

        if (protocolString.empty() || parameterString.empty())
        {
            logError("INVALID PARAMETERS\n");
            exit(-1);
        }

        params.protocol   = protocolString.c_str();
        params.parameters = parameterString.c_str();
        CHECK_DW_ERROR(dwSAL_createSensor(&m_lidarSensor, params, m_sal));
        // Get lidar properties
        CHECK_DW_ERROR(dwSensorLidar_getProperties(&m_lidarProperties, m_lidarSensor));

        // Allocate bigger buffer in case certain spin exceeds the pointsPerSpin in lidar property
        m_pointCloudBufferCapacity = m_lidarProperties.pointsPerSecond;
        m_pointCloud.reset(new float32_t[m_pointCloudBufferCapacity * m_lidarProperties.pointStride]);
    }

    void preparePointCloudDumps()
    {
        m_outputDir = getArgument("output-dir");

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

    /// -----------------------------
    /// Initialize everything of a sample here incl. SDK components
    /// -----------------------------
    bool onInitialize() override
    {
        log("Starting my sample application...\n");

        preparePointCloudDumps();

        initializeDriveWorks(m_context);

        dwVisualizationInitialize(&m_visualizationContext, m_context);

        // -----------------------------
        // Initialize RenderEngine
        // -----------------------------
        dwRenderEngineParams renderEngineParams{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&renderEngineParams,
                                                        getWindowWidth(),
                                                        getWindowHeight()));
        renderEngineParams.defaultTile.backgroundColor = {0.0f, 0.0f, 1.0f, 1.0f};
        CHECK_DW_ERROR_MSG(dwRenderEngine_initialize(&m_renderEngine, &renderEngineParams, m_visualizationContext),
                           "Cannot initialize Render Engine, maybe no GL context available?");

        CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_pointCloudBuffer, DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                                   sizeof(dwVector3f), 0, m_pointCloudBufferCapacity, m_renderEngine));

        CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_gridBuffer, DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D,
                                                   sizeof(dwVector3f), 0, 10000, m_renderEngine));

        dwMatrix4f identity = DW_IDENTITY_MATRIX4F;
        CHECK_DW_ERROR(dwRenderEngine_setBufferPlanarGrid3D(m_gridBuffer, {0.f, 0.f, 100.f, 100.f},
                                                            5.0f, 5.0f,
                                                            &identity, m_renderEngine));

        dwRenderEngine_getBufferMaxPrimitiveCount(&m_gridBufferPrimitiveCount, m_gridBuffer, m_renderEngine);

        dwSensor_start(m_lidarSensor);

        return true;
    }

    ///------------------------------------------------------------------------------
    /// This method is executed when user presses `R`, it indicates that sample has to reset
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        dwSensor_reset(m_lidarSensor);
        dwRenderEngine_reset(m_renderEngine);
    }

    ///------------------------------------------------------------------------------
    /// This method is executed on release, free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if (m_pointCloudBuffer != 0)
        {
            CHECK_DW_ERROR(dwRenderEngine_destroyBuffer(m_pointCloudBuffer, m_renderEngine));
        }
        if (m_gridBuffer != 0)
        {
            CHECK_DW_ERROR(dwRenderEngine_destroyBuffer(m_gridBuffer, m_renderEngine));
        }

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        if (m_visualizationContext != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwVisualizationRelease(m_visualizationContext));
        }

        if (m_lidarSensor != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwSAL_releaseSensor(m_lidarSensor));
        }

        // -----------------------------------------
        // Release DriveWorks context and SAL
        // -----------------------------------------
        dwSAL_release(m_sal);

        if (m_context != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRelease(m_context));
        }

        CHECK_DW_ERROR(dwLogger_release());
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRectf bounds{.x = 0.0f, .y = 0.0f};
        bounds.width  = width;
        bounds.height = height;
        dwRenderEngine_setBounds(bounds, m_renderEngine);
    }

    void updateFrame(uint32_t accumulatedPoints, uint32_t packetCount,
                     dwTime_t hostTimestamp, dwTime_t sensorTimestamp)
    {
        m_pointCloudBufferSize = accumulatedPoints;
        // Grab properties, in case they were changed while running
        dwSensorLidar_getProperties(&m_lidarProperties, m_lidarSensor);

        if (!m_outputDir.empty())
        {
            dumpLidarFrame(accumulatedPoints, hostTimestamp);
        }

        dwRenderEngine_setBuffer(m_pointCloudBuffer,
                                 DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                 m_pointCloud.get(),
                                 sizeof(dwLidarPointXYZI),
                                 0,
                                 m_pointCloudBufferSize,
                                 m_renderEngine);

        m_message1 = "Host timestamp    (us) " + std::to_string(hostTimestamp);
        m_message2 = "Sensor timestamp (us) " + std::to_string(sensorTimestamp);
        m_message3 = "Packets per scan         " + std::to_string(packetCount);
        m_message4 = "Points per scan           " + std::to_string(accumulatedPoints);
        m_message5 = "Frequency (Hz)           " + std::to_string(m_lidarProperties.spinFrequency);
        m_message6 = "Lidar Device               " + std::string{m_lidarProperties.deviceString};
        m_message7 = "Press ESC to exit";
    }

    void dumpLidarFrame(uint32_t accumulatedPoints, dwTime_t timestamp)
    {
        const std::string lidarFilename = std::to_string(timestamp) + ".bin";
        std::ofstream fout;
        fout.open(lidarFilename, std::ios::binary | std::ios::out);
        fout.write(reinterpret_cast<char*>(m_pointCloud.get()), accumulatedPoints * m_lidarProperties.pointStride * sizeof(float32_t));
        fout.close();
    }

    void computeSpin()
    {
        const dwLidarDecodedPacket* nextPacket;
        static uint32_t packetCount       = 0;
        static uint32_t accumulatedPoints = 0;
        static bool endOfSpin             = false;
        static auto tStart                = std::chrono::high_resolution_clock::now();
        static auto tEnd                  = tStart;

        // For recorded data throttling check how long to a full spin and match to the lidar frequency
        if (m_recordedLidar && endOfSpin)
        {
            tEnd                = std::chrono::high_resolution_clock::now();
            float64_t duration  = std::chrono::duration<float64_t, std::milli>(tEnd - tStart).count();
            float64_t sleepTime = 1000.0 / m_lidarProperties.spinFrequency - duration;

            if (sleepTime > 0.0)
                return;
            else
                endOfSpin = false;
        }

        // Empty the queue and append all points within the same spin.
        // Update render structures only when a full spin is done.
        dwStatus status = DW_NOT_AVAILABLE;

        dwTime_t hostTimestamp   = 0;
        dwTime_t sensorTimestamp = 0;

        tStart = std::chrono::high_resolution_clock::now();
        while (1)
        {
            status = dwSensorLidar_readPacket(&nextPacket, 100000, m_lidarSensor);
            if (status == DW_SUCCESS)
            {
                packetCount++;
                hostTimestamp   = nextPacket->hostTimestamp;
                sensorTimestamp = nextPacket->sensorTimestamp;

                // Append the packet to the buffer
                float32_t* map = &m_pointCloud[accumulatedPoints * m_lidarProperties.pointStride];
                memcpy(map, nextPacket->pointsXYZI, nextPacket->nPoints * sizeof(dwLidarPointXYZI));

                accumulatedPoints += nextPacket->nPoints;

                // If we go beyond a full spin, update the render data then return
                if (nextPacket->scanComplete)
                {
                    updateFrame(accumulatedPoints,
                                packetCount,
                                hostTimestamp,
                                sensorTimestamp);

                    accumulatedPoints = 0;
                    packetCount       = 0;
                    endOfSpin         = true;
                    dwSensorLidar_returnPacket(nextPacket, m_lidarSensor);
                    return;
                }

                dwSensorLidar_returnPacket(nextPacket, m_lidarSensor);
            }
            else if (status == DW_END_OF_STREAM)
            {
                updateFrame(accumulatedPoints,
                            packetCount,
                            hostTimestamp,
                            sensorTimestamp);

                // For recorded data, start over at the end of the file
                dwSensor_reset(m_lidarSensor);
                accumulatedPoints = 0;
                packetCount       = 0;
                endOfSpin         = true;

                return;
            }
            else if (status == DW_TIME_OUT)
            {
                std::cout << "Read lidar packet: timeout" << std::endl;
            }
            else
            {
                stop();
                return;
            }
        }
    }

    void onProcess() override
    {
        computeSpin();
    }

    void onRender() override
    {
        // render text in the middle of the window
        dwRenderEngine_reset(m_renderEngine);
        dwRenderEngine_setTile(0, m_renderEngine);
        dwRenderEngine_setModelView(getMouseView().getModelView(), m_renderEngine);
        dwRenderEngine_setProjection(getMouseView().getProjection(), m_renderEngine);

        dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_DARKGREY, m_renderEngine);

        dwRenderEngine_setBackgroundColor({0.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_renderBuffer(m_gridBuffer, m_gridBufferPrimitiveCount, m_renderEngine);

        dwRenderEngine_setColorByValue(m_colorByValueMode, 130.0f, m_renderEngine);

        dwRenderEngine_renderBuffer(m_pointCloudBuffer, m_pointCloudBufferSize, m_renderEngine);

        dwRenderEngine_setProjection(&DW_IDENTITY_MATRIX4F, m_renderEngine);
        dwRenderEngine_setModelView(&DW_IDENTITY_MATRIX4F, m_renderEngine);
        dwVector2f range{static_cast<float32_t>(getWindowWidth()),
                         static_cast<float32_t>(getWindowHeight())};
        dwRenderEngine_setCoordinateRange2D(range, m_renderEngine);
        dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_16, m_renderEngine);
        dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_GREEN, m_renderEngine);
        dwRenderEngine_renderText2D(m_message1.c_str(), {20.f, getWindowHeight() - 30.f}, m_renderEngine);
        dwRenderEngine_renderText2D(m_message2.c_str(), {20.f, getWindowHeight() - 50.f}, m_renderEngine);
        dwRenderEngine_renderText2D(m_message3.c_str(), {20.f, getWindowHeight() - 70.f}, m_renderEngine);
        dwRenderEngine_renderText2D(m_message4.c_str(), {20.f, getWindowHeight() - 90.f}, m_renderEngine);
        dwRenderEngine_renderText2D(m_message5.c_str(), {20.f, getWindowHeight() - 110.f}, m_renderEngine);
        dwRenderEngine_renderText2D(m_message6.c_str(), {20.f, getWindowHeight() - 130.f}, m_renderEngine);
        dwRenderEngine_renderText2D(m_message7.c_str(), {20.f, 20.f}, m_renderEngine);

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    // parse user given arguments and bail out if there is --help request or proceed
    ProgramArguments args(argc, argv,
                          {

                              ProgramArguments::Option_t("protocol", "lidar.virtual"),
                              ProgramArguments::Option_t("params", ("file=" + dw_samples::SamplesDataPath::get() + "/samples/sensors/lidar/hesai-P128-dual_v6_codecs.bin").c_str()),
                              ProgramArguments::Option_t("output-dir", ""),
                              ProgramArguments::Option_t("show-intensity", "false")});

    // -------------------
    // initialize and start a window application (with offscreen support if required)
    LidarReplaySample app(args);

    app.initializeWindow("Lidar Replay Sample", 1024, 800, args.enabled("offscreen"));

    return app.run();
}
