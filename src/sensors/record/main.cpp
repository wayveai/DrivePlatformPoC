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
// SPDX-FileCopyrightText: Copyright (c) 2015-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <dw/sensors/gps/GPS.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/lidar/Lidar.h>
#include <dw/sensors/radar/Radar.h>

// Include all relevant DriveWorks modules

using namespace dw_samples::common;

class SensorRecordSample : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t m_context = DW_NULL_HANDLE;
    dwSALHandle_t m_sal         = DW_NULL_HANDLE;

    std::mutex m_mutex;
    std::thread m_gpsThread;
    std::thread m_canThread;
    std::thread m_lidarThread;
    std::thread m_radarThread;

    volatile bool m_gpsThreadDone   = false;
    volatile bool m_canThreadDone   = false;
    volatile bool m_lidarThreadDone = false;
    volatile bool m_radarThreadDone = false;

    void lockMutex()
    {
        m_mutex.lock();
    }

    void unlockMutex()
    {
        m_mutex.unlock();
    }

public:
    SensorRecordSample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

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

    bool onInitialize() override
    {
        if (getArgs().get("write-file-lidar").empty() &&
            getArgs().get("write-file-radar").empty() &&
            getArgs().get("write-file-gps").empty() &&
            getArgs().get("write-file-can").empty())
        {
            getArgs().printHelp();
            return false;
        }

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        initializeDriveWorks(m_context);
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

        if (!getArgs().get("write-file-gps").empty())
        {
            m_gpsThread = std::thread(rungps, getArgs(), m_sal, this, &m_gpsThreadDone);
        }
        else
        {
            m_gpsThreadDone = true;
        }
        if (!getArgs().get("write-file-can").empty())
        {
            m_canThread = std::thread(runcan, getArgs(), m_sal, this, &m_canThreadDone);
        }
        else
        {
            m_canThreadDone = true;
        }
        if (!getArgs().get("write-file-lidar").empty())
        {
            m_lidarThread = std::thread(runlidar, getArgs(), m_sal, this, &m_lidarThreadDone);
        }
        else
        {
            m_lidarThreadDone = true;
        }
        if (!getArgs().get("write-file-radar").empty())
        {
            m_radarThread = std::thread(runradar, getArgs(), m_sal, this, &m_radarThreadDone);
        }
        else
        {
            m_radarThreadDone = true;
        }

        return true;
    }

    void onReset() override
    {
    }

    void onRelease() override
    {
        if (m_gpsThread.joinable())
            m_gpsThread.join();
        if (m_canThread.joinable())
            m_canThread.join();
        if (m_lidarThread.joinable())
            m_lidarThread.join();
        if (m_radarThread.joinable())
            m_radarThread.join();

        // -----------------------------------------
        // Release DriveWorks context and SAL
        // -----------------------------------------
        dwSAL_release(m_sal);
        dwRelease(m_context);
        dwLogger_release();
    }

    void onProcess() override
    {
        if (m_gpsThreadDone && m_canThreadDone && m_lidarThreadDone && m_radarThreadDone)
        {
            stop();
        }
    }

    void onKeyDown(int key, int /*scancode*/, int /*mods*/) override
    {
        log("key down: %d\n", key);
    }

private:
    //------------------------------------------------------------------------------
    static void rungps(ProgramArguments arguments, dwSALHandle_t sal, SensorRecordSample* app, volatile bool* done)
    {
        *done                                 = false;
        dwSensorHandle_t gpsSensor            = DW_NULL_HANDLE;
        dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
        {
            app->lockMutex();

            std::string gpsDriver = "gps.uart";
            std::string gpsParams = "";

            if (arguments.get("gps-driver") != "")
            {
                gpsDriver = arguments.get("gps-driver");
            }
            if (arguments.get("gps-params") != "")
            {
                gpsParams = arguments.get("gps-params");
            }

            // create GPS bus interface
            {
                dwSensorParams params{};
                params.parameters = gpsParams.c_str();
                params.protocol   = gpsDriver.c_str();
                if (dwSAL_createSensor(&gpsSensor, params, sal) != DW_SUCCESS)
                {
                    std::cout << "Cannot create sensor "
                              << params.protocol << " with "
                              << params.parameters << std::endl;

                    app->unlockMutex();
                    app->stop();
                    return;
                }
            }

            std::string newParams = "";
            if (arguments.has("write-file-gps"))
            {
                newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-gps"));
            }
            dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
            dwSensorSerializer_initialize(&serializer, &serializerParams, gpsSensor);
            dwSensorSerializer_start(serializer);

            if (dwSensor_start(gpsSensor) != DW_SUCCESS)
                app->stop();
            app->unlockMutex();
        }

        // Message msg;
        while (app->shouldRun())
        {
            std::this_thread::yield();

            const uint8_t* data = nullptr;
            size_t size         = 0;

            dwStatus status = DW_NOT_READY;
            while (status == DW_NOT_READY)
            {
                status = dwSensor_readRawData(&data, &size, 1000000, gpsSensor);
                if (status != DW_SUCCESS)
                {
                    break;
                }
                status = dwSensorGPS_processRawData(data, size, gpsSensor);

                dwSensorSerializer_serializeDataAsync(data, size, serializer);

                dwSensor_returnRawData(data, gpsSensor);
            }

            if (status == DW_END_OF_STREAM)
            {
                std::cout << "GPS end of stream reached" << std::endl;
                break;
            }
            else if (status == DW_TIME_OUT)
                continue;

            // if previous process raw data has not failed, then we must have valid message available
            dwGPSFrame frame{};
            if (dwSensorGPS_popFrame(&frame, gpsSensor) != DW_SUCCESS)
            {
                std::cerr << "GPS message was not found in the raw stream" << std::endl;
                continue;
            }

            // log message
            app->lockMutex();
            std::cout << frame.timestamp_us;
            if (status != DW_SUCCESS) // msg.is_error)
            {
                std::cout << " ERROR " << dwGetStatusName(status); // msg.frame.id;
            }
            else
            {
                std::cout << std::setprecision(10)
                          << " lat: " << frame.latitude
                          << " lon: " << frame.longitude
                          << " alt: " << frame.altitude
                          << " course: " << frame.course
                          << " speed: " << frame.speed
                          << " climb: " << frame.climb
                          << " hdop: " << frame.hdop
                          << " vdop: " << frame.vdop;
            }
            std::cout << std::endl;
            app->unlockMutex();
        }

        dwSensorSerializer_stop(serializer);

        dwSensorSerializer_release(serializer);

        dwSensor_stop(gpsSensor);

        dwSAL_releaseSensor(gpsSensor);
        *done = true;
    }

    //------------------------------------------------------------------------------
    static void runcan(ProgramArguments arguments, dwSALHandle_t sal, SensorRecordSample* app, volatile bool* done)
    {
        *done                                 = false;
        dwSensorHandle_t canSensor            = DW_NULL_HANDLE;
        dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
        {
            app->lockMutex();
            std::string canParam  = "";
            std::string canDriver = "can.socket";

            if (arguments.get("can-driver") != "")
            {
                canDriver = arguments.get("can-driver");
            }
            if (arguments.get("can-params") != "")
            {
                canParam = arguments.get("can-params");
            }
            canParam += ",serialize=true";

            // create CAN bus interface
            {
                dwSensorParams canparams{};
                canparams.parameters = canParam.c_str();
                canparams.protocol   = canDriver.c_str();
                if (dwSAL_createSensor(&canSensor, canparams, sal) != DW_SUCCESS)
                {
                    std::cout << "Cannot create sensor " << canparams.protocol << " with "
                              << canparams.parameters << std::endl;

                    app->unlockMutex();
                    app->stop();
                    return;
                }
            }

            std::string newParams = "";
            if (arguments.has("write-file-can"))
            {
                newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-can"));
            }
            dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
            dwSensorSerializer_initialize(&serializer, &serializerParams, canSensor);
            dwSensorSerializer_start(serializer);
            if (dwSensor_start(canSensor) != DW_SUCCESS)
                app->stop();

            app->unlockMutex();
        }

        // Message msg;
        while (app->shouldRun())
        {
            dwCANMessage msg{};

            const uint8_t* data = nullptr;
            size_t size         = 0;

            dwStatus status = DW_NOT_READY;
            while (status == DW_NOT_READY)
            {
                status = dwSensor_readRawData(&data, &size, 100000, canSensor);
                if (status != DW_SUCCESS)
                {
                    break;
                }

                status = dwSensorCAN_processRawData(data, size, canSensor);

                dwSensorSerializer_serializeDataAsync(data, size, serializer);

                dwSensor_returnRawData(data, canSensor);

                status = status == DW_SUCCESS ? dwSensorCAN_popMessage(&msg, canSensor) : status;
            }

            if (status == DW_END_OF_STREAM)
            {
                std::cout << "CAN end of stream reached" << std::endl;
                break;
            }
            else if (status == DW_TIME_OUT)
                continue;

            // log message
            app->lockMutex();
            std::cout << msg.timestamp_us;
            if (status != DW_SUCCESS) // msg.is_error)
            {
                std::cout << " ERROR " << dwGetStatusName(status); // msg.frame.id;
            }
            else
            {
                std::cout << " [0x" << std::hex << msg.id << "] -> ";
                for (auto i = 0; i < msg.size; i++)
                    std::cout << "0x" << std::hex << (int)msg.data[i] << " ";
                std::cout << std::dec;
            }
            std::cout << std::endl;
            app->unlockMutex();
        }

        dwSensorSerializer_stop(serializer);
        dwSensorSerializer_release(serializer);
        dwSensor_stop(canSensor);

        dwSAL_releaseSensor(canSensor);
        *done = true;
    }

    //------------------------------------------------------------------------------
    static void runlidar(ProgramArguments arguments, dwSALHandle_t sal, SensorRecordSample* app, volatile bool* done)
    {
        *done                                 = false;
        dwSensorHandle_t lidarSensor          = DW_NULL_HANDLE;
        dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
        {
            app->lockMutex();

            std::string lidarDriver = "lidar.uart";
            std::string lidarParams = "";

            if (arguments.get("lidar-driver") != "")
            {
                lidarDriver = arguments.get("lidar-driver");
            }
            if (arguments.get("lidar-params") != "")
            {
                lidarParams = arguments.get("lidar-params");
            }

            // create lidar bus interface
            {
                dwSensorParams params{};
                params.parameters = lidarParams.c_str();
                params.protocol   = lidarDriver.c_str();
                if (dwSAL_createSensor(&lidarSensor, params, sal) != DW_SUCCESS)
                {
                    std::cout << "Cannot create sensor "
                              << params.protocol << " with "
                              << params.parameters << std::endl;

                    app->unlockMutex();
                    app->stop();
                    return;
                }
            }

            std::string newParams = "";
            if (arguments.has("write-file-lidar"))
            {
                newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-lidar"));
            }
            dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
            dwSensorSerializer_initialize(&serializer, &serializerParams, lidarSensor);
            dwSensorSerializer_start(serializer);
            app->unlockMutex();
        }

        // To serialize one needs to disable decoding
        dwSensorLidar_disableDecoding(lidarSensor);

        dwLidarProperties lidarProperties;

        dwSensorLidar_getProperties(&lidarProperties, lidarSensor);

        const dwLidarDecodedPacket* frame;

        // Message msg;
        int packetCount = 0;
        auto start      = std::chrono::high_resolution_clock::now();

        if (dwSensor_start(lidarSensor) != DW_SUCCESS)
            app->stop();

        while (app->shouldRun())
        {
            const uint8_t* data = nullptr;
            size_t size         = 0;

            dwStatus status = DW_SUCCESS;

            status = dwSensor_readRawData(&data, &size, 5000, lidarSensor);

            if (status == DW_SUCCESS)
            {

                // If we are reading too fast, we can throttle here. real sensors are slower than the
                // disk write speed. Virtual sensors can be a lot faster
                do
                    status = dwSensorSerializer_serializeDataAsync(data, size, serializer);
                while (status == DW_NOT_AVAILABLE || status == DW_BUFFER_FULL);

                if (status != DW_SUCCESS)
                {
                    std::cout << "Error serializing packet " << packetCount << std::endl;
                }

                // Here we can optionally decode the packet
                status = dwSensorLidar_processRawData(&frame, data, size, lidarSensor);

                status = dwSensor_returnRawData(data, lidarSensor);
                if (status != DW_SUCCESS)
                {
                    std::cout << "Error returning packet " << packetCount << std::endl;
                }
                packetCount++;

                if (packetCount % 10 == 0)
                    std::cout << "Processed " << packetCount << std::endl;
            }
            else if (status == DW_END_OF_STREAM)
            {
                std::cout << "lidar end of stream reached" << std::endl;
                break;
            }
            else
            {
                if (status != DW_TIME_OUT)
                    std::cout << "Error reading packet " << packetCount << std::endl;
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "Processed " << packetCount << " packets in " << elapsed.count() / 1000.0 << " s\n";

        dwSensorSerializer_stop(serializer);

        dwSensorSerializer_release(serializer);

        dwSensor_stop(lidarSensor);

        dwSAL_releaseSensor(lidarSensor);
        *done = true;
    }

    //------------------------------------------------------------------------------
    static void runradar(ProgramArguments arguments, dwSALHandle_t sal, SensorRecordSample* app, volatile bool* done)
    {
        *done                                 = false;
        dwSensorHandle_t radarSensor          = DW_NULL_HANDLE;
        dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
        {
            app->lockMutex();

            std::string radarDriver = "radar.socket";
            std::string radarParams = "decoding=false,";

            if (arguments.get("radar-driver") != "")
            {
                radarDriver = arguments.get("radar-driver");
            }
            if (arguments.get("radar-params") != "")
            {
                radarParams += arguments.get("radar-params");
            }

            // create radar bus interface
            {
                dwSensorParams params{};
                params.parameters = radarParams.c_str();
                params.protocol   = radarDriver.c_str();
                if (dwSAL_createSensor(&radarSensor, params, sal) != DW_SUCCESS)
                {
                    std::cout << "Cannot create sensor "
                              << params.protocol << " with "
                              << params.parameters << std::endl;
                    app->unlockMutex();
                    app->stop();
                    return;
                }
            }
            std::string newParams = "";
            if (arguments.has("write-file-radar"))
            {
                newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-radar"));
            }
            dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
            dwSensorSerializer_initialize(&serializer, &serializerParams, radarSensor);
            dwSensorSerializer_start(serializer);
            app->unlockMutex();
        }

        // To serialize one needs to disable decoding
        dwSensorRadar_setDataDecoding(false, radarSensor);

        dwRadarProperties radarProperties;

        dwSensorRadar_getProperties(&radarProperties, radarSensor);

        const dwRadarScan* frame = nullptr;

        // Message msg;
        int packetCount = 0;
        auto start      = std::chrono::high_resolution_clock::now();
        std::chrono::microseconds usec(1);

        if (dwSensor_start(radarSensor) != DW_SUCCESS)
            app->stop();
        while (app->shouldRun())
        {
            const uint8_t* data = nullptr;
            size_t size         = 0;

            dwStatus status = DW_SUCCESS;
            status          = dwSensor_readRawData(&data, &size, 500000, radarSensor);
            if (status == DW_SUCCESS)
            {

                // If we are reading too fast, we can throttle here. real sensors are slower than the
                // disk write speed. Virtual sensors can be a lot faster
                do
                {
                    status = dwSensorSerializer_serializeDataAsync(data, size, serializer);
                } while (status == DW_NOT_AVAILABLE || status == DW_BUFFER_FULL);

                if (status != DW_SUCCESS)
                {
                    std::cout << "Error serializing packet " << packetCount << std::endl;
                }

                // Here we can optionally decode the packet
                status = dwSensorRadar_processRawData(&frame, data, size, radarSensor);
                if (status != DW_SUCCESS)
                {
                    std::cout << "Error processing raw data" << std::endl;
                    return;
                }

                status = dwSensor_returnRawData(data, radarSensor);
                if (status != DW_SUCCESS)
                {
                    std::cout << "Error returning packet " << packetCount << std::endl;
                }
                packetCount++;

                if (packetCount % 10 == 0)
                    std::cout << "Processed " << packetCount << std::endl;
            }
            else if (status == DW_END_OF_STREAM)
            {
                std::cout << "radar end of stream reached" << std::endl;
                break;
            }
            else
            {
                if (status != DW_TIME_OUT)
                    std::cout << "Error reading packet " << packetCount << std::endl;
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "Processed " << packetCount << " packets in " << elapsed.count() / 1000.0 << " s\n";

        dwSensorSerializer_stop(serializer);

        dwSensorSerializer_release(serializer);

        dwSensor_stop(radarSensor);

        dwSAL_releaseSensor(radarSensor);
        *done = true;
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    // parse user given arguments and bail out if there is --help request or proceed
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("can-driver", "can.socket"),
                           ProgramArguments::Option_t("can-params", ""),
                           ProgramArguments::Option_t("gps-driver", "gps.uart"),
                           ProgramArguments::Option_t("gps-params", ""),
                           ProgramArguments::Option_t("lidar-driver", "lidar.socket"),
                           ProgramArguments::Option_t("lidar-params", ""),
                           ProgramArguments::Option_t("radar-driver", "radar.socket"),
                           ProgramArguments::Option_t("radar-params", ""),
                           ProgramArguments::Option_t("write-file-can", ""),
                           ProgramArguments::Option_t("write-file-gps", ""),
                           ProgramArguments::Option_t("write-file-lidar", ""),
                           ProgramArguments::Option_t("write-file-radar", "")},
                          "One or more of the following output file options is required:\n"
                          "\t --write-file-gps=/path/to/file.gps \t: file to record GPS data to\n"
                          "\t --write-file-can=/path/to/canbusfile \t: file to record CAN data to\n"
                          "\t --write-file-lidar=/path/to/lidarfile \t: file to record Lidar data to\n"
                          "\t --write-file-radar=/path/to/radarfile \t: file to record Radar data to\n"
                          "Additional options are:\n"
                          "\t --can-driver=can.socket \t\t: CAN driver to open (default=can.socket)\n"
                          "\t --can-params=device=can0,bus=d \t: parameters passed to CAN sensor\n"
                          "\t --gps-driver=gps.uart \t\t\t: GPS sensor driver (default=gps.uart)\n"
                          "\t --gps-params=device=/dev/ttyACM0 \t: parameters passed to GPS sensor\n"
                          "\t --lidar-driver=lidar.socket \t\t\t: Lidar sensor driver (default=lidar.socket)\n"
                          "\t --lidar-params=device=QUAN_M18A,file=filename\t: parameters passed to LIDAR sensor\n"
                          "\t --radar-driver=radar.socket \t\t\t: Radar sensor driver (default=radar.socket)\n"
                          "\t --radar-params=file=filename\t: parameters passed to RADAR sensor\n");

    SensorRecordSample app(args);

    return app.run();
}
