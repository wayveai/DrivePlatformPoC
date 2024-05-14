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
// SPDX-FileCopyrightText: Copyright (c) 2015-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <framework/ProgramArguments.hpp>
#include <framework/SamplesDataPath.hpp>
#include <framework/Log.hpp>
#include <framework/Checks.hpp>

// CORE
#include <dw/core/logger/Logger.h>
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>
#include <dw/core/signal/SignalStatus.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/gps/GPS.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static bool gRun      = true;
static bool gDualMode = false;

//------------------------------------------------------------------------------
void sig_int_handler(int)
{
    gRun = false;
}

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
#ifndef WINDOWS
    struct sigaction action = {};
    action.sa_handler       = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
#endif

    gRun = true;

    ProgramArguments arguments(
        {ProgramArguments::Option_t("driver", "gps.virtual"),
         ProgramArguments::Option_t("dual-mode", "false"),
         ProgramArguments::Option_t("params", (std::string("file=") + dw_samples::SamplesDataPath::get() + "/samples/sensors/gps/1.gps").c_str())});

    if (!arguments.parse(argc, argv) || (!arguments.has("driver") && !arguments.has("params")))
    {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "\t--driver=gps.virtual \t\t\t: one of the available GPS drivers "
                  << "(see sample_sensors_info)\n";
        std::cout << "\t--params=file=file.gps,arg2=value \t: comma separated "
                  << "key=value parameters for the sensor "
                  << "(see sample_sensor_info for a set of supported parameters)\n";

        return -1;
    }

    dwContextHandle_t sdk = DW_NULL_HANDLE;
    dwSALHandle_t hal     = DW_NULL_HANDLE;

    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

    CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &sdkParams));

    // create HAL module of the SDK
    dwSAL_initialize(&hal, sdk);

    // Disable dual mode since there will be conflict for some specific sensor if the resource is exclusive
    gDualMode = arguments.get("dual-mode") == "true";
    // open same GPS sensor twice, to demonstrate capability of sensor data sharing
    dwSensorHandle_t gpsSensor[2] = {DW_NULL_HANDLE, DW_NULL_HANDLE};
    int numSensors                = gDualMode ? 2 : 1;
    for (int32_t i = 0; i < numSensors; i++)
    {
        dwSensorParams params{};
        std::string parameterString = arguments.get("params");
        params.parameters           = parameterString.c_str();
        params.protocol             = arguments.get("driver").c_str();
        if (dwSAL_createSensor(&gpsSensor[i], params, hal) != DW_SUCCESS)
        {
            std::cout << "Cannot create sensor " << params.protocol
                      << " with " << params.parameters << std::endl;

            dwSAL_release(hal);
            dwRelease(sdk);
            dwLogger_release();

            return -1;
        }
    }

    for (int32_t i = 0; i < numSensors; i++)
    {
        gRun = gRun && dwSensor_start(gpsSensor[i]) == DW_SUCCESS;
    }

    // Message msg;
    bool sensorRun[2] = {gRun, gRun};
    while (gRun)
    {
        if (!sensorRun[0] && !sensorRun[numSensors - 1])
            break;

        for (int i = 0; i < numSensors; i++)
        {
            if (!sensorRun[i])
                continue;

            dwGPSFrame frame;
            dwStatus status = DW_FAILURE;

            status = dwSensorGPS_readFrame(&frame, 50000, gpsSensor[i]);

            if (status == DW_END_OF_STREAM)
            {
                std::cout << "GPS[" << i << "] end of stream reached" << std::endl;
                sensorRun[i] = false;
                break;
            }
            else if (status == DW_TIME_OUT)
                continue;

            // log message
            std::cout << "GPS[" << i << "] - " << frame.timestamp_us;
            if (status != DW_SUCCESS) // msg.is_error)
            {
                std::cout << " ERROR " << dwGetStatusName(status); // msg.frame.id;
            }
            else
            {
                std::cout << std::setprecision(10);

                if (dwSignal_checkSignalValidity(frame.validityInfo.latitude) == DW_SUCCESS)
                    std::cout << " lat: " << frame.latitude;

                if (dwSignal_checkSignalValidity(frame.validityInfo.longitude) == DW_SUCCESS)
                    std::cout << " lon: " << frame.longitude;

                if (dwSignal_checkSignalValidity(frame.validityInfo.altitude) == DW_SUCCESS)
                    std::cout << " alt: " << frame.altitude;

                if (dwSignal_checkSignalValidity(frame.validityInfo.course) == DW_SUCCESS)
                    std::cout << " course: " << frame.course;

                if (dwSignal_checkSignalValidity(frame.validityInfo.speed) == DW_SUCCESS)
                    std::cout << " speed: " << frame.speed;

                if (dwSignal_checkSignalValidity(frame.validityInfo.climb) == DW_SUCCESS)
                    std::cout << " climb: " << frame.climb;

                if (dwSignal_checkSignalValidity(frame.validityInfo.hdop) == DW_SUCCESS)
                    std::cout << " hdop: " << frame.hdop;

                if (dwSignal_checkSignalValidity(frame.validityInfo.vdop) == DW_SUCCESS)
                    std::cout << " vdop: " << frame.vdop;

                if (dwSignal_checkSignalValidity(frame.validityInfo.hacc) == DW_SUCCESS)
                    std::cout << " hacc: " << frame.hacc;

                if (dwSignal_checkSignalValidity(frame.validityInfo.vacc) == DW_SUCCESS)
                    std::cout << " vacc: " << frame.vacc;

                if (dwSignal_checkSignalValidity(frame.validityInfo.mode) == DW_SUCCESS)
                    std::cout << " gps mode: " << frame.mode;

                if (dwSignal_checkSignalValidity(frame.validityInfo.utcTimeUs) == DW_SUCCESS)
                    std::cout << " gps utc time: " << frame.utcTimeUs;
            }
            std::cout << std::endl;
        }
    }

    for (int32_t i = 0; i < numSensors; i++)
    {
        dwSensor_stop(gpsSensor[i]);
        dwSAL_releaseSensor(gpsSensor[i]);
    }

    // release used objects in correct order
    dwSAL_release(hal);
    dwRelease(sdk);
    dwLogger_release();

    return 0;
}
