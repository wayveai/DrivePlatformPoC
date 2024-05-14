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
// SPDX-FileCopyrightText: Copyright (c) 2020-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <cstring>

// Sample Includes
#include <framework/DriveWorksSample.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/Checks.hpp>
#include <framework/Log.hpp>

// DW
#include <dw/core/base/Version.h>

// RIG
#include <dw/rig/Rig.h>

// SENSORS
#include <dw/sensors/Sensors.h>
#include <dw/sensors/sensormanager/SensorManager.h>

static bool gRun = true;

void sig_int_handler(int)
{
    gRun = false;
}

void getLidarTimestamp(dwTime_t* timestamp, dwSensorHandle_t lidar)
{
    const dwTime_t timeout = 5000000; // 5 seconds [us]
    const dwLidarDecodedPacket* nextPacket;

    dwStatus status = dwSensorLidar_readPacket(&nextPacket, timeout, lidar);
    if (status == DW_SUCCESS)
    {
        *timestamp = nextPacket->hostTimestamp;
        dwSensorLidar_returnPacket(nextPacket, lidar);
    }
    else if (status == DW_TIME_OUT)
    {
        std::cout << "Read lidar packet: timeout" << std::endl;
    }
    else if (status == DW_END_OF_STREAM)
    {
        gRun = false;
    }
    else
    {
        // Should not timeout.
        CHECK_DW_ERROR(status);
    }
}

int main(int argc, const char** argv)
{
    ProgramArguments arguments(
        {
            ProgramArguments::Option_t("time-params", (std::string("file=") + dw_samples::SamplesDataPath::get() + "/samples/sensors/time/time_nvpps.bin").c_str()),
            ProgramArguments::Option_t("lidar-params", (std::string("file=") + dw_samples::SamplesDataPath::get() + "/samples/sensors/time/lidar_hdl32e.bin").c_str()),
        });

    if (!arguments.parse(argc, argv))
    {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "\t--time-params=file=/path/to/file\t: parameters passed to TimeSensor driver"
                  << "(see sample_sensor_info for a set of supported parameters)\n";
        std::cout << "\t--lidar-params=file=/path/to/file\t: parameters passed to Lidar driver"
                  << "(see sample_sensor_info for a set of supported parameters)\n";
        return -1;
    }

    std::cout << "Program Arguments:\n"
              << arguments.printList() << std::endl;

    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_DEBUG);

    std::string syncString = "output-timestamp=synced," + arguments.get("lidar-params");
    std::string hostString = "output-timestamp=host," + arguments.get("lidar-params");
    std::string utcString  = "output-timestamp=raw," + arguments.get("lidar-params");

    dwSensorParams lidarParams{};
    lidarParams.protocol = "lidar.virtual";

    dwSensorParams timeParams{};
    timeParams.protocol   = "time.virtual";
    timeParams.parameters = arguments.get("time-params").c_str();

    dwContextHandle_t sdk = DW_NULL_HANDLE;
    dwSALHandle_t sal     = DW_NULL_HANDLE;

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};
    CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &sdkParams));

    // create HAL module of the SDK
    CHECK_DW_ERROR(dwSAL_initialize(&sal, sdk));

    dwSensorHandle_t timeSensor        = DW_NULL_HANDLE;
    dwSensorHandle_t lidarSensorSynced = DW_NULL_HANDLE;
    dwSensorHandle_t lidarSensorRaw    = DW_NULL_HANDLE;
    dwSensorHandle_t lidarSensor       = DW_NULL_HANDLE;

    // Creating time sensor with SAL enables time synchronization
    lidarParams.parameters = syncString.c_str();
    CHECK_DW_ERROR(dwSAL_createSensor(&timeSensor, timeParams, sal));
    CHECK_DW_ERROR(dwSAL_createSensor(&lidarSensorSynced, lidarParams, sal));

    // Create identical lidar without time synchronization for comparison
    lidarParams.parameters = hostString.c_str();
    CHECK_DW_ERROR(dwSAL_createSensor(&lidarSensor, lidarParams, sal));

    // Create identical lidar requesting raw timestamps
    lidarParams.parameters = utcString.c_str();
    CHECK_DW_ERROR(dwSAL_createSensor(&lidarSensorRaw, lidarParams, sal));

    CHECK_DW_ERROR(dwSensor_start(timeSensor));
    CHECK_DW_ERROR(dwSensor_start(lidarSensorSynced));

    CHECK_DW_ERROR(dwSensor_start(lidarSensor));

    CHECK_DW_ERROR(dwSensor_start(lidarSensorRaw));

    dwTime_t hostTimestampSynced = 0;
    dwTime_t hostTimestamp       = 0;
    dwTime_t totalDeltaSynced    = 0;
    dwTime_t totalDeltaOrig      = 0;

    uint64_t count = 0;

    while (gRun)
    {
        auto prev = hostTimestampSynced;
        getLidarTimestamp(&hostTimestampSynced, lidarSensorSynced);
        if (prev)
        {
            totalDeltaSynced += hostTimestampSynced - prev;
        }

        prev = hostTimestamp;
        getLidarTimestamp(&hostTimestamp, lidarSensor);
        if (prev)
        {
            totalDeltaOrig += hostTimestamp - prev;
        }

        dwTime_t rawTimestamp{};
        getLidarTimestamp(&rawTimestamp, lidarSensorRaw);

        count++;

        std::cout << "[Lidar] timestamp synced: " << hostTimestampSynced << " | original: " << hostTimestamp << " | raw: " << rawTimestamp << std::endl;
    }

    std::cout << "Average timestamp delta synced: " << totalDeltaSynced / count
              << " | original: " << totalDeltaOrig / count << std::endl;

    dwSensor_stop(timeSensor);
    dwSensor_stop(lidarSensor);
    dwSensor_stop(lidarSensorSynced);

    dwSAL_releaseSensor(lidarSensor);
    dwSAL_releaseSensor(timeSensor);
    dwSAL_releaseSensor(lidarSensorSynced);
    dwSAL_releaseSensor(lidarSensorRaw);

    dwSAL_release(sal);
    dwRelease(sdk);
    return 0;
}
