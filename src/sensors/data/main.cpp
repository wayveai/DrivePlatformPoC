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
#include <csignal>

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

int main(int argc, const char** argv)
{
    ProgramArguments arguments(
        {ProgramArguments::Option_t("driver", "data.virtual"),
         ProgramArguments::Option_t("params", (std::string("file=") + dw_samples::SamplesDataPath::get() + "/samples/sensors/data/data_socket.bin").c_str())});

    if (!arguments.parse(argc, argv))
    {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "\t--driver=data.virtual \t\t\t: one of the available GPS drivers "
                  << "(see sample_sensors_info)\n";
        std::cout << "\t--params=file=data_socket.bin,arg2=value \t: comma separated "
                  << "key=value parameters for the sensor "
                  << "(see sample_sensor_info for a set of supported parameters)\n";
        return -1;
    }

    std::cout << "Program Arguments:\n"
              << arguments.printList() << std::endl;

    std::signal(SIGINT, sig_int_handler);

    dwContextHandle_t sdk      = DW_NULL_HANDLE;
    dwSALHandle_t hal          = DW_NULL_HANDLE;
    dwSensorManagerHandle_t sm = DW_NULL_HANDLE;

    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
    CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

    CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &sdkParams));

    // create HAL module of the SDK
    CHECK_DW_ERROR(dwSAL_initialize(&hal, sdk));

    CHECK_DW_ERROR(dwSensorManager_initialize(&sm, 16, hal));

    dwSensorParams params{};
    std::string parameterString = arguments.get("params");
    params.parameters           = parameterString.c_str();
    params.protocol             = arguments.get("driver").c_str();

    // start data sensor
    CHECK_DW_ERROR(dwSensorManager_addSensor(params, 0, sm));
    CHECK_DW_ERROR(dwSensorManager_start(sm));

    while (gRun)
    {
        const dwSensorEvent* ev{};
        dwStatus status = dwSensorManager_acquireNextEvent(&ev, 1000000, sm);

        if (status == DW_END_OF_STREAM)
        {
            std::cout << "Data sensor end of stream reached" << std::endl;
            break;
        }
        else if (status == DW_TIME_OUT || status == DW_NOT_READY)
            continue;
        else if (status == DW_SUCCESS && ev->type == DW_SENSOR_DATA)
        {
            const dwDataPacket* frame = ev->dataFrame;
            std::cout << "[" << frame->hostTimestamp << "] received frame of size: " << frame->size << std::endl;
        }
        else
        {
            std::cerr << " ERROR when reading data packet" << dwGetStatusName(status);
            break;
        }

        dwSensorManager_releaseAcquiredEvent(ev, sm);
    }

    // release objects in correct order
    dwSensorManager_release(sm);
    dwSAL_release(hal);
    dwRelease(sdk);
    dwLogger_release();

    return 0;
}
