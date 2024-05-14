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

#include <framework/Log.hpp>
#include <framework/Checks.hpp>

// CORE
#include <dw/core/context/Context.h>
#include <dw/core/logger/Logger.h>
#include <dw/core/base/Version.h>

// HAL
#include <dw/sensors/Sensors.h>

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    dwContextHandle_t sdk = DW_NULL_HANDLE;
    dwSALHandle_t hal     = DW_NULL_HANDLE;

    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
    CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

    CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &sdkParams));

    // create HAL module of the SDK
    CHECK_DW_ERROR(dwSAL_initialize(&hal, sdk));

    dwPlatformOS currentPlatform;
    dwPlatformOS platform[] = {DW_PLATFORM_OS_LINUX, DW_PLATFORM_OS_V5L, DW_PLATFORM_OS_V5Q};
    CHECK_DW_ERROR(dwSAL_getPlatform(&currentPlatform, hal));

    // get information about available sensors on each platform
    for (size_t i = 0; i < sizeof(platform) / sizeof(dwPlatformOS); i++)
    {
        const char* name = nullptr;
        CHECK_DW_ERROR(dwSAL_getPlatformInfo(&name, platform[i], hal));
        std::cout << "Platform: " << name;
        if (platform[i] == currentPlatform)
            std::cout << " - CURRENT";

        std::cout << ": " << std::endl;

        uint32_t numSensors = 0;
        CHECK_DW_ERROR(dwSAL_getNumSensors(&numSensors, platform[i], hal));
        for (uint32_t j = 0; j < numSensors; j++)
        {
            const char* protocol = "";
            const char* params   = "";
            CHECK_DW_ERROR(dwSAL_getSensorProtocol(&protocol, j, platform[i], hal));
            CHECK_DW_ERROR(dwSAL_getSensorParameterString(&params, j, platform[i], hal));

            std::cout << "   Sensor [" << j << "] : "
                      << protocol << " ? " << params << std::endl;
        }
        if (numSensors == 0)
        {
            std::cout << "   NO SENSORS AVAILABLE" << std::endl;
        }
        std::cout << std::endl;
    }

    // release used objects in correct order
    dwSAL_release(hal);
    dwRelease(sdk);
    dwLogger_release();

    return 0;
}
