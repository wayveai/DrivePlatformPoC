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
// SPDX-FileCopyrightText: Copyright (c) 2016-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>
#include <dw/rig/Rig.h>
#include <framework/ProgramArguments.hpp>
#include <framework/SamplesDataPath.hpp>

#include <iostream>
#include <string>

#define PRINT_MEMBER(data, member) printf(" " #member ": \t%f\n", data.member)

dwContextHandle_t g_context = DW_NULL_HANDLE;
dwRigHandle_t g_rig         = DW_NULL_HANDLE;

// -----------------------------------------------------------------------------------------------------------
// Initialize SDK and load rig.json configuration file
// -----------------------------------------------------------------------------------------------------------
void initialize(const char* rigconfigFilename)
{
    dwContextParameters contextParams{};

    // Initialize driveworks sdk
    dwStatus status = dwInitialize(&g_context, DW_VERSION, &contextParams);
    if (status != DW_SUCCESS)
    {
        printf("Error dwInitialize: %s\n", dwGetStatusName(status));
        exit(-1);
    }

    status = dwRig_initializeFromFile(&g_rig, g_context, rigconfigFilename);
    if (status != DW_SUCCESS)
    {
        printf("Error dwEgomotion_initialize: %s\n", dwGetStatusName(status));
        exit(-1);
    }
}

// -----------------------------------------------------------------------------------------------------------
// Print vehilce information found in the rig file
// -----------------------------------------------------------------------------------------------------------
void printVehicle()
{
    dwGenericVehicle vehicle;

    dwStatus status = dwRig_getGenericVehicle(&vehicle, g_rig);
    if (status != DW_SUCCESS)
    {
        printf("No vehicle information available.\n");
        return;
    }

    printf("Vehicle Information:\n");
    PRINT_MEMBER(vehicle, body.width);
    PRINT_MEMBER(vehicle, body.height);
    PRINT_MEMBER(vehicle, body.length);

    printf("Has cabin: %s", vehicle.hasCabin ? "true" : "false");
    printf("Trailers: %d", vehicle.numTrailers);
}

// -----------------------------------------------------------------------------------------------------------
// Print all sensor information found in the rig file
// -----------------------------------------------------------------------------------------------------------
void printSensors()
{
    uint32_t sensorCount;
    dwStatus status;

    dwRig_getSensorCount(&sensorCount, g_rig);
    printf("Sensor Count: %d\n", sensorCount);

    for (uint32_t i = 0; i < sensorCount; ++i)
    {
        printf(" Sensor %d:\n", i);

        const char* name;
        status = dwRig_getSensorName(&name, i, g_rig);
        if (status == DW_SUCCESS)
            printf("  Name:\t\t%s\n", name);

        const char* protocol;
        status = dwRig_getSensorProtocol(&protocol, i, g_rig);
        if (status == DW_SUCCESS)
            printf("  Protocol:\t%s\n", protocol);

        const char* dataFile = nullptr;
        status               = dwRig_getSensorDataPath(&dataFile, i, g_rig);
        if (status == DW_SUCCESS)
            printf("  File:\t%s\n", dataFile);

        const char* timestampFile = nullptr;
        status                    = dwRig_getCameraTimestampPath(&timestampFile, i, g_rig);
        if (status == DW_SUCCESS && timestampFile)
            printf("  Timestamp:\t%s\n", timestampFile);
    }
}

// -----------------------------------------------------------------------------------------------------------
// Save rig file
// -----------------------------------------------------------------------------------------------------------
void saveRig(const char* rigconfigFilename)
{
    if (g_rig)
    {
        dwRig_serializeToFile(rigconfigFilename, g_rig);
    }
}

// -----------------------------------------------------------------------------------------------------------
// Release all used memory
// -----------------------------------------------------------------------------------------------------------
void release()
{
    if (g_rig)
    {
        dwRig_release(g_rig);
    }
    if (g_context)
    {
        dwRelease(g_context);
    }
}
// -----------------------------------------------------------------------------------------------------------
int main(int argc, const char* argv[])
{
    ProgramArguments arguments(
        {
            ProgramArguments::Option_t("rigconfig", (dw_samples::SamplesDataPath::get() + std::string{"/samples/sfm/triangulation/rig.json"}).c_str()),
            ProgramArguments::Option_t("outputrigconfig", "", "specify the output rig file name if wanted."),
        });

    if (!arguments.parse(argc, argv))
        return -1; // Exit if not all require arguments are provided

    std::cout << "Program Arguments:\n"
              << arguments.printList() << std::endl;

    initialize(arguments.get("rigconfig").c_str());
    printVehicle();
    printSensors();
    if (!arguments.get("outputrigconfig").empty())
    {
        saveRig(arguments.get("outputrigconfig").c_str());
    }
    release();

    return 0;
}
