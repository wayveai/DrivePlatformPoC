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
#include <memory>
#include <cstring>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>

// framework
#include <framework/ProgramArguments.hpp>
#include <framework/SamplesDataPath.hpp>
#include <framework/Log.hpp>
#include <framework/Checks.hpp>

// CORE
#include <dw/core/logger/Logger.h>
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>

// simple plugin-based interpreter
#include "interpreter.hpp"

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool gRun = true;

//------------------------------------------------------------------------------
void sig_int_handler(int /*sig*/)
{
    gRun = false;
}

//------------------------------------------------------------------------------
void printAllSignalValues(dwCANInterpreterHandle_t canParser)
{
    dwStatus status;
    uint32_t num;
    status = dwCANInterpreter_getNumberSignals(&num, canParser);

    if (status == DW_SUCCESS && num > 0)
    {
        float32_t value    = 0;
        dwTime_t timestamp = 0;
        const char* name;

        for (uint32_t i = 0; i < num; ++i)
        {
            if (dwCANInterpreter_getSignalName(&name, i, canParser) == DW_SUCCESS)
            {
                if (dwCANInterpreter_getf32(&value, &timestamp, i, canParser) == DW_SUCCESS)
                {
                    if (0 == strcmp(name, CAN_CAR_SPEED.c_str()))
                        std::cout << " Car speed " << value << " m/s at [" << timestamp << "]";
                    else if (0 == strcmp(name, CAN_CAR_STEERING.c_str()))
                        std::cout << " Car steering " << value << " rad at [" << timestamp << "]";
                    else
                        std::cout << name << ":" << value << (i < num - 1 ? ", " : "") << std::endl;
                }
            }
        }
    }
}

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // install signal handler to react on ctrl+c
    {
        struct sigaction action = {};
        action.sa_handler       = sig_int_handler;

        sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
        sigaction(SIGINT, &action, NULL);  // Ctrl-C
        sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
        sigaction(SIGABRT, &action, NULL); // abort() called.
        sigaction(SIGTERM, &action, NULL); // kill command
    }

    ProgramArguments arguments(
        {
            ProgramArguments::Option_t("driver", "can.virtual"),
            ProgramArguments::Option_t("params", (std::string("file=") + dw_samples::SamplesDataPath::get() + "/samples/sensors/can/canbus_dbc.can").c_str()),
            ProgramArguments::Option_t("dbc", (dw_samples::SamplesDataPath::get() + "/samples/sensors/can/sample.dbc").c_str()),
            ProgramArguments::Option_t("csv", "false"),
        });

    if (!arguments.parse(argc, argv))
    {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "\t--driver=can.virtual \t\t\t: one of the available CAN drivers "
                  << "(see sample_sensors_info)\n";
        std::cout << "\t--params=file=canbus_dbc.can\t: comma separated "
                  << "key=value parameters for the sensor "
                  << "(see sample_sensor_info for a set of supported parameters)\n";
        std::cout << "\t--dbc=sample.dbc\t\t: input dbc file. (set to plugin, to run plugin-based interpreter)\n";
        std::cout << "\t--csv=false\t\t: specify if each CAN signal and its TimeStamp should be saved in CSV files"
                  << std::endl;

        return -1;
    }

    std::cout << "Program Arguments:\n"
              << arguments.printList() << std::endl;

    bool CSVOutput = false;
    if (arguments.has("csv") && arguments.get("csv") == "true")
    {
        std::cout << "Save interpretted CAN signals in CSV format" << std::endl;
        CSVOutput = true;
    }

    dwContextHandle_t sdk = DW_NULL_HANDLE;
    dwSALHandle_t hal     = DW_NULL_HANDLE;
    {
        // create a Logger to log to console
        // we keep the ownership of the logger at the application level
        dwLogger_initialize(getConsoleLoggerCallback(true));
        dwLogger_setLogLevel(DW_LOG_VERBOSE);

        // instantiate Driveworks SDK context
        dwContextParameters sdkParams = {};

        CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &sdkParams));

        // create HAL module of the SDK
        dwSAL_initialize(&hal, sdk);
    }

    // create CAN bus interface
    dwSensorHandle_t canSensor = DW_NULL_HANDLE;
    {
        dwSensorParams params{};
        params.parameters = arguments.get("params").c_str();
        params.protocol   = arguments.get("driver").c_str();
        if (dwSAL_createSensor(&canSensor, params, hal) != DW_SUCCESS)
        {
            std::cout << "Cannot create sensor "
                      << params.protocol << " with " << params.parameters << std::endl;

            dwSAL_release(hal);
            dwRelease(sdk);
            dwLogger_release();

            return -1;
        }
    }

    dwCANInterpreterHandle_t canParser = DW_NULL_HANDLE;

    // if interpreter is provided, create an instance of it
    if (arguments.has("dbc") && arguments.get("dbc") != "plugin")
    {
        std::cout << "Create DBC-based CAN message interpreter" << std::endl;

        std::string inputFilePath = arguments.get("dbc");

        dwStatus result = dwCANInterpreter_buildFromDBC(&canParser, inputFilePath.c_str(), sdk);
        if (result != DW_SUCCESS)
        {
            std::cout << "Cannot create DBC-based CAN message interpreter" << std::endl;
        }
    }
    // if not create a plugin based one
    else
    {
        std::cout << "Create Plugin-based CAN message interpreter" << std::endl;

        dwCANInterpreterInterface interpreter;

        interpreter.addMessage             = smpl_addMessage;
        interpreter.getDataf32             = smpl_getDataf32;
        interpreter.getDatai32             = smpl_getDatai32;
        interpreter.getNumAvailableSignals = smpl_getNumAvailableSignals;
        interpreter.getSignalInfo          = smpl_getSignalInfo;

        std::cout << "Create simple default CAN message interpreter" << std::endl;
        dwStatus result = dwCANInterpreter_buildFromCallbacks(&canParser, interpreter, NULL, sdk);
        if (result != DW_SUCCESS)
        {
            std::cout << "Cannot create callback based CAN message interpreter" << std::endl;
        }
    }

    gRun = dwSensor_start(canSensor) == DW_SUCCESS;

    std::map<const char*, int> mymap;

    std::ofstream myfile[200]; // fo creating the CSV file
    int dbcCounter = 0;
    // receive messages
    while (gRun)
    {
        std::this_thread::yield();

        dwCANMessage msg;
        dwStatus status = dwSensorCAN_readMessage(&msg, 100000, canSensor);

        if (status == DW_TIME_OUT)
        {
            continue;
        }
        if (status == DW_END_OF_STREAM)
        {
            std::cout << "EndOfStream" << std::endl;
            break;
        }

        // pass message to interpreter
        if (status == DW_SUCCESS && canParser)
        {
            dwCANInterpreter_consume(&msg, canParser);
        }

        // log message
        std::cout << msg.timestamp_us;
        if (status != DW_SUCCESS)
        {
            std::cout << " ERROR " << dwGetStatusName(status);
        }
        else
        {
            std::cout << " [0x" << std::setfill('0') << std::setw(3) << std::right << std::hex << msg.id << "] -> ";
            for (auto i = 0; i < msg.size; i++)
            {
                if (i % 8 == 0)
                    std::cout << std::endl;
                std::cout << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << (int)msg.data[i] << " ";
            }
            std::cout << std::endl;
            std::cout << std::dec;

            // use parser to get meaningful information from the data
            if (canParser)
            {
                printAllSignalValues(canParser);

                dwStatus status;
                uint32_t num;
                status = dwCANInterpreter_getNumberSignals(&num, canParser);
                if (CSVOutput)
                {
                    if (status == DW_SUCCESS && num > 0)
                    {
                        float32_t value    = 0;
                        dwTime_t timestamp = 0;
                        const char* name;

                        for (uint32_t i = 0; i < num; ++i)
                        {
                            if (dwCANInterpreter_getSignalName(&name, i, canParser) == DW_SUCCESS && CSVOutput)
                            {
                                if (mymap.count(name) == 0)
                                {
                                    const char* two = ".csv";
                                    char result[100];     // array to hold the result.
                                    strcpy(result, name); // copy string one into the result.
                                    strcat(result, two);  // append string two to the result.
                                    mymap[name] = dbcCounter;
                                    myfile[dbcCounter].open(result);
                                    myfile[dbcCounter] << "timestamp," << name << "\n";

                                    dbcCounter++;
                                }
                                else
                                {
                                    if (dwCANInterpreter_getf32(&value, &timestamp, i, canParser) == DW_SUCCESS)
                                    {
                                        myfile[mymap.find(name)->second] << timestamp << ',' << value << "\n";
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        std::cout << std::endl;
    }

    if (canParser != DW_NULL_HANDLE)
        dwCANInterpreter_release(canParser);

    dwSensor_stop(canSensor);
    dwSAL_releaseSensor(canSensor);

    // release used objects in correct order
    dwSAL_release(hal);
    dwRelease(sdk);
    dwLogger_release();

    return 0;
}
