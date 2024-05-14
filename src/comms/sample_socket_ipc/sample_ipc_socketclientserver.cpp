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

#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>
#include <dw/comms/socketipc/SocketClientServer.h>

#include <framework/Log.hpp>
#include <framework/Checks.hpp>
#include <framework/ProgramArguments.hpp>

#include <csignal>
#include <iostream>
#include <atomic>
#include <thread>

//------------------------------------------------------------------------------
// Variables of working/debugging status of the program.
//------------------------------------------------------------------------------
static std::atomic<bool> g_run{true};

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------

extern "C" void sig_int_handler(int)
{
    g_run = false;
}

dwStatus runServer(ProgramArguments const& arguments, dwContextHandle_t ctx)
{
    auto port = static_cast<uint16_t>(std::stoul(arguments.get("port")));

    auto socketServer = dwSocketServerHandle_t{DW_NULL_HANDLE};
    CHECK_DW_ERROR(dwSocketServer_initialize(&socketServer, port, 2, ctx));

    // accept two connections (use two connections for illustration,
    // a single connection can also be used bi-directionally)
    auto socketConnectionRead  = dwSocketConnectionHandle_t{DW_NULL_HANDLE};
    auto socketConnectionWrite = dwSocketConnectionHandle_t{DW_NULL_HANDLE};

    auto status = DW_TIME_OUT;
    while (g_run && status == DW_TIME_OUT)
    {
        status = dwSocketServer_accept(&socketConnectionRead, 10000, socketServer);
    }

    status = DW_TIME_OUT;
    while (g_run && status == DW_TIME_OUT)
    {
        status = dwSocketServer_accept(&socketConnectionWrite, 10000, socketServer);
    }

    if (socketConnectionRead && socketConnectionWrite && status != DW_FAILURE)
    {
        while (g_run)
        {
            size_t data;
            auto size = sizeof(decltype(data));

            // receive data
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_read(reinterpret_cast<uint8_t*>(&data), &size, DW_TIMEOUT_INFINITE,
                                                  socketConnectionRead)) == DW_END_OF_STREAM)
            {
                break;
            }
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
            {
                break;
            }

            std::cout << "Socket Server received " << data << std::endl;

            // send data back
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_write(reinterpret_cast<uint8_t*>(&data), &size, DW_TIMEOUT_INFINITE,
                                                   socketConnectionWrite)) == DW_END_OF_STREAM)
            {
                break;
            }
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
            {
                break;
            }

            std::cout << "Socket Server send " << data << std::endl;
        }
    }

    if (socketConnectionWrite)
    {
        CHECK_DW_ERROR(dwSocketConnection_release(socketConnectionWrite));
    }
    if (socketConnectionRead)
    {
        CHECK_DW_ERROR(dwSocketConnection_release(socketConnectionRead));
    }
    CHECK_DW_ERROR(dwSocketServer_release(socketServer));

    return DW_SUCCESS;
}

dwStatus runClient(ProgramArguments const& arguments, dwContextHandle_t ctx)
{
    auto ip   = arguments.get("ip");
    auto port = static_cast<uint16_t>(std::stoul(arguments.get("port")));

    auto socketClient = dwSocketClientHandle_t{DW_NULL_HANDLE};
    CHECK_DW_ERROR(dwSocketClient_initialize(&socketClient, 2, ctx));

    // connect two connections (use two connections for illustration,
    // a single connection can also be used bi-directionally)
    auto socketConnectionWrite = dwSocketConnectionHandle_t{DW_NULL_HANDLE};
    auto socketConnectionRead  = dwSocketConnectionHandle_t{DW_NULL_HANDLE};

    auto status = DW_TIME_OUT;
    while (g_run && status == DW_TIME_OUT)
    {
        status = dwSocketClient_connect(&socketConnectionWrite, ip.c_str(), port, 10000, socketClient);
    }

    status = DW_TIME_OUT;
    while (g_run && status == DW_TIME_OUT)
    {
        status = dwSocketClient_connect(&socketConnectionRead, ip.c_str(), port, 10000, socketClient);
    }

    if (socketConnectionWrite && socketConnectionRead && status != DW_FAILURE)
    {
        while (g_run)
        {
            // send some data
            static size_t dataRef = 0;
            ++dataRef;
            decltype(dataRef) data;
            auto size = sizeof(decltype(data));

            // send data
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_write(reinterpret_cast<uint8_t*>(&dataRef), &size, DW_TIMEOUT_INFINITE,
                                                   socketConnectionWrite)) == DW_END_OF_STREAM)
            {
                break;
            }
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
            {
                break;
            }

            std::cout << "Socket Client send " << dataRef << std::endl;

            // receive data
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_read(reinterpret_cast<uint8_t*>(&data), &size, DW_TIMEOUT_INFINITE,
                                                  socketConnectionRead)) == DW_END_OF_STREAM)
            {
                break;
            }
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
            {
                break;
            }

            std::cout << "Socket Client received " << data << std::endl;
        }
    }

    if (socketConnectionWrite)
    {
        CHECK_DW_ERROR(dwSocketConnection_release(socketConnectionWrite));
    }
    if (socketConnectionRead)
    {
        CHECK_DW_ERROR(dwSocketConnection_release(socketConnectionRead));
    }
    CHECK_DW_ERROR(dwSocketClient_release(socketClient));

    return DW_SUCCESS;
}

int main(int argc, const char** argv)
{
    ProgramArguments arguments({ProgramArguments::Option_t("role", "server", "client or server"),
                                ProgramArguments::Option_t("ip", "127.0.0.1", "The server IP the client connects to"),
                                ProgramArguments::Option_t("port", "49252", "The port the server will listen on / the client will connect to")});

    if (!arguments.parse(argc, argv))
    {
        std::exit(-1); // Exit if not all require arguments are provided
    }
    else
    {
        std::cout << "Program Arguments:\n"
                  << arguments.printList() << std::endl;
    }

    std::signal(SIGHUP, sig_int_handler);  // controlling terminal closed, Ctrl-D
    std::signal(SIGINT, sig_int_handler);  // Ctrl-C
    std::signal(SIGQUIT, sig_int_handler); // Ctrl-\, clean quit with core dump
    std::signal(SIGABRT, sig_int_handler); // abort() called.
    std::signal(SIGTERM, sig_int_handler); // kill command
    std::signal(SIGSTOP, sig_int_handler); // kill command
    g_run = true;

    // Initialize context
    auto ctx = dwContextHandle_t{DW_NULL_HANDLE};
    {
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true, true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));
        CHECK_DW_ERROR(dwInitialize(&ctx, DW_VERSION, nullptr));
    }

    // Run client / server
    auto const role = arguments.get("role");

    auto status = DW_FAILURE;
    if (role == "server")
    {
        status = runServer(arguments, ctx);
    }
    else if (role == "client")
    {
        status = runClient(arguments, ctx);
    }
    else
    {
        std::cerr << "Invalid role parameter '" << role << "' provided (use either 'client' or 'server')"
                  << std::endl;
    }

    CHECK_DW_ERROR(dwRelease(ctx));

    return status == DW_SUCCESS;
}
