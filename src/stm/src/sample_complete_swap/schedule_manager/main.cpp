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
// SPDX-FileCopyrightText: Copyright (c) 2018-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <stm_manager.h>
#include <memory>
#include <unistd.h>
#include <signal.h> // sigaction(), SIG*
#include <stdexcept>
#include <regex>
#include <unordered_map>
#include <chrono>
#include <thread>

#include <stm_manager.h>
#include "manager.hpp"

constexpr int defaultTimeMs   = 1000;
constexpr int defaultCycleNum = 10;
constexpr int defaultRunNum   = 10000;

static void clientTerminate(int signum)
{
    (void)signum;
    stmScheduleManagerExit();
}

int main(int argc, const char** argv)
{
    using namespace stm::manager;

    bool verbose = false;

    std::string schedManName{"schedule_manager"};
    std::int32_t schedDiscriminator = -1;
    std::uint64_t execPeriodMs      = defaultTimeMs;
    std::uint64_t execGapMs         = defaultTimeMs;
    std::uint64_t execRunCount      = defaultRunNum;
    std::uint64_t execCycleCount    = defaultCycleNum;
    bool exceptionAllowed           = true;
    // List of schedules, to be repeated in a cycle
    std::vector<std::uint16_t> inputScheduleIdList{101, 102};

    ArgParse parser("This is sample schedule manager tool for STM. This is used for STM testing");
    parser.addMandatoryArgument("schedule-manager-name", ArgParse::Actions::copyToString(schedManName),
                                "The is name of schedule manage which needs to connect to stm master.");
    parser.addOptionalArgument("--discriminator", ArgParse::Actions::copyToInt(schedDiscriminator),
                               "Specify which instance of stm is this. Defaults is -1.", "-D");
    parser.addOptionalArgument("--verbose", ArgParse::Actions::enableFlag(verbose),
                               "Enable runtime debug printing. Note that this will affect the accuracy of runnable timing.", "-v", true);
    parser.addOptionalArgument("--period", ArgParse::Actions::copyToInt(execPeriodMs),
                               "Specify the period (ms) for which schedules are executed. Default is 1000", "-p");
    parser.addOptionalArgument("--gap", ArgParse::Actions::copyToInt(execGapMs),
                               "Specify the break/delta (ms) between end of previous exec and start of next exec. Default is 1000", "-g");
    parser.addOptionalArgument("--run-count", ArgParse::Actions::copyToInt(execRunCount),
                               "Number of schedule to run before restarting the cycle. Default is 10000", "-r");
    parser.addOptionalArgument("--cycle-count", ArgParse::Actions::copyToInt(execCycleCount),
                               "Number of times the execution cycle is to be repeated. Default is 10", "-c");
    parser.addOptionalArgument("--execution-exception-allow", ArgParse::Actions::enableFlag(exceptionAllowed),
                               "allow exception during stopping and starting the schedule.", "-x", true);

    parser.parse(argc, argv);

    // Set up signal handlers
    struct sigaction terminateHandler;
    terminateHandler.sa_handler = clientTerminate;
    sigaction(SIGTERM, &terminateHandler, NULL);
    sigaction(SIGINT, &terminateHandler, NULL);

    Manager schedule_manager(inputScheduleIdList, schedManName, schedDiscriminator, verbose, execPeriodMs, execGapMs, execRunCount, execCycleCount, exceptionAllowed);

    schedule_manager.process();
}
