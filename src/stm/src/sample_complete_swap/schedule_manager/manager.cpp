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

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <stdexcept>
#include <regex>
#include <unordered_map>
#include <atomic>
#include <pthread.h>

#include "manager.hpp"

namespace stm
{
namespace manager
{
Manager::Manager(
    std::vector<std::uint16_t> inputScheduleIdList,
    std::string schedManName,
    std::int32_t schedDiscriminator,
    bool verbose,
    std::uint64_t execPeriod,
    std::uint64_t execGap,
    std::uint64_t execRunCount,
    std::uint64_t execCycleCount,
    bool executeExceptionFlag)
    : m_schedManName(std::move(schedManName))
    , m_schedDiscriminator(schedDiscriminator)
    , m_scheduleIdList(inputScheduleIdList)
    , m_execPeriod(execPeriod)
    , m_execGap(execGap)
    , m_execRunCount(execRunCount)
    , m_execCycleCount(execCycleCount)
    , m_executeExceptionAllow(executeExceptionFlag)
{
    setVerbosity(verbose);

    logVerbose("scheduleIdList contains: ", m_scheduleIdList.size(), " items.");
    for (auto scheduleId : m_scheduleIdList)
    {
        logVerbose("\t: ", scheduleId);
    }

    setup();
}

void Manager::setup()
{
    logVerbose("Schedule Manager Init : Started");
    if (STM_SUCCESS != stmScheduleManagerInitWithDiscriminator(m_schedManName.c_str(), m_schedDiscriminator))
    {
        throw std::runtime_error("Failed at stmScheduleManagerInitWithDiscriminator");
    }
    else
    {
        logVerbose("Schedule Manager Init : Done");
    }
    return;
}

void Manager::execute()
{
    for (uint64_t i = 0; i < m_execCycleCount; ++i)
    {
        uint64_t currentSchedIndex = 0;
        for (auto it = m_scheduleIdList.begin(); it != m_scheduleIdList.end(); ++it)
        {
            if (currentSchedIndex < m_execRunCount)
            {
                if (STM_SUCCESS != stmStartSchedule(*it))
                {
                    throw std::runtime_error("Failed at stmStartSchedule");
                }
                logVerbose("Start Schedule Execution of Id ", *it);
                std::this_thread::sleep_for(std::chrono::milliseconds(m_execPeriod));

                if (STM_SUCCESS != stmStopSchedule(*it))
                {
                    throw std::runtime_error("Failed at stmStopSchedule");
                }
                logVerbose("Stop Schedule Execution of Id ", *it);
                std::this_thread::sleep_for(std::chrono::milliseconds(m_execGap));
            }
            else
            {
                break;
            }
            currentSchedIndex++;
        }
    }
    return;
}

void Manager::teardown()
{
    logVerbose("Schedule Manager Exit : Started");
    if (STM_SUCCESS != stmScheduleManagerExit())
    {
        throw std::runtime_error("Failed at stmScheduleManagerExit");
    }
    else
    {
        logVerbose("Schedule Manager Exit : Done");
    }
    return;
}

void Manager::process()
{
    try
    {
        execute();
    }
    catch (const std::exception& e)
    {
        if (!m_executeExceptionAllow)
        {
            throw;
        }
        logVerbose("Ignoring error at schedule execution ", e.what());
    }
    teardown();
}
}
}
