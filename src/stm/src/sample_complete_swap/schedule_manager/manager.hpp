/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
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
#ifndef STM_TESTS_MANAGER_H
#define STM_TESTS_MANAGER_H

#include <stm_manager.h>
#include <memory>
#include <iostream>
#include <unistd.h>
#include <signal.h> // sigaction(), SIG*
#include <stdexcept>
#include <regex>
#include <unordered_map>
#include <chrono>
#include <thread>

#include "argparse.hpp"
#include "log.hpp"

namespace stm
{
namespace manager
{
class Manager
{
private:
    std::string m_schedManName;
    std::int32_t m_schedDiscriminator;
    std::vector<std::uint16_t> m_scheduleIdList;
    std::uint64_t m_execPeriod;
    std::uint64_t m_execGap;
    std::uint64_t m_execRunCount;
    std::uint64_t m_execCycleCount;
    bool m_executeExceptionAllow;

public:
    Manager(
        std::vector<std::uint16_t> inputScheduleIdList,
        std::string schedManName,
        std::int32_t schedDiscriminator,
        bool verbose,
        std::uint64_t execPeriod,
        std::uint64_t execGap,
        std::uint64_t execRunCount,
        std::uint64_t execCycleCount,
        bool executeExceptionFlag);

    ~Manager() = default;

    // Copying risks messing with the run graph. Forcing explicit instantiation of each node.
    Manager(const Manager&) = delete;            // Copy constructor.
    Manager& operator=(const Manager&) = delete; // Copy assignment.

    // Class can only be instantiated through a shared pointer, no need to move.
    Manager(Manager&&) = delete;            // Move constructor.
    Manager& operator=(Manager&&) = delete; // Move Assignment

    void setup();
    void execute();
    void teardown();
    void process();
};
}
}

#endif //STM_TESTS_MANAGER_H
