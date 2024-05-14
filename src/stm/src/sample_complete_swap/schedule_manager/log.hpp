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

#ifndef STM_TESTS_DETAILS_LOG_H
#define STM_TESTS_DETAILS_LOG_H

#include <iostream>
#include <mutex>
#include <utility>

#include "singleton.hpp"

namespace stm
{
namespace manager
{
class TransactionalLogger
{
    std::mutex m_mutex;
    std::string m_logPrefix;

    template <typename T>
    void logInternal(std::ostream& ostream, T&& arg)
    {
        ostream << std::forward<T>(arg);
    }

    template <typename T, typename... Ts>
    void logInternal(std::ostream& ostream, T&& arg, Ts&&... rest)
    {
        ostream << std::forward<T>(arg);
        logInternal(ostream, std::forward<Ts>(rest)...);
    }

public:
    TransactionalLogger(std::string logPrefix)
        : m_mutex()
        , m_logPrefix(std::move(logPrefix))
    {
    }

    virtual ~TransactionalLogger() = default;

    template <typename... Ts>
    void logToStream(std::ostream& ostream, Ts&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        ostream << m_logPrefix;
        logInternal(ostream, std::forward<Ts>(args)...);
        ostream << std::endl;
    }
};

class LevelledLogger : public TransactionalLogger
{
    bool m_verbose;

public:
    LevelledLogger(std::string logPrefix)
        : TransactionalLogger(std::move(logPrefix))
        , m_verbose(false)
    {
    }

    void setVerbosity(bool verbose)
    {
        m_verbose = verbose;
    }

    template <typename... Ts>
    void log(Ts&&... args)
    {
        logToStream(std::cout, std::forward<Ts>(args)...);
    }

    template <typename... Ts>
    void logVerbose(Ts&&... args)
    {
        if (m_verbose)
            log(std::forward<Ts>(args)...);
    }

    template <typename... Ts>
    void logError(Ts&&... args)
    {
        logToStream(std::cerr, std::forward<Ts>(args)...);
    }
};

class ManagerLogger : public LevelledLogger
{
public:
    ManagerLogger()
        : LevelledLogger("[MANAGER] ")
    {
    }
};

template <typename... Ts>
void log(Ts&&... args)
{
    Singleton<ManagerLogger>::get().log(std::forward<Ts>(args)...);
}

template <typename... Ts>
void logVerbose(Ts&&... args)
{
    Singleton<ManagerLogger>::get().logVerbose(std::forward<Ts>(args)...);
}

template <typename... Ts>
void logError(Ts&&... args)
{
    Singleton<ManagerLogger>::get().logError(std::forward<Ts>(args)...);
}

inline void setVerbosity(bool verbose)
{
    Singleton<ManagerLogger>::get().setVerbosity(verbose);
}
}
}

#endif //STM_TESTS_DETAILS_LOG_H
