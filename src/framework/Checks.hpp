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
// SPDX-FileCopyrightText: Copyright (c) 2017-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_COMMON_CHECKS_HPP_
#define SAMPLES_COMMON_CHECKS_HPP_

#include <iostream>
#include <string>
#include <stdexcept>
#include <time.h>
#include <cuda_runtime.h>

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

inline void getDateString(char* buf, size_t length)
{
    time_t now          = ::time(0);
    struct tm* calendar = localtime(&now);
    strftime(buf, length, "[%Y-%m-%d %X] ", calendar);
}

//------------------------------------------------------------------------------
// macro to easily check for dw errors
#define CHECK_DW_ERROR(x)                                                                                                                                                                                                   \
    {                                                                                                                                                                                                                       \
        dwStatus RESULT{x};                                                                                                                                                                                                 \
        if (RESULT != DW_SUCCESS)                                                                                                                                                                                           \
        {                                                                                                                                                                                                                   \
            char buf[80];                                                                                                                                                                                                   \
            getDateString(buf, 80);                                                                                                                                                                                         \
            throw std::runtime_error(std::string(buf) + std::string("DW Error ") + dwGetStatusName(RESULT) + std::string(" executing DW function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
        }                                                                                                                                                                                                                   \
    };

#define CHECK_DW_ERROR_NOTHROW(x)                                                                                                                                                                                             \
    {                                                                                                                                                                                                                         \
        dwStatus result{x};                                                                                                                                                                                                   \
        if (result != DW_SUCCESS)                                                                                                                                                                                             \
        {                                                                                                                                                                                                                     \
            char buf[80];                                                                                                                                                                                                     \
            getDateString(buf, 80);                                                                                                                                                                                           \
            std::cerr << (std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) + std::string(" executing DW function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)) << std::endl; \
        }                                                                                                                                                                                                                     \
    };

#define CHECK_DW_ERROR_AND_RETURN(x)                                                                                                                                                                                          \
    {                                                                                                                                                                                                                         \
        dwStatus result{x};                                                                                                                                                                                                   \
        if (result != DW_SUCCESS)                                                                                                                                                                                             \
        {                                                                                                                                                                                                                     \
            char buf[80];                                                                                                                                                                                                     \
            getDateString(buf, 80);                                                                                                                                                                                           \
            std::cerr << (std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) + std::string(" executing DW function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)) << std::endl; \
            return result;                                                                                                                                                                                                    \
        }                                                                                                                                                                                                                     \
    };

#define CHECK_DW_ERROR_MSG(x, description)                                                                                                                                                                                                                      \
    {                                                                                                                                                                                                                                                           \
        dwStatus result{x};                                                                                                                                                                                                                                     \
        if (result != DW_SUCCESS)                                                                                                                                                                                                                               \
        {                                                                                                                                                                                                                                                       \
            char buf[80];                                                                                                                                                                                                                                       \
            getDateString(buf, 80);                                                                                                                                                                                                                             \
            throw std::runtime_error(std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) + std::string(" executing DW function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__) + std::string(" -> ") + description); \
        }                                                                                                                                                                                                                                                       \
    };

#define CHECK_DW_ERROR_MSG_NOTHROW(x, description)                                                \
    {                                                                                             \
        dwStatus result{x};                                                                       \
        if (result != DW_SUCCESS)                                                                 \
        {                                                                                         \
            char buf[80];                                                                         \
            getDateString(buf, 80);                                                               \
            std::cerr << (std::string(buf) + std::string("DW Error ") + dwGetStatusName(result) + \
                          std::string(" executing DW function:\n " #x) +                          \
                          std::string("\n at " __FILE__ ":") + std::to_string(__LINE__) +         \
                          std::string(" -> ") + description)                                      \
                      << std::endl;                                                               \
        }                                                                                         \
    };

//------------------------------------------------------------------------------
// macro to easily check for cuda errors
#define CHECK_CUDA_ERROR(x)                                                                                                                                                                                                        \
    {                                                                                                                                                                                                                              \
        x;                                                                                                                                                                                                                         \
        auto result = cudaGetLastError();                                                                                                                                                                                          \
        if (result != cudaSuccess)                                                                                                                                                                                                 \
        {                                                                                                                                                                                                                          \
            char buf[80];                                                                                                                                                                                                          \
            getDateString(buf, 80);                                                                                                                                                                                                \
            throw std::runtime_error(std::string(buf) + std::string("CUDA Error ") + cudaGetErrorString(result) + std::string(" executing CUDA function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
        }                                                                                                                                                                                                                          \
    };

#endif // SAMPLES_COMMON_CHECKS_HPP_
