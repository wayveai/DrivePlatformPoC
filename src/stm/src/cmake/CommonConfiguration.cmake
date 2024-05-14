# /////////////////////////////////////////////////////////////////////////////////////////
#
# Notice
# ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
# NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
# THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
# MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
#
# NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
# information or for any infringement of patents or other rights of third parties that may
# result from its use. No license is granted by implication or otherwise under any patent
# or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
# expressly authorized by NVIDIA. Details are subject to change without notice.
# This code supersedes and replaces all information previously supplied.
# NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
# components in life support devices or systems without express written approval of
# NVIDIA CORPORATION & AFFILIATES.
#
# SPDX-FileCopyrightText: Copyright (c) 2018-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
# /////////////////////////////////////////////////////////////////////////////////////////

#-------------------------------------------------------------------------------
# Debug symbols
#-------------------------------------------------------------------------------
# Enable minimal (level 1) debug info on experimental builds for
# informative stack trace including function names
if(STM_BUILD_EXPERIMENTAL AND NOT CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g1")
endif()

#-------------------------------------------------------------------------------
# Enable C++11
#-------------------------------------------------------------------------------
if(CMAKE_VERSION VERSION_GREATER 3.1)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
else()
    if(LINUX OR VIBRANTE)
        include(CheckCXXCompilerFlag)
        CHECK_CXX_COMPILER_FLAG(-std=c++14 COMPILER_SUPPORTS_CXX14)
        if(COMPILER_SUPPORTS_CXX14)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
        else()
            message(ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++14 support")
        endif()
    endif()
endif()

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
find_package(Threads REQUIRED)
message("ALL CXX FLAGS: ${CMAKE_CXX_FLAGS}")

#-------------------------------------------------------------------------------
# Find Nvidia driver version
#-------------------------------------------------------------------------------
# Determine currently installed linux driver major version hint
set(nvidia-driver-version_FOUND FALSE)
set(nvidia-driver-version "current")
if (EXISTS /proc/driver/nvidia/version)
    file(READ /proc/driver/nvidia/version nvidia-driver-version-file)
    if(${nvidia-driver-version-file} MATCHES "NVIDIA UNIX.*Kernel Module  ([0123456789]+)\\.[0123456789]+")
        set(nvidia-driver-version_FOUND TRUE)
        set(nvidia-driver-version ${CMAKE_MATCH_1})
    endif()
endif()

