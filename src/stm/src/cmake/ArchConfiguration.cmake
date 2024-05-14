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
# Platform selection
#-------------------------------------------------------------------------------

if(VIBRANTE)
    message(STATUS "Cross Compiling for Vibrante")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(LINUX TRUE)
        add_definitions(-DLINUX)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        message(FATAL_ERROR "Direct compilation not supported for ${CMAKE_SYSTEM_PROCESSOR}, use cross compilation.")
    else()
        message(FATAL_ERROR "Unsupported Linux CPU architecture ${CMAKE_SYSTEM_PROCESSOR}.")
    endif()
else()
    message(FATAL_ERROR "Cannot identify OS")
endif()

# Position independent code enforce for 64bit systems and armv7l
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
   set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

#-------------------------------------------------------------------------------
# Architecture selection
#-------------------------------------------------------------------------------

if(VIBRANTE)
    # Select device architecture
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        if(VIBRANTE_V5Q)
            set(ARCH_DIR "qnx-aarch64")
        else()
            set(ARCH_DIR "linux-aarch64")
        endif()
    else()
        set(ARCH_DIR "linux-armv7l")
    endif()
    set(C_ARCH_DIR ${ARCH_DIR})
    set(CPP_ARCH_DIR ${ARCH_DIR})
else()
    # Linux
    set(C_ARCH_DIR "linux-x86/abiC")
    set(CPP_ARCH_DIR "linux-x86/abi98")
endif()

# Dependencies that are C++ abi dependent are stored under STM_CPP_ARCH_DIR
unset(STM_CPP_ARCH_DIR CACHE)
set(STM_CPP_ARCH_DIR ${CPP_ARCH_DIR} CACHE INTERNAL "")

# Dependencies that are C only and don't care about abi are stored under STM_C_ARCH_DIR
unset(STM_C_ARCH_DIR CACHE)
set(STM_C_ARCH_DIR ${C_ARCH_DIR} CACHE INTERNAL "")


