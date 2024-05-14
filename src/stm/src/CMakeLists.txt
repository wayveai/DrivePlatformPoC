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
# SPDX-FileCopyrightText: Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


#-------------------------------------------------------------------------------
# Options
#-------------------------------------------------------------------------------
option(STM_BASE_DIR "STM_BASE_DIR" OFF)


#-------------------------------------------------------------------------------
# Set basic configurations
#-------------------------------------------------------------------------------
include(CrossCompiling)
include(SetBuildType) # Set the build mtype before project is created


#-------------------------------------------------------------------------------
# Root project
#-------------------------------------------------------------------------------
project(STM-Tests C CXX)


#-------------------------------------------------------------------------------
# Basic STM configuration
#-------------------------------------------------------------------------------
include(ArchConfiguration)
include(CommonConfiguration)
include(RuntimeConfiguration)
include(MakeCUDASourceLibs)


#-------------------------------------------------------------------------------
# CUDA dependency
# Note: needs to be after defining all configurations
#-------------------------------------------------------------------------------
find_package(CUDA)
if(VIBRANTE)
    list(APPEND CUDA_NVCC_FLAGS -lineinfo --gpu-architecture=compute_53)
    list(APPEND CUDA_NVCC_FLAGS --gpu-code=sm_60,sm_62,sm_72,compute_53)
else()
    list(APPEND CUDA_NVCC_FLAGS -lineinfo --gpu-architecture=compute_53)
    if(NOT CUDA_HOST_GPU_CODE_FLAGS)
        list(APPEND CUDA_NVCC_FLAGS --gpu-code=compute_53)
    else()
        # Compile for local host arch only
        list(APPEND CUDA_NVCC_FLAGS ${CUDA_HOST_GPU_CODE_FLAGS})
    endif()
endif()
set(CUDA_NON_PROPAGATED_HOST_FLAGS "-Wold-style-cast") # disable certain CUDA flags to be not propagated
list(APPEND CUDA_NVCC_FLAGS -std=c++11)
include_directories(SYSTEM ${CUDA_TOOLKIT_INCLUDE})

#-------------------------------------------------------------------------------
# Setup 3rdparty dependencies
#-------------------------------------------------------------------------------
set(3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/" CACHE PATH "Path to directory containing installed 3rdparty libraries")
set(3RDPARTY_C_DIR "${3RDPARTY_DIR}/${STM_C_ARCH_DIR}" CACHE PATH "")
set(3RDPARTY_CPP_DIR "${3RDPARTY_DIR}/${STM_C_ARCH_DIR}" CACHE PATH "")
include(3rdparty)


#-------------------------------------------------------------------------------
# Setup DriveOS dependencies. VIBRANTE dependencies set in toolchain
#-------------------------------------------------------------------------------
if(NOT VIBRANTE)
    message(STATUS "Building for x86")
    include_directories("${3RDPARTY_DIR}/driveos/x86")
endif()

#-------------------------------------------------------------------------------
# Setup STM dependencies
#-------------------------------------------------------------------------------
include_directories(${STM_BASE_DIR}/include)
find_library(STM_RUNTIME "stm_runtime" HINTS ${STM_BASE_DIR}/lib NO_CMAKE_FIND_ROOT_PATH)
if(NOT STM_RUNTIME)
    message(FATAL_ERROR "STM library not found.")
endif()

set(STM_DEPENDENT_LIBS
    ${STM_RUNTIME}
    ${CUDA_CUDA_LIBRARY}
)
if(VIBRANTE)
    set(STM_DEPENDENT_LIBS ${STM_DEPENDENT_LIBS} vibrante)
if (NOT (${VIBRANTE_PDK_DECIMAL} LESS 5010900))
if (CUPTI_DEBUG)
    set(STM_DEPENDENT_LIBS ${STM_DEPENDENT_LIBS} ${CUDA_cupti_LIBRARY})
endif()
endif()

if ((${VIBRANTE_PDK_DECIMAL} GREATER_EQUAL 6000400))
    include_directories(${STM_BASE_DIR}/include/cupva)
    find_library(CUPVA_HOST_LIB "cupva_host" HINTS ${STM_BASE_DIR}/lib NO_CMAKE_FIND_ROOT_PATH)
    if(NOT CUPVA_HOST_LIB)
        message(FATAL_ERROR "cupva_host library not found.")
    endif()
    set(STM_DEPENDENT_LIBS ${STM_DEPENDENT_LIBS} ${CUPVA_HOST_LIB})
endif()

if ((${VIBRANTE_PDK_DECIMAL} GREATER_EQUAL 6000600))
    find_library(VULKAN_LIB "nvidia-vksc-core" HINTS ${VIBRANTE_PDK}/lib-target NO_CMAKE_FIND_ROOT_PATH)
    if(NOT VULKAN_LIB)
        message(FATAL_ERROR "nvidia-vksc-core library not found.")
endif()
    set(STM_DEPENDENT_LIBS ${STM_DEPENDENT_LIBS} ${VULKAN_LIB})
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    set(STM_DEPENDENT_LIBS ${STM_DEPENDENT_LIBS} socket)
endif()
endif()

#-------------------------------------------------------------------------------
# 3rdparties from source
#-------------------------------------------------------------------------------

add_subdirectory(cpu_simple)
add_subdirectory(cpu_gpu_simple)
add_subdirectory(sample_complete_swap)
