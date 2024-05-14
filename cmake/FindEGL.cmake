################################################################################
#
# Notice
# ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS"
# NVIDIA MAKES NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR
# OTHERWISE WITH RESPECT TO THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED
# WARRANTIES OF NONINFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR
# PURPOSE.
#
# NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences
# of use of such information or for any infringement of patents or other rights
# of third parties that may result from its use. No license is granted by
# implication or otherwise under any patent or patent rights of NVIDIA
# CORPORATION & AFFILIATES. No third party distribution is allowed unless
# expressly authorized by NVIDIA. Details are subject to change without notice.
# This code supersedes and replaces all information previously supplied. NVIDIA
# CORPORATION & AFFILIATES products are not authorized for use as critical
# components in life support devices or systems without express written approval
# of NVIDIA CORPORATION & AFFILIATES.
#
# SPDX-FileCopyrightText: Copyright (c) 2016-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this material and related documentation without an express
# license agreement from NVIDIA CORPORATION or its affiliates is strictly
# prohibited.
#
################################################################################

# - Try to find EGL
# Once done this will define
#  EGL_FOUND - System has EGL
#  EGL_INCLUDE_DIR - The EGL include directories
#  EGL_LIBRARIES - The libraries needed to use EGL
if (EGL_FOUND)
  return()
endif()

find_package(PkgConfig)
pkg_check_modules(PC_EGL QUIET egl)

find_path(EGL_INCLUDE_DIR EGL/egl.h
          HINTS ${PC_EGL_INCLUDEDIR} ${PC_EGL_INCLUDE_DIRS} ${VIBRANTE_PDK}/include)

find_library(EGL_LIBRARY EGL
             NO_DEFAULT_PATH
             HINTS "/usr/lib/nvidia-${nvidia-driver-version}" ${PC_EGL_LIBDIR} ${PC_EGL_LIBRARY_DIRS} ${VIBRANTE_PDK}/lib-target)

set(EGL_LIBRARIES ${EGL_LIBRARY})

option(DW_EXPERIMENTAL_FORCE_EGL "Force enable EGL support if EGL support is found" OFF)
mark_as_advanced(DW_EXPERIMENTAL_FORCE_EGL)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set EGL_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(EGL DEFAULT_MSG
                                  EGL_LIBRARY EGL_INCLUDE_DIR)

mark_as_advanced(EGL_INCLUDE_DIR EGL_LIBRARY)

# Do it in this file so that SDKConfiguration and Samples can re-use this logic
if (EGL_FOUND)
    message(STATUS "Found ${EGL_LIBRARY}:")
    message(STATUS " - Includes: [${EGL_INCLUDE_DIR}]")
    message(STATUS " - Libraries: [${EGL_LIBRARIES}]")
    if (VIBRANTE OR DW_EXPERIMENTAL_FORCE_EGL)
        find_package(GLES REQUIRED)
        if(NOT VIBRANTE_V5Q)
            find_package(DRM REQUIRED)
        else()
            find_package(DRM)
        endif()

        message(STATUS "DW_EXPERIMENTAL_FORCE_EGL set and EGL Support Enabled")
        add_library(egl SHARED IMPORTED)
        set_target_properties(egl PROPERTIES
                              IMPORTED_LOCATION ${EGL_LIBRARY}
                              INTERFACE_LINK_LIBRARIES ${EGL_LIBRARIES}
                              IMPORTED_LINK_INTERFACE_LIBRARIES "drm")

        # Workaround for nvcc + recent gcc libc's, having issues with `/usr/include`
        # being defined as system includes (it's not necessary to provide this path in this case)
        if (NOT ${EGL_INCLUDE_DIR} STREQUAL "/usr/include")
          set_target_properties(egl PROPERTIES
                                INTERFACE_SYSTEM_INCLUDE_DIRECTORIES ${EGL_INCLUDE_DIR})
        endif()

        set_property(TARGET egl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
        set_target_properties(egl PROPERTIES
                              IMPORTED_LOCATION_RELEASE ${EGL_LIBRARY})
        set_target_properties(egl PROPERTIES MAP_IMPORTED_CONFIG_PROFILE "Release")
        set(DW_USE_EGL ON)
    else()
        message(STATUS "DW_EXPERIMENTAL_FORCE_EGL not set, EGL Support Disabled")
        add_library(egl INTERFACE)
    endif()
else()
    if (DW_EXPERIMENTAL_FORCE_EGL)
        message(WARNING "DW_EXPERIMENTAL_FORCE_EGL requested but EGL not found.")
        if (LINUX)
            message(FATAL_ERROR "Install missing dependency with: 'sudo apt install libgles2-mesa-dev'?")
        else()
            message(FATAL_ERROR "Install missing dependency if EGL support is desired")
        endif()
    endif()
    add_library(egl INTERFACE)
endif()
