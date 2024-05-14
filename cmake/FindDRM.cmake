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
# SPDX-FileCopyrightText: Copyright (c) 2018-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# - Try to find DRM
# Once done this will define
#  DRM_FOUND - System has DRM

find_package(PkgConfig)
pkg_check_modules(PC_DRM QUIET libdrm)

find_path(DRM_INCLUDE_DIR /drm.h
          /usr/include/libdrm /usr/include/drm)

find_library(DRM_LIBRARY drm
             PATHS /usr/lib)

if (NOT DRM_INCLUDE_DIR)
    if (LINUX)
        message(WARNING "DRM header not found. Use: 'sudo apt install libdrm-dev'")
    endif()
    message(WARNING "DRM header not found")
else()
    message(STATUS "Found: ${DRM_LIBRARY}")
    message(STATUS "Header at: ${DRM_INCLUDE_DIR}")
endif()

if (DRM_LIBRARY)
    add_library(drm SHARED IMPORTED)
    set_target_properties(drm PROPERTIES
                          IMPORTED_LOCATION ${DRM_LIBRARY}
                          INTERFACE_INCLUDE_DIRECTORIES ${DRM_INCLUDE_DIR}
                          INTERFACE_LINK_LIBRARIES ${DRM_LIBRARY})
    set_property(TARGET drm APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
    set_target_properties(drm PROPERTIES
                          IMPORTED_LOCATION_RELEASE ${DRM_LIBRARY})
    set_target_properties(drm PROPERTIES MAP_IMPORTED_CONFIG_PROFILE "Release")
    set(DRM_FOUND TRUE)
else()
    add_library(drm INTERFACE)
endif()
