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

# - Try to find GLES
# Once done this will define
#  GLES_FOUND - System has GLES

find_package(PkgConfig)
pkg_check_modules(PC_GLES QUIET gles)

find_path(GLES_INCLUDE_DIR GLES3/gl31.h
          HINTS ${PC_GLES_INCLUDEDIR} ${PC_GLES_INCLUDE_DIRS} ${VIBRANTE_PDK}/include
          PATHS /usr/include)
find_path(GLES_INCLUDE_DIR2 GLES2/gl2ext.h
          HINTS ${PC_GLES_INCLUDEDIR} ${PC_GLES_INCLUDE_DIRS} ${VIBRANTE_PDK}/include
          PATHS /usr/include)

find_library(GLES_LIBRARY GLESv2
             HINTS ${PC_GLES_LIBDIR} ${PC_GLES_LIBRARY_DIRS} ${VIBRANTE_PDK}/lib-target
             PATHS /usr/lib)

if ((NOT GLES_INCLUDE_DIR) OR (NOT GLES_INCLUDE_DIR2))
    if (LINUX)
        message(FATAL_ERROR "GLES header not found. Use: 'sudo apt install libgles2-mesa-dev'")
    endif()
    message(FATAL_ERROR "GLES header not found")
endif()

if (GLES_LIBRARY)
    add_library(gles SHARED IMPORTED)
    set_target_properties(gles PROPERTIES
                          IMPORTED_LOCATION ${GLES_LIBRARY}
                          INTERFACE_SYSTEM_INCLUDE_DIRECTORIES ${GLES_INCLUDE_DIR}
                          INTERFACE_LINK_LIBRARIES ${GLES_LIBRARY})
    set_property(TARGET gles APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
    set_target_properties(gles PROPERTIES
                          IMPORTED_LOCATION_RELEASE ${GLES_LIBRARY})
    set_target_properties(gles PROPERTIES MAP_IMPORTED_CONFIG_PROFILE "Release")
    set(GLES_FOUND TRUE)
endif()
