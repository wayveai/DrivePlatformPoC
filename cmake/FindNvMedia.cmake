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
# SPDX-FileCopyrightText: Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# FindNvMedia
# -----------
#
# Finds the NVIDIA (R) NvMedia libraries for hardware-accelerated processing
# of image and video data.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``NvMedia::NvMedia``
#   The NvMedia library.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``NvMedia_FOUND``
#   True if the system has the NvMedia library.
# ``NvMedia_VERSION``
#   The version of the NvMedia library that was found.
# ``NvMedia_INCLUDE_DIRS``
#   Include directories needed to use NvMedia.
# ``NvMedia_LIBRARIES``
#   Libraries needed to link to NvMedia.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``NvMedia_INCLUDE_DIR``
#   The directory containing ``nvmedia_core.h``.
# ``NvMedia_LIBRARY``
#   The path to ``libnvmedia.so`` or ``libnvmedia_core.so``.

set(_NvMedia_FIND_LIBRARY_PATHS)
set(_NvMedia_FIND_PATH_PATHS)

if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "^(aarch64-linux-gnu|aarch64-unknown-nto-qnx)$" AND VIBRANTE_PDK)
    list(APPEND _NvMedia_FIND_LIBRARY_PATHS "${VIBRANTE_PDK}/lib-target")
    list(APPEND _NvMedia_FIND_PATH_PATHS "${VIBRANTE_PDK}/include")
endif()

find_path(NvMedia_INCLUDE_DIR
    NAMES nvmedia_core.h
    PATHS ${_NvMedia_FIND_PATH_PATHS}
)
unset(_NvMedia_FIND_PATH_PATHS)

set(_NvMedia_SONAME
    "${CMAKE_SHARED_LIBRARY_PREFIX}nvmedia${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(NvMedia_LIBRARY
    NAMES "${_NvMedia_SONAME}"
    PATHS ${_NvMedia_FIND_LIBRARY_PATHS}
)
if(NOT NvMedia_LIBRARY)
  set(_NvMedia_SONAME
      "${CMAKE_SHARED_LIBRARY_PREFIX}nvmedia_core${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
  find_library(NvMedia_LIBRARY
      NAMES "${_NvMedia_SONAME}"
      PATHS ${_NvMedia_FIND_LIBRARY_PATHS}
  )
endif()
unset(_NvMedia_FIND_LIBRARY_PATHS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NvMedia
    FOUND_VAR NvMedia_FOUND
    REQUIRED_VARS NvMedia_INCLUDE_DIR NvMedia_LIBRARY
)

if(NvMedia_FOUND)
    set(NvMedia_INCLUDE_DIRS "${NvMedia_INCLUDE_DIR}")
    set(NvMedia_LIBRARIES "${NvMedia_LIBRARY}")
    mark_as_advanced(NvMedia_INCLUDE_DIR NvMedia_LIBRARY)

    if(NOT TARGET NvMedia::NvMedia)
        add_library(NvMedia::NvMedia SHARED IMPORTED)
        set_target_properties(NvMedia::NvMedia PROPERTIES
            IMPORTED_LOCATION "${NvMedia_LIBRARY}"
            IMPORTED_SONAME "${_NvMedia_SONAME}"
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${NvMedia_INCLUDE_DIR}"
        )
    endif()
endif()
unset(_NvMedia_SONAME)
