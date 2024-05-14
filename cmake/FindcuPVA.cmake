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
# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# FindcuPVA
# ---------
#
# Finds the NVIDIA® CUDA® Programmable Vision Accelerator (cuPVA) libraries.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``cuPVA::cuPVA``
#   The cuPVA libraries.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``cuPVA_FOUND``
#   True if the system has the cuPVA libraries.
# ``cuPVA_DEFINITIONS``
#   Compile definitions needed to use cuPVA.
# ``cuPVA_INCLUDE_DIRS``
#   Include directories needed to use cuPVA.
# ``cuPVA_LIBRARIES``
#   Libraries needed to link to cuPVA.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``cuPVA_INCLUDE_DIR``
#   The directory containing ``cupva_types_wrapper.h``.
# ``cuPVA_HOST_LIBRARY``
#   The path to ``libcupva_host.so`` or ``libcupva_host_native.so``.
# ``cuPVA_WRAPPER_LIBRARY``
#   The path to ``libcupva_wrapper.a``.

set(_cuPVA_FIND_LIBRARY_PATHS)
set(_cuPVA_FIND_PATH_PATHS)

if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "^aarch64-(linux-gnu|unknown-nto-qnx)$")
  list(APPEND _cuPVA_FIND_LIBRARY_PATHS
    "/usr/local/driveworks/targets/${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}/lib"
  )
  list(APPEND _cuPVA_FIND_PATH_PATHS
    "/usr/local/driveworks/targets/${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}/include"
  )
endif()

list(APPEND _cuPVA_FIND_LIBRARY_PATHS /usr/local/driveworks/lib)
list(APPEND _cuPVA_FIND_PATH_PATHS /usr/local/driveworks/include)

if(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-linux-gnu)
  list(APPEND _cuPVA_FIND_LIBRARY_PATHS /usr/lib/aarch64-linux-gnu)
elseif(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-unknown-nto-qnx)
  list(APPEND _cuPVA_FIND_LIBRARY_PATHS /usr/lib/aarch64-qnx710)
else()
  list(APPEND _cuPVA_FIND_LIBRARY_PATHS /usr/lib/x86_64-linux-gnu)
endif()

find_path(cuPVA_INCLUDE_DIR
  NAMES cupva_types_wrapper.h
  PATHS ${_cuPVA_FIND_PATH_PATHS}
  PATH_SUFFIXES cupva
)

unset(_cuPVA_FIND_PATH_PATHS)

if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "^aarch64-(linux-gnu|unknown-nto-qnx)$")
  set(_cuPVA_HOST_SONAME
    "${CMAKE_SHARED_LIBRARY_PREFIX}cupva_host${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
else()
  set(_cuPVA_HOST_SONAME
    "${CMAKE_SHARED_LIBRARY_PREFIX}cupva_host_native${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
endif()

find_library(cuPVA_HOST_LIBRARY
  NAMES "${_cuPVA_HOST_SONAME}"
  PATHS ${_cuPVA_FIND_LIBRARY_PATHS}
)

if(NOT cuPVA_HOST_LIBRARY)
  if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "^aarch64-(linux-gnu|unknown-nto-qnx)$")
    set(_cuPVA_HOST_SONAME
      "${CMAKE_SHARED_LIBRARY_PREFIX}cupva_host_gen2${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
  else()
    set(_cuPVA_HOST_SONAME
      "${CMAKE_SHARED_LIBRARY_PREFIX}cupva_host_native_gen2${CMAKE_SHARED_LIBRARY_SUFFIX}"
    )
  endif()

  find_library(cuPVA_HOST_LIBRARY
    NAMES "${_cuPVA_HOST_SONAME}"
    PATHS ${_cuPVA_FIND_LIBRARY_PATHS}
  )
endif()

find_library(cuPVA_WRAPPER_LIBRARY
  NAMES "${CMAKE_STATIC_LIBRARY_PREFIX}cupva_wrapper${CMAKE_STATIC_LIBRARY_SUFFIX}"
  PATHS ${_cuPVA_FIND_LIBRARY_PATHS}
)

unset(_cuPVA_CUPVA_ALGO_LIBRARY_DIR)
unset(_cuPVA_FIND_LIBRARY_PATHS)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(cuPVA
  FOUND_VAR cuPVA_FOUND
  REQUIRED_VARS
    cuPVA_INCLUDE_DIR
    cuPVA_HOST_LIBRARY
    cuPVA_WRAPPER_LIBRARY
)

if(cuPVA_FOUND)
  set(cuPVA_DEFINITIONS -DDW_SDK_BUILD_PVA)
  set(cuPVA_INCLUDE_DIRS "${cuPVA_INCLUDE_DIR}")
  set(cuPVA_LIBRARIES "${cuPVA_HOST_LIBRARY}" "${cuPVA_WRAPPER_LIBRARY}")
  set(cuPVA_DEFINITIONS DW_SDK_BUILD_PVA)
  mark_as_advanced(
    cuPVA_INCLUDE_DIR
    cuPVA_HOST_LIBRARY
    cuPVA_WRAPPER_LIBRARY
  )

  if(NOT TARGET cuPVA::cupva_host)
    add_library(cuPVA::cupva_host SHARED IMPORTED)
    set_target_properties(cuPVA::cupva_host PROPERTIES
      IMPORTED_LOCATION "${cuPVA_HOST_LIBRARY}"
      IMPORTED_SONAME "${_cuPVA_HOST_SONAME}"
    )
  endif()

  if(NOT TARGET cuPVA::cupva_wrapper)
    add_library(cuPVA::cupva_wrapper STATIC IMPORTED)
    set_target_properties(cuPVA::cupva_wrapper PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES CXX
      IMPORTED_LOCATION "${cuPVA_WRAPPER_LIBRARY}"
    )
  endif()

  if(NOT TARGET cuPVA::cuPVA)
    add_library(cuPVA::cuPVA INTERFACE IMPORTED)
    set_target_properties(cuPVA::cuPVA PROPERTIES
      INTERFACE_COMPILE_DEFINITIONS DW_SDK_BUILD_PVA
      INTERFACE_INCLUDE_DIRECTORIES "${cuPVA_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "cuPVA::cupva_host;cuPVA::cupva_wrapper"
    )
  endif()
endif()

unset(_cuPVA_HOST_SONAME)
