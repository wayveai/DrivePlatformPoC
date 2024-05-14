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

# FindCUDART
# ----------
#
# Finds the NVIDIA® CUDA® driver and runtime (CUDART) libraries.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``CUDART::CUDART``
#   The CUDART libraries.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``CUDART_FOUND``
#   True if the system has the CUDART libraries.
# ``CUDART_VERSION``
#   The version of the CUDART libraries that was found.
# ``CUDART_INCLUDE_DIRS``
#   Include directories needed to use the CUDART libraries.
# ``CUDART_LIBRARIES``
#   Libraries needed to link to the CUDART libraries.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``CUDART_INCLUDE_DIR``
#   The directory containing ``cuda.h``.
# ``CUDART_LIBRARY_DIR``
#   The directory containing ``libcuda.so`` and ``libcudart.so``.
# ``CUDART_CUDA_LIBRARY``
#   The path to ``libcuda.so``.
# ``CUDART_CUDART_LIBRARY``
#   The path to ``libcudart.so``.

set(_CUDART_FIND_PATH_PATHS)

if(CUDART_LIBRARY_DIR)
  set(_CUDART_FIND_LIBRARY_PATHS "${CUDART_LIBRARY_DIR}")
else()
  set(_CUDART_FIND_LIBRARY_PATHS)
endif()

if(CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES)
  list(APPEND _CUDART_FIND_PATH_PATHS
    "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}"
  )

  get_filename_component(_CUDART_CUDA_TOOLKIT_DIR
    "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}" DIRECTORY
  )
  list(APPEND _CUDART_FIND_LIBRARY_PATHS
    "${_CUDART_CUDA_TOOLKIT_DIR}/lib"
    "${_CUDART_CUDA_TOOLKIT_DIR}/lib/stubs"
    "${_CUDART_CUDA_TOOLKIT_DIR}/lib64"
    "${_CUDART_CUDA_TOOLKIT_DIR}/lib64/stubs"
  )
  unset(_CUDART_CUDA_TOOLKIT_DIR)
endif()

if(CMAKE_CUDA_COMPILER_TOOLKIT_VERSION)
  string(REPLACE . ; _CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
    "${CMAKE_CUDA_COMPILER_TOOLKIT_VERSION}"
  )
  list(GET _CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR 0 1
    _CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
  )
  list(JOIN _CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR .
    _CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
  )

  if(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-linux-gnu)
    set(_CUDART_LIBRARY_ARCHITECTURE aarch64-linux)
  elseif(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-unknown-nto-qnx)
    set(_CUDART_LIBRARY_ARCHITECTURE aarch64-qnx)
    list(APPEND _CUDART_FIND_PATH_PATHS
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/include"
    )
    list(APPEND _CUDART_FIND_LIBRARY_PATHS
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib/stubs"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib64"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib64/stubs"
    )
  else()
    set(_CUDART_LIBRARY_ARCHITECTURE x86_64-linux)
  endif()

  list(APPEND _CUDART_FIND_PATH_PATHS
    "/usr/local/cuda-${_CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDART_LIBRARY_ARCHITECTURE}/include"
  )
  list(APPEND _CUDART_FIND_LIBRARY_PATHS
    "/usr/local/cuda-${_CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDART_LIBRARY_ARCHITECTURE}/lib"
    "/usr/local/cuda-${_CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDART_LIBRARY_ARCHITECTURE}/lib/stubs"
    "/usr/local/cuda-${_CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDART_LIBRARY_ARCHITECTURE}/lib64"
    "/usr/local/cuda-${_CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDART_LIBRARY_ARCHITECTURE}/lib64/stubs"
  )
  unset(_CUDART_LIBRARY_ARCHITECTURE)
  unset(_CUDART_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR)

  set(CUDART_VERSION "${CMAKE_CUDA_COMPILER_VERSION}")
else()
  set(CUDART_VERSION)
endif()

find_path(CUDART_INCLUDE_DIR
  NAMES cuda.h
  PATHS ${_CUDART_FIND_PATH_PATHS}
)

unset(_CUDART_FIND_PATH_PATHS)

set(_CUDART_CUDA_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}cuda${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(CUDART_CUDA_LIBRARY
  NAMES "${_CUDART_CUDA_SONAME}"
  PATHS ${_CUDART_FIND_LIBRARY_PATHS}
)

if(CUDART_CUDA_LIBRARY)
  get_filename_component(_CUDART_CUDA_LIBRARY_DIR
    "${CUDART_CUDA_LIBRARY}" DIRECTORY
  )
  list(INSERT _CUDART_FIND_LIBRARY_PATHS 0
    "${_CUDART_CUDA_LIBRARY_DIR}"
  )
  unset(_CUDART_CUDA_LIBRARY_DIR)
endif()

set(_CUDART_CUDART_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}cudart${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(CUDART_CUDART_LIBRARY
  NAMES "${_CUDART_CUDART_SONAME}"
  PATHS ${_CUDART_FIND_LIBRARY_PATHS}
)

unset(_CUDART_FIND_LIBRARY_PATHS)

find_package(Threads QUIET MODULE REQUIRED)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(CUDART
  FOUND_VAR CUDART_FOUND
  REQUIRED_VARS
    CUDART_INCLUDE_DIR
    CUDART_CUDA_LIBRARY
    CUDART_CUDART_LIBRARY
  VERSION_VAR CUDART_VERSION
)

if(CUDART_FOUND)
  set(CUDART_INCLUDE_DIRS "${CUDART_INCLUDE_DIR}")
  set(CUDART_LIBRARIES
    "${CUDART_CUDA_LIBRARY}"
    "${CUDART_CUDART_LIBRARY}"
    ${CMAKE_THREAD_LIBS_INIT}
  )
  mark_as_advanced(
    CUDART_INCLUDE_DIR
    CUDART_CUDA_LIBRARY
    CUDART_CUDART_LIBRARY
  )

  if(CUDART_LIBRARY_DIR)
    mark_as_advanced(CUDART_LIBRARY_DIR)
  endif()

  if(NOT TARGET CUDART::cuda)
    add_library(CUDART::cuda SHARED IMPORTED)
    set_target_properties(CUDART::cuda PROPERTIES
      IMPORTED_LOCATION "${CUDART_CUDA_LIBRARY}"
      IMPORTED_SONAME "${_CUDART_CUDA_LIBRARY_SONAME}"
    )
  endif()

  if(NOT TARGET CUDART::cudart)
    add_library(CUDART::cudart SHARED IMPORTED)
    set_target_properties(CUDART::cudart PROPERTIES
      IMPORTED_LOCATION "${CUDART_CUDART_LIBRARY}"
      IMPORTED_SONAME "${_CUDART_CUDART_SONAME}"
    )
  endif()

  if(NOT TARGET CUDART::CUDART)
    add_library(CUDART::CUDART INTERFACE IMPORTED)
    set_target_properties(CUDART::CUDART PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CUDART_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "CUDART::cuda;CUDART::cudart;Threads::Threads"

    )
  endif()
endif()

unset(_CUDART_CUDART_SONAME)
unset(_CUDART_CUDA_SONAME)
