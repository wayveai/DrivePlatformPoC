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

# FindcuDLA
# ---------
#
# Finds the NVIDIA® CUDA® Deep Learning Accelerator (cuDLA) library.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``cuDLA::cuDLA``
#   The cuDLA library.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``cuDLA_FOUND``
#   True if the system has the cuDLA library.
# ``cuDLA_VERSION``
#   The version of the cuDLA library that was found.
# ``cuDLA_INCLUDE_DIRS``
#   Include directories needed to use the cuDLA library.
# ``cuDLA_LIBRARIES``
#   Libraries needed to link to the cuDLA library.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``cuDLA_INCLUDE_DIR``
#   The directory containing ``cudla.h``.
# ``cuDLA_LIBRARY_DIR``
#   The directory containing ``libcudla.so``.
# ``cuDLA_LIBRARY``
#   The path to ``libcudla.so``.

set(_CUDLA_FIND_PATH_PATHS)

if(cuDLA_LIBRARY_DIR)
  set(_CUDLA_FIND_LIBRARY_PATHS "${cuDLA_LIBRARY_DIR}")
else()
  set(_CUDLA_FIND_LIBRARY_PATHS)
endif()

if(CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES)
  list(APPEND _CUDLA_FIND_PATH_PATHS
    "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}"
  )

  get_filename_component(_CUDLA_CUDA_TOOLKIT_DIR
    "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}" DIRECTORY
  )
  list(APPEND _CUDLA_FIND_LIBRARY_PATHS
    "${_CUDLA_CUDA_TOOLKIT_DIR}/lib"
    "${_CUDLA_CUDA_TOOLKIT_DIR}/lib/stubs"
    "${_CUDLA_CUDA_TOOLKIT_DIR}/lib64"
    "${_CUDLA_CUDA_TOOLKIT_DIR}/lib64/stubs"
  )
  unset(_CUDLA_CUDA_TOOLKIT_DIR)
endif()

if(CMAKE_CUDA_COMPILER_TOOLKIT_VERSION)
  string(REPLACE . ; _CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
    "${CMAKE_CUDA_COMPILER_TOOLKIT_VERSION}"
  )
  list(GET _CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR 0 1
    _CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
  )
  list(JOIN _CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR .
    _CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
  )

  if(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-linux-gnu)
    set(_CUDLA_LIBRARY_ARCHITECTURE aarch64-linux)
  elseif(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-unknown-nto-qnx)
    set(_CUDLA_LIBRARY_ARCHITECTURE aarch64-qnx)
    list(APPEND _CUDLA_FIND_PATH_PATHS
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/include"
    )
    list(APPEND _CUDLA_FIND_LIBRARY_PATHS
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib/stubs"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib64"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib64/stubs"
    )
  else()
    set(_CUDLA_LIBRARY_ARCHITECTURE x86_64-linux)
  endif()

  list(APPEND _CUDLA_FIND_PATH_PATHS
    "/usr/local/cuda-${_CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDLA_LIBRARY_ARCHITECTURE}/include"
  )
  list(APPEND _CUDLA_FIND_LIBRARY_PATHS
    "/usr/local/cuda-${_CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDLA_LIBRARY_ARCHITECTURE}/lib"
    "/usr/local/cuda-${_CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDLA_LIBRARY_ARCHITECTURE}/lib/stubs"
    "/usr/local/cuda-${_CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDLA_LIBRARY_ARCHITECTURE}/lib64"
    "/usr/local/cuda-${_CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUDLA_LIBRARY_ARCHITECTURE}/lib64/stubs"
  )
  unset(_CUDLA_LIBRARY_ARCHITECTURE)
  unset(_CUDLA_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR)

  set(cuDLA_VERSION "${CMAKE_CUDA_COMPILER_VERSION}")
else()
  set(cuDLA_VERSION)
endif()

find_path(cuDLA_INCLUDE_DIR
  NAMES cudla.h
  PATHS ${_CUDLA_FIND_PATH_PATHS}
)

unset(_CUDLA_FIND_PATH_PATHS)

set(_CUDLA_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}cudla${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(cuDLA_LIBRARY
  NAMES "${_CUDLA_SONAME}"
  PATHS ${_CUDLA_FIND_LIBRARY_PATHS}
)

unset(_CUDLA_FIND_LIBRARY_PATHS)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(cuDLA
  FOUND_VAR cuDLA_FOUND
  REQUIRED_VARS cuDLA_INCLUDE_DIR cuDLA_LIBRARY
  VERSION_VAR cuDLA_VERSION
)

if(cuDLA_FOUND)
  set(cuDLA_INCLUDE_DIRS "${cuDLA_INCLUDE_DIR}")
  set(cuDLA_LIBRARIES "${cuDLA_LIBRARY}")
  mark_as_advanced(cuDLA_INCLUDE_DIR cuDLA_LIBRARY)

  if(cuDLA_LIBRARY_DIR)
    mark_as_advanced(cuDLA_LIBRARY_DIR)
  endif()

  if(NOT TARGET cuDLA::cuDLA)
    add_library(cuDLA::cuDLA SHARED IMPORTED)
    set_target_properties(cuDLA::cuDLA PROPERTIES
      IMPORTED_LOCATION "${cuDLA_LIBRARY}"
      IMPORTED_SONAME "${_CUDLA_SONAME}"
      INTERFACE_INCLUDE_DIRECTORIES "${cuDLA_INCLUDE_DIR}"
    )
  endif()
endif()

unset(_CUDLA_SONAME)
