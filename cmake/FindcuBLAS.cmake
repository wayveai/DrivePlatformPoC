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

# FindcuBLAS
# ----------
#
# Finds the NVIDIA® CUDA® Basic Linear Algebra Subroutine (cuBLAS) libraries.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``cuBLAS::cuBLAS``
#   The cuBLAS libraries.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``cuBLAS_FOUND``
#   True if the system has the cuBLAS libraries.
# ``cuBLAS_VERSION``
#   The version of the cuBLAS libraries that was found.
# ``cuBLAS_INCLUDE_DIRS``
#   Include directories needed to use the cuBLAS libraries.
# ``cuBLAS_LIBRARIES``
#   Libraries needed to link to the cuBLAS libraries.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``cuBLAS_INCLUDE_DIR``
#   The directory containing ``cublas_v2.h``.
# ``cuBLAS_LIBRARY_DIR``
#   The directory containing ``libcublas.so`` and ``libcublasLt.so``.
# ``cuBLAS_CUBLAS_LIBRARY``
#   The path to ``libcublas.so``.
# ``cuBLAS_CUBLASLT_LIBRARY``
#   The path to ``libcublasLt.so``.

set(_CUBLAS_FIND_PATH_PATHS)

if(cuBLAS_LIBRARY_DIR)
  set(_CUBLAS_FIND_LIBRARY_PATHS "${cuBLAS_LIBRARY_DIR}")
else()
  set(_CUBLAS_FIND_LIBRARY_PATHS)
endif()

if(CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES)
  list(APPEND _CUBLAS_FIND_PATH_PATHS
    "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}"
  )

  get_filename_component(_CUBLAS_CUDA_TOOLKIT_DIR
    "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}" DIRECTORY
  )
  list(APPEND _CUBLAS_FIND_LIBRARY_PATHS
    "${_CUBLAS_CUDA_TOOLKIT_DIR}/lib"
    "${_CUBLAS_CUDA_TOOLKIT_DIR}/lib/stubs"
    "${_CUBLAS_CUDA_TOOLKIT_DIR}/lib64"
    "${_CUBLAS_CUDA_TOOLKIT_DIR}/lib64/stubs"
  )
  unset(_CUBLAS_CUDA_TOOLKIT_DIR)
endif()

if(CMAKE_CUDA_COMPILER_TOOLKIT_VERSION)
  string(REPLACE . ; _CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
    "${CMAKE_CUDA_COMPILER_TOOLKIT_VERSION}"
  )
  list(GET _CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR 0 1
    _CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
  )
  list(JOIN _CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR .
    _CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR
  )

  if(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-linux-gnu)
    set(_CUBLAS_LIBRARY_ARCHITECTURE aarch64-linux)
  elseif(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-unknown-nto-qnx)
    set(_CUBLAS_LIBRARY_ARCHITECTURE aarch64-qnx)
    list(APPEND _CUBLAS_FIND_PATH_PATHS
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/include"
    )
    list(APPEND _CUBLAS_FIND_LIBRARY_PATHS
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib/stubs"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib64"
      "/usr/local/cuda-safe-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/aarch64-qnx/lib64/stubs"
    )
  else()
    set(_CUBLAS_LIBRARY_ARCHITECTURE x86_64-linux)
  endif()

  list(APPEND _CUBLAS_FIND_PATH_PATHS
    "/usr/local/cuda-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUBLAS_LIBRARY_ARCHITECTURE}/include"
  )

  list(APPEND _CUBLAS_FIND_LIBRARY_PATHS
    "/usr/local/cuda-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUBLAS_LIBRARY_ARCHITECTURE}/lib"
    "/usr/local/cuda-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUBLAS_LIBRARY_ARCHITECTURE}/lib/stubs"
    "/usr/local/cuda-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUBLAS_LIBRARY_ARCHITECTURE}/lib64"
    "/usr/local/cuda-${_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR}/targets/${_CUBLAS_LIBRARY_ARCHITECTURE}/lib64/stubs"
  )
  unset(_CUBLAS_LIBRARY_ARCHITECTURE)
  unset(_CUBLAS_CUDA_COMPILER_TOOLKIT_VERSION_MAJOR_MINOR)

  set(cuBLAS_VERSION "${CMAKE_CUDA_COMPILER_VERSION}")
else()
  set(cuBLAS_VERSION)
endif()

find_path(cuBLAS_INCLUDE_DIR
  NAMES cublas_v2.h
  PATHS ${_CUBLAS_FIND_PATH_PATHS}
)

unset(_CUBLAS_FIND_PATH_PATHS)

set(_CUBLAS_CUBLAS_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}cublas${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(cuBLAS_CUBLAS_LIBRARY
  NAMES "${_CUBLAS_CUBLAS_SONAME}"
  PATHS ${_CUBLAS_FIND_LIBRARY_PATHS}
)

if(cuBLAS_CUBLAS_LIBRARY)
  get_filename_component(_CUBLAS_CUBLAS_LIBRARY_DIR
    "${cuBLAS_CUBLAS_LIBRARY}" DIRECTORY
  )
  list(INSERT _CUBLAS_FIND_LIBRARY_PATHS 0
    "${_CUBLAS_CUBLAS_LIBRARY_DIR}"
  )
  unset(_CUBLAS_CUBLAS_LIBRARY_DIR)
endif()

set(_CUBLAS_CUBLASLT_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}cublasLt${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(cuBLAS_CUBLASLT_LIBRARY
  NAMES "${_CUBLAS_CUBLASLT_SONAME}"
  PATHS ${_CUBLAS_FIND_LIBRARY_PATHS}
)

unset(_CUBLAS_FIND_LIBRARY_PATHS)

find_package(Threads QUIET MODULE REQUIRED)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(cuBLAS
  FOUND_VAR cuBLAS_FOUND
  REQUIRED_VARS
    cuBLAS_INCLUDE_DIR
    cuBLAS_CUBLAS_LIBRARY
    cuBLAS_CUBLASLT_LIBRARY
  VERSION_VAR cuBLAS_VERSION
)

if(cuBLAS_FOUND)
  set(cuBLAS_INCLUDE_DIRS "${cuBLAS_INCLUDE_DIR}")
  set(cuBLAS_LIBRARIES
    "${cuBLAS_CUBLAS_LIBRARY}"
    "${cuBLAS_CUBLASLT_LIBRARY}"
    ${CMAKE_THREAD_LIBS_INIT}
  )
  mark_as_advanced(
    cuBLAS_INCLUDE_DIR
    cuBLAS_CUBLAS_LIBRARY
    cuBLAS_CUBLASLT_LIBRARY
  )

  if(cuBLAS_LIBRARY_DIR)
    mark_as_advanced(cuBLAS_LIBRARY_DIR)
  endif()

  if(NOT TARGET cuBLAS::cublas)
    add_library(cuBLAS::cublas SHARED IMPORTED)
    set_target_properties(cuBLAS::cublas PROPERTIES
      IMPORTED_LOCATION "${cuBLAS_CUBLAS_LIBRARY}"
      IMPORTED_SONAME "${_CUBLAS_CUBLAS_LIBRARY_SONAME}"
    )
  endif()

  if(NOT TARGET cuBLAS::cublasLt)
    add_library(cuBLAS::cublasLt SHARED IMPORTED)
    set_target_properties(cuBLAS::cublasLt PROPERTIES
      IMPORTED_LOCATION "${cuBLAS_CUBLASLT_LIBRARY}"
      IMPORTED_SONAME "${_CUBLAS_CUBLASLT_SONAME}"
    )
  endif()

  if(NOT TARGET cuBLAS::cuBLAS)
    add_library(cuBLAS::cuBLAS INTERFACE IMPORTED)
    set_target_properties(cuBLAS::cuBLAS PROPERTIES
      INTERFACE_LINK_LIBRARIES "cuBLAS::cublas;cuBLAS::cublasLt;Threads::Threads"
      INTERFACE_INCLUDE_DIRECTORIES "${cuBLAS_INCLUDE_DIR}"
    )
  endif()
endif()

unset(_CUBLAS_CUBLASLT_SONAME)
unset(_CUBLAS_CUBLAS_SONAME)
