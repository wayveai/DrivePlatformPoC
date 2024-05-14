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
# SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# FindcuDNN
# ---------
#
# Finds the NVIDIA® CUDA® Deep Neural Network library (cuDNN) GPU-accelerated
# library of primitives for deep neural networks.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``cuDNN::cuDNN``
#   The cuDNN library
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``cuDNN_FOUND``
#   True if the system has the cuDNN library.
# ``cuDNN_VERSION``
#   The version of the cuDNN library that was found.
# ``cuDNN_INCLUDE_DIRS``
#   Include directories needed to use cuDNN.
# ``cuDNN_LIBRARIES``
#   Libraries needed to link to cuDNN.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``cuDNN_INCLUDE_DIR``
#   The directory containing ``cudnn.h``.
# ``cuDNN_LIBRARY``
#   The path to ``libcudnn.so``.

set(_cuDNN_PATHS
  /usr
  /usr/local/driveworks
)

if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "^(aarch64-linux-gnu|aarch64-unknown-nto-qnx)$")
  list(APPEND _cuDNN_PATHS
    "/usr/local/driveworks/targets/${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}"
  )
endif()

find_path(cuDNN_INCLUDE_DIR
  NAMES cudnn.h
  PATHS ${_cuDNN_PATHS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
)

set(_cuDNN_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}cudnn${CMAKE_SHARED_LIBRARY_SUFFIX}"
)

find_library(cuDNN_LIBRARY
  NAMES "${_cuDNN_SONAME}"
  PATHS ${_cuDNN_PATHS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
)

unset(_cuDNN_PATHS)

set(cuDNN_VERSION)
if(cuDNN_INCLUDE_DIR)
    file(STRINGS "${cuDNN_INCLUDE_DIR}/cudnn_version.h" _cuDNN_VERSION_STRINGS
        REGEX "^#define CUDNN_(MAJOR|MINOR|PATCHLEVEL)[ ]+[0-9]+$"
    )

    string(REGEX REPLACE ".*;#define CUDNN_MAJOR[ ]+([0-9]+);.*"
        "\\1" cuDNN_VERSION_MAJOR ";${_cuDNN_VERSION_STRINGS};"
    )

    string(REGEX REPLACE ".*;#define CUDNN_MINOR[ ]+([0-9]+);.*"
        "\\1" cuDNN_VERSION_MINOR ";${_cuDNN_VERSION_STRINGS};"
    )

    string(REGEX REPLACE ".*;#define CUDNN_PATCHLEVEL[ ]+([0-9]+);.*"
        "\\1" cuDNN_VERSION_PATCH ";${_cuDNN_VERSION_STRINGS};"
    )

    unset(_cuDNN_VERSION_STRINGS)

    set(cuDNN_VERSION
        "${cuDNN_VERSION_MAJOR}.${cuDNN_VERSION_MINOR}.${cuDNN_VERSION_PATCH}"
    )
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(cuDNN
  FOUND_VAR cuDNN_FOUND
  REQUIRED_VARS cuDNN_INCLUDE_DIR cuDNN_LIBRARY
  VERSION_VAR cuDNN_VERSION
)

if(cuDNN_FOUND)
  set(cuDNN_INCLUDE_DIRS "${cuDNN_INCLUDE_DIR}")
  set(cuDNN_LIBRARIES "${cuDNN_LIBRARY}")

  mark_as_advanced(cuDNN_INCLUDE_DIR cuDNN_LIBRARY)

  if(NOT TARGET cuDNN::cuDNN)
    add_library(cuDNN::cuDNN SHARED IMPORTED)
    set_target_properties(cuDNN::cuDNN PROPERTIES
      IMPORTED_LOCATION "${cuDNN_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${cuDNN_INCLUDE_DIR}"
      IMPORTED_SONAME "${_cuDNN_SONAME}"
      SOVERSION "${cuDNN_VERSION_MAJOR}"
      VERSION "${cuDNN_VERSION}"
    )
  endif()
endif()

unset(_cuDNN_SONAME)
