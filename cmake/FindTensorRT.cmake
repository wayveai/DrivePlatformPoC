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
# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# Finds the NVIDIA (R) TensorRT (TM) SDK for high-performance deep learning
# inference.
#
# Imported Targets
# ^^^^^^^^^^^^^^^^
#
# This module provides the following imported targets, if found:
#
# ``TensorRT::TensorRT``
#   The TensorRT libraries
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This will define the following variables:
#
# ``TensorRT_FOUND``
#   True if the system has the TensorRT libraries.
# ``TensorRT_VERSION``
#   The version of the TensorRT libraries that were found.
# ``TensorRT_INCLUDE_DIRS``
#   Include directories needed to use TensorRT.
# ``TensorRT_LIBRARIES``
#   Libraries needed to link to TensorRT.
#
# Cache Variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``TensorRT_INCLUDE_DIR``
#   The directory containing ``NvInfer.h``.
# ``TensorRT_nvinfer_LIBRARY``
#   The path to ``libnvinfer.so``.
# ``TensorRT_nvonnxparser_LIBRARY``
#   The path to ``libnvonnxparser.so``.
# ``TensorRT_nvparsers_LIBRARY``
#   The path to ``libnvparsers.so``.

find_path(TensorRT_INCLUDE_DIR
    NAMES NvInfer.h
    PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
)

set(_TensorRT_VERSION_FILEPATH "${TensorRT_INCLUDE_DIR}/NvInferVersion.h")
if(TensorRT_INCLUDE_DIR AND EXISTS "${_TensorRT_VERSION_FILEPATH}")
    file(STRINGS "${_TensorRT_VERSION_FILEPATH}" _TensorRT_VERSION_STRINGS
        REGEX "^#define[ ]+NV_TENSORRT_(MAJOR|MINOR|PATCH|BUILD)[ ]+[0-9]+.*$"
    )
    if(_TensorRT_VERSION_STRINGS)
        string(REGEX REPLACE ".*#define[ ]+NV_TENSORRT_MAJOR[ ]+([0-9]+).*" "\\1"
            TensorRT_VERSION_MAJOR "${_TensorRT_VERSION_STRINGS}"
        )
        string(REGEX REPLACE ".*#define[ ]+NV_TENSORRT_MINOR[ ]+([0-9]+).*" "\\1"
            TensorRT_VERSION_MINOR "${_TensorRT_VERSION_STRINGS}"
        )
        string(REGEX REPLACE ".*#define[ ]+NV_TENSORRT_PATCH[ ]+([0-9]+).*" "\\1"
            TensorRT_VERSION_PATCH "${_TensorRT_VERSION_STRINGS}"
        )
        string(REGEX REPLACE ".*#define[ ]+NV_TENSORRT_BUILD[ ]+([0-9]+).*" "\\1"
            TensorRT_VERSION_BUILD "${_TensorRT_VERSION_STRINGS}"
        )
        if(TensorRT_VERSION_MAJOR
            AND TensorRT_VERSION_MINOR
            AND TensorRT_VERSION_PATCH
            AND TensorRT_VERSION_BUILD
        )
            set(TensorRT_VERSION
                "${TensorRT_VERSION_MAJOR}.${TensorRT_VERSION_MINOR}.${TensorRT_VERSION_PATCH}.${TensorRT_VERSION_BUILD}"
            )
        else()
            set(TensorRT_VERSION)
        endif()
    else()
        set(TensorRT_VERSION)
    endif()
    unset(_TensorRT_VERSION_STRINGS)
else()
    set(TensorRT_VERSION)
endif()
unset(_TensorRT_VERSION_FILEPATH)

set(_TensorRT_nvinfer_SONAME
    "${CMAKE_SHARED_LIBRARY_PREFIX}nvinfer${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(TensorRT_nvinfer_LIBRARY
    NAMES "${_TensorRT_nvinfer_SONAME}"
    PATH_SUFFIXES
        "${CMAKE_LIBRARY_ARCHITECTURE}"
        "${CMAKE_LIBRARY_ARCHITECTURE}/stubs"
)
get_filename_component(_TensorRT_LIBRARY_DIR "${TensorRT_nvinfer_LIBRARY}" DIRECTORY)

set(_TensorRT_nvonnxparser_SONAME
    "${CMAKE_SHARED_LIBRARY_PREFIX}nvonnxparser${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(TensorRT_nvonnxparser_LIBRARY
    NAMES "${_TensorRT_nvonnxparser_SONAME}"
    HINTS "${_TensorRT_LIBRARY_DIR}"
)

set(_TensorRT_nvparsers_SONAME
    "${CMAKE_SHARED_LIBRARY_PREFIX}nvparsers${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
find_library(TensorRT_nvparsers_LIBRARY
    NAMES "${_TensorRT_nvparsers_SONAME}"
    HINTS "${_TensorRT_LIBRARY_DIR}"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
    FOUND_VAR TensorRT_FOUND
    REQUIRED_VARS
        TensorRT_INCLUDE_DIR
        TensorRT_nvinfer_LIBRARY
        TensorRT_nvonnxparser_LIBRARY
        TensorRT_nvparsers_LIBRARY
    VERSION_VAR TensorRT_VERSION
)

if(TensorRT_FOUND)
    set(TensorRT_INCLUDE_DIRS "${TensorRT_INCLUDE_DIR}")
    set(TensorRT_LIBRARIES
        "${TensorRT_nvinfer_LIBRARY}"
        "${TensorRT_nvonnxparser_LIBRARY}"
        "${TensorRT_nvparsers_LIBRARY}"
    )
    mark_as_advanced(
        TensorRT_INCLUDE_DIR
        TensorRT_nvinfer_LIBRARY
        TensorRT_nvonnxparser_LIBRARY
        TensorRT_nvparsers_LIBRARY
    )

    if(NOT TARGET TensorRT::nvinfer)
        add_library(TensorRT::nvinfer SHARED IMPORTED)
        set_target_properties(TensorRT::nvinfer PROPERTIES
            IMPORTED_LOCATION "${TensorRT_nvinfer_LIBRARY}"
            IMPORTED_SONAME "${_TensorRT_VERSION_nvinfer_SONAME}"
            SOVERSION "${TensorRT_VERSION_MAJOR}"
            VERSION "${TensorRT_VERSION}"
        )
    endif()

    if(NOT TARGET TensorRT::nvonnxparser)
        add_library(TensorRT::nvonnxparser SHARED IMPORTED)
        set_target_properties(TensorRT::nvonnxparser PROPERTIES
            IMPORTED_LOCATION "${TensorRT_nvonnxparser_LIBRARY}"
            IMPORTED_SONAME "${_TensorRT_VERSION_nvonnxparser_SONAME}"
            SOVERSION "${TensorRT_VERSION_MAJOR}"
            VERSION "${TensorRT_VERSION}"
        )
    endif()

    if(NOT TARGET TensorRT::nvparsers)
        add_library(TensorRT::nvparsers SHARED IMPORTED)
        set_target_properties(TensorRT::nvparsers PROPERTIES
            IMPORTED_LOCATION "${TensorRT_nvparsers_LIBRARY}"
            IMPORTED_SONAME "${_TensorRT_VERSION_nvparsers_SONAME}"
            SOVERSION "${TensorRT_VERSION_MAJOR}"
            VERSION "${TensorRT_VERSION}"
        )
    endif()

    if(NOT TARGET TensorRT::TensorRT)
        set(_TensorRT_INTERFACE_LINK_LIBRARIES
            TensorRT::nvinfer
            TensorRT::nvonnxparser
            TensorRT::nvparsers
        )
        add_library(TensorRT::TensorRT INTERFACE IMPORTED)
        set_target_properties(TensorRT::TensorRT PROPERTIES
            INTERFACE_LINK_LIBRARIES "${_TensorRT_INTERFACE_LINK_LIBRARIES}"
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}"
        )
        unset(_TensorRT_INTERFACE_LINK_LIBRARIES)
    endif()
endif()
