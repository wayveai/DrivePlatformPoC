# /////////////////////////////////////////////////////////////////////////////////////////
#
# Notice
# ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
# NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
# THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
# MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
#
# NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
# information or for any infringement of patents or other rights of third parties that may
# result from its use. No license is granted by implication or otherwise under any patent
# or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
# expressly authorized by NVIDIA. Details are subject to change without notice.
# This code supersedes and replaces all information previously supplied.
# NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
# components in life support devices or systems without express written approval of
# NVIDIA CORPORATION & AFFILIATES.
#
# SPDX-FileCopyrightText: Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
# /////////////////////////////////////////////////////////////////////////////////////////

set(CMAKE_SYSTEM_NAME "QNX")
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(VIBRANTE_BUILD ON)       #flags for the CMakeList.txt
set(ARCH "aarch64le")

# need that one here, because this is a toolchain file and hence executed before
# default cmake settings are set
set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ".so")

# QNX-required environment variables
if(NOT DEFINED ENV{QNX_HOST} OR NOT DEFINED ENV{QNX_TARGET})
    message(FATAL_ERROR "Need to define QNX_HOST and QNX_TARGET")
endif()

set(QNX_HOST "$ENV{QNX_HOST}")
set(QNX_TARGET "$ENV{QNX_TARGET}")
set(QNX_ARCH "${ARCH}")
set(ENV{QNX_ARCH} "${QNX_ARCH}")

# QNX-required definitions & compiler flags
set(QNX_TOOLCHAIN_VERSION "8.3.0")
set(QNX_TOOLCHAIN_TRIPLE "aarch64-unknown-nto-qnx7.1.0")
set(QNX_TOOLCHAIN_PREFIX "${QNX_HOST}/usr/bin/${QNX_TOOLCHAIN_TRIPLE}-")
set(QNX_TOOLCHAIN_CFLAGS_CPP_ONLY "\
        -isystem ${QNX_TARGET}/usr/include/c++/v1")

set(QNX_TOOLCHAIN_CFLAGS "\
        -fno-tree-vectorize \
        -isystem ${QNX_HOST}/usr/lib/gcc/${QNX_TOOLCHAIN_TRIPLE}/${QNX_TOOLCHAIN_VERSION}/include \
        -isystem ${QNX_TARGET}/usr/include \
        -isystem ${QNX_TARGET}/usr/include/io-pkt \
        -D_POSIX_C_SOURCE=200112L -D_QNX_SOURCE \
        -DWIN_INTERFACE_CUSTOM -D_FILE_OFFSET_BITS=64")

set(BSP_LIBRARY_DIRS ${VIBRANTE_PDK}/nvidia-bsp/aarch64le/usr/lib)

# check that Vibrante PDK must be set
if(NOT DEFINED VIBRANTE_PDK)
    if(DEFINED ENV{VIBRANTE_PDK})
        message(STATUS "VIBRANTE_PDK = ENV : $ENV{VIBRANTE_PDK}")
        set(VIBRANTE_PDK $ENV{VIBRANTE_PDK} CACHE STRING "Path to the vibrante-XXX-qnx path for cross-compilation" FORCE)
    endif()
else()
     message(STATUS "VIBRANTE_PDK = ${VIBRANTE_PDK}")
endif()

if(DEFINED VIBRANTE_PDK)
  if(NOT IS_ABSOLUTE ${VIBRANTE_PDK})
      get_filename_component(VIBRANTE_PDK "${SDK_BINARY_DIR}/${VIBRANTE_PDK}" ABSOLUTE)
  endif()
endif()

set(VIBRANTE TRUE)
set(VIBRANTE_V5Q TRUE)
add_definitions(-DVIBRANTE -DVIBRANTE_V5Q)

set(CMAKE_CXX_COMPILER "${QNX_TOOLCHAIN_PREFIX}g++")
set(CMAKE_C_COMPILER "${QNX_TOOLCHAIN_PREFIX}gcc")
set(GCC_COMPILER_VERSION ${QNX_TOOLCHAIN_TRIPLE} CACHE STRING "GCC Compiler version")

# setup compiler for cross-compilation
set(CMAKE_CXX_FLAGS           "-fPIC"               CACHE STRING "c++ flags")
set(CMAKE_C_FLAGS             "-fPIC"               CACHE STRING "c flags")
set(CMAKE_SHARED_LINKER_FLAGS ""                    CACHE STRING "shared linker flags")
set(CMAKE_MODULE_LINKER_FLAGS ""                    CACHE STRING "module linker flags")
set(CMAKE_EXE_LINKER_FLAGS    ""                    CACHE STRING "executable linker flags")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${QNX_TOOLCHAIN_CFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${QNX_TOOLCHAIN_CFLAGS_CPP_ONLY} ${QNX_TOOLCHAIN_CFLAGS}")

set(LD_PATH ${VIBRANTE_PDK}/lib-target)

# Please, be careful looks like "-Wl,-unresolved-symbols=ignore-in-shared-libs" can lead to silent "ld" problems
set(CMAKE_SHARED_LINKER_FLAGS   "-L${LD_PATH} -Wl,-rpath-link,${LD_PATH} ${CMAKE_SHARED_LINKER_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS   "-L${LD_PATH} -Wl,-rpath-link,${LD_PATH} ${CMAKE_SHARED_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS      "-L${LD_PATH} -Wl,-rpath-link,${LD_PATH} ${CMAKE_EXE_LINKER_FLAGS} -L${BSP_LIBRARY_DIRS} -Wl,-rpath-link,${BSP_LIBRARY_DIRS} ")

# Set cmake root path. If there is no "/usr/local" in CMAKE_FIND_ROOT_PATH then FindCUDA.cmake doesn't work
SET(CMAKE_FIND_ROOT_PATH ${VIBRANTE_PDK} /usr/local/)

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# set system default include dir
include_directories(BEFORE SYSTEM ${VIBRANTE_PDK}/include)

if(NOT DEFINED VIBRANTE_PDK_BRANCH AND VIBRANTE_PDK)
    if(EXISTS "${VIBRANTE_PDK}/lib-target/version-pdk.txt")
        set(VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-pdk.txt")
    elseif(EXISTS "${VIBRANTE_PDK}/lib-target/version-sdk.txt")
        set(VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-sdk.txt")
    endif()

    if(VIBRANTE_PDK_FILE)
       file(READ ${VIBRANTE_PDK_FILE} version-nv-pdk)
       if(${version-nv-pdk} MATCHES "^(.+)-[0123456789]+")
           set(VIBRANTE_PDK_BRANCH ${CMAKE_MATCH_1} CACHE STRING "Cross-compilation PDK branch name")
           message(STATUS "VIBRANTE_PDK_BRANCH = ${VIBRANTE_PDK_BRANCH}")
       else()
           message(FATAL_ERROR "Can't determine PDK branch for PDK ${VIBRANTE_PDK}")
       endif()
    else()
       message(FATAL_ERROR "Can't open ${VIBRANTE_PDK}/lib-target/version-(pdk/sdk).txt for PDK branch detection")
    endif()
endif()

if(DEFINED VIBRANTE_PDK_BRANCH)
  string(REPLACE "." ";" PDK_VERSION_LIST ${VIBRANTE_PDK_BRANCH})

  # Some PDK's have less than three version numbers. Pad the list so we always
  # have at least three numbers, allowing pre-existing logic depending on major,
  # minor, patch versioning to work without modifications
  list(LENGTH PDK_VERSION_LIST _PDK_VERSION_LIST_LENGTH)
  while(_PDK_VERSION_LIST_LENGTH LESS 3)
    list(APPEND PDK_VERSION_LIST 0)
    math(EXPR _PDK_VERSION_LIST_LENGTH "${_PDK_VERSION_LIST_LENGTH} + 1")
  endwhile()

  set(VIBRANTE_PDK_PATCH 0)
  set(VIBRANTE_PDK_BUILD 0)

  list(LENGTH PDK_VERSION_LIST PDK_VERSION_LIST_LENGTH)

  list(GET PDK_VERSION_LIST 0 VIBRANTE_PDK_MAJOR)
  list(GET PDK_VERSION_LIST 1 VIBRANTE_PDK_MINOR)
  if(PDK_VERSION_LIST_LENGTH GREATER 2)
    list(GET PDK_VERSION_LIST 2 VIBRANTE_PDK_PATCH)
  endif()

  if(PDK_VERSION_LIST_LENGTH GREATER 3)
    list(GET PDK_VERSION_LIST 3 VIBRANTE_PDK_BUILD)
  endif()

  set(VIBRANTE_PDK_VERSION ${VIBRANTE_PDK_MAJOR}.${VIBRANTE_PDK_MINOR}.${VIBRANTE_PDK_PATCH}.${VIBRANTE_PDK_BUILD})

  add_definitions(-DVIBRANTE_PDK_VERSION=\"${VIBRANTE_PDK_VERSION}\") # requires escaping so it is treated as a string 
                                                                      # and not an invalid floating point with too many decimal points
  add_definitions(-DVIBRANTE_PDK_MAJOR=${VIBRANTE_PDK_MAJOR})
  add_definitions(-DVIBRANTE_PDK_MINOR=${VIBRANTE_PDK_MINOR})
  add_definitions(-DVIBRANTE_PDK_PATCH=${VIBRANTE_PDK_PATCH})
  add_definitions(-DVIBRANTE_PDK_BUILD=${VIBRANTE_PDK_BUILD})

  MATH(EXPR VIBRANTE_PDK_DECIMAL "${VIBRANTE_PDK_MAJOR} * 1000000 + \
                                  ${VIBRANTE_PDK_MINOR} * 10000 + \
                                  ${VIBRANTE_PDK_PATCH} * 100 + \
                                  ${VIBRANTE_PDK_BUILD}")
  add_definitions(-DVIBRANTE_PDK_DECIMAL=${VIBRANTE_PDK_DECIMAL})

  message(STATUS "Vibrante version ${VIBRANTE_PDK_VERSION}")
endif()

message(STATUS "Vibrante version decimal ${VIBRANTE_PDK_DECIMAL}")

set(PDK_LIBRARIES
  vibrante
  nvmedia
  nvmedia_dla
  nvmedia_tensor)
