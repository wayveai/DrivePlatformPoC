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
# SPDX-FileCopyrightText: Copyright (c) 2016-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_VERSION 7.1.0)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(ARCH 8.3.0,gcc_ntoaarch64le)
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-unknown-nto-qnx)

# QNX-required environment variables
if(NOT DEFINED ENV{QNX_HOST} OR NOT DEFINED ENV{QNX_TARGET})
    message(FATAL_ERROR "Need to define QNX_HOST and QNX_TARGET")
endif()

set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES CUDA_DIR)
if(NOT DEFINED CUDA_DIR)
    message(FATAL_ERROR "Need to define CUDA_DIR")
endif()

set(QNX_HOST "$ENV{QNX_HOST}")
set(QNX_TARGET "$ENV{QNX_TARGET}")

# QNX-required definitions & compiler flags
set(QNX_TOOLCHAIN_PATH "${QNX_HOST}/usr/bin")
set(QNX_TOOLCHAIN_TRIPLE "aarch64-unknown-nto-qnx7.1.0")
set(QNX_TOOLCHAIN_PREFIX "${QNX_TOOLCHAIN_PATH}/${QNX_TOOLCHAIN_TRIPLE}")
add_compile_definitions(
    _FILE_OFFSET_BITS=64
    _POSIX_C_SOURCE=200112L
    _QNX_SOURCE
    FD_SETSIZE=1024
    WIN_INTERFACE_CUSTOM
)

if(NOT VIBRANTE_PDK)
    if(ENV{VIBRANTE_PDK})
        set(VIBRANTE_PDK "$ENV{VIBRANTE_PDK}")
    else()
        set(_PDK_TOP_DIRS)
        if(ENV{NV_WORKSPACE})
            list(APPEND _PDK_TOP_DIRS "$ENV{NV_WORKSPACE}")
        endif()
        if(ENV{PDK_TOP})
            list(APPEND _PDK_TOP_DIRS "$ENV{PDK_TOP}")
        endif()
        list(APPEND _PDK_TOP_DIRS
            /drive
            /rfs
            /usr/local/driveos-sdk
        )
        string(TOLOWER "${CMAKE_SYSTEM_NAME}" _SYSTEM_NAME_LOWER)
        set(VIBRANTE_PDK)
        foreach(_PDK_TOP_DIR ${_PDK_TOP_DIRS})
            if(EXISTS "${_PDK_TOP_DIR}/drive-${_SYSTEM_NAME_LOWER}")
                set(VIBRANTE_PDK
                    "${_PDK_TOP_DIR}/drive-${_SYSTEM_NAME_LOWER}"
                )
                break()
            endif()
            if(EXISTS "${_PDK_TOP_DIR}/drive-t234ref-${_SYSTEM_NAME_LOWER}")
                set(VIBRANTE_PDK
                    "${_PDK_TOP_DIR}/drive-t234ref-${_SYSTEM_NAME_LOWER}"
                )
                break()
            endif()
        endforeach()
        unset(_PDK_TOP_DIRS)
        unset(_SYSTEM_NAME_LOWER)
    endif()
    set(VIBRANTE_PDK "${VIBRANTE_PDK}" CACHE PATH
        "Path to ${CMAKE_SYSTEM_NAME} PDK for cross-compilation." FORCE
    )
endif()

if(NOT VIBRANTE_PDK)
    message(FATAL_ERROR
        "Could NOT find ${CMAKE_SYSTEM_NAME} PDK for cross-compilation. Path "
        "must be specified using the cache variable VIBRANTE_PDK."
    )
endif()

list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES VIBRANTE_PDK)

message(STATUS "VIBRANTE_PDK = ${VIBRANTE_PDK}")

set(VIBRANTE ON)
set(VIBRANTE_V5Q ON)
add_definitions(-DVIBRANTE -DVIBRANTE_V5Q -DNVMEDIA_NVSCI_ENABLE)

if(EXISTS "${VIBRANTE_PDK}/lib-target/version-pdk.txt")
    set(_VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-pdk.txt")
elseif(EXISTS "${VIBRANTE_PDK}/lib-target/version-sdk.txt")
    set(_VIBRANTE_PDK_FILE "${VIBRANTE_PDK}/lib-target/version-sdk.txt")
else()
    message(FATAL_ERROR
        "Could NOT open file \"${VIBRANTE_PDK}/lib-target/version-(pdk/sdk).txt\" for PDK branch detection"
    )
endif()
file(READ "${_VIBRANTE_PDK_FILE}" _VERSION_NV_PDK)
unset(_VIBRANTE_PDK_FILE)

if(_VERSION_NV_PDK MATCHES "^(.+)-[0123456789]+")
    set(_VIBRANTE_PDK_BRANCH "${CMAKE_MATCH_1}")
else()
    message(FATAL_ERROR
        "Could NOT determine PDK branch for PDK \"${VIBRANTE_PDK}\""
    )
endif()
unset(_VERSION_NV_PDK)

string(REPLACE "." ";" _VIBRANTE_PDK_VERSION_LIST "${_VIBRANTE_PDK_BRANCH}")
unset(_VIBRANTE_PDK_BRANCH)

list(LENGTH _VIBRANTE_PDK_VERSION_LIST _VIBRANTE_PDK_VERSION_LIST_LENGTH)
while(_VIBRANTE_PDK_VERSION_LIST_LENGTH LESS 4)
    list(APPEND VIBRANTE_PDK_VERSION_LIST 0)
    list(LENGTH _VIBRANTE_PDK_VERSION_LIST _VIBRANTE_PDK_VERSION_LIST_LENGTH)
endwhile()
unset(_VIBRANTE_PDK_VERSION_LIST_LENGTH)

list(GET _VIBRANTE_PDK_VERSION_LIST 0 _VIBRANTE_PDK_MAJOR)
list(GET _VIBRANTE_PDK_VERSION_LIST 1 _VIBRANTE_PDK_MINOR)
list(GET _VIBRANTE_PDK_VERSION_LIST 2 _VIBRANTE_PDK_PATCH)
list(GET _VIBRANTE_PDK_VERSION_LIST 3 _VIBRANTE_PDK_BUILD)
unset(_VIBRANTE_PDK_VERSION_LIST)

if(_VIBRANTE_PDK_MAJOR MATCHES "^([0-9]+)")
    set(VIBRANTE_PDK_MAJOR "${CMAKE_MATCH_1}")
else()
    message(FATAL_ERROR "Invalid PDK major version \"${_VIBRANTE_PDK_MAJOR}\"")
endif()
unset(_VIBRANTE_PDK_MAJOR)

if(_VIBRANTE_PDK_MINOR MATCHES "^([0-9]+)")
    set(VIBRANTE_PDK_MINOR "${CMAKE_MATCH_1}")
else()
    message(FATAL_ERROR "Invalid PDK minor version \"${_VIBRANTE_PDK_MINOR}\"")
endif()
unset(_VIBRANTE_PDK_MINOR)

if(_VIBRANTE_PDK_PATCH MATCHES "^([0-9]+)")
    set(VIBRANTE_PDK_PATCH "${CMAKE_MATCH_1}")
else()
    message(FATAL_ERROR "Invalid PDK patch version \"${_VIBRANTE_PDK_PATCH}\"")
endif()
unset(_VIBRANTE_PDK_PATCH)

if(_VIBRANTE_PDK_BUILD MATCHES "^([0-9]+)")
    set(VIBRANTE_PDK_BUILD "${CMAKE_MATCH_1}")
else()
    message(FATAL_ERROR "Invalid PDK build version \"${_VIBRANTE_PDK_BUILD}\"")
endif()
unset(_VIBRANTE_PDK_PATCH)

set(VIBRANTE_PDK_VERSION
    "${VIBRANTE_PDK_MAJOR}.${VIBRANTE_PDK_MINOR}.${VIBRANTE_PDK_PATCH}.${VIBRANTE_PDK_BUILD}"
)

math(EXPR VIBRANTE_PDK_DECIMAL
    "${VIBRANTE_PDK_MAJOR} * 1000000 + ${VIBRANTE_PDK_MINOR} * 10000 + ${VIBRANTE_PDK_PATCH} * 100 + ${VIBRANTE_PDK_BUILD}"
)

message(STATUS "Found PDK version ${VIBRANTE_PDK_VERSION} (${VIBRANTE_PDK_DECIMAL})")

# VIBRANTE_PDK_VERSION requires escaping so that it is treated as a string and
# not as an invalid floating point with too many decimal points.
add_compile_definitions(
    VIBRANTE_PDK_MAJOR=${VIBRANTE_PDK_MAJOR}
    VIBRANTE_PDK_MINOR=${VIBRANTE_PDK_MINOR}
    VIBRANTE_PDK_PATCH=${VIBRANTE_PDK_PATCH}
    VIBRANTE_PDK_BUILD=${VIBRANTE_PDK_BUILD}
    VIBRANTE_PDK_VERSION=\"${VIBRANTE_PDK_VERSION}\"
    VIBRANTE_PDK_DECIMAL=${VIBRANTE_PDK_DECIMAL}
)

if(NOT EXISTS $ENV{HOME}/.qnx/license/licenses)
    message(FATAL_ERROR
        "QNX Software Development Platform Developer License could NOT be "
        "found at ~/.qnx/license/licenses."
    )
endif()

get_filename_component(QNX_TOOLCHAIN_PATH "${QNX_TOOLCHAIN_PATH}" REALPATH)
set(CMAKE_C_COMPILER "${QNX_TOOLCHAIN_PATH}/qcc" CACHE FILEPATH
    "Path to the C compiler."
)
set(CMAKE_CXX_COMPILER "${QNX_TOOLCHAIN_PATH}/q++" CACHE FILEPATH
    "Path to the CXX compiler."
)

set(_CXX_FLAGS_BEFORE "-V${ARCH}")
set(_CXX_FLAGS_AFTER "-N8M")
set(CMAKE_C_FLAGS
    "${_CXX_FLAGS_BEFORE} ${CMAKE_C_FLAGS} ${_CXX_FLAGS_AFTER}"
)
set(CMAKE_CXX_FLAGS
    "${_CXX_FLAGS_BEFORE} ${CMAKE_CXX_FLAGS} ${_CXX_FLAGS_AFTER}"
)
unset(_CXX_FLAGS_BEFORE)

string(REPLACE " " "," _CXX_FLAGS_FOR_CUDA "${_CXX_FLAGS_AFTER}")
unset(_CXX_FLAGS_AFTER)

set(_CUDA_FLAGS_BEFORE "-qpp-config ${ARCH}")
set(_CUDA_FLAGS_AFTER "-Xcompiler ${_CXX_FLAGS_FOR_CUDA}")
unset(_CXX_FLAGS_FOR_CUDA)
set(CMAKE_CUDA_FLAGS
    "${_CUDA_FLAGS_BEFORE} ${CMAKE_CUDA_FLAGS} ${_CUDA_FLAGS_AFTER}"
)
unset(_CUDA_FLAGS_BEFORE)
unset(_CUDA_FLAGS_AFTER)


set(CMAKE_ADDR2LINE "${QNX_TOOLCHAIN_PREFIX}-addr2line" CACHE FILEPATH
    "Path to the tool to convert addresses into file names and line numbers."
)
set(CMAKE_AR "${QNX_TOOLCHAIN_PREFIX}-ar" CACHE FILEPATH
    "Path to the tool to  create, modify, and extract from archives."
)
set(CMAKE_LINKER "${QNX_TOOLCHAIN_PREFIX}-ld" CACHE FILEPATH
    "Path to the linker."
)
set(CMAKE_NM "${QNX_TOOLCHAIN_PREFIX}-nm" CACHE FILEPATH
    "Path to the tool to list symbols from object files."
)
set(CMAKE_OBJCOPY "${QNX_TOOLCHAIN_PREFIX}-objcopy" CACHE FILEPATH
    "Path to the tool to copy and translate object files."
)
set(CMAKE_OBJDUMP "${QNX_TOOLCHAIN_PREFIX}-objdump" CACHE FILEPATH
    "Path to the tool to display information from object files."
)
set(CMAKE_RANLIB "${QNX_TOOLCHAIN_PREFIX}-ranlib" CACHE FILEPATH
    "Path to the tool to generate an index to an archive."
)
set(CMAKE_READELF "${QNX_TOOLCHAIN_PREFIX}-readelf" CACHE FILEPATH
    "Path to the tool to display information about ELF files."
)
set(CMAKE_STRIP "${QNX_TOOLCHAIN_PREFIX}-strip" CACHE FILEPATH
    "Path to the tool to discard symbols and other data from object files."
)

# Set default library search path
set(CMAKE_SYSTEM_LIBRARY_PATH
    "${VIBRANTE_PDK}/lib-target"
    "${VIBRANTE_PDK}/nvidia-bsp/aarch64le/usr/lib"
    "${CUDA_DIR}/targets/aarch64-qnx/lib"
    "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
)
set(_LINKER_FLAGS_BEFORE "-V${ARCH}")
set(_LINKER_FLAGS_AFTER)
foreach(_LIBRARY_PATH ${CMAKE_SYSTEM_LIBRARY_PATH})
    string(APPEND _LINKER_FLAGS_AFTER
        " -L${_LIBRARY_PATH}"
        " -Wl,-rpath-link,${_LIBRARY_PATH}"
    )
endforeach()
set(CMAKE_EXE_LINKER_FLAGS
    "${_LINKER_FLAGS_BEFORE} ${CMAKE_EXE_LINKER_FLAGS} ${_LINKER_FLAGS_AFTER}"
)
set(CMAKE_MODULE_LINKER_FLAGS
    "${_LINKER_FLAGS_BEFORE} ${CMAKE_MODULE_LINKER_FLAGS} ${_LINKER_FLAGS_AFTER}"
)
set(CMAKE_SHARED_LINKER_FLAGS
    "${_LINKER_FLAGS_BEFORE} ${CMAKE_SHARED_LINKER_FLAGS} ${_LINKER_FLAGS_AFTER}"
)
unset(_LINKER_FLAGS_BEFORE)
unset(_LINKER_FLAGS_AFTER)

# Set cmake root path.
set(CMAKE_FIND_ROOT_PATH "${VIBRANTE_PDK}")

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)

# set system default include dir
#
# NOTE: Set PDK include BEFORE the  CUDA include. This is important for
# ensuring that PDK and CUDA directories are searched in a predictable
# deterministic manner. In the case of include files that exist in both
# places, this ensures that we always check PDK first, and THEN CUDA.
set(CMAKE_SYSTEM_INCLUDE_PATH
    "${VIBRANTE_PDK}/include"
    "${CUDA_DIR}/targets/aarch64-qnx/include"
    "/usr/include/${CMAKE_LIBRARY_ARCHITECTURE}"
)
include_directories(BEFORE SYSTEM ${CMAKE_SYSTEM_INCLUDE_PATH})
