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

#-------------------------------------------------------------------------------
# Samples Installation configuration
#-------------------------------------------------------------------------------
set(SDK_SAMPLE_DESTINATION  "bin")
set(SDK_LIBRARY_DESTINATION "lib")
set(SDK_ARCHIVE_DESTINATION "lib")

function(sdk_add_sample SAMPLE)
    install(TARGETS ${SAMPLE}
        RUNTIME DESTINATION ${SDK_SAMPLE_DESTINATION}
        LIBRARY DESTINATION ${SDK_LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${SDK_ARCHIVE_DESTINATION}
    )
endfunction()

function(sdk_install_sample_library SAMPLE_LIB)
    install(TARGETS ${SAMPLE_LIB}
        DESTINATION ${SDK_SAMPLE_DESTINATION}
    )
endfunction()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX
        "${SDK_BINARY_DIR}/install/usr/local/driveworks/samples" CACHE PATH
        "Install path prefix, prepended onto install directories." FORCE
    )
endif()

if(NOT CMAKE_LIBRARY_ARCHITECTURE MATCHES "^aarch64-(linux-gnu|unknown-nto-qnx)$" AND
    CMAKE_INSTALL_PREFIX STREQUAL "${SDK_BINARY_DIR}/install/usr/local/driveworks/samples"
)
    install(CODE "
        message(STATUS
            \"Create symlink from \\\"${SDK_BINARY_DIR}/install/usr/local/driveworks/data\\\" \"
            \"to \\\"/usr/local/driveworks/data\\\"\"
        )
        file(MAKE_DIRECTORY
            \"${SDK_BINARY_DIR}/install/usr/local/driveworks\"
        )
        file(CREATE_LINK
             /usr/local/driveworks/data
             \"${SDK_BINARY_DIR}/install/usr/local/driveworks/data\"
             SYMBOLIC
        )
    ")
endif()

if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "^aarch64-(linux-gnu|unknown-nto-qnx)$")
    set(CMAKE_INSTALL_RPATH /usr/local/driveworks/samples/bin /usr/local/driveworks/lib)
else()
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${SDK_SAMPLE_DESTINATION}")
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

if(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-linux-gnu)
    message(STATUS
        "**** Please copy the contents of `${CMAKE_INSTALL_PREFIX}/${SDK_SAMPLE_DESTINATION}' "
        "on the host filesystem to `/usr/local/driveworks/samples/bin' on the "
        "target filesystem. ****"
    )
elseif(CMAKE_LIBRARY_ARCHITECTURE STREQUAL aarch64-unknown-nto-qnx)
    message(STATUS
        "**** Please mount `${CMAKE_INSTALL_PREFIX}/${SDK_SAMPLE_DESTINATION}' "
        "on the host filesystem onto `/usr/local/driveworks/samples/bin' on "
        "the target filesystem using NFS. ****"
    )
else()
    message(STATUS
        "**** Samples will be installed to `${CMAKE_INSTALL_PREFIX}/${SDK_SAMPLE_DESTINATION}' "
        "on the host filesystem. ****"
        )
endif()
