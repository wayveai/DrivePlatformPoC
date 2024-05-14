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
# SPDX-FileCopyrightText: Copyright (c) 2018-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

project(stm_test_vpu C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(SOURCES
    vpu/VpuTestContext.cpp
    main.cpp
)

include_directories(vpu)
include_directories(vpu/vpu_kernel)

if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    find_library(VPU_KERNEL libmat_add_c_lib_qnx.a vpu/vpu_kernel vpu)
else()
    find_library(VPU_KERNEL libmat_add_c_lib_linux.a vpu/vpu_kernel vpu)
endif()
if(NOT VPU_KERNEL)
    message(FATAL_ERROR "vpu kernel for vpu_simple not found.")
endif()

find_library(PVA_HOST_LIB cupva_host ${STM_BASE_DIR}/lib)
if(NOT PVA_HOST_LIB)
    message(FATAL_ERROR "cupva host library not found.")
endif()

set(LIBRARIES
    nvscibuf
    ${VPU_KERNEL}
    ${PVA_HOST_LIB}
    ${PDK_LIBRARIES}
    ${STM_DEPENDENT_LIBS}
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE "${LIBRARIES}")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${STM_TEST_CURRENT_BINARY_DIR}
        LIBRARY DESTINATION ${STM_TEST_CURRENT_LIBRARY_DIR}
        ARCHIVE DESTINATION ${STM_TEST_CURRENT_ARCHIVE_DIR}
)