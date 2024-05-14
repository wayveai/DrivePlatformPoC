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

#-------------------------------------------------------------------------------
# Build options
#-------------------------------------------------------------------------------
set(STM_CUDA_CHECKS            OFF CACHE BOOL "Build STM with enabled CUDA checks")

#-------------------------------------------------------------------------------
# Build flags
#-------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    # disable variable length array, see MISRA C:2012 17.2
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=vla")

    # FIXME: disabled as nvrm_channel.h has dangerous format conversions
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

    # extra warnings to be closer to MISRA compliance
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wuninitialized -Wunused -Wunused-value -Wunused-parameter -Wextra -Wno-missing-field-initializers -Wswitch-enum -Wswitch-default -Wshadow -Wfloat-equal")

    if(NOT VIBRANTE)
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined -Wl,--as-needed")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined -Wl,--as-needed")
    endif()

    if(STM_BUILD_EXPERIMENTAL OR CMAKE_BUILD_TYPE MATCHES Debug)
        # Make functions visible so there is backtrace information
        set(STM_MODULE_CXX_FLAGS "-fvisibility=default -funwind-tables -fasynchronous-unwind-tables")
    else()
        # Hide as much as possible from a non-experimental release
        set(STM_MODULE_CXX_FLAGS "-fvisibility=hidden")
    endif()

    # force C99 standard for C compiler. C compiler is used to test public headers, hence enforce C99 on public headers
    set(CMAKE_C_STANDARD 99)
endif()

#-------------------------------------------------------------------------------
# CUDA configuration
#-------------------------------------------------------------------------------
# Remove warnings from Eigen headers
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-Xcudafe \"--diag_suppress=useless_using_declaration --diag_suppress=set_but_not_used --diag_suppress=never_defined --diag_suppress=noreturn_function_does_return --diag_suppress=decl_modifiers_ignored --diag_suppress=boolean_controlling_expr_is_constant\"")

# Add debug info
# Note: This is disabled because it takes very long to do JIT compilation of kernels
#       with debug info. Enable this if you intend to do CUDA debugging.
# set(CUDA_NVCC_FLAGS_DEBUG "--debug;--device-debug;${CUDA_NVCC_FLAGS_DEBUG}")
