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
# Build flags
#-------------------------------------------------------------------------------

# Enable lightweight support for detecting buffer overflows in various
# functions that perform operations on memory and strings.
add_compile_definitions(
    "$<$<CONFIG:Release>:_FORTIFY_SOURCE=2>"
)

include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)

set(_CXX_COMPILE_OPTIONS)

# Keep the frame pointer in a register for functions that do not need one.
check_c_compiler_flag(-fno-omit-frame-pointer
    C_COMPILER_FLAG_FNO_OMIT_FRAME_POINTER
)
check_cxx_compiler_flag(-fno-omit-frame-pointer
    CXX_COMPILER_FLAG_FNO_OMIT_FRAME_POINTER
)
if(C_COMPILER_FLAG_FNO_OMIT_FRAME_POINTER AND
        CXX_COMPILER_FLAG_FNO_OMIT_FRAME_POINTER
)
    list(APPEND _CXX_COMPILE_OPTIONS -fno-omit-frame-pointer)
endif()

# Disable vectorization.
check_c_compiler_flag(-fno-tree-vectorize
    C_COMPILER_FLAG_FNO_TREE_VECTORIZE
)
check_cxx_compiler_flag(-fno-tree-vectorize
    CXX_COMPILER_FLAG_FNO_TREE_VECTORIZE
)
if(C_COMPILER_FLAG_FNO_TREE_VECTORIZE AND
        CXX_COMPILER_FLAG_FNO_TREE_VECTORIZE
)
    list(APPEND _CXX_COMPILE_OPTIONS -fno-tree-vectorize)
endif()

# Emit extra code to check for buffer overflows, such as stack smashing attacks.
check_c_compiler_flag(-fstack-protector-strong
    C_COMPILER_FLAG_FSTACK_PROTECTOR_STRONG
)
check_cxx_compiler_flag(-fstack-protector-strong
    CXX_COMPILER_FLAG_FSTACK_PROTECTOR_STRONG
)
if(C_COMPILER_FLAG_FSTACK_PROTECTOR_STRONG AND
        CXX_COMPILER_FLAG_FSTACK_PROTECTOR_STRONG
)
    list(APPEND _CXX_COMPILE_OPTIONS -fstack-protector-strong)
else()
    check_c_compiler_flag(-fstack-protector
        C_COMPILER_FLAG_FSTACK_PROTECTOR
    )
    check_cxx_compiler_flag(-fstack-protector
        CXX_COMPILER_FLAG_FSTACK_PROTECTOR
    )
    if(C_COMPILER_FLAG_FSTACK_PROTECTOR AND CXX_COMPILER_FLAG_FSTACK_PROTECTOR)
        list(APPEND _CXX_COMPILE_OPTIONS -fstack-protector)
    endif()
endif()

# Enable errors about constructions that some users consider questionable and
# that are easy to avoid, or modify to prevent the error, even in conjunction
# with macros.
check_c_compiler_flag(-Werror=all CXX_COMPILER_FLAG_WERROR_ALL)
check_cxx_compiler_flag(-Werror=all CXX_COMPILER_FLAG_WERROR_ALL)
if(C_COMPILER_FLAG_WERROR_ALL AND CXX_COMPILER_FLAG_WERROR_ALL)
    list(APPEND _CXX_COMPILE_OPTIONS -Werror=all)
endif()

if(_CXX_COMPILE_OPTIONS)
    add_compile_options("$<$<COMPILE_LANGUAGE:C,CXX>:${_CXX_COMPILE_OPTIONS}>")
endif()

string(REPLACE ";" "," _CXX_COMPILE_OPTIONS_FOR_CUDA
    "${_CXX_COMPILE_OPTIONS}"
)
unset(_CXX_COMPILE_OPTIONS)
set(_CUDA_COMPILER_FLAGS
    -Werror cross-execution-space-call
    -Xcompiler ${_C_COMPILER_FLAGS_FOR_CUDA}
)
unset(_C_COMPILER_FLAGS_FOR_CUDA)

add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:${_CUDA_COMPILER_FLAGS}>")
unset(_CUDA_COMPILER_FLAGS)

#-------------------------------------------------------------------------------
# Configured headers
#-------------------------------------------------------------------------------
include_directories("${SDK_BINARY_DIR}/configured")
