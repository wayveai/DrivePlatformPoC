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

#############
# Macro IncludeTargetDirectories
#    target - Name of the target
#
# Adds include directories from the target globally via include_directories()
# Necessary for cuda files
macro(IncludeTargetDirectories targets)
    foreach(target ${targets})
        if(TARGET ${target})
            get_property(system_includes_set TARGET ${target} PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES SET)
            if(${system_includes_set})
              get_property(system_includes_set TARGET ${target} PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
              include_directories(SYSTEM ${system_includes_set})
              CUDA_SYSTEM_INCLUDE_DIRECTORIES(${system_includes_set})
            endif()

            get_property(includes_set TARGET ${target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES SET)
            if(${includes_set})
                get_property(includes TARGET ${target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
                include_directories(${includes})
            endif()
        endif()
    endforeach()
endmacro()
