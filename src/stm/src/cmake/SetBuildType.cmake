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
# This include makes sure that only (Debug|Release) build types are used
# If no build is specified for a single-configuration generator, Release is used
# by default.
#-------------------------------------------------------------------------------

#Single-configuration system, check build type
if(CMAKE_BUILD_TYPE)
    if(NOT "${CMAKE_BUILD_TYPE}" MATCHES "^(Debug|Release)$")
        message(WARNING "CMAKE_BUILD_TYPE must be one of (Debug|Release). Using Release as default.")
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type. Options: Debug,Release" FORCE)
    endif()
else()
    message(WARNING "CMAKE_BUILD_TYPE not defined. Using Release as default.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type. Options: Debug,Release")
endif()
