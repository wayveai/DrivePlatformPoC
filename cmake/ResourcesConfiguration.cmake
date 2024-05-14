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
# SPDX-FileCopyrightText: Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

if(NOT TRT_VERSION)
    if(VIBRANTE_PDK_VERSION)
        if(VIBRANTE_PDK_VERSION VERSION_EQUAL 5.1.12.0)
            set(TRT_VERSION 6.0.0.11)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 5.1.15.0 OR VIBRANTE_PDK_VERSION VERSION_EQUAL 5.1.15.2)
            set(TRT_VERSION 6.2.0.3)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 5.2.0.0)
            set(TRT_VERSION 6.3.1.3)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 5.2.3.0)
            set(TRT_VERSION 6.4.0.6)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 5.2.6.0)
            set(TRT_VERSION 6.5.0.7)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.0.0)
            set(TRT_VERSION 7.2.1.6)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.1.0)
            set(TRT_VERSION 8.1.0.3)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.1.1)
            set(TRT_VERSION 8.1.0.6)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.1.2)
            set(TRT_VERSION 8.3.0.6)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.2.0)
            set(TRT_VERSION 8.3.0.10)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.2.1)
            set(TRT_VERSION 8.3.0.12)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.3.0)
            set(TRT_VERSION 8.4.10.4)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.3.1)
            set(TRT_VERSION 8.4.10.5)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.4.0)
            set(TRT_VERSION 8.4.11.6)
        elseif(VIBRANTE_PDK_VERSION VERSION_EQUAL 6.0.5.0)
            set(TRT_VERSION 8.4.12.5)
        elseif(VIBRANTE_PDK_VERSION VERSION_GREATER_EQUAL 6.0.6.0)
            set(TRT_VERSION 8.5.10.4)
        else()
            message(FATAL_ERROR "ResourcesConfiguration.cmake: Unknown PDK version ${VIBRANTE_PDK_VERSION}")
        endif()
    else()
        set(TRT_VERSION)
    endif()
endif()

if(TRT_VERSION)
    if(TRT_VERSION VERSION_EQUAL 6.0.0.11)
        set(CUDNN_VERSION 7.6.3)
    elseif(TRT_VERSION VERSION_EQUAL 6.2.0.3 OR TRT_VERSION VERSION_EQUAL 6.3.1.3 OR TRT_VERSION VERSION_EQUAL 6.4.0.6 OR TRT_VERSION VERSION_EQUAL 6.5.0.7)
        set(CUDNN_VERSION 7.6.6)
    elseif(TRT_VERSION VERSION_EQUAL 7.1.0.5)
        set(CUDNN_VERSION 8.0.0)
    elseif(TRT_VERSION VERSION_EQUAL 7.2.1.6)
        set(CUDNN_VERSION 8.0.4)
    elseif(TRT_VERSION VERSION_EQUAL 8.1.0.3)
        set(CUDNN_VERSION 8.2.5)
    elseif(TRT_VERSION VERSION_EQUAL 8.1.0.6 OR TRT_VERSION VERSION_EQUAL 8.3.0.6 OR TRT_VERSION VERSION_EQUAL 8.3.0.10 OR TRT_VERSION VERSION_EQUAL 8.3.0.12)
        set(CUDNN_VERSION 8.2.6)
    elseif(TRT_VERSION VERSION_EQUAL 8.4.10.4 OR TRT_VERSION VERSION_EQUAL 8.4.10.5)
        set(CUDNN_VERSION 8.3.3)
    elseif(TRT_VERSION VERSION_EQUAL 8.4.11.6)
        set(CUDNN_VERSION 8.4.1)
    elseif(TRT_VERSION VERSION_EQUAL 8.4.12.5)
        set(CUDNN_VERSION 8.4.12)
    elseif(TRT_VERSION VERSION_GREATER_EQUAL 8.5.10.4)
        set(CUDNN_VERSION 8.6.0)
    else()
        message(FATAL_ERROR "ResourcesConfiguration.cmake: Unknown TensorRT version ${TRT_VERSION}")
    endif()
else()
    set(CUDNN_VERSION)
endif()
