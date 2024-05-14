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
# Macro MakeCUDASourceLibs
#
# - Separates cuda sources from normal sources
# - Creates a static lib for each each cuda source
# - Returns the remaining sources and the created libs
macro(MakeCUDASourceLibs OUT_SOURCES_CPP_VAR
                         OUT_CUDA_SOURCE_OBJECTS
                         OUT_CUDA_SOURCE_LIBS
                         TARGET_NAME
                         ALL_SOURCES
                         BASE_FOLDER)
    #Split into cuda and non-cuda files
    set(SOURCES_CUDA)
    set(${OUT_SOURCES_CPP_VAR})
    foreach(source_name ${ALL_SOURCES})
        if("${source_name}" MATCHES "\.cu$")
            list(APPEND SOURCES_CUDA "${source_name}")
        else()
            list(APPEND ${OUT_SOURCES_CPP_VAR} "${source_name}")
        endif()
    endforeach()

    set(${OUT_CUDA_SOURCE_LIBS}) # These should be made dependencies of the main target
    set(${OUT_CUDA_SOURCE_OBJECTS}) # These should be made dependencies of the main target
    if(CMAKE_GENERATOR MATCHES "^Visual Studio")
        # Create cuda targets
        # Note: each cuda file is an independent static lib to enable parallel compilaiton
        #       in the visual studio IDE
        # Note: this cannot be used for linux because linux complains about undefined references
        #       between static libs. Windows waits for the final link to check for undefined refs.
        foreach(FILE ${SOURCES_CUDA})
            set(lib_name_base lib_${FILE})
            string(REPLACE "/" "_" lib_name_base ${lib_name_base})
            string(REPLACE "." "_" lib_name_base ${lib_name_base})
            string(REPLACE "_cu" "" lib_name_base ${lib_name_base})

            # ensure unique name of cuda target
            if(TARGET ${lib_name_base})
                set(targetNum 0)
                while(TARGET ${lib_name_base}${targetNum})
                    math(EXPR targetNum ${targetNum}+1)
                endwhile()
                set(lib_name ${lib_name_base}${targetNum})
            else()
                set(lib_name ${lib_name_base})
            endif()

            cuda_add_library(${lib_name} STATIC ${FILE})
            set_property(TARGET ${lib_name} PROPERTY FOLDER "${BASE_FOLDER}/CUDA")

            list(APPEND ${OUT_CUDA_SOURCE_LIBS} ${lib_name})
        endforeach()
    else()
        # Create cuda objects
        # Note: it would be ideal to create a single object target per module with all objects
        #       but cmake doesn't allow to add object files to an object target.
        #       So to have both static module lib and shared dw lib use the same objects, this is needed.
        cuda_compile(CUDA_OBJECTS ${SOURCES_CUDA} SHARED)
        list(APPEND ${OUT_CUDA_SOURCE_OBJECTS} ${CUDA_OBJECTS})
    endif()
endmacro()
