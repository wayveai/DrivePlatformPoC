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

project(samples_framework)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(SOURCES
    CameraFramePipeline.cpp
    CameraFramePipeline.hpp
    Checks.hpp
    CudaTimer.hpp
    DriveWorksSample.cpp
    DriveWorksSample.hpp
    DriveWorksSampleSeek.cpp
    DriveWorksSampleSeek.hpp
    Grid.cpp
    Grid.hpp
    Log.cpp
    Log.hpp
    Mat3.hpp
    Mat4.hpp
    MathUtils.cpp
    MathUtils.hpp
    MouseView3D.cpp
    MouseView3D.hpp
    ProfilerCUDA.cpp
    ProfilerCUDA.hpp
    ProgramArguments.cpp
    ProgramArguments.hpp
    RenderUtils.cpp
    RenderUtils.hpp
    SampleFramework.cpp
    SampleFramework.hpp
    SamplesDataPath.cpp
    SamplesDataPath.hpp
    ScreenshotHelper.cpp
    ScreenshotHelper.hpp
    SimpleCamera.cpp
    SimpleCamera.hpp
    SimpleRecordingPlayer.cpp
    SimpleRecordingPlayer.hpp
    SimpleRenderer.cpp
    SimpleRenderer.hpp
    SimpleStreamer.hpp
    StatsCounter.hpp
    Window.cpp
    Window.hpp
    WindowGLFW.cpp
    WindowGLFW.hpp
)

set(LIBRARIES
    drm
    glfw-src
    lodepng-src
    ${Driveworks_LIBRARIES}
)

if(DW_USE_EGL)
    list(APPEND SOURCES
         WindowEGL.cpp
         WindowEGL.hpp
    )

    if(VIBRANTE_V5Q)
        list(APPEND SOURCES
            WindowQNXEGL.cpp
            WindowQNXEGL.hpp
        )
        list(APPEND LIBRARIES screen)
    elseif(VIBRANTE_V5L OR LINUX)
        list(APPEND SOURCES
            WindowLinuxEGL.cpp
            WindowLinuxEGL.hpp
        )
    endif()
endif()

if(LINUX)
    list(APPEND LIBRARIES glew-src)
endif()

if(VIBRANTE)
    list(APPEND LIBRARIES
        gles
    )

    if(VIBRANTE_V5L)
        list(APPEND LIBRARIES
            vibrante
            vibrante_Xlibs
        )
    endif()
endif()

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBRARIES})
target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/..
                           ${CMAKE_CURRENT_SOURCE_DIR}/../..)

#-------------------------------------------------------------------------------
# Special aligned allocator library
#-------------------------------------------------------------------------------
add_library(samples_allocator STATIC Allocator.cpp)
target_link_libraries(samples_allocator PUBLIC dwdynamicmemory)
