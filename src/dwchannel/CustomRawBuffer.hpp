/////////////////////////////////////////////////////////////////////////////////////////
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
// expressly authorized by NVIDIA. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA CORPORATION & AFFILIATES.
//
// SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef CUSTOM_RAW_BUFFER_HPP
#define CUSTOM_RAW_BUFFER_HPP

#include <dw/core/base/Types.h>
#include <dwcgf/channel/ChannelPacketTypes.hpp>

// Enumerate types of memory for raw buffer
enum MemoryType
{
    CPU,
    CUDA
};

/**
 * @brief CustomRawBuffer is a simple header that points to a contiguous chunk of memory.
 *
 */
struct CustomRawBuffer
{
    dwTime_t timestamp;
    /**
     * if memoryType = CPU, buffer is a host pointer and if memoryType = CUDA, buffer is device pointer.
     */
    MemoryType memoryType;
    /**
     * The pointer to the raw buffer memory
     */
    void* buffer;
    /**
     * The capacity in bytes of the raw buffer memory
     */
    size_t capacity;
    /**
     * The current size of this buffer.
     */
    size_t size;
};

/**
 * @brief Declare the ChannelPacketTypeID for CustomRawBuffer
 *
 * This declaration should not be in the range of ids that will be taken by CGF. Defining it equal to
 * DWFRAMEWORK_MAX_INTERNAL_TYPES ensures thatit won't conflict. In order to manage all the type IDs it is
 * recommended to group all the declarations into an enum. However, as this sample is only defining a sinlge
 * type the enum is not needed.
 */
constexpr dw::framework::ChannelPacketTypeID CustomRawBufferTypeID = dw::framework::DWFRAMEWORK_MAX_INTERNAL_TYPES;

/**
 * Declare CustomRawBuffer as a non-POD type, with SpecimenType CustomRawBuffer meaning that it will be expected that
 * the application pass a CustomRawBuffer as a specimen to be able to allocate CustomRawBuffer. When used as a specimen,
 * CustomRawBuffer::buffer will be ignored. CustomRawBuffer::memoryType and CustomRawBuffer::size will be used to know how much
 * and what kind of memory to allocate.
 */
DWFRAMEWORK_DECLARE_PACKET_TYPE_RELATION(CustomRawBuffer, CustomRawBuffer, CustomRawBufferTypeID);

#endif // CUSTOM_RAW_BUFFER_HPP