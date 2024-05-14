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
// SPDX-FileCopyrightText: Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVRADAR_PROPERTIES_H_
#define NVRADAR_PROPERTIES_H_

// STL
#include <cmath>

// DW
#include <dw/core/base/Types.h>

namespace dw
{
namespace plugin
{
namespace radar
{

//################################################################################
//################ Sensor specific parameters and data structures ################
//################################################################################

//GENERAL
static const uint32_t PACKETS_PER_SCAN        = 2;
static const uint16_t MAX_PACKET_SIZE         = 1024;
static const uint16_t LAST_PACKET_SIZE        = 992;
static const uint16_t MAX_SCANS_PER_SECOND    = 20;
static const uint16_t MAX_DETECTIONS_PER_SCAN = 100;
static const uint32_t MAX_POINTS_PER_PACKET   = std::ceil(MAX_DETECTIONS_PER_SCAN / PACKETS_PER_SCAN);
static const uint32_t PAYLOAD_OFFSET          = sizeof(uint32_t) + sizeof(dwTime_t);
static const uint16_t MAX_SCAN_SIZE           = (PACKETS_PER_SCAN - 1) * MAX_PACKET_SIZE + LAST_PACKET_SIZE;
static const uint32_t MAX_BUFFER_SIZE         = PACKETS_PER_SCAN * MAX_PACKET_SIZE;

//DATA STRUCTURES
#pragma pack(1)
typedef struct
{
    uint8_t rawData[MAX_PACKET_SIZE + PAYLOAD_OFFSET];
} rawPacket;

typedef struct
{
    float32_t radius;          // 4 Bytes [meter]
    float32_t radial_velocity; // 4       [meter/second]
    float32_t azimuth_angle;   // 4       [rad]
    float32_t rcs;             // 4       [dB]
    float32_t elevation_angle; // 4       [rad]
} RadarDetection;              // 20

typedef struct
{
    dwTime_t sensor_timestamp;                   // 8
    uint32_t num_returns;                        // 4
    float32_t doppler_ambiguity;                 // 4
    RadarDetection det[MAX_DETECTIONS_PER_SCAN]; // MAX_DETECTIONS_PER_SCAN * sizeof(RadarDetection)
} RadarOutput;                                   // 16 + MAX_DETECTIONS_PER_SCAN * sizeof(RadarDetection) = 2016
#pragma pack()

} // namespace radar
} // namespace plugin
} // namespace dw

#endif
