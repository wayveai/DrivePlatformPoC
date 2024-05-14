/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2021-2023 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
///////////////////////////////////////////////////////////////////////////////////////
#ifndef NVLIDAR_PROPERTIES_H_
#define NVLIDAR_PROPERTIES_H_
#include <dw/core/base/Types.h>
#include <cmath>

namespace dw
{
namespace plugin
{
namespace lidar
{

//################################################################################
//################ Sensor specific parameters and data structures ################
//################################################################################

//GENERAL
static const uint32_t MAX_POINTS_PER_SPIN  = 50000;
static const uint32_t POINT_STRIDE         = 4U; // x, y, z and intensity
static const uint16_t MAX_UDP_PAYLOAD_SIZE = 4096;
static const uint16_t LAST_PACKET_SIZE     = 273;
static const uint32_t PACKETS_PER_SLICE    = 15;
static const uint16_t MAX_SPINS_PER_SECOND = 30;
static const uint32_t MAX_POINTS_PER_SLICE = 1800;
static const uint32_t MAX_PAYLOAD_SIZE     = (PACKETS_PER_SLICE - 1) * MAX_UDP_PAYLOAD_SIZE + LAST_PACKET_SIZE;
static const uint32_t MAX_BUFFER_SIZE      = PACKETS_PER_SLICE * MAX_UDP_PAYLOAD_SIZE;
static const uint16_t SLOT_SIZE            = 10;
static const uint32_t PAYLOAD_OFFSET       = sizeof(uint32_t) + sizeof(dwTime_t);
static const uint32_t PACKETS_PER_SPIN     = PACKETS_PER_SLICE * static_cast<int>(std::ceil((float)MAX_POINTS_PER_SPIN / (float)MAX_POINTS_PER_SLICE));

//DATA STRUCTURES
#pragma pack(1)
typedef struct
{
    uint8_t rawData[MAX_UDP_PAYLOAD_SIZE + PAYLOAD_OFFSET];
} rawPacket;

typedef struct
{
    //Cartesian coordinates
    float x;         // 4 Bytes [m]
    float y;         // 4       [m]
    float z;         // 4       [m]
    float intensity; // 4       [0.0-1.0]
} LidarPointXYZI;    // 16

typedef struct
{
    //Polar coordinates
    float theta;     // 4 [m]
    float phi;       // 4 [m]
    float radius;    // 4 [m]
    float intensity; // 4 [0.0-1.0]
} LidarPointRTHI;    // 16

typedef struct
{
    bool scan_complete;                        // 1
    dwTime_t sensor_timestamp;                 // 8
    uint32_t max_points_scan;                  // 4
    uint32_t n_points;                         // 4
    LidarPointRTHI rthi[MAX_POINTS_PER_SLICE]; // MAX_POINTS_PER_SLICE * sizeof(LidarPointRTHI)
    LidarPointXYZI xyzi[MAX_POINTS_PER_SLICE]; // MAX_POINTS_PER_SLICE * sizeof(LidarPointXYZI)
} LidarPacket;                                 // 1 + 8 + 4 + 4 + MAX_POINTS_PER_SLICE * 16 + MAX_POINTS_PER_SLICE * 16 = 57617
#pragma pack()

} // namespace lidar
} // namespace plugin
} // namespace dw

#endif
