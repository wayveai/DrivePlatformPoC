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
// SPDX-FileCopyrightText: Copyright (c) 2015-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "interpreter.hpp"
#include <cstdio>
#include <iostream>
#include <algorithm>

struct State
{
    // Speed_ms
    float32_t speed_ms; // Speed (m/s)
    dwTime_t speed_ts;
    bool speed_newdata;

    // Steering_rad
    float32_t steering_rad; // Steering (rad)
    dwTime_t steering_ts;
    bool steering_newdata;
};

static State g_state{};

// -----------------------------------------------------------------------------
uint32_t smpl_getNumAvailableSignals(void*)
{
    // our own CAN data (which is interpretable by this interpreter) stores only up-to 1 signal per message
    return g_state.speed_newdata || g_state.steering_newdata ? 1 : 0;
}

// -----------------------------------------------------------------------------
bool smpl_getSignalInfo(const char** name, dwTrivialDataType* type, dwCANVehicleData* data,
                        uint32_t idx, void* userData)
{
    (void)idx;
    (void)userData;

    if (g_state.speed_newdata)
    {
        *type = DW_TYPE_FLOAT32;
        *data = DW_CAN_CAR_SPEED;
        *name = CAN_CAR_SPEED.c_str();
    }
    else if (g_state.steering_newdata)
    {
        *type = DW_TYPE_FLOAT32;
        *data = DW_CAN_STEERING_ANGLE;
        *name = CAN_CAR_STEERING.c_str();
    }
    else
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
bool smpl_getDataf32(float32_t* value, dwTime_t* timestamp, uint32_t idx, void* userData)
{
    (void)idx;
    (void)userData;

    if (g_state.speed_newdata)
    {
        *value                = static_cast<float32_t>(g_state.speed_ms);
        *timestamp            = g_state.speed_ts;
        g_state.speed_newdata = false;

        return true;
    }
    else if (g_state.steering_newdata)
    {
        *value                   = static_cast<float32_t>(g_state.steering_rad / 16.);
        *timestamp               = g_state.steering_ts;
        g_state.steering_newdata = false;

        return true;
    }

    return false;
}
// -----------------------------------------------------------------------------
bool smpl_getDatai32(int32_t* value, dwTime_t* timestamp, uint32_t idx, void* userData)
{
    (void)idx;
    (void)value;
    (void)userData;
    (void)timestamp;

    return false;
}

// -----------------------------------------------------------------------------
void smpl_addMessage(const dwCANMessage* msg, void* userData)
{
    (void)userData;

    // we are expecting float values to be stored in payload
    if (msg->size < 4)
    {
        std::cerr << "CAN interpreter: expected payload with at least 4 bytes" << std::endl;
        return;
    }

    union _Data
    {
        uint8_t byte[8];
        float32_t fval;
        float64_t dval;
        int32_t ival;
    } data;

    memcpy(data.byte, msg->data, std::min(msg->size, uint16_t{8}));

    g_state.speed_newdata    = false;
    g_state.steering_newdata = false;

    // in the default implementation we use same CAN ids as vehicle data ids
    switch (msg->id)
    {

    // SG_ CAN_CAR_SPEED : 0|32@1- (1E-005,0) [-100000|100000] "m/s"  SAMPLE
    case DW_CAN_CAR_SPEED:
        g_state.speed_newdata = true;
        g_state.speed_ts      = msg->timestamp_us;
        g_state.speed_ms      = static_cast<float32_t>(data.ival) * 1e-05f;

        break;

    // SG_ CAN_CAR_STEERING : 0|32@1- (6.25E-007,0) [-100000|100000] "rad"  SAMPLE
    case DW_CAN_STEERING_ANGLE:
        g_state.steering_newdata = true;
        g_state.steering_ts      = msg->timestamp_us;
        g_state.steering_rad     = static_cast<float32_t>(data.ival) * 6.25e-7f * 16.f; // our sample data stores angles divided by 16
        break;

    default:
        break;
    }
}
