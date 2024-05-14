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
// SPDX-FileCopyrightText: Copyright (c) 2015-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_SENSORS_CANBUS_LOGGER_INTERPRETER_HPP_
#define SAMPLES_SENSORS_CANBUS_LOGGER_INTERPRETER_HPP_

#include <dw/sensors/canbus/VehicleData.h>
#include <dw/sensors/canbus/CAN.h>

#include <cstddef>
#include <string>
#include <string.h>

// -----------------------------------------------------------------------------
// CAN bus codes
// A set of virtual CAN bus codes to simulate virtual car
// -----------------------------------------------------------------------------
static std::string CAN_CAR_SPEED    = "M_SPEED.CAN_CAR_SPEED";
static std::string CAN_CAR_STEERING = "M_STEERING.CAN_CAR_STEERING";

// -----------------------------------------------------------------------------
void smpl_addMessage(const dwCANMessage* msg, void* data);

uint32_t smpl_getNumAvailableSignals(void* data);
bool smpl_getSignalInfo(const char** name, dwTrivialDataType* type, dwCANVehicleData* data,
                        uint32_t idx, void* userData);

bool smpl_getDataf32(float32_t* value, dwTime_t* timestamp, uint32_t idx, void* data);
bool smpl_getDatai32(int32_t* value, dwTime_t* timestamp, uint32_t idx, void* data);

#endif // SAMPLES_SENSORS_CANBUS_LOGGER_INTERPRETER_HPP_
