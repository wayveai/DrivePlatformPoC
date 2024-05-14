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
// SPDX-FileCopyrightText: Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string>
#include <iostream>
#include <cstring>
#include <cmath>

#include <framework/SamplesDataPath.hpp>

#include <dw/core/base/Version.h>
#include <dw/control/vehicleio/plugins/VehicleIODriver.h>
#include <dw/sensors/canbus/Interpreter.h>
#include <framework/Checks.hpp>

#include "driverConf.hpp"

// exported functions
extern "C" {

static dwCANInterpreterHandle_t gIntp{};
static dwContextHandle_t gSdk = DW_NULL_HANDLE;

static void checkError(dwStatus& result, const dwStatus& status, const char* errorMessage)
{
    if (status != DW_SUCCESS)
        std::cerr << "VehicleIO Plugin Error: " << errorMessage << std::endl;

    if (result != DW_SUCCESS)
        return;
    result = status;
}

static void setSignalValid(dwSignalValidity& validity)
{
    dwSignal_encodeSignalValidity(&validity,
                                  DW_SIGNAL_STATUS_LAST_VALID,
                                  DW_SIGNAL_TIMEOUT_NONE,
                                  DW_SIGNAL_E2E_NO_ERROR);
}

static void parseSteeringReport(dwVehicleIOState* state)
{
    dwStatus status;
    uint32_t num;
    status = dwCANInterpreter_getNumberSignals(&num, gIntp);

    if (status == DW_SUCCESS && num > 0)
    {
        float32_t value    = 0;
        dwTime_t timestamp = 0;
        const char* name;

        for (uint32_t i = 0; i < num; ++i)
        {
            if (dwCANInterpreter_getSignalName(&name, i, gIntp) == DW_SUCCESS)
            {
                if (dwCANInterpreter_getf32(&value, &timestamp, i, gIntp) == DW_SUCCESS)
                {
                    if (0 == strcmp(name, GENERIC_ID_STEERING_WHEEL_ANGLE_REPORT))
                        state->steeringWheelAngle = value;
                    else if (0 == strcmp(name, GENERIC_ID_STEERING_WHEEL_TORQUE_REPORT))
                        state->steeringWheelTorque = value;
                    else if (0 == strcmp(name, GENERIC_ID_STEERING_WHEEL_SPEED_REPORT))
                        state->speed = value;
                }
            }
        }
    }
}

static void parseSteeringReportExt(dwVehicleIONonSafetyState* nonSafeState, dwVehicleIOActuationFeedback* actuationFeedback)
{
    dwStatus status;
    uint32_t num;
    status = dwCANInterpreter_getNumberSignals(&num, gIntp);

    if (status == DW_SUCCESS && num > 0)
    {
        float32_t value    = 0;
        dwTime_t timestamp = 0;
        const char* name;

        for (uint32_t i = 0; i < num; ++i)
        {
            if (dwCANInterpreter_getSignalName(&name, i, gIntp) == DW_SUCCESS)
            {
                if (dwCANInterpreter_getf32(&value, &timestamp, i, gIntp) == DW_SUCCESS)
                {
                    if (0 == strcmp(name, GENERIC_ID_STEERING_WHEEL_ANGLE_REPORT))
                    {
                        if (actuationFeedback)
                        {
                            actuationFeedback->steeringWheelAngle = value;
                            setSignalValid(actuationFeedback->validityInfo.steeringWheelAngle);
                        }
                    }
                    else if (0 == strcmp(name, GENERIC_ID_STEERING_WHEEL_TORQUE_REPORT))
                    {
                        if (actuationFeedback)
                        {
                            actuationFeedback->steeringWheelTorque = value;
                            setSignalValid(actuationFeedback->validityInfo.steeringWheelTorque);
                        }
                    }
                    else if (0 == strcmp(name, GENERIC_ID_STEERING_WHEEL_SPEED_REPORT))
                    {
                        if (nonSafeState)
                        {
                            nonSafeState->speedESC = value;
                            setSignalValid(nonSafeState->validityInfo.speedESC);
                        }
                    }
                }
            }
        }
    }
}

static dwStatus encodeSteering(dwCANMessage& msgCAN, const dwVehicleIOCommand* cmd)
{
    constexpr float32_t MAX_STEERING_ANGLE = 0.459F;
    constexpr float32_t MAX_STEERING_SPEED = 4.363F;

    float32_t swa = cmd->steeringWheelAngle;
    float32_t sws = cmd->maxSteeringWheelSpeed;

    if (std::fabs(swa) > MAX_STEERING_ANGLE)
    {
        swa = std::copysign(MAX_STEERING_ANGLE, swa);
    }

    if (std::fabs(sws) > MAX_STEERING_SPEED)
    {
        sws = std::copysign(MAX_STEERING_SPEED, sws);
    }

    dwStatus status;
    dwStatus result = DW_SUCCESS;

    status = dwCANInterpreter_encodef32(swa,
                                        GENERIC_ID_STEERING_WHEEL_ANGLE_COMMAND,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering angle failed");

    // assume using steering wheel angle interface
    status = dwCANInterpreter_encodei32(cmd->steeringWheelValid,
                                        GENERIC_ID_STEERING_WHEEL_ANGLE_COMMAND_VALID,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering valid failed");

    status = dwCANInterpreter_encodef32(sws,
                                        GENERIC_ID_STEERING_WHEEL_STEER_SPEED,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering speed failed");

    status = dwCANInterpreter_encodei32(cmd->clearFaults ? true : false,
                                        GENERIC_ID_STEERING_WHEEL_STEER_CLEAR_FAULT,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering clear fault failed");
    return result;
}

static dwStatus encodeSteering_new(dwCANMessage& msgCAN, const dwVehicleIOSafetyCommand* cmd)
{
    constexpr float32_t MAX_STEERING_ANGLE = 0.459F;
    constexpr float32_t MAX_STEERING_SPEED = 4.363F;

    float32_t swa = cmd->latCtrlSteeringWheelAngleRequest;
    float32_t sws = cmd->latCtrlSteeringWheelAngleRateMax;

    if (std::fabs(swa) > MAX_STEERING_ANGLE)
    {
        swa = std::copysign(MAX_STEERING_ANGLE, swa);
    }

    if (std::fabs(sws) > MAX_STEERING_SPEED)
    {
        sws = std::copysign(MAX_STEERING_SPEED, sws);
    }

    dwStatus status;
    dwStatus result = DW_SUCCESS;

    status = dwCANInterpreter_encodef32(swa,
                                        GENERIC_ID_STEERING_WHEEL_ANGLE_COMMAND,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering angle failed");

    // assume using steering wheel angle interface
    status = dwCANInterpreter_encodei32(dwSignal_checkSignalValidity(cmd->latCtrlSteeringWheelAngleRequest) == DW_SUCCESS,
                                        GENERIC_ID_STEERING_WHEEL_ANGLE_COMMAND_VALID,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering valid failed");

    status = dwCANInterpreter_encodef32(sws,
                                        GENERIC_ID_STEERING_WHEEL_STEER_SPEED,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering speed failed");

    bool clearFaults = true;
    if (dwSignal_checkSignalValidity(cmd->validityInfo.latCtrlModeRequest) == DW_SUCCESS &&
        dwSignal_checkSignalValidity(cmd->validityInfo.lonCtrlSafetyLimRequest) == DW_SUCCESS)
    {
        clearFaults = cmd->latCtrlModeRequest == DW_VIO_LAT_CTRL_MODE_REQUEST_IDLE &&
                      cmd->lonCtrlSafetyLimRequest == DW_VIO_LON_CTRL_SAFETY_LIM_REQUEST_NONE;
    }

    status = dwCANInterpreter_encodei32(clearFaults ? true : false,
                                        GENERIC_ID_STEERING_WHEEL_STEER_CLEAR_FAULT,
                                        &msgCAN, gIntp);
    checkError(result, status, "encode steering clear fault failed");
    return result;
}

//################################################################################################
dwStatus _dwVehicleIODriver_initialize()
{
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    dwContextParameters sdkParams = {};
    sdkParams.skipCudaInit        = true;
    CHECK_DW_ERROR(dwInitialize(&gSdk, DW_VERSION, &sdkParams));

    const std::string dbcPath = dw_samples::SamplesDataPath::get() + "/samples/vehicleio/AutonomousVehicleCANSignals.dbc";

    status = dwCANInterpreter_buildFromDBC(&gIntp, dbcPath.c_str(), gSdk);
    checkError(result, status, "cannot create DBC-based CAN message interpreter");

    return result;
}

//################################################################################################
dwStatus _dwVehicleIODriver_release()
{
    dwCANInterpreter_release(gIntp);
    dwRelease(gSdk);
    return DW_SUCCESS;
}

//################################################################################################
dwStatus _dwVehicleIODriver_consume(const dwCANMessage* msg, dwVehicleIOState* state)
{
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    status = dwCANInterpreter_consume(msg, gIntp);
    checkError(result, status, "consume message failed");
    parseSteeringReport(state);

    return result;
}

//################################################################################################
dwStatus _dwVehicleIODriver_consumeForSafeState(const dwCANMessage* msg, dwVehicleIOSafetyState* safeState)
{
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    static_cast<void>(safeState);

    status = dwCANInterpreter_consume(msg, gIntp);
    checkError(result, status, "consumeForSafeState message failed");

    return result;
}

//################################################################################################
dwStatus _dwVehicleIODriver_consumeForNonSafeState(const dwCANMessage* msg, dwVehicleIONonSafetyState* nonSafeState)
{
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    status = dwCANInterpreter_consume(msg, gIntp);
    checkError(result, status, "consumeForNonSafeState message failed");
    parseSteeringReportExt(nonSafeState, nullptr);

    return result;
}

//################################################################################################
dwStatus _dwVehicleIODriver_consumeForActuationFeedback(const dwCANMessage* msg, dwVehicleIOActuationFeedback* actuationFeedback)
{
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    status = dwCANInterpreter_consume(msg, gIntp);
    checkError(result, status, "consumeForActuationFeedback message failed");
    parseSteeringReportExt(nullptr, actuationFeedback);

    return result;
}

dwStatus _dwVehicleIODriver_setDrivingMode(const dwVehicleIODrivingMode mode)
{
    dwStatus ret = DW_SUCCESS;

    switch (mode)
    {
    case DW_VEHICLEIO_DRIVING_LIMITED:
        break;
    case DW_VEHICLEIO_DRIVING_LIMITED_ND:
        break;
    case DW_VEHICLEIO_DRIVING_COLLISION_AVOIDANCE:
        break;
    case DW_VEHICLEIO_DRIVING_NO_SAFETY:
        break;
    case DW_VEHICLEIO_DRIVING_MODE_INVALID:
        ret = DW_NOT_SUPPORTED;
        break;
    default:
        ret = DW_NOT_SUPPORTED;
        break;
    }

    return ret;
}

//################################################################################################
dwStatus _dwVehicleIODriver_sendCommand(const dwVehicleIOCommand* cmd,
                                        dwSensorHandle_t sensor)
{
    dwCANMessage msgCAN{};
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    status = dwCANInterpreter_createMessageFromName(&msgCAN, GENERIC_MSG_STEERING_CMD, gIntp);
    checkError(result, status, "create steering message failed");

    status = encodeSteering(msgCAN, cmd);
    checkError(result, status, "encode steering failed");

    status = dwSensorCAN_sendMessage(&msgCAN, 1000000, sensor);
    checkError(result, status, "send steering failed");

    return result;
}

//################################################################################################
dwStatus _dwVehicleIODriver_sendSafetyCommand(const dwVehicleIOSafetyCommand* cmd,
                                              dwSensorHandle_t sensor)
{
    dwCANMessage msgCAN{};
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    status = dwCANInterpreter_createMessageFromName(&msgCAN, GENERIC_MSG_STEERING_CMD, gIntp);
    checkError(result, status, "create steering message failed");

    status = encodeSteering_new(msgCAN, cmd);
    checkError(result, status, "encode steering failed");

    status = dwSensorCAN_sendMessage(&msgCAN, 1000000, sensor);
    checkError(result, status, "send steering failed");

    return result;
}

//################################################################################################
dwStatus _dwVehicleIODriver_sendNonSafetyCommand(const dwVehicleIONonSafetyCommand*,
                                                 dwSensorHandle_t)
{
    return DW_NOT_IMPLEMENTED;
}

//################################################################################################
dwStatus _dwVehicleIODriver_sendMiscCommand(const dwVehicleIOMiscCommand*,
                                            dwSensorHandle_t)
{
    return DW_NOT_IMPLEMENTED;
}

//################################################################################################
dwStatus _dwVehicleIODriver_clearFaults(dwSensorHandle_t sensor, const dwVehicleIOState* state)
{
    dwCANMessage msgCAN;
    dwStatus status;

    if (state->overrides & DW_VEHICLEIO_OVERRIDE_STEERING)
    {
        status = dwCANInterpreter_createMessageFromName(&msgCAN, GENERIC_MSG_STEERING_CMD, gIntp);
        if (status != DW_SUCCESS)
            return status;

        status = dwCANInterpreter_encodei32(true,
                                            GENERIC_ID_STEERING_WHEEL_STEER_CLEAR_FAULT,
                                            &msgCAN, gIntp);
        if (status != DW_SUCCESS)
            return status;
    }

    status = dwSensorCAN_sendMessage(&msgCAN, 1000000, sensor);
    return status;
}

//################################################################################################
dwStatus _dwVehicleIODriver_clearFaults_new(dwSensorHandle_t sensor, const dwVehicleIOSafetyState* safeState,
                                            const dwVehicleIONonSafetyState* nonSafeState,
                                            const dwVehicleIOActuationFeedback* actuationFeedback)
{
    dwCANMessage msgCAN;
    dwStatus status;

    static_cast<void>(safeState);
    static_cast<void>(nonSafeState);

    if (actuationFeedback->latCtrlDriverInterventionStatus == DW_VIO_LAT_CTRL_DRIVER_INTERVENTION_STATUS_DRVL3INTERRUPT)
    {
        status = dwCANInterpreter_createMessageFromName(&msgCAN, GENERIC_MSG_STEERING_CMD, gIntp);
        if (status != DW_SUCCESS)
            return status;

        status = dwCANInterpreter_encodei32(true,
                                            GENERIC_ID_STEERING_WHEEL_STEER_CLEAR_FAULT,
                                            &msgCAN, gIntp);
        if (status != DW_SUCCESS)
            return status;
    }

    status = dwSensorCAN_sendMessage(&msgCAN, 1000000, sensor);
    return status;
}

//################################################################################################
dwStatus _dwVehicleIODriver_reset()
{
    return DW_SUCCESS;
}

} // extern "C"
