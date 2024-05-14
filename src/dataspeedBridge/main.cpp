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
// SPDX-FileCopyrightText: Copyright (c) 2015-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <framework/ProgramArguments.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/SamplesDataPath.hpp>
#include <framework/MathUtils.hpp>
#include <framework/Checks.hpp>

#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>
#include <dw/control/vehicleio/VehicleIO.h>

#define GENERIC_ID_STEERING_REPORT 0x00
#define GENERIC_ID_BRAKE_REPORT 0x07
#define GENERIC_ID_THROTTLE_REPORT 0x09
#define GENERIC_ID_GEAR_REPORT 0x0D
#define GENERIC_ID_BRAKE_CMD 0x08
#define GENERIC_ID_STEERING_CMD 0x06
#define GENERIC_ID_THROTTLE_CMD 0x0A
#define GENERIC_ID_GEAR_CMD 0x0E
#define GENERIC_ID_MISC_REPORT 0x12
#define GENERIC_ID_TURNSIGNAL_CMD 0x11
#define GENERIC_ID_TURNSIGNAL_REPORT 0x13
#define GENERIC_ID_WHEEL_SPEED_REPORT 0x0C

static dwContextHandle_t gSdk = DW_NULL_HANDLE;
static dwSALHandle_t gHal     = DW_NULL_HANDLE;
static dwVehicleIOSafetyCommand vioSafeCmd{};
static dwVehicleIOSafetyState vioSafeState{};
static dwVehicleIONonSafetyState vioNonSafeState{};
static dwVehicleIOActuationFeedback vioActuationFeedback{};
static dwRigHandle_t gRig            = DW_NULL_HANDLE;
static dwSensorHandle_t gCanSensor   = DW_NULL_HANDLE;
static dwVehicleIOHandle_t vehicleIO = DW_NULL_HANDLE;
static dwCANInterpreterHandle_t gIntpGeneric{};

static void checkError(const dwStatus status, const uint32_t line)
{
    if (status == DW_SUCCESS)
        return;

    throw std::runtime_error("DataSpeed Bridge: error at line #" +
                             std::to_string(line) + ": " +
                             std::string(dwGetStatusName(status)));
}

static void setSignalValid(dwSignalValidity& validity)
{
    dwSignal_encodeSignalValidity(&validity,
                                  DW_SIGNAL_STATUS_LAST_VALID,
                                  DW_SIGNAL_TIMEOUT_NONE,
                                  DW_SIGNAL_E2E_NO_ERROR);
}

static void setSignalInvalid(dwSignalValidity& validity)
{
    dwSignal_encodeSignalValidity(&validity,
                                  DW_SIGNAL_STATUS_INIT,
                                  DW_SIGNAL_TIMEOUT_NEVER_RECEIVED,
                                  DW_SIGNAL_E2E_SEQ_ERROR);
}

template <typename T>
static void setValidSignal(bool present, T& assign, T const& value)
{
    if (present)
    {
        assign = value;
    }
}

template <typename T>
static void assignFromMultipleSignals(bool first, bool second, T& assign, T const& valueFirst, T const& valueSecond)
{
    if (first && second)
    {
        std::cerr << "LegacyConvertor:addSensor: Multiple signals are valid for same legacyStruct, Please Check." << std::endl;
    }

    if (first)
    {
        assign = valueFirst;
    }
    if (second)
    {
        assign = valueSecond;
    }
}

static void encodeGenericSteeringReport(dwCANMessage& report, const dwVehicleIOActuationFeedback& actuationFeedback)
{
    dwStatus status;

    status = dwCANInterpreter_createMessage(&report, GENERIC_ID_STEERING_REPORT, gIntpGeneric);
    checkError(status, __LINE__);

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.steeringWheelAngle) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(RAD2DEG(actuationFeedback.steeringWheelAngle), "Steering_Report.SteeringWheelAngle", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.steeringWheelTorque) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(actuationFeedback.steeringWheelTorque, "Steering_Report.SteeringWheelDriverTorque", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.latCtrlDriverInterventionStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodei32((actuationFeedback.latCtrlDriverInterventionStatus == DW_VIO_LAT_CTRL_DRIVER_INTERVENTION_STATUS_DRVL3INTERRUPT) ? 1 : 0,
                                            "Steering_Report.SteeringWheelDriverOverride", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.latCtrlStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(actuationFeedback.latCtrlStatus == DW_VIO_LAT_CTRL_STATUS_CTRL, "Steering_Report.SteeringWheelAngleEnabled", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.latCtrlErrorStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(actuationFeedback.latCtrlErrorStatus, "Steering_Report.SteeringFaults", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }
}

static void encodeGenericBrakeReport(dwCANMessage& report, const dwVehicleIOSafetyCommand& safeCmd, const dwVehicleIONonSafetyState& nonSafeState, const dwVehicleIOActuationFeedback& actuationFeedback)
{
    dwStatus status;

    status = dwCANInterpreter_createMessage(&report, GENERIC_ID_BRAKE_REPORT, gIntpGeneric);
    checkError(status, __LINE__);

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.driverBrakePedal) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(nonSafeState.driverBrakePedal, "Brake_Report.BrakePedalInput", &report, gIntpGeneric);
        checkError(status, __LINE__);

        status = dwCANInterpreter_encodef32(nonSafeState.driverBrakePedal, "Brake_Report.BrakePedalOutput", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.latCtrlModeStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodei32(actuationFeedback.latCtrlModeStatus > DW_VIO_LAT_CTRL_MODE_STATUS_IDLE, "Brake_Report.BrakeEnabled", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.longCtrlFaultStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodei32(actuationFeedback.longCtrlFaultStatus & DW_VIO_LONG_CTRL_FAULT_STATUS_BRAKE_DEGRADATION, "Brake_Report.BrakeFaults", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(safeCmd.validityInfo.longCtrlBrakePedalRequest) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(safeCmd.longCtrlBrakePedalRequest, "Brake_Report.BrakeCmd", &report, gIntpGeneric); // use the brake value from the last brake command
        checkError(status, __LINE__);
    }
}

static void encodeGenericThrottleReport(dwCANMessage& report, const dwVehicleIOSafetyCommand& safeCmd, const dwVehicleIONonSafetyState& nonSafeState, const dwVehicleIOActuationFeedback& actuationFeedback)
{
    dwStatus status;

    status = dwCANInterpreter_createMessage(&report, GENERIC_ID_THROTTLE_REPORT, gIntpGeneric);
    checkError(status, __LINE__);

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.throttleValue) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(nonSafeState.throttleValue, "Throttle_Report.ThrottlePedalInput", &report, gIntpGeneric);
        checkError(status, __LINE__);

        status = dwCANInterpreter_encodef32(nonSafeState.throttleValue, "Throttle_Report.ThrottlePedalOutput", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    // use the throttle value from the last throttle command
    if (dwSignal_checkSignalValidity(safeCmd.validityInfo.longCtrlThrottlePedalRequest) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(safeCmd.longCtrlThrottlePedalRequest, "Throttle_Report.ThrottleCMD", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.driverOverrideThrottle) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodei32((actuationFeedback.driverOverrideThrottle == DW_VIO_DRIVER_OVERRIDE_THROTTLE_DRV_OVERRIDE) ? 1 : 0, "Throttle_Report.ThrottleOverrideReport", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.latCtrlModeStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodei32(actuationFeedback.latCtrlModeStatus > DW_VIO_LAT_CTRL_MODE_STATUS_IDLE, "Throttle_Report.ThrottleEnabled", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.longCtrlFaultStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodei32(actuationFeedback.longCtrlFaultStatus, "Throttle_Report.ThrottleFaultReport", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }
}

dwVehicleIOGear convertToLegacyGear(const dwVehicleIONonSafetyState& nonSafeState, const dwVehicleIOActuationFeedback& actuationFeedback)
{
    dwVioGearStatus gearStatus = DW_VIO_GEAR_STATUS_FORCE32;
    assignFromMultipleSignals((dwSignal_checkSignalValidity(actuationFeedback.validityInfo.gearStatus) == DW_SUCCESS),
                              (dwSignal_checkSignalValidity(nonSafeState.validityInfo.gearStatus) == DW_SUCCESS),
                              gearStatus,
                              actuationFeedback.gearStatus,
                              nonSafeState.gearStatus);

    dwVehicleIOGear gear = DW_VEHICLEIO_GEAR_UNKNOWN;
    if (gearStatus != DW_VIO_GEAR_STATUS_FORCE32)
    {
        switch (gearStatus)
        {
        case DW_VIO_GEAR_STATUS_R:
        case DW_VIO_GEAR_STATUS_R2:
            gear = DW_VEHICLEIO_GEAR_REVERSE;
            break;
        case DW_VIO_GEAR_STATUS_N:
            gear = DW_VEHICLEIO_GEAR_NEUTRAL;
            break;
        case DW_VIO_GEAR_STATUS_D1:
        case DW_VIO_GEAR_STATUS_D2:
        case DW_VIO_GEAR_STATUS_D3:
        case DW_VIO_GEAR_STATUS_D4:
        case DW_VIO_GEAR_STATUS_D5:
        case DW_VIO_GEAR_STATUS_D6:
        case DW_VIO_GEAR_STATUS_D7:
        case DW_VIO_GEAR_STATUS_D8:
        case DW_VIO_GEAR_STATUS_D9:
        case DW_VIO_GEAR_STATUS_PWRFREE:
            gear = DW_VEHICLEIO_GEAR_DRIVE;
            break;
        case DW_VIO_GEAR_STATUS_P:
            gear = DW_VEHICLEIO_GEAR_PARK;
            break;
        case DW_VIO_GEAR_STATUS_FORCE32:
        default:
            gear = DW_VEHICLEIO_GEAR_UNKNOWN;
            break;
        }
    }

    return gear;
}

static void encodeGenericGearReport(dwCANMessage& report, const dwVehicleIONonSafetyState& nonSafeState, const dwVehicleIOActuationFeedback& actuationFeedback)
{
    dwStatus status;

    status = dwCANInterpreter_createMessage(&report, GENERIC_ID_GEAR_REPORT, gIntpGeneric);
    checkError(status, __LINE__);

    if (dwSignal_checkSignalValidity(actuationFeedback.validityInfo.gearStatus) == DW_SUCCESS ||
        dwSignal_checkSignalValidity(nonSafeState.validityInfo.gearStatus) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodei32(convertToLegacyGear(nonSafeState, actuationFeedback), "Gear_Report.GearState", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }
}

static void encodeGenericTurnSignalReport(dwCANMessage& report, const dwVehicleIONonSafetyState& nonSafeState)
{
    dwStatus status;

    status = dwCANInterpreter_createMessage(&report, GENERIC_ID_TURNSIGNAL_REPORT, gIntpGeneric);
    checkError(status, __LINE__);

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.turnSignalStatus) == DW_SUCCESS)
    {
        dwVehicleIOTurnSignal turnSignal = DW_VEHICLEIO_TURNSIGNAL_UNKNOWN;
        setValidSignal(nonSafeState.turnSignalStatus <= DW_VIO_TURN_SIGNAL_STATUS_EMERGENCY, turnSignal, static_cast<dwVehicleIOTurnSignal>(nonSafeState.turnSignalStatus));

        status = dwCANInterpreter_encodei32(turnSignal, "TurnSignal_Report.TurnSignal", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }
}

static void encodeGenericWheelSpeedReport(dwCANMessage& report, const dwVehicleIONonSafetyState& nonSafeState)
{
    dwStatus status;

    status = dwCANInterpreter_createMessage(&report, GENERIC_ID_WHEEL_SPEED_REPORT, gIntpGeneric);
    checkError(status, __LINE__);

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.wheelSpeed[0]) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(nonSafeState.wheelSpeed[0], "WheelSpeed_Report.LeftFrontWheelSpd", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.wheelSpeed[1]) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(nonSafeState.wheelSpeed[1], "WheelSpeed_Report.RightFrontWheelSpd", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.wheelSpeed[2]) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(nonSafeState.wheelSpeed[2], "WheelSpeed_Report.LeftRearWheelSpd", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.wheelSpeed[3]) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(nonSafeState.wheelSpeed[3], "WheelSpeed_Report.RightRearWheelSpd", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }
}

static void encodeGenericMiscReport(dwCANMessage& report, const dwVehicleIONonSafetyState& nonSafeState)
{
    dwStatus status;

    status = dwCANInterpreter_createMessage(&report, GENERIC_ID_MISC_REPORT, gIntpGeneric);
    checkError(status, __LINE__);

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.headlightState) == DW_SUCCESS)
    {
        bool highBeamHeadlights = nonSafeState.headlightState == DW_VIO_HEADLIGHT_STATE_HIGH_BEAM;
        status                  = dwCANInterpreter_encodei32(highBeamHeadlights, "Misc_Report.HighBeamHeadLight", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }

    if (dwSignal_checkSignalValidity(nonSafeState.validityInfo.speedESC) == DW_SUCCESS)
    {
        status = dwCANInterpreter_encodef32(nonSafeState.speedESC, "Misc_Report.VehicleSpeed", &report, gIntpGeneric);
        checkError(status, __LINE__);
    }
}

static void processGeneric2DataspeedCommand(const dwCANMessage& genericMsg)
{

    if (!(genericMsg.id == GENERIC_ID_STEERING_CMD || genericMsg.id == GENERIC_ID_BRAKE_CMD || genericMsg.id == GENERIC_ID_THROTTLE_CMD ||
          genericMsg.id == GENERIC_ID_GEAR_CMD || genericMsg.id == GENERIC_ID_TURNSIGNAL_CMD))
        return;

    dwCANInterpreter_consume(&genericMsg, gIntpGeneric);
    uint32_t nSig = 0;
    dwCANInterpreter_getNumberSignals(&nSig, gIntpGeneric);

    for (uint32_t i = 0; i < nSig; i++)
    {
        dwTime_t unused;
        const char* sigName;
        dwCANInterpreter_getSignalName(&sigName, i, gIntpGeneric);

        // Steering commands
        if (strcmp(sigName, "Steering_Cmd.SteeringWheelAngleCmd") == 0)
        {
            float32_t value;
            dwCANInterpreter_getf32(&value, &unused, i, gIntpGeneric);
            vioSafeCmd.latCtrlSteeringWheelAngleRequest = value;
            setSignalValid(vioSafeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
        }
        if (strcmp(sigName, "Steering_Cmd.SteeringWheelAngleCmdValid") == 0)
        {
            int32_t iflag;
            dwCANInterpreter_geti32(&iflag, &unused, i, gIntpGeneric);
            if (iflag == 0)
                setSignalInvalid(vioSafeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
            else
                setSignalValid(vioSafeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
        }
        if (strcmp(sigName, "Steering_Cmd.SteeringSteerSpeed") == 0)
        {
            float32_t value;
            dwCANInterpreter_getf32(&value, &unused, i, gIntpGeneric);
            vioSafeCmd.latCtrlSteeringWheelAngleRateMax = value;
            setSignalValid(vioSafeCmd.validityInfo.latCtrlSteeringWheelAngleRateMax);
        }

        // Brake commands
        if (strcmp(sigName, "Brake_Cmd.BrakePedalCmdValid") == 0)
        {
            int32_t iflag;
            dwCANInterpreter_geti32(&iflag, &unused, i, gIntpGeneric);
            if (iflag == 0)
                setSignalInvalid(vioSafeCmd.validityInfo.longCtrlBrakePedalRequest);
            else
                setSignalValid(vioSafeCmd.validityInfo.longCtrlBrakePedalRequest);
        }
        if (strcmp(sigName, "Brake_Cmd.BrakePedalCmd") == 0)
        {
            float32_t value;
            dwCANInterpreter_getf32(&value, &unused, i, gIntpGeneric);
            vioSafeCmd.longCtrlBrakePedalRequest = value;
        }

        // Throttle commands
        if (strcmp(sigName, "Throttle_Cmd.ThrottlePedalCmdValid") == 0)
        {
            int32_t iflag;
            dwCANInterpreter_geti32(&iflag, &unused, i, gIntpGeneric);
            if (iflag == 0)
                setSignalInvalid(vioSafeCmd.validityInfo.longCtrlThrottlePedalRequest);
            else
                setSignalValid(vioSafeCmd.validityInfo.longCtrlThrottlePedalRequest);
        }
        if (strcmp(sigName, "Throttle_Cmd.ThrottlePedalCmd") == 0)
        {
            float32_t value;
            dwCANInterpreter_getf32(&value, &unused, i, gIntpGeneric);
            vioSafeCmd.longCtrlThrottlePedalRequest = value;
        }
    }

    /*
     * Do not send DataSpeed Commands on every CAN message (which will
     * flood the CAN bus, since one command translates to multiple CAN
     * messages), instead, send the commands on every Generic Steering
     * command.
     */

    if (genericMsg.id != GENERIC_ID_STEERING_CMD)
        return;

    dwStatus status;
    status = dwVehicleIO_sendSafetyCommand(&vioSafeCmd, vehicleIO);
    if (status != DW_SUCCESS)
    {
        std::cerr << "DataSpeed Bridge: error from dwVehicleIO_sendSafetyCommand: "
                  << dwGetStatusName(status) << std::endl;
        return;
    }
}

static void processDataspeed2GenericReport(const dwVehicleIOSafetyCommand& safeCmd,
                                           const dwVehicleIONonSafetyState& nonSafeState,
                                           const dwVehicleIOActuationFeedback& actuationFeedback)
{
    dwCANMessage genericReport{};

    encodeGenericSteeringReport(genericReport, actuationFeedback);
    dwSensorCAN_sendMessage(&genericReport, 1000000, gCanSensor);

    encodeGenericBrakeReport(genericReport, safeCmd, nonSafeState, actuationFeedback);
    dwSensorCAN_sendMessage(&genericReport, 1000000, gCanSensor);

    encodeGenericThrottleReport(genericReport, safeCmd, nonSafeState, actuationFeedback);
    dwSensorCAN_sendMessage(&genericReport, 1000000, gCanSensor);

    encodeGenericMiscReport(genericReport, nonSafeState);
    dwSensorCAN_sendMessage(&genericReport, 1000000, gCanSensor);

    encodeGenericGearReport(genericReport, nonSafeState, actuationFeedback);
    dwSensorCAN_sendMessage(&genericReport, 1000000, gCanSensor);

    encodeGenericTurnSignalReport(genericReport, nonSafeState);
    dwSensorCAN_sendMessage(&genericReport, 1000000, gCanSensor);

    encodeGenericWheelSpeedReport(genericReport, nonSafeState);
    dwSensorCAN_sendMessage(&genericReport, 1000000, gCanSensor);
}

static void initSdk(dwContextHandle_t* context)
{
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);
    dwContextParameters sdkParams = {};
    CHECK_DW_ERROR(dwInitialize(context, DW_VERSION, &sdkParams));
}

int main(int argc, const char** argv)
{
    struct sigaction action = {};
    action.sa_handler       = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command

    ProgramArguments arguments(
        {
            ProgramArguments::Option_t("driver", "can.socket"),
            ProgramArguments::Option_t("params", "device=vcan0"),
            ProgramArguments::Option_t("rig", (dw_samples::SamplesDataPath::get() + "/samples/vehicleio/rig-dataspeedBridge.json").c_str()),
            ProgramArguments::Option_t("dbc", (dw_samples::SamplesDataPath::get() + "/samples/sensors/can/AutonomousVehicleCANSignals.dbc").c_str()),
        });

    if (!arguments.parse(argc, argv))
    {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "--driver=can.socket ";
        std::cout << "--params=device=vcan0 ";
        std::cout << "--dbc=AutonomousVehicleCANSignals.dbc \t" << std::endl;
        return -1;
    }

    std::cout << "Program Arguments:\n"
              << arguments.printList() << std::endl;

    initSdk(&gSdk);

    dwStatus status;

    status = dwRig_initializeFromFile(&gRig, gSdk, arguments.get("rig").c_str());
    if (status != DW_SUCCESS)
    {
        std::cerr << "DataSpeed Bridge: cannot load vehicle information from given rig file: "
                  << arguments.get("rig")
                  << " (" << dwGetStatusName(status) << ")"
                  << std::endl;
        dwRelease(gSdk);
        dwLogger_release();
        return -1;
    }

    status = dwCANInterpreter_buildFromDBC(&gIntpGeneric, arguments.get("dbc").c_str(), gSdk);
    if (status != DW_SUCCESS)
    {
        std::cerr << "DataSpeed Bridge: cannot create DBC-based CAN message interpreter from DBC file "
                  << arguments.get("dbc")
                  << " (" << dwGetStatusName(status) << ")"
                  << std::endl;
        return -1;
    }

    dwSAL_initialize(&gHal, gSdk);

    dwSensorParams params{};
    std::string parameterString = arguments.get("params");
    params.parameters           = parameterString.c_str();
    params.protocol             = arguments.get("driver").c_str();

    status = dwSAL_createSensor(&gCanSensor, params, gHal);
    if (status != DW_SUCCESS)
    {
        std::cerr << "DataSpeed Bridge: cannot create sensor "
                  << params.protocol << " with "
                  << params.parameters
                  << " (" << dwGetStatusName(status) << ")"
                  << std::endl;
        dwSAL_release(gHal);
        dwRelease(gSdk);
        dwLogger_release();
        return -1;
    }

    const dwVehicle* vehicle = nullptr;
    dwRig_getVehicle(&vehicle, gRig);

    status = dwVehicleIO_initialize(&vehicleIO, DW_VEHICLEIO_DATASPEED, vehicle, gSdk);
    if (status != DW_SUCCESS)
    {
        std::cerr << "DataSpeed Bridge: cannot create VehicleIO controller"
                  << " (" << dwGetStatusName(status) << ")"
                  << std::endl;
        dwSAL_releaseSensor(gCanSensor);
        dwSAL_release(gHal);
        dwRelease(gSdk);
        dwLogger_release();
        return -1;
    }

    dwVehicleIO_addCANSensor(0, gCanSensor, vehicleIO);

    status = dwVehicleIO_setDrivingMode(DW_VEHICLEIO_DRIVING_LIMITED_ND, vehicleIO);
    if (status != DW_SUCCESS)
    {
        std::cerr << "DataSpeed Bridge: cannot change driving mode"
                  << " (" << dwGetStatusName(status) << ")"
                  << std::endl;
        dwVehicleIO_release(vehicleIO);
        dwSAL_releaseSensor(gCanSensor);
        dwSAL_release(gHal);
        dwRelease(gSdk);
        dwLogger_release();
        return -1;
    }

    gRun = dwSensor_start(gCanSensor) == DW_SUCCESS;

    while (gRun)
    {
        dwCANMessage msg{};

        status = dwSensorCAN_readMessage(&msg, 10000000, gCanSensor);
        if (status == DW_END_OF_STREAM)
        {
            std::cerr << "DataSpeed Bridge: reached CAN EOF" << std::endl;
            break;
        }
        else if (status == DW_TIME_OUT)
        {
            std::cerr << "DataSpeed Bridge: timeout, please check that "
                         "correct DataSpeed CAN bus has been specified and "
                         "the bus is connected."
                      << std::endl;
            continue;
        }
        else if (status != DW_SUCCESS)
        {
            std::cerr << "DataSpeed Bridge: CAN error "
                      << " (" << dwGetStatusName(status) << ")"
                      << std::endl;
            continue;
        }

        status = dwVehicleIO_consumeCANFrame(&msg, 0, vehicleIO);
        if (status != DW_SUCCESS)
        {
            std::cerr << "DataSpeed Bridge: can't consume CAN"
                      << " (" << dwGetStatusName(status) << ")"
                      << std::endl;
            break;
        }

        /*
         * Do not send Generic Reports on every CAN message (which will
         * flood the CAN bus), instead, send the generic reports on every
         * DataSpeed's speed/steering report, plus send messages if any of
         * the engagement button status changed.
         */
        dwTime_t oldTimestamp = vioNonSafeState.frontSteeringTimestamp;
        bool shouldSend       = false;
        dwVehicleIO_getVehicleSafetyState(&vioSafeState, vehicleIO);
        dwVehicleIO_getVehicleNonSafetyState(&vioNonSafeState, vehicleIO);
        dwVehicleIO_getVehicleActuationFeedback(&vioActuationFeedback, vehicleIO);

        shouldSend |= ((dwSignal_checkSignalValidity(vioNonSafeState.validityInfo.frontSteeringTimestamp) == DW_SUCCESS) &&
                       (vioNonSafeState.frontSteeringTimestamp != oldTimestamp));

        if (shouldSend)
        {
            processDataspeed2GenericReport(vioSafeCmd, vioNonSafeState, vioActuationFeedback);
        }

        processGeneric2DataspeedCommand(msg);
    }

    if (gIntpGeneric != DW_NULL_HANDLE)
        dwCANInterpreter_release(gIntpGeneric);

    dwSensor_stop(gCanSensor);
    dwSAL_releaseSensor(gCanSensor);
    dwSAL_release(gHal);
    dwRelease(gSdk);
    dwLogger_release();
}
