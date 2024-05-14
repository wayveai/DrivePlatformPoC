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
// SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef __DRIVERCONF_HPP__
#define __DRIVERCONF_HPP__

enum
{
    GENERIC_GEAR_UNKNOWN = 0,
    GENERIC_GEAR_PARK    = 1,
    GENERIC_GEAR_REVERSE = 2,
    GENERIC_GEAR_NEUTRAL = 3,
    GENERIC_GEAR_DRIVE   = 4,
    GENERIC_GEAR_LOW     = 5
};

enum
{
    GENERIC_TURNSIGNAL_UNKNOWN   = 0,
    GENERIC_TURNSIGNAL_OFF       = 1,
    GENERIC_TURNSIGNAL_LEFT      = 2,
    GENERIC_TURNSIGNAL_RIGHT     = 3,
    GENERIC_TURNSIGNAL_EMERGENCY = 4
};

static constexpr auto GENERIC_MSG_BRAKE_CMD            = "Brake_Cmd";
static constexpr auto GENERIC_MSG_BRAKE_REPORT         = "Brake_Report";
static constexpr auto GENERIC_MSG_GEAR_CMD             = "Gear_Cmd";
static constexpr auto GENERIC_MSG_GEAR_REPORT          = "Gear_Report";
static constexpr auto GENERIC_MSG_STEERING_CMD         = "Steering_Cmd";
static constexpr auto GENERIC_MSG_STEERING_REPORT      = "Steering_Report";
static constexpr auto GENERIC_MSG_THROTTLE_CMD         = "Throttle_Cmd";
static constexpr auto GENERIC_MSG_THROTTLE_REPORT      = "Throttle_Report";
static constexpr auto GENERIC_MSG_TIRE_PRESSURE_REPORT = "TirePressure_Report";
static constexpr auto GENERIC_MSG_TURNSIGNAL_CMD       = "TurnSignal_Cmd";
static constexpr auto GENERIC_MSG_TURNSIGNAL_REPORT    = "TurnSignal_Report";
static constexpr auto GENERIC_MSG_MISC_REPORT          = "Misc_Report";
static constexpr auto GENERIC_MSG_WHEEL_SPEED_REPORT   = "WheelSpeed_Report";

static constexpr auto GENERIC_ID_STEERING_WHEEL_ANGLE_REPORT           = "Steering_Report.SteeringWheelAngle";
static constexpr auto GENERIC_ID_STEERING_WHEEL_TORQUE_REPORT          = "Steering_Report.SteeringWheelDriverTorque";
static constexpr auto GENERIC_ID_STEERING_WHEEL_DRIVER_OVERRIDE_REPORT = "Steering_Report.SteeringWheelDriverOverride";
static constexpr auto GENERIC_ID_STEERING_WHEEL_SPEED_REPORT           = "Steering_Report.SteeringWheelSpeed";
static constexpr auto GENERIC_ID_STEERING_WHEEL_ENABLED_REPORT         = "Steering_Report.SteeringWheelAngleEnabled";
static constexpr auto GENERIC_ID_STEERING_WHEEL_FAULT_REPORT           = "Steering_Report.SteeringFaults";
static constexpr auto GENERIC_ID_STEERING_WHEEL_ANGLE_COMMAND          = "Steering_Cmd.SteeringWheelAngleCmd";
static constexpr auto GENERIC_ID_STEERING_WHEEL_ANGLE_COMMAND_VALID    = "Steering_Cmd.SteeringWheelAngleCmdValid";
static constexpr auto GENERIC_ID_STEERING_WHEEL_STEER_SPEED            = "Steering_Cmd.SteeringSteerSpeed";
static constexpr auto GENERIC_ID_STEERING_WHEEL_STEER_CLEAR_FAULT      = "Steering_Cmd.SteeringSteerClearFault";

static constexpr auto GENERIC_ID_BRAKE_CMD_REPORT          = "Brake_Report.BrakeCmd";
static constexpr auto GENERIC_ID_BRAKE_VALUE_REPORT        = "Brake_Report.BrakePedalInput";
static constexpr auto GENERIC_ID_BRAKE_OVERRIDE_REPORT     = "Brake_Report.BrakeOverrideReport";
static constexpr auto GENERIC_ID_BRAKE_ENABLED             = "Brake_Report.BrakeEnabled";
static constexpr auto GENERIC_ID_BRAKE_FAULT_REPORT        = "Brake_Report.BrakeFaults";
static constexpr auto GENERIC_ID_BRAKE_VALUE_COMMAND       = "Brake_Cmd.BrakePedalCmd";
static constexpr auto GENERIC_ID_BRAKE_PERCENT_COMMAND     = "Brake_Cmd.BrakePedalPercentCmd";
static constexpr auto GENERIC_ID_BRAKE_TORQUE_COMMAND      = "Brake_Cmd.BrakePedalTorqueCmd";
static constexpr auto GENERIC_ID_BRAKE_COMMAND_VALID       = "Brake_Cmd.BrakePedalCmdValid";
static constexpr auto GENERIC_ID_BRAKE_CLEAR_FAULT_COMMAND = "Brake_Cmd.BrakeClearFaultCmd";

static constexpr auto GENERIC_ID_THROTTLE_VALUE_REPORT        = "Throttle_Report.ThrottlePedalInput";
static constexpr auto GENERIC_ID_THROTTLE_CMD_REPORT          = "Throttle_Report.ThrottleCMD";
static constexpr auto GENERIC_ID_THROTTLE_FAULT_REPORT        = "Throttle_Report.ThrottleFaultReport";
static constexpr auto GENERIC_ID_THROTTLE_ENABLED_REPORT      = "Throttle_Report.ThrottleEnabled";
static constexpr auto GENERIC_ID_THROTTLE_OVERRIDE_REPORT     = "Throttle_Report.ThrottleOverrideReport";
static constexpr auto GENERIC_ID_THROTTLE_VALUE_COMMAND       = "Throttle_Cmd.ThrottlePedalCmd";
static constexpr auto GENERIC_ID_THROTTLE_PERCENT_COMMAND     = "Throttle_Cmd.ThrottlePercentCmd";
static constexpr auto GENERIC_ID_THROTTLE_COMMAND_VALID       = "Throttle_Cmd.ThrottlePedalCmdValid";
static constexpr auto GENERIC_ID_THROTTLE_CLEAR_FAULT_COMMAND = "Throttle_Cmd.ThrottleClearFaultCmd";

static constexpr auto GENERIC_ID_WHEEL_SPEED_REPORT_FRONT_LEFT  = "WheelSpeed_Report.LeftFrontWheelSpd";
static constexpr auto GENERIC_ID_WHEEL_SPEED_REPORT_FRONT_RIGHT = "WheelSpeed_Report.RightFrontWheelSpd";
static constexpr auto GENERIC_ID_WHEEL_SPEED_REPORT_REAR_LEFT   = "WheelSpeed_Report.LeftRearWheelSpd";
static constexpr auto GENERIC_ID_WHEEL_SPEED_REPORT_REAR_RIGHT  = "WheelSpeed_Report.RightRearWheelSpd";

static constexpr auto GENERIC_ID_TIRE_PRESSURE_REPORT_FRONT_LEFT  = "TirePressure_Report.TirePressureLeftFront";
static constexpr auto GENERIC_ID_TIRE_PRESSURE_REPORT_FRONT_RIGHT = "TirePressure_Report.TirePressureRightFront";
static constexpr auto GENERIC_ID_TIRE_PRESSURE_REPORT_REAR_LEFT   = "TirePressure_Report.TirePressureLeftRear";
static constexpr auto GENERIC_ID_TIRE_PRESSURE_REPORT_REAR_RIGHT  = "TirePressure_Report.TirePressureRightRear";

static constexpr auto GENERIC_ID_GEAR_COMMAND_CLEAR_FAULT = "Gear_Cmd.GearCmdClearFault";
static constexpr auto GENERIC_ID_GEAR_COMMAND_VALID       = "Gear_Cmd.GearCmdValid";
static constexpr auto GENERIC_ID_GEAR_VALUE_REPORT        = "Gear_Report.GearState";
static constexpr auto GENERIC_ID_GEAR_VALUE_COMMAND       = "Gear_Cmd.GearCmd";
static constexpr auto GENERIC_ID_GEAR_CLEAR_FAULT_COMMAND = "Gear_Cmd.GearClearFaultCmd";

static constexpr auto GENERIC_ID_TURNSIGNAL_VALUE_REPORT  = "TurnSignal_Report.TurnSignal";
static constexpr auto GENERIC_ID_TURNSIGNAL_VALUE_COMMAND = "TurnSignal_Cmd.TurnSignalCmd";

static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_ONOFF        = "Misc_Report.CruiseControlOnOff";
static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_RESET        = "Misc_Report.CruiseControlReset";
static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_CANCEL       = "Misc_Report.CruiseControlCancel";
static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_RESETCANCEL  = "Misc_Report.CruiseControlResetCancel";
static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_DECREMENT    = "Misc_Report.CruiseControlDecrement";
static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_INCREMENT    = "Misc_Report.CruiseControlIncrement";
static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_GAPDECREMENT = "Misc_Report.CruiseControlGapDecrement";
static constexpr auto GENERIC_ID_MISC_REPORT_CRUISE_CONTROL_GAPINCREMENT = "Misc_Report.CruiseControlGapIncrement";
static constexpr auto GENERIC_ID_MISC_REPORT_DOOR_DRIVER                 = "Misc_Report.DoorDriver";
static constexpr auto GENERIC_ID_MISC_REPORT_DOOR_HOOD                   = "Misc_Report.DoorHood";
static constexpr auto GENERIC_ID_MISC_REPORT_DOOR_PASSENGER              = "Misc_Report.DoorPassenger";
static constexpr auto GENERIC_ID_MISC_REPORT_DOOR_TRUNK                  = "Misc_Report.DoorTrunk";
static constexpr auto GENERIC_ID_MISC_REPORT_HIGH_BEAM                   = "Misc_Report.HighBeamHeadLight";
static constexpr auto GENERIC_ID_MISC_REPORT_LANE_ASSIST_ONOFF           = "Misc_Report.LaneAssistOnOff";
static constexpr auto GENERIC_ID_MISC_REPORT_FUEL_LEVEL                  = "Misc_Report.FuelLevel";
static constexpr auto GENERIC_ID_MISC_REPORT_VEHICLE_SPEED               = "Misc_Report.VehicleSpeed";
static constexpr auto GENERIC_ID_MISC_REPORT_WIPER                       = "Misc_Report.Wiper";

#endif /* __DRIVERCONF_HPP__ */
