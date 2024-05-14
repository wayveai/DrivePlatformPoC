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
// SPDX-FileCopyrightText: Copyright (c) 2017-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <libgen.h>
#include <unistd.h>
#include <dw/core/base/Version.h>
#include <dwshared/dwfoundation/dw/core/logger/Logger.hpp>
#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif
#include <framework/DriveWorksSample.hpp>

// Include all relevant DriveWorks modules
#include <dw/control/vehicleio/VehicleIO.h>
#include <dw/sensors/sensormanager/SensorManager.h>

using namespace dw_samples::common;

static constexpr char8_t LOG_TAG[] = "sample_vehicleio";

class VehicleIOSample : public DriveWorksSample
{
    static void setSignalValidIf(bool valid, dwSignalValidity& validity)
    {
        if (valid)
        {
            dwSignal_encodeSignalValidity(&validity,
                                          DW_SIGNAL_STATUS_LAST_VALID,
                                          DW_SIGNAL_TIMEOUT_NONE,
                                          DW_SIGNAL_E2E_NO_ERROR);
        }
        else
        {
            dwSignal_encodeSignalValidity(&validity,
                                          DW_SIGNAL_STATUS_INIT,
                                          DW_SIGNAL_TIMEOUT_NEVER_RECEIVED,
                                          DW_SIGNAL_E2E_NO_INFORMATION);
        }
    }

private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t m_context                           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_visualizationContext = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine                 = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                                   = DW_NULL_HANDLE;

    // New VehicleIO structures
    dwVehicleIOSafetyCommand m_safeCmd{};
    dwVehicleIONonSafetyState m_nonSafeState{};
    dwVehicleIOActuationFeedback m_actuationFeedback{};

    dwRigHandle_t m_rig             = DW_NULL_HANDLE;
    dwSensorManagerHandle_t m_sm    = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO = DW_NULL_HANDLE;
    const dwSensorEvent* m_ev{};
    dwTime_t m_timeToSend{};
    bool m_cmdEnable = false;
    dwTime_t m_intervalUs;

    typedef enum {
        JS_A_DRIVE                   = 0,
        JS_B_REVERSE                 = 1,
        JS_X_NEUTRAL                 = 2,
        JS_Y_PARK                    = 3,
        JS_Disable                   = 4,
        JS_Enable                    = 5,
        JS_Steering_multiplier_left  = 6,
        JS_Steering_multiplier_right = 7,
        JS_Max_Button                = 7,
    } JS_buttons;

    typedef enum {
        JS_brakeValue    = 2,
        JS_steeringValue = 3,
        JS_throttleValue = 5,
        JS_Max_Axis      = 5,
    } JS_axes;

    bool has_safety_faults(dwVehicleIOActuationFeedback const& vioActuationFeedback)
    {
        return vioActuationFeedback.longCtrlStatus == DW_VIO_LONG_CTRL_STATUS_UNKNOWN ||
               vioActuationFeedback.longCtrlStatus == DW_VIO_LONG_CTRL_STATUS_ERROR ||
               vioActuationFeedback.latCtrlStatus == DW_VIO_LAT_CTRL_STATUS_ERROR;
    }

    bool get_overrides_state(dwVehicleIOActuationFeedback const& vioActuationFeedback)
    {
        return vioActuationFeedback.driverOverrideThrottle == DW_VIO_DRIVER_OVERRIDE_THROTTLE_DRV_OVERRIDE ||
               vioActuationFeedback.latCtrlDriverInterventionStatus == DW_VIO_LAT_CTRL_DRIVER_INTERVENTION_STATUS_DRVL3INTERRUPT;
    }

public:
    VehicleIOSample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    void initializeDriveWorks(dwContextHandle_t& context) const
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initializeExtended(getConsoleLoggerExtendedCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};

#ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
#endif

        CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
    }

    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        initializeDriveWorks(m_context);
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

        CHECK_DW_ERROR(dwVisualizationInitialize(&m_visualizationContext, m_context));

        // -----------------------------
        // Initialize RenderEngine
        // -----------------------------
        dwRenderEngineParams renderEngineParams{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&renderEngineParams,
                                                        getWindowWidth(),
                                                        getWindowHeight()));
        renderEngineParams.defaultTile.backgroundColor = {0.0f, 0.0f, 0.0f, 1.0f};
        CHECK_DW_ERROR_MSG(dwRenderEngine_initialize(&m_renderEngine, &renderEngineParams, m_visualizationContext),
                           "VehicleIO sample: Cannot initialize Render Engine, maybe no GL context available?");

        constexpr dwTime_t DEFAULT_INTERVAL = 10'000;
        m_intervalUs                        = atoi(getArgument("interval").c_str()) * 1000;
        if (m_intervalUs <= 0)
        {
            m_intervalUs = DEFAULT_INTERVAL;
        }
        DW_LOGW << "Interval : " << m_intervalUs << " us\n";
        DW_LOGW << "Rig : " << getArgument("rig").c_str() << " us\n";
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rig, m_context, getArgument("rig").c_str()));
        CHECK_DW_ERROR(dwSensorManager_initializeFromRig(&m_sm, m_rig, 16, m_sal));

        dwStatus status = DW_FAILURE;

        status = dwVehicleIO_initializeFromRig(&m_vehicleIO, m_rig, m_context);
        if (status != DW_SUCCESS)
        {
            logError("VehicleIO sample: Cannot create VehicleIO controller\n");
            return false;
        }

        uint32_t vehicleIOCount;
        uint32_t sensorId;
        dwSensorHandle_t sensorHandle;
        dwRig_getVehicleIOConfigCount(&vehicleIOCount, m_rig);

        for (uint32_t vehicleIOId = 0; vehicleIOId < vehicleIOCount; vehicleIOId++)
        {
            if (dwRig_findSensorIdFromVehicleIOId(&sensorId, vehicleIOId, m_rig) != DW_SUCCESS)
            {
                std::cout << "Cannot find sensor ID from vehicleIO ID: " << vehicleIOId << std::endl;
                break;
            }
            if (dwSensorManager_getSensorHandle(&sensorHandle, sensorId, m_sm) != DW_SUCCESS)
            {
                std::cout << "Cannot find sensor handle from sensor ID: " << sensorId << std::endl;
                break;
            }
            dwSensorType type{};
            dwRig_getSensorType(&type, sensorId, m_rig);
            if (type == DW_SENSOR_CAN)
                dwVehicleIO_addCANSensor(vehicleIOId, sensorHandle, m_vehicleIO);
            else
                dwVehicleIO_addDataSensor(vehicleIOId, sensorHandle, m_vehicleIO);
        }

        if (dwVehicleIO_setDrivingMode(DW_VEHICLEIO_DRIVING_LIMITED_ND, m_vehicleIO) != DW_SUCCESS)
        {
            if (getArgument("allow-no-safety") == "yes")
            {
                if (dwVehicleIO_setDrivingMode(DW_VEHICLEIO_DRIVING_NO_SAFETY, m_vehicleIO) != DW_SUCCESS)
                {
                    logError("VehicleIO sample: Cannot set no-safety driving mode\n");
                    return false;
                }
            }
            else
            {
                logError("VehicleIO sample: Cannot set limited no-disengage driving mode, actuation will be disabled.\n");
            }
        }

        resetCmd();
        resetMiscCmd();
        CHECK_DW_ERROR(dwSensorManager_start(m_sm));

        return true;
    }

    void resetCmd()
    {
        m_cmdEnable                                = false;
        m_safeCmd.latCtrlSteeringWheelAngleRequest = 0.0f;
        m_safeCmd.latCtrlSteeringWheelAngleRateMax = M_PI;
        m_safeCmd.latCtrlFrontWheelAngleRequest    = 0.0f;
        m_safeCmd.latCtrlCurvRequest               = 0.0f;
        m_safeCmd.longCtrlThrottlePedalRequest     = 0.0;
        m_safeCmd.longCtrlBrakePedalRequest        = 0.0f;
        m_safeCmd.longCtrlDrivePositionCommand     = DW_VIO_LONG_CTRL_DRIVE_POSITION_COMMAND_IDLE;

        // Validity
        setSignalValidIf(false, m_safeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
        setSignalValidIf(false, m_safeCmd.validityInfo.latCtrlSteeringWheelAngleRateMax);
        setSignalValidIf(false, m_safeCmd.validityInfo.latCtrlFrontWheelAngleRequest);
        setSignalValidIf(false, m_safeCmd.validityInfo.latCtrlCurvRequest);
        setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlThrottlePedalRequest);
        setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlBrakePedalRequest);
        setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlDrivePositionCommand);
    }

    void resetMiscCmd()
    {
    }

    void onReset() override
    {
        resetCmd();
        resetMiscCmd();
        dwSensorManager_stop(m_sm);
        dwSensorManager_reset(m_sm);
        dwSensorManager_start(m_sm);
        dwRenderEngine_reset(m_renderEngine);
    }

    void onRelease() override
    {

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        if (m_visualizationContext != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwVisualizationRelease(m_visualizationContext));
        }

        if (m_ev)
        {
            CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(m_ev, m_sm));
            m_ev = nullptr;
        }

        if (m_vehicleIO != DW_NULL_HANDLE)
        {
            dwVehicleIO_release(m_vehicleIO);
        }

        if (m_rig != DW_NULL_HANDLE)
        {
            dwRig_release(m_rig);
        }

        dwSensorManager_release(m_sm);
        dwSAL_release(m_sal);

        if (m_context != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRelease(m_context));
        }

        CHECK_DW_ERROR(dwLogger_release());
    }

    void onResizeWindow(int width, int height) override
    {
        dwRectf bounds{.x = 0.0f, .y = 0.0f};
        bounds.width  = width;
        bounds.height = height;
        dwRenderEngine_setBounds(bounds, m_renderEngine);
    }

    void handleJoystick()
    {
        int bcount;
        int acount;
        const unsigned char* buttons;
        const float* axes;

        buttons = glfwGetJoystickButtons(0, &bcount);
        axes    = glfwGetJoystickAxes(0, &acount);

        if (bcount < JS_Max_Button || acount < JS_Max_Axis)
            return;

        if (buttons[JS_Enable])
        {
            m_cmdEnable = true;
            // Validity
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlBrakePedalRequest);
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlThrottlePedalRequest);
            setSignalValidIf(true, m_safeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlDrivePositionCommand);
        }

        if (buttons[JS_Disable])
            resetCmd();

        float32_t& brakeValue = m_safeCmd.longCtrlBrakePedalRequest;
        if (axes[JS_brakeValue] + 1 >= 0.01)
            brakeValue = 0.16 + 0.17 * (axes[JS_brakeValue] + 1) / 2;
        else
            brakeValue = 0;

        m_safeCmd.longCtrlThrottlePedalRequest     = 0.7 * (axes[JS_throttleValue] + 1) / 2;
        m_safeCmd.latCtrlSteeringWheelAngleRequest = -(axes[JS_steeringValue] * 470) * static_cast<float32_t>(M_PI) / 180.0f;
    }

    void onProcess() override
    {
        if (m_ev)
        {
            CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(m_ev, m_sm));
            m_ev = nullptr;
        }

        handleJoystick();

        dwTime_t now{};
        dwContext_getCurrentTime(&now, m_context);
        if (now >= m_timeToSend)
        {
            if (m_cmdEnable)
            {
                dwVehicleIO_sendSafetyCommand(&m_safeCmd, m_vehicleIO);
            }

            m_timeToSend = now + m_intervalUs;

            DW_LOGW << "VehicleIO sample: Status\n";
            log("steeringWheelAngle: %f\n", static_cast<float64_t>(m_actuationFeedback.steeringWheelAngle));
            log("throttleValue: %f\n", static_cast<float64_t>(m_nonSafeState.throttleValue));
            log("brake value: %f\n", static_cast<float64_t>(m_nonSafeState.driverBrakePedal));
            log("faults: %u\n", has_safety_faults(m_actuationFeedback));
            log("overrides: %u\n\n", get_overrides_state(m_actuationFeedback));
        }

        dwStatus status;

        status = dwSensorManager_acquireNextEvent(&m_ev, m_intervalUs, m_sm);
        if (status == DW_END_OF_STREAM)
        {
            DW_LOGW << "VehicleIO sample: reached EOF\n";
            reset();
            return;
        }
        else if (status != DW_SUCCESS)
        {
            if (status != DW_TIME_OUT)
            {
                DW_LOGW << "VehicleIO sample: sensor error " << dwGetStatusName(status) << "\n";
            }
            return;
        }

        if (m_ev->type == DW_SENSOR_CAN)
        {
            status = dwVehicleIO_consumeCANFrame(&m_ev->canFrame, m_ev->sensorIndex, m_vehicleIO);
            if (status != DW_SUCCESS)
            {
                logError("VehicleIO sample: can't consume CAN\n");
                return;
            }
        }
        else if (m_ev->type == DW_SENSOR_DATA)
        {
            status = dwVehicleIO_consumeDataPacket(m_ev->dataFrame, m_ev->sensorIndex, m_vehicleIO);
            if (status != DW_SUCCESS)
            {
                logError("VehicleIO sample: can't consume data packet\n");
                return;
            }
        }

        dwVehicleIO_getVehicleNonSafetyState(&m_nonSafeState, m_vehicleIO);
        dwVehicleIO_getVehicleActuationFeedback(&m_actuationFeedback, m_vehicleIO);
    }

    void onRender() override
    {
        // render text in the middle of the window
        dwRenderEngine_reset(m_renderEngine);
        dwRenderEngine_setTile(0, m_renderEngine);
        dwRectf viewport{};
        dwRenderEngine_getViewport(&viewport, m_renderEngine);
        dwRenderEngine_setCoordinateRange2D({viewport.width, viewport.height}, m_renderEngine);
        dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_GREEN, m_renderEngine);

        dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_12, m_renderEngine);
        dwRenderEngine_renderText2D("(ESC to quit)", {10.0f, 20.0f}, m_renderEngine);
        dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_20, m_renderEngine);

        std::stringstream logBrakeThrottle;
        logBrakeThrottle << "Brake/Throttle Data"
                         << "\n";
        logBrakeThrottle << "\n";
        logBrakeThrottle << " brake: " << m_nonSafeState.driverBrakePedal << "\n";
        logBrakeThrottle << " brakeTorqueActual: " << m_actuationFeedback.brakeTorque << "\n";
        logBrakeThrottle << " throttleValue: " << m_nonSafeState.throttleValue << "\n";

        dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_DARKRED, m_renderEngine);
        dwRenderEngine_renderText2D(logBrakeThrottle.str().c_str(), {400.0f, 320.0f}, m_renderEngine);

        std::stringstream logSteering;
        logSteering << "Steering Data"
                    << "\n";
        logSteering << "\n";
        logSteering << " steeringWheelAngle: " << m_actuationFeedback.steeringWheelAngle << "\n";
        logSteering << " steeringWheelTorque: " << m_actuationFeedback.steeringWheelTorque << "\n\n";
        logSteering << " frontSteeringAngle: " << m_nonSafeState.frontSteeringAngle << "\n";
        logSteering << " frontSteeringTimestamp: " << m_nonSafeState.frontSteeringTimestamp << "\n";

        dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_LIGHTGREEN, m_renderEngine);
        dwRenderEngine_renderText2D(logSteering.str().c_str(), {10.0f, 320.0f}, m_renderEngine);

        std::stringstream logMisc;
        logMisc << "Miscellaneous"
                << "\n";
        logMisc << "\n";
        logMisc << " gear:  " << static_cast<uint32_t>(m_nonSafeState.gearStatus) << "\n";
        logMisc << " turnSignal:  " << static_cast<uint32_t>(m_nonSafeState.turnSignalStatus) << "\n\n";
        logMisc << " timestamp:  " << m_nonSafeState.timestamp_us << "\n";

        dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_ORANGE, m_renderEngine);
        dwRenderEngine_renderText2D(logMisc.str().c_str(), {400.0f, 60.0f}, m_renderEngine);

        std::stringstream logSpeed;
        logSpeed << " Speed Data"
                 << "\n";
        logSpeed << "\n";
        logSpeed << " VehicleSpeed:  " << m_nonSafeState.speedESC << "\n";
        logSpeed << " wheelSpeed FL:  " << m_nonSafeState.wheelSpeed[DW_VEHICLE_WHEEL_FRONT_LEFT] << "\n";
        logSpeed << " wheelSpeed FR:  " << m_nonSafeState.wheelSpeed[DW_VEHICLE_WHEEL_FRONT_RIGHT] << "\n";
        logSpeed << " wheelSpeed RL:  " << m_nonSafeState.wheelSpeed[DW_VEHICLE_WHEEL_REAR_LEFT] << "\n";
        logSpeed << " wheelSpeed RR:  " << m_nonSafeState.wheelSpeed[DW_VEHICLE_WHEEL_REAR_RIGHT] << "\n\n";

        logSpeed << " wheelTimestamp FL:  " << m_nonSafeState.wheelTicksTimestamp[DW_VEHICLE_WHEEL_FRONT_LEFT] << "\n";
        logSpeed << " wheelTimestamp FR:  " << m_nonSafeState.wheelTicksTimestamp[DW_VEHICLE_WHEEL_FRONT_RIGHT] << "\n";
        logSpeed << " wheelTimestamp RL:  " << m_nonSafeState.wheelTicksTimestamp[DW_VEHICLE_WHEEL_REAR_LEFT] << "\n";
        logSpeed << " wheelTimestamp RR:  " << m_nonSafeState.wheelTicksTimestamp[DW_VEHICLE_WHEEL_REAR_RIGHT] << "\n";

        dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_LIGHTBLUE, m_renderEngine);
        dwRenderEngine_renderText2D(logSpeed.str().c_str(), {10.0f, 60.0f}, m_renderEngine);

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    ///------------------------------------------------------------------------------
    /// React to user inputs
    ///------------------------------------------------------------------------------
    void onKeyDown(int key, int /*scancode*/, int /*mods*/) override
    {
        static float32_t steeringStep = 15.0f * static_cast<float32_t>(M_PI) / 180.0f;

        switch (key)
        {
        case GLFW_KEY_E:
            resetCmd();
            m_cmdEnable = true;
            break;

        case GLFW_KEY_UP:
            // Validity
            setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlDrivePositionCommand);

            if (dwSignal_checkSignalValidity(m_safeCmd.validityInfo.longCtrlBrakePedalRequest) == DW_SUCCESS)
            {
                m_safeCmd.longCtrlBrakePedalRequest = 0;
                setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlBrakePedalRequest);
                break;
            }
            m_safeCmd.longCtrlThrottlePedalRequest += 0.07;
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlThrottlePedalRequest);
            break;

        case GLFW_KEY_DOWN:
            if (dwSignal_checkSignalValidity(m_safeCmd.validityInfo.longCtrlThrottlePedalRequest) == DW_SUCCESS)
            {
                m_safeCmd.longCtrlThrottlePedalRequest = 0;
                setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlThrottlePedalRequest);
                break;
            }
            m_safeCmd.longCtrlBrakePedalRequest += 0.04;
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlBrakePedalRequest);
            break;

        case GLFW_KEY_LEFT:
            m_safeCmd.latCtrlSteeringWheelAngleRequest += steeringStep;
            setSignalValidIf(true, m_safeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
            break;

        case GLFW_KEY_RIGHT:
            m_safeCmd.latCtrlSteeringWheelAngleRequest -= steeringStep;
            setSignalValidIf(true, m_safeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
            break;

        case GLFW_KEY_F1:
            // Shall come over DI. No longer supported by this sample(turnSig)
            break;

        case GLFW_KEY_F2:
            // Shall come over DI. No longer supported by this sample(turnSig)
            break;

        case GLFW_KEY_F3:
            m_safeCmd.longCtrlAccelRequest += 0.02;
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlAccelRequest);
            break;

        case GLFW_KEY_F4:
            m_safeCmd.longCtrlAccelRequest -= 0.02;
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlAccelRequest);
            break;

        case GLFW_KEY_D:
            resetCmd();
            resetMiscCmd();
            break;

        case GLFW_KEY_P:
            setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlDrivePositionCommand);
            m_safeCmd.longCtrlDrivePositionCommand = DW_VIO_LONG_CTRL_DRIVE_POSITION_COMMAND_P;
            break;

        case GLFW_KEY_I:
            resetMiscCmd();
            break;

        case GLFW_KEY_L:
            // Shall come over DI. No longer supported by this sample(doorLock)
            break;

        case GLFW_KEY_U:
            // Shall come over DI. No longer supported by this sample(doorLock)
            break;

        case GLFW_KEY_S:
            // Shall come over DI. No longer supported by this sample(moonroof)
            break;

        case GLFW_KEY_M:
            // Shall come over DI. No longer supported by this sample(mirrors)
            break;

        case GLFW_KEY_H:
            // Shall come over DI. No longer supported by this sample(headlights)
            break;

        case GLFW_KEY_B:
            // Shall come over DI. No longer supported by this sample(displayBrightnessValue)
            break;

        case GLFW_KEY_N:
            // Shall come over DI. No longer supported by this sample(displayBrightnessValue)
            break;

        case GLFW_KEY_F10:
            // Shall come over DI. No longer supported by this sample(mirrors)
            break;

        default:
            DW_LOGW << "VehicleIO sample: No command for this key\n";
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    // parse user given arguments and bail out if there is --help request or proceed
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("rig", (dw_samples::SamplesDataPath::get() + "/samples/vehicleio/rig.json").c_str()),
                           ProgramArguments::Option_t("interval", "10", "Time interval between VIO in msec. The default is 10msec"),
                           ProgramArguments::Option_t("allow-no-safety", "no", "Allow issuing actuation commands without software limiters. To allow, set to 'yes'.")});

    // -------------------
    // initialize and start a window application (with offscreen support if required)
    VehicleIOSample app(args);

    app.initializeWindow("Vehicle IO Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
