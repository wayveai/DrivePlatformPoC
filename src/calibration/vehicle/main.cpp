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
//SDK
#include <dw/core/base/Version.h>
#include <dw/rig/Rig.h>
#include <dw/egomotion/base/Egomotion.h>
#include <dw/calibration/engine/Engine.h>

//Sensor Input
#include <dw/sensors/sensormanager/SensorManager.h>
#include <dw/sensors/canbus/Interpreter.h>

//Rendering
#include <dwvisualization/core/RenderEngine.h>

//Sample common files
#include <framework/DriveWorksSample.hpp>
#include <framework/Checks.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/MathUtils.hpp>
#include <framework/WindowGLFW.hpp>

using namespace dw_samples::common;

class VehicleCalibrationSample : public DriveWorksSample
{
private:
    //Modules
    dwContextHandle_t m_context                       = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz              = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine             = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                               = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfiguration                  = DW_NULL_HANDLE;
    dwEgomotionHandle_t m_egoMotion                   = DW_NULL_HANDLE;
    dwCalibrationRoutineHandle_t m_calibrationRoutine = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO                   = DW_NULL_HANDLE;

    dwCalibrationEngineHandle_t m_calibrationEngine = DW_NULL_HANDLE;
    const dwVehicle* m_vehicle                      = nullptr;

    dwImageProperties m_cameraOutputProperties{};

    // Calibration Info
    uint32_t m_IMUIndex    = std::numeric_limits<decltype(m_IMUIndex)>::max();
    uint32_t m_cameraIndex = std::numeric_limits<decltype(m_cameraIndex)>::max();
    uint32_t m_vehCANIdx   = std::numeric_limits<decltype(m_vehCANIdx)>::max();

    dwCalibrationStatus m_status;
    dwCameraModelHandle_t m_camera = DW_NULL_HANDLE;

    // Data looping / timestamp adaptation
    dwTime_t m_loopDataOffsetTimestamp = 0;
    dwTime_t m_loopDataFirstTimestamp  = 0;
    dwTime_t m_loopDataLatestTimestamp = 0;

    //Camera Sensor
    dwImageHandle_t m_imageRgba;
    dwImageHandle_t* m_currentImageRgba  = nullptr;
    dwTime_t m_currentImageRgbaTimestamp = {};

    //Rendering
    const dwVector4f m_colorBlue = {0.0f / 255.0f, 0.0f / 255.0f, 230.0f / 255.0f, 1.0f};

    static constexpr uint32_t LINESPACING          = 20;
    static constexpr uint32_t BORDEROFFSET         = 20;
    static constexpr uint32_t NUM_PITCHROLL_POINTS = 12;

    dwVector2f m_scaledCameraDimensions{};
    dwVector2f m_cameraDimensions{};
    dwVector3f m_cameraOpticalCenterDir{};

    dwSensorManagerHandle_t m_sensorManager = DW_NULL_HANDLE;

    std::unique_ptr<dw_samples::common::SimpleImageStreamerGL<dwImageGL>> m_streamerInput2GL;

public:
    VehicleCalibrationSample(const ProgramArguments& args)
        : DriveWorksSample(args)
        , m_status{.started = false, .state = DW_CALIBRATION_STATE_NOT_ACCEPTED, .percentageComplete = 0.0f}
    {
    }

    // -----------------------------------------------------------------------------
    // Initialize all modules
    bool onInitialize() override
    {
        initializeDriveWorks();

        initializeModules();

        initializeSensorManager();

        initializeCameraSensor();

        startSensorManager();

        startCalibration();

        if (getWindow())
        {
            initializeRenderer();
        }

        return true;
    }

    // -----------------------------------------------------------------------------
    // Render vehicle steering offset calibration status at the bottom left corner orner
    void onRender() override
    {
        CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        if (m_currentImageRgba)
        {
            dwImageGL const* imageGL = m_streamerInput2GL->post(*m_currentImageRgba);
            dwVector2f range{};
            range.x = imageGL->prop.width;
            range.y = imageGL->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));
        }

        dwVector2f range = {static_cast<float32_t>(getWindowWidth()), static_cast<float32_t>(getWindowHeight())};
        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));

        //now render text to the screen
        CHECK_DW_ERROR(dwRenderEngine_setColor(m_colorBlue, m_renderEngine));

        std::stringstream ss;
        ss << "Rendering:\n\n";
        ss << convertStatusToText() << "\n\n";
        ss << convertResultToText() << "\n\n";

        if (m_currentImageRgba)
        {
            ss << "Timestamp: " << m_currentImageRgbaTimestamp;
        }

        float32_t currentHeight = getWindowHeight() - 24 * LINESPACING;
        CHECK_DW_ERROR(dwRenderEngine_renderText2D(ss.str().c_str(), {BORDEROFFSET, currentHeight}, m_renderEngine));

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    // -----------------------------------------------------------------------------
    // Read the CAN, IMU and camera frames and operate on them
    void onProcess() override
    {
        m_currentImageRgba = nullptr;

        // Loop over signals until an image is available
        while (!m_currentImageRgba)
        {

            const dwSensorEvent* acquiredEvent = nullptr;
            auto status                        = dwSensorManager_acquireNextEvent(&acquiredEvent, 0, m_sensorManager);

            if (status == DW_TIME_OUT)
                return;

            if (status != DW_SUCCESS)
            {
                if (status != DW_END_OF_STREAM)
                    printf("Error reading sensor %s\n", dwGetStatusName(status));
                else
                {
                    handleEndOfStream();
                    break;
                }
            }

            // handle signals
            switch (acquiredEvent->type)
            {
            case DW_SENSOR_CAN:
            {
                handleCAN(acquiredEvent->canFrame);
                break;
            }

            case DW_SENSOR_IMU:
            {
                handleIMU(acquiredEvent->imuFrame);
                break;
            }

            case DW_SENSOR_CAMERA:
            {
                handleCamera(acquiredEvent->camFrames[0]);
                break;
            }

            case DW_SENSOR_GPS:
            case DW_SENSOR_RADAR:
            case DW_SENSOR_TIME:
            case DW_SENSOR_LIDAR:
            case DW_SENSOR_DATA:
            case DW_SENSOR_COUNT:
            default:
                break;
            }

            CHECK_DW_ERROR(dwSensorManager_releaseAcquiredEvent(acquiredEvent, m_sensorManager));
        }
    }

    // -----------------------------------------------------------------------------
    // Resize window
    void onResizeWindow(int width, int height) override
    {
        CHECK_DW_ERROR(dwRenderEngine_reset(m_renderEngine));
        dwRectf rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));

        m_scaledCameraDimensions.x = static_cast<float32_t>(width) / m_cameraDimensions.x;

        m_scaledCameraDimensions.y = static_cast<float32_t>(height) / m_cameraDimensions.y;
    }

    // -----------------------------------------------------------------------------
    // Release all modules
    void onRelease() override
    {
        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        m_streamerInput2GL.reset();

        if (m_camera)
        {
            CHECK_DW_ERROR(dwCameraModel_release(m_camera));
        }

        if (m_calibrationEngine)
        {
            CHECK_DW_ERROR(dwCalibrationEngine_stopCalibration(m_calibrationRoutine, m_calibrationEngine))
            CHECK_DW_ERROR(dwCalibrationEngine_release(m_calibrationEngine));
        }

        if (m_egoMotion)
        {
            CHECK_DW_ERROR(dwEgomotion_release(m_egoMotion));
        }

        if (m_vehicleIO)
        {
            CHECK_DW_ERROR(dwVehicleIO_release(m_vehicleIO));
        }

        if (m_rigConfiguration)
        {
            CHECK_DW_ERROR(dwRig_release(m_rigConfiguration));
        }

        if (m_sensorManager)
        {
            CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
            CHECK_DW_ERROR(dwSensorManager_release(m_sensorManager));
        }

        if (m_sal)
        {
            CHECK_DW_ERROR(dwSAL_release(m_sal));
        }

        if (m_viz)
        {
            CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        }
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    // -----------------------------------------------------------------------------
    // Reset egomotion and sensor manager
    void onReset() override
    {
        // restart on reset
        CHECK_DW_ERROR(dwEgomotion_reset(m_egoMotion));

        // reset sensor manager
        CHECK_DW_ERROR(dwSensorManager_stop(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_reset(m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));

        m_loopDataOffsetTimestamp += m_loopDataLatestTimestamp - m_loopDataFirstTimestamp;
        m_loopDataFirstTimestamp = 0;
    }

private:
    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks()
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};

#ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
#endif

        CHECK_DW_ERROR(dwInitialize(&m_context, DW_VERSION, &sdkParams));
    }

    // -----------------------------------------------------------------------------
    // Initialize rig configuration
    // Initialize egomotion
    // Initialize vehicle io
    // Initialize calibration engine
    void initializeModules()
    {
        uint32_t imuIndex = static_cast<uint32_t>(std::stoul(getArgument("imuIndex")));
        uint32_t canIndex = static_cast<uint32_t>(std::stoul(getArgument("canIndex")));

        //initialize our rig configuration module
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rigConfiguration, m_context, getArgument("rig").c_str()));

        CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&m_IMUIndex, DW_SENSOR_IMU, imuIndex, m_rigConfiguration));

        CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&m_vehCANIdx, DW_SENSOR_CAN, canIndex, m_rigConfiguration));

        CHECK_DW_ERROR(dwRig_getVehicle(&m_vehicle, m_rigConfiguration));

        dwEgomotionParameters params{};
        CHECK_DW_ERROR(dwEgomotion_initParamsFromRigByIndex(&params, m_rigConfiguration,
                                                            m_IMUIndex, m_vehCANIdx));
        params.historySize     = 10000;
        params.automaticUpdate = true;

        params.motionModel = DW_EGOMOTION_IMU_ODOMETRY;
        CHECK_DW_ERROR(dwEgomotion_initialize(&m_egoMotion, &params, m_context));

        // initialize vehicle io
        CHECK_DW_ERROR(dwVehicleIO_initializeFromRig(&m_vehicleIO, m_rigConfiguration, m_context));

        //finally initialize our calibration module
        CHECK_DW_ERROR(dwCalibrationEngine_initialize(&m_calibrationEngine, m_rigConfiguration, m_context));
    }

    // -----------------------------------------------------------------------------
    // Initialize camera sensor
    void initializeCameraSensor()
    {
        uint32_t cameraIndex = static_cast<uint32_t>(std::stoul(getArgument("cameraIndex")));

        CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&m_cameraIndex, DW_SENSOR_CAMERA,
                                                   cameraIndex, m_rigConfiguration));

        dwSensorHandle_t cameraSensor = DW_NULL_HANDLE;
        CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&cameraSensor, m_cameraIndex, m_sensorManager));

        CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&m_cameraOutputProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, cameraSensor));

        CHECK_DW_ERROR(dwImage_create(&m_imageRgba, m_cameraOutputProperties, m_context));

        CHECK_DW_ERROR(dwCameraModel_initialize(&m_camera, 0, m_rigConfiguration));

        m_cameraDimensions.x = m_cameraOutputProperties.width;
        m_cameraDimensions.y = m_cameraOutputProperties.height;

        m_scaledCameraDimensions.x = static_cast<float32_t>(getWindowWidth()) / m_cameraDimensions.x;
        m_scaledCameraDimensions.y = static_cast<float32_t>(getWindowHeight()) / m_cameraDimensions.y;
    }

    // -----------------------------------------------------------------------------
    // Initialize sensor manager
    void initializeSensorManager()
    {
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
        CHECK_DW_ERROR(dwSensorManager_initializeFromRig(&m_sensorManager, m_rigConfiguration, 1024, m_sal));
    }

    // -----------------------------------------------------------------------------
    // Initialize render engine
    void initializeRenderer()
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        {
            dwImageProperties displayProperties = m_cameraOutputProperties;
            displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;
            m_streamerInput2GL                  = std::make_unique<SimpleImageStreamerGL<dwImageGL>>(displayProperties, 1000, m_context);
        }

        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        params.defaultTile.lineWidth = 2.0f;
        params.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_16;
        params.maxBufferCount        = 4;
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        CHECK_DW_ERROR(dwCameraModel_pixel2Ray(&m_cameraOpticalCenterDir.x,
                                               &m_cameraOpticalCenterDir.y,
                                               &m_cameraOpticalCenterDir.z,
                                               m_cameraDimensions.x / 2.f,
                                               m_cameraDimensions.y / 2.f,
                                               m_camera));

        glDepthFunc(GL_ALWAYS);
    }

    // -----------------------------------------------------------------------------
    // Start sensor manager for handling camera, CAN, and IMU data
    void startSensorManager()
    {
        CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));
    }

    // -----------------------------------------------------------------------------
    // Initialize vehicle calibration engine
    // Start calibration
    void startCalibration()
    {

        dwCalibrationVehicleParams vehicleParams{};
        CHECK_DW_ERROR(dwCalibrationEngine_initializeVehicle(&m_calibrationRoutine, m_vehCANIdx,
                                                             &vehicleParams,
                                                             m_egoMotion, m_vehicle, m_calibrationEngine));
        // Start calibration
        CHECK_DW_ERROR(dwCalibrationEngine_startCalibration(m_calibrationRoutine, m_calibrationEngine))
    }

    // -----------------------------------------------------------------------------
    // Read CAN data, feed and update egomotion module
    void handleCAN(const dwCANMessage& canMsg)
    {
        dwCANMessage msg = canMsg;
        msg.timestamp_us += m_loopDataOffsetTimestamp;
        if (!m_loopDataFirstTimestamp)
            m_loopDataFirstTimestamp = msg.timestamp_us;
        m_loopDataLatestTimestamp    = std::max(msg.timestamp_us, m_loopDataLatestTimestamp);

        CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&msg, m_vehCANIdx, m_vehicleIO));

        dwVehicleIOSafetyState vehicleIOSafetyState{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&vehicleIOSafetyState, m_vehicleIO));

        dwVehicleIONonSafetyState vehicleIONonSafetyState{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&vehicleIONonSafetyState, m_vehicleIO));

        dwVehicleIOActuationFeedback vehicleIOActuationFeedback{};
        CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&vehicleIOActuationFeedback, m_vehicleIO));

        CHECK_DW_ERROR(dwEgomotion_addVehicleIOState(&vehicleIOSafetyState, &vehicleIONonSafetyState, &vehicleIOActuationFeedback, m_egoMotion));

        CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIONonSafetyState(&vehicleIONonSafetyState, m_vehCANIdx, m_calibrationEngine));
        CHECK_DW_ERROR(dwCalibrationEngine_addVehicleIOActuationFeedback(&vehicleIOActuationFeedback, m_vehCANIdx, m_calibrationEngine));

        CHECK_DW_ERROR(dwCalibrationEngine_getCalibrationStatus(&m_status, m_calibrationRoutine, m_calibrationEngine));

        // slow down sample if calibration converged
        if (m_status.state == DW_CALIBRATION_STATE_ACCEPTED)
            setProcessRate(60);
        else
            setProcessRate(1000);
    }

    // -----------------------------------------------------------------------------
    // Get IMU frame and feed egomotion and calibration engine
    void handleIMU(const dwIMUFrame& imuFrame)
    {
        dwIMUFrame msg = imuFrame;

        msg.hostTimestamp += m_loopDataOffsetTimestamp;
        if (!m_loopDataFirstTimestamp)
            m_loopDataFirstTimestamp = msg.hostTimestamp;
        m_loopDataLatestTimestamp    = std::max(msg.hostTimestamp, m_loopDataLatestTimestamp);

        dwEgomotion_addIMUMeasurement(&msg, m_egoMotion);
    }

    // -----------------------------------------------------------------------------
    // Get camera image and estimate gravity
    void handleCamera(const dwCameraFrameHandle_t& frame)
    {
        dwImageHandle_t image;
        CHECK_DW_ERROR(dwSensorCamera_getImage(&image, DW_CAMERA_OUTPUT_CUDA_YUV420_UINT8_SEMIPLANAR, frame));

        CHECK_DW_ERROR(dwImage_copyConvert(m_imageRgba, image, m_context));
        m_currentImageRgba = &m_imageRgba;
        CHECK_DW_ERROR(dwImage_getTimestamp(&m_currentImageRgbaTimestamp, image));
    }

    // -----------------------------------------------------------------------------
    // Reset in case of end of stream
    void handleEndOfStream()
    {
        reset();
    }

    // -----------------------------------------------------------------------------
    // Convert calibration status to text
    std::string convertStatusToText()
    {
        std::stringstream ss;

        const char* str = nullptr;
        CHECK_DW_ERROR(dwCalibrationState_toString(&str, m_status.state));
        ss << "State: " << str;

        ss.precision(2);
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss << " (" << 100.f * m_status.percentageComplete << "%)";

        return ss.str();
    }

    // -----------------------------------------------------------------------------
    // Convert calibration result to text
    std::string convertResultToText()
    {
        std::stringstream ss;
        if (m_status.state == DW_CALIBRATION_STATE_ACCEPTED)
        {
            dwVehicleSteeringProperties steering{};
            CHECK_DW_ERROR(dwCalibrationEngine_getVehicleSteeringProperties(&steering, m_calibrationRoutine, m_calibrationEngine));

            ss.precision(4);
            ss.setf(std::ios::fixed, std::ios::floatfield);
            ss << "Calibrated Steering Offset(deg): " << RAD2DEG(steering.frontSteeringOffset);
        }
        else
        {
            ss << "Calibrated Steering Offset(deg): "
               << " n/a";
        }

        return ss.str();
    }
};

//#######################################################################################
int main(int argc, const char** argv)
{
    const std::string samplePath = dw_samples::SamplesDataPath::get() + "/samples/recordings/highway0/";

    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("rig", (samplePath + "rig.json").c_str(), "Path to rig configuration file."),
                              ProgramArguments::Option_t("cameraIndex", "0", "The index in the rig file of the camera for visualization"),
                              ProgramArguments::Option_t("imuIndex", "0", "The index in the rig file of the imu"),
                              ProgramArguments::Option_t("canIndex", "0", "The index in the rig file of the vehicle can"),
                          });

    VehicleCalibrationSample app(args);

    if (args.get("offscreen") != "2")
    {
        app.initializeWindow("Steering Calibration Sample", 1280, 800, args.get("offscreen") == "1");
    }

    return app.run();
}
