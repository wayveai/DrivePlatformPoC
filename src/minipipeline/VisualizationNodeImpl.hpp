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
// SPDX-FileCopyrightText: Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SMP_VISUALIZATION_NODE_IMPL_HPP_
#define SMP_VISUALIZATION_NODE_IMPL_HPP_

#include <dwcgf/node/SimpleNodeT.hpp>
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/radar/Radar.h>
#include <dwvisualization/core/RenderEngine.h>
#include <dw/control/vehicleio/VehicleIO.h>
#include <dw/imageprocessing/features/FeatureList.h>
#include <dwvisualization/interop/ImageStreamer.h>
#include <dw/imageprocessing/geometry/imagetransformation/ImageTransformation.h>

#include "VisualizationNode.hpp"
#include "WindowApp.hpp"
#include "ContentLayout.hpp"
#include <framework/MathUtils.hpp>

namespace minipipeline
{
/**
* LaneSafetyCheck 3D trackball
* Simple track ball for RR radar safety check visualization
* The track ball has top down view by default and is controlled by mouse and key
* mouse: left key to rotate, wheel to zoom
* key: "-" to zoom out and "=" to zoom in, "r" to reset to default view
*/
struct SimpleTrackBall
{

public:
    SimpleTrackBall()
    {
        Init();
    }

    ~SimpleTrackBall() {}

    void Init()
    {
        deltaVerAngle = 0.0f;
        deltaHorAngle = 0.0f;
        deltaMove     = 0.0f;
        radius        = 200.0f;
        nearPlane     = 10.0f;
        farPlane      = 1000.0f;

        vertAngle = DEG2RAD(89.9f);
        horAngle  = DEG2RAD(179.9);
        fovRads   = DEG2RAD(60.0f);

        up[0]     = 0.0f;
        up[1]     = 0.0f;
        up[2]     = 1.0f;
        center[0] = 0.0f;
        center[1] = 0.0f;
        center[2] = 0.0f;

        memset(modelview.array, 0, 16 * sizeof(float));
        memset(projection.array, 0, 16 * sizeof(float));

        mouseLeft  = false;
        mouseRight = false;

        currentX = -1;
        currentY = -1;

        UpdateEye();
    }

    void UpdateEye()
    {
        eye[0] = radius * cos(vertAngle + deltaVerAngle) * cos(horAngle + deltaHorAngle);
        eye[1] = radius * cos(vertAngle + deltaVerAngle) * sin(horAngle + deltaHorAngle);
        eye[2] = radius * sin(vertAngle + deltaVerAngle);
    }

    // Update eye from cartesian coordinate eye position
    void UpdateEye(const float x, const float y, const float z)
    {
        radius    = std::sqrt(x * x + y * y + z * z);
        vertAngle = std::fabs(std::asin(z / radius));
        horAngle  = std::atan2(y, x);

        UpdateEye();
    }

    const dwMatrix4f* GetModelView()
    {
        // Compute model view matrix and return
        lookAt(modelview.array, eye, center, up);
        return &modelview;
    }

    // Get model view when eye and center are both offset by offsetX & offsetY
    const dwMatrix4f* GetModelView(const float32_t offsetX, const float32_t offsetY)
    {
        // Update radius, vertAngle, and horAngle (= r, theta, pi in polar coordinates)
        if (offsetX != 0 || offsetY != 0)
        {
            UpdateEye(eye[0] + offsetX, eye[1] + offsetY, eye[2]);

            // Update center
            center[0] += offsetX;
            center[1] += offsetY;
        }

        return GetModelView();
    }

    const dwMatrix4f* GetProjection(float aspect)
    {
        // Compute projection matrix and return
        perspective(projection.array, fovRads, aspect, nearPlane, farPlane);
        return &projection;
    }

    void OnLeftMouseDown(float x, float y)
    {
        // only start motion if the left button is pressed
        mouseLeft = true;
        currentX  = (int32_t)floor(x);
        currentY  = (int32_t)floor(y);
    }

    void OnLeftMouseUp()
    {
        mouseLeft = false;
        vertAngle += deltaVerAngle;
        horAngle += deltaHorAngle;
        deltaHorAngle = 0.0f;
        deltaVerAngle = 0.0f;
    }

    void OnMouseMove(float x, float y)
    {
        // this will only be true when the left button is down
        if (mouseLeft)
        {
            // update deltaAngle
            deltaVerAngle = (y - currentY);
            deltaHorAngle = -(x - currentX);
            // scale deltaAngle
            deltaVerAngle *= 0.005f;
            deltaHorAngle *= 0.005f;
            // Limit the vertical angle (0.1 to 89.9 degrees)
            if ((vertAngle + deltaVerAngle) > DEG2RAD(89.9))
                deltaVerAngle = DEG2RAD(89.9) - vertAngle;
            if ((vertAngle + deltaVerAngle) < DEG2RAD(0.1))
                deltaVerAngle = DEG2RAD(0.1) - vertAngle;

            UpdateEye();
        }
    }

    void OnMouseWheel(float offset)
    {
        radius -= 15.0 * offset;
        if (radius > farPlane)
            radius = farPlane - 1.0;
        else if (radius < nearPlane)
            radius = nearPlane + 1.0;

        UpdateEye();
    }

private:
    // Initialize 3D view related variables.
    float deltaVerAngle;
    float deltaHorAngle;
    float deltaMove;
    float radius;
    float nearPlane;
    float farPlane;
    float vertAngle;
    float horAngle;
    float fovRads;

    float eye[3];
    float up[3];
    float center[3];

    dwMatrix4f modelview;
    dwMatrix4f projection;

    bool mouseLeft;
    bool mouseRight;

    int32_t currentX;
    int32_t currentY;
};

class VisualizationNodeImpl : public dw::framework::SimpleNodeT<VisualizationNode>
{
public:
    static constexpr char LOG_TAG[] = "VisualizationNode";
    using Base                      = SimpleNodeT<VisualizationNode>;

    // Initialization and destruction
    VisualizationNodeImpl(const VisualizationNodeParams& params, const dwContextHandle_t ctx);
    ~VisualizationNodeImpl() override;

    dwStatus reset() final;
    dwStatus teardownImpl() override final;

    // VIO command handling functions
    void updateVIOLongCtrlAccelRequest(float32_t diff);
    void updateVIOLatCtrlSteeringWheelAngleRequest(float32_t diff);
    void updateVIOLongCtrlDrivePositionCommand();
    void setSignalValidIf(bool valid, dwSignalValidity& validity);

private:
    /**
     * @brief Screen views, change by pressing key F3.
     *        Default screen view is SCREEN_SINGLE_CAMERA.
     */
    enum class SensorScreenSelect : uint8_t
    {
        SCREEN_SINGLE_CAMERA = 0,
        SCREEN_VAL_CONFIG,
        SCREEN_MULTI_CAMERA,
        SCREEN_RADAR_POINT_CLOUD,
        SCREEN_LIDAR_POINT_CLOUD,
        MAX_SENSOR_SCREEN,
    };

    /**
     * @brief Render frame sequence.
     */
    enum class RenderType : uint8_t
    {
        RENDER_ACQUIRE_FRAME = 0,
        RENDER_CAMERA_FRAME,
        RENDER_INFO_BAR,
        RENDER_TRACKED_OBJECT,
        //These should be disabled on default
        RENDER_DEBUG,
        RENDER_COUNT
    };

    uint8_t m_requestSensorScreen{};
    uint8_t m_currentSensorScreen{};

    void initialize();
    void initPorts();
    void initPasses();

    void initRenderBuffer();
    void releaseRenderBuffer();

    dwStatus preProcessImage();
    void preProcess3DView(const bool updateView, const bool updateProjection, const bool hasGrid);
    void registerEventCallback();

    // Window App user callback functions
    static void onToggleCameraViewMode(void* instance_);
    static void onMouseMoveCallback(void* instance_, float x, float y);
    static void onMouseUpCallback(void* instance_, int button, float x, float y, int mods);
    static void onMouseDownCallback(void* instance_, int button, float x, float y, int mods);
    static void onMouseWheelCallback(void* instance_, float dx, float dy);
    static void onKeyPressCallback(void* instance_, int key, int scancode, int action, int mods);

    // Internal states of node
    size_t m_epochCount;
    VisualizationNodeParams m_params{};

    //////////////////////
    // Render
    //////////////////////
    static constexpr uint32_t RENDER_BUFFER_SIZE = 96 * 1024;

    // Functions for rendering
    void initializeRenderer(uint32_t width, uint32_t height);
    dwStatus renderCameraViews(RenderType render);
    void renderViewDetections(RenderType render, size_t camIdx);
    void renderInfoBar(bool inMultiview);
    void settingRenderScreen();
    void renderMainInfo(bool inMultiview);

    // Render rect
    void setRenderRect(uint32_t x, uint32_t y, uint32_t width, uint32_t height);

    // Render Radar
    dwRadarScan m_radarProcessedData;
    dwStatus getRadarData();
    void renderRadar3DPointCloud();

    // Render IMU
    using IMUString = dw::framework::FixedString<1024>;
    IMUString getIMUdata(dwIMUFrame& frame);
    inline void upadteIMUString(IMUString& result, const char8_t* tag, float64_t value)
    {
        if (std::isnan(value))
        {
            return;
        }
        result += tag;
        result += ":";
        result += value;
        result += " ";
    }

    // Render Lidar
    dwPointCloud m_cpuPointCloud{};
    void renderLidar3DPointCloud();

    // Render VAL
    using VALString = dw::framework::FixedString<64>;
    dwVehicleIOSafetyCommand m_safeCmd{};
    float32_t m_currentSpeed;
    void resetSafetyCommand();
    void renderVALConfig();

    // Variables for rendering
    dwImageHandle_t m_cudaRgbaRenderImage[MAX_CAMERA_COUNT] = {};
    dwImageHandle_t m_frameGL[MAX_CAMERA_COUNT]             = {};
    dwImageTransformationHandle_t m_cudaRgbaImageTransform  = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz                    = DW_NULL_HANDLE;
    dwContextHandle_t m_ctx                                 = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine                   = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer                           = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_image2GL[MAX_CAMERA_COUNT]    = {};
    dwImageGL* m_currentImageGL[MAX_CAMERA_COUNT]           = {};
    SimpleTrackBall m_trackBall{};
    dwRect m_screenRect{};
    bool m_updateLayout = true;

    // ContentLayout share pointers
    std::shared_ptr<ContentLayout> m_pstLayout           = nullptr;
    std::shared_ptr<ContentLayout> m_singleContentLayout = nullptr;
    std::shared_ptr<ContentLayout> m_multiContentLayout  = nullptr;

    // Variables for renderBuffer
    static constexpr uint32_t POINT_CLOUD_MAX_CAPACITY = 2500000;
    uint32_t m_pointCloudBuffer{};
    uint32_t m_gridBuffer{};
    uint32_t m_gridBufferPrimitiveCount{};

#ifdef PERCEPTION_ENABLED
    // perception
    using YoloClassName = dw::core::FixedString<16>;
    dw::core::VectorFixed<YoloClassName, YOLO_MAX_BOX_NUM> m_label;
    dw::core::VectorFixed<dwRectf, YOLO_MAX_BOX_NUM> m_trackedBoxListFloat;
    uint32_t m_boxNum                       = 0;
    const YoloClassName YOLO_CLASS_NAMES[3] = {"yellow_light", "green_light", "red_light"};
#endif
};

} // namespace minipipeline

#endif // SMP_VISUALIZATION_NODE_IMPL_HPP_
