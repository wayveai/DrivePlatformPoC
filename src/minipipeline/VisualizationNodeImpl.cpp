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

#include "VisualizationNodeImpl.hpp"
#include <dwshared/dwfoundation/dw/core/logger/Logger.hpp>
#include <dw/pointcloudprocessing/pointcloud/PointCloud.h>

#include <framework/Window.hpp>
#include <framework/Checks.hpp>

#include <unistd.h>
#ifndef DW_USE_EGL
#include <GL/glxew.h> // for glXGetCurrentContext()
#endif

namespace minipipeline
{

using namespace dw;

constexpr char VisualizationNodeImpl::LOG_TAG[];

void VisualizationNodeImpl::initPorts()
{
    // Create input/output ports
    NODE_INIT_INPUT_ARRAY_PORTS("IMAGE"_sv);
    NODE_INIT_INPUT_PORT("RADAR_PROCESSED_DATA"_sv);
    NODE_INIT_INPUT_PORT("IMU_FRAME"_sv);
    NODE_INIT_INPUT_ARRAY_PORTS("LIDAR_POINT_CLOUD"_sv);
    NODE_INIT_INPUT_ARRAY_PORTS("LIDAR_PACKET_ARRAYS"_sv);
    NODE_INIT_INPUT_PORT("VIO_SAFETY_STATE"_sv);
    NODE_INIT_INPUT_PORT("VIO_NON_SAFETY_STATE"_sv);
    NODE_INIT_OUTPUT_PORT("VIO_SAFETY_CMD"_sv);
#ifdef PERCEPTION_ENABLED
    NODE_INIT_INPUT_PORT("BOX_NUM"_sv);
    NODE_INIT_INPUT_PORT("BOX_ARR"_sv);
#endif
}

void VisualizationNodeImpl::initPasses()
{
    NODE_REGISTER_PASS("ACQUIRE_FRAME"_sv, [this]() {
        if (m_params.offscreen)
        {
            return DW_SUCCESS;
        }

        gWindow->makeCurrent();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        dwRectf bounds{0.0f, 0.0f, 0.0f, 0.0f};
        bounds.height = gWindow->height();
        bounds.width  = gWindow->width();
        dwRenderEngine_setBounds(bounds, m_renderEngine);
        settingRenderScreen();
        FRWK_CHECK_DW_ERROR(preProcessImage());
        gWindow->resetCurrent();
        return DW_SUCCESS;
    });
    NODE_REGISTER_PASS("RENDER_FRAME"_sv, [this]() { return renderCameraViews(RenderType::RENDER_CAMERA_FRAME); });
    NODE_REGISTER_PASS("RENDER_INFO_BAR"_sv, [this]() { return renderCameraViews(RenderType::RENDER_INFO_BAR); });
    NODE_REGISTER_PASS("RENDER_DEBUG"_sv, [this]() {
        if (m_params.offscreen)
        {
            m_epochCount++;
            return DW_SUCCESS;
        }

        dwStatus ret = renderCameraViews(RenderType::RENDER_DEBUG);
        gWindow->makeCurrent();
        gWindow->swapBuffers();
        if (gWindow->shouldClose())
        {
            gRun = false;
        }
        gWindow->resetCurrent();
        m_epochCount++;
        return ret; });
}

void VisualizationNodeImpl::onToggleCameraViewMode(void* instance_)
{
    VisualizationNodeImpl* instance = static_cast<VisualizationNodeImpl*>(instance_);

    if (!instance || !gRun)
        return;

    instance->m_requestSensorScreen = (instance->m_requestSensorScreen + 1) % static_cast<uint8_t>(SensorScreenSelect::MAX_SENSOR_SCREEN);
    instance->m_updateLayout        = true;
}

void VisualizationNodeImpl::onKeyPressCallback(void* instance_, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;

    VisualizationNodeImpl* instance = static_cast<VisualizationNodeImpl*>(instance_);
    if (instance == nullptr)
    {
        return;
    }

    if (action == GLFW_PRESS)
    {
        switch (key)
        {
        case GLFW_KEY_UP:
        {
            instance->updateVIOLongCtrlAccelRequest(0.02);
            DW_LOGD << "GLFW_KEY_UP: +accel 0.02" << Logger::State::endl;
            break;
        }
        case GLFW_KEY_DOWN:
        {
            instance->updateVIOLongCtrlAccelRequest(-0.02);
            DW_LOGD << "GLFW_KEY_DOWN: -accel 0.02" << Logger::State::endl;
            break;
        }
        case GLFW_KEY_LEFT:
        {
            instance->updateVIOLatCtrlSteeringWheelAngleRequest(-0.02);
            DW_LOGD << "GLFW_KEY_LEFT: -steering wheel angle 0.02" << Logger::State::endl;
            break;
        }
        case GLFW_KEY_RIGHT:
        {
            instance->updateVIOLatCtrlSteeringWheelAngleRequest(0.02);
            DW_LOGD << "GLFW_KEY_RIGHT: +steering wheel angle 0.02" << Logger::State::endl;
            break;
        }
        case GLFW_KEY_D:
        {
            instance->updateVIOLongCtrlDrivePositionCommand();
            DW_LOGD << "GLFW_KEY_D: +gear state changed" << Logger::State::endl;
            break;
        }
        case GLFW_KEY_R:
        {
            raise(SIGUSR1); // signal nodes reset
            break;
        }
        }
    }
}

void VisualizationNodeImpl::onMouseMoveCallback(void* instance_, float x, float y)
{
    VisualizationNodeImpl* instance = static_cast<VisualizationNodeImpl*>(instance_);
    if (!instance)
    {
        return;
    }

    instance->m_trackBall.OnMouseMove(x, y);
}

void VisualizationNodeImpl::onMouseUpCallback(void* instance_, int button, float x, float y, int mods)
{
    (void)button;
    (void)mods;
    (void)x;
    (void)y;

    VisualizationNodeImpl* instance = static_cast<VisualizationNodeImpl*>(instance_);
    if (!instance)
    {
        return;
    }

    instance->m_trackBall.OnLeftMouseUp();
}

void VisualizationNodeImpl::onMouseDownCallback(void* instance_, int button, float x, float y, int mods)
{
    (void)button;
    (void)mods;
    VisualizationNodeImpl* instance = static_cast<VisualizationNodeImpl*>(instance_);
    if (!instance)
    {
        return;
    }

    instance->m_trackBall.OnLeftMouseDown(x, y);
}

void VisualizationNodeImpl::onMouseWheelCallback(void* instance_, float dx, float dy)
{
    (void)dx;
    VisualizationNodeImpl* instance = static_cast<VisualizationNodeImpl*>(instance_);
    if (!instance)
    {
        return;
    }

    instance->m_trackBall.OnMouseWheel(dy);
}

void VisualizationNodeImpl::registerEventCallback()
{
    gUserCallbackData         = this;
    gUserToggleCameraViewMode = &(VisualizationNodeImpl::onToggleCameraViewMode);
    gUserMouseUpCallback      = &(VisualizationNodeImpl::onMouseUpCallback);
    gUserMouseDownCallback    = &(VisualizationNodeImpl::onMouseDownCallback);
    gUserMouseMoveCallback    = &(VisualizationNodeImpl::onMouseMoveCallback);
    gUserMouseWheelCallback   = &(VisualizationNodeImpl::onMouseWheelCallback);
    gUserKeyPressCallback     = &(VisualizationNodeImpl::onKeyPressCallback);
}

void VisualizationNodeImpl::initialize()
{
    if (!m_params.offscreen)
    {
        gWindow->setWindowVisibility(true);
        if (m_params.fullscreen)
        {
            gWindow->setFullScreen();
        }
        else
        {
            if ((m_params.winSizeW == 0) || (m_params.winSizeH == 0))
            {
                int w, h;
                gWindow->getDesktopResolution(w, h);
                m_params.winSizeW = w;
                m_params.winSizeH = h;
            }
            gWindow->setWindowSize(m_params.winSizeW, m_params.winSizeH);
        }
    }

    gWindow->makeCurrent();
    dwRenderEngineParams renderEngineParams{};
    // Window size same with gWindow in SampleFramwork
    uint32_t screenWidth = gWindow->width(), screenHeight = gWindow->height();
    FRWK_CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&renderEngineParams, screenWidth, screenHeight));
    renderEngineParams.defaultTile.lineWidth = 0.2f;
    renderEngineParams.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_20;
    renderEngineParams.bufferSize            = RENDER_BUFFER_SIZE;

    FRWK_CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &renderEngineParams, m_viz));

    FRWK_CHECK_DW_ERROR(dwRenderer_initialize(&m_renderer, m_viz));

    m_screenRect.x      = 0;
    m_screenRect.y      = 0;
    m_screenRect.width  = screenWidth;
    m_screenRect.height = screenHeight;
    dwRenderer_setRect(m_screenRect, m_renderer);

    dwImageProperties imgProp{};
    imgProp.type   = DW_IMAGE_CUDA;
    imgProp.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    for (size_t camIdx = 0; camIdx < MAX_CAMERA_COUNT; ++camIdx)
    {
        if (m_params.camEnable[camIdx])
        {
            DW_LOGD << "initialize: camIdx: " << camIdx << ", enabled: " << m_params.camEnable[camIdx] << Logger::State::endl;
            imgProp.width  = m_params.imageWidth[camIdx];
            imgProp.height = m_params.imageHeight[camIdx];
            FRWK_CHECK_DW_ERROR(dwImage_create(&m_cudaRgbaRenderImage[camIdx], imgProp, m_ctx));
            FRWK_CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_image2GL[camIdx], &imgProp, DW_IMAGE_GL, m_ctx));
            FRWK_CHECK_DW_ERROR(dwImageStreamerGL_setCUDAStream(m_params.stream, m_image2GL[camIdx]));
        }
    }

    //Init render Window and Layout
    m_currentSensorScreen = static_cast<uint8_t>(SensorScreenSelect::SCREEN_SINGLE_CAMERA);
    m_requestSensorScreen = static_cast<uint8_t>(SensorScreenSelect::SCREEN_SINGLE_CAMERA);

    m_singleContentLayout = std::make_shared<ContentLayout>(m_renderEngine);
    m_multiContentLayout  = std::make_shared<ContentLayout>(m_renderEngine, MAX_CAMERA_COUNT);

    DW_LOGD << "initialize: single layout" << Logger::State::endl;
    m_pstLayout = m_singleContentLayout;
    m_pstLayout->initialize();
    m_updateLayout = false;

    //Init render Buffer
    initRenderBuffer();

    //Init Image Transformation
    dwImageTransformationParameters imgTransformParams{};
    FRWK_CHECK_DW_ERROR(dwImageTransformation_initialize(&m_cudaRgbaImageTransform, imgTransformParams, m_ctx));
    FRWK_CHECK_DW_ERROR(dwImageTransformation_setBorderMode(DW_IMAGEPROCESSING_BORDER_MODE_ZERO, m_cudaRgbaImageTransform));
    FRWK_CHECK_DW_ERROR(dwImageTransformation_setInterpolationMode(DW_IMAGEPROCESSING_INTERPOLATION_DEFAULT, m_cudaRgbaImageTransform));

    m_cpuPointCloud.format   = DW_POINTCLOUD_FORMAT_XYZI;
    m_cpuPointCloud.type     = DW_MEMORY_TYPE_CPU;
    m_cpuPointCloud.capacity = POINT_CLOUD_MAX_CAPACITY;

    FRWK_CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_cpuPointCloud));

    gWindow->resetCurrent();
}

void VisualizationNodeImpl::settingRenderScreen()
{
    if (m_updateLayout)
    {
        m_pstLayout->release();
        m_currentSensorScreen = m_requestSensorScreen;

        switch (m_currentSensorScreen)
        {
        case static_cast<uint8_t>(SensorScreenSelect::SCREEN_SINGLE_CAMERA):
        case static_cast<uint8_t>(SensorScreenSelect::SCREEN_VAL_CONFIG):
        case static_cast<uint8_t>(SensorScreenSelect::SCREEN_RADAR_POINT_CLOUD):
        case static_cast<uint8_t>(SensorScreenSelect::SCREEN_LIDAR_POINT_CLOUD):
        {
            m_pstLayout = m_singleContentLayout;
        }
        break;
        case static_cast<uint8_t>(SensorScreenSelect::SCREEN_MULTI_CAMERA):
        {
            m_pstLayout = m_multiContentLayout;
        }
        break;
        default:
        {
            throw ExceptionWithStatus(DW_OUT_OF_BOUNDS, "settingRenderScreen: screenIndex is not find.");
        }
        }
        m_updateLayout = false;
        m_pstLayout->initialize();
    }
}

void VisualizationNodeImpl::initRenderBuffer()
{
    CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_pointCloudBuffer, DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                               4 * sizeof(float32_t), 0, POINT_CLOUD_MAX_CAPACITY, m_renderEngine))

    CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_gridBuffer, DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D,
                                               sizeof(dwVector3f), 0, 10000, m_renderEngine))
    dwMatrix4f identity = DW_IDENTITY_MATRIX4F;

    CHECK_DW_ERROR(dwRenderEngine_setBufferPlanarGrid3D(m_gridBuffer, {0.f, 0.f, 100.f, 100.f},
                                                        5.0f, 5.0f,
                                                        &identity, m_renderEngine))

    dwRenderEngine_getBufferMaxPrimitiveCount(&m_gridBufferPrimitiveCount, m_gridBuffer, m_renderEngine);
}

void VisualizationNodeImpl::releaseRenderBuffer()
{
    if (m_pointCloudBuffer != 0)
    {
        FRWK_CHECK_DW_ERROR_NOTHROW(dwRenderEngine_destroyBuffer(m_pointCloudBuffer, m_renderEngine))
    }

    if (m_gridBuffer != 0)
    {
        FRWK_CHECK_DW_ERROR_NOTHROW(dwRenderEngine_destroyBuffer(m_gridBuffer, m_renderEngine))
    }
}

VisualizationNodeImpl::VisualizationNodeImpl(const VisualizationNodeParams& params, const dwContextHandle_t ctx)
    : m_epochCount(0)
    , m_params(params)
    , m_ctx(ctx)
{
    if (!initWindowApp(m_params.fullscreen, m_params.offscreen, m_params.winSizeW, m_params.winSizeH, this))
    {
        throw ExceptionWithStatus(DW_FAILURE, "VisualizationNodeImpl: initWindowApp failed");
    }
#ifdef DW_USE_EGL
    void* egl = nullptr;
    egl       = gWindow->getEGLDisplay();
    CHECK_DW_ERROR(dwContext_setEGLDisplay(egl, m_ctx));
#endif
#ifndef DW_USE_EGL
    // init DW Visualization SDK
    if (glXGetCurrentContext() != nullptr)
#endif
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_ctx));
    }
    initializeRenderer(m_params.winSizeW, m_params.winSizeH);

    initPorts();
    initPasses();
    registerEventCallback();
    if (!m_params.offscreen)
    {
        initialize();
    }

    // Init VAL safety command
    resetSafetyCommand();

    DW_LOGD << "VisualizationNodeImpl: created" << Logger::State::endl;
}

///////////////////////////////////////////////////////////////////////////////////////
VisualizationNodeImpl::~VisualizationNodeImpl()
{
    if (gWindow)
    {
        gWindow->makeCurrent();
    }

    releaseRenderBuffer();

    if (m_cudaRgbaImageTransform)
        FRWK_CHECK_DW_ERROR_NOTHROW(dwImageTransformation_release(m_cudaRgbaImageTransform));

    if (m_renderEngine)
        FRWK_CHECK_DW_ERROR_NOTHROW(dwRenderEngine_release(m_renderEngine));

    if (m_renderer)
        FRWK_CHECK_DW_ERROR_NOTHROW(dwRenderer_release(m_renderer));

    for (size_t camIdx = 0; camIdx < MAX_CAMERA_COUNT; ++camIdx)
    {
        if (m_image2GL[camIdx])
        {
            FRWK_CHECK_DW_ERROR_NOTHROW(dwImageStreamerGL_release(m_image2GL[camIdx]));
        }
        if (m_cudaRgbaRenderImage[camIdx])
        {
            FRWK_CHECK_DW_ERROR_NOTHROW(dwImage_destroy(m_cudaRgbaRenderImage[camIdx]));
        }
    }

    FRWK_CHECK_DW_ERROR_NOTHROW(dwPointCloud_destroyBuffer(&m_cpuPointCloud));
    if (gWindow)
    {
        gWindow->resetCurrent();
        gWindow->releaseContext();
    }
    releaseWindowApp();
    if (m_viz)
    {
        dwVisualizationRelease(m_viz);
        m_viz = DW_NULL_HANDLE;
    }

    DW_LOGD << "VisualizationNodeImpl: destructed" << Logger::State::endl;
}

dwStatus VisualizationNodeImpl::reset()
{
    resetSafetyCommand();
    return Base::reset();
}

dwStatus VisualizationNodeImpl::teardownImpl()
{
    for (size_t camIdx = 0; camIdx < MAX_CAMERA_COUNT; ++camIdx)
    {
        m_currentImageGL[camIdx] = nullptr;
    }

    // only send once
    static bool exitSent = false;
    if (!gRun && !exitSent)
    {
        exitSent = true;
        // send exit signal to all processes
        DW_LOGD << "ESC pressed. Send exit signal to launcher" << Logger::State::endl;
        if (kill(getppid(), SIGTERM) == -1)
        {
            DW_LOGE << "Failed sent exit signal to launcher" << Logger::State::endl;
            exitSent = false;
        }
    }

    return Base::teardownImpl();
}

///////////////////////////////////////////////////////////////////////////////////////
dwStatus VisualizationNodeImpl::getRadarData()
{
    auto& inPortRadarProcessedData = NODE_GET_INPUT_PORT("RADAR_PROCESSED_DATA"_sv);
    if (inPortRadarProcessedData.isBufferAvailable())
    {
        m_radarProcessedData = *inPortRadarProcessedData.getBuffer();
        DW_LOGD << "Epoch: " << m_epochCount << ", Radar processed data received total count: " << m_radarProcessedData.numReturns << Logger::State::endl;
    }

    return DW_SUCCESS;
}

void VisualizationNodeImpl::preProcess3DView(const bool updateView, const bool updateProjection, const bool hasGrid)
{
    dwVector2f bounds{};
    bounds.x = gWindow->width();
    bounds.y = gWindow->height();

    float32_t winAspect = static_cast<float32_t>(gWindow->width()) / static_cast<float32_t>(gWindow->height());

    FRWK_CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(bounds, m_renderEngine));

    if (updateView)
    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_setModelView(m_trackBall.GetModelView(), m_renderEngine));
    }

    if (updateProjection)
    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_setProjection(m_trackBall.GetProjection(winAspect), m_renderEngine));
    }

    FRWK_CHECK_DW_ERROR(dwRenderEngine_setBackgroundColor({0.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine));

    if (hasGrid)
    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_gridBuffer, m_gridBufferPrimitiveCount, m_renderEngine));
    }
}

dwStatus VisualizationNodeImpl::preProcessImage()
{
    uint8_t startIdx = 0;
    uint8_t endIdx   = 0;
    if (m_currentSensorScreen == static_cast<uint8_t>(SensorScreenSelect::SCREEN_SINGLE_CAMERA))
    {
        startIdx = m_params.masterCameraIndex;
        endIdx   = startIdx + 1;
    }
    else if (m_currentSensorScreen == static_cast<uint8_t>(SensorScreenSelect::SCREEN_MULTI_CAMERA))
    {
        startIdx = 0;
        endIdx   = MAX_CAMERA_COUNT;
    }

    for (uint8_t camIdx = startIdx; camIdx < endIdx; ++camIdx)
    {
        if (m_params.camEnable[camIdx])
        {
            if (NODE_GET_INPUT_ARRAY_PORT("IMAGE"_sv, camIdx).isBufferAvailable())
            {
                // Copy convert RGBFP16 image to RGBA image
                dwImage_copyConvertAsync(m_cudaRgbaRenderImage[camIdx], *NODE_GET_INPUT_ARRAY_PORT("IMAGE"_sv, camIdx).getBuffer(), m_params.stream, m_ctx);
                dwStatus status = dwImageStreamerGL_producerSend(m_cudaRgbaRenderImage[camIdx], m_image2GL[camIdx]);
                if (status != DW_SUCCESS)
                {
                    if (status != DW_BUSY_WAITING)
                    {
                        DW_LOGE << "dwImageStreamerGL_producerSend failed " << status << Logger::State::endl;
                        return status;
                    }
                    else
                    {
                        DW_LOGW << "dwImageStreamerGL_producerSend busy" << Logger::State::endl;
                    }
                }
            }
        }
    }
#ifdef PERCEPTION_ENABLED
    auto& inPortBoxNum = NODE_GET_INPUT_PORT("BOX_NUM"_sv);
    if (inPortBoxNum.isBufferAvailable())
    {
        m_boxNum = *inPortBoxNum.getBuffer();
    }
    m_trackedBoxListFloat.clear();
    m_label.clear();
    auto& inPortBoxArr = NODE_GET_INPUT_PORT("BOX_ARR"_sv);
    if (inPortBoxArr.isBufferAvailable())
    {
        YoloScoreRectArray arr = *inPortBoxArr.getBuffer();
        for (uint32_t i = 0; i < arr.count; i++)
        {
            YoloScoreRect box = arr.elements[i];
            dwRectf bboxFloat = box.rectf;
            m_trackedBoxListFloat.push_back(bboxFloat);
            m_label.push_back(YOLO_CLASS_NAMES[box.classIndex]);
        }
    }
#endif
    return DW_SUCCESS;
}

void VisualizationNodeImpl::renderRadar3DPointCloud()
{
    getRadarData();

    m_pstLayout->setCurrentTile();
    m_pstLayout->resetCurrentTileBackground();
    preProcess3DView(true, true, true);

    dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_32, m_renderEngine);
    dwRenderEngine_setColor(DW_RENDERER_COLOR_WHITE, m_renderEngine);
    dwRenderEngine_renderText2D("Radar 3D Point Cloud", {80.f, 800.f}, m_renderEngine);

    FRWK_CHECK_DW_ERROR(dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_XY, 130.0f, m_renderEngine));

    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_setPointSize(3.0f, m_renderEngine));
        FRWK_CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_pointCloudBuffer,
                                                     DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                                     m_radarProcessedData.data,
                                                     sizeof(float32_t) * 4,
                                                     0,
                                                     m_radarProcessedData.numReturns,
                                                     m_renderEngine));

        FRWK_CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_pointCloudBuffer, m_radarProcessedData.numReturns, m_renderEngine));
    }
}

void VisualizationNodeImpl::renderLidar3DPointCloud()
{
    m_pstLayout->setCurrentTile();
    m_pstLayout->resetCurrentTileBackground();
    preProcess3DView(true, true, true);

    dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_32, m_renderEngine);
    dwRenderEngine_setColor(DW_RENDERER_COLOR_WHITE, m_renderEngine);
    dwRenderEngine_renderText2D("Lidar 3D Point Cloud", {80.f, 800.f}, m_renderEngine);

    FRWK_CHECK_DW_ERROR(dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_XY, 130.0f, m_renderEngine));

    for (size_t idx = 0; idx < MAX_LIDAR_COUNT; ++idx)
    {
        if (NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).isBufferAvailable())
        {
            DW_LOGD << "Epoch: " << m_epochCount << ", Lidar point cloud received: " << NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).getBuffer()->size << Logger::State::endl;
            void* renderPtr = NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).getBuffer()->points;
            if (NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).getBuffer()->type == DW_MEMORY_TYPE_CUDA)
            {
                size_t objectSize = sizeof(dwLidarPointXYZI);
                FRWK_CHECK_CUDA_ERROR(cudaMemcpy(m_cpuPointCloud.points, NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).getBuffer()->points, NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).getBuffer()->size * objectSize, cudaMemcpyDeviceToHost));
                renderPtr = m_cpuPointCloud.points;
            }

            FRWK_CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_pointCloudBuffer,
                                                         DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                                         renderPtr,
                                                         sizeof(dwLidarPointXYZI),
                                                         0,
                                                         NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).getBuffer()->size,
                                                         m_renderEngine));

            FRWK_CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_pointCloudBuffer, NODE_GET_INPUT_ARRAY_PORT("LIDAR_POINT_CLOUD"_sv, idx).getBuffer()->size, m_renderEngine));
            m_cpuPointCloud.size = 0;
            break;
        }

        if (NODE_GET_INPUT_ARRAY_PORT("LIDAR_PACKET_ARRAYS"_sv, idx).isBufferAvailable())
        {
            DW_LOGD << "Epoch: " << m_epochCount << ", Lidar packet array received: " << NODE_GET_INPUT_ARRAY_PORT("LIDAR_PACKET_ARRAYS"_sv, idx).getBuffer()->packetSize << Logger::State::endl;
            auto buffers = NODE_GET_INPUT_ARRAY_PORT("LIDAR_PACKET_ARRAYS"_sv, idx).getBufferIter();

            for (auto lidarPackets : buffers)
            {
                for (size_t packetIndex = 0; packetIndex < lidarPackets->packetSize; ++packetIndex)
                {
                    auto lidarPacket = &lidarPackets->packets[packetIndex];
                    // Append the packet to the buffer
                    float32_t* map = &(reinterpret_cast<float32_t*>(m_cpuPointCloud.points)[m_cpuPointCloud.size * sizeof(dwLidarPointXYZI)]);
                    std::memcpy(map, lidarPacket->pointsXYZI, lidarPacket->nPoints * sizeof(dwLidarPointXYZI));
                    m_cpuPointCloud.size += lidarPacket->nPoints;
                }
            }

            FRWK_CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_pointCloudBuffer,
                                                         DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                                         m_cpuPointCloud.points,
                                                         sizeof(dwLidarPointXYZI),
                                                         0,
                                                         m_cpuPointCloud.size,
                                                         m_renderEngine));

            FRWK_CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_pointCloudBuffer, m_cpuPointCloud.size, m_renderEngine));
            m_cpuPointCloud.size = 0;
            break;
        }
    }
}

//#######################################################################################
// Initialize Renderer
//#######################################################################################
void VisualizationNodeImpl::initializeRenderer(uint32_t width, uint32_t height)
{
    CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_ctx));
    // Set some renderer defaults
    dwRenderEngineParams reParams{};
    CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&reParams, width, height));
    reParams.maxBufferCount = 20;
    m_renderEngine          = DW_NULL_HANDLE;
    CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &reParams, m_viz));
}

dwStatus VisualizationNodeImpl::renderCameraViews(RenderType render)
{
    if (m_params.offscreen)
    {
        return DW_SUCCESS;
    }

    gWindow->makeCurrent();
    switch (m_currentSensorScreen)
    {
    case static_cast<uint8_t>(SensorScreenSelect::SCREEN_SINGLE_CAMERA):
    {
        if (m_params.camEnable[m_params.masterCameraIndex])
        {
            m_pstLayout->setCurrentTile();
            setRenderRect(0, 0, gWindow->width(), gWindow->height());
            renderViewDetections(render, m_params.masterCameraIndex);
        }
    }
    break;
    case static_cast<uint8_t>(SensorScreenSelect::SCREEN_MULTI_CAMERA):
    {
        for (size_t camIdx = 0; camIdx < MAX_CAMERA_COUNT; ++camIdx)
        {
            if (m_params.camEnable[camIdx])
            {
                m_pstLayout->setCurrentSubTile(camIdx);
                dwRenderEngineTileState state{};
                dwRenderEngine_getState(&state, m_renderEngine);
                auto& viewport = state.layout.viewport;
                setRenderRect(viewport.x, viewport.y, viewport.width, viewport.height);
                renderViewDetections(render, camIdx);
            }
        }
    }
    break;
    case static_cast<uint8_t>(SensorScreenSelect::SCREEN_RADAR_POINT_CLOUD):
        if (render == RenderType::RENDER_DEBUG)
        {
            renderRadar3DPointCloud();
        }
        break;
    case static_cast<uint8_t>(SensorScreenSelect::SCREEN_LIDAR_POINT_CLOUD):
        if (render == RenderType::RENDER_DEBUG)
        {
            renderLidar3DPointCloud();
        }
        break;
    case static_cast<uint8_t>(SensorScreenSelect::SCREEN_VAL_CONFIG):
        if (render == RenderType::RENDER_DEBUG)
        {
            renderVALConfig();
        }
        break;
    default:
    {
        DW_LOGE << "VisualizationNodeImpl: renderCameraViews: screenIndex is not find: " << m_currentSensorScreen << Logger::State::endl;
        return DW_OUT_OF_BOUNDS;
    }
    }

    gWindow->resetCurrent();
    return DW_SUCCESS;
}

void VisualizationNodeImpl::setRenderRect(uint32_t x, uint32_t y, uint32_t width, uint32_t height)
{
    // set screen display resolution
    m_screenRect.x      = x;
    m_screenRect.y      = y;
    m_screenRect.width  = width;
    m_screenRect.height = height;

    dwRenderer_setRect(m_screenRect, m_renderer);
}

void VisualizationNodeImpl::renderInfoBar(bool inMultiview)
{
    renderMainInfo(inMultiview);
};

VisualizationNodeImpl::IMUString VisualizationNodeImpl::getIMUdata(dwIMUFrame& frame)
{
    DW_LOGD << "Epoch: " << m_epochCount << ", IMU flag: " << frame.flags << Logger::State::endl;
    IMUString result;
    bool containIMUInfo = false;
    {
        result += "[";
        result += frame.timestamp_us;
        result += "] ";

        // orientation
        if (frame.flags & (DW_IMU_ROLL | DW_IMU_PITCH | DW_IMU_YAW))
        {
            result += "Orientation[";
            upadteIMUString(result, "R", frame.orientation[0]);
            upadteIMUString(result, "P", frame.orientation[1]);
            upadteIMUString(result, "Y", frame.orientation[2]);
            result += "] ";
            containIMUInfo = true;
        }

        // orientationQuaternion
        if (frame.flags & (DW_IMU_QUATERNION_X | DW_IMU_QUATERNION_Y | DW_IMU_QUATERNION_Z | DW_IMU_QUATERNION_W))
        {
            result += "OrientationQuaternion[";
            upadteIMUString(result, "X", frame.orientationQuaternion.x);
            upadteIMUString(result, "Y", frame.orientationQuaternion.y);
            upadteIMUString(result, "Z", frame.orientationQuaternion.z);
            upadteIMUString(result, "W", frame.orientationQuaternion.w);
            result += "] ";
            containIMUInfo = true;
        }

        // gyroscope
        if (frame.flags & (DW_IMU_ROLL_RATE | DW_IMU_PITCH_RATE | DW_IMU_YAW_RATE))
        {
            result += "Gyro[";
            upadteIMUString(result, "X", frame.turnrate[0]);
            upadteIMUString(result, "Y", frame.turnrate[1]);
            upadteIMUString(result, "Z", frame.turnrate[2]);
            result += "] ";
            containIMUInfo = true;
        }

        // heading (i.e. compass)
        if (frame.flags & DW_IMU_HEADING)
        {
            result += "Heading[";

            if (frame.headingType == DW_IMU_HEADING_TRUE)
            {
                result += "True:";
            }
            else if (frame.headingType == DW_IMU_HEADING_MAGNETIC)
            {
                result += "Magnetic:";
            }
            else
            {
                result += "Unknown:";
            }

            result += frame.heading;
            result += "] ";
            containIMUInfo = true;
        }

        // acceleration
        if (frame.flags & (DW_IMU_ACCELERATION_X | DW_IMU_ACCELERATION_Y | DW_IMU_ACCELERATION_Z))
        {
            result += "Acceleration[";
            upadteIMUString(result, "X", frame.acceleration[0]);
            upadteIMUString(result, "Y", frame.acceleration[1]);
            upadteIMUString(result, "Z", frame.acceleration[2]);
            result += "] ";
            containIMUInfo = true;
        }

        // magnetometer
        if (frame.flags & (DW_IMU_MAGNETOMETER_X | DW_IMU_MAGNETOMETER_Y | DW_IMU_MAGNETOMETER_Z))
        {
            result += "Magnetometer[";
            upadteIMUString(result, "X", frame.magnetometer[0]);
            upadteIMUString(result, "Y", frame.magnetometer[1]);
            upadteIMUString(result, "Z", frame.magnetometer[2]);
            result += "] ";
            containIMUInfo = true;
        }
        result += "INS status ";
        result += frame.alignmentStatus;
        result += "\n";

        if (containIMUInfo == false)
            result += "No IMU related info";

        result += "\n";
    }

    return result;
}

void VisualizationNodeImpl::renderMainInfo(bool inMultiview)
{
    dwRenderEngineFont textSize = DW_RENDER_ENGINE_FONT_VERDANA_32;
    dwVector2f pos{0, 80.0f};

    if (inMultiview)
    {
        textSize = DW_RENDER_ENGINE_FONT_VERDANA_20;
        pos.y    = 184.0f;
    }

    IMUString imuString("IMU: ");
    auto& inPortImuFrame = NODE_GET_INPUT_PORT("IMU_FRAME"_sv);
    if (inPortImuFrame.isBufferAvailable())
    {
        imuString += getIMUdata(*inPortImuFrame.getBuffer()).c_str();
        DW_LOGD << "Epoch: " << m_epochCount << ", received IMU data: " << imuString.c_str() << Logger::State::endl;
    }

    dwRenderEngine_setFont(textSize, m_renderEngine);
    dwRenderEngine_setColor(DW_RENDERER_COLOR_WHITE, m_renderEngine);
    dwRenderEngine_renderText2D(imuString.c_str(), pos, m_renderEngine);

#ifdef PERCEPTION_ENABLED
    CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_RED, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setLineWidth(2.0f, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D,
                                         m_trackedBoxListFloat.data(), sizeof(dwRectf), 0,
                                         m_trackedBoxListFloat.size(), m_renderEngine));
    for (uint32_t boxIdx = 0U; boxIdx < m_trackedBoxListFloat.size(); ++boxIdx)
    {
        const dwRectf& trackedBox = m_trackedBoxListFloat[boxIdx];
        // Render box id
        dwVector2f pos{static_cast<float32_t>(trackedBox.x),
                       static_cast<float32_t>(trackedBox.y)};
        dwRenderEngine_renderText2D(m_label[boxIdx].c_str(), pos, m_renderEngine);
    }
#endif
}

void VisualizationNodeImpl::renderVALConfig()
{
    DW_LOGD << "Epoch: " << m_epochCount << ", renderVALConfig" << Logger::State::endl;

    m_pstLayout->setCurrentTile();
    m_pstLayout->resetCurrentTileBackground();

    dwVector2f bounds{};
    bounds.x = gWindow->width();
    bounds.y = gWindow->height();

    FRWK_CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(bounds, m_renderEngine));
    FRWK_CHECK_DW_ERROR(dwRenderEngine_setBackgroundColor(DW_RENDERER_COLOR_NVIDIA_GREEN, m_renderEngine));

    // Config Title
    dwRenderEngineFont textSize = DW_RENDER_ENGINE_FONT_VERDANA_64;
    dwVector2f pos{850.0f, 100.0f};
    VALString accConfigTitleString("Safety Command Config");
    dwRenderEngine_setFont(textSize, m_renderEngine);
    dwRenderEngine_setColor(DW_RENDERER_COLOR_BLACK, m_renderEngine);
    dwRenderEngine_renderText2D(accConfigTitleString.c_str(), pos, m_renderEngine);

    // Configs
    textSize = DW_RENDER_ENGINE_FONT_VERDANA_48;
    dwRenderEngine_setFont(textSize, m_renderEngine);
    VALString longCtrlAccelRequestString("Acceleration (Press Up/Down to change): ");
    longCtrlAccelRequestString += m_safeCmd.longCtrlAccelRequest;
    pos.x = 100.0f;
    pos.y = 200.0f;
    dwRenderEngine_renderText2D(longCtrlAccelRequestString.c_str(), pos, m_renderEngine);

    VALString latCtrlSteeringWheelAngleRequestString("Steering Wheel Angle(Press Left/Right to change): ");
    latCtrlSteeringWheelAngleRequestString += m_safeCmd.latCtrlSteeringWheelAngleRequest;
    pos.y += 100.0f;
    dwRenderEngine_renderText2D(latCtrlSteeringWheelAngleRequestString.c_str(), pos, m_renderEngine);

    VALString longCtrlDrivePositionCommandString("Gear State(Press D to change): ");
    switch (m_safeCmd.longCtrlDrivePositionCommand)
    {
    case DW_VIO_LONG_CTRL_DRIVE_POSITION_COMMAND_IDLE:
        longCtrlDrivePositionCommandString += "IDLE";
        break;
    case DW_VIO_LONG_CTRL_DRIVE_POSITION_COMMAND_D:
        longCtrlDrivePositionCommandString += "D";
        break;
    case DW_VIO_LONG_CTRL_DRIVE_POSITION_COMMAND_R:
        longCtrlDrivePositionCommandString += "R";
        break;
    case DW_VIO_LONG_CTRL_DRIVE_POSITION_COMMAND_P:
        longCtrlDrivePositionCommandString += "P";
        break;
    default:
        DW_LOGE << "Epoch: " << m_epochCount << ", renderVALConfig: wrong longCtrlDrivePositionCommand value" << Logger::State::endl;
        break;
    }
    pos.y += 100.0f;
    dwRenderEngine_renderText2D(longCtrlDrivePositionCommandString.c_str(), pos, m_renderEngine);

    auto& inPortNonSafetyState = NODE_GET_INPUT_PORT("VIO_NON_SAFETY_STATE"_sv);
    if (inPortNonSafetyState.isBufferAvailable())
    {
        DW_LOGD << "Epoch: " << m_epochCount << ", received non safety state: " << Logger::State::endl;
        dwVehicleIONonSafetyState nonSafetyState = *inPortNonSafetyState.getBuffer();

        // State Title
        textSize = DW_RENDER_ENGINE_FONT_VERDANA_64;
        dwRenderEngine_setFont(textSize, m_renderEngine);
        pos.x = 850.0f;
        pos.y += 150.0f;
        dwRenderEngine_renderText2D("Vehicle State", pos, m_renderEngine);
        textSize = DW_RENDER_ENGINE_FONT_VERDANA_48;
        dwRenderEngine_setFont(textSize, m_renderEngine);
        pos.x = 100.0f;

        // States
        VALString speedString("Speed: ");
        speedString += nonSafetyState.speedESC;
        pos.y += 100.0f;
        dwRenderEngine_renderText2D(speedString.c_str(), pos, m_renderEngine);

        VALString wheelSpeedString("Wheel Speed: ");
        wheelSpeedString += nonSafetyState.wheelSpeed[0];
        wheelSpeedString += ", ";
        wheelSpeedString += nonSafetyState.wheelSpeed[1];
        wheelSpeedString += ", ";
        wheelSpeedString += nonSafetyState.wheelSpeed[2];
        wheelSpeedString += ", ";
        wheelSpeedString += nonSafetyState.wheelSpeed[3];
        pos.y += 100.0f;
        dwRenderEngine_renderText2D(wheelSpeedString.c_str(), pos, m_renderEngine);

        VALString angleString("Angle: ");
        angleString += nonSafetyState.frontSteeringAngle;
        pos.y += 100.0f;
        dwRenderEngine_renderText2D(angleString.c_str(), pos, m_renderEngine);

        VALString gearString("Gear: ");
        gearString += nonSafetyState.gearStatus;
        pos.y += 100.0f;
        dwRenderEngine_renderText2D(gearString.c_str(), pos, m_renderEngine);
    }
    else
    {
        textSize = DW_RENDER_ENGINE_FONT_VERDANA_64;
        dwRenderEngine_setFont(textSize, m_renderEngine);
        pos.x = 850.0f;
        pos.y += 150.0f;
        dwRenderEngine_renderText2D("No Vehicle State!", pos, m_renderEngine);
    }
}

// Render one supported feature on all camera frames
void VisualizationNodeImpl::renderViewDetections(RenderType render, size_t camIdx)
{
    switch (render)
    {
    case RenderType::RENDER_CAMERA_FRAME:
    {
        m_pstLayout->resetCurrentTileBackground();
        if (NODE_GET_INPUT_ARRAY_PORT("IMAGE"_sv, camIdx).isBufferAvailable())
        {
            DW_LOGD << "Epoch: " << m_epochCount << ", camera " << camIdx << " received image" << Logger::State::endl;
            FRWK_CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&m_frameGL[camIdx], 33000, m_image2GL[camIdx]));
            FRWK_CHECK_DW_ERROR(dwImage_getGL(&m_currentImageGL[camIdx], m_frameGL[camIdx]));
            if (m_currentImageGL[camIdx])
            {
                dwVector2f range{};
                range.x = m_currentImageGL[camIdx]->prop.width;
                range.y = m_currentImageGL[camIdx]->prop.height;
                FRWK_CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
                FRWK_CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_currentImageGL[camIdx], {0.0f, 0.0f, range.x, range.y}, m_renderEngine));
            }
            FRWK_CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&m_frameGL[camIdx], m_image2GL[camIdx]));
            FRWK_CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 33000, m_image2GL[camIdx]));
            m_frameGL[camIdx] = nullptr;
        }
    }
    break;
    case RenderType::RENDER_TRACKED_OBJECT:
        break;
    case RenderType::RENDER_DEBUG:
        break;
    case RenderType::RENDER_INFO_BAR:
        if (m_currentSensorScreen == static_cast<uint8_t>(SensorScreenSelect::SCREEN_SINGLE_CAMERA))
        {
            renderInfoBar(false);
        }
        break;
    default:
        throw ExceptionWithStatus(DW_OUT_OF_BOUNDS, "VisualizationNodeImpl::renderViewDetections unkown detection type");
        break;
    }
}

void VisualizationNodeImpl::resetSafetyCommand()
{
    m_currentSpeed                             = 0.0f;
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

void VisualizationNodeImpl::updateVIOLongCtrlAccelRequest(float32_t diff)
{
    m_safeCmd.longCtrlAccelRequest += diff;
    setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlAccelRequest);

    DW_LOGD << "Updated longCtrlAccelRequest" << Logger::State::endl;
    auto& outPortSafetyCmd = NODE_GET_OUTPUT_PORT("VIO_SAFETY_CMD"_sv);
    if (outPortSafetyCmd.isBufferAvailable())
    {
        *outPortSafetyCmd.getBuffer() = m_safeCmd;
        outPortSafetyCmd.send();
        DW_LOGD << "Safety Command sent, longCtrlAccelRequest: " << m_safeCmd.longCtrlAccelRequest << Logger::State::endl;
    }

    setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlAccelRequest);
}

void VisualizationNodeImpl::updateVIOLatCtrlSteeringWheelAngleRequest(float32_t diff)
{
    m_safeCmd.latCtrlSteeringWheelAngleRequest += diff;
    setSignalValidIf(true, m_safeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);

    DW_LOGD << "Updated latCtrlSteeringWheelAngleRequest" << Logger::State::endl;
    auto& outPortSafetyCmd = NODE_GET_OUTPUT_PORT("VIO_SAFETY_CMD"_sv);
    if (outPortSafetyCmd.isBufferAvailable())
    {
        *outPortSafetyCmd.getBuffer() = m_safeCmd;
        outPortSafetyCmd.send();
        DW_LOGD << "Safety Command sent, latCtrlSteeringWheelAngleRequest: " << m_safeCmd.latCtrlSteeringWheelAngleRequest << Logger::State::endl;
    }

    setSignalValidIf(false, m_safeCmd.validityInfo.latCtrlSteeringWheelAngleRequest);
}

void VisualizationNodeImpl::updateVIOLongCtrlDrivePositionCommand()
{
    m_safeCmd.longCtrlDrivePositionCommand = static_cast<dwVioLongCtrlDrivePositionCommand>((m_safeCmd.longCtrlDrivePositionCommand + 1) % 4);
    setSignalValidIf(true, m_safeCmd.validityInfo.longCtrlDrivePositionCommand);

    DW_LOGD << "Updated longCtrlDrivePositionCommand" << Logger::State::endl;
    auto& outPortSafetyCmd = NODE_GET_OUTPUT_PORT("VIO_SAFETY_CMD"_sv);
    if (outPortSafetyCmd.isBufferAvailable())
    {
        *outPortSafetyCmd.getBuffer() = m_safeCmd;
        outPortSafetyCmd.send();
        DW_LOGD << "Safety Command sent, longCtrlDrivePositionCommand: " << m_safeCmd.longCtrlDrivePositionCommand << Logger::State::endl;
    }

    setSignalValidIf(false, m_safeCmd.validityInfo.longCtrlDrivePositionCommand);
}

void VisualizationNodeImpl::setSignalValidIf(bool valid, dwSignalValidity& validity)
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

} // namespace minipipeline
