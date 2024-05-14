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
// SPDX-FileCopyrightText: Copyright (c) 2019-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <framework/DriveWorksSample.hpp>

// Context, SAL
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>
#include <dw/sensors/Sensors.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>

// Imageprocessing
#include <dw/imageprocessing/ccl/ConnectedComponents.h>

// Framework
#include <framework/SimpleCamera.hpp>
#include <framework/SimpleStreamer.hpp>

using namespace dw_samples::common;

class ConnectedComponentsApp : public DriveWorksSample
{
public:
    // ------------------------------------------------
    // Driveworks Context
    // ------------------------------------------------
    dwContextHandle_t m_context          = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                  = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer        = DW_NULL_HANDLE;

    static const uint32_t WINDOW_HEIGHT = 800;
    static const uint32_t WINDOW_WIDTH  = 1280;

    // Simple camera
    std::unique_ptr<SimpleCamera> m_camera;

    // streamer CUDA to GL
    std::unique_ptr<SimpleImageStreamerGL<dwImageGL>> m_streamerCUDA2GL;

    // output CUDA image containing labels
    dwImageHandle_t m_labelImage = DW_NULL_HANDLE;

    // final GL image shown on the screen
    dwImageGL* m_displayImage = nullptr;

    // connected components
    dwConnectedComponentsHandle_t m_ccl = DW_NULL_HANDLE;

public:
    ConnectedComponentsApp(const ProgramArguments& args);

    void initializeDriveWorks(dwContextHandle_t& context) const;

    // Sample framework
    bool onInitialize() override final;
    void onRender() override final;
    void onResizeWindow(int32_t width, int32_t height) override final;
    void onProcess() override final;
    void onRelease() override final;
};

//#######################################################################################
ConnectedComponentsApp::ConnectedComponentsApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
}

//#######################################################################################
/// -----------------------------
/// Initialize Logger and DriveWorks context
/// -----------------------------
void ConnectedComponentsApp::initializeDriveWorks(dwContextHandle_t& context) const
{
    // initialize logger to print verbose message on console in color
    CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
    CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

    // initialize SDK context, using data folder
    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = getEGLDisplay();
#endif

    CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
}

//#######################################################################################
bool ConnectedComponentsApp::onInitialize()
{
    // Initialize DriveWorks context
    initializeDriveWorks(m_context);
    CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
    CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

    dwStatus status;

    // Initialize camera
    const std::string& videoFile = getArgument("video");
    std::string paramString      = std::string("video=") + videoFile;

    dwSensorParams params{};
    params.protocol   = "camera.virtual";
    params.parameters = paramString.c_str();

    std::string extension;
    size_t found = videoFile.find_last_of(".");

    if (found != std::string::npos)
    {
        extension = videoFile.substr(found + 1);
    }

    m_camera.reset(new SimpleCamera(params, m_sal, m_context));

    dwImageProperties cameraOutputProps = m_camera->getOutputProperties();

    cameraOutputProps.type   = DW_IMAGE_CUDA;
    cameraOutputProps.format = DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR;

    m_camera->setOutputProperties(cameraOutputProps);

    // initialize connected components
    status = dwConnectedComponents_initialize(&m_ccl, &cameraOutputProps, m_context);
    if (status != DW_SUCCESS)
    {
        logError("Cannot initialize connected components: %s\n", dwGetStatusName(status));
        return false;
    }

    // Create output image containing labels
    dwImageProperties imageProps{};
    imageProps.type         = DW_IMAGE_CUDA;
    imageProps.width        = cameraOutputProps.width;
    imageProps.height       = cameraOutputProps.height;
    imageProps.format       = DW_IMAGE_FORMAT_RGBA_UINT8;
    imageProps.memoryLayout = DW_IMAGE_MEMORY_TYPE_PITCH;

    status = dwImage_create(&m_labelImage, imageProps, m_context);
    if (status != DW_SUCCESS)
    {
        logError("Cannot create output label image: %s\n", dwGetStatusName(status));
        return false;
    }

    // initialize image streamer
    m_streamerCUDA2GL.reset(new SimpleImageStreamerGL<dwImageGL>(imageProps, 1000, m_context));

    // Initialize Renderer
    CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));
    dwRenderer_initialize(&m_renderer, m_viz);
    onResizeWindow(getWindowWidth(), getWindowHeight());

    return true;
}

void ConnectedComponentsApp::onResizeWindow(int32_t width, int32_t height)
{
    if (m_renderer == nullptr)
    {
        return;
    }

    dwRect rect;
    rect.width  = width;
    rect.height = height;
    rect.x      = 0;
    rect.y      = 0;

    dwRenderer_setRect(rect, m_renderer);
}

//#######################################################################################
void ConnectedComponentsApp::onProcess()
{
    dwImageHandle_t frame = m_camera->readFrame();

    if (frame == nullptr)
    {
        m_camera->resetCamera();
        m_displayImage = nullptr;
        return;
    }

    dwStatus status = DW_SUCCESS;

    dwImageCUDA* inputImage = nullptr;
    status                  = dwImage_getCUDA(&inputImage, frame);
    if (status != DW_SUCCESS)
    {
        logError("Error acquiring input cuda image from handle: %s\n", dwGetStatusName(status));
        return;
    }

    status = dwConnectedComponents_bindInput(inputImage, m_ccl);
    if (status != DW_SUCCESS)
    {
        logError("Error binding input image: %s\n", dwGetStatusName(status));
    }

    dwImageCUDA* labelImage = nullptr;
    status                  = dwImage_getCUDA(&labelImage, m_labelImage);
    if (status != DW_SUCCESS)
    {
        logError("Error acquiring output cuda image from handle: %s\n", dwGetStatusName(status));
        return;
    }

    status = dwConnectedComponents_bindOutputLabels(labelImage, m_ccl);
    if (status != DW_SUCCESS)
    {
        logError("Error binding output image: %s\n", dwGetStatusName(status));
    }

    status = dwConnectedComponents_process(m_ccl);
    if (status != DW_SUCCESS)
    {
        logError("Error executing CCL: %s\n", dwGetStatusName(status));
        return;
    }

    m_displayImage = m_streamerCUDA2GL->post(m_labelImage);
    if (m_displayImage == nullptr)
    {
        logError("Error streaming label image to GL: %s\n", dwGetStatusName(status));
    }
}

//#######################################################################################
void ConnectedComponentsApp::onRender()
{
    glClearColor(0.0, 0.3, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);

    if (m_displayImage != nullptr)
    {
        dwRenderer_renderTexture(m_displayImage->tex, m_displayImage->target, m_renderer);
    }
}

//#######################################################################################
void ConnectedComponentsApp::onRelease()
{
    dwStatus status = DW_SUCCESS;

    if (m_labelImage != DW_NULL_HANDLE)
    {
        status = dwImage_destroy(m_labelImage);
        if (status != DW_SUCCESS)
        {
            logError("Cannot destroy m_labelImageCUDA: %s\n", dwGetStatusName(status));
        }
    }

    if (m_ccl != DW_NULL_HANDLE)
    {
        status = dwConnectedComponents_release(m_ccl);
        if (status != DW_SUCCESS)
        {
            logError("Cannot destroy m_ccl: %s\n", dwGetStatusName(status));
        }
    }

    if (m_renderer != DW_NULL_HANDLE)
    {
        dwRenderer_release(m_renderer);
    }

    m_camera.reset();
    m_streamerCUDA2GL.reset();

    // -----------------------------------
    // Release SDK
    // -----------------------------------
    CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
    CHECK_DW_ERROR(dwSAL_release(m_sal));
    CHECK_DW_ERROR(dwRelease(m_context));
    CHECK_DW_ERROR(dwLogger_release());
}

//#######################################################################################
int main(int argc, const char** argv)
{
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/recordings/suburb0/video_0_roof_front_120.mp4").c_str(), "path to video"),
                          },
                          "Sample illustrating connected components algorithm");

    // Window/GL based application
    ConnectedComponentsApp app(args);
    app.initializeWindow("Connected Components Sample",
                         ConnectedComponentsApp::WINDOW_WIDTH,
                         ConnectedComponentsApp::WINDOW_HEIGHT,
                         args.enabled("offscreen"));
    return app.run();
}
