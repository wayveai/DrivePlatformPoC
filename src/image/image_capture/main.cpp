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

#define _CRT_SECURE_NO_WARNINGS

#include <memory>
#include <thread>
#include <string>

// IMAGE
#include <dwvisualization/image/FrameCapture.h>

#include <image_common/utils.hpp>

#include <dw/core/base/Version.h>
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>

/**
 * Class that holds functions anda variables common to all stereo samples
 */
using namespace dw_samples::common;

class ImageStreamerCaptureApp : public DriveWorksSample
{
public:
    static const uint32_t WINDOW_HEIGHT = 800;
    static const uint32_t WINDOW_WIDTH  = 1280;

    ImageStreamerCaptureApp(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    bool onInitialize() override final;
    void onProcess() override final;
    void onRender() override final;
    void onRelease() override final;

    std::unique_ptr<SimpleImageStreamerGL<>> m_streamer;

    // CUDA image with RGBA format
    dwImageHandle_t m_rgbaCUDA{};

private:
    void initializeDriveWorks(dwContextHandle_t& context) const;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                   = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer         = DW_NULL_HANDLE;
    dwFrameCaptureHandle_t m_frameCapture = DW_NULL_HANDLE;

    bool m_screeCap;
    uint32_t m_frameCount;
};

//#######################################################################################
/// -----------------------------
/// Initialize Logger and DriveWorks context
/// -----------------------------
void ImageStreamerCaptureApp::initializeDriveWorks(dwContextHandle_t& context) const
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
void ImageStreamerCaptureApp::onRelease()
{
    m_streamer.reset(nullptr);
    if (m_rgbaCUDA)
    {
        dwStatus status = dwImage_destroy(m_rgbaCUDA);
        if (status != DW_SUCCESS)
        {
            logError("Cannot destroy m_rgbaCUDA: %s\n", dwGetStatusName(status));
        }
    }

    dwFrameCapture_release(m_frameCapture);

    if (m_renderEngine != DW_NULL_HANDLE)
    {
        CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
    }

    dwRenderer_release(m_renderer);
    dwSAL_release(m_sal);

    CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
    CHECK_DW_ERROR(dwRelease(m_context));
    CHECK_DW_ERROR(dwLogger_release());
}

//#######################################################################################
bool ImageStreamerCaptureApp::onInitialize()
{
    // -----------------------------------------
    // Initialize DriveWorks context and SAL
    // -----------------------------------------
    {
        initializeDriveWorks(m_context);
        dwSAL_initialize(&m_sal, m_context);
    }

    // -----------------------------
    // Initialize Renderer
    // -----------------------------
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        // init render engine with default params
        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        CHECK_DW_ERROR_MSG(dwRenderer_initialize(&m_renderer, m_viz),
                           "Cannot initialize Renderer, maybe no GL context available?");
        dwRect rect;
        rect.width  = getWindowWidth();
        rect.height = getWindowHeight();
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, m_renderer);
    }

    uint32_t imageWidth  = WINDOW_WIDTH;
    uint32_t imageHeight = WINDOW_HEIGHT;

    // set capture params
    dwFrameCaptureParams frameParams{};

    std::string file_name = std::string(getArgument("capture-file"));
    std::string file_ext  = file_name.substr(file_name.find_last_of(".") + 1);

    std::string params = "type=disk";
    params += ",format=" + file_ext;
    params += ",bitrate=" + getArgument("capture-bitrate");
    params += ",framerate=" + getArgument("capture-framerate");
    params += ",file=" + std::string(getenv("TEST_TMPDIR") ? getenv("TEST_TMPDIR") : "") + file_name;

    frameParams.params.parameters = params.c_str();
    frameParams.width             = imageWidth;
    frameParams.height            = imageHeight;
    frameParams.mode              = DW_FRAMECAPTURE_MODE_SERIALIZE;

    m_screeCap = getArgument("capture-screen") == "1";

    if (m_screeCap)
    {
        frameParams.serializeGL = true;
    }

    CHECK_DW_ERROR(dwFrameCapture_initialize(&m_frameCapture, &frameParams, m_sal, m_context));

    m_frameCount = 0;

    // the image is going to be format DW_IMAGE_RGBA and type DW_TYPE_UINT_8
    // we create a interleaved RGBA image
    dwImageProperties cudaProp{DW_IMAGE_CUDA, imageWidth, imageHeight, DW_IMAGE_FORMAT_RGBA_UINT8};

    // create a synthetic cuda image
    dwStatus status = dwImage_create(&m_rgbaCUDA, cudaProp, m_context);
    if (status != DW_SUCCESS)
    {
        logError("Cannot create m_rgbaCPU: %s\n", dwGetStatusName(status));
        return false;
    }

    // streamer used for display
    m_streamer.reset(new SimpleImageStreamerGL<>(cudaProp, 10000, m_context));

    return true;
}

//#######################################################################################
void ImageStreamerCaptureApp::onProcess()
{
}

//#######################################################################################
void ImageStreamerCaptureApp::onRender()
{
    // cuda kernel for generating a synthetic image
    generateImage(m_rgbaCUDA, m_frameCount);

    dwImageHandle_t frameGL = m_streamer->post(m_rgbaCUDA);
    dwImageGL* glImage;
    dwImage_getGL(&glImage, frameGL);

    dwRenderer_renderTexture(glImage->tex, glImage->target, m_renderer);

    // render
    dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer);
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_64, m_renderer);
    dwRenderer_renderText(m_frameCount % WINDOW_WIDTH, m_frameCount % WINDOW_HEIGHT, "DriveWorks", m_renderer);

    CHECK_DW_ERROR(dwFrameCapture_appendFrame(m_rgbaCUDA, m_frameCapture));

    m_frameCount++;

    renderutils::renderFPS(m_renderEngine, getCurrentFPS());
}

//#######################################################################################
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t{"capture-bitrate", "10000000", "Capture bitrate."},
                           ProgramArguments::Option_t{"capture-framerate", "30", "Capture framerate."},
                           ProgramArguments::Option_t{"capture-file", "capture.h264", "Capture path."},
                           ProgramArguments::Option_t{"capture-screen", "0", "Capture screen or serialize synthetic cuda image."}});

    // -------------------
    // initialize and start a window application
    ImageStreamerCaptureApp app(args);

    app.initializeWindow("Image Capture Sample", ImageStreamerCaptureApp::WINDOW_WIDTH, ImageStreamerCaptureApp::WINDOW_HEIGHT, args.enabled("offscreen"));
    app.setProcessRate(std::stoi(args.get("capture-framerate")));

    return app.run();
}
