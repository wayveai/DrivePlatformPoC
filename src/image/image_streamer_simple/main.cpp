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

#include <framework/DriveWorksSample.hpp>

#include <memory>
#include <thread>
#include <string>

// Context, SAL
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>
#include <dw/sensors/Sensors.h>

// IMAGE
#include <dw/interop/streamer/ImageStreamer.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>

#include <image_common/utils.hpp>

/**
 * Class that holds functions anda variables common to all stereo samples
 */
using namespace dw_samples::common;

class ImageStreamerSimpleApp : public DriveWorksSample
{
public:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                   = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer         = DW_NULL_HANDLE;

    const uint32_t WINDOW_HEIGHT = 800;
    const uint32_t WINDOW_WIDTH  = 1280;

    // streamer from CPU to CUDA
    dwImageStreamerHandle_t m_streamerCPU2CUDA = DW_NULL_HANDLE;

    // streamer from CUDA to GL, used for display
    dwImageStreamerHandle_t m_streamerCUDA2GL = DW_NULL_HANDLE;

    // CPU image with RGB format
    dwImageHandle_t m_rgbCPU;

    // CUDA image with RGBA format that we are going to copy convert onto
    dwImageHandle_t m_rgbaCUDA;

    uint32_t m_frameCount;

public:
    ImageStreamerSimpleApp(const ProgramArguments& args);

    void initializeDriveWorks(dwContextHandle_t& context) const;

    // Sample framework
    bool onInitialize() override final;
    void onRender() override final;
    void onProcess() override final {}
    void onRelease() override final;
};

//#######################################################################################
ImageStreamerSimpleApp::ImageStreamerSimpleApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
    std::cout << "This sample illustrates how to use an image streamer given a CPU image. This will create an "
                 "empty dwImageCPU, stream it to a dwImageCUDA, apply some simple operations in a kernel and "
                 "then stream it to a dwImageGL for rendering. The purpose is to show how to properly "
                 "create, use and destroy an image streamer."
              << std::endl;
}

//#######################################################################################
/// -----------------------------
/// Initialize Logger and DriveWorks context
/// -----------------------------
void ImageStreamerSimpleApp::initializeDriveWorks(dwContextHandle_t& context) const
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
bool ImageStreamerSimpleApp::onInitialize()
{
    // -----------------------------------------
    // Initialize DriveWorks context and SAL
    // -----------------------------------------
    {
        initializeDriveWorks(m_context);
        dwSAL_initialize(&m_sal, m_context);
        dwVisualizationInitialize(&m_viz, m_context);
    }

    m_frameCount = 0;

    // we create a dwImageCPU on the CPU by allocating memory
    uint32_t imageWidth  = 800;
    uint32_t imageHeight = 600;
    dwStatus status;

    // the image is going to be format interleaved RGB and type UINT_8
    dwImageProperties cpuProp{};
    cpuProp.type         = DW_IMAGE_CPU;
    cpuProp.width        = imageWidth;
    cpuProp.height       = imageHeight;
    cpuProp.format       = DW_IMAGE_FORMAT_RGB_UINT8;
    cpuProp.memoryLayout = DW_IMAGE_MEMORY_TYPE_DEFAULT; // not necessary to specify if init with {}

    status = dwImage_create(&m_rgbCPU, cpuProp, m_context);
    if (status != DW_SUCCESS)
    {
        logError("Cannot create m_rgbaCPU: %s\n", dwGetStatusName(status));
        return false;
    }

    // we now create an ImageStreamer to stream the CPU image to CUDA
    // the streamer is setup to stream an image with properties cpuProp to a dwImageCUDA with the
    // same properties, except for the type, which is going to be DW_IMAGE_CUDA
    // the streamer needs to be released at the end
    status = dwImageStreamer_initialize(&m_streamerCPU2CUDA, &cpuProp, DW_IMAGE_CUDA, m_context);
    if (status != DW_SUCCESS)
    {
        logError("Cannot init image streamer m_streamerCPU2CUDA: %s\n", dwGetStatusName(status));
        return false;
    }

    // create a new imageStreamer to stream CUDA to GL and get a openGL texture to render on screen
    // properties are the same as cpu image except for the type. For demonstration purposes the image is
    // created as RGB, format converted in CUDA to rgba and then streamed to gL
    dwImageProperties cudaProp = cpuProp;
    cudaProp.type              = DW_IMAGE_CUDA;
    cudaProp.format            = DW_IMAGE_FORMAT_RGBA_UINT8;

    status = dwImage_create(&m_rgbaCUDA, cudaProp, m_context);
    if (status != DW_SUCCESS)
    {
        logError("Cannot create m_rgbaCUDA: %s\n", dwGetStatusName(status));
        return false;
    }

    status = dwImageStreamerGL_initialize(&m_streamerCUDA2GL, &cudaProp, DW_IMAGE_GL, m_context);
    if (status != DW_SUCCESS)
    {
        logError("Cannot init gl image streamer m_streamerCUDA2GL: %s\n", dwGetStatusName(status));
        return false;
    }

    // -----------------------------
    // Initialize Renderer
    // -----------------------------
    {
        // init render engine with default params
        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        dwRenderer_initialize(&m_renderer, m_viz);
        dwRect rect;
        rect.width  = getWindowWidth();
        rect.height = getWindowHeight();
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, m_renderer);
    }

    return true;
}

//#######################################################################################
void ImageStreamerSimpleApp::onRender()
{
    dwStatus status;

    // post the cpu image. This will push the the image through the stream for type conversion.
    status = dwImageStreamer_producerSend(m_rgbCPU, m_streamerCPU2CUDA);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot post m_rgbaCPU in m_streamerCPU2CUDA: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // use a pointer to a dwImageCUDA to receive the converted image from the streamer. We receive
    // a pointer as the image is only "borrowed" from the streamer who has the ownership. Since the image is
    // borrowed we need to return it when we are done using it
    dwImageHandle_t rgbCUDA;
    // receive the converted CUDA image from the stream, timeout set to 1000 ms
    status = dwImageStreamer_consumerReceive(&rgbCUDA, 1000, m_streamerCPU2CUDA);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // now the converted image can be as a cuda image, for example we run the cuda kernel that is called
    // by this function (implemented in utils.cu)
    generateImage(rgbCUDA, m_frameCount);

    // format convert the RGB into RGBA
    status = dwImage_copyConvert(m_rgbaCUDA, rgbCUDA, m_context);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot format convert: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // the converted cuda image is now posted on the gl streamer. Until we return the gl image we cannot return
    // the cuda image
    status = dwImageStreamerGL_producerSend(m_rgbaCUDA, m_streamerCUDA2GL);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // pointer to the dwImageGL we get from the streamer, it contains the texture and target we need for
    // rendering
    dwImageHandle_t glImageNew;
    // receive a dwImageGL that we can render
    status = dwImageStreamerGL_consumerReceive(&glImageNew, 1000, m_streamerCUDA2GL);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // render
    {
        dwImageGL* glImage;
        CHECK_DW_ERROR(dwImage_getGL(&glImage, glImageNew));

        glClearColor(0.0, 0.0, 1.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT);
        dwRenderer_renderTexture(glImage->tex, glImage->target, m_renderer);
    }

    // return the received gl since we don't use it anymore
    status = dwImageStreamerGL_consumerReturn(&glImageNew, m_streamerCUDA2GL);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // wait to get back the cuda image we posted in the cuda->gl stream. We will receive a pointer to it and,
    // to be sure we are getting back the same image we posted, we compare the pointer to our dwImageCPU
    status = dwImageStreamerGL_producerReturn(nullptr, 1000, m_streamerCUDA2GL);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        //throw std::runtime_error(err.str().c_str());
    }

    // now that we are done with gl, we can return the dwImageCUDA to the streamer, which is the owner of it
    status = dwImageStreamer_consumerReturn(&rgbCUDA, m_streamerCPU2CUDA);
    if (status != DW_SUCCESS)
    {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // wait to get back the cpu image we posted at the beginning. We will receive a pointer to it and,
    // to be sure we are getting back the same image we posted, we compare the pointer to our dwImageCPU
    status = dwImageStreamer_producerReturn(nullptr, 1000, m_streamerCPU2CUDA);
    if ((status != DW_SUCCESS))
    {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    m_frameCount++;

    renderutils::renderFPS(m_renderEngine, getCurrentFPS());
}

//#######################################################################################
void ImageStreamerSimpleApp::onRelease()
{
    dwStatus status;

    status = dwImage_destroy(m_rgbCPU);
    if (status != DW_SUCCESS)
    {
        logError("Cannot destroy m_rgbCPU: %s\n", dwGetStatusName(status));
    }

    status = dwImage_destroy(m_rgbaCUDA);
    if (status != DW_SUCCESS)
    {
        logError("Cannot destroy m_rgbaCUDA: %s\n", dwGetStatusName(status));
    }

    // release streamers
    status = dwImageStreamerGL_release(m_streamerCUDA2GL);
    if (status != DW_SUCCESS)
    {
        logError("Cannot release m_streamerCUDA2GL: %s\n", dwGetStatusName(status));
    }

    status = dwImageStreamer_release(m_streamerCPU2CUDA);
    if (status != DW_SUCCESS)
    {
        logError("Cannot release m_streamerCPU2CUDA: %s\n", dwGetStatusName(status));
    }

    if (m_renderEngine != DW_NULL_HANDLE)
    {
        CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
    }

    dwRenderer_release(m_renderer);

    // -----------------------------------
    // Release SDK
    // -----------------------------------
    dwSAL_release(m_sal);

    CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
    CHECK_DW_ERROR(dwRelease(m_context));
    CHECK_DW_ERROR(dwLogger_release());
}

//#######################################################################################
int main(int argc, const char** argv)
{
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {},
                          "Sample illustrating how to use an image streamer given a CPU image");

    // Window/GL based application
    ImageStreamerSimpleApp app(args);
    app.initializeWindow("Simple Image Streamer Sample", 1280, 800, args.enabled("offscreen"));
    return app.run();
}
