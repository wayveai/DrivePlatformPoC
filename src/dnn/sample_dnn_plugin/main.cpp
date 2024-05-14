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

// Samples
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SamplesDataPath.hpp>
#include <framework/WindowGLFW.hpp>
#include <samples/SampleDNNPluginPath.hpp>

// Core
#include <dw/core/context/Context.h>
#include <dw/core/logger/Logger.h>
#include <dw/core/base/Version.h>

// DNN
#include <dw/dnn/DNN.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>

// Misc
#include <sstream>
#include <iomanip>

using namespace dw_samples::common;

class DNNPluginSample : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t m_sdk              = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                  = DW_NULL_HANDLE;

    // ------------------------------------------------
    // DNN
    // ------------------------------------------------
    dwDNNHandle_t m_dnn                         = DW_NULL_HANDLE;
    dwDataConditionerHandle_t m_dataConditioner = DW_NULL_HANDLE;
    float32_t* m_dnnInputDevice;
    float32_t* m_dnnOutputDevice;
    std::unique_ptr<float32_t[]> m_dnnOutputHost;
    std::unique_ptr<uint8_t[]> m_dnnInputHost;

    dwBlobSize m_networkInputDimensions;
    dwBlobSize m_networkOutputDimensions;
    uint32_t m_totalSizeInput;
    uint32_t m_totalSizeOutput;
    dwRect m_detectionRegion;

    dwImageHandle_t m_inputImage;
    dwImageHandle_t m_meanImage;
    dwImageCUDA* m_meanImageCUDA;
    uint32_t m_numImageChannels = 4U;
    int32_t m_detectedDigit     = 0;
    float32_t m_detectionScore  = 0.0f;

    // ------------------------------------------------
    // Renderer
    // ------------------------------------------------
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    std::unique_ptr<SimpleImageStreamerGL<>> m_streamerCUDA2GL;
    cudaStream_t m_cudaStream = 0;
    dwImageGL* m_imgGl;
    bool m_mouseLeft = false;

public:
    /// -----------------------------
    /// Initialize application
    /// -----------------------------
    DNNPluginSample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize modules
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks SDK context and SAL
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
            CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};

#ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
#endif

            CHECK_DW_ERROR(dwInitialize(&m_sdk, DW_VERSION, &sdkParams));
            CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_sdk));
        }

        //------------------------------------------------------------------------------
        // initialize Renderer
        //------------------------------------------------------------------------------
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_sdk));

            // Setup render engine
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            params.defaultTile.lineWidth = 0.2f;
            params.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_20;
            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));
        }

        //------------------------------------------------------------------------------
        // initialize DNN
        //------------------------------------------------------------------------------
        {
            // If not specified, load the correct network based on platform
            std::string tensorRTModel = getArgument("tensorRT_model");
            if (tensorRTModel.empty())
            {
                tensorRTModel = SamplesDataPath::get() + "/samples/dnn/";
                tensorRTModel += getPlatformPrefix();
                tensorRTModel += "/mnist.bin";
            }

            dwDNNPluginConfiguration pluginConf{};
            pluginConf.numCustomLayers = 1;

            std::string pluginPath = SampleDNNPluginPath::get() + "/libdnn_pool_plugin.so";
            dwDNNCustomLayer customLayer{};
            customLayer.pluginLibraryPath = pluginPath.c_str();

            pluginConf.customLayers = &customLayer;

            // Initialize DNN from a TensorRT file
            CHECK_DW_ERROR(dwDNN_initializeTensorRTFromFile(&m_dnn, tensorRTModel.c_str(), &pluginConf,
                                                            DW_PROCESSOR_TYPE_GPU, m_sdk));

            CHECK_DW_ERROR(dwDNN_setCUDAStream(m_cudaStream, m_dnn));

            // Get input and output dimensions
            CHECK_DW_ERROR(dwDNN_getInputSize(&m_networkInputDimensions, 0U, m_dnn));
            CHECK_DW_ERROR(dwDNN_getOutputSize(&m_networkOutputDimensions, 0U, m_dnn));

            auto getTotalSize = [](const dwBlobSize& blobSize) {
                return blobSize.channels * blobSize.height * blobSize.width;
            };

            // Calculate total size needed to store input and output
            m_totalSizeInput  = getTotalSize(m_networkInputDimensions);
            m_totalSizeOutput = getTotalSize(m_networkOutputDimensions);

            // Allocate GPU memory
            CHECK_CUDA_ERROR(cudaMalloc((void**)&m_dnnInputDevice, sizeof(float32_t) * m_totalSizeInput));
            CHECK_CUDA_ERROR(cudaMalloc((void**)&m_dnnOutputDevice,
                                        sizeof(float32_t) * m_totalSizeOutput));
            // Allocate CPU memory for reading the output of DNN
            m_dnnOutputHost.reset(new float32_t[m_totalSizeOutput]);

            // Get mean data
            std::string filename = SamplesDataPath::get() + "/samples/dnn/meanData.txt";
            std::ifstream fin(filename);
            std::unique_ptr<float32_t[]> meanDataHost(new float32_t[m_networkInputDimensions.height *
                                                                    m_networkInputDimensions.width]);
            for (uint32_t idx = 0U; idx < m_networkInputDimensions.height * m_networkInputDimensions.width;
                 idx++)
            {
                fin >> meanDataHost[idx];
            }
            fin.close();

            size_t inputImageSize = m_networkInputDimensions.width * m_networkInputDimensions.height *
                                    m_numImageChannels;
            m_dnnInputHost.reset(new uint8_t[inputImageSize]);
            std::fill(m_dnnInputHost.get(), m_dnnInputHost.get() + inputImageSize, 255);

            dwImageProperties inputImageProps{};
            inputImageProps.format       = DW_IMAGE_FORMAT_RGBA_UINT8;
            inputImageProps.height       = m_networkInputDimensions.height;
            inputImageProps.width        = m_networkInputDimensions.width;
            inputImageProps.memoryLayout = DW_IMAGE_MEMORY_TYPE_PITCH;
            inputImageProps.type         = DW_IMAGE_CUDA;
            dwImage_create(&m_inputImage, inputImageProps, m_sdk);

            dwImageProperties meanImageProps{};
            meanImageProps.format       = DW_IMAGE_FORMAT_R_FLOAT32;
            meanImageProps.height       = m_networkInputDimensions.height;
            meanImageProps.width        = m_networkInputDimensions.width;
            meanImageProps.memoryLayout = DW_IMAGE_MEMORY_TYPE_PITCH;
            meanImageProps.type         = DW_IMAGE_CUDA;
            dwImage_create(&m_meanImage, meanImageProps, m_sdk);
            dwImage_getCUDA(&m_meanImageCUDA, m_meanImage);
            cudaMemcpy2D(m_meanImageCUDA->dptr[0], m_meanImageCUDA->pitch[0], meanDataHost.get(),
                         sizeof(float32_t) * m_networkInputDimensions.width,
                         sizeof(float32_t) * m_networkInputDimensions.width,
                         m_networkInputDimensions.height, cudaMemcpyHostToDevice);

            dwDNNMetaData metadata{};
            CHECK_DW_ERROR(dwDNN_getMetaData(&metadata, m_dnn));

            // Initialie data conditioner
            CHECK_DW_ERROR(dwDataConditioner_initialize(&m_dataConditioner, &m_networkInputDimensions, 1U,
                                                        &metadata.dataConditionerParams, m_cudaStream,
                                                        m_sdk));

            dwImageProperties displayProperties{};
            displayProperties.memoryLayout = DW_IMAGE_MEMORY_TYPE_PITCH;
            displayProperties.type         = DW_IMAGE_CUDA;
            displayProperties.format       = DW_IMAGE_FORMAT_RGBA_UINT8;
            displayProperties.height       = m_networkInputDimensions.height;
            displayProperties.width        = m_networkInputDimensions.width;

            m_streamerCUDA2GL.reset(new SimpleImageStreamerGL<>(displayProperties, 1000, m_sdk));

            // Detection region
            m_detectionRegion.width  = displayProperties.width;
            m_detectionRegion.height = displayProperties.height;
            m_detectionRegion.x      = 0;
            m_detectionRegion.y      = 0;
        }

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - collect sensor frame
    ///     - run detection and tracking
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        // read from camera
        dwImageCUDA* rgbaImage = nullptr;
        getNextFrame(&rgbaImage, &m_imgGl);
        std::this_thread::yield();
        while (rgbaImage == nullptr)
        {
            onReset();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            getNextFrame(&rgbaImage, &m_imgGl);
        }

        // Run data conditioner to prepare input for the network
        CHECK_DW_ERROR(dwDataConditioner_prepareDataRaw(m_dnnInputDevice, &rgbaImage, 1, &m_detectionRegion,
                                                        cudaAddressModeClamp, m_dataConditioner));
        // Run DNN on the output of data conditioner
        CHECK_DW_ERROR(dwDNN_inferRaw(&m_dnnOutputDevice, &m_dnnInputDevice, 1U, m_dnn));

        getOutputFromDNN();
    }

    ///------------------------------------------------------------------------------
    /// Render sample output on screen
    ///     - render video
    ///     - render boxes with labels
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        dwVector2f range{};
        range.x = m_imgGl->prop.width;
        range.y = m_imgGl->prop.height;
        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_imgGl, {0.0f, 0.0f, range.x, range.y}, m_renderEngine));

        std::string detectedLabel = "Detected digit: " + std::to_string(m_detectedDigit);
        std::stringstream detectedProbLabel;
        detectedProbLabel << "Detection score: " << std::setprecision(2) << m_detectionScore;
        CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_24, m_renderEngine));
        if (m_detectionScore < 4.0f)
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_RED, m_renderEngine));
        }
        else if (m_detectionScore < 8.0f)
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_ORANGE, m_renderEngine));
        }
        else
        {
            CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_GREEN, m_renderEngine));
        }

        CHECK_DW_ERROR(dwRenderEngine_renderText2D(detectedLabel.c_str(),
                                                   dwVector2f{0.0f, range.y - 0.8f},
                                                   m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_renderText2D(detectedProbLabel.str().c_str(),
                                                   dwVector2f{0.0f, range.y - 0.0f},
                                                   m_renderEngine));
    }

    ///------------------------------------------------------------------------------
    /// Free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        m_streamerCUDA2GL.reset();

        // Free GPU memory
        if (m_dnnOutputDevice)
        {
            CHECK_CUDA_ERROR(cudaFree(m_dnnOutputDevice));
        }

        if (m_dnnInputDevice)
        {
            CHECK_CUDA_ERROR(cudaFree(m_dnnInputDevice));
        }

        CHECK_DW_ERROR(dwImage_destroy(m_inputImage));
        CHECK_DW_ERROR(dwImage_destroy(m_meanImage));

        // Release detector
        CHECK_DW_ERROR(dwDNN_release(m_dnn));
        // Release data conditioner
        CHECK_DW_ERROR(dwDataConditioner_release(m_dataConditioner));
        // Release render engine
        CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        // Release SDK
        CHECK_DW_ERROR(dwSAL_release(m_sal));
        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_sdk));
    }

    ///------------------------------------------------------------------------------
    /// Reset tracker and detector
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        CHECK_DW_ERROR(dwDNN_reset(m_dnn));
        CHECK_DW_ERROR(dwDataConditioner_reset(m_dataConditioner));
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        {
            CHECK_DW_ERROR(dwRenderEngine_reset(m_renderEngine));
            dwRectf rect;
            rect.width  = width;
            rect.height = height;
            rect.x      = 0;
            rect.y      = 0;
            CHECK_DW_ERROR(dwRenderEngine_setBounds(rect, m_renderEngine));
        }
    }

private:
    //------------------------------------------------------------------------------
    void getNextFrame(dwImageCUDA** nextFrameCUDA, dwImageGL** nextFrameGL)
    {
        dwImage_getCUDA(nextFrameCUDA, m_inputImage);
        dwImageCUDA* nextFrameCUDAref = *nextFrameCUDA;
        cudaMemcpy2D(nextFrameCUDAref->dptr[0], nextFrameCUDAref->pitch[0], m_dnnInputHost.get(),
                     sizeof(uint8_t) * m_networkInputDimensions.width * m_numImageChannels,
                     sizeof(uint8_t) * m_networkInputDimensions.width * m_numImageChannels,
                     m_networkInputDimensions.height, cudaMemcpyHostToDevice);
        dwImageHandle_t frameGL = m_streamerCUDA2GL->post(m_inputImage);
        dwImage_getGL(nextFrameGL, frameGL);
    }

    //------------------------------------------------------------------------------
    void getOutputFromDNN()
    {
        // Copy data to host
        CHECK_CUDA_ERROR(cudaMemcpy(m_dnnOutputHost.get(), m_dnnOutputDevice,
                                    m_totalSizeOutput * sizeof(float32_t),
                                    cudaMemcpyDeviceToHost));

        // Get the digit with maximum response
        float32_t maxProb = 0.0f;
        int32_t digit     = -1;
        for (int32_t dIdx = 0; dIdx < static_cast<int32_t>(m_totalSizeOutput); dIdx++)
        {
            float32_t prob = m_dnnOutputHost[dIdx];
            if (prob > maxProb)
            {
                maxProb = prob;
                digit   = dIdx;
            }
        }

        m_detectedDigit  = digit;
        m_detectionScore = maxProb;
    }

    // ------------------------------------------------
    std::string getPlatformPrefix()
    {
        static const int32_t CUDA_AMPERE_MAJOR_COMPUTE_CAPABILITY           = 8;
        static const int32_t CUDA_TURING_VOLTA_MAJOR_COMPUTE_CAPABILITY     = 7;
        static const int32_t CUDA_VOLTA_DISCRETE_MINOR_COMPUTE_CAPABILITY   = 0;
        static const int32_t CUDA_VOLTA_INTEGRATED_MINOR_COMPUTE_CAPABILITY = 2;
        static const int32_t CUDA_TURING_DISCRETE_MINOR_COMPUTE_CAPABILITY  = 5;

        std::string path;
        int32_t currentGPU;
        dwGPUDeviceProperties gpuProp{};

        CHECK_DW_ERROR(dwContext_getGPUDeviceCurrent(&currentGPU, m_sdk));
        CHECK_DW_ERROR(dwContext_getGPUProperties(&gpuProp, currentGPU, m_sdk));

        if (gpuProp.major == CUDA_AMPERE_MAJOR_COMPUTE_CAPABILITY)
        {
            if (gpuProp.integrated)
            {
                path = "ampere-integrated";
            }
            else
            {
                path = "ampere-discrete";
            }
        }
        else if (gpuProp.major == CUDA_TURING_VOLTA_MAJOR_COMPUTE_CAPABILITY)
        {
            if (gpuProp.minor == CUDA_TURING_DISCRETE_MINOR_COMPUTE_CAPABILITY)
            {
                path = "turing";
            }
            else if (gpuProp.minor == CUDA_VOLTA_INTEGRATED_MINOR_COMPUTE_CAPABILITY)
            {
                path = "volta-integrated";
            }
            else if (gpuProp.minor == CUDA_VOLTA_DISCRETE_MINOR_COMPUTE_CAPABILITY)
            {
                path = "volta-discrete";
            }
        }
        else
        {
            throw std::runtime_error("Current GPU is not supported.");
        }

        return path;
    }

    // ------------------------------------------------
    void drawAt(float32_t x, float32_t y)
    {
        float32_t scaleFactorX = static_cast<float32_t>(m_networkInputDimensions.width) / getWindowWidth();
        float32_t scaleFactorY = static_cast<float32_t>(m_networkInputDimensions.height) / getWindowHeight();
        int32_t scaledX        = static_cast<int32_t>(x * scaleFactorX);
        int32_t scaledY        = static_cast<int32_t>(y * scaleFactorY);
        if (scaledX >= 0 && scaledX < static_cast<int32_t>(m_networkInputDimensions.width) &&
            scaledY >= 0 && scaledY < static_cast<int32_t>(m_networkInputDimensions.height))
        {
            uint32_t planeIdx = scaledY * m_networkInputDimensions.width * m_numImageChannels +
                                scaledX * m_numImageChannels;
            m_dnnInputHost[planeIdx + 0] = 0;
            m_dnnInputHost[planeIdx + 1] = 0;
            m_dnnInputHost[planeIdx + 2] = 0;
        }
    }

    // ------------------------------------------------
    virtual void onMouseDown(int button, float x, float y, int /*mods*/)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            // start drawing if the left button is pressed
            m_mouseLeft = true;
            drawAt(x, y);
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            // Clear the screen
            std::fill(m_dnnInputHost.get(),
                      m_dnnInputHost.get() + m_numImageChannels * m_networkInputDimensions.width * m_networkInputDimensions.height, 255);
        }
    }

    // ------------------------------------------------
    virtual void onMouseUp(int button, float /*x*/, float /*y*/, int /*mods*/)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            // stop drawing if the left button is released
            m_mouseLeft = false;
        }
    }

    // ------------------------------------------------
    virtual void onMouseMove(float x, float y)
    {
        if (m_mouseLeft)
        {
            // Draw if mouse is moving while the left button is pressed.
            drawAt(x, y);
        }
    }
};

int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("tensorRT_model", "", (std::string("path to TensorRT model file. By default: ") + dw_samples::SamplesDataPath::get() + "/samples/dnn/<gpu-architecture>/mnist.bin").c_str())},
                          "DNN plugin sample which recognizes digit using DNN with a plugin.");

    DNNPluginSample app(args);
    app.initializeWindow("DNN Plugin Sample", 800, 800, args.enabled("offscreen"));

    return app.run();
}
