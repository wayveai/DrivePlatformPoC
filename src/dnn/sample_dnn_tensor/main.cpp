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
// SPDX-FileCopyrightText: Copyright (c) 2020-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <framework/SimpleCamera.hpp>
#include <framework/Checks.hpp>

// Core
#include <dw/core/context/Context.h>
#include <dw/core/logger/Logger.h>
#include <dw/core/base/Version.h>

// HAL
#include <dw/sensors/Sensors.h>

// DNN
#include <dw/dnn/DNN.h>
#include <dw/interop/streamer/TensorStreamer.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>

using namespace dw_samples::common;

class DNNTensorSample : public DriveWorksSample
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
    typedef std::pair<dwRectf, float32_t> BBoxConf;
    static constexpr float32_t COVERAGE_THRESHOLD       = 0.6f;
    static constexpr uint32_t NUM_OUTPUT_TENSORS        = 2U;
    const uint32_t m_maxDetections                      = 1000U;
    const float32_t m_nonMaxSuppressionOverlapThreshold = 0.5;

    dwDNNHandle_t m_dnn                                                  = DW_NULL_HANDLE;
    dwDataConditionerHandle_t m_dataConditioner                          = DW_NULL_HANDLE;
    dwDNNTensorStreamerHandle_t m_dnnOutputStreamers[NUM_OUTPUT_TENSORS] = {DW_NULL_HANDLE};
    dwDNNTensorHandle_t m_dnnInput                                       = DW_NULL_HANDLE;
    dwDNNTensorHandle_t m_dnnOutputsDevice[NUM_OUTPUT_TENSORS]           = {DW_NULL_HANDLE};
    std::vector<dwBox2D> m_detectedBoxList;
    std::vector<dwRectf> m_detectedBoxListFloat;

    uint32_t m_cellSize = 1U;
    uint32_t m_cvgIdx;
    uint32_t m_bboxIdx;

    dwRect m_detectionRegion;

    // ------------------------------------------------
    // CUDLA: 1 input 2 outputs
    // Use sample_weights.caffemodel to generate dla loadable
    // Then tensorRT_optimization to generate dla.dnn
    // ------------------------------------------------
    bool m_usecuDLA        = false;
    uint32_t m_dlaEngineNo = 0;

    // ------------------------------------------------
    // YOLO: 1 input and 1 output
    // Switch to Yolo onnx from now on for GPU processer
    // ------------------------------------------------
    static constexpr float32_t CONFIDENCE_THRESHOLD = 0.45f;
    static constexpr float32_t SCORE_THRESHOLD      = 0.25f;

    const std::string YOLO_CLASS_NAMES[80] = {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
                                              "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
                                              "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
                                              "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
                                              "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
                                              "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
                                              "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
                                              "hair drier", "toothbrush"};
    typedef struct YoloScoreRect
    {
        dwRectf rectf;
        float32_t score;
        uint16_t classIndex;
    } YoloScoreRect;

    std::vector<std::string> m_label;

    // ------------------------------------------------
    // Renderer
    // ------------------------------------------------
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwImageHandle_t m_imageRGBA;
    std::unique_ptr<SimpleImageStreamerGL<>> m_streamerCUDA2GL;
    cudaStream_t m_cudaStream = 0;

    // ------------------------------------------------
    // Camera
    // ------------------------------------------------
    std::unique_ptr<SimpleCamera> m_camera;
    dwImageGL* m_imgGl;
    dwImageProperties m_rcbProperties;

    // image width and height
    uint32_t m_imageWidth;
    uint32_t m_imageHeight;
    bool m_isRaw = false;

public:
    /// -----------------------------
    /// Initialize application
    /// -----------------------------
    DNNTensorSample(const ProgramArguments& args)
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
        // initialize Sensors
        //------------------------------------------------------------------------------
        {
            dwSensorParams params{};
            std::string parameterString;
            {
#ifdef VIBRANTE
                if (getArgument("input-type").compare("camera") == 0)
                {
                    parameterString = "camera-type=" + getArgument("camera-type");
                    parameterString += ",camera-group=" + getArgument("camera-group");
                    parameterString += ",serialize=false,camera-count=4";
                    std::string cameraMask[4] = {"0001", "0010", "0100", "1000"};
                    uint32_t cameraIdx        = std::stoi(getArgument("camera-index"));
                    if (cameraIdx < 0 || cameraIdx > 3)
                    {
                        std::cerr << "Error: camera index must be 0, 1, 2 or 3" << std::endl;
                        return false;
                    }
                    parameterString += ",camera-mask=" + cameraMask[cameraIdx];

                    params.parameters = parameterString.c_str();
                    params.protocol   = "camera.gmsl";
                }
                else
#endif
                {
                    parameterString   = getArgs().parameterString();
                    params.parameters = parameterString.c_str();
                    params.protocol   = "camera.virtual";

                    std::string videoFormat = getArgument("video");
                }

                {
                    m_camera.reset(new SimpleCamera(params, m_sal, m_sdk));
                    dwImageProperties outputProperties = m_camera->getOutputProperties();
                    outputProperties.type              = DW_IMAGE_CUDA;
                    m_camera->setOutputProperties(outputProperties);
                }
            }

            if (m_camera == nullptr)
            {
                logError("Camera could not be created\n");
                return false;
            }

            std::cout << "Camera image with " << m_camera->getCameraProperties().resolution.x << "x"
                      << m_camera->getCameraProperties().resolution.y << " at "
                      << m_camera->getCameraProperties().framerate << " FPS" << std::endl;

            dwImageProperties displayProperties = m_camera->getOutputProperties();
            displayProperties.format            = DW_IMAGE_FORMAT_RGBA_UINT8;

            CHECK_DW_ERROR(dwImage_create(&m_imageRGBA, displayProperties, m_sdk));

            m_streamerCUDA2GL.reset(new SimpleImageStreamerGL<>(displayProperties, 1000, m_sdk));

            m_rcbProperties = m_camera->getOutputProperties();

            m_imageWidth  = displayProperties.width;
            m_imageHeight = displayProperties.height;
        }

        //------------------------------------------------------------------------------
        // initialize DNN
        //------------------------------------------------------------------------------
        {
#ifdef VIBRANTE
            m_usecuDLA    = getArgument("cudla").compare("1") == 0;
            m_dlaEngineNo = std::atoi(getArgument("dla-engine").c_str());
#endif
            // If not specified, load the correct network based on platform
            std::string tensorRTModel = getArgument("tensorRT_model");
            if (tensorRTModel.empty())
            {
                tensorRTModel = dw_samples::SamplesDataPath::get() + "/samples/detector/";
                tensorRTModel += getPlatformPrefix();
                tensorRTModel += "/tensorRT_model";
                if (m_usecuDLA)
                    tensorRTModel += ".dla";
                tensorRTModel += ".bin";
            }

            // Initialize DNN from a TensorRT file
            CHECK_DW_ERROR(dwDNN_initializeTensorRTFromFileWithEngineId(&m_dnn, tensorRTModel.c_str(), nullptr,
                                                                        m_usecuDLA ? DW_PROCESSOR_TYPE_CUDLA : DW_PROCESSOR_TYPE_GPU,
                                                                        m_dlaEngineNo, m_sdk));

            CHECK_DW_ERROR(dwDNN_setCUDAStream(m_cudaStream, m_dnn));

            // Get input and output dimensions
            uint32_t numOutputTensors = m_usecuDLA ? NUM_OUTPUT_TENSORS : 1U;
            dwDNNTensorProperties inputProps;
            dwDNNTensorProperties outputProps[numOutputTensors];

            // Allocate input tensor
            CHECK_DW_ERROR(dwDNN_getInputTensorProperties(&inputProps, 0U, m_dnn));
            CHECK_DW_ERROR(dwDNNTensor_create(&m_dnnInput, &inputProps, m_sdk));

            // Allocate outputs
            for (uint32_t outputIdx = 0U; outputIdx < numOutputTensors; ++outputIdx)
            {
                CHECK_DW_ERROR(dwDNN_getOutputTensorProperties(&outputProps[outputIdx], outputIdx, m_dnn));

                // Allocate device tensors
                CHECK_DW_ERROR(dwDNNTensor_create(&m_dnnOutputsDevice[outputIdx], &outputProps[outputIdx], m_sdk));
                // Allocate host tensors
                dwDNNTensorProperties hostProps = outputProps[outputIdx];
                hostProps.tensorType            = DW_DNN_TENSOR_TYPE_CPU;

                // Allocate streamer
                CHECK_DW_ERROR(dwDNNTensorStreamer_initialize(&m_dnnOutputStreamers[outputIdx],
                                                              &outputProps[outputIdx],
                                                              hostProps.tensorType, m_sdk));
            }

            // Get coverage and bounding box blob indices
            if (m_usecuDLA)
            {
                const char* coverageBlobName    = "coverage";
                const char* boundingBoxBlobName = "bboxes";
                CHECK_DW_ERROR(dwDNN_getOutputIndex(&m_cvgIdx, coverageBlobName, m_dnn));
                CHECK_DW_ERROR(dwDNN_getOutputIndex(&m_bboxIdx, boundingBoxBlobName, m_dnn));
            }
            else
            {
                const char* coverageBlobName = "output0";
                CHECK_DW_ERROR(dwDNN_getOutputIndex(&m_cvgIdx, coverageBlobName, m_dnn));
            }

            // Get metadata from DNN module
            // DNN loads metadata automatically from json file stored next to the dnn model,
            // with the same name but additional .json extension if present.
            // Otherwise, the metadata will be filled with default values and the dataconditioner parameters
            // should be filled manually.
            dwDNNMetaData metadata;
            CHECK_DW_ERROR(dwDNN_getMetaData(&metadata, m_dnn));

            // Initialie data conditioner
            CHECK_DW_ERROR(dwDataConditioner_initializeFromTensorProperties(&m_dataConditioner, &inputProps, 1U,
                                                                            &metadata.dataConditionerParams, m_cudaStream,
                                                                            m_sdk));

            // Reserve space for detected objects
            m_detectedBoxList.reserve(m_maxDetections);
            m_detectedBoxListFloat.reserve(m_maxDetections);

            // Detection region
            m_detectionRegion.width = std::min(static_cast<uint32_t>(inputProps.dimensionSize[0]),
                                               m_imageWidth);
            m_detectionRegion.height = std::min(static_cast<uint32_t>(inputProps.dimensionSize[1]),
                                                m_imageHeight);
            m_detectionRegion.x = (m_imageWidth - m_detectionRegion.width) / 2;
            m_detectionRegion.y = (m_imageHeight - m_detectionRegion.height) / 2;

            // Compute a pixel (cell) in output in relation to a pixel in input of the network
            uint32_t gridW = outputProps[0].dimensionSize[0];
            m_cellSize     = inputProps.dimensionSize[0] / gridW;
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
        dwImageCUDA* yuvImage = nullptr;
        getNextFrame(&yuvImage, &m_imgGl);
        std::this_thread::yield();
        while (yuvImage == nullptr)
        {
            onReset();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            getNextFrame(&yuvImage, &m_imgGl);
        }

        // Run data conditioner to prepare input for the network
        CHECK_DW_ERROR(dwDataConditioner_prepareData(m_dnnInput, &m_imageRGBA, 1, &m_detectionRegion,
                                                     cudaAddressModeClamp, m_dataConditioner));

        uint32_t numOutputTensors = m_usecuDLA ? NUM_OUTPUT_TENSORS : 1U;
        // Run DNN on the output of data conditioner
        dwConstDNNTensorHandle_t inputs[1U] = {m_dnnInput};
        CHECK_DW_ERROR(dwDNN_infer(m_dnnOutputsDevice, numOutputTensors, inputs, 1U, m_dnn));

        // Stream output to host
        dwDNNTensorHandle_t outputsHost[numOutputTensors];
        for (uint32_t outputIdx = 0U; outputIdx < numOutputTensors; ++outputIdx)
        {
            dwDNNTensorStreamerHandle_t streamer = m_dnnOutputStreamers[outputIdx];
            CHECK_DW_ERROR(dwDNNTensorStreamer_producerSend(m_dnnOutputsDevice[outputIdx],
                                                            streamer));

            CHECK_DW_ERROR(dwDNNTensorStreamer_consumerReceive(&outputsHost[outputIdx], 1000,
                                                               streamer));
        }

        // Interpret output blobs to extract detected boxes
        if (m_usecuDLA)
        {
            interpretOutput(outputsHost[m_cvgIdx], outputsHost[m_bboxIdx],
                            &m_detectionRegion);
        }
        else
        {
            interpretOutput(outputsHost[m_cvgIdx],
                            &m_detectionRegion);
        }

        // Return streamed outputs
        for (uint32_t outputIdx = 0U; outputIdx < numOutputTensors; ++outputIdx)
        {
            dwDNNTensorStreamerHandle_t streamer = m_dnnOutputStreamers[outputIdx];
            CHECK_DW_ERROR(dwDNNTensorStreamer_consumerReturn(&outputsHost[outputIdx], streamer));

            CHECK_DW_ERROR(dwDNNTensorStreamer_producerReturn(nullptr, 1000, streamer));
        }
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

        // Render detection region
        dwRectf detectionRegionFloat;
        detectionRegionFloat.x      = m_detectionRegion.x;
        detectionRegionFloat.y      = m_detectionRegion.y;
        detectionRegionFloat.width  = m_detectionRegion.width;
        detectionRegionFloat.height = m_detectionRegion.height;

        CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_YELLOW, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, &detectionRegionFloat,
                                             sizeof(dwRectf), 0, 1, m_renderEngine));

        CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_RED, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setLineWidth(2.0f, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D,
                                             m_detectedBoxListFloat.data(), sizeof(dwRectf), 0,
                                             m_detectedBoxListFloat.size(), m_renderEngine));
    }

    ///------------------------------------------------------------------------------
    /// Free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        m_streamerCUDA2GL.reset();
        m_camera.reset();

        if (m_imageRGBA)
        {
            CHECK_DW_ERROR(dwImage_destroy(m_imageRGBA));
        }

        if (m_dnnInput)
        {
            CHECK_DW_ERROR(dwDNNTensor_destroy(m_dnnInput));
        }

        for (uint32_t outputIdx = 0U; outputIdx < NUM_OUTPUT_TENSORS; ++outputIdx)
        {
            if (m_dnnOutputsDevice[outputIdx])
            {
                CHECK_DW_ERROR(dwDNNTensor_destroy(m_dnnOutputsDevice[outputIdx]));
            }
            if (m_dnnOutputStreamers[outputIdx])
            {
                CHECK_DW_ERROR(dwDNNTensorStreamer_release(m_dnnOutputStreamers[outputIdx]));
            }
        }

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
        dwImageHandle_t nextFrame = m_camera->readFrame();
        if (nextFrame == nullptr)
        {
            m_camera->resetCamera();
        }
        else
        {
            dwImage_getCUDA(nextFrameCUDA, nextFrame);
            CHECK_DW_ERROR(dwImage_copyConvert(m_imageRGBA, nextFrame, m_sdk));
            dwImageHandle_t frameGL = m_streamerCUDA2GL->post(m_imageRGBA);
            dwImage_getGL(nextFrameGL, frameGL);
        }
    }

    //------------------------------------------------------------------------------
    void interpretOutput(dwDNNTensorHandle_t outConf, dwDNNTensorHandle_t outBBox, const dwRect* const roi)
    {
        // Clear detection list
        m_detectedBoxList.clear();
        m_detectedBoxListFloat.clear();

        uint32_t numBBoxes = 0U;

        void* tmpConf;
        CHECK_DW_ERROR(dwDNNTensor_lock(&tmpConf, outConf));
        dwFloat16_t* confData = reinterpret_cast<dwFloat16_t*>(tmpConf);

        void* tmpBbox;
        CHECK_DW_ERROR(dwDNNTensor_lock(&tmpBbox, outBBox));
        dwFloat16_t* bboxData = reinterpret_cast<dwFloat16_t*>(tmpBbox);

        size_t offsetY;
        size_t strideY;
        size_t height;
        uint32_t indices[4] = {0, 0, 0, 0};
        CHECK_DW_ERROR(dwDNNTensor_getLayoutView(&offsetY, &strideY, &height, indices, 4U, 1U, outConf));

        for (uint32_t gridY = 0U; gridY < height; ++gridY)
        {
            size_t offsetX;
            size_t strideX;
            size_t width;
            uint32_t subIndices[4] = {0, gridY, 0, 0};
            CHECK_DW_ERROR(dwDNNTensor_getLayoutView(&offsetX, &strideX, &width,
                                                     subIndices, 4U, 0U, outConf));
            for (uint32_t gridX = 0U; gridX < width; gridX += strideX)
            {
                dwFloat16_t conf = confData[offsetX + gridX * strideX];
                if (conf > COVERAGE_THRESHOLD && numBBoxes < m_maxDetections)
                {
                    // This is a detection!
                    float32_t imageX = (float32_t)gridX * (float32_t)m_cellSize;
                    float32_t imageY = (float32_t)gridY * (float32_t)m_cellSize;

                    size_t bboxOffset;
                    size_t bboxStride;
                    size_t numDimensions;
                    uint32_t bboxIndices[4] = {gridX, gridY, 0, 0};
                    CHECK_DW_ERROR(dwDNNTensor_getLayoutView(&bboxOffset, &bboxStride,
                                                             &numDimensions,
                                                             bboxIndices, 4U, 2U, outBBox));
                    dwFloat16_t* bboxOut = &(bboxData[bboxOffset + 0 * bboxStride]);

                    float32_t boxX1;
                    float32_t boxY1;
                    float32_t boxX2;
                    float32_t boxY2;

                    dwDataConditioner_outputPositionToInput(&boxX1, &boxY1, bboxOut[0 * bboxStride] + imageX,
                                                            bboxOut[1 * bboxStride] + imageY, roi,
                                                            m_dataConditioner);
                    dwDataConditioner_outputPositionToInput(&boxX2, &boxY2,
                                                            bboxOut[2 * bboxStride] + imageX,
                                                            bboxOut[3 * bboxStride] + imageY, roi,
                                                            m_dataConditioner);
                    dwRectf bboxFloat{boxX1, boxY1, boxX2 - boxX1, boxY2 - boxY1};
                    dwBox2D bbox;
                    bbox.width  = static_cast<int32_t>(std::round(bboxFloat.width));
                    bbox.height = static_cast<int32_t>(std::round(bboxFloat.height));
                    bbox.x      = static_cast<int32_t>(std::round(bboxFloat.x));
                    bbox.y      = static_cast<int32_t>(std::round(bboxFloat.y));

                    m_detectedBoxList.push_back(bbox);
                    m_detectedBoxListFloat.push_back(bboxFloat);
                    numBBoxes++;
                }
            }
        }

        CHECK_DW_ERROR(dwDNNTensor_unlock(outConf));
        CHECK_DW_ERROR(dwDNNTensor_unlock(outBBox));
    }

    //------------------------------------------------------------------------------
    static bool sort_score(YoloScoreRect box1, YoloScoreRect box2)
    {
        return box1.score > box2.score ? true : false;
    }

    /**
     * @brief calculate the IOU(Intersection over Union) of two boxes.
     *
     * @param[in] box1 The decription of box one.
     * @param[in] box2 The decription of box two.
     * @retval IOU value.
     */
    float32_t calculateIouOfBoxes(dwRectf box1, dwRectf box2)
    {
        float32_t x1        = std::max(box1.x, box2.x);
        float32_t y1        = std::max(box1.y, box2.y);
        float32_t x2        = std::min(box1.x + box1.width, box2.x + box2.width);
        float32_t y2        = std::min(box1.y + box1.height, box2.y + box2.height);
        float32_t w         = std::max(0.0f, x2 - x1);
        float32_t h         = std::max(0.0f, y2 - y1);
        float32_t over_area = w * h;
        return float32_t(over_area) / float32_t(box1.width * box1.height + box2.width * box2.height - over_area);
    }

    /**
     * @brief do nms(non maximum suppression) for Yolo output boxes.
     *
     * @param[in] boxes The boxes which are going to be operated with nms.
     * @param[in] threshold The threshold. Used in nms to delete duplicate boxes.
     * @return The boxes which have been operated with nms.
     */
    std::vector<YoloScoreRect> doNmsForYoloOutputBoxes(std::vector<YoloScoreRect>& boxes, float32_t threshold)
    {
        std::vector<YoloScoreRect> results;
        std::sort(boxes.begin(), boxes.end(), sort_score);
        while (boxes.size() > 0)
        {
            results.push_back(boxes[0]);
            uint32_t index = 1;
            while (index < boxes.size())
            {
                float32_t iou_value = calculateIouOfBoxes(boxes[0].rectf, boxes[index].rectf);
                if (iou_value > threshold)
                {
                    boxes.erase(boxes.begin() + index);
                }
                else
                {
                    index++;
                }
            }
            boxes.erase(boxes.begin());
        }
        return results;
    }

    //------------------------------------------------------------------------------
    void interpretOutput(dwDNNTensorHandle_t outConf, const dwRect* const roi)
    {
        // Clear detection list
        m_detectedBoxList.clear();
        m_detectedBoxListFloat.clear();
        m_label.clear();

        uint32_t numBBoxes = 0U;

        void* tmpConf;
        CHECK_DW_ERROR(dwDNNTensor_lock(&tmpConf, outConf));
        float32_t* confData = reinterpret_cast<float32_t*>(tmpConf);

        size_t offsetY;
        size_t strideY;
        size_t height;
        uint32_t indices[4] = {0, 0, 0, 0};
        CHECK_DW_ERROR(dwDNNTensor_getLayoutView(&offsetY, &strideY, &height, indices, 4U, 1U, outConf));

        std::vector<YoloScoreRect> tmpRes;
        for (uint16_t gridY = 0U; gridY < height; ++gridY)
        {
            const float32_t* outConfRow = &confData[gridY * strideY];
            if (outConfRow[4] < CONFIDENCE_THRESHOLD || numBBoxes >= 100)
            {
                continue;
            }
            uint16_t maxIndex  = 0;
            float32_t maxScore = 0;
            for (uint16_t i = 5; i < 85; i++)
            { // The col 5-85 represents the probability of each class.
                if (outConfRow[i] > maxScore)
                {
                    maxScore = outConfRow[i];
                    maxIndex = i;
                }
            }

            if (maxScore > SCORE_THRESHOLD)
            {
                // This is a detection!
                float32_t imageX = (float32_t)outConfRow[0];
                float32_t imageY = (float32_t)outConfRow[1];
                float32_t bboxW  = (float32_t)outConfRow[2];
                float32_t bboxH  = (float32_t)outConfRow[3];

                float32_t boxX1Tmp = (float32_t)(imageX - 0.5 * bboxW);
                float32_t boxY1Tmp = (float32_t)(imageY - 0.5 * bboxH);
                float32_t boxX2Tmp = (float32_t)(imageX + 0.5 * bboxW);
                float32_t boxY2Tmp = (float32_t)(imageY + 0.5 * bboxH);

                float32_t boxX1;
                float32_t boxY1;
                float32_t boxX2;
                float32_t boxY2;

                dwDataConditioner_outputPositionToInput(&boxX1, &boxY1, boxX1Tmp,
                                                        boxY1Tmp, roi,
                                                        m_dataConditioner);
                dwDataConditioner_outputPositionToInput(&boxX2, &boxY2,
                                                        boxX2Tmp,
                                                        boxY2Tmp, roi,
                                                        m_dataConditioner);
                dwRectf bboxFloat{boxX1, boxY1, boxX2 - boxX1, boxY2 - boxY1};
                tmpRes.push_back({bboxFloat, maxScore, (uint16_t)(maxIndex - 5)});
                numBBoxes++;
            }
        }

        CHECK_DW_ERROR(dwDNNTensor_unlock(outConf));

        std::vector<YoloScoreRect> tmpResAfterNMS = doNmsForYoloOutputBoxes(tmpRes, float32_t(0.45));
        for (uint32_t i = 0; i < tmpResAfterNMS.size(); i++)
        {
            YoloScoreRect box = tmpResAfterNMS[i];
            dwRectf bboxFloat = box.rectf;
            dwBox2D bbox;
            bbox.width  = static_cast<int32_t>(std::round(bboxFloat.width));
            bbox.height = static_cast<int32_t>(std::round(bboxFloat.height));
            bbox.x      = static_cast<int32_t>(std::round(bboxFloat.x));
            bbox.y      = static_cast<int32_t>(std::round(bboxFloat.y));

            if (YOLO_CLASS_NAMES[box.classIndex] == "car")
            {
                m_detectedBoxList.push_back(bbox);
                m_detectedBoxListFloat.push_back(bboxFloat);
                m_label.push_back(YOLO_CLASS_NAMES[box.classIndex]);
            }
        }
    }

    //------------------------------------------------------------------------------
    float32_t overlap(const dwRectf& boxA, const dwRectf& boxB)
    {

        int32_t overlapWidth = std::min(boxA.x + boxA.width,
                                        boxB.x + boxB.width) -
                               std::max(boxA.x, boxB.x);
        int32_t overlapHeight = std::min(boxA.y + boxA.height,
                                         boxB.y + boxB.height) -
                                std::max(boxA.y, boxB.y);

        return (overlapWidth < 0 || overlapHeight < 0) ? 0.0f : (overlapWidth * overlapHeight);
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

        if (m_usecuDLA)
        {
            path = "cudla";
        }
        else if (gpuProp.major == CUDA_AMPERE_MAJOR_COMPUTE_CAPABILITY)
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
            path = "pascal";
        }

        return path;
    }
};

int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
#ifdef VIBRANTE
                              ProgramArguments::Option_t("camera-type", "ar0231-rccb-bae-sf3324", "camera gmsl type (see sample_sensors_info for all available camera types on this platform)"),
                              ProgramArguments::Option_t("camera-group", "a", "input port"),
                              ProgramArguments::Option_t("camera-index", "0", "camera index within the camera-group 0-3"),
                              ProgramArguments::Option_t("input-type", "video", "input type either video or camera"),
                              ProgramArguments::Option_t("cudla", "0", "run inference on cudla"),
                              ProgramArguments::Option_t("dla-engine", "0", "dla engine number to run on if --cudla=1"),
#endif
                              ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str(), "path to video"),
                              ProgramArguments::Option_t("tensorRT_model", "", (std::string("path to TensorRT model file. By default: ") + dw_samples::SamplesDataPath::get() + "/samples/detector/<gpu-architecture>/tensorRT_model.bin").c_str())},
                          "DNN Tensor sample which detects and tracks cars.");

    DNNTensorSample app(args);
    app.initializeWindow("DNN Tensor Sample", 1280, 800, args.enabled("offscreen"));

    if (!args.enabled("offscreen"))
        app.setProcessRate(30);

    return app.run();
}
