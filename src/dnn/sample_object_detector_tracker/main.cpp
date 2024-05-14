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

// Samples
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SimpleCamera.hpp>

// Core
#include <dw/core/context/Context.h>
#include <dw/core/logger/Logger.h>
#include <dw/core/base/Version.h>

// HAL
#include <dw/sensors/Sensors.h>

// DNN
#include <dw/dnn/DNN.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>

// Tracker
#include <dw/imageprocessing/featuredetector/FeatureDetector.h>
#include <dw/imageprocessing/tracking/featuretracker/FeatureTracker.h>
#include <dw/imageprocessing/tracking/boxtracker2d/BoxTracker2D.h>

using namespace dw_samples::common;

class ObjectTrackerApp : public DriveWorksSample
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
    const uint32_t m_maxDetections                      = 1000U;
    const float32_t m_nonMaxSuppressionOverlapThreshold = 0.5;

    dwDNNHandle_t m_dnn                         = DW_NULL_HANDLE;
    dwDataConditionerHandle_t m_dataConditioner = DW_NULL_HANDLE;
    std::vector<dwBox2D> m_detectedBoxList;
    std::vector<dwRectf> m_detectedBoxListFloat;
    float32_t* m_dnnInputDevice                      = nullptr;
    float32_t* m_dnnOutputsDevice[2]                 = {nullptr};
    std::unique_ptr<float32_t[]> m_dnnOutputsHost[2] = {nullptr};

    uint32_t m_cvgIdx;
    uint32_t m_bboxIdx;
    dwBlobSize m_networkInputDimensions;
    dwBlobSize m_networkOutputDimensions[2];

    uint32_t m_totalSizeInput;
    uint32_t m_totalSizesOutput[2];
    dwRect m_detectionRegion;

    // ------------------------------------------------
    // YOLO: 1 input and 1 output
    // Switch to Yolo onnx from now on
    // ------------------------------------------------
    static constexpr bool USE_YOLO                  = true;
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
    std::vector<dwTrackedBox2D> m_detectedTrackBox;

    // ------------------------------------------------
    // Feature Tracker
    // ------------------------------------------------
    uint32_t m_maxFeatureCount;
    uint32_t m_historyCapacity;

    dwFeature2DDetectorHandle_t m_featureDetector = DW_NULL_HANDLE;
    dwFeature2DTrackerHandle_t m_featureTracker   = DW_NULL_HANDLE;

    dwFeatureHistoryArray m_featureHistoryCPU = {};
    dwFeatureHistoryArray m_featureHistoryGPU = {};
    dwFeatureArray m_featureDetectedGPU       = {};

    dwPyramidImage m_pyramidPrevious = {};
    dwPyramidImage m_pyramidCurrent  = {};

    uint8_t* m_featureMask;
    size_t m_maskPitch;
    dwVector2ui m_maskSize{};

    // ------------------------------------------------
    // Box Tracker
    // ------------------------------------------------
    dwBoxTracker2DHandle_t m_boxTracker;
    std::vector<float32_t> m_previousFeatureLocations;
    std::vector<float32_t> m_currentFeatureLocations;
    std::vector<dwFeature2DStatus> m_featureStatuses;
    const dwTrackedBox2D* m_trackedBoxes = nullptr;
    size_t m_numTrackedBoxes             = 0;
    std::vector<dwRectf> m_trackedBoxListFloat;

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
    ObjectTrackerApp(const ProgramArguments& args)
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
                    parameterString += ",slave=" + getArgument("slave");
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

                m_camera.reset(new SimpleCamera(params, m_sal, m_sdk));
                dwImageProperties outputProperties = m_camera->getOutputProperties();
                outputProperties.type              = DW_IMAGE_CUDA;
                m_camera->setOutputProperties(outputProperties);
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
            // If not specified, load the correct network based on platform
            std::string tensorRTModel = getArgument("tensorRT_model");
            if (tensorRTModel.empty())
            {
                tensorRTModel = dw_samples::SamplesDataPath::get() + "/samples/detector/";
                tensorRTModel += getPlatformPrefix();
                tensorRTModel += "/tensorRT_model.bin";
            }

            // Initialize DNN from a TensorRT file
            CHECK_DW_ERROR(dwDNN_initializeTensorRTFromFile(&m_dnn, tensorRTModel.c_str(), nullptr,
                                                            DW_PROCESSOR_TYPE_GPU, m_sdk));

            CHECK_DW_ERROR(dwDNN_setCUDAStream(m_cudaStream, m_dnn));

            auto getTotalSize = [](const dwBlobSize& blobSize) {
                return blobSize.channels * blobSize.height * blobSize.width;
            };

            // Get input dimensions
            CHECK_DW_ERROR(dwDNN_getInputSize(&m_networkInputDimensions, 0U, m_dnn));
            // Calculate total size needed to store input
            m_totalSizeInput = getTotalSize(m_networkInputDimensions);
            // Allocate GPU memory
            CHECK_CUDA_ERROR(cudaMalloc((void**)&m_dnnInputDevice, sizeof(float32_t) * m_totalSizeInput));

            if (USE_YOLO)
            {
                // Get output dimensions
                CHECK_DW_ERROR(dwDNN_getOutputSize(&m_networkOutputDimensions[0], 0U, m_dnn));

                // Calculate total size needed to store output
                m_totalSizesOutput[0] = getTotalSize(m_networkOutputDimensions[0]);

                // Get coverage and bounding box blob indices
                const char* coverageBlobName = "output0";
                CHECK_DW_ERROR(dwDNN_getOutputIndex(&m_cvgIdx, coverageBlobName, m_dnn));

                // Allocate GPU memory
                CHECK_CUDA_ERROR(cudaMalloc((void**)&m_dnnOutputsDevice[0],
                                            sizeof(float32_t) * m_totalSizesOutput[0]));

                // Allocate CPU memory for reading the output of DNN
                m_dnnOutputsHost[0].reset(new float32_t[m_totalSizesOutput[0]]);
            }
            else
            {
                // Get output dimensions
                CHECK_DW_ERROR(dwDNN_getOutputSize(&m_networkOutputDimensions[0], 0U, m_dnn));
                CHECK_DW_ERROR(dwDNN_getOutputSize(&m_networkOutputDimensions[1], 1U, m_dnn));

                // Calculate total size needed to store output
                m_totalSizesOutput[0] = getTotalSize(m_networkOutputDimensions[0]);
                m_totalSizesOutput[1] = getTotalSize(m_networkOutputDimensions[1]);

                // Get coverage and bounding box blob indices
                const char* coverageBlobName    = "coverage_sig";
                const char* boundingBoxBlobName = "bbox_regressor";
                CHECK_DW_ERROR(dwDNN_getOutputIndex(&m_cvgIdx, coverageBlobName, m_dnn));
                CHECK_DW_ERROR(dwDNN_getOutputIndex(&m_bboxIdx, boundingBoxBlobName, m_dnn));

                // Allocate GPU memory
                CHECK_CUDA_ERROR(cudaMalloc((void**)&m_dnnOutputsDevice[0],
                                            sizeof(float32_t) * m_totalSizesOutput[0]));
                CHECK_CUDA_ERROR(cudaMalloc((void**)&m_dnnOutputsDevice[1],
                                            sizeof(float32_t) * m_totalSizesOutput[1]));

                // Allocate CPU memory for reading the output of DNN
                m_dnnOutputsHost[0].reset(new float32_t[m_totalSizesOutput[0]]);
                m_dnnOutputsHost[1].reset(new float32_t[m_totalSizesOutput[1]]);
            }

            // Get metadata from DNN module
            // DNN loads metadata automatically from json file stored next to the dnn model,
            // with the same name but additional .json extension if present.
            // Otherwise, the metadata will be filled with default values and the dataconditioner parameters
            // should be filled manually.
            dwDNNMetaData metadata;
            CHECK_DW_ERROR(dwDNN_getMetaData(&metadata, m_dnn));

            // Initialie data conditioner
            CHECK_DW_ERROR(dwDataConditioner_initialize(&m_dataConditioner, &m_networkInputDimensions, 1U,
                                                        &metadata.dataConditionerParams, m_cudaStream,
                                                        m_sdk));

            // Reserve space for detected objects
            m_detectedBoxList.reserve(m_maxDetections);
            m_detectedBoxListFloat.reserve(m_maxDetections);

            // Detection region
            m_detectionRegion.width = std::min(static_cast<uint32_t>(m_networkInputDimensions.width),
                                               m_imageWidth);
            m_detectionRegion.height = std::min(static_cast<uint32_t>(m_networkInputDimensions.height),
                                                m_imageHeight);
            m_detectionRegion.x = (m_imageWidth - m_detectionRegion.width) / 2;
            m_detectionRegion.y = (m_imageHeight - m_detectionRegion.height) / 2;
        }

        //------------------------------------------------------------------------------
        // Initialize Feature Tracker
        //------------------------------------------------------------------------------
        {
            if (USE_YOLO)
            {
                m_maxFeatureCount = 2000;
                m_historyCapacity = 3;
            }
            else
            {
                m_maxFeatureCount = 4000;
                m_historyCapacity = 10;
            }

            dwFeature2DDetectorConfig featureDetectorConfig{};
            featureDetectorConfig.imageWidth  = m_imageWidth;
            featureDetectorConfig.imageHeight = m_imageHeight;
            CHECK_DW_ERROR(dwFeature2DDetector_initDefaultParams(&featureDetectorConfig));
            featureDetectorConfig.maxFeatureCount = m_maxFeatureCount;
            CHECK_DW_ERROR(dwFeature2DDetector_initialize(&m_featureDetector, &featureDetectorConfig,
                                                          m_cudaStream, m_sdk));

            dwFeature2DTrackerConfig featureTrackerConfig{};
            featureTrackerConfig.imageWidth  = m_imageWidth;
            featureTrackerConfig.imageHeight = m_imageHeight;
            CHECK_DW_ERROR(dwFeature2DTracker_initDefaultParams(&featureTrackerConfig));
            featureTrackerConfig.maxFeatureCount = m_maxFeatureCount;
            featureTrackerConfig.historyCapacity = m_historyCapacity;
            featureTrackerConfig.detectorType    = featureDetectorConfig.type;
            CHECK_DW_ERROR(dwFeature2DTracker_initialize(&m_featureTracker, &featureTrackerConfig,
                                                         m_cudaStream, m_sdk));

            // Tracker pyramid init
            CHECK_DW_ERROR(dwPyramid_create(&m_pyramidPrevious, featureTrackerConfig.pyramidLevelCount, m_imageWidth,
                                            m_imageHeight, DW_TYPE_UINT8, m_sdk));
            CHECK_DW_ERROR(dwPyramid_create(&m_pyramidCurrent, featureTrackerConfig.pyramidLevelCount, m_imageWidth,
                                            m_imageHeight, DW_TYPE_UINT8, m_sdk));

            CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&m_featureHistoryCPU, m_maxFeatureCount,
                                                           m_historyCapacity, DW_MEMORY_TYPE_CPU, nullptr, m_sdk));
            CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&m_featureHistoryGPU, m_maxFeatureCount,
                                                           m_historyCapacity, DW_MEMORY_TYPE_CUDA, nullptr, m_sdk));
            CHECK_DW_ERROR(dwFeatureArray_createNew(&m_featureDetectedGPU, m_maxFeatureCount,
                                                    DW_MEMORY_TYPE_CUDA, nullptr, m_sdk));

            // Set up mask. Apply feature tracking only to half of the image
            dwPyramidImageProperties pyramidProps{};
            CHECK_DW_ERROR(dwPyramid_getProperties(&pyramidProps, &m_pyramidCurrent, m_sdk));
            m_maskSize.x = pyramidProps.levelProps[featureDetectorConfig.detectionLevel].width;
            m_maskSize.y = pyramidProps.levelProps[featureDetectorConfig.detectionLevel].height;
            CHECK_CUDA_ERROR(cudaMallocPitch(&m_featureMask, &m_maskPitch, m_maskSize.x, m_maskSize.y));
            CHECK_CUDA_ERROR(cudaMemset(m_featureMask, 255, m_maskPitch * m_maskSize.y));
            CHECK_CUDA_ERROR(cudaMemset(m_featureMask, 0, m_maskPitch * m_maskSize.y / 2));

            CHECK_DW_ERROR(dwFeature2DDetector_setMask(m_featureMask, m_maskPitch, m_maskSize.x,
                                                       m_maskSize.y, m_featureDetector));
        }

        //------------------------------------------------------------------------------
        // Initialize Box Tracker
        //------------------------------------------------------------------------------
        {
            dwBoxTracker2DParams params{};
            dwBoxTracker2D_initParams(&params);
            params.maxBoxImageScale    = 0.5f;
            params.minBoxImageScale    = 0.005f;
            params.similarityThreshold = 0.2f;
            params.groupThreshold      = 2.0f;
            params.maxBoxCount         = m_maxDetections;
            CHECK_DW_ERROR(dwBoxTracker2D_initialize(&m_boxTracker, &params, m_imageWidth, m_imageHeight,
                                                     m_sdk));
            // Reserve for storing feature locations and statuses in CPU
            m_currentFeatureLocations.reserve(2 * m_maxFeatureCount);
            m_previousFeatureLocations.reserve(2 * m_maxFeatureCount);
            m_featureStatuses.reserve(2 * m_maxFeatureCount);
            m_trackedBoxListFloat.reserve(m_maxDetections);
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
        dwImageCUDA* rgbaImage;
        CHECK_DW_ERROR(dwImage_getCUDA(&rgbaImage, m_imageRGBA));
        CHECK_DW_ERROR(dwDataConditioner_prepareDataRaw(m_dnnInputDevice, &rgbaImage, 1, &m_detectionRegion,
                                                        cudaAddressModeClamp, m_dataConditioner));
        // Run DNN on the output of data conditioner
        CHECK_DW_ERROR(dwDNN_inferRaw(m_dnnOutputsDevice, &m_dnnInputDevice, 1U, m_dnn));

        // Copy output back
        CHECK_CUDA_ERROR(cudaMemcpy(m_dnnOutputsHost[0].get(), m_dnnOutputsDevice[0],
                                    sizeof(float32_t) * m_totalSizesOutput[0], cudaMemcpyDeviceToHost));

        // Interpret output blobs to extract detected boxes
        if (USE_YOLO)
        {
            interpretOutput(m_dnnOutputsHost[m_cvgIdx].get(),
                            &m_detectionRegion);
        }
        else
        {
            CHECK_CUDA_ERROR(cudaMemcpy(m_dnnOutputsHost[1].get(), m_dnnOutputsDevice[1],
                                        sizeof(float32_t) * m_totalSizesOutput[1], cudaMemcpyDeviceToHost));
            interpretOutput(m_dnnOutputsHost[m_cvgIdx].get(), m_dnnOutputsHost[m_bboxIdx].get(),
                            &m_detectionRegion);
        }

        // Track objects
        runTracker(yuvImage);
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
                                             m_trackedBoxListFloat.data(), sizeof(dwRectf), 0,
                                             m_trackedBoxListFloat.size(), m_renderEngine));

        CHECK_DW_ERROR(dwRenderEngine_setPointSize(2.0f, m_renderEngine));
        // Draw tracked features that belong to detected objects
        for (uint32_t boxIdx = 0U; boxIdx < m_numTrackedBoxes; ++boxIdx)
        {
            const dwTrackedBox2D& trackedBox = m_trackedBoxes[boxIdx];
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D,
                                  trackedBox.featureLocations, sizeof(dwVector2f),
                                  0, trackedBox.nFeatures, m_renderEngine);
            // Render box id
            dwVector2f pos{static_cast<float32_t>(trackedBox.box.x),
                           static_cast<float32_t>(trackedBox.box.y)};
            dwRenderEngine_renderText2D(std::to_string(trackedBox.id).c_str(), pos, m_renderEngine);
        }
    }

    ///------------------------------------------------------------------------------
    /// Free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        // Free GPU memory
        if (m_dnnOutputsDevice[0])
        {
            CHECK_CUDA_ERROR(cudaFree(m_dnnOutputsDevice[0]));
        }
        if (m_dnnOutputsDevice[1])
        {
            CHECK_CUDA_ERROR(cudaFree(m_dnnOutputsDevice[1]));
        }
        if (m_featureMask)
        {
            CHECK_CUDA_ERROR(cudaFree(m_featureMask));
        }
        if (m_imageRGBA)
        {
            CHECK_DW_ERROR(dwImage_destroy(m_imageRGBA));
        }

        // Release box tracker
        CHECK_DW_ERROR(dwBoxTracker2D_release(m_boxTracker));

        // Release feature tracker and list
        CHECK_DW_ERROR(dwFeatureHistoryArray_destroy(m_featureHistoryCPU));
        CHECK_DW_ERROR(dwFeatureHistoryArray_destroy(m_featureHistoryGPU));
        CHECK_DW_ERROR(dwFeatureArray_destroy(m_featureDetectedGPU));

        CHECK_DW_ERROR(dwFeature2DDetector_release(m_featureDetector));
        CHECK_DW_ERROR(dwFeature2DTracker_release(m_featureTracker));

        // Release pyramids
        CHECK_DW_ERROR(dwPyramid_destroy(m_pyramidCurrent));
        CHECK_DW_ERROR(dwPyramid_destroy(m_pyramidPrevious));

        // Release detector
        CHECK_DW_ERROR(dwDNN_release(m_dnn));
        // Release data conditioner
        CHECK_DW_ERROR(dwDataConditioner_release(m_dataConditioner));
        // Release render engine
        CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        // Release camera
        m_camera.reset();
        m_streamerCUDA2GL.reset();

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

        CHECK_DW_ERROR(dwFeatureHistoryArray_reset(&m_featureHistoryGPU, m_cudaStream));
        CHECK_DW_ERROR(dwFeatureArray_reset(&m_featureDetectedGPU, m_cudaStream));

        CHECK_DW_ERROR(dwFeature2DDetector_reset(m_featureDetector));
        CHECK_DW_ERROR(dwFeature2DTracker_reset(m_featureTracker));
        CHECK_DW_ERROR(dwBoxTracker2D_reset(m_boxTracker));

        CHECK_DW_ERROR(dwFeature2DDetector_setMask(m_featureMask, m_maskPitch, m_maskSize.x,
                                                   m_maskSize.y, m_featureDetector));
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
    void interpretOutput(const float32_t* outConf, const float32_t* outBBox, const dwRect* const roi)
    {
        // Clear detection list
        m_detectedBoxList.clear();
        m_detectedBoxListFloat.clear();

        uint32_t numBBoxes = 0U;
        uint16_t gridH     = m_networkOutputDimensions[0].height;
        uint16_t gridW     = m_networkOutputDimensions[0].width;
        uint16_t cellSize  = m_networkInputDimensions.height / gridH;
        uint32_t gridSize  = gridH * gridW;

        for (uint16_t gridY = 0U; gridY < gridH; ++gridY)
        {
            const float32_t* outConfRow = &outConf[gridY * gridW];
            for (uint16_t gridX = 0U; gridX < gridW; ++gridX)
            {
                float32_t conf = outConfRow[gridX];
                if (conf > COVERAGE_THRESHOLD && numBBoxes < m_maxDetections)
                {
                    // This is a detection!
                    float32_t imageX = (float32_t)gridX * (float32_t)cellSize;
                    float32_t imageY = (float32_t)gridY * (float32_t)cellSize;
                    uint32_t offset  = gridY * gridW + gridX;

                    float32_t boxX1;
                    float32_t boxY1;
                    float32_t boxX2;
                    float32_t boxY2;

                    dwDataConditioner_outputPositionToInput(&boxX1, &boxY1, outBBox[offset] + imageX,
                                                            outBBox[gridSize + offset] + imageY, roi,
                                                            m_dataConditioner);
                    dwDataConditioner_outputPositionToInput(&boxX2, &boxY2,
                                                            outBBox[gridSize * 2 + offset] + imageX,
                                                            outBBox[gridSize * 3 + offset] + imageY, roi,
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
    void interpretOutput(const float32_t* outConf, const dwRect* const roi)
    {
        // Clear detection list
        m_detectedBoxList.clear();
        m_detectedBoxListFloat.clear();
        m_detectedTrackBox.clear();
        m_label.clear();

        uint32_t numBBoxes = 0U;
        uint16_t gridH     = m_networkOutputDimensions[0].height;
        uint16_t gridW     = m_networkOutputDimensions[0].width;
        std::vector<YoloScoreRect> tmpRes;

        for (uint16_t gridY = 0U; gridY < gridH; ++gridY)
        {
            const float32_t* outConfRow = &outConf[gridY * gridW];
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
                dwTrackedBox2D tmpTrackBox;
                tmpTrackBox.box        = bbox;
                tmpTrackBox.id         = -1;
                tmpTrackBox.confidence = box.score;
                m_detectedTrackBox.push_back(tmpTrackBox);
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

    //------------------------------------------------------------------------------
    void runTracker(const dwImageCUDA* image)
    {
        // add candidates to box tracker
        if (USE_YOLO)
        {
            CHECK_DW_ERROR(dwBoxTracker2D_addPreClustered(m_detectedTrackBox.data(), m_detectedTrackBox.size(), m_boxTracker));
        }
        else
        {
            CHECK_DW_ERROR(dwBoxTracker2D_add(m_detectedBoxList.data(), m_detectedBoxList.size(), m_boxTracker));
        }

        // track features
        uint32_t featureCount = trackFeatures(image);

        // If this is not the first frame, update the features
        if (getFrameIndex() != 0)
        {
            // update box features
            CHECK_DW_ERROR(dwBoxTracker2D_updateFeatures(m_previousFeatureLocations.data(),
                                                         m_featureStatuses.data(),
                                                         featureCount, m_boxTracker));
        }

        // Run box tracker
        CHECK_DW_ERROR(dwBoxTracker2D_track(m_currentFeatureLocations.data(), m_featureStatuses.data(),
                                            m_previousFeatureLocations.data(), m_boxTracker));

        // Get tracked boxes
        CHECK_DW_ERROR(dwBoxTracker2D_get(&m_trackedBoxes, &m_numTrackedBoxes, m_boxTracker));

        // Extract boxes from tracked object list
        m_trackedBoxListFloat.clear();
        for (uint32_t tIdx = 0U; tIdx < m_numTrackedBoxes; ++tIdx)
        {
            const dwBox2D& box = m_trackedBoxes[tIdx].box;
            dwRectf rectf;
            rectf.x      = static_cast<float32_t>(box.x);
            rectf.y      = static_cast<float32_t>(box.y);
            rectf.width  = static_cast<float32_t>(box.width);
            rectf.height = static_cast<float32_t>(box.height);
            m_trackedBoxListFloat.push_back(rectf);
        }
    }

    // ------------------------------------------------
    // Feature tracking
    // ------------------------------------------------
    uint32_t trackFeatures(const dwImageCUDA* image)
    {
        std::swap(m_pyramidCurrent, m_pyramidPrevious);

        // build pyramid
        CHECK_DW_ERROR(dwImageFilter_computePyramid(&m_pyramidCurrent, image, m_cudaStream, m_sdk));

        // track features
        dwFeatureArray featurePredicted{};
        CHECK_DW_ERROR(dwFeature2DTracker_trackFeatures(&m_featureHistoryGPU, &featurePredicted,
                                                        nullptr, &m_featureDetectedGPU, nullptr,
                                                        &m_pyramidPrevious, &m_pyramidCurrent,
                                                        m_featureTracker));

        //Get feature info to CPU
        CHECK_DW_ERROR(dwFeatureHistoryArray_copyAsync(&m_featureHistoryCPU, &m_featureHistoryGPU, 0));

        // Update feature locations after tracking
        uint32_t featureCount = updateFeatureLocationsStatuses();

        // detect new features
        CHECK_DW_ERROR(dwFeature2DDetector_detectFromPyramid(&m_featureDetectedGPU, &m_pyramidCurrent,
                                                             &featurePredicted, nullptr, m_featureDetector));

        return featureCount;
    }

    // ------------------------------------------------
    uint32_t updateFeatureLocationsStatuses()
    {
        // Get previous locations and update box tracker
        dwFeatureArray curFeatures{};
        dwFeatureArray preFeatures{};
        CHECK_DW_ERROR(dwFeatureHistoryArray_getCurrent(&curFeatures, &m_featureHistoryCPU));
        CHECK_DW_ERROR(dwFeatureHistoryArray_getPrevious(&preFeatures, &m_featureHistoryCPU));

        dwVector2f* preLocations = preFeatures.locations;
        dwVector2f* curLocations = curFeatures.locations;
        uint32_t newSize         = std::min(m_maxFeatureCount, *m_featureHistoryCPU.featureCount);

        m_previousFeatureLocations.clear();
        m_currentFeatureLocations.clear();
        m_featureStatuses.clear();
        for (uint32_t featureIdx = 0; featureIdx < newSize; featureIdx++)
        {
            m_previousFeatureLocations.push_back(preLocations[featureIdx].x);
            m_previousFeatureLocations.push_back(preLocations[featureIdx].y);
            m_currentFeatureLocations.push_back(curLocations[featureIdx].x);
            m_currentFeatureLocations.push_back(curLocations[featureIdx].y);

            m_featureStatuses.push_back(m_featureHistoryCPU.statuses[featureIdx]);
        }
        return newSize;
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
                              ProgramArguments::Option_t("slave", "0", "activate slave mode for Tegra B"),
                              ProgramArguments::Option_t("input-type", "video", "input type either video or camera"),
#endif
                              ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str(), "path to video"),
                              ProgramArguments::Option_t("tensorRT_model", "", (std::string("path to TensorRT model file. By default: ") + dw_samples::SamplesDataPath::get() + "/samples/detector/<gpu-architecture>/tensorRT_model.bin").c_str())},
                          "Object Tracker sample which detects and tracks cars.");

    ObjectTrackerApp app(args);
    app.initializeWindow("Object Detector Tracker Sample", 1280, 800, args.enabled("offscreen"));

    if (!args.enabled("offscreen"))
        app.setProcessRate(30);

    return app.run();
}
