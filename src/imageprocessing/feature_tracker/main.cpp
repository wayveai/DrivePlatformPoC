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

#include <framework/DriveWorksSample.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Log.hpp>
#include <dw/core/base/Version.h>

#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>
#include <dwvisualization/interop/ImageStreamer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>

#ifdef DW_SDK_BUILD_PVA
#include <dw/imageprocessing/pyramid/PyramidPVA.h>
#include <cupva_host_wrapper.h>

#if VIBRANTE_PDK_DECIMAL >= 6000400 && !defined(DW_IS_SAFETY)
#include "cupva_cuda_wrapper.h"
#endif
#endif
#include <dw/imageprocessing/tracking/featuretracker/FeatureTracker.h>
#include <dw/imageprocessing/featuredetector/FeatureDetector.h>

#include <dw/imageprocessing/geometry/imagetransformation/ImageTransformation.h>

// Screen capture
#include <dwvisualization/image/FrameCapture.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Camera Feature Tracker
// The Camera Feature Tracker sample demonstrates the feature
// tracking capabilities of the dw_imageprocessing module. It loads a video stream and
// reads the images sequentially. For each frame, it tracks features from the previous frame
// and combine them with the newly detected features in the current frame.
//------------------------------------------------------------------------------
class CameraFeatureTracker : public DriveWorksSample
{
private:
    std::unique_ptr<ScreenshotHelper> m_screenshot;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                   = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer         = DW_NULL_HANDLE;
    dwSensorHandle_t m_camera             = DW_NULL_HANDLE;
    dwCameraFrameHandle_t m_frame         = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_image2GL    = DW_NULL_HANDLE;
    dwCameraProperties m_cameraProps      = {};
    dwImageProperties m_cameraImageProps  = {};
    bool m_loop{std::stoi(getArgument("loop")) != 0};

    dwImageHandle_t m_processImageDownscale{DW_NULL_HANDLE};
    dwImageTransformationHandle_t m_imageTransformer = DW_NULL_HANDLE;

    dwRenderBufferHandle_t m_featureRenderBuffer;
    dwRenderBufferHandle_t m_featureHistoryRenderBuffer;

    dwImageHandle_t m_imageYUV{DW_NULL_HANDLE};
    dwImageHandle_t m_imageRGBA{DW_NULL_HANDLE};

    bool drawHistory = true;

    // use half resolution for processing
    bool m_useHalfRes{std::stoi(getArgument("useHalfRes")) != 0};
    // Custom downscaling.
    size_t m_customDownscale{std::stoul(getArgument("useCustomDownscale"))};
    bool m_usePvaPyramid{std::atoi(getArgument("pvaPyramid").c_str()) == 1};

    // tracker handles
    dwFeature2DTrackerHandle_t tracker   = DW_NULL_HANDLE;
    dwFeature2DDetectorHandle_t detector = DW_NULL_HANDLE;
    uint32_t maxFeatureCount             = 0;
    uint32_t historyCapacity             = 0;

// pyramid handles
#ifdef DW_SDK_BUILD_PVA
    cupvaStream_t m_cupvaStream;
    dwPyramidPVAHandle_t m_pyramidPVA = DW_NULL_HANDLE;
#endif

    cudaStream_t m_cudaStream = 0;

    dwPyramidImage pyramidCurrent  = {};
    dwPyramidImage pyramidPrevious = {};

    // These point into the buffers of featureList
    dwFeatureHistoryArray featureHistoryCPU = {};
    dwFeatureHistoryArray featureHistoryGPU = {};
    dwFeatureArray featuresDetected         = {};
    float32_t* d_nccScores                  = nullptr;

    // Frame index for early stop
    uint32_t m_stopFrameIdx{0};
    uint32_t m_curFrameIdx{0};

    //------------------------------------------------------------------------------
    // Screen Capture
    //------------------------------------------------------------------------------
    bool m_enableScreenCapture = std::stoi(getArgument("capture-screen"));
    dwFrameCaptureHandle_t m_screenCapture{DW_NULL_HANDLE};
    bool m_shouldCaptureScreen{false};
    int32_t m_screenCaptureStartFrame{0};
    int32_t m_screenCaptureEndFrame{-1};
    int32_t m_screenCaptureFrameCount{0};

public:
    CameraFeatureTracker(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize Renderer, Sensors, Image Streamers and Tracker
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Get values from command line
        // -----------------------------------------
        maxFeatureCount = std::stoi(getArgument("maxFeatureCount"));
        historyCapacity = std::stoi(getArgument("historyCapacity"));

        uint32_t pvaPyramidEngineNo = std::atoi(getArgument("pvaPyramidEngineNo").c_str());

        uint32_t detectMode                 = std::stoi(getArgument("detectMode"));
        bool pvaDetector                    = std::atoi(getArgument("pvaDetector").c_str()) == 1;
        uint32_t pvaDetectorEngineNo        = std::atoi(getArgument("pvaDetectorEngineNo").c_str());
        uint32_t detectLevel                = std::stoi(getArgument("detectLevel"));
        uint32_t numEvenDistributionPerCell = std::stoi(getArgument("numEvenDistributionPerCell"));
        uint32_t harrisRadius               = std::stoi(getArgument("harrisRadius"));
        uint32_t blockSize                  = std::stoi(getArgument("blockSize"));
        uint32_t gradientSize               = std::stoi(getArgument("gradientSize"));
        uint32_t maskType                   = std::stoi(getArgument("maskType"));
        uint32_t enableMaskAdjustment       = std::stoi(getArgument("enableMaskAdjustment"));

        float32_t scoreThreshold  = std::stof(getArgument("scoreThreshold"));
        float32_t detailThreshold = std::stof(getArgument("detailThreshold"));
        float32_t harrisK         = std::stof(getArgument("harrisK"));

        uint32_t trackMode               = std::stoi(getArgument("trackMode"));
        bool pvaTracker                  = std::atoi(getArgument("pvaTracker").c_str()) == 1;
        uint32_t pvaTrackerEngineNo      = std::atoi(getArgument("pvaTrackerEngineNo").c_str());
        uint32_t windowSize              = std::stoi(getArgument("windowSize"));
        bool useHalfDetector             = std::stoi(getArgument("useHalfDetector")) != 0;
        bool enableAdaptiveWindowSize    = std::stoi(getArgument("enableAdaptiveWindowSize")) != 0;
        uint32_t numIterTranslation      = std::stoi(getArgument("numIterTranslation"));
        uint32_t numIterScaling          = std::stoi(getArgument("numIterScaling"));
        uint32_t numTranslationOnlyLevel = std::stoi(getArgument("numTranslationOnlyLevel"));
        bool useHalfTracker              = std::stoi(getArgument("useHalfTracker")) != 0;

        float32_t nccKillThreshold      = std::stof(getArgument("nccKillThreshold"));
        float32_t nccUpdateThreshold    = std::stof(getArgument("nccUpdateThreshold"));
        float32_t displacementThreshold = std::stof(getArgument("displacementThreshold"));

        float32_t largeMotionKillRatio = std::stof(getArgument("largeMotionKillRatio"));
        float32_t maxScaleChange       = std::stof(getArgument("maxScaleChange"));

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);
            CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
        }

// -----------------------------------------
// Initialize cudaStream
// -----------------------------------------
#ifndef DW_SDK_BUILD_PVA
        cudaStreamCreateWithFlags(&m_cudaStream, cudaStreamNonBlocking);
#else
#if VIBRANTE_PDK_DECIMAL < 6000400
        if (!(pvaTracker || pvaDetector || m_usePvaPyramid))
#endif
        {
            cudaStreamCreateWithFlags(&m_cudaStream, cudaStreamNonBlocking);
        }
#endif

        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            std::string file = "video=" + getArgument("video");

            dwSensorParams sensorParams{};
            sensorParams.protocol = "camera.virtual";

            sensorParams.parameters = file.c_str();
            CHECK_DW_ERROR_MSG(dwSAL_createSensor(&m_camera, sensorParams, m_sal),
                               "Cannot create virtual camera sensor, maybe wrong video file?");

            CHECK_DW_ERROR(dwSensorCamera_setCUDAStream(m_cudaStream, m_camera));
            CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&m_cameraProps, m_camera));
            printf("Camera image with %dx%d at %f FPS\n", m_cameraProps.resolution.x,
                   m_cameraProps.resolution.y, m_cameraProps.framerate);

            // we would like the application run as fast as the original video
            setProcessRate(m_cameraProps.framerate);
        }

        // ------------------------------------------------------------------
        // initialize image buffers and streamer for processing and rendering
        // ------------------------------------------------------------------
        {
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&m_cameraImageProps, DW_CAMERA_OUTPUT_CUDA_YUV420_UINT8_SEMIPLANAR, m_camera));

            // initialize image streamer
            if (getWindow())
            {
                dwImageProperties props{m_cameraImageProps};
                props.format = DW_IMAGE_FORMAT_RGBA_UINT8;
                CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_image2GL, &props, DW_IMAGE_GL, m_context));
            }

            // The useHalfRes option overrides any custom downscaling
            if (m_useHalfRes)
            {
                m_customDownscale = 2;
            }

            if (1 < m_customDownscale)
            {
                // Downscale as specified.
                // set the image property
                m_cameraImageProps.width /= m_customDownscale;
                m_cameraImageProps.height /= m_customDownscale;

                // initialize image transformation
                dwImageTransformationParameters imgTransformParams{};
                CHECK_DW_ERROR(dwImageTransformation_initialize(&m_imageTransformer, imgTransformParams, m_context));
                CHECK_DW_ERROR(dwImageTransformation_setCUDAStream(m_cudaStream, m_imageTransformer));
                CHECK_DW_ERROR(dwImageTransformation_setBorderMode(DW_IMAGEPROCESSING_BORDER_MODE_ZERO, m_imageTransformer));
                CHECK_DW_ERROR(dwImageTransformation_setInterpolationMode(DW_IMAGEPROCESSING_INTERPOLATION_DEFAULT, m_imageTransformer));
                CHECK_DW_ERROR(dwImage_create(&m_processImageDownscale, m_cameraImageProps, m_context));
            }

            // terminal output to confirm the processing image size
            printf("Processing image with %dx%d\n", m_cameraImageProps.width, m_cameraImageProps.height);
        }

        // -------------------------------
        // Initialize Renderer and Buffers
        // -------------------------------
        if (getWindow())
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

            // Init render engine with default params
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
            CHECK_DW_ERROR(dwRenderer_setRect(rect, m_renderer));

            // Init image buffer
            dwRenderBufferVertexLayout layout;
            layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
            layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
            layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
            layout.colFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
            layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
            layout.texFormat   = DW_RENDER_FORMAT_NULL;

            CHECK_DW_ERROR(dwRenderBuffer_initialize(&m_featureRenderBuffer, layout,
                                                     DW_RENDER_PRIM_POINTLIST,
                                                     maxFeatureCount, m_viz));
            CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors((float)m_cameraImageProps.width,
                                                                         (float)m_cameraImageProps.height,
                                                                         m_featureRenderBuffer));

            CHECK_DW_ERROR(dwRenderBuffer_initialize(&m_featureHistoryRenderBuffer, layout,
                                                     DW_RENDER_PRIM_LINELIST,
                                                     maxFeatureCount * 2 * historyCapacity,
                                                     m_viz));
            CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors((float)m_cameraImageProps.width,
                                                                         (float)m_cameraImageProps.height,
                                                                         m_featureHistoryRenderBuffer));
        }

        // -----------------------------
        // Initialize feature tracker
        // -----------------------------
        {
            CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&featureHistoryCPU, maxFeatureCount,
                                                           historyCapacity, DW_MEMORY_TYPE_CPU,
                                                           m_cudaStream, m_context));

            CHECK_DW_ERROR(dwFeatureHistoryArray_createNew(&featureHistoryGPU, maxFeatureCount,
                                                           historyCapacity, DW_MEMORY_TYPE_CUDA,
                                                           m_cudaStream, m_context));

            CHECK_DW_ERROR(dwFeatureArray_createNew(&featuresDetected, maxFeatureCount,
                                                    DW_MEMORY_TYPE_CUDA, m_cudaStream, m_context));

            uint32_t pvaEngineCount               = 1;
            dwProcessorType detectorProcessorType = DW_PROCESSOR_TYPE_GPU;
            if (pvaDetector)
            {
                if (pvaDetectorEngineNo >= pvaEngineCount)
                {
                    std::string errorMsg = "pvaDetectorEngineNo exceeds the number of PVAs "
                                           "available on the platform. Number of available PVAs is ";
                    errorMsg += std::to_string(pvaEngineCount) + ".";
                    throw std::runtime_error(errorMsg);
                }
                detectorProcessorType = static_cast<dwProcessorType>(static_cast<uint32_t>(DW_PROCESSOR_TYPE_PVA_0) +
                                                                     pvaDetectorEngineNo);
            }
            dwProcessorType trackerProcessorType = DW_PROCESSOR_TYPE_GPU;
            if (pvaTracker)
            {
                if (pvaTrackerEngineNo >= pvaEngineCount)
                {
                    std::string errorMsg = "pvaTrackerEngineNo exceeds the number of PVAs "
                                           "available on the platform. Number of available PVAs is ";
                    errorMsg += std::to_string(pvaEngineCount) + ".";
                    throw std::runtime_error(errorMsg);
                }
                trackerProcessorType = static_cast<dwProcessorType>(static_cast<uint32_t>(DW_PROCESSOR_TYPE_PVA_0) +
                                                                    pvaTrackerEngineNo);
            }

            dwFeature2DDetectorConfig detectorConfig = {};
            detectorConfig.imageWidth                = m_cameraImageProps.width;
            detectorConfig.imageHeight               = m_cameraImageProps.height;
            dwFeature2DDetector_initDefaultParams(&detectorConfig);
            detectorConfig.type                       = static_cast<dwFeature2DDetectorType>(detectMode);
            detectorConfig.maxFeatureCount            = maxFeatureCount;
            detectorConfig.detectionLevel             = detectLevel;
            detectorConfig.scoreThreshold             = scoreThreshold;
            detectorConfig.detailThreshold            = detailThreshold;
            detectorConfig.numEvenDistributionPerCell = numEvenDistributionPerCell;
            detectorConfig.harrisRadius               = harrisRadius;
            detectorConfig.harrisK                    = harrisK;
            detectorConfig.blockSize                  = blockSize;
            detectorConfig.gradientSize               = gradientSize;
            detectorConfig.useHalf                    = useHalfDetector;
            detectorConfig.isMaskAdjustmentEnabled    = enableMaskAdjustment;
            detectorConfig.maskType                   = static_cast<dwFeature2DSelectionMaskType>(maskType);
            detectorConfig.processorType              = detectorProcessorType;
            // Image resolution dependent parameters
            setIfValid(detectorConfig.cellSize, "cellSize");
            setIfValid(detectorConfig.NMSRadius, "NMSRadius");

            CHECK_DW_ERROR(dwFeature2DDetector_initialize(&detector, &detectorConfig, m_cudaStream, m_context));

#ifdef DW_SDK_BUILD_PVA
#if VIBRANTE_PDK_DECIMAL >= 6000400
#ifdef DW_IS_SAFETY
            if ((cupvaStreamCreate(&m_cupvaStream, pvaEngineType::PVA_PVA0, pvaAffinityType::PVA_VPU_ANY)) != pvaError::PVA_ERROR_NONE)
            {
                printf("\n Cupva Stream create failed");
            }
#else
            if ((cupvaCudaCreateStream(&m_cupvaStream, m_cudaStream, pvaEngineType::PVA_PVA0, pvaAffinityType::PVA_VPU_ANY)) != pvaError::PVA_ERROR_NONE)
            {
                printf("\n cupva Stream create failed");
            }
#endif
#else
            if ((cupvaStreamCreate(&m_cupvaStream, cupvaEngineType_t::CUPVA_PVA0, cupvaAffinityType_t::CUPVA_VPU_ANY)) != cupvaError_t::ErrorNone)
            {
                printf("\n cupva Stream create failed");
            }
#endif
            if (pvaDetector)
            {
                CHECK_DW_ERROR(dwFeature2DDetector_setPVAStream(m_cupvaStream, detector));
            }
#endif

            dwFeature2DTrackerConfig trackerConfig = {};
            trackerConfig.imageWidth               = m_cameraImageProps.width;
            trackerConfig.imageHeight              = m_cameraImageProps.height;
            dwFeature2DTracker_initDefaultParams(&trackerConfig);
            trackerConfig.algorithm                  = static_cast<dwFeature2DTrackerAlgorithm>(trackMode);
            trackerConfig.detectorType               = detectorConfig.type;
            trackerConfig.maxFeatureCount            = maxFeatureCount;
            trackerConfig.historyCapacity            = historyCapacity;
            trackerConfig.windowSizeLK               = windowSize;
            trackerConfig.enableAdaptiveWindowSizeLK = enableAdaptiveWindowSize;
            trackerConfig.numIterTranslationOnly     = numIterTranslation;
            trackerConfig.numIterScaling             = numIterScaling;
            trackerConfig.numLevelTranslationOnly    = numTranslationOnlyLevel;
            trackerConfig.nccUpdateThreshold         = nccUpdateThreshold;
            trackerConfig.nccKillThreshold           = nccKillThreshold;
            trackerConfig.displacementThreshold      = displacementThreshold;
            trackerConfig.useHalf                    = useHalfTracker;
            trackerConfig.processorType              = trackerProcessorType;
            trackerConfig.largeMotionKillRatio       = largeMotionKillRatio;
            trackerConfig.maxScaleChange             = maxScaleChange;
            // Image resolution dependent parameters
            setIfValid(trackerConfig.pyramidLevelCount, "pyramidLevel");
            // WAR Only detector Ex uses ncc, ncc is not being compacted right now
            // Need to enable sparse output when using ncc
            trackerConfig.enableSparseOutput = (detectorConfig.type == DW_FEATURE2D_DETECTOR_TYPE_EX) ? 1 : 0;

            CHECK_DW_ERROR(dwFeature2DTracker_initialize(&tracker, &trackerConfig, m_cudaStream, m_context));
#ifdef DW_SDK_BUILD_PVA
            if (pvaTracker)
            {
                CHECK_DW_ERROR(dwFeature2DTracker_setPVAStream(m_cupvaStream, tracker));
            }
#endif
            CHECK_CUDA_ERROR(cudaMalloc(&d_nccScores, maxFeatureCount * sizeof(float32_t)));

            // Initialize pyramid
            dwTrivialDataType pyramidType = m_usePvaPyramid ? DW_TYPE_UINT8 : DW_TYPE_FLOAT32;
            CHECK_DW_ERROR(dwPyramid_create(
                &pyramidPrevious, trackerConfig.pyramidLevelCount, m_cameraImageProps.width,
                m_cameraImageProps.height, pyramidType, m_context));
            CHECK_DW_ERROR(dwPyramid_create(
                &pyramidCurrent, trackerConfig.pyramidLevelCount, m_cameraImageProps.width,
                m_cameraImageProps.height, pyramidType, m_context));

            if (m_usePvaPyramid)
            {
#ifdef DW_SDK_BUILD_PVA
                dwPyramidPVAParams imagePyramidParams{};
                CHECK_DW_ERROR(dwPyramidPVA_initDefaultParams(&imagePyramidParams));

                if (trackerConfig.pyramidLevelCount < 3 || trackerConfig.pyramidLevelCount > 6)
                {
                    throw std::runtime_error("PVA pyramid supports level 3 to 6 only.");
                }

                if (pvaPyramidEngineNo != 0)
                {
                    throw std::runtime_error("Unexpected PVA engine, PVA engine must be 0.");
                }

                imagePyramidParams.processorType = static_cast<dwProcessorType>(static_cast<uint32_t>(DW_PROCESSOR_TYPE_PVA_0) + pvaPyramidEngineNo);
                imagePyramidParams.vpuIndex      = 0;
                imagePyramidParams.imageWidth    = m_cameraImageProps.width;
                imagePyramidParams.imageHeight   = m_cameraImageProps.height;
                imagePyramidParams.levelCount    = trackerConfig.pyramidLevelCount;

                CHECK_DW_ERROR(dwPyramidPVA_initialize(&m_pyramidPVA,
                                                       &imagePyramidParams,
                                                       &pyramidPrevious,
                                                       m_cudaStream,
                                                       m_context));

                CHECK_DW_ERROR(dwPyramidPVA_setPVAStream(m_cupvaStream, m_pyramidPVA));
#else
                (void)pvaPyramidEngineNo;
                throw std::runtime_error("PVA pyramid is not supported for this platform.");
#endif
            }
        }

        // -----------------------------
        // Set stop frame
        // -----------------------------
        {
            m_stopFrameIdx = static_cast<uint32_t>(std::stoi(getArgument("stopFrameIdx")));
        }

        // -----------------------------
        // Screen capture module
        // -----------------------------
        if (m_enableScreenCapture)
        {
            m_screenCaptureStartFrame = std::stoi(getArgument("capture-start-frame"));
            m_screenCaptureEndFrame   = std::stoi(getArgument("capture-end-frame"));
            dwFrameCaptureParams params{};
            std::string paramStr = "type=disk";
            paramStr += ",format=h264";
            paramStr += ",bitrate=20000000";
            paramStr += ",framerate=" + getArgument("capture-fps");
            paramStr += ",file=" + getArgument("capture-file");
            params.params.parameters = paramStr.c_str();
            params.width             = getWindowWidth();
            params.height            = getWindowHeight();
#if VIBRANTE_PDK_DECIMAL < 6000400
            params.serializeGL = true;
            params.mode        = DW_FRAMECAPTURE_MODE_SCREENCAP | DW_FRAMECAPTURE_MODE_SERIALIZE;
#else
            params.mode = DW_FRAMECAPTURE_MODE_SERIALIZE;
#endif
            CHECK_DW_ERROR(dwFrameCapture_initialize(&m_screenCapture, &params, m_sal, m_context));
            m_screenCaptureFrameCount = 0;
        }

        m_screenshot = std::make_unique<ScreenshotHelper>(m_context, m_sal, getWindowWidth(), getWindowHeight(), "CameraFeatureTracker");

        // Start Sensors
        CHECK_DW_ERROR(dwSensor_start(m_camera));

        return true;
    }

    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks(dwContextHandle_t& context) const
    {
        // initialize logger to print verbose message on console in color
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

        // initialize SDK context, using data folder
        dwContextParameters sdkParams = {};

#ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
        sdkParams.enablePVA  = true;
#endif

        CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
    }

    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        m_curFrameIdx = 0;

        if (m_frame)
        {
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));
        }

        CHECK_DW_ERROR(dwSensor_reset(m_camera));

        if (m_usePvaPyramid)
        {
#ifdef DW_SDK_BUILD_PVA
            CHECK_DW_ERROR(dwPyramidPVA_reset(m_pyramidPVA));
#endif
        }
        CHECK_DW_ERROR(dwFeatureHistoryArray_reset(&featureHistoryGPU, m_cudaStream));
        CHECK_DW_ERROR(dwFeatureArray_reset(&featuresDetected, m_cudaStream));
        CHECK_DW_ERROR(dwFeature2DDetector_reset(detector));
        CHECK_DW_ERROR(dwFeature2DTracker_reset(tracker));
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if (m_frame)
        {
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));
        }

        // stop sensor
        CHECK_DW_ERROR(dwSensor_stop(m_camera));

        // release screenshot taker and video capturer
        m_screenshot.reset();
        if (m_screenCapture)
        {
            dwFrameCapture_release(m_screenCapture);
        }

        // release feature tracker
        {
            CHECK_DW_ERROR(dwPyramid_destroy(pyramidPrevious));
            CHECK_DW_ERROR(dwPyramid_destroy(pyramidCurrent));
#ifdef DW_SDK_BUILD_PVA
            if (m_pyramidPVA)
            {
                CHECK_DW_ERROR(dwPyramidPVA_release(m_pyramidPVA));
            }
#endif
            CHECK_DW_ERROR(dwFeatureHistoryArray_destroy(featureHistoryCPU));
            CHECK_DW_ERROR(dwFeatureHistoryArray_destroy(featureHistoryGPU));
            CHECK_DW_ERROR(dwFeatureArray_destroy(featuresDetected));

            CHECK_DW_ERROR(dwFeature2DDetector_release(detector));
            CHECK_DW_ERROR(dwFeature2DTracker_release(tracker));

            CHECK_CUDA_ERROR(cudaFree(d_nccScores));
        }

        // release renderer and streamer
        if (getWindow())
        {
            CHECK_DW_ERROR(dwRenderBuffer_release(m_featureRenderBuffer));
            CHECK_DW_ERROR(dwRenderBuffer_release(m_featureHistoryRenderBuffer));

            CHECK_DW_ERROR(dwImageStreamerGL_release(m_image2GL));

            CHECK_DW_ERROR(dwRenderer_release(m_renderer));
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
            CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        }

        // release image buffer and transfomer
        if (m_processImageDownscale)
        {
            CHECK_DW_ERROR(dwImage_destroy(m_processImageDownscale));
        }
        if (m_imageTransformer)
        {
            CHECK_DW_ERROR(dwImageTransformation_release(m_imageTransformer));
        }

        // release sensor
        CHECK_DW_ERROR(dwSAL_releaseSensor(m_camera));

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        {
            CHECK_DW_ERROR(dwSAL_release(m_sal));
            CHECK_DW_ERROR(dwRelease(m_context));
            CHECK_DW_ERROR(dwLogger_release());
        }

        // release CUDA stream
        if (m_cudaStream)
        {
            CHECK_CUDA_ERROR(cudaStreamDestroy(m_cudaStream));
        }
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        CHECK_DW_ERROR(dwRenderer_setRect(rect, m_renderer));
    }

    void onKeyDown(int key, int scancode, int mods) override
    {
        (void)scancode;
        (void)mods;

        if (key == GLFW_KEY_S)
        {
            m_screenshot->triggerScreenshot();
        }

        if (key == GLFW_KEY_H)
            drawHistory = !drawHistory;
    }

    void onProcess() override
    {
        // return the previous frame to camera
        if (m_frame)
        {
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));
        }

        // ---------------------------
        // grab frame from camera
        // ---------------------------
        dwStatus status;
        do
        {
            status = dwSensorCamera_readFrame(&m_frame, 600000, m_camera);
            switch (status)
            {
            case DW_SUCCESS:
            case DW_TIME_OUT:
            case DW_NOT_READY:
                break;
            case DW_END_OF_STREAM:
                log("Video reached end of stream.\n");
                if (m_loop)
                {
                    onReset();
                    break;
                }
                else
                {
                    stop();
                    return;
                }
            default:
                CHECK_DW_ERROR(status);
            }
        } while (status != DW_SUCCESS);

        // Copy raw frame to process image
        CHECK_DW_ERROR(dwSensorCamera_getImage(&m_imageYUV, DW_CAMERA_OUTPUT_CUDA_YUV420_UINT8_SEMIPLANAR, m_frame));
        CHECK_DW_ERROR(dwSensorCamera_getImage(&m_imageRGBA, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_frame));

        dwImageHandle_t trackImage = DW_NULL_HANDLE;
        if (1 < m_customDownscale)
        {
            CHECK_DW_ERROR(dwImageTransformation_copyFullImage(m_processImageDownscale, m_imageYUV, m_imageTransformer));
            trackImage = m_processImageDownscale;
        }
        else
        {
            trackImage = m_imageYUV;
        }

        // ---------------------------
        // track the features in the frame
        // ---------------------------
        trackFrame(trackImage);

        // Increment current frame index and complete comparison for potential early termination
        ++m_curFrameIdx;
        if (m_stopFrameIdx != 0 && m_curFrameIdx >= m_stopFrameIdx)
        {
            stop();
        }

        // Trigger screen capture
        if (m_enableScreenCapture)
        {
            if (m_screenCapture != DW_NULL_HANDLE &&
                m_screenCaptureFrameCount >= m_screenCaptureStartFrame &&
                (m_screenCaptureFrameCount < m_screenCaptureEndFrame || m_screenCaptureEndFrame == -1))
            {
                m_shouldCaptureScreen = true;
            }

            m_screenCaptureFrameCount++;
        }
    }

    ///------------------------------------------------------------------------------
    /// Rendering
    ///     - push the RGBA cuda image through the streamer to convert it into GL
    ///     - render frame on screen
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        if (m_imageRGBA)
        {
            // stream the camera frame
            dwImageHandle_t frameGL{};
            dwImageGL* imageGL{nullptr};
            CHECK_DW_ERROR(dwImageStreamerGL_producerSend(m_imageRGBA, m_image2GL));
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&frameGL, 33000, m_image2GL));
            CHECK_DW_ERROR(dwImage_getGL(&imageGL, frameGL));

            char stime[64];

            CHECK_DW_ERROR(dwRenderer_renderTexture(imageGL->tex, imageGL->target, m_renderer));
            CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer));
            CHECK_DW_ERROR(dwRenderer_renderText(10, 10, stime, m_renderer));

            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&frameGL, m_image2GL));
            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 33000, m_image2GL));

            ///////////////////
            // Draw features
            uint32_t drawCount = 0;
            uint32_t maxVerts, stride;
            struct
            {
                float pos[2];
                float color[3];
            } * map;

            if (drawHistory)
            {
                CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer));
                CHECK_DW_ERROR(dwRenderer_setLineWidth(1, m_renderer));

                dwRenderBuffer_map((float**)&map, &maxVerts, &stride, m_featureHistoryRenderBuffer);

                uint32_t validFeatureCount = 0;
                for (uint32_t i = 0; i < *(featureHistoryCPU.featureCount); i++)
                {
                    if (featureHistoryCPU.statuses[i] == DW_FEATURE2D_STATUS_INVALID)
                    {
                        continue;
                    }

                    ++validFeatureCount;
                    uint32_t age = featureHistoryCPU.ages[i];

                    dwVector4f color = getFeatureRenderingColor(age);

                    // age is not capped by historyCapacity, so this operation is necessary when accessing locationHistroy.
                    const uint32_t drawAge = std::min(age, historyCapacity);

                    dwFeatureArray preFeature{};
                    CHECK_DW_ERROR(dwFeatureHistoryArray_get(&preFeature, drawAge - 1, &featureHistoryCPU));

                    for (int32_t histIdx = static_cast<int32_t>(drawAge) - 2; histIdx >= 0; histIdx--)
                    {
                        map[drawCount].pos[0]   = preFeature.locations[i].x;
                        map[drawCount].pos[1]   = preFeature.locations[i].y;
                        map[drawCount].color[0] = color.x;
                        map[drawCount].color[1] = color.y;
                        map[drawCount].color[2] = color.z;
                        drawCount++;

                        dwFeatureArray curFeature{};
                        CHECK_DW_ERROR(dwFeatureHistoryArray_get(&curFeature, histIdx, &featureHistoryCPU));

                        map[drawCount].pos[0]   = curFeature.locations[i].x;
                        map[drawCount].pos[1]   = curFeature.locations[i].y;
                        map[drawCount].color[0] = color.x;
                        map[drawCount].color[1] = color.y;
                        map[drawCount].color[2] = color.z;
                        drawCount++;

                        preFeature = curFeature;
                    }
                }

                CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer));
                CHECK_DW_ERROR(dwRenderer_renderText(10, 25, stime, m_renderer));

                CHECK_DW_ERROR(dwRenderBuffer_unmap(drawCount, m_featureHistoryRenderBuffer));
                CHECK_DW_ERROR(dwRenderer_renderBuffer(m_featureHistoryRenderBuffer, m_renderer));
            }
            else
            {
                CHECK_DW_ERROR(dwRenderer_setPointSize(4.0f, m_renderer));

                dwFeatureArray curFeature{};
                CHECK_DW_ERROR(dwFeatureHistoryArray_getCurrent(&curFeature, &featureHistoryCPU));

                CHECK_DW_ERROR(dwRenderBuffer_map((float**)&map, &maxVerts, &stride, m_featureRenderBuffer));

                if (stride != sizeof(*map) / sizeof(float))
                    throw std::runtime_error("Unexpected stride");

                for (uint32_t i = 0; i < *(curFeature.featureCount); i++)
                {
                    dwVector4f color = getFeatureRenderingColor(curFeature.ages[i]);

                    map[drawCount].pos[0]   = curFeature.locations[i].x;
                    map[drawCount].pos[1]   = curFeature.locations[i].y;
                    map[drawCount].color[0] = color.x;
                    map[drawCount].color[1] = color.y;
                    map[drawCount].color[2] = color.z;
                    drawCount++;
                }

                CHECK_DW_ERROR(dwRenderBuffer_unmap(drawCount, m_featureRenderBuffer));
                CHECK_DW_ERROR(dwRenderer_renderBuffer(m_featureRenderBuffer, m_renderer));
            }
        }

        // Capture screen
        if (m_shouldCaptureScreen)
        {
            dwRect roi{0, 0, getWindowWidth(), getWindowHeight()};
            const dwImageGL* imageGL{nullptr};
            CHECK_DW_ERROR(dwFrameCapture_screenCapture(&imageGL, roi, m_screenCapture));
#if VIBRANTE_PDK_DECIMAL < 6000400
            CHECK_DW_ERROR(dwFrameCapture_appendFrameGL(imageGL, m_screenCapture));
#endif
            m_shouldCaptureScreen = false;
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());

        // screenshot if required
        m_screenshot->processScreenshotTrig();
    }

    void trackFrame(dwImageHandle_t image)
    {
        ProfileCUDASection s(getProfilerCUDA(), "trackFrame");

        std::swap(pyramidCurrent, pyramidPrevious);
        dwFeatureArray featurePredicted{};

        {
            ProfileCUDASection s(getProfilerCUDA(), "computePyramid");
            if (m_usePvaPyramid)
            {
#ifdef DW_SDK_BUILD_PVA
                CHECK_DW_ERROR(dwPyramidPVA_computePyramid(&pyramidCurrent,
                                                           image,
                                                           m_cudaStream,
                                                           m_pyramidPVA));
#endif
            }
            else
            {
                dwImageCUDA* imageCUDA = nullptr;
                CHECK_DW_ERROR(dwImage_getCUDA(&imageCUDA, image));
                CHECK_CUDA_ERROR(dwImageFilter_computePyramid(&pyramidCurrent, imageCUDA,
                                                              m_cudaStream, m_context));
            }
        }

        {
            ProfileCUDASection s(getProfilerCUDA(), "trackCall");
            CHECK_CUDA_ERROR(dwFeature2DTracker_trackFeatures(
                &featureHistoryGPU, &featurePredicted, d_nccScores,
                &featuresDetected, nullptr, &pyramidPrevious, &pyramidCurrent, tracker));
        }

        {
            ProfileCUDASection s(getProfilerCUDA(), "detectNewFeatures");
            CHECK_DW_ERROR(dwFeature2DDetector_detectFromPyramid(
                &featuresDetected, &pyramidCurrent,
                &featurePredicted, d_nccScores, detector));
        }

        {
            // Get tracked feature info to CPU
            ProfileCUDASection s(getProfilerCUDA(), "downloadToCPU");
            CHECK_DW_ERROR(dwFeatureHistoryArray_copyAsync(&featureHistoryCPU, &featureHistoryGPU, m_cudaStream));
            CHECK_CUDA_ERROR(cudaStreamSynchronize(m_cudaStream));
        }
    }

    dwVector4f getFeatureRenderingColor(uint32_t age)
    {
        dwVector4f color;
        if (age < 5)
        {
            color = DW_RENDERER_COLOR_RED;
        }
        else if (age < 10)
        {
            color = DW_RENDERER_COLOR_YELLOW;
        }
        else if (age < 20)
        {
            color = DW_RENDERER_COLOR_GREEN;
        }
        else
        {
            color = DW_RENDERER_COLOR_LIGHTBLUE;
        }
        return color;
    }

    template <typename T>
    void setIfValid(T& ret, const char* argName)
    {
        const std::string valStr = getArgument(argName);
        if (valStr.empty())
        {
            return;
        }

        if (std::is_integral<T>::value)
        {
            ret = static_cast<T>(std::stoi(valStr));
        }
        else if (std::is_floating_point<T>::value)
        {
            ret = static_cast<T>(std::stof(valStr));
        }
        else
        {
            throw std::runtime_error("setIfValid: unhandled type");
        }
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str(),
                                                         "Supported file fromat: raw, lraw, H264"),
                              ProgramArguments::Option_t("maxFeatureCount", "4000",
                                                         "specifies the maximum number of features that can be stored."),
                              ProgramArguments::Option_t("historyCapacity", "60",
                                                         "specifies how many features are going to be stored"),
                              ProgramArguments::Option_t("pvaPyramid", "0",
                                                         "--pvaPyramid=0, use GPU to perform pyramid;"
                                                         "--pvaPyramid=1, use PVA to perform pyramid;"),
                              ProgramArguments::Option_t("pvaPyramidEngineNo", "0",
                                                         "--pvaPyramidEngineNo=0 processor is PVA0, valid only when --pvaPyramid=1;"
                                                         "--pvaPyramidEngineNo=1 processor is PVA1, valid only when --pvaPyramid=1;"),
                              ProgramArguments::Option_t("pyramidLevel", "",
                                                         "the number of pyramid levels while tracking; use the default value if empty."
                                                         "If PVA pyramid is enabled, only pyramidLevels 3 to 6 is supported"),
                              ProgramArguments::Option_t("detectMode", "1",
                                                         "--detectMode=0, use standard harris detector, valid only when --pvaDetector=0;"
                                                         "--detectMode=1, use extended harris detector;"),
                              ProgramArguments::Option_t("pvaDetector", "0",
                                                         "--pvaDetector=0 use GPU to perform feature detector;"
                                                         "--pvaDetector=1 use PVA to perform feature detector;"),
                              ProgramArguments::Option_t("pvaDetectorEngineNo", "0",
                                                         "--pvaDetectorEngineNo=0 processor is PVA0, valid only when --pvaDetector=1;"
                                                         "--pvaDetectorEngineNo=1 processor is PVA1, valid only when --pvaDetector=1;"),
                              ProgramArguments::Option_t("detectLevel", "1",
                                                         "pyramid level at which feature detection is performed."),
                              ProgramArguments::Option_t("cellSize", "", "the size of a cell; use the default value if empty. Valid only when --pvaDetector=0"),
                              ProgramArguments::Option_t("blockSize", "3", "Block window size used to compute the Harris Corner score, Must be 3, 5 or 7."
                                                                           "valid only when --detectMode=0 and --pvaDetector=1"),
                              ProgramArguments::Option_t("gradientSize", "3", "Gradient window size, Must be 3, 5 or 7."
                                                                              "valid only when --detectMode=0 and --pvaDetector=1"),
                              ProgramArguments::Option_t("scoreThreshold", "4e-5",
                                                         "the minimum score for which a point is classified as a feature. "
                                                         "When --detectMode=0 and --pvaDetector=1, it should be a uint value which is larger than 0."),
                              ProgramArguments::Option_t("detailThreshold", "0.0128",
                                                         "features with score > detailThreshold will be kept, valid only when --pvaDetector=0 and --detectMode=0"),
                              ProgramArguments::Option_t("numEvenDistributionPerCell", "5",
                                                         "number of features even distribution per cell, valid only when --pvaDetector=0 and --detectMode=0"),
                              ProgramArguments::Option_t("harrisK", "0.05",
                                                         "harris K during detection, valid only when --detectMode=1, --detectMode=0 and --pvaDetector=1"),
                              ProgramArguments::Option_t("harrisRadius", "1",
                                                         "harris radius, valid only when --detectMode=1"),
                              ProgramArguments::Option_t("NMSRadius", "",
                                                         "non-maximum suppression filter radius; use the default value if empty. Valid only when --detectMode=1"),
                              ProgramArguments::Option_t("maskType", "1",
                                                         "--maskType=0 provides a uniform distribution output,"
                                                         "--maskType=1 provides a gaussian distribution output,"
                                                         "valid only when --detectMode=1"),
                              ProgramArguments::Option_t("enableMaskAdjustment", "1",
                                                         "set it as 1 will update distribution mask before each detection, "
                                                         "valid only when --detectMode=1"),

                              ProgramArguments::Option_t("trackMode", "2",
                                                         "--trackMode=0, use translation-only KLT tracker or Sparse LK PVA tracker;"
                                                         "--trackMode=1, use translation-scale KLT tracker;"
                                                         "--trackMode=2, use translation-scale fast KLT tracker."),
                              ProgramArguments::Option_t("pvaTracker", "0",
                                                         "set it as 1 to use PVA for tracking algorithm."),
                              ProgramArguments::Option_t("pvaTrackerEngineNo", "0",
                                                         "which PVA engine the tracking algorithm runs on"),
                              ProgramArguments::Option_t("windowSize", "10",
                                                         "feature window size"),
                              ProgramArguments::Option_t("enableAdaptiveWindowSize", "1",
                                                         "set it as 1 to use full window size at the lowest and the highest levels, "
                                                         "and smaller window size at the rest of levels during tracking, valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("numIterTranslation", "8",
                                                         "KLT iteration number for translation-only tracking or Sparse LK PVA tracking iteration number"),
                              ProgramArguments::Option_t("numIterScaling", "10",
                                                         "KLT iteration number for translation-scaling tracking, valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("numTranslationOnlyLevel", "4",
                                                         "tracker will apply translation-only tracking on the highest numTranslationOnlyLevel "
                                                         "level images, valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("nccUpdateThreshold", "0.95",
                                                         "the minimum ncc threshold that will cause the feature tracker to update the image template for a particular feature, "
                                                         "valid only when --trackMode=1"),
                              ProgramArguments::Option_t("nccKillThreshold", "0.3",
                                                         "the minimum ncc threshold to mantain a particular feature in the tracker, "
                                                         "valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("largeMotionKillRatio", "0.33",
                                                         "Defines the ratio value that is used to determine if the feature's movement is too large. "
                                                         "Features will be killed if the motion is larger than the template size times the large motion killing threshold during tracking. "
                                                         "Use 0.33 for front cameras, and 0.5 for side cameras, valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("maxScaleChange", "1.8", "the maximum allowed scale change for the tracked points across consecutive frames. "
                                                                                  "Use 1.8 for front cameras, and 3.0 for side cameras, "
                                                                                  "valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("displacementThreshold", "0.1",
                                                         "the early stop threshold during translation-only tracking, valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("useHalfDetector", "0",
                                                         "set it as 1 to use fp16 for detection"),
                              ProgramArguments::Option_t("useHalfTracker", "0",
                                                         "set it as 1  to use fp16 for tracking, valid only when --trackMode=1 or 2"),
                              ProgramArguments::Option_t("useHalfRes", "1",
                                                         "--useHalfRes=0, use full resolution for image processing; "
                                                         "--useHalfRes=1, use half resolution for image processing"),
                              ProgramArguments::Option_t("useCustomDownscale", "1",
                                                         "--useCustomDownscale=1, no change in resolution for image processing; "
                                                         "--useCustomDownscale=n, divide by n to scale width and height (ignored if useHalfRes is also enabled)."),
                              ProgramArguments::Option_t("stopFrameIdx", "0", "Frame index to stop processing, 0 to process the whole video"),
                              ProgramArguments::Option_t("loop", "0", "loop video when reaching the end of video stream"),
                              // Screen capture
                              ProgramArguments::Option_t("capture-screen", "0",
                                                         "--capture-screen=0, disable screen capture; "
                                                         "--capture-screen=1, enable screen capture"),
                              ProgramArguments::Option_t("capture-file", "capture.mp4", "screen capture output filename"),
                              ProgramArguments::Option_t("capture-fps", "15", "screen capture framerate"),
                              ProgramArguments::Option_t("capture-start-frame", "0", "Frame index where the screen capture starts"),
                              ProgramArguments::Option_t("capture-end-frame", "-1", "Frame index where the screen capture ends, -1 to capture all frames"),
                          },
                          "Camera Tracker sample which tracks Harris features and playback the results in a GL window.");

    // -------------------
    // initialize and start a window application
    CameraFeatureTracker app(args);

    app.initializeWindow("Feature Tracker Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
