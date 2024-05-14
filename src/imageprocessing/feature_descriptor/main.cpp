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
#include <dwshared/dwfoundation/dw/cuda/misc/Checks.hpp>
#include <dw/imageprocessing/featuredetector/FeatureDetector.h>
#include <dw/imageprocessing/featuredetector/descriptor/FeatureDescriptor.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Camera Fast9 Feature Detector
// The Camera Fast9 Feature Detector sample demonstrates the feature detection capabilities
// using fast9 algorithm of the dw_imageprocessing module. It loads a video stream and reads
// the images sequentially. For each frame, it detects feature points using fast9 algorithm.

//------------------------------------------------------------------------------
class CameraORBFeatureDescriptor : public DriveWorksSample
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

    dwImageHandle_t m_processImage = DW_NULL_HANDLE;

    dwRenderBufferHandle_t m_featureRenderBuffer = DW_NULL_HANDLE;

    bool m_usePinnedMemory = false;

    dwImageHandle_t m_imageRGBA = DW_NULL_HANDLE;

    static constexpr uint32_t MAX_PYRAMID_LEVELS = 5U;

    uint32_t m_numPyramidLevels{};

    // detectors for all the levels
    dwFeature2DDetectorHandle_t m_detector[MAX_PYRAMID_LEVELS];
    cupvaStream_t m_cupvaStream;
    cudaStream_t m_stream;

    // descriptor computers for all the levels
    dwFeature2DDescriptorHandle_t m_descriptorCalc[MAX_PYRAMID_LEVELS];

    uint32_t m_maxFeatureCount{};

    uint32_t m_featCountLevel[MAX_PYRAMID_LEVELS];

    // Frame index for early stop
    uint32_t m_stopFrameIdx{};
    uint32_t m_curFrameIdx{};

    float32_t* m_dNccScores = nullptr;

    // pyramid handles
    dwPyramidImage m_pyramidCurrent;

    //These point into the buffers of featureList
    dwFeatureArray m_featuresDetectedCPU;
    dwFeatureArray m_featuresDetectedGPU[MAX_PYRAMID_LEVELS];

    dwFeatureDescriptorArray m_descriptorsGPU[MAX_PYRAMID_LEVELS];

    dwFeatureDescriptorArray m_descriptorsCPU[MAX_PYRAMID_LEVELS];

    // Screen Capture
    //------------------------------------------------------------------------------
    bool m_enableScreenCapture = std::stoi(getArgument("capture-screen"));
    dwFrameCaptureHandle_t m_screenCapture{DW_NULL_HANDLE};
    bool m_shouldCaptureScreen{false};
    int32_t m_screenCaptureStartFrame{};
    int32_t m_screenCaptureEndFrame{-1};
    int32_t m_screenCaptureFrameCount{};

public:
    CameraORBFeatureDescriptor(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    void initializeRenderer()
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
        CHECK_DW_ERROR(dwRenderer_setRect(rect, m_renderer));
    }

    void initializeSensors()
    {
        std::string file  = "video=" + getArgument("video");
        m_usePinnedMemory = getArgument("usePinnedMemory").compare("1") == 0;
        dwSensorParams sensorParams{};
        sensorParams.protocol = "camera.virtual";
        if (m_usePinnedMemory)
        {
            file += ",usePinnedMemory=1";
        }
        sensorParams.parameters = file.c_str();
        CHECK_DW_ERROR_MSG(dwSAL_createSensor(&m_camera, sensorParams, m_sal),
                           "Cannot create virtual camera sensor, maybe wrong video file?");
        CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&m_cameraProps, m_camera));
        printf("Camera image with %dx%d at %f FPS\n", m_cameraProps.resolution.x,
               m_cameraProps.resolution.y, m_cameraProps.framerate);
        // we would like the application run as fast as the original video
        setProcessRate(m_cameraProps.framerate);
    }

    void initializeRenderBuffer()
    {
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
        layout.colFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;
        CHECK_DW_ERROR(dwRenderBuffer_initialize(&m_featureRenderBuffer, layout,
                                                 DW_RENDER_PRIM_POINTLIST,
                                                 m_maxFeatureCount, m_viz));
        CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)m_cameraImageProps.width,
                                                                     (float32_t)m_cameraImageProps.height,
                                                                     m_featureRenderBuffer));
    }

    bool initializeDescriptor()
    {
        float32_t factor    = 0.5;
        m_featCountLevel[0] = static_cast<uint32_t>(m_maxFeatureCount * (1 - factor) / (1 - pow(factor, m_numPyramidLevels)));
        for (uint32_t i = 1; i < m_numPyramidLevels; i++)
        {
            m_featCountLevel[i] = m_featCountLevel[i - 1] * factor;
        }
        DW_CHECK_CUDA_ERROR(cudaStreamCreate(&m_stream));
        CHECK_DW_ERROR(dwFeatureArray_createNew(&m_featuresDetectedCPU, m_featCountLevel[0], DW_MEMORY_TYPE_CPU, m_stream, m_context));
        for (uint32_t i = 0; i < m_numPyramidLevels; i++)
        {

            CHECK_DW_ERROR(dwFeatureArray_createNew(&m_featuresDetectedGPU[i], m_featCountLevel[i],
                                                    DW_MEMORY_TYPE_CUDA, m_stream, m_context));

            CHECK_DW_ERROR(dwFeatureDescriptorArray_create(&m_descriptorsGPU[i], DW_TYPE_UINT16, 16, m_featCountLevel[i], DW_MEMORY_TYPE_CUDA, m_stream, m_context));
            CHECK_DW_ERROR(dwFeatureDescriptorArray_create(&m_descriptorsCPU[i], DW_TYPE_UINT16, 16, m_featCountLevel[i], DW_MEMORY_TYPE_CPU, m_stream, m_context));
        }
#ifdef DW_SDK_BUILD_PVA
        cupvaStreamCreate(&m_cupvaStream,
                          pvaEngineType::PVA_PVA0, pvaAffinityType::PVA_VPU_ANY);
#endif

        dwProcessorType processorType = DW_PROCESSOR_TYPE_PVA_0;
        uint32_t pvaEngineNo          = std::atoi(getArgument("pvaEngineNo").c_str());
        if (pvaEngineNo != 0)
        {
            printf("\nOnly PVA Engine 1 is supported\n");
            return false;
        }
        float32_t scoreThreshold = std::stof(getArgument("scoreThreshold"));

        dwFeature2DDetectorConfig detectorConfig = {};
        dwFeature2DDetector_initDefaultParams(&detectorConfig);
        detectorConfig.type           = DW_FEATURE2D_DETECTOR_TYPE_FAST9;
        detectorConfig.imageWidth     = m_cameraImageProps.width;
        detectorConfig.imageHeight    = m_cameraImageProps.height;
        detectorConfig.processorType  = processorType;
        detectorConfig.scoreThreshold = scoreThreshold;

        for (uint32_t i = 0; i < m_numPyramidLevels; i++)
        {
            detectorConfig.maxFeatureCount = m_featCountLevel[i];
            detectorConfig.detectionLevel  = i;
            CHECK_DW_ERROR(dwFeature2DDetector_initialize(&m_detector[i], &detectorConfig, m_stream, m_context));
            CHECK_DW_ERROR(dwFeature2DDetector_setPVAStream(m_cupvaStream, m_detector[i]));
        }

        dwFeature2DDescriptorConfig descriptorConfig = {};
        descriptorConfig.algorithm                   = DW_FEATURE2D_DESCRIPTOR_ALGORITHM_ORB;
        descriptorConfig.processorType               = processorType;

        dwTrivialDataType data_type;
        std::string dataType = getArgument("dataType");

        if (dataType == "float16")
        {
            data_type = DW_TYPE_FLOAT16;
        }
        else if (dataType == "float32")
        {
            data_type = DW_TYPE_FLOAT32;
        }
        else if (dataType == "uint16")
        {
            data_type = DW_TYPE_UINT16;
        }
        else if (dataType == "uint8")
        {
            data_type = DW_TYPE_UINT8;
        }
        else
        {
            printf("\nData type is not supported");
            return false;
        }
        CHECK_DW_ERROR(dwPyramid_create(
            &m_pyramidCurrent, m_numPyramidLevels, m_cameraImageProps.width,
            m_cameraImageProps.height, data_type, m_context));
        dwPyramidImageProperties props;
        CHECK_DW_ERROR(dwPyramid_getProperties(&props, &m_pyramidCurrent, m_context));

        for (uint32_t i = 0; i < m_numPyramidLevels; i++)
        {
            descriptorConfig.imageHeight     = props.levelProps[i].height;
            descriptorConfig.imageWidth      = props.levelProps[i].width;
            descriptorConfig.maxFeatureCount = m_featCountLevel[i];
            CHECK_DW_ERROR(dwFeature2DDescriptor_initialize(&m_descriptorCalc[i], &descriptorConfig, m_stream, m_context));
            CHECK_DW_ERROR(dwFeature2DDescriptor_setPVAStream(m_cupvaStream, m_descriptorCalc[i]));
        }

        CHECK_CUDA_ERROR(cudaMalloc(&m_dNccScores, m_maxFeatureCount * sizeof(float32_t)));
        return true;
    }

    /// -----------------------------
    /// Initialize Renderer, Sensors, Image Streamers and Detector
    /// -----------------------------
    bool onInitialize() override
    {

        // -----------------------------------------
        // Get values from command line
        // -----------------------------------------
        m_maxFeatureCount  = std::stoi(getArgument("maxFeatureCount"));
        m_numPyramidLevels = std::stoi(getArgument("pyramidLevel"));

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(m_context);
            CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        initializeRenderer();

        // -----------------------------
        // initialize sensors
        // -----------------------------
        initializeSensors();

        // -----------------------------
        // initialize streamer and software isp for raw video playback, if the video input is raw
        // -----------------------------

        dwImageProperties from{};

        {
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&from, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera));

            // set the image property
            m_cameraImageProps.width  = m_cameraProps.resolution.x;
            m_cameraImageProps.height = m_cameraProps.resolution.y;
        }

        // set the image property
        m_cameraImageProps.width  = m_cameraProps.resolution.x;
        m_cameraImageProps.height = m_cameraProps.resolution.y;

        CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_image2GL, &from, DW_IMAGE_GL, m_context));

        // Set up the image for feature detector to process
        dwImageProperties processProps = from;
        processProps.format            = DW_IMAGE_FORMAT_RGB_FLOAT16_PLANAR;
        CHECK_DW_ERROR(dwImage_create(&m_processImage, processProps, m_context));

        // -----------------------------
        // Start Sensors
        // -----------------------------
        CHECK_DW_ERROR(dwSensor_start(m_camera));

        m_screenshot.reset(new ScreenshotHelper(m_context, m_sal, getWindowWidth(), getWindowHeight(), "CameraFast9FeatureDetector"));

        // -----------------------------
        // Initialize render buffer
        // -----------------------------
        initializeRenderBuffer();

        // -----------------------------
        // Initialize feature detector
        // -----------------------------

        if (!initializeDescriptor())
        {
            return false;
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
            params.mode              = DW_FRAMECAPTURE_MODE_SCREENCAP | DW_FRAMECAPTURE_MODE_SERIALIZE;
            params.serializeGL       = true;
            CHECK_DW_ERROR(dwFrameCapture_initialize(&m_screenCapture, &params, m_sal, m_context));
            m_screenCaptureFrameCount = 0;
        }
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
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));

        CHECK_DW_ERROR(dwSensor_reset(m_camera));

        CHECK_DW_ERROR(dwFeatureArray_reset(&m_featuresDetectedCPU, cudaStream_t(0)));
        for (uint32_t i = 0; i < m_numPyramidLevels; i++)
        {
            CHECK_DW_ERROR(dwFeatureArray_reset(&m_featuresDetectedGPU[i], cudaStream_t(0)));
            CHECK_DW_ERROR(dwFeature2DDetector_reset(m_detector[i]));
        }
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if (m_frame)
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));

        // stop sensor
        CHECK_DW_ERROR(dwSensor_stop(m_camera));

        CHECK_DW_ERROR(dwImage_destroy(m_processImage));

        m_screenshot.reset();

        // release sensor
        CHECK_DW_ERROR(dwSAL_releaseSensor(m_camera));

        // release renderer and streamer
        dwRenderBuffer_release(m_featureRenderBuffer);
        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        CHECK_DW_ERROR(dwRenderer_release(m_renderer));
        CHECK_DW_ERROR(dwImageStreamerGL_release(m_image2GL));

        // release feature detector
        {
            CHECK_DW_ERROR(dwFeatureArray_destroy(m_featuresDetectedCPU));
            CHECK_DW_ERROR(dwPyramid_destroy(m_pyramidCurrent));
            for (uint32_t i = 0; i < m_numPyramidLevels; i++)
            {

                CHECK_DW_ERROR(dwFeatureArray_destroy(m_featuresDetectedGPU[i]));
                CHECK_DW_ERROR(dwFeatureDescriptorArray_destroy(&m_descriptorsGPU[i]));
                CHECK_DW_ERROR(dwFeatureDescriptorArray_destroy(&m_descriptorsCPU[i]));
                CHECK_DW_ERROR(dwFeature2DDetector_release(m_detector[i]));
                CHECK_DW_ERROR(dwFeature2DDescriptor_release(m_descriptorCalc[i]))
            }
        }

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        {
            CHECK_DW_ERROR(dwSAL_release(m_sal));
            CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
            CHECK_DW_ERROR(dwRelease(m_context));
            CHECK_DW_ERROR(dwLogger_release());
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
    }

    void onProcess() override
    {
        // return the previous frame to camera
        if (m_frame)
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));

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
                dwSensor_reset(m_camera);
                log("Video reached end of stream.\n");
                break;
            default:
                CHECK_DW_ERROR(status);
            }
        } while (status != DW_SUCCESS);

        CHECK_DW_ERROR(dwSensorCamera_getImage(&m_imageRGBA, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_frame));

        CHECK_DW_ERROR(dwImage_copyConvert(m_processImage, m_imageRGBA, m_context));

        dwImageCUDA* imageCUDA = nullptr;
        dwImageCUDA planeG;
        CHECK_DW_ERROR(dwImage_getCUDA(&imageCUDA, m_processImage));
        CHECK_DW_ERROR(dwImageCUDA_getPlaneAsImage(&planeG, imageCUDA, 1));

        // ---------------------------
        // detect the features in the frame
        // ---------------------------
        detectFrame(&planeG);

        // Increment current frame index and complete comparison for potential early termination
        ++m_curFrameIdx;
        if (m_stopFrameIdx != 0 && m_curFrameIdx >= m_stopFrameIdx)
        {
            stop();
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
            CHECK_DW_ERROR(dwImageStreamerGL_producerSend(m_imageRGBA, m_image2GL));

            dwImageHandle_t frameGL;
            CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&frameGL, 33000, m_image2GL));

            dwImageGL* imageGL = nullptr;
            CHECK_DW_ERROR(dwImage_getGL(&imageGL, frameGL));

            char stime[64];
            sprintf(stime, "Frame time: %lu [us]", imageGL->timestamp_us);

            CHECK_DW_ERROR(dwRenderer_renderTexture(imageGL->tex, imageGL->target, m_renderer));
            CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer));
            CHECK_DW_ERROR(dwRenderer_renderText(10, 10, stime, m_renderer));

            CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&frameGL, m_image2GL));
            CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 33000, m_image2GL));

            ///////////////////
            //Draw features
            uint32_t drawCount = 0;
            uint32_t maxVerts, stride;
            struct
            {
                float pos[2];
                float color[3];
            } * map;

            CHECK_DW_ERROR(dwRenderer_setPointSize(4.0f, m_renderer));

            CHECK_DW_ERROR(dwRenderBuffer_map((float**)&map, &maxVerts, &stride, m_featureRenderBuffer));

            if (stride != sizeof(*map) / sizeof(float))
                throw std::runtime_error("Unexpected stride");

            for (uint32_t i = 0; i < *(m_featuresDetectedCPU.featureCount); i++)
            {
                dwVector4f color = DW_RENDERER_COLOR_RED;

                map[drawCount].pos[0]   = m_featuresDetectedCPU.locations[i].x;
                map[drawCount].pos[1]   = m_featuresDetectedCPU.locations[i].y;
                map[drawCount].color[0] = color.x;
                map[drawCount].color[1] = color.y;
                map[drawCount].color[2] = color.z;
                drawCount++;
            }

            CHECK_DW_ERROR(dwRenderBuffer_unmap(drawCount, m_featureRenderBuffer));
            CHECK_DW_ERROR(dwRenderer_renderBuffer(m_featureRenderBuffer, m_renderer));
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());

        // screenshot if required
        m_screenshot->processScreenshotTrig();
    }

    void detectFrame(dwImageCUDA* plane)
    {
        ProfileCUDASection s(getProfilerCUDA(), "detectFrame");

        {
            ProfileCUDASection s(getProfilerCUDA(), "computePyramid");
            CHECK_CUDA_ERROR(dwImageFilter_computePyramid(&m_pyramidCurrent, plane,
                                                          0, m_context));
        }

        {

            ProfileCUDASection s(getProfilerCUDA(), "ORB feature detector and descriptor");
            for (uint32_t i = 0; i < m_numPyramidLevels; i++)
            {

                CHECK_DW_ERROR(dwFeature2DDetector_detectFromPyramid(
                    &m_featuresDetectedGPU[i], &m_pyramidCurrent,
                    &m_featuresDetectedGPU[i], m_dNccScores, m_detector[i]));

                CHECK_DW_ERROR(dwFeature2DDescriptor_bindBuffers(m_pyramidCurrent.levelImages[i], &m_featuresDetectedGPU[i], m_descriptorCalc[i]));
                CHECK_DW_ERROR(dwFeature2DDescriptor_bindOutput(&m_descriptorsGPU[i], m_descriptorCalc[i]));
                CHECK_DW_ERROR(dwFeature2DDescriptor_processImage(DW_FEATURE2D_DESCRIPTOR_STAGE_GPU_ASYNC, m_descriptorCalc[i]));
                CHECK_DW_ERROR(dwFeature2DDescriptor_processImage(DW_FEATURE2D_DESCRIPTOR_STAGE_CPU_SYNC, m_descriptorCalc[i]));
                CHECK_DW_ERROR(dwFeature2DDescriptor_processImage(DW_FEATURE2D_DESCRIPTOR_STAGE_PVA_ASYNC, m_descriptorCalc[i]));
                CHECK_DW_ERROR(dwFeature2DDescriptor_processImage(DW_FEATURE2D_DESCRIPTOR_STAGE_CPU_SYNC_POSTPROCESS, m_descriptorCalc[i]));
                CHECK_DW_ERROR(dwFeature2DDescriptor_processImage(DW_FEATURE2D_DESCRIPTOR_STAGE_GPU_ASYNC_POSTPROCESS, m_descriptorCalc[i]));
            }
        }

        {
            //Get detected feature info to CPU
            ProfileCUDASection s(getProfilerCUDA(), "downloadToCPU");
            for (uint32_t i = 0; i < m_numPyramidLevels; i++)
            {
                CHECK_DW_ERROR(dwFeatureDescriptorArray_copyAsync(&m_descriptorsCPU[i], &m_descriptorsGPU[i], m_stream));
            }
            CHECK_DW_ERROR(dwFeatureArray_copyAsync(&m_featuresDetectedCPU, &m_featuresDetectedGPU[0], m_stream));
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
                                                         "Supported file fromat: H264; "),
                              ProgramArguments::Option_t("maxFeatureCount", "1024",
                                                         "If using PVA to do fast9 feature detection, m_maxFeatureCount needs to "
                                                         "be set as no more than 1024"),
                              ProgramArguments::Option_t("pyramidLevel", "3",
                                                         "Number of pyramid levels"),
                              ProgramArguments::Option_t("pvaEngineNo", "0",
                                                         "--pvaEngineNo=0 processor is PVA0;"),
                              ProgramArguments::Option_t("scoreThreshold", "5.0"),
                              ProgramArguments::Option_t("stopFrameIdx", "0", "Frame index to stop processing, 0 to process the whole video"),
                              ProgramArguments::Option_t("dataType", "float32", "Data type for frame, can be float16, float32, uint16 or uint8"),
                              // Screen capture
                              ProgramArguments::Option_t("capture-screen", "0",
                                                         "--capture-screen=0, disable screen capture; "
                                                         "--capture-screen=1, enable screen capture"),
                              ProgramArguments::Option_t("capture-file", "capture.mp4", "screen capture output filename"),
                              ProgramArguments::Option_t("capture-fps", "15", "screen capture framerate"),
                              ProgramArguments::Option_t("capture-start-frame", "0", "Frame index where the screen capture starts"),
                              ProgramArguments::Option_t("capture-end-frame", "-1", "Frame index where the screen capture ends, -1 to capture all frames"),
                          },
                          "Camera Orb Descriptor sample which detects features and playback the results in a GL window.");
    // -------------------
    // initialize and start a window application
    CameraORBFeatureDescriptor app(args);

    app.initializeWindow("Orb Descriptor Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
