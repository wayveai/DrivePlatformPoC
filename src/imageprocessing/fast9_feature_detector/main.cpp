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

#include <dw/imageprocessing/featuredetector/FeatureDetector.h>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Camera Fast9 Feature Detector
// The Camera Fast9 Feature Detector sample demonstrates the feature detection capabilities
// using fast9 algorithm of the dw_imageprocessing module. It loads a video stream and reads
// the images sequentially. For each frame, it detects feature points using fast9 algorithm.

//------------------------------------------------------------------------------
class CameraFast9FeatureDetector : public DriveWorksSample
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

    dwRenderBufferHandle_t m_featureRenderBuffer;

    bool m_usePinnedMemory = false;

    dwImageHandle_t m_imageRGBA = DW_NULL_HANDLE;

    dwFeature2DDetectorHandle_t detector;
    uint32_t maxFeatureCount;

    // pyramid handles
    dwPyramidImage pyramidCurrent;

    //These point into the buffers of featureList
    dwFeatureArray featuresDetectedCPU;
    dwFeatureArray featuresDetectedGPU;

public:
    CameraFast9FeatureDetector(const ProgramArguments& args)
        : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize Renderer, Sensors, Image Streamers and Detector
    /// -----------------------------
    bool onInitialize() override
    {

        // -----------------------------------------
        // Get values from command line
        // -----------------------------------------
        maxFeatureCount = std::stoi(getArgument("maxFeatureCount"));

        uint32_t nmsRadius       = std::stoi(getArgument("NMSRadius"));
        float32_t scoreThreshold = std::stof(getArgument("scoreThreshold"));

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

        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            std::string file = "video=" + getArgument("video");

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
                                                     maxFeatureCount, m_viz));
            CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors((float)m_cameraImageProps.width,
                                                                         (float)m_cameraImageProps.height,
                                                                         m_featureRenderBuffer));
        }

        // -----------------------------
        // Initialize feature detector
        // -----------------------------
        {
            CHECK_DW_ERROR(dwFeatureArray_createNew(&featuresDetectedCPU, maxFeatureCount,
                                                    DW_MEMORY_TYPE_CPU, nullptr, m_context));
            CHECK_DW_ERROR(dwFeatureArray_createNew(&featuresDetectedGPU, maxFeatureCount,
                                                    DW_MEMORY_TYPE_CUDA, nullptr, m_context));

            dwProcessorType detectorProcessorType    = DW_PROCESSOR_TYPE_GPU;
            dwFeature2DDetectorConfig detectorConfig = {};
            dwFeature2DDetector_initDefaultParams(&detectorConfig);
            detectorConfig.type            = DW_FEATURE2D_DETECTOR_TYPE_FAST9;
            detectorConfig.imageWidth      = m_cameraImageProps.width;
            detectorConfig.imageHeight     = m_cameraImageProps.height;
            detectorConfig.maxFeatureCount = maxFeatureCount;
            detectorConfig.scoreThreshold  = scoreThreshold;
            detectorConfig.NMSRadius       = nmsRadius;
            detectorConfig.processorType   = detectorProcessorType;

            CHECK_DW_ERROR(dwFeature2DDetector_initialize(&detector, &detectorConfig, 0, m_context));

            CHECK_DW_ERROR(dwPyramid_create(
                &pyramidCurrent, 1u, m_cameraImageProps.width,
                m_cameraImageProps.height, DW_TYPE_FLOAT32, m_context));
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

        if (m_frame)
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));

        CHECK_DW_ERROR(dwSensor_reset(m_camera));

        CHECK_DW_ERROR(dwFeatureArray_reset(&featuresDetectedCPU, cudaStream_t(0)));
        CHECK_DW_ERROR(dwFeatureArray_reset(&featuresDetectedGPU, cudaStream_t(0)));
        CHECK_DW_ERROR(dwFeature2DDetector_reset(detector));
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
            CHECK_DW_ERROR(dwPyramid_destroy(pyramidCurrent));
            CHECK_DW_ERROR(dwFeatureArray_destroy(featuresDetectedCPU));
            CHECK_DW_ERROR(dwFeatureArray_destroy(featuresDetectedGPU));
            CHECK_DW_ERROR(dwFeature2DDetector_release(detector));
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
        CHECK_DW_ERROR(dwImage_getCUDA(&imageCUDA, m_processImage));

        // ---------------------------
        // detect the features in the frame
        // ---------------------------
        detectFrame(imageCUDA);
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

            for (uint32_t i = 0; i < *(featuresDetectedCPU.featureCount); i++)
            {
                dwVector4f color = DW_RENDERER_COLOR_RED;

                map[drawCount].pos[0]   = featuresDetectedCPU.locations[i].x;
                map[drawCount].pos[1]   = featuresDetectedCPU.locations[i].y;
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
            CHECK_CUDA_ERROR(dwImageFilter_computePyramid(&pyramidCurrent, plane,
                                                          0, m_context));
        }

        {
            ProfileCUDASection s(getProfilerCUDA(), "detectNewFeatures");
            CHECK_DW_ERROR(dwFeature2DDetector_detectFromImage(
                &featuresDetectedGPU, pyramidCurrent.levelImages[0],
                &featuresDetectedGPU, nullptr, detector));
        }

        {
            //Get detected feature info to CPU
            ProfileCUDASection s(getProfilerCUDA(), "downloadToCPU");
            CHECK_DW_ERROR(dwFeatureArray_copyAsync(&featuresDetectedCPU, &featuresDetectedGPU, 0));
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
                                                         "Supported file fromat: raw, lraw, H264; "
                                                         "Add ', disable-isp=1' for 8MP camera"),
                              ProgramArguments::Option_t("maxFeatureCount", "4096"),
                              ProgramArguments::Option_t("scoreThreshold", "56"),
                              ProgramArguments::Option_t("NMSRadius", "0"),
                              ProgramArguments::Option_t("usePinnedMemory", "0"),
                          },
                          "Camera Fast9 Feature Detector sample which detects features and playback the results in a GL window.");

    // -------------------
    // initialize and start a window application
    CameraFast9FeatureDetector app(args);

    app.initializeWindow("Fast9 Feature Detector Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
