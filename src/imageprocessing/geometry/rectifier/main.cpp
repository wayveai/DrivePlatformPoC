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

// Driveworks
#include <dw/core/base/Version.h>
#include <dw/calibration/cameramodel/CameraModel.h>
#include <dw/imageprocessing/geometry/rectifier/Rectifier.h>

// Sample framework
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SimpleRenderer.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/MathUtils.hpp>
#include <framework/WindowGLFW.hpp>

// IMAGE
#include <dwvisualization/image/FrameCapture.h>

/**
 * Class that holds functions and variables common to all samples
 */
using namespace dw_samples::common;

class RectifierApp : public DriveWorksSample
{
public:
    // ------------------------------------------------
    // Sample constants
    // ------------------------------------------------
    static const uint32_t NUM_CAMERAS  = 4;
    static const uint32_t FRAME_WIDTH  = 1280;
    static const uint32_t FRAME_HEIGHT = 800;
    static const uint32_t FPS          = 30;

    // ------------------------------------------------
    // Driveworks context and modules
    // ------------------------------------------------
    dwContextHandle_t m_context               = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz      = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine     = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                       = DW_NULL_HANDLE;
    dwSensorHandle_t m_cameraSensor           = DW_NULL_HANDLE;
    dwCameraModelHandle_t m_cameraModelIn     = DW_NULL_HANDLE;
    dwCameraModelHandle_t m_cameraModelOut    = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_streamerCUDA2GL = DW_NULL_HANDLE;
    dwRectifierHandle_t m_rectifier           = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer             = DW_NULL_HANDLE;

    dwFrameCaptureHandle_t m_frameCap = DW_NULL_HANDLE;
    // ------------------------------------------------
    // Variables
    // ------------------------------------------------
    dwImageHandle_t m_rectifiedImage{};
    uint32_t m_cameraWidth   = 0U;
    uint32_t m_cameraHeight  = 0U;
    uint32_t m_cameraCount   = 0U;
    float32_t m_fovX         = 0.0f;
    float32_t m_fovY         = 0.0f;
    const char* m_cameraName = nullptr;

    std::string m_rigConfigFilename;
    std::string m_videoFilename;

    std::unique_ptr<SimpleCamera> m_camera;
    dwImageHandle_t m_image;
    dwCameraFrameHandle_t m_frame = DW_NULL_HANDLE;

    dwMatrix3f m_homography;
    dwVector3f m_translate{};
    dwTransformation3f m_transformation;

    uint32_t m_tile[2];
    dwVector2f* cpuDistMap;
    bool m_renderDistortion = true;
    dwVector4f colors[10]   = {DW_RENDERER_COLOR_YELLOW,
                             DW_RENDERER_COLOR_BLUE,
                             DW_RENDERER_COLOR_LIGHTPURPLE,
                             DW_RENDERER_COLOR_DARKRED,
                             DW_RENDERER_COLOR_LIGHTGREY,
                             DW_RENDERER_COLOR_LIGHTBLUE,
                             DW_RENDERER_COLOR_GREEN,
                             DW_RENDERER_COLOR_WHITE,
                             DW_RENDERER_COLOR_ORANGE,
                             DW_RENDERER_COLOR_RED};

    struct ColoredPoint2f
    {
        dwVector2f pos;
        dwRenderEngineColorRGBA color;
    };
    uint32_t m_bufferVertex;
    ColoredPoint2f* m_vertexDist;
    ColoredPoint2f* m_vertexUndist;

    uint32_t m_lineCount;
    const uint32_t m_renderStep = 30;

    // ------------------------------------------------
    // Sample constructor and methods
    // ------------------------------------------------
    RectifierApp(const ProgramArguments& args);

    // Sample framework
    bool onInitialize() override final;
    void onProcess() override final;
    void onRender() override final;
    void onKeyDown(int key, int scancode, int mods) override final;
    void onReset() override final;
    void onRelease() override final;

    // Sample initialization
    void initializeDriveWorks(dwContextHandle_t& context) const;
    bool initDriveworks();
    bool initCameras();
    bool initImageStreamer();
    bool initRenderer();
    bool initRectifier();
    bool createVideoReplay(dwSensorHandle_t* salSensor,
                           uint32_t* cameraWidth,
                           uint32_t* cameraHeight,
                           uint32_t* cameraSiblings,
                           float32_t* cameraFrameRate,
                           dwImageType* imageType,
                           dwSALHandle_t sal,
                           const std::string& videoFName);
    void updateHomography();
    void setRendererRect(int x, int y);

    void computeRenderGrid(dwVector2f* distortionPoints, uint32_t width, uint32_t height, uint32_t step);
    void renderDistortionGrid(ColoredPoint2f* buffer);
};

//#######################################################################################
RectifierApp::RectifierApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
    m_rigConfigFilename = args.get("rig");
    m_videoFilename     = getArgs().get("video");
    m_fovX              = DEG2RAD(atof(args.get("fovX").c_str()));
    m_fovY              = DEG2RAD(atof(args.get("fovY").c_str()));
    m_cameraName        = args.get("camera-name").c_str();
}

//#######################################################################################
bool RectifierApp::onInitialize()
{

    if (!initDriveworks())
    {
        logError("DriveWorks initialization failed");
        return false;
    }

    if (!initCameras())
    {
        logError("Camera initialization failed");
        return false;
    }

    if (!initImageStreamer())
    {
        logError("ImageStreamer initialization failed");
        return false;
    }

    if (!initRectifier())
    {
        logError("Rectifier initialization failed");
        return false;
    }

    if (!initRenderer())
    {
        logError("Renderer initialization failed");
        return false;
    }

    dwFrameCaptureParams frameParams{};
    std::string params = "type=disk,format=h264,bitrate=10000000, framerate=30";
    params += ",file=" + getArgument("record-video");
    frameParams.params.parameters = params.c_str();
    frameParams.width             = m_cameraWidth;
    frameParams.height            = m_cameraHeight;
    frameParams.mode              = DW_FRAMECAPTURE_MODE_SERIALIZE;

    if (!getArgument("record-video").empty())
        CHECK_DW_ERROR(dwFrameCapture_initialize(&m_frameCap, &frameParams, m_sal, m_context));

    // Limit FPS
    DriveWorksSample::setProcessRate(FPS);

    return true;
}

//#######################################################################################
void RectifierApp::onProcess()
{
    // Release frame
    if (m_frame)
    {
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));
    }

    // ---------------------------
    // grab frame from camera
    // ---------------------------
    uint32_t countFailure = 0;
    dwStatus status       = DW_NOT_READY;

    while ((status == DW_NOT_READY) || (status == DW_END_OF_STREAM) || (status == DW_TIME_OUT))
    {
        status = dwSensorCamera_readFrame(&m_frame, 600000, m_cameraSensor);
        countFailure++;
        if (countFailure == 1000000)
        {
            std::cout << "Camera virtual doesn't seem responsive, exit loop and stopping the sample" << std::endl;
            stop();
            return;
        }

        if (status == DW_END_OF_STREAM)
        {
            std::cout << "Video reached end of stream" << std::endl;
            CHECK_DW_ERROR(dwSensor_reset(m_cameraSensor));
        }
        else if ((status != DW_TIME_OUT) && (status != DW_NOT_READY))
        {
            CHECK_DW_ERROR(status);
        }
    }

    // get imageCUDA from the frame
    CHECK_DW_ERROR(dwSensorCamera_getImage(&m_image, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_frame));

    if (m_image != nullptr)
    {
        // Get image
        dwImageCUDA* rgbaImage;
        dwImage_getCUDA(&rgbaImage, m_image);

        // Rectify image
        dwImageCUDA* rectifiedImage;
        dwImage_getCUDA(&rectifiedImage, m_rectifiedImage);
        CHECK_DW_ERROR(dwRectifier_warp(rectifiedImage, rgbaImage, true, m_rectifier));

        if (m_frameCap)
            CHECK_DW_ERROR(dwFrameCapture_appendFrame(m_rectifiedImage, m_frameCap));
    }
    else
    {
        log("Camera reached end of stream\n");
        DriveWorksSample::reset();
    }
}

void RectifierApp::computeRenderGrid(dwVector2f* distortionPoints, uint32_t width, uint32_t height, uint32_t step)
{
    uint32_t gridWidth  = (width) / step;
    uint32_t gridHeight = (height) / step;

    uint32_t countVert = 0;
    uint32_t countHor  = gridWidth * gridHeight * 2;

    for (uint32_t i = step; i < height; i += step)
    {
        float32_t ratioI = 0.0f;
        {
            if (static_cast<float32_t>(i) > (static_cast<float32_t>(height) / 2.0f))
                ratioI = i - (static_cast<float32_t>(height) / 2.0f);
            else
                ratioI = (static_cast<float32_t>(height) / 2.0f) - i;
            ratioI /= (static_cast<float32_t>(height) / 2.0f);
        }
        for (uint32_t j = step; j < width; j += step)
        {
            float32_t ratioJ = 0.0f;
            {
                if (static_cast<float32_t>(j) > (static_cast<float32_t>(width) / 2.0f))
                    ratioJ = j - (static_cast<float32_t>(width) / 2.0f);
                else
                    ratioJ = (static_cast<float32_t>(width) / 2.0f) - j;
                ratioJ /= (static_cast<float32_t>(width) / 2.0f);
            }

            float32_t transparency = (1.0f - std::max(ratioI, ratioJ)) * 0.8f;
            dwRenderEngineColorRGBA colorGrid{0.2f, 0.2f, 0.8f, transparency};

            // if no translation is present (homography is identity) the distortion map describes the shape
            // of the distorted lense, mark in green
            if ((m_translate.x == 0.0f) && (m_translate.y == 0.0f) && (m_translate.z == 0.0f))
                colorGrid = {0.2f, 0.8f, 0.2f, transparency};

            colorGrid.w = transparency;

            dwVector2f points[2];

            // if given distortion points, render a distorted grid, otherwise a rectified grid
            if (distortionPoints)
            {
                points[0] = distortionPoints[j + (i - step) * width];
                points[1] = distortionPoints[j + i * width];

                m_vertexDist[countVert].pos     = points[0];
                m_vertexDist[countVert++].color = colorGrid;
                m_vertexDist[countVert].pos     = points[1];
                m_vertexDist[countVert++].color = colorGrid;
                m_vertexDist[countHor].pos      = points[0];
                m_vertexDist[countHor++].color  = colorGrid;
            }
            else
            {
                points[0].x = j;
                points[0].y = i - step;
                points[1].x = j;
                points[1].y = i;

                m_vertexUndist[countVert].pos     = points[0];
                m_vertexUndist[countVert++].color = colorGrid;
                m_vertexUndist[countVert].pos     = points[1];
                m_vertexUndist[countVert++].color = colorGrid;
                m_vertexUndist[countHor].pos      = points[0];
                m_vertexUndist[countHor++].color  = colorGrid;
            }

            if (distortionPoints)
            {
                points[0] = distortionPoints[(j - step) + (i - step) * width];

                m_vertexDist[countHor].pos     = points[0];
                m_vertexDist[countHor++].color = colorGrid;
            }
            else
            {
                points[0].x = j - step;
                points[0].y = i - step;

                m_vertexUndist[countHor].pos     = points[0];
                m_vertexUndist[countHor++].color = colorGrid;
            }
        }
    }
}

//#######################################################################################
void RectifierApp::renderDistortionGrid(ColoredPoint2f* buffer)

{
    CHECK_DW_ERROR(dwRenderEngine_setBuffer(m_bufferVertex, DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_2D,
                                            buffer,
                                            sizeof(ColoredPoint2f), 0, m_lineCount, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_ATTRIBUTE_RGBA, 1, m_renderEngine));

    glEnable(GL_BLEND);
    dwRenderEngine_setLineWidth(2, m_renderEngine);
    CHECK_DW_ERROR(dwRenderEngine_renderBuffer(m_bufferVertex,
                                               m_lineCount,
                                               m_renderEngine));
}

void RectifierApp::onRender()
{
    dwImageHandle_t imageGL;
    dwImageStreamerGL_producerSend(m_image, m_streamerCUDA2GL);
    dwStatus status = dwImageStreamerGL_consumerReceive(&imageGL, 500000, m_streamerCUDA2GL);

    if (status != DW_SUCCESS)
    {
        logError("Did not receive GL frame within 500ms");
    }
    else
    {
        // Render input image
        dwImageGL* frameGL;
        dwImage_getGL(&frameGL, imageGL);

        CHECK_DW_ERROR(dwRenderEngine_setTile(m_tile[0], m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        dwVector2f range{};
        range.x = frameGL->prop.width;
        range.y = frameGL->prop.height;
        CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_renderImage2D(frameGL, {0, 0, range.x, range.y}, m_renderEngine));

        //render distortion map
        if (m_renderDistortion)
        {
            dwImageCUDA distMap;
            dwRectifier_getDistortionMap(&distMap, m_rectifier);

            dwRenderEngine_setColor(DW_RENDERER_COLOR_LIGHTBLUE, m_renderEngine);
            dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_16, m_renderEngine);
            CHECK_DW_ERROR(dwRenderEngine_renderText2D("Input", {25, 40}, m_renderEngine));
            renderDistortionGrid(m_vertexDist);

            if ((m_translate.x == 0.0f) && (m_translate.y == 0.0f) && (m_translate.z == 0.0f))
            {
                dwRenderEngine_renderText2D("No homography", {static_cast<float32_t>(getWindowWidth()) - 300, 40}, m_renderEngine);
            }
            else
            {
                std::string position = std::string("Translation: ") +
                                       std::to_string(static_cast<int32_t>(m_translate.x)) + std::string(",") +
                                       std::to_string(static_cast<int32_t>(m_translate.y)) + std::string(",") +
                                       std::to_string(static_cast<int32_t>(m_translate.z));
                dwRenderEngine_renderText2D(position.c_str(), {static_cast<float32_t>(getWindowWidth()) - 300, 40}, m_renderEngine);
            }
        }

        // return frame
        dwImageStreamerGL_consumerReturn(&imageGL, m_streamerCUDA2GL);
    }

    CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 30000, m_streamerCUDA2GL));

    // Render output image
    dwImageHandle_t frameGLOut;
    dwImageStreamerGL_producerSend(m_rectifiedImage, m_streamerCUDA2GL);
    status = dwImageStreamerGL_consumerReceive(&frameGLOut, 500000, m_streamerCUDA2GL);

    if (status != DW_SUCCESS)
    {
        logError("Did not receive GL frame within 500ms");
    }
    else
    {
        // render received texture
        dwImageGL* dwFrameGL;
        dwImage_getGL(&dwFrameGL, frameGLOut);
        {
            CHECK_DW_ERROR(dwRenderEngine_setTile(m_tile[1], m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

            dwVector2f range{};
            range.x = dwFrameGL->prop.width;
            range.y = dwFrameGL->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(dwFrameGL, {0, 0, range.x, range.y}, m_renderEngine));

            if (m_renderDistortion)
            {
                dwRenderEngine_setColor(DW_RENDERER_COLOR_LIGHTBLUE, m_renderEngine);
                dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_16, m_renderEngine);
                CHECK_DW_ERROR(dwRenderEngine_renderText2D("Rectified", {25, 40}, m_renderEngine));
                renderDistortionGrid(m_vertexUndist);
            }
        }

        // return frame
        dwImageStreamerGL_consumerReturn(&frameGLOut, m_streamerCUDA2GL);
    }

    CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 30000, m_streamerCUDA2GL));

    CHECK_GL_ERROR();

    renderutils::renderFPS(m_renderEngine, getCurrentFPS());
}

//#######################################################################################
void RectifierApp::updateHomography()
{
    // camera output looks where the input camera looks
    // define camera output rotation
    dwMatrix3f camOutRotationMatrix;
    camOutRotationMatrix.array[0] = m_transformation.array[0];
    camOutRotationMatrix.array[3] = m_transformation.array[1];
    camOutRotationMatrix.array[6] = m_transformation.array[2];
    camOutRotationMatrix.array[1] = m_transformation.array[4];
    camOutRotationMatrix.array[4] = m_transformation.array[5];
    camOutRotationMatrix.array[7] = m_transformation.array[6];
    camOutRotationMatrix.array[2] = m_transformation.array[8];
    camOutRotationMatrix.array[5] = m_transformation.array[9];
    camOutRotationMatrix.array[8] = m_transformation.array[10];

    // normal to the output plane
    float32_t normalToPlane[3] = {-1, 0, 0};
    float32_t distanceToPlane  = m_transformation.array[3 * 4 + 0];
    // define camera output translation, same as the input camera + user input
    float32_t camOutTranslate[3] = {m_transformation.array[12] + m_translate.x,
                                    m_transformation.array[13] + m_translate.y,
                                    m_transformation.array[14] + m_translate.z};

    computeHomography(&m_homography, m_transformation, camOutRotationMatrix, camOutTranslate, normalToPlane, distanceToPlane);
    dwRectifier_setHomography(&m_homography, m_rectifier);

    // update grid for rendering
    dwImageCUDA distMap;
    dwRectifier_getDistortionMap(&distMap, m_rectifier);
    CHECK_CUDA_ERROR(cudaMemcpy2D(cpuDistMap, sizeof(dwVector2f) * distMap.prop.width, distMap.dptr[0],
                                  distMap.pitch[0], sizeof(dwVector2f) * distMap.prop.width, distMap.prop.height, cudaMemcpyDeviceToHost));

    computeRenderGrid(cpuDistMap, distMap.prop.width, distMap.prop.height, m_renderStep);
    computeRenderGrid(nullptr, distMap.prop.width, distMap.prop.height, m_renderStep);
}

void RectifierApp::onKeyDown(int key, int /*scancode*/, int /*mods*/)
{
    if (key == GLFW_KEY_LEFT)
    {
        m_translate.y += 0.5;
        updateHomography();
    }
    if (key == GLFW_KEY_RIGHT)
    {
        m_translate.y -= 0.5;
        updateHomography();
    }

    if (key == GLFW_KEY_UP)
    {
        m_translate.x += 0.5;
        updateHomography();
    }
    if (key == GLFW_KEY_DOWN)
    {
        m_translate.x -= 0.5;
        updateHomography();
    }

    if (key == GLFW_KEY_U)
    {
        m_translate.z += 0.5;
        updateHomography();
    }
    if (key == GLFW_KEY_J)
    {
        m_translate.z -= 0.5;
        updateHomography();
    }
    if (key == GLFW_KEY_ENTER)
    {
        m_renderDistortion = !m_renderDistortion;
    }
}

//#######################################################################################
void RectifierApp::onReset()
{
    // reset camera
    m_camera->resetCamera();
    // reset camera
    if (m_frame)
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_frame));

    dwSensor_reset(m_cameraSensor);
}

//#######################################################################################
void RectifierApp::onRelease()
{
    delete m_vertexDist;
    delete m_vertexUndist;

    if (m_rectifiedImage)
        dwImage_destroy(m_rectifiedImage);

    m_camera.reset();

    dwStatus status = dwSensor_stop(m_cameraSensor);
    if (status != DW_SUCCESS)
    {
        logError("Cannot stop sensor: %s\n", dwGetStatusName(status));
    }

    if (m_frameCap)
        dwFrameCapture_release(m_frameCap);

    status = dwSAL_releaseSensor(m_cameraSensor);

    if (status != DW_SUCCESS)
    {
        logError("Cannot release sensor: %s\n", dwGetStatusName(status));
    }

    if (m_renderEngine != DW_NULL_HANDLE)
    {
        dwRenderEngine_destroyBuffer(m_bufferVertex, m_renderEngine);
        dwRenderEngine_release(m_renderEngine);
    }

    status = dwCameraModel_release(m_cameraModelIn);
    if (status != DW_SUCCESS)
    {
        logError("Cannot release camera: %s\n", dwGetStatusName(status));
    }

    status = dwCameraModel_release(m_cameraModelOut);
    if (status != DW_SUCCESS)
    {
        logError("Cannot release camera out: %s\n", dwGetStatusName(status));
    }

    status = dwImageStreamerGL_release(m_streamerCUDA2GL);
    if (status != DW_SUCCESS)
    {
        logError("Cannot release CUDA2GL streamer: %s\n", dwGetStatusName(status));
    }

    status = dwRectifier_release(m_rectifier);
    if (status != DW_SUCCESS)
    {
        logError("Cannot release rectifier: %s\n", dwGetStatusName(status));
    }

    status = dwSAL_release(m_sal);
    if (status != DW_SUCCESS)
    {
        logError("Cannot release SAL: %s\n", dwGetStatusName(status));
    }

    CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
    CHECK_DW_ERROR(dwRelease(m_context));
    CHECK_DW_ERROR(dwLogger_release());
}

//#######################################################################################
/// -----------------------------
/// Initialize Logger and DriveWorks context
/// -----------------------------
void RectifierApp::initializeDriveWorks(dwContextHandle_t& context) const
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
bool RectifierApp::initDriveworks()
{
    initializeDriveWorks(m_context);

    CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
    CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

    return true;
}

//#######################################################################################
bool RectifierApp::initCameras()
{
    // create GMSL Camera interface
    uint32_t cameraSiblings   = 0U;
    float32_t cameraFramerate = 0.0f;
    dwImageType imageType;

    bool bStatus = createVideoReplay(&m_cameraSensor, &m_cameraWidth, &m_cameraHeight, &cameraSiblings,
                                     &cameraFramerate, &imageType, m_sal, m_videoFilename);

    if (bStatus != true)
    {
        logError("Cannot create video replay");
        return false;
    }

    std::cout << "Camera image with " << m_cameraWidth << "x" << m_cameraHeight << " at "
              << cameraFramerate << " FPS" << std::endl;

    dwStatus status;

    // initialize input camera models from rig
    dwRigHandle_t rigConf = DW_NULL_HANDLE;

    // load vehicle configuration
    status = dwRig_initializeFromFile(&rigConf, m_context, m_rigConfigFilename.c_str());
    if (status != DW_SUCCESS)
    {
        logError("Cannot load vehicle rig configuration from %s: %s\n",
                 m_rigConfigFilename.c_str(), dwGetStatusName(status));
        return false;
    }

    uint32_t sensorId;
    CHECK_DW_ERROR(dwRig_findSensorByName(&sensorId, m_cameraName, rigConf));

    CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&m_transformation, sensorId, rigConf));
    dwFThetaCameraConfig config{};
    dwRig_getFThetaCameraConfig(&config, 0, rigConf);
    CHECK_DW_ERROR(dwCameraModel_initialize(&m_cameraModelIn, sensorId, rigConf));

    dwRig_release(rigConf);
    if (status != DW_SUCCESS)
    {
        logError("Cannot initialize cameras from rig: %s\n", dwGetStatusName(status));
        return false;
    }

    // initialize input camera
    dwSensorParams params{};
    std::string arguments = "video=" + m_videoFilename;
    params.parameters     = arguments.c_str();
    params.protocol       = "camera.virtual";

    dwImageProperties props{};
    props.type   = DW_IMAGE_CUDA;
    props.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    props.width  = m_cameraWidth;
    props.height = m_cameraHeight;

    m_camera = std::unique_ptr<SimpleCamera>(new SimpleCamera(props, params, m_sal, m_context));

    m_camera->enableGLOutput();

    // initialize output camera model as simple pinhole if fov is < 180 degree, tilted stereographic otherwise
    if (RAD2DEG(m_fovX) >= 180)
    {
        dwStereographicCameraConfig cameraConf = {};

        cameraConf.u0     = static_cast<float32_t>(m_cameraWidth / 2) - 1;
        cameraConf.v0     = static_cast<float32_t>(m_cameraHeight / 2) - 1;
        cameraConf.width  = m_cameraWidth;
        cameraConf.height = m_cameraHeight;
        cameraConf.hFOV   = m_fovX;

        CHECK_DW_ERROR(dwCameraModel_initializeStereographic(&m_cameraModelOut, &cameraConf, m_context));
    }
    else
    {
        dwPinholeCameraConfig cameraConf = {};
        cameraConf.distortion[0]         = 0.f;
        cameraConf.distortion[1]         = 0.f;
        cameraConf.distortion[2]         = 0.f;

        cameraConf.u0     = static_cast<float32_t>(m_cameraWidth / 2);
        cameraConf.v0     = static_cast<float32_t>(m_cameraHeight / 2);
        cameraConf.width  = m_cameraWidth;
        cameraConf.height = m_cameraHeight;

        dwVector2f focal  = focalFromFOV({m_fovX, m_fovY}, {cameraConf.width, cameraConf.height});
        cameraConf.focalX = focal.x;
        cameraConf.focalY = focal.y;

        CHECK_DW_ERROR(dwCameraModel_initializePinhole(&m_cameraModelOut, &cameraConf, m_context));
    }

    return true;
}

//#######################################################################################
bool RectifierApp::initImageStreamer()
{
    dwImageProperties props{};
    props.type   = DW_IMAGE_CUDA;
    props.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    props.width  = m_cameraWidth;
    props.height = m_cameraHeight;

    CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_streamerCUDA2GL, &props, DW_IMAGE_GL, m_context));

    return true;
}

//#######################################################################################
bool RectifierApp::initRenderer()
{
    // init render engine with default params
    dwRenderEngineParams params{};
    CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
    CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

    dwRenderEngineTileState paramList[2];
    for (uint32_t i = 0; i < 2; ++i)
    {
        dwRenderEngine_initTileState(&paramList[i]);
        paramList[i].modelViewMatrix = DW_IDENTITY_MATRIX4F;
        paramList[i].font            = DW_RENDER_ENGINE_FONT_VERDANA_24;
    }

    dwRenderEngine_addTilesByCount(m_tile, 2, 2, paramList, m_renderEngine);

    dwImageCUDA distMap;
    dwRectifier_getDistortionMap(&distMap, m_rectifier);
    CHECK_CUDA_ERROR(cudaMemcpy2D(cpuDistMap, sizeof(dwVector2f) * distMap.prop.width, distMap.dptr[0],
                                  distMap.pitch[0], sizeof(dwVector2f) * distMap.prop.width, distMap.prop.height, cudaMemcpyDeviceToHost));

    uint32_t gridWidth  = (distMap.prop.width) / m_renderStep;
    uint32_t gridHeight = (distMap.prop.height) / m_renderStep;

    m_lineCount = gridHeight * gridWidth * 2;

    m_vertexDist   = new ColoredPoint2f[m_lineCount * 2];
    m_vertexUndist = new ColoredPoint2f[m_lineCount * 2];
    CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_bufferVertex, DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_2D, sizeof(ColoredPoint2f),
                                               0, m_lineCount * 2, m_renderEngine));

    computeRenderGrid(cpuDistMap, distMap.prop.width, distMap.prop.height, m_renderStep);
    computeRenderGrid(nullptr, distMap.prop.width, distMap.prop.height, m_renderStep);
    return true;
}

//#######################################################################################
bool RectifierApp::initRectifier()
{
    CHECK_DW_ERROR(dwRectifier_initialize(&m_rectifier, m_cameraModelIn,
                                          m_cameraModelOut, m_context));

    dwImageProperties props{};
    props.memoryLayout = DW_IMAGE_MEMORY_TYPE_PITCH;
    props.height       = static_cast<int32_t>(m_cameraHeight);
    props.width        = static_cast<int32_t>(m_cameraWidth);
    props.format       = DW_IMAGE_FORMAT_RGBA_UINT8;
    props.type         = DW_IMAGE_CUDA;

    // Add extra attributes to enable tne NVMediaLDC api.
    CHECK_DW_ERROR(dwRectifier_appendAllocationAttributes(&props, m_rectifier));
    CHECK_DW_ERROR(dwImage_create(&m_rectifiedImage, props, m_context));

    // if it is stereographic projection, we tilt to match the horizon of the camera with the world horizon. We only need to correct pitch and roll, as yaw would be parallel to the horizon and unnecessary to correct
    if (RAD2DEG(m_fovX) >= 180)
        CHECK_DW_ERROR(dwRectifier_setHomographyFromRotation(-7.410536527633667f, -47.1883316040039f, 0.0f, m_rectifier));

    CHECK_DW_ERROR(dwRectifier_getHomography(&m_homography, m_rectifier));

    cpuDistMap = reinterpret_cast<dwVector2f*>(malloc(sizeof(dwVector2f) * props.width * props.height * 2));

    return true;
}

//#######################################################################################
bool RectifierApp::createVideoReplay(dwSensorHandle_t* salSensor,
                                     uint32_t* cameraWidth,
                                     uint32_t* cameraHeight,
                                     uint32_t* cameraSiblings,
                                     float32_t* cameraFrameRate,
                                     dwImageType* imageType,
                                     dwSALHandle_t sal,
                                     const std::string& videoFName)
{
    dwStatus status;
    std::string arguments = "video=" + videoFName;

    dwSensorParams params{};
    params.parameters = arguments.c_str();
    params.protocol   = "camera.virtual";
    status            = dwSAL_createSensor(salSensor, params, sal);
    if (status != DW_SUCCESS)
    {
        logError("Cannot create sensor: %s\n", dwGetStatusName(status));
        return false;
    }

    dwImageProperties cameraImageProperties{};
    status = dwSensorCamera_getImageProperties(&cameraImageProperties,
                                               DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                               *salSensor);
    if (status != DW_SUCCESS)
    {
        logError("Cannot get camera image properties: %s\n", dwGetStatusName(status));
        return false;
    }

    dwCameraProperties cameraProperties{};
    status = dwSensorCamera_getSensorProperties(&cameraProperties, *salSensor);
    if (status != DW_SUCCESS)
    {
        logError("Cannot get camera sensor properties: %s\n", dwGetStatusName(status));
        return false;
    }

    *cameraWidth     = cameraImageProperties.width;
    *cameraHeight    = cameraImageProperties.height;
    *imageType       = cameraImageProperties.type;
    *cameraFrameRate = cameraProperties.framerate;
    *cameraSiblings  = cameraProperties.siblings;

    // we would like the application run as fast as the original video
    setProcessRate(cameraProperties.framerate);

    // -----------------------------
    // Start Sensors
    // -----------------------------
    CHECK_DW_ERROR(dwSensor_start(*salSensor));

    return true;
}

//#######################################################################################
void RectifierApp::setRendererRect(int x, int y)
{
    dwRect rectangle{};
    rectangle.width  = DriveWorksSample::getWindowWidth() / 2;
    rectangle.height = DriveWorksSample::getWindowHeight();
    rectangle.x      = x;
    rectangle.y      = y;

    dwRenderer_setRect(rectangle, m_renderer);
}

//#######################################################################################
int main(int argc, const char** argv)
{
    // define all arguments used by the application
    const ProgramArguments arguments = ProgramArguments(argc, argv,
                                                        {ProgramArguments::Option_t("rig", (dw_samples::SamplesDataPath::get() + "/samples/sfm/triangulation/rig.json").c_str()),
                                                         ProgramArguments::Option_t("video", (dw_samples::SamplesDataPath::get() + std::string{"/samples/sfm/triangulation/video_0.h264"}).c_str()),
                                                         ProgramArguments::Option_t("fovX", "120"),
                                                         ProgramArguments::Option_t("fovY", "90"),
                                                         ProgramArguments::Option_t("camera-name", "SVIEW_FR"),
                                                         ProgramArguments::Option_t("record-video", "", "record the rectified output to video (h264/h265/mp4)")},
                                                        "The Video Rectification sample demonstrates how to remove fisheye distortion from a video captured "
                                                        "on a camera with a fisheeye lens.");

    // Window/GL based application
    RectifierApp app(arguments);
    app.initializeWindow("Video Rectifier Sample",
                         RectifierApp::FRAME_WIDTH, RectifierApp::FRAME_HEIGHT / 2,
                         arguments.enabled("offscreen"));
    return app.run();
}
