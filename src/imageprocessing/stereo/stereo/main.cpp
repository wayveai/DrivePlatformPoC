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

// Driveworks
#include <dw/core/base/Version.h>

// Sample framework
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/WindowGLFW.hpp>

// Stereo
#include <dw/imageprocessing/stereo/Stereo.h>

// Stereo sample utils
#include "utils.hpp"

// Render Engine
#include <dwvisualization/core/RenderEngine.h>

using namespace dw_samples::common;

class StereoApp : public DriveWorksSample
{
private:
    // Module handles
    dwContextHandle_t m_context                               = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_vizContext               = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine                     = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                                       = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfiguration                          = DW_NULL_HANDLE;
    dwStereoRectifierHandle_t m_stereoRectifier               = DW_NULL_HANDLE;
    dwStereoHandle_t m_stereoAlgorithm                        = DW_NULL_HANDLE;
    dwPyramidImage m_pyramids[DW_STEREO_SIDE_COUNT]           = {};
    dwCameraModelHandle_t m_cameraModel[DW_STEREO_SIDE_COUNT] = {DW_NULL_HANDLE, DW_NULL_HANDLE};

    // Image handles
    dwImageHandle_t m_stereoImages[DW_STEREO_SIDE_COUNT];    // input from video
    dwImageHandle_t m_outputRectified[DW_STEREO_SIDE_COUNT]; // output of the stereo rectifier (RGBA)
    dwImageHandle_t m_cudaCropImages[DW_STEREO_SIDE_COUNT];  // crop of stereo rectifier output of even size
    dwImageHandle_t m_inputImagesYUV[DW_STEREO_SIDE_COUNT];  // input for stereo algorithm (YUV420)
    dwImageHandle_t m_colorDisparity;                        // color disparity output
    dwImageHandle_t m_outputAnaglyph;                        // anaglyph output
    dwImageHandle_t m_disparity;                             // gray disparity output

    // Rendering tiles
    uint32_t m_tiles[4];
    uint32_t m_tileTop;

    // Streamers
    std::unique_ptr<SimpleImageStreamerGL<>> m_cuda2glInput[DW_STEREO_SIDE_COUNT];           // input rectifier streamer
    std::unique_ptr<SimpleImageStreamerGL<dwImageGL>> m_cuda2glOutput[DW_STEREO_SIDE_COUNT]; // output rectifier streamer
    std::unique_ptr<SimpleImageStreamerGL<>> m_cudaRGBA2gl;                                  // output anaglyph streamer
    std::unique_ptr<SimpleImageStreamerGL<>> m_cudaDISP2gl[DW_STEREO_SIDE_COUNT];            // output GL disparity streamer
    std::unique_ptr<SimpleImageStreamer<>> m_cudaDISP2cpu[DW_STEREO_SIDE_COUNT];             // output CPU disparity streamer

    // CPU image for depth rendering annotation
    dwImageCPU* m_disparityCPU[DW_STEREO_SIDE_COUNT]; // gray disparity CPU output

    // GL images for output
    dwImageGL* m_displayInput[DW_STEREO_SIDE_COUNT];     // rectifier input
    dwImageGL* m_displayOutput[DW_STEREO_SIDE_COUNT];    // rectifier output
    dwImageGL* m_displayAnaglyph;                        // anaglyph output
    dwImageGL* m_displayDisparity[DW_STEREO_SIDE_COUNT]; // disparity output

    // Camera sensor indices
    uint32_t m_cameraLeftSensorIdx  = std::numeric_limits<decltype(m_cameraLeftSensorIdx)>::max();
    uint32_t m_cameraRightSensorIdx = std::numeric_limits<decltype(m_cameraRightSensorIdx)>::max();

    // Rectifier rendering
    bool m_renderRectifier = false;
    std::vector<dwVector2f> m_lines;

    // Stereo algorithm and rendering variables
    float32_t m_depthGain;                          // gain for disparity/depth rendering
    float32_t m_invalidThreshold           = -1.0f; // threshold for invalid disparities
    static constexpr float32_t INV_THR_MAX = 6.0f;  // max invalid threshold
    bool m_occlusion                       = true;
    bool m_occlusionInfill                 = false;
    bool m_infill                          = false;
    uint32_t m_levelStop;    // last pyramid level where disparity is estimated
    dwStereoSide m_side;     // compute disparity only for left side or both
    uint32_t m_maxDisparity; // maximum disparity searched for
    float32_t m_maxDistance; // maximum distance to visualize depth
    dwBox2D m_roi;           // stereo rectifier output ROI

    dwImageProperties m_imageProperties{};
    float32_t m_focalLength[DW_STEREO_SIDE_COUNT];
    float32_t m_baseline;
    dwVector2f m_depth[DW_STEREO_SIDE_COUNT];
    bool m_depthFlag[DW_STEREO_SIDE_COUNT] = {false, false};
    float32_t m_barOffset, m_barLength;
    std::vector<std::vector<dwVector2f>> m_barLines;
    std::string m_inputText, m_outputText, m_disparityText;

    // live camera
    // camera can have single image input side-by-side (ZEN camera) or two separate inputs
    bool m_inputTypeCamera = false;
    std::unique_ptr<SimpleCamera> m_stereoCamera[DW_STEREO_SIDE_COUNT];
    dwImageHandle_t m_stereoFrames[DW_STEREO_SIDE_COUNT] = {DW_NULL_HANDLE, DW_NULL_HANDLE};

public:
    StereoApp(const ProgramArguments& args)
        : DriveWorksSample(args)
    {
        m_levelStop    = std::stoi(args.get("level").c_str());
        m_side         = args.get("single_side") == std::string("0") ? DW_STEREO_SIDE_BOTH : DW_STEREO_SIDE_LEFT;
        m_maxDisparity = std::stoi(args.get("maxDisparity"));
        m_maxDistance  = std::stof(args.get("maxDistance"));

        // gain for the color coding, proportional to the level we stop at.
        m_depthGain = 1 << m_levelStop;
    }

    // -----------------------------------------------------------------------------
    // Initialize all modules
    bool onInitialize() override
    {
        initializeDriveWorks(m_context);

        initializeRenderer();

        initializeStereoCamera();

        // get calibrated cameras from the rig configuration (see also sample_rig for more details)
        initializeCalibratedCameras();

        initializeStereoRectifier();

        initializeStereoRectifierRendering();

        initializeStereoAlgorithm();

        initializeStereoAlgorithmRendering();

        return true;
    }

    // -----------------------------------------------------------------------------
    // Read stero pair from input sensors
    // Rectify stereo images
    // Compute disparity with stereo algorithm
    // Capture rectified videos if requested
    // Render data on screen
    void onProcess()
    {
        // get stereo images
        if (!readStereoImages(m_stereoImages))
            return;

        // get CUDA images from containers
        dwImageCUDA* imageCUDA[DW_STEREO_SIDE_COUNT];
        dwImageCUDA* rectifiedCuda[DW_STEREO_SIDE_COUNT];
        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
        {
            CHECK_DW_ERROR(dwImage_getCUDA(&imageCUDA[side], m_stereoImages[side]));
            CHECK_DW_ERROR(dwImage_getCUDA(&rectifiedCuda[side], m_outputRectified[side]));
        }

        // rectify stereo images
        CHECK_DW_ERROR(dwStereoRectifier_rectify(rectifiedCuda[DW_STEREO_SIDE_LEFT],
                                                 rectifiedCuda[DW_STEREO_SIDE_RIGHT],
                                                 imageCUDA[DW_STEREO_SIDE_LEFT], imageCUDA[DW_STEREO_SIDE_RIGHT],
                                                 m_stereoRectifier));

        // create anaglyph of the input (left red, right blue)
        dwImageCUDA* imageAnaglyphCUDA;
        CHECK_DW_ERROR(dwImage_getCUDA(&imageAnaglyphCUDA, m_outputAnaglyph));
        createAnaglyph(*imageAnaglyphCUDA, *rectifiedCuda[DW_STEREO_SIDE_LEFT], *rectifiedCuda[DW_STEREO_SIDE_RIGHT]);

        // get display image for anaglyph
        dwImageHandle_t imageGL = m_cudaRGBA2gl->post(m_outputAnaglyph);
        CHECK_DW_ERROR(dwImage_getGL(&m_displayAnaglyph, imageGL));

        // crop rectified images to even size
        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
        {
            // ROI mapping
            dwImageCUDA* cudaCrop;
            CHECK_DW_ERROR(dwImage_getCUDA(&cudaCrop, m_cudaCropImages[side]));
            dwRect rect{0, 0, static_cast<int32_t>(m_imageProperties.width), static_cast<int32_t>(m_imageProperties.height)};
            CHECK_DW_ERROR(dwImageCUDA_mapToROI(cudaCrop, rectifiedCuda[side], rect));
        }

        // convert from RGBA to YUV420
        dwImageCUDA* inputImagesCUDA[DW_STEREO_SIDE_BOTH];
        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
        {
            CHECK_DW_ERROR(dwImage_copyConvert(m_inputImagesYUV[side], m_cudaCropImages[side], m_context));
            CHECK_DW_ERROR(dwImage_getCUDA(&inputImagesCUDA[side], m_inputImagesYUV[side]));
        }

        // build pyramids
        {
            ProfileCUDASection s(getProfilerCUDA(), "Pyramid build");
            for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
            {
                CHECK_DW_ERROR(dwImageFilter_computePyramid(&m_pyramids[side], inputImagesCUDA[side], 0, m_context));
            }
        }

        // compute disparity
        {
            ProfileCUDASection s(getProfilerCUDA(), "Stereo");

            CHECK_DW_ERROR(dwStereo_computeDisparity(&m_pyramids[DW_STEREO_SIDE_LEFT],
                                                     &m_pyramids[DW_STEREO_SIDE_RIGHT],
                                                     m_stereoAlgorithm));
        }

        // get output and prepare for display
        for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
        {
            const dwImageCUDA *disparity, *confidence;
            CHECK_DW_ERROR(dwStereo_getDisparity(&disparity, static_cast<dwStereoSide>(side), m_stereoAlgorithm));
            // get disparity in CPU
            dwImageCUDA* d;
            CHECK_DW_ERROR(dwImage_getCUDA(&d, m_disparity));
            void* ptr                         = d->dptr[0];
            d->dptr[0]                        = disparity->dptr[0];
            dwImageHandle_t imageCPUDisparity = m_cudaDISP2cpu[side]->post(m_disparity);
            d->dptr[0]                        = ptr;
            CHECK_DW_ERROR(dwImage_getCPU(&m_disparityCPU[side], imageCPUDisparity));

            // get colored disparity image
            dwImageCUDA* colorDisparityCUDA;
            CHECK_DW_ERROR(dwImage_getCUDA(&colorDisparityCUDA, m_colorDisparity));
            colorCode(colorDisparityCUDA, *disparity, m_depthGain, m_maxDistance, m_focalLength[side] * m_baseline);

            // get confidence
            CHECK_DW_ERROR(dwStereo_getConfidence(&confidence, static_cast<dwStereoSide>(side), m_stereoAlgorithm));

            // mix disparity and confidence, where confidence is occlusion, show black, where it is invalidity show white, leave
            // as is otherwise. See README for instructions on how to change the threshold of invalidity
            if ((m_occlusionInfill == false) && (m_occlusion == true))
            {
                mixDispConf(colorDisparityCUDA, *confidence, m_invalidThreshold >= 0.0f);
            }

            // stream disparity and confidence
            dwImageHandle_t imageGLDisparity = m_cudaDISP2gl[side]->post(m_colorDisparity);
            CHECK_DW_ERROR(dwImage_getGL(&m_displayDisparity[side], imageGLDisparity));
        }
    }

    // -----------------------------------------------------------------------------
    // Render stereo rectifer or stereo disparity
    // Stereo rectify displays the input stereo images in the top row and the rectified stereo images in the bottom row
    // In rectified images, the horizontal lines should cross the same pixels in the left and right image
    // Stereo disparity displays the anaglyph image in the top row and the disparity images in the bottom row
    // In disparity images, colors identify disparity: smaller disparity (closer objects) in red, larger disparity (further objects) in blue
    void onRender()
    {
        // Render input and rectified images with horizontal lines to visually check for rectification accuracy
        if (m_renderRectifier)
        {
            // get display images
            for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
            {
                // camera reset at end of file can leave m_stereoImages as null while reseting.
                if (m_stereoImages[side] == nullptr)
                {
                    return;
                }
                CHECK_DW_ERROR(dwRenderEngine_setTile(m_tiles[side], m_renderEngine));

                dwImageHandle_t displayFrame = m_cuda2glInput[side]->post(m_stereoImages[side]);

                CHECK_DW_ERROR(dwImage_getGL(&m_displayInput[side], displayFrame));

                CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_displayInput[side], {0.0f, 0.0f, 1.0f, 1.0f}, m_renderEngine));

                // Renders lines that show that non-rectified images have pixels that don't lie on the same horizontal line
                CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_RED, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_2D, m_lines.data(),
                                                     sizeof(dwVector2f),
                                                     0,
                                                     m_lines.size() / 2,
                                                     m_renderEngine));

                CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_WHITE, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderText2D(m_inputText.c_str(), dwVector2f{0.02f, 0.97f}, m_renderEngine));
            }

            for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
            {
                CHECK_DW_ERROR(dwRenderEngine_setTile(m_tiles[side + 2], m_renderEngine));

                m_displayOutput[side] = m_cuda2glOutput[side]->post(m_outputRectified[side]);

                CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_displayOutput[side], {0.0f, 0.0f, 1.0f, 1.0f}, m_renderEngine));

                // Renders lines that show that rectified images have pixels that lie on the same horizontal line
                CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_RED, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_2D, m_lines.data(),
                                                     sizeof(dwVector2f),
                                                     0,
                                                     m_lines.size() / 2,
                                                     m_renderEngine));

                CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_WHITE, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderText2D(m_outputText.c_str(), dwVector2f{0.02f, 0.97f}, m_renderEngine));
            }
        }
        else
        {
            CHECK_DW_ERROR(dwRenderEngine_setTile(m_tileTop, m_renderEngine));

            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_displayAnaglyph, {0.0f, 0.0f, 1.0f, 1.0f}, m_renderEngine));

            CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_GREEN, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderText2D("Input", dwVector2f{0.02f, 0.97f}, m_renderEngine));

            for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
            {
                CHECK_DW_ERROR(dwRenderEngine_setTile(m_tiles[side + 2], m_renderEngine));

                CHECK_DW_ERROR(dwRenderEngine_renderImage2D(m_displayDisparity[side], {0.0f, 0.0f, 1.0f, 1.0f}, m_renderEngine));

                CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_WHITE, m_renderEngine));
                CHECK_DW_ERROR(dwRenderEngine_renderText2D(m_disparityText.c_str(), dwVector2f{0.02f, 0.97f}, m_renderEngine));

                renderColorMap(side);

                // Render depth on mouse click
                if (m_depthFlag[side])
                {
                    uint32_t imageRow = static_cast<uint32_t>(m_depth[side].y * m_disparityCPU[side]->prop.height);
                    uint32_t imageCol = static_cast<uint32_t>(m_depth[side].x * m_disparityCPU[side]->prop.width);
                    uint32_t disp     = m_disparityCPU[side]->data[0][imageRow * m_disparityCPU[side]->prop.width + imageCol];
                    char depth[10];
                    sprintf(depth, "%.1f", m_focalLength[side] * m_baseline / (disp * m_depthGain));
                    CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_WHITE, m_renderEngine));
                    CHECK_DW_ERROR(dwRenderEngine_renderText2D(depth, dwVector2f{m_depth[side].x, m_depth[side].y}, m_renderEngine));
                }
            }
        }

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    // -----------------------------------------------------------------------------
    // Release all modules and handles
    void onRelease()
    {
        m_cudaRGBA2gl.reset();
        for (int32_t i = 0; i < DW_STEREO_SIDE_COUNT; i++)
        {
            m_cuda2glInput[i].reset();
            m_cuda2glOutput[i].reset();
            m_cudaDISP2gl[i].reset();
            m_cudaDISP2cpu[i].reset();
        }

        CHECK_DW_ERROR(dwImage_destroy(m_outputAnaglyph));
        CHECK_DW_ERROR(dwImage_destroy(m_colorDisparity));

        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
            CHECK_DW_ERROR(dwImage_destroy(m_inputImagesYUV[side]));

        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
            CHECK_DW_ERROR(dwImage_destroy(m_cudaCropImages[side]));

        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
        {
            if (m_stereoFrames[side])
                CHECK_DW_ERROR(dwImage_destroy(m_stereoFrames[side]));
        }

        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
            CHECK_DW_ERROR(dwPyramid_destroy(m_pyramids[side]));

        CHECK_DW_ERROR(dwStereoRectifier_release(m_stereoRectifier));
        CHECK_DW_ERROR(dwStereo_release(m_stereoAlgorithm));

        for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
            m_stereoCamera[side].reset();

        for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
        {
            if (m_cameraModel[side] != DW_NULL_HANDLE)
                CHECK_DW_ERROR(dwCameraModel_release(m_cameraModel[side]));
        }
        if (m_rigConfiguration != DW_NULL_HANDLE)
            CHECK_DW_ERROR(dwRig_release(m_rigConfiguration));

        if (m_renderEngine != DW_NULL_HANDLE)
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));

        if (m_vizContext != DW_NULL_HANDLE)
            CHECK_DW_ERROR(dwVisualizationRelease(m_vizContext));

        if (m_sal != DW_NULL_HANDLE)
            CHECK_DW_ERROR(dwSAL_release(m_sal));

        if (m_context != DW_NULL_HANDLE)
            CHECK_DW_ERROR(dwRelease(m_context));

        CHECK_DW_ERROR(dwLogger_release());
    }

    // -----------------------------------------------------------------------------
    // Actions for key pressing
    void onKeyDown(int32_t key, int32_t /*scancode*/, int32_t /*mods*/)
    {
        if (key == GLFW_KEY_O)
        {
            std::cout << "Toggle occlusion" << std::endl;
            m_occlusion = (m_occlusion == true) ? false : true;
            CHECK_DW_ERROR(dwStereo_setOcclusionTest(m_occlusion, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_K)
        {
            if (m_occlusion == true)
            {
                std::cout << "Toggle occlusion infill" << std::endl;
                m_occlusionInfill = (m_occlusionInfill == true) ? false : true;
                CHECK_DW_ERROR(dwStereo_setOcclusionInfill(m_occlusionInfill, m_stereoAlgorithm));
            }
            else
            {
                std::cout << "Cannot toggle occlusion infill, occlusion test is off" << std::endl;
            }
        }
        else if (key == GLFW_KEY_I)
        {
            std::cout << "Toggle invalidity infill" << std::endl;
            m_infill = (m_infill == true) ? false : true;
            CHECK_DW_ERROR(dwStereo_setInfill(m_infill, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_0)
        {
            std::cout << "Refinement 0" << std::endl;
            CHECK_DW_ERROR(dwStereo_setRefinementLevel(0, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_1)
        {
            std::cout << "Refinement 1" << std::endl;
            CHECK_DW_ERROR(dwStereo_setRefinementLevel(1, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_2)
        {
            std::cout << "Refinement 2" << std::endl;
            CHECK_DW_ERROR(dwStereo_setRefinementLevel(2, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_3)
        {
            std::cout << "Refinement 3" << std::endl;
            CHECK_DW_ERROR(dwStereo_setRefinementLevel(3, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_4)
        {
            std::cout << "Refinement 4" << std::endl;
            CHECK_DW_ERROR(dwStereo_setRefinementLevel(4, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_5)
        {
            std::cout << "Refinement 5" << std::endl;
            CHECK_DW_ERROR(dwStereo_setRefinementLevel(5, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_6)
        {
            std::cout << "Refinement 6" << std::endl;
            CHECK_DW_ERROR(dwStereo_setRefinementLevel(6, m_stereoAlgorithm));
        }
        else if (key == GLFW_KEY_KP_ADD)
        {
            m_invalidThreshold += 1.0f;
            if (m_invalidThreshold > INV_THR_MAX)
            {
                m_invalidThreshold = INV_THR_MAX;
            }
            CHECK_DW_ERROR(dwStereo_setInvalidThreshold(m_invalidThreshold, m_stereoAlgorithm));
            std::cout << "Invalidity thr " << m_invalidThreshold << std::endl;
        }
        else if (key == GLFW_KEY_KP_SUBTRACT)
        {
            m_invalidThreshold -= 1.0f;
            if (m_invalidThreshold < -1.0f)
            {
                m_invalidThreshold = -1.0f;
                CHECK_DW_ERROR(dwStereo_setInvalidThreshold(0.0f, m_stereoAlgorithm));
            }
            if (m_invalidThreshold >= 0.0f)
            {
                CHECK_DW_ERROR(dwStereo_setInvalidThreshold(m_invalidThreshold, m_stereoAlgorithm));
                std::cout << "Invalidity thr " << m_invalidThreshold << std::endl;
            }
            else
            {
                std::cout << "Invalidity off " << std::endl;
            }
        }
        else if (key == GLFW_KEY_L)
        {
            m_renderRectifier = !m_renderRectifier;
        }
    }

    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int32_t width, int32_t height) override
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

    void onMouseDown(int32_t button, float32_t x, float32_t y, int32_t /*mods*/) override
    {
        if (button != GLFW_MOUSE_BUTTON_1)
            return;
        uint32_t selectedTile = 0;
        dwVector2f screenPos{x, y};
        dwVector2f screenSize{static_cast<float32_t>(getWindowWidth()),
                              static_cast<float32_t>(getWindowHeight())};

        CHECK_DW_ERROR(dwRenderEngine_getTileByScreenCoordinates(&selectedTile,
                                                                 screenPos,
                                                                 screenSize,
                                                                 m_renderEngine));

        dwVector3f worldPos{};
        CHECK_DW_ERROR(dwRenderEngine_screenToWorld3D(&worldPos, screenPos,
                                                      screenSize,
                                                      m_renderEngine));

        if (selectedTile == m_tiles[2] || selectedTile == m_tiles[3])
        {
            dwStereoSide side = selectedTile == m_tiles[2] ? DW_STEREO_SIDE_LEFT : DW_STEREO_SIDE_RIGHT;
            m_depthFlag[side] = true;
            m_depth[side].x   = (worldPos.x + 1.f) / 2;
            m_depth[side].y   = (1.f - worldPos.y) / 2;
        }
    }

private:
    // -----------------------------
    // Initialize Logger and DriveWorks context
    void initializeDriveWorks(dwContextHandle_t& context)
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

        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
    }

    // -----------------------------------------------------------------------------
    // Initialize stereo cameras or videos
    void initializeStereoCamera()
    {
        if (m_inputTypeCamera)
        {
            // for now assume ZED camera (side-by-side images)
            m_stereoCamera[0] = initFromCamera();

            m_imageProperties.width  = m_stereoCamera[0]->getImageProperties().width / 2;
            m_imageProperties.height = m_stereoCamera[0]->getImageProperties().height / 2;

            // only for single input camera
            // we create 2 images that are pointing to nothing, so that they will point to the cropped stereo input
            void* empty[1]   = {nullptr};
            size_t emptyP[1] = {0};
            CHECK_DW_ERROR(dwImage_createAndBindBuffer(&m_stereoFrames[0], m_imageProperties, empty, emptyP, 0, m_context));
            CHECK_DW_ERROR(dwImage_createAndBindBuffer(&m_stereoFrames[1], m_imageProperties, empty, emptyP, 0, m_context));
        }
        else
        {
            m_stereoCamera[0] = initFromVideo(getArgs().get("video-left"));
            m_stereoCamera[1] = initFromVideo(getArgs().get("video-right"));

            m_inputText = "Input " + std::to_string(m_imageProperties.width) + "x" + std::to_string(m_imageProperties.height);
        }
    }

    // -----------------------------------------------------------------------------
    // Initialize stereo input from two videos
    std::unique_ptr<SimpleCamera> initFromVideo(const std::string& videoFName)
    {
        std::string arguments = "video=" + videoFName;
        dwSensorParams params{};
        params.parameters = arguments.c_str();
        params.protocol   = "camera.virtual";

        return initInput(params);
    }

    // -----------------------------------------------------------------------------
    // Initialize stereo video from a USB camera
    std::unique_ptr<SimpleCamera> initFromCamera()
    {
        dwSensorParams params{};
        params.protocol        = "camera.usb";
        std::string parameters = "device=" + getArgs().get("device");
        params.parameters      = parameters.c_str();

        return initInput(params);
    }

    // -----------------------------------------------------------------------------
    // Initialize camera sensor based on one of the functions above
    std::unique_ptr<SimpleCamera> initInput(const dwSensorParams& params)
    {
        m_imageProperties.type   = DW_IMAGE_CUDA;
        m_imageProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;

        std::unique_ptr<SimpleCamera> camera(new SimpleCamera(m_imageProperties, params, m_sal, m_context));

        dwCameraProperties cameraProperties     = camera->getCameraProperties();
        dwImageProperties cameraImageProperties = camera->getImageProperties();

        m_imageProperties.width  = cameraImageProperties.width;
        m_imageProperties.height = cameraImageProperties.height;

        {
            std::stringstream ss;
            ss << "Camera image with " << cameraImageProperties.width << "x" << cameraImageProperties.height
               << " at " << cameraProperties.framerate << " FPS" << std::endl;
            log("%s", ss.str().c_str());
        }

        return camera;
    }

    // -----------------------------------------------------------------------------
    // Initialize calibrated cameras from the cameras in the rig file
    void initializeCalibratedCameras()
    {
        if (getArgs().get("rigconfig").empty())
        {
            throw std::runtime_error("Rig configuration file not specified, please provide a rig "
                                     "configuration file with the calibration of the stereo camera");
        }

        // initialize rig configuration
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rigConfiguration, m_context,
                                                getArgs().get("rigconfig").c_str()));

        // get camera indices
        CHECK_DW_ERROR(dwRig_findSensorByName(&m_cameraLeftSensorIdx, getArgument("sensor-left").c_str(), m_rigConfiguration));
        CHECK_DW_ERROR(dwRig_findSensorByName(&m_cameraRightSensorIdx, getArgument("sensor-right").c_str(), m_rigConfiguration));

        // get the camera models stored in the rig (contain the intrinsics)
        CHECK_DW_ERROR(dwCameraModel_initialize(&m_cameraModel[0], m_cameraLeftSensorIdx,
                                                m_rigConfiguration));
        CHECK_DW_ERROR(dwCameraModel_initialize(&m_cameraModel[1], m_cameraRightSensorIdx,
                                                m_rigConfiguration));
    }

    // -----------------------------------------------------------------------------
    // Initialize stereo rectifier with the two sensor2rig extrinsics transformationso
    // Get the output crop ROI
    void initializeStereoRectifier()
    {
        // get the extrinsics transformation matrices (NOTE that this sample assumes the stereo cameras are the
        // first two of the enumerated camera sensors and are ordered LEFT and RIGHT)
        dwTransformation3f left2Rig, right2Rig;
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&left2Rig, m_cameraLeftSensorIdx, m_rigConfiguration));
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&right2Rig, m_cameraRightSensorIdx, m_rigConfiguration));

        // initialize the stereo rectifier using the camera models (intrinsics) and transformations (extrinsics)
        CHECK_DW_ERROR(dwStereoRectifier_initialize(&m_stereoRectifier, m_cameraModel[0], m_cameraModel[1],
                                                    left2Rig, right2Rig, m_context));

        //container for output of the stereo rectifier, the image size is found in the CropROI
        CHECK_DW_ERROR(dwStereoRectifier_getCropROI(&m_roi, m_stereoRectifier));

        m_outputText = "Output " + std::to_string(m_roi.width) + "x" + std::to_string(m_roi.height);

        // get focal length and baseline
        dwMatrix34f mLeft, mRight;
        CHECK_DW_ERROR(dwStereoRectifier_getProjectionMatrix(&mLeft, DW_STEREO_SIDE_LEFT, m_stereoRectifier));
        CHECK_DW_ERROR(dwStereoRectifier_getProjectionMatrix(&mRight, DW_STEREO_SIDE_RIGHT, m_stereoRectifier));
        m_focalLength[DW_STEREO_SIDE_LEFT]  = mLeft.array[0];
        m_focalLength[DW_STEREO_SIDE_RIGHT] = mRight.array[0];
        m_baseline                          = -mRight.array[9] / mRight.array[0];
    }

    // -----------------------------------------------------------------------------
    // Initialize stereo rectifier rendering variables: images and streamers
    // Resize image properties if size is not even (necessary as copy convert from RGBA to YUV works only for even images)
    void initializeStereoRectifierRendering()
    {
        dwImageProperties propsInput = m_imageProperties;
        propsInput.type              = DW_IMAGE_CUDA;
        propsInput.format            = DW_IMAGE_FORMAT_RGBA_UINT8;
        propsInput.width             = m_roi.width;
        propsInput.height            = m_roi.height;
        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
            CHECK_DW_ERROR(dwImage_create(&m_outputRectified[side], propsInput, m_context));

        // initialize the streamer for display
        for (uint32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
        {
            m_cuda2glInput[side].reset(new SimpleImageStreamerGL<>(m_imageProperties, 10000, m_context));
            m_cuda2glOutput[side].reset(new SimpleImageStreamerGL<dwImageGL>(propsInput, 10000, m_context));
        }

        std::cout << "Rectified image: " << m_roi.width << "x" << m_roi.height << std::endl;

        // Coordinates of lines to show rectification correctness
        for (uint32_t i = 5; i < 90; i += 10)
        {
            m_lines.push_back(dwVector2f{0, i / 100.f});
            m_lines.push_back(dwVector2f{1.0f, i / 100.f});
        }

        // StereoRectifier output is RGBA
        m_imageProperties        = propsInput;
        m_imageProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;

        // It is necessary to make image width and height even, as we have to convert from RGBA to YUV420
        // and this is allowed only for even sizes
        if (m_imageProperties.width % 2 != 0)
            m_imageProperties.width--;
        if (m_imageProperties.height % 2 != 0)
            m_imageProperties.height--;

        // create images to contain cropped stereo input
        for (uint32_t side = 0; side < DW_STEREO_SIDE_COUNT; ++side)
            CHECK_DW_ERROR(dwImage_create(&m_cudaCropImages[side], m_imageProperties, m_context));
    }

    // -----------------------------------------------------------------------------
    // Initialize renderer
    void initializeRenderer()
    {
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_vizContext, m_context));

        // init render engine with default params
        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_vizContext));

        // Setup tiles for rectified and depth rendering
        dwRenderEngineTileState paramList[4];
        for (uint32_t i = 0; i < 4; ++i)
        {
            paramList[i]                       = params.defaultTile;
            paramList[i].projectionMatrix      = DW_IDENTITY_MATRIX4F;
            paramList[i].modelViewMatrix       = DW_IDENTITY_MATRIX4F;
            paramList[i].layout.positionLayout = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_RELATIVE;
            paramList[i].layout.sizeLayout     = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_RELATIVE;
            paramList[i].lineWidth             = 1.0f;
            paramList[i].font                  = DW_RENDER_ENGINE_FONT_VERDANA_20;
        }
        CHECK_DW_ERROR(dwRenderEngine_addTilesByCount(m_tiles, 4, 2, paramList, m_renderEngine));

        // Setup extra tile for anaglyph rendering in the center of the top half screen
        dwRenderEngineTileState tileTopParams = params.defaultTile;
        tileTopParams.projectionMatrix        = DW_IDENTITY_MATRIX4F;
        tileTopParams.modelViewMatrix         = DW_IDENTITY_MATRIX4F;
        tileTopParams.layout.positionLayout   = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_RELATIVE;
        tileTopParams.layout.sizeLayout       = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_RELATIVE;
        tileTopParams.layout.viewport         = {0.25f, 0.f, 0.5f, 0.5f};
        tileTopParams.layout.positionType     = DW_RENDER_ENGINE_TILE_POSITION_TYPE_TOP_LEFT;
        tileTopParams.font                    = DW_RENDER_ENGINE_FONT_VERDANA_20;
        CHECK_DW_ERROR(dwRenderEngine_addTile(&m_tileTop, &tileTopParams, m_renderEngine));

        // Set up color lines for rendering color map
        m_barOffset    = 0.53f;
        m_barLength    = 0.45f;
        float32_t step = m_barLength / m_maxDisparity;
        for (uint32_t i = 0; i < m_maxDisparity; ++i)
        {
            std::vector<dwVector2f> line(2);
            line[0].x = 1.0f - m_barOffset + i * step;
            line[1].x = 1.0f - m_barOffset + (i + 1) * step;
            line[0].y = line[1].y = 0.98f;
            m_barLines.push_back(line);
        }
    }

    // -----------------------------------------------------------------------------
    // Initialize pyramids and stereo algorithm
    // Stereo algorithm works up to the m_levelStop of the pyramid (a higher level will speed up the computation)
    // Stereo matching is performed up to a disparity of m_maxDisparity (a larger baseline will require a larger max disparity)
    void initializeStereoAlgorithm()
    {
        // the stereo algorithm requires Y channel
        m_imageProperties.format = DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR;

        // the stereo algorithm inputs two gaussian pyramids built from the rectified input. This is because the
        // algorithm requires a multi resolution representation.
        // the default input has a very high resolution so many levels guarantee a better coverage of the image
        for (int32_t i = 0; i < DW_STEREO_SIDE_BOTH; ++i)
        {
            CHECK_DW_ERROR(dwPyramid_create(&m_pyramids[i], 8, m_imageProperties.width,
                                            m_imageProperties.height, DW_TYPE_UINT8, m_context));
        }

        // stereo parameters setup by the module. By default the level we stop at is 0, which is max resolution
        // in order to improve performance we stop at level specified by default as 1 in the input arguments
        dwStereoParams stereoParams;
        CHECK_DW_ERROR(dwStereo_initParams(&stereoParams));

        // level at which to stop computing disparity map
        stereoParams.levelStop = m_levelStop;
        // specifies which side to compute the disparity map from, if BOTH, LEFT or RIGHT only
        stereoParams.side = m_side;
        // since the pyramid is built for the stereo purpose we set the levels the same as the pyramids. In other
        // use cases it can be possible that the pyramid has too many levels and not all are necessary for the
        // stereo algorithm, so we can decide to use less
        stereoParams.levelCount = m_pyramids[0].levelCount;
        // set maximum disparity
        stereoParams.maxDisparityRange = m_maxDisparity;

        CHECK_DW_ERROR(dwStereo_initialize(&m_stereoAlgorithm, m_imageProperties.width, m_imageProperties.height,
                                           &stereoParams, m_context));

        uint32_t dispWidth, dispHeight;
        CHECK_DW_ERROR(dwStereo_getSize(&dispWidth, &dispHeight, m_levelStop, m_stereoAlgorithm));
        m_disparityText = std::to_string(dispWidth) + "x" + std::to_string(dispHeight);

        // create containers for input to the stereo algorithm
        for (uint32_t i = 0; i < DW_STEREO_SIDE_COUNT; ++i)
            CHECK_DW_ERROR(dwImage_create(&m_inputImagesYUV[i], m_imageProperties, m_context));
    }

    // -----------------------------------------------------------------------------
    // Initialize stereo rectifier rendering variables
    void initializeStereoAlgorithmRendering()
    {
        // properties for the display of the input image, rendered as anaglyph (both images overlapping)
        dwImageProperties displayProperties{};
        displayProperties.type   = DW_IMAGE_CUDA;
        displayProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;
        displayProperties.width  = m_imageProperties.width;
        displayProperties.height = m_imageProperties.height;

        m_cudaRGBA2gl.reset(new SimpleImageStreamerGL<>(displayProperties, 10000, m_context));

        CHECK_DW_ERROR(dwImage_create(&m_outputAnaglyph, displayProperties, m_context));

        // the output of the disparity map, although it appears scaled, has the resolution of the
        // level we stop at. For this reason we need to setup and image streamer with the proper resolution
        CHECK_DW_ERROR(dwStereo_getSize(&displayProperties.width, &displayProperties.height, m_levelStop,
                                        m_stereoAlgorithm));

        for (int32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
            m_cudaDISP2gl[side].reset(new SimpleImageStreamerGL<>(displayProperties, 10000, m_context));

        std::cout << "Disparity image: " << displayProperties.width << "x" << displayProperties.height << std::endl;

        // the disparity map is single channel, so we color it for better visualization (warm colors closer)
        CHECK_DW_ERROR(dwImage_create(&m_colorDisparity, displayProperties, m_context));
        displayProperties.format = DW_IMAGE_FORMAT_R_UINT8;
        for (int32_t side = 0; side < DW_STEREO_SIDE_BOTH; ++side)
            m_cudaDISP2cpu[side].reset(new SimpleImageStreamer<>(displayProperties, DW_IMAGE_CPU, 10000, m_context));

        CHECK_DW_ERROR(dwImage_create(&m_disparity, displayProperties, m_context));
    }

    // -----------------------------------------------------------------------------
    // Read pair of images from input sensors
    bool readStereoImages(dwImageHandle_t gStereoImages[DW_STEREO_SIDE_BOTH])
    {
        bool endStatus = true;

        dwImageProperties cameraProperties = m_stereoCamera[0]->getImageProperties();

        if (m_inputTypeCamera)
        {
            // for now the sample illustrates only USB cameras with a single input
            dwImageHandle_t image = m_stereoCamera[0]->readFrame();
            if (!image)
            {
                m_stereoCamera[0]->resetCamera();

                return false;
            }

            dwImageCUDA* stereoFrame;
            CHECK_DW_ERROR(dwImage_getCUDA(&stereoFrame, image));

            dwRect roi[2];
            roi[0].height = cameraProperties.height;
            roi[0].width  = cameraProperties.width / 2;
            roi[0].x      = 0;
            roi[0].y      = 0;
            roi[1].height = cameraProperties.height;
            roi[1].width  = cameraProperties.width / 2;
            roi[1].x      = cameraProperties.width / 2;
            roi[1].y      = 0;

            // image is split in two by remapping to two separate dwImageCUDA
            dwImageCUDA* stereoImage[2];
            CHECK_DW_ERROR(dwImage_getCUDA(&stereoImage[0], m_stereoFrames[0]));
            CHECK_DW_ERROR(dwImage_getCUDA(&stereoImage[1], m_stereoFrames[0]));
            CHECK_DW_ERROR(dwImageCUDA_mapToROI(stereoImage[0], stereoFrame, roi[0]));
            CHECK_DW_ERROR(dwImageCUDA_mapToROI(stereoImage[1], stereoFrame, roi[1]));
            gStereoImages[0] = m_stereoFrames[0];
            gStereoImages[1] = m_stereoFrames[1];
        }
        else
        {
            for (int32_t i = 0; i < DW_STEREO_SIDE_BOTH; ++i)
            {
                gStereoImages[i] = m_stereoCamera[i]->readFrame();
                if (!gStereoImages[i])
                {
                    m_stereoCamera[i]->resetCamera();

                    endStatus &= false;
                }
            }
        }
        return endStatus;
    }

    // -----------------------------------------------------------------------------
    // Render color to distance map (red closer, blue further)
    void renderColorMap(uint32_t side)
    {
        float32_t step = m_barLength / m_maxDisparity;
        for (uint32_t i = 0; i < m_maxDisparity; ++i)
        {
            // Normalize disparity to gray in [-1 , 1]
            float32_t grayNorm = 2 * static_cast<float32_t>(m_maxDisparity - i) / m_maxDisparity - 1;
            CHECK_DW_ERROR(dwRenderEngine_setLineWidth(10.f, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_setColor(fromNormGrayToColor(grayNorm), m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_2D, m_barLines[i].data(),
                                                 sizeof(dwVector2f),
                                                 0,
                                                 1,
                                                 m_renderEngine));
        }

        CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_16, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setColor(DW_RENDER_ENGINE_COLOR_WHITE, m_renderEngine));

        // Render furthest depth value (disparity = 1)
        char depthStr[10];
        sprintf(depthStr, ">%.1f", m_maxDistance);
        CHECK_DW_ERROR(dwRenderEngine_renderText2D(depthStr, dwVector2f{1.0f - m_barOffset + m_barLength, 0.97f}, m_renderEngine));

        // Render 6 intermediate values
        uint32_t numberOfSteps = 6;
        step                   = m_barLength / numberOfSteps;
        float32_t minDepth     = m_focalLength[side] * m_baseline / m_maxDisparity;
        float32_t stepDepth    = (m_maxDistance - minDepth) / numberOfSteps;

        for (uint32_t i = 0; i < numberOfSteps; ++i)
        {
            sprintf(depthStr, "%.1f", minDepth + (i * stepDepth));
            CHECK_DW_ERROR(dwRenderEngine_renderText2D(depthStr,
                                                       dwVector2f{1.0f - m_barOffset + i * step, 0.97f},
                                                       m_renderEngine));
        }

        CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_20, m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setLineWidth(1.f, m_renderEngine));
    }
};

//#######################################################################################
int main(int argc, const char** argv)
{
    ProgramArguments args(argc, argv, {
                                          ProgramArguments::Option_t{"rigconfig", (dw_samples::SamplesDataPath::get() + "/samples/stereo/full.json").c_str(), "Rig configuration file."}, ProgramArguments::Option_t{"video-left", (dw_samples::SamplesDataPath::get() + std::string{"/samples/stereo/left_1.h264"}).c_str(), "Left input video."}, ProgramArguments::Option_t{"video-right", (dw_samples::SamplesDataPath::get() + std::string{"/samples/stereo/right_1.h264"}).c_str(), "Right input video."}, ProgramArguments::Option_t{"sensor-left", "left_60FOV", "The index of the left camera sensor in the rig file"}, ProgramArguments::Option_t{"sensor-right", "right_60FOV", "The index of the right camera sensor in the rig file"}, ProgramArguments::Option_t{"level", "1", "Pyramid level for stereo algorithm"}, ProgramArguments::Option_t{"maxDisparity", "128", "Max disparity for stereo algorithm"}, ProgramArguments::Option_t{"maxDistance", "40.0", "Max distance in meters for depth visualization"}, ProgramArguments::Option_t{"single_side", "0", "If set to 1 only left disparity is computed."},
                                      });

    StereoApp app(args);

    app.initializeWindow("Stereo Disparity Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
