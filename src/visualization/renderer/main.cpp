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
// SPDX-FileCopyrightText: Copyright (c) 2017-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <dw/core/base/Version.h>
#include <framework/DriveWorksSample.hpp>

// Include all relevant DriveWorks modules

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Template of a sample. Put some description what the sample does here
//------------------------------------------------------------------------------
class RendererSample : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t m_context                           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_visualizationContext = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer                         = DW_NULL_HANDLE;
    dwRenderBufferHandle_t m_renderBuffer                 = DW_NULL_HANDLE;

public:
    RendererSample(const ProgramArguments& args)
        : DriveWorksSample(args) {}

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

    /// -----------------------------
    /// Initialize everything of a sample here incl. SDK components
    /// -----------------------------
    bool onInitialize() override
    {
        log("Starting my sample application...\n");

        initializeDriveWorks(m_context);

        CHECK_DW_ERROR(dwVisualizationInitialize(&m_visualizationContext, m_context));
        CHECK_DW_ERROR(dwRenderer_initialize(&m_renderer, m_visualizationContext));

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        // Prepare some data for rendering
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        dwRenderBuffer_initialize(&m_renderBuffer, layout, DW_RENDER_PRIM_POINTLIST, 4, m_visualizationContext);

        // update the data
        float32_t* map;
        uint32_t maxVerts, stride;

        if (dwRenderBuffer_map(&map, &maxVerts, &stride, m_renderBuffer) == DW_SUCCESS)
        {
            map[0] = 0.60f;
            map[1] = 0.25f;
            map[2] = 0.60f;
            map[3] = 0.75f;
            map[4] = 0.90f;
            map[5] = 0.25f;
            map[6] = 0.90f;
            map[7] = 0.75f;

            dwRenderBuffer_unmap(maxVerts, m_renderBuffer);
        }

        // Set some renderer defaults
        dwRect rect;
        rect.width  = getWindowWidth();
        rect.height = getWindowHeight();
        rect.x      = 0;
        rect.y      = 0;

        dwRenderer_setRect(rect, m_renderer);
        dwRenderer_setPointSize(10.0f, m_renderer);
        dwRenderer_setColor(DW_RENDERER_COLOR_RED, m_renderer);
        return true;
    }

    ///------------------------------------------------------------------------------
    /// This method is executed when user presses `R`, it indicates that sample has to reset
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        dwRenderer_reset(m_renderer);
    }

    ///------------------------------------------------------------------------------
    /// This method is executed on release, free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        if (m_renderBuffer != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderBuffer_release(m_renderBuffer));
        }

        if (m_renderer != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderer_release(m_renderer));
        }

        if (m_visualizationContext != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwVisualizationRelease(m_visualizationContext));
        }

        if (m_context != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRelease(m_context));
        }

        CHECK_DW_ERROR(dwLogger_release());
    }

    void onResizeWindow(int width, int height) override
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;

        dwRenderer_setRect(rect, m_renderer);

        glViewport(0, 0, width, height);
    }

    void onProcess() override
    {
        static float xOffset;
        static float yOffset;
        static float angle = 0.0;

        xOffset = 0.01f * cosf(angle);
        yOffset = 0.01f * sinf(angle);

        // update the data
        float32_t* map;
        uint32_t maxVerts, stride;

        if (dwRenderBuffer_map(&map, &maxVerts, &stride, m_renderBuffer) == DW_SUCCESS)
        {
            map[0] = 0.60f + xOffset;
            map[1] = 0.25f + yOffset;
            map[2] = 0.60f + xOffset;
            map[3] = 0.75f + yOffset;
            map[4] = 0.90f + xOffset;
            map[5] = 0.25f + yOffset;
            map[6] = 0.90f + xOffset;
            map[7] = 0.75f + yOffset;

            dwRenderBuffer_unmap(maxVerts, m_renderBuffer);

            angle += 0.001f;
        }
    }

    void render2Dtests()
    {
        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer);
        dwRenderer_renderBuffer(m_renderBuffer, m_renderer);

        dwRenderer_setColor(DW_RENDERER_COLOR_RED, m_renderer);

        int yCoord = 20;
        int offset = 90;

        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_8, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 8", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_12, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 12", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 16", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_20, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 20", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_24, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 24", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_32, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 32", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_48, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 48", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_64, m_renderer);
        dwRenderer_renderText(20, yCoord, "Verdana 64", m_renderer);
        yCoord += offset;
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_20, m_renderer);
        dwRenderer_renderText(20, yCoord, "Test line break\n\tand tabs", m_renderer);
    }

    void onRender() override
    {
        glDepthFunc(GL_LESS);

        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        render2Dtests();

        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer);
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, m_renderer);
        dwRenderer_renderText(10, getWindowHeight() - 20, "(ESC to quit)", m_renderer);
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
    ProgramArguments args(argc, argv,
                          {},
                          "This sample shows how to use the renderer.");

    // -------------------
    // initialize and start a window application (with offscreen support if required)
    RendererSample app(args);

    app.initializeWindow("Renderer Sample", 1280, 800, args.enabled("offscreen"));
    app.setProcessRate(10000);

    return app.run();
}
