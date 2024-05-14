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
#include <framework/WindowGLFW.hpp>

// Include all relevant DriveWorks modules

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// RenderEngine sample
//------------------------------------------------------------------------------
class MySample : public DriveWorksSample
{
private:
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;

    static constexpr float32_t M_PI_F   = static_cast<float32_t>(M_PI);
    static const uint32_t TILE_COUNT    = 12;
    static const uint32_t TILES_PER_ROW = 3;

    uint32_t m_tiles[TILE_COUNT];
    dwImageGL m_eyeImage;
    dwImageGL m_sampleImage;
    bool m_toggleHelpKeyPressed = false;
    bool m_helpVisible          = false;
    uint32_t m_helpTile         = 0;
    dwRenderEngineTileState m_helpTileParams{};

    std::vector<dwVector3f> m_worldPoints;
    std::vector<dwVector3f> m_plotPoints;
    dwVector3f m_rainbowPoint{};
    dwVector3f m_tunnelPoint{};

    typedef struct ColoredPoint
    {
        dwVector3f pos;
        dwRenderEngineColorRGBA color;
    } ColoredPoint;
    std::vector<ColoredPoint> m_hugePointCloud;
    static const uint32_t HUGE_POINT_CLOUD_COUNT = 100000;
    bool m_hugePointCloudVisible                 = false;
    uint32_t m_hugePointCloudBufferId            = 0;

    uint32_t m_sphereBufferId         = 0;
    uint32_t m_ellipticalGridBufferId = 0;

public:
    MySample(const ProgramArguments& args)
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

    bool onInitialize() override
    {
        log("Starting render engine sample...\n");

        initializeDriveWorks(m_context);
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));

        dwRenderEngineParams params{};
        CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params,
                                                        static_cast<uint32_t>(getWindowWidth()),
                                                        static_cast<uint32_t>(getWindowHeight())));
        params.bufferSize = sizeof(dwVector4f) * 10000;
        CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

        dwRenderEngineTileState paramList[TILE_COUNT];
        for (uint32_t i = 0; i < TILE_COUNT; ++i)
            dwRenderEngine_initTileState(&paramList[i]);

        dwRenderEngine_addTilesByCount(m_tiles, TILE_COUNT, TILES_PER_ROW, paramList, m_renderEngine);

        dwRenderEngine_createImageGLFromFile(&m_eyeImage,
                                             (dw_samples::SamplesDataPath::get() +
                                              "/samples/renderer/eye.png")
                                                 .c_str(),
                                             m_renderEngine);

        dwRenderEngine_createImageGLFromFile(&m_sampleImage,
                                             (dw_samples::SamplesDataPath::get() +
                                              "/samples/renderer/cars.png")
                                                 .c_str(),
                                             m_renderEngine);

        dwRenderEngine_initTileState(&m_helpTileParams);
        m_helpTileParams.layout.viewport.width  = 450;
        m_helpTileParams.layout.viewport.height = 240;

        m_helpTileParams.layout.viewport.x     = 0;
        m_helpTileParams.layout.viewport.y     = 0;
        m_helpTileParams.backgroundColor       = {1.0f, 1.0f, 0.0f, 1.0f};
        m_helpTileParams.layout.positionType   = DW_RENDER_ENGINE_TILE_POSITION_TYPE_CENTER;
        m_helpTileParams.layout.positionLayout = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;
        m_helpTileParams.layout.sizeLayout     = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_ABSOLUTE;

        toggleHelp();

        dwRenderEngine_createBuffer(&m_hugePointCloudBufferId,
                                    DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                    sizeof(ColoredPoint),
                                    0,
                                    HUGE_POINT_CLOUD_COUNT,
                                    m_renderEngine);

        for (uint32_t i = 0; i < HUGE_POINT_CLOUD_COUNT; ++i)
        {
            m_hugePointCloud.push_back({{getRandom() * 2.0f - 1.0f,
                                         getRandom() * 2.0f - 1.0f,
                                         getRandom() * 2.0f - 1.0f},
                                        {1.0f,
                                         0.0f, 0.0f, 1.0f}});
        }

        dwRenderEngine_setBuffer(m_hugePointCloudBufferId,
                                 DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                 m_hugePointCloud.data(), sizeof(ColoredPoint),
                                 0, static_cast<uint32_t>(m_hugePointCloud.size()), m_renderEngine);
        return true;
    }

    void onReset() override
    {
        dwRenderEngine_reset(m_renderEngine);
    }

    void onRelease() override
    {
        dwRenderEngine_destroyBuffer(m_hugePointCloudBufferId, m_renderEngine);

        dwRenderEngine_destroyBuffer(m_ellipticalGridBufferId, m_renderEngine);

        dwRenderEngine_destroyBuffer(m_sphereBufferId, m_renderEngine);

        dwRenderEngine_destroyImageGL(&m_eyeImage, m_renderEngine);

        dwRenderEngine_destroyImageGL(&m_sampleImage, m_renderEngine);

        if (m_renderEngine != DW_NULL_HANDLE)
        {
            CHECK_DW_ERROR(dwRenderEngine_release(m_renderEngine));
        }

        CHECK_DW_ERROR(dwVisualizationRelease(m_viz));
        CHECK_DW_ERROR(dwRelease(m_context));
        CHECK_DW_ERROR(dwLogger_release());
    }

    void onResizeWindow(int width, int height) override
    {
        dwRenderEngine_reset(m_renderEngine);
        dwRectf rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderEngine_setBounds(rect, m_renderEngine);
    }

    void onMouseMove(float x, float y) override
    {
        uint32_t selectedTile = 0;
        dwVector2f screenPos{x, y};
        dwVector2f screenSize{static_cast<float32_t>(getWindowWidth()),
                              static_cast<float32_t>(getWindowHeight())};

        dwRenderEngine_getTileByScreenCoordinates(&selectedTile,
                                                  screenPos,
                                                  screenSize,
                                                  m_renderEngine);

        bool isTunnels = selectedTile == m_tiles[2];

        if (!isTunnels)
            return;

        float32_t z = m_tunnelPoint.z;
        dwRenderEngine_screenToWorld3D(&m_tunnelPoint, screenPos,
                                       screenSize,
                                       m_renderEngine);
        if (m_tunnelPoint.z > 0.0f)
        {
            m_tunnelPoint.z = z;
        }
    }

    void onMouseDown(int button, float x, float y, int mods) override
    {
        uint32_t selectedTile = 0;
        dwVector2f screenPos{x, y};
        dwVector2f screenSize{static_cast<float32_t>(getWindowWidth()),
                              static_cast<float32_t>(getWindowHeight())};

        dwRenderEngine_getTileByScreenCoordinates(&selectedTile,
                                                  screenPos,
                                                  screenSize,
                                                  m_renderEngine);

        bool isPlanarGrid    = selectedTile == m_tiles[9];
        bool isPlot          = selectedTile == m_tiles[5];
        bool isRainbowPoints = selectedTile == m_tiles[1];

        if (button == GLFW_MOUSE_BUTTON_2)
        {
            if (isPlanarGrid)
                m_worldPoints.clear();
            if (isPlot)
                m_plotPoints.clear();
            return;
        }
        (void)mods;
        dwVector3f worldPos{};

        dwRenderEngine_screenToWorld3D(&worldPos, screenPos,
                                       screenSize,
                                       m_renderEngine);
        if (isPlanarGrid)
            m_worldPoints.push_back(worldPos);
        if (isPlot)
            m_plotPoints.push_back(worldPos);
        if (isRainbowPoints)
            m_rainbowPoint = worldPos;

        std::cout << "3D pos in tile " << selectedTile << ": "
                  << worldPos.x
                  << " "
                  << worldPos.y
                  << " "
                  << worldPos.z
                  << std::endl;
    }

    void onKeyDown(int key, int scancode, int mods) override
    {
        (void)scancode;
        (void)mods;

        if (key == GLFW_KEY_F1)
        {
            m_toggleHelpKeyPressed = true;
        }
        else if (key == GLFW_KEY_F2)
        {
            toggleHugePointCloud();
        }
    }

    void onProcess() override
    {
    }

    void toggleHugePointCloud()
    {
        m_hugePointCloudVisible = !m_hugePointCloudVisible;
    }

    void toggleHelp()
    {
        if (!m_helpVisible)
        {
            dwRenderEngine_addTile(&m_helpTile, &m_helpTileParams, m_renderEngine);
        }
        else
        {
            dwRenderEngine_removeTile(m_helpTile, m_renderEngine);
        }
        m_helpVisible = !m_helpVisible;
    }

    void renderHelp()
    {
        if (!m_helpVisible)
            return;

        const char* help = R"(Press F2 to toggle large static point cloud (watch increased FPS)

Tiles from left to right:
1. Labeled points and boxes in 2D.
2. 3D points colored by value.
3. 3D lines and an arrow in 3D.
4. A triangle in 2D and color it by value.
5. 3D boxes.
6. A 2D plot.
7. A 2D histogram.
8. Images in 3D space.
9. Arrows in 2D along with images in 2D.
10. A planar grid in 3D, a labeled 3D point, and 3D text.
11. 2D ellipses.
12. An elliptical grid in 3D and 3D ellipses.
)";

        dwRenderEngine_setTile(m_helpTile, m_renderEngine);
        dwRenderEngine_resetTile(m_renderEngine);
        dwRenderEngine_setColor({0.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_24, m_renderEngine);
        dwRenderEngine_renderText2D("Help - Press F1 to dismiss", {0.0f, 0.1f}, m_renderEngine);
        dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_16, m_renderEngine);
        dwRenderEngine_renderText2D(help, {0.0f, 0.2f}, m_renderEngine);
    }

    /**
     * @brief renderLabeled2DPoints This function demonstrates how to use modelview matrix
     * and rendering of 2D points with labels.
     */
    void renderLabeled2DPoints()
    {

        const uint32_t count = 10;
        typedef struct
        {
            float32_t x;
            float32_t y;
            dwRenderEngineColorRGBA color;
        } Point2D;
        Point2D points[count];

        typedef struct
        {
            dwRectf rect;
            float32_t intensity;
        } Box2D;
        Box2D boxes[count];
        const uint32_t maxLength = 1024;
        char buffer[count][maxLength];
        char* labels[count];

        float32_t boxRadius = 0.02f;
        for (uint32_t i = 0; i < count; ++i)
        {
            points[i].x     = (i) / static_cast<float32_t>(count);
            points[i].y     = cosf(points[i].x);
            points[i].color = {getRandom(), getRandom(), getRandom(), 1.0f};

            boxes[i].rect.x      = points[i].x - boxRadius;
            boxes[i].rect.y      = points[i].y - boxRadius;
            boxes[i].rect.width  = boxRadius * 2.0f;
            boxes[i].rect.height = boxRadius * 2.0f;
            boxes[i].intensity   = (i) / static_cast<float32_t>(count);

            snprintf(buffer[i], maxLength, "%0.1f", points[i].x);
            labels[i] = &buffer[i][0];
        }

        dwMatrix4f translate{};
        translate.array[0]  = 1.0f;
        translate.array[5]  = 1.0f;
        translate.array[10] = 1.0f;
        translate.array[15] = 1.0f;
        translate.array[12] = 0.05f;
        translate.array[13] = -0.1f;

        dwRenderEngine_setModelView(&translate, m_renderEngine);

        dwRenderEngine_setPointSize(3.0f, m_renderEngine);

        dwRenderEngine_setColor({0.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_ATTRIBUTE_RGBA, 1.0f, m_renderEngine);
        dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D,
                                        points,
                                        sizeof(Point2D),
                                        0,
                                        const_cast<const char**>(labels),
                                        count,
                                        m_renderEngine);

        for (uint32_t i = 0; i < count; ++i)
        {
            snprintf(buffer[i], maxLength, "%d", i);
        }
        dwRenderEngine_setColor({1.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_INTENSITY, 1.0f, m_renderEngine);

        dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D,
                                        boxes,
                                        sizeof(Box2D),
                                        0,
                                        const_cast<const char**>(labels),
                                        count,
                                        m_renderEngine);
    }

    /**
     * @brief renderRandomRainbow3DPoints This function demonstrates how to render
     * 3D points and also how to color them by value.
     * It also demonstrates how to use the look at and perspective functions.
     * Try changing the setColorByValue to DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_Z.
     */
    void renderRandomRainbow3DPoints()
    {

        const uint32_t randomPointCount = 10000;
        dwVector4f randomPoints[randomPointCount];

        static float32_t start = 0.0f;
        auto distance          = [](const dwVector3f& a, const dwVector4f& b) -> float32_t {
            dwVector3f diff = {
                a.x - b.x,
                a.y - b.y,
                a.z - b.z};
            return sqrtf(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
        };
        for (uint32_t i = 0; i < randomPointCount; ++i)
        {
            randomPoints[i].x = getRandom() * 2 - 1;
            randomPoints[i].y = getRandom() * 2 - 1;
            randomPoints[i].z = cosf(randomPoints[i].y * 6.0f);
            randomPoints[i].w = distance(m_rainbowPoint,
                                         {randomPoints[i].x,
                                          randomPoints[i].y,
                                          randomPoints[i].z,
                                          1.0f}) /
                                    3.0f -
                                start; // The intensity color
        }
        start += 0.01f;

        dwRenderEngine_setBackgroundColor({0.25f, 0.25f, 0.25f, 1.0f}, m_renderEngine);

        dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_INTENSITY, 1.0f, m_renderEngine);

        dwRenderEngine_setLookAtByAngles(10.0f * M_PI_F / 180.0f,
                                         45.0f * M_PI_F / 180.0f,
                                         3.0f,
                                         {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setPerspectiveProjection(60.0f * M_PI_F / 180.0f, 16.0f / 9.0f, 0.01f, 1000.0f, m_renderEngine);

        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                              randomPoints,
                              sizeof(dwVector4f),
                              0,
                              randomPointCount,
                              m_renderEngine);
    }

    /**
     * @brief render3DLines This function demonstrates how to render
     * 3D lines.
     * It also demonstrates how to use the look at and perspective functions.
     */
    void render3DLines()
    {
        const uint32_t count = 8 * 5;
        dwVector3f lines[count * 2];

        for (uint32_t i = 0; i < count * 2; i += 8)
        {
            float32_t randomZ = getRandom() * 2 + m_tunnelPoint.z;

            lines[i + 0].z = randomZ;
            lines[i + 0].y = -1.0f + m_tunnelPoint.y;
            lines[i + 0].x = -1.0f + m_tunnelPoint.x;

            lines[i + 1].z = randomZ;
            lines[i + 1].y = 1.0f + m_tunnelPoint.y;
            lines[i + 1].x = -1.0f + m_tunnelPoint.x;

            lines[i + 2].z = randomZ;
            lines[i + 2].y = -1.0f + m_tunnelPoint.y;
            lines[i + 2].x = 1.0f + m_tunnelPoint.x;

            lines[i + 3].z = randomZ;
            lines[i + 3].y = 1.0f + m_tunnelPoint.y;
            lines[i + 3].x = 1.0f + m_tunnelPoint.x;

            lines[i + 4].z = randomZ;
            lines[i + 4].y = 1.0f + m_tunnelPoint.y;
            lines[i + 4].x = -1.0f + m_tunnelPoint.x;

            lines[i + 5].z = randomZ;
            lines[i + 5].y = 1.0f + m_tunnelPoint.y;
            lines[i + 5].x = 1.0f + m_tunnelPoint.x;

            lines[i + 6].z = randomZ;
            lines[i + 6].y = -1.0f + m_tunnelPoint.y;
            lines[i + 6].x = -1.0f + m_tunnelPoint.x;

            lines[i + 7].z = randomZ;
            lines[i + 7].y = -1.0f + m_tunnelPoint.y;
            lines[i + 7].x = 1.0f + m_tunnelPoint.x;
        }

        dwRenderEngine_setBackgroundColor({0.0f, 0.0f, 0.2f, 1.0f}, m_renderEngine);

        dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setLookAtByAngles(90.0f * M_PI_F / 180.0f,
                                         90.0f * M_PI_F / 180.0f,
                                         2.0f,
                                         {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setPerspectiveProjection(60.0f * M_PI_F / 180.0f, 16.0f / 9.0f, 0.01f, 5.0f, m_renderEngine);

        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D,
                              lines,
                              sizeof(dwVector3f),
                              0,
                              count,
                              m_renderEngine);

        const uint32_t arrowCount = 1;
        dwVector3f arrows[arrowCount * 2];

        arrows[0].x = 0.0f;
        arrows[0].z = 0.0f;
        arrows[0].y = -2.0f;

        arrows[1] = m_tunnelPoint;
        dwRenderEngine_setColor({0.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setLineWidth(1.0f, m_renderEngine);

        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ARROWS_3D,
                              arrows,
                              sizeof(dwVector3f) * 2,
                              0,
                              arrowCount,
                              m_renderEngine);
    }

    /**
     * @brief renderMoving2DTriangle Demonstrates how to render a 2D triangle that
     * moves around.
     */
    void renderMoving2DTriangle()
    {

        const uint32_t count = 1;
        dwVector2f triangles[count * 3];

        static dwVector2f center{0.5f, 0.5f};
        static dwVector2f direction{0.005f, 0.004f};

        float32_t radius = 0.1f;

        triangles[0].x = center.x + radius;
        triangles[0].y = center.y + radius;

        triangles[1].x = center.x - radius;
        triangles[1].y = center.y + radius;

        triangles[2].x = center.x + radius;
        triangles[2].y = center.y - radius;

        if (center.x >= 1.0f - radius ||
            center.x < radius)
        {
            direction.x = -direction.x;
        }

        if (center.y >= 1.0f - radius ||
            center.y < radius)
        {
            direction.y = -direction.y;
        }

        center.x += direction.x;
        center.y += direction.y;

        dwRenderEngine_setBackgroundColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_X, 0.5f, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_TRIANGLES_2D,
                              triangles,
                              sizeof(dwVector2f),
                              0,
                              count,
                              m_renderEngine);
    }

    /**
     * @brief renderRandom3DBoxes Demonstrates how to render 3D boxes.
     */
    void renderRandom3DBoxes()
    {

        const uint32_t boxCount = 3;
        typedef struct
        {
            float32_t aRandomFloatToDemonstrateOffset;
            bool aRandomBoolToDemonstrateOffset;
            dwVector3f pos;
            dwVector3f size;
            int32_t aRandomIntToDemonstrateStride;
        } Box3D;
        Box3D boxes[boxCount];
        for (uint32_t i = 0; i < boxCount; ++i)
        {
            boxes[i].pos.x = getRandom() * 4 - 2;
            boxes[i].pos.y = getRandom() * 4 - 2;
            boxes[i].pos.z = getRandom() * 4 - 2;

            boxes[i].size.x = 1.5f;
            boxes[i].size.y = 0.5f;
            boxes[i].size.z = 1.5f;
        }

        dwRenderEngine_setLineWidth(3.0f, m_renderEngine);
        dwRenderEngine_setColor({0.0f, 0.5f, 1.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setLookAtByAngles(10.0f * M_PI_F / 180.0f,
                                         10.0f * M_PI_F / 180.0f,
                                         5.0f,
                                         {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setPerspectiveProjection(60.0f * M_PI_F / 180.0f,
                                                16.0f / 9.0f, 0.01f, 1000.0f, m_renderEngine);

        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_3D,
                              boxes,
                              sizeof(Box3D), // NOTE THE STRIDE
                              // NOTE THE OFFSET POINTS TO WHERE THE FIRST
                              // FIELD OF THE DATA STARTS FOR EACH ELEMENT
                              offsetof(Box3D, pos),
                              boxCount,
                              m_renderEngine);
    }

    /**
     * @brief render2DPlot Demonstrates how to render a plot in 2D.
     */
    void render2DPlot()
    {
        const uint32_t dataCount = 500;
        dwVector2f sinCurve[dataCount];
        dwVector2f cosCurve[dataCount];
        static float32_t start = 1000.0f;

        dwVector2f pointOnCurve{start + dataCount / 2,
                                0.0f};
        pointOnCurve.y = sinf(pointOnCurve.x * 0.01f) * pointOnCurve.x;

        for (uint32_t i = 0; i < dataCount; ++i)
        {
            float32_t x = i + start;

            sinCurve[i].x = x;
            sinCurve[i].y = sinf(x * 0.01f) * x;
            cosCurve[i].x = x;
            cosCurve[i].y = cosf(x * 0.01f) * x;
        }
        start += 1.0f;

        dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
        dwRenderEngine_setColor({0.0f, 1.0f, 0.0f, 1.0f},
                                m_renderEngine);
        dwRenderEnginePlotType types[] = {
            DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP,
            DW_RENDER_ENGINE_PLOT_TYPE_LINESTRIP,
            DW_RENDER_ENGINE_PLOT_TYPE_POINTS};
        const char* firstLabel   = "sin(x)*x";
        const char* secondLabel  = "cos(x)*x";
        const char* halfwayPoint = "halfway point";
        const char* labels[]     = {
            firstLabel,
            secondLabel,
            halfwayPoint};

        const dwVector2f* curves[] = {
            sinCurve,
            cosCurve,
            &pointOnCurve};
        uint32_t strides[] = {sizeof(dwVector2f), sizeof(dwVector2f),
                              sizeof(dwVector2f)};
        uint32_t counts[]                = {dataCount, dataCount, 1};
        float32_t lineWidths[]           = {2.0f, 3.0f, 10.0f};
        dwRenderEngineColorRGBA colors[] = {
            {0.0f, 1.0f, 0.0f, 1.0f},
            {1.0f, 1.0f, 1.0f, 1.0f},
            {1.0f, 1.0f, 0.0f, 1.0f}};
        uint32_t offsets[] = {0, 0, 0};
        float32_t negInf   = -std::numeric_limits<float32_t>::infinity();
        dwRenderEngine_renderPlots2D(types,
                                     reinterpret_cast<const void**>(curves), strides,
                                     offsets,
                                     counts, colors, lineWidths, labels,
                                     3,
                                     {negInf, negInf, negInf, negInf},
                                     {0.0f, 0.0f, 1.0f, 1.0f},
                                     {1.0f, 0.0f, 0.0f, 1.0f},
                                     2.0f,
                                     "A Plot", "Time", "Money",
                                     m_renderEngine);

        if (m_plotPoints.size() > 0)
        {
            dwRenderEngine_setPointSize(5.0f, m_renderEngine);

            dwRenderEngine_setColor({0.2f, 0.9f, 1.0f, 1.0f}, m_renderEngine);
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                  m_plotPoints.data(),
                                  sizeof(dwVector3f),
                                  0,
                                  static_cast<uint32_t>(m_plotPoints.size()),
                                  m_renderEngine);
            const uint32_t labelSize = 100;
            char label[labelSize];
            for (const auto& point : m_plotPoints)
            {
                snprintf(label, labelSize, "(%0.1f, %0.1f)", point.x, point.y);
                dwRenderEngine_renderText3D(label, point, m_renderEngine);
            }
        }
    }

    /**
     * @brief render2DHistogram Demonstrates how to render a 2D histogram.
     */
    void render2DHistogram()
    {
        const uint32_t dataCount = 500;
        float32_t data[dataCount];

        static float32_t start = 0.0f;
        for (uint32_t i = 0; i < dataCount; ++i)
        {

            data[i] = fabsf(tanf((i + start) * 0.005f) * static_cast<float32_t>(gaussrand()));
        }

        start += 1.0f;
        dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
        dwRenderEngine_setColor({0.95f, 0.6f, 0.25f, 1.0f}, m_renderEngine);
        float32_t negInf = -std::numeric_limits<float32_t>::infinity();
        dwRenderEngine_renderPlot2D(DW_RENDER_ENGINE_PLOT_TYPE_HISTOGRAM,
                                    data, sizeof(float32_t), 0,
                                    dataCount, "Function",
                                    {negInf, negInf, negInf, 10.0f},
                                    {0.0f, 0.0f, 1.0f, 1.0f},
                                    {1.0f, 0.0f, 1.0f, 1.0f},
                                    2.0f,
                                    "Noise", " Center\n of Mass", "Value",
                                    m_renderEngine);
    }

    /**
     * @brief renderImagesIn3DSpace Demonstrates the use of two images in 3D and a line strip.
     */
    void renderImagesIn3DSpace()
    {

        static float32_t scale     = 10.0f;
        static float32_t direction = 0.1f;
        const float32_t speed      = 0.01f;
        if (scale >= 10.0f)
        {
            direction = -speed;
        }
        if (scale <= 6.0f)
        {
            direction = speed;
        }
        scale += direction;

        dwRenderEngine_setLookAtByAngles(90.0f * M_PI_F / 180.0f,
                                         90.0f * M_PI_F / 180.0f,
                                         scale,
                                         {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setPerspectiveProjection(60.0f * M_PI_F / 180.0f, 16.0f / 9.0f, 0.01f, 1000.0f, m_renderEngine);

        dwRenderEngine_renderImage3D(&m_eyeImage, {-1.25f, -1.0f, 1.0f, 1.0f}, &DW_IDENTITY_MATRIX4F,
                                     m_renderEngine);

        dwRenderEngine_renderImage3D(&m_eyeImage, {0.25f, -1.0f, 1.0f, 1.0f}, &DW_IDENTITY_MATRIX4F,
                                     m_renderEngine);

        dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_32, m_renderEngine);
        dwRenderEngine_renderText3D("o", {-0.25f, -2.0f, 0.0f}, m_renderEngine);

        const uint32_t smilePointCount = 100;
        dwVector3f smile[smilePointCount];

        float32_t segLength = 2 * M_PI_F / smilePointCount;
        uint32_t index      = 0;
        for (float32_t i = -M_PI_F; i < M_PI_F; i += segLength, ++index)
        {
            smile[index].x = i;
            smile[index].y = -cosf(smile[index].x) - 2;
            smile[index].z = 0.0f;
        }

        dwRenderEngine_setLineWidth(3.0f, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_3D,
                              smile,
                              sizeof(dwVector3f),
                              0,
                              smilePointCount,
                              m_renderEngine);
    }

    /**
     * @brief render2DImage Demonstrates how to render an image in 2D space with 2D arrows.
     */
    void render2DImage()
    {
        // NOTE: The use of setCoordinateRange2D to control
        // the normalization factor of the data rendered.
        // Because the range is the image space, the rendered
        // data is expected to be in image space coordinates.
        dwVector2f range{};
        range.x = m_sampleImage.prop.width;
        range.y = m_sampleImage.prop.height;
        dwRenderEngineTileLayout layout{};
        dwRenderEngine_getLayout(&layout, m_renderEngine);
        layout.useAspectRatio = true;
        layout.aspectRatio    = range.x / range.y;
        dwRenderEngine_setLayout(layout, m_renderEngine);
        dwRenderEngine_setCoordinateRange2D(range,
                                            m_renderEngine);

        dwRenderEngine_renderImage2D(&m_sampleImage, {0.0f, 0.0f, range.x, range.y},
                                     m_renderEngine);

        dwVector4f arrows[2];
        arrows[0].x = 0.75f * range.x;
        arrows[0].y = 0.75f * range.y;
        arrows[0].z = 0.5f * range.x;
        arrows[0].w = 0.5f * range.y;

        arrows[1].x = 0.0f;
        arrows[1].y = 0.0f;
        arrows[1].z = 0.25f * range.x;
        arrows[1].w = 0.85f * range.y;

        dwRenderEngine_setLineWidth(3.0f, m_renderEngine);

        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ARROWS_2D,
                              arrows,
                              sizeof(dwVector4f),
                              0,
                              2,
                              m_renderEngine);

        dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_24, m_renderEngine);

        dwRenderEngine_setColor({1.0f, 1.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_renderText2D("Look here", {arrows[1].z, arrows[1].w}, m_renderEngine);

        dwRenderEngine_renderText2D("and here", {arrows[0].x, arrows[0].y}, m_renderEngine);
    }

    void render3DPlanarGrid()
    {
        glEnable(GL_DEPTH_TEST);
        static float32_t angle = 0.0f;

        const float32_t speed      = 0.1f;
        static float32_t direction = speed;
        angle += direction;

        dwRenderEngine_setLookAtByAngles(angle * M_PI_F / 180.0f,
                                         15.0f * M_PI_F / 180.0f,
                                         5.0f,
                                         {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setPerspectiveProjection(60.0f * M_PI_F / 180.0f, 16.0f / 9.0f, 0.01f, 1000.0f, m_renderEngine);

        dwRenderEngine_setBackgroundColor({0.5f, 0.0f, 1.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setColor({0.5f, 0.0f, 1.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setLineWidth(3.0f, m_renderEngine);
        dwMatrix4f backGridTransform = DW_IDENTITY_MATRIX4F;
        backGridTransform.array[14]  = -0.1f;
        dwRenderEngine_renderPlanarGrid3D({0.0f, 0.0f, 5.0f, 5.0f}, 0.01f, 0.01f, &backGridTransform, m_renderEngine);
        dwRenderEngine_setColor({0.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setLineWidth(1.0f, m_renderEngine);
        dwRenderEngine_renderPlanarGrid3D({0.0f, 0.0f, 5.0f, 5.0f}, 0.75f, 0.75f, &DW_IDENTITY_MATRIX4F, m_renderEngine);

        dwRenderEngine_renderText3D("-x ground", {-5.0f, 0.0f, 0.25f}, m_renderEngine);

        dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setPointSize(2.0f, m_renderEngine);

        if (m_worldPoints.size() > 0)
        {
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                  m_worldPoints.data(),
                                  sizeof(dwVector3f),
                                  0,
                                  static_cast<uint32_t>(m_worldPoints.size()),
                                  m_renderEngine);
            const uint32_t labelSize = 100;
            char label[labelSize];
            for (const auto& point : m_worldPoints)
            {
                snprintf(label, labelSize, "(%0.1f, %0.1f, %0.1f)", point.x, point.y, point.z);
                dwRenderEngine_renderText3D(label, point, m_renderEngine);
            }
        }

        dwRenderEngine_setLineWidth(2.0f, m_renderEngine);
        for (int32_t i = 0; i < 3; i++)
        {
            float32_t localAxis[3] = {
                i == 0 ? 1.f : 0.f,
                i == 1 ? 1.f : 0.f,
                i == 2 ? 1.f : 0.f};
            dwVector3f arrow[2];

            // origin
            arrow[0].x = 0;
            arrow[0].y = 0;
            arrow[0].z = 0;

            arrow[1].x = localAxis[0];
            arrow[1].y = localAxis[1];
            arrow[1].z = localAxis[2];

            dwRenderEngine_setColor({localAxis[0], localAxis[1], localAxis[2], 1.0f}, m_renderEngine);
            dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ARROWS_3D,
                                  arrow,
                                  sizeof(dwVector3f) * 2,
                                  0,
                                  1,
                                  m_renderEngine);
        }

        dwRenderEngineTileState state{};
        dwRenderEngine_getState(&state, m_renderEngine);

        dwRenderEngine_setProjection(&DW_IDENTITY_MATRIX4F, m_renderEngine);
        dwRenderEngine_setModelView(&DW_IDENTITY_MATRIX4F, m_renderEngine);
        dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
        bool hasPoints = !m_worldPoints.empty();
        dwRenderEngine_renderText2D(hasPoints ? "Right click to clear..." : "Click on the grid...",
                                    {0, 0.1f}, m_renderEngine);

        dwRenderEngine_setState(&state, m_renderEngine);
    }

    /**
     * @brief render2DEllipses Demonstrates rendering 2D text and ellipses.
     */
    void render2DEllipses()
    {

        dwMatrix4f rotation{};
        static float32_t angle = 0.0f;
        angle += 1.0f * M_PI_F / 180.0f;
        rotation.array[0] = cosf(angle);
        rotation.array[1] = -sinf(angle);
        rotation.array[4] = sinf(angle);
        rotation.array[5] = cosf(angle);

        rotation.array[10] = 1.0f;
        rotation.array[15] = 1.0f;

        dwRenderEngine_setModelView(&rotation, m_renderEngine);
        dwRenderEngine_setLineWidth(3.0f, m_renderEngine);

        const uint32_t count = 5;
        dwRectf ellipses[count];
        const uint32_t maxLength = 100;
        char score[maxLength];
        for (uint32_t i = 0; i < count; ++i)
        {
            ellipses[i].x      = 0.5f;
            ellipses[i].y      = 0.5f;
            ellipses[i].width  = 0.1f * i;
            ellipses[i].height = 0.1f * i;
            snprintf(score, maxLength, "%d", 1000 - (i + 1) * 100);
            if (i < count - 1)
                dwRenderEngine_renderText2D(score, {0.5f, ellipses[i].y + 0.1f * (i + 1)},
                                            m_renderEngine);
        }
        dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_X, 1.0f, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ELLIPSES_2D,
                              ellipses,
                              sizeof(dwRectf),
                              0,
                              count,
                              m_renderEngine);

        for (uint32_t i = 0; i < count; ++i)
        {
            ellipses[i].x      = 0.5f;
            ellipses[i].y      = 0.5f;
            ellipses[i].width  = 0.005f * i;
            ellipses[i].height = 0.005f * i;
        }

        dwRenderEngine_setColor({1.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ELLIPSES_2D,
                              ellipses,
                              sizeof(dwRectf),
                              0,
                              count,
                              m_renderEngine);
    }

    /**
     * @brief render3DEllipticalGrid Demonstrates how to draw a 3D elliptical grid.
     */
    void render3DEllipticalGrid()
    {

        const uint32_t sphereCount = 2;
        typedef struct
        {
            dwVector3f pos;
            dwVector2f size;
        } Ellipse3D;

        Ellipse3D ellipses[sphereCount];
        ellipses[0].pos.x  = -1.0f;
        ellipses[0].pos.y  = 0.0f;
        ellipses[0].pos.z  = 1.0f;
        ellipses[0].size.x = 0.25f;
        ellipses[0].size.y = 0.25f;

        ellipses[1].pos.x  = -1.0f;
        ellipses[1].pos.y  = 3.0f;
        ellipses[1].pos.z  = 1.0f;
        ellipses[1].size.x = 0.5f;
        ellipses[1].size.y = 0.5f;

        static float32_t angle = 0.0f;

        const float32_t speed      = 1.0f;
        static float32_t direction = speed;

        if (angle >= 89.0f)
        {
            direction = -speed;
        }
        if (angle < 0.0f)
        {
            direction = speed;
        }
        angle += direction;

        dwRenderEngine_setLookAtByAngles(0.0f * M_PI_F / 180.0f,
                                         angle * M_PI_F / 180.0f,
                                         5.0f,
                                         {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setPerspectiveProjection(60.0f * M_PI_F / 180.0f,
                                                16.0f / 9.0f, 0.01f, 1000.0f, m_renderEngine);

        dwRenderEngine_setBackgroundColor({0.1f,
                                           0.1f,
                                           0.1f,
                                           1.0f},
                                          m_renderEngine);
        dwRenderEngine_setColor({1.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setLineWidth(3.0f, m_renderEngine);

        static bool firstTime = true;
        // Spheres can be rather large in memory.
        // Therefore, to increase perf, we will create a buffer
        // and then set it once.
        if (firstTime)
        {
            firstTime = false;
            dwRenderEngine_createBuffer(&m_sphereBufferId, DW_RENDER_ENGINE_PRIMITIVE_TYPE_ELLIPSES_3D,
                                        sizeof(Ellipse3D),
                                        0,
                                        sphereCount, m_renderEngine);

            dwRenderEngine_setBuffer(m_sphereBufferId,
                                     DW_RENDER_ENGINE_PRIMITIVE_TYPE_ELLIPSES_3D,
                                     ellipses,
                                     sizeof(Ellipse3D),
                                     0,
                                     sphereCount, m_renderEngine);

            // Here is an example of creating a grid in a buffer so that it is not created every render call
            dwRenderEngine_createBuffer(&m_ellipticalGridBufferId,
                                        DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINES_3D,
                                        sizeof(dwVector3f) * 2,
                                        0,
                                        10000,
                                        m_renderEngine);
            dwRenderEngine_setBufferEllipticalGrid3D(m_ellipticalGridBufferId,
                                                     {0.0f, 0.0f, 10.0f, 10.0f},
                                                     0.5f, 0.5f, &DW_IDENTITY_MATRIX4F, m_renderEngine);
        }
        uint32_t primitiveCount = 0;
        dwRenderEngine_getBufferMaxPrimitiveCount(&primitiveCount, m_ellipticalGridBufferId, m_renderEngine);
        dwRenderEngine_renderBuffer(m_ellipticalGridBufferId, primitiveCount, m_renderEngine);

        dwRenderEngine_renderEllipticalGrid3D({0.0f, 0.0f, 10.0f, 10.0f},
                                              0.5f, 0.5f, &DW_IDENTITY_MATRIX4F, m_renderEngine);

        dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_X, 1.0f, m_renderEngine);

        dwRenderEngine_renderBuffer(m_sphereBufferId,
                                    sphereCount,
                                    m_renderEngine);

        const uint32_t maxLength = 100;
        char label[maxLength];
        dwRenderEngine_setColor({1.0f, 1.0f, 0.0f, 1.0f}, m_renderEngine);
        for (uint32_t i = 0; i < sphereCount; ++i)
        {
            snprintf(label, maxLength, "Sphere %d", i + 1);

            dwRenderEngine_renderText3D(label, ellipses[i].pos, m_renderEngine);
        }
    }

    void renderHugePointCloud()
    {
        static float32_t angle = 0.0f;

        const float32_t speed      = 0.01f;
        static float32_t direction = speed;

        if (angle >= 89.0f)
        {
            direction = -speed;
        }
        if (angle < 0.0f)
        {
            direction = speed;
        }
        angle += direction;

        dwRenderEngineTileState oldState{};
        dwRenderEngine_getState(&oldState, m_renderEngine);
        dwRenderEngine_setLookAtByAngles(0.0f * M_PI_F / 180.0f,
                                         angle * M_PI_F / 180.0f,
                                         5.0f,
                                         {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, m_renderEngine);

        dwRenderEngine_setPerspectiveProjection(60.0f * M_PI_F / 180.0f,
                                                16.0f / 9.0f, 0.01f, 1000.0f, m_renderEngine);
        dwRenderEngine_setBackgroundColor({0.0f, 0.0f, 0.0f, 1.0f}, m_renderEngine);
        dwRenderEngine_setColorByValue(DW_RENDER_ENGINE_COLOR_BY_VALUE_MODE_ATTRIBUTE_RGBA, 1.0f, m_renderEngine);
        dwRenderEngine_renderBuffer(m_hugePointCloudBufferId,
                                    static_cast<uint32_t>(m_hugePointCloud.size()),
                                    m_renderEngine);
        dwRenderEngine_setState(&oldState, m_renderEngine);
    }

    void onRender() override
    {
        if (m_toggleHelpKeyPressed)
        {
            toggleHelp();
            m_toggleHelpKeyPressed = false;
        }

        dwRenderEngine_reset(m_renderEngine);

        if (m_hugePointCloudVisible)
        {
            dwRenderEngine_setTile(0, m_renderEngine);

            renderHugePointCloud();
        }
        else
        {

            dwRenderEngine_setTile(m_tiles[0], m_renderEngine);

            renderLabeled2DPoints();

            dwRenderEngine_setTile(m_tiles[1], m_renderEngine);

            renderRandomRainbow3DPoints();

            dwRenderEngine_setTile(m_tiles[2], m_renderEngine);

            render3DLines();

            dwRenderEngine_setTile(m_tiles[3], m_renderEngine);

            renderMoving2DTriangle();

            dwRenderEngine_setTile(m_tiles[4], m_renderEngine);

            renderRandom3DBoxes();

            dwRenderEngine_setTile(m_tiles[5], m_renderEngine);

            render2DPlot();

            dwRenderEngine_setTile(m_tiles[6], m_renderEngine);

            render2DHistogram();

            dwRenderEngine_setTile(m_tiles[7], m_renderEngine);

            renderImagesIn3DSpace();

            dwRenderEngine_setTile(m_tiles[8], m_renderEngine);

            render2DImage();

            dwRenderEngine_setTile(m_tiles[9], m_renderEngine);

            render3DPlanarGrid();

            dwRenderEngine_setTile(m_tiles[10], m_renderEngine);

            render2DEllipses();

            dwRenderEngine_setTile(m_tiles[11], m_renderEngine);

            render3DEllipticalGrid();
        }

        renderHelp();

        dwRenderEngine_setTile(0, m_renderEngine);

        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }

    float32_t getRandom()
    {
        return static_cast<float32_t>(rand()) / RAND_MAX;
    }

    const int NSUM = 25;

    double gaussrand()
    {
        float64_t x = 0;
        int32_t i;
        for (i = 0; i < NSUM; i++)
            x += static_cast<float64_t>(rand()) / RAND_MAX;

        x -= NSUM / 2.0;
        x /= sqrt(NSUM / 12.0);

        return x;
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{

    // -------------------
    // define all arguments used by the application
    // parse user given arguments and bail out if there is --help request or proceed
    ProgramArguments args(argc, argv,
                          {},
                          "");

    // -------------------
    // initialize and start a window application (with offscreen support if required)
    MySample app(args);

    app.setProcessRate(10000);
    app.initializeWindow("Render Engine Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
