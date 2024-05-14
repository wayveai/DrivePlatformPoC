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
// SPDX-FileCopyrightText: Copyright (c) 2017-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_COMMON_SIMPLERENDERER_HPP_
#define SAMPLES_COMMON_SIMPLERENDERER_HPP_

#include <framework/Checks.hpp>

#include <dwvisualization/core/Renderer.h>
#include <dwvisualization/core/Visualization.h>
#include <dw/image/Image.h>
#include <dwvisualization/image/Image.h>

#include <vector>

namespace dw_samples
{
namespace common
{

class SimpleRenderer
{
public:
    SimpleRenderer(dwRendererHandle_t renderer, dwVisualizationContextHandle_t ctx);
    ~SimpleRenderer();

    /// Specifies a 2D line segment between point a and point b, in floating point coordinates
    typedef struct
    {
        dwVector2f a;
        dwVector2f b;
    } dwLineSegment2Df;

    void setScreenRect(dwRect screenRect);

    void setRenderBufferNormCoords(float32_t width, float32_t height, dwRenderBufferPrimitiveType type);

    void setColor(const dwVector4f color);

    void render(float32_t const* vertices2D, uint32_t numVertices, dwRenderBufferPrimitiveType type);

    void renderQuad(dwImageGL* input);

    void renderText(uint32_t textX, uint32_t textY, const dwVector4f color, std::string text,
                    dwRendererFonts font = DW_RENDER_FONT_VERDANA_20);

    void setRectangleCoords(float32_t* coords, dwRect rectangle, uint32_t vertexStride);

    void renderRectangle(const dwRect& rectangle, const dwVector4f color);

    void renderRectangles(const dwRect* rectangles, uint32_t numBoxes);

    void renderRectangles(const std::vector<dwRect>& rectangles);

    void renderRectanglesWithLabels(const std::vector<std::pair<dwRect, std::string>>& rectanglesWithLabels,
                                    float32_t normalizationWidth, float32_t normalizationHeight);

    void renderLineSegments(const std::vector<dwLineSegment2Df>& segments, float32_t lineWidth, const dwVector4f color);

    void renderPolyline(const std::vector<dwVector2f>& points, float32_t lineWidth, const dwVector4f color);

    void renderPoints(const std::vector<dwVector2f>& points, float32_t pointSize, const dwVector4f color);

private:
    static constexpr auto m_maxVertexCount = 20000u;

    // one render buffer per primitive
    void fillBuffer(float32_t const* vertices2D, uint32_t numVertices, dwRenderBufferPrimitiveType type) const;
    dwRenderBufferHandle_t m_renderBuffer[5];

    // handle to a renderer, not owned by this class
    dwRendererHandle_t m_renderer;
};

} // namespace common
} // namespace dw_samples

#endif
