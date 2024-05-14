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

#include "RenderUtils.hpp"

namespace renderutils
{
void renderFPS(dwRenderEngineHandle_t renderEngine, const float32_t fps)
{
    if (renderEngine != DW_NULL_HANDLE)
    {
        CHECK_DW_ERROR(dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_20, renderEngine));

        // render
        constexpr uint32_t textBufferSize = 128;
        char fpsText[textBufferSize];
        snprintf(fpsText, textBufferSize, "FPS: %.02f", fps);
        dwVector2f range{.x = 1.0f, .y = 1.0f};
        dwVector2f fpsTextPos = {.x = 0.f, .y = 1.f};

        renderText(fpsText, fpsTextPos, range, renderEngine);
    }
}

void renderText(const char* textBuff,
                const dwVector2f& textPos,
                const dwVector2f& range,
                dwRenderEngineHandle_t renderEngine)
{
    // store previous tile
    uint32_t previousTile = 0;
    CHECK_DW_ERROR(dwRenderEngine_getTile(&previousTile, renderEngine));

    // select default tile
    CHECK_DW_ERROR(dwRenderEngine_setTile(0, renderEngine));

    // get default tile state
    dwRenderEngineTileState previousDefaultState{};
    CHECK_DW_ERROR(dwRenderEngine_getState(&previousDefaultState, renderEngine));

    // set text render settings
    CHECK_DW_ERROR(dwRenderEngine_setModelView(&DW_IDENTITY_MATRIX4F, renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, renderEngine));

    // render
    dwRenderEngine_renderText2D(textBuff, textPos, renderEngine);

    // restore previous settings
    CHECK_DW_ERROR(dwRenderEngine_setState(&previousDefaultState, renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setTile(previousTile, renderEngine));
}
}
