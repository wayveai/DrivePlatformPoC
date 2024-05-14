/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2022 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SMP_CONTENT_LAYOUT_HPP_
#define SMP_CONTENT_LAYOUT_HPP_

#include <dwcgf/Types.hpp>
#include <dwcgf/Exception.hpp>
#include <dwvisualization/core/RenderEngine.h>

namespace minipipeline
{

/**
 * @brief ContentLayout provides 2 types of content layouts, single content layout
 *        and multiple content layout. dwRenderEngine requires create tiles to render
 *        contents. ContentLayout also helps renderer to add and to remove tiles
 *        during rendering. Maximum sub-tile number is 9. For sub-tiles, maximum tiles
 *        per row is 3.
 */
class ContentLayout
{
public:
    /**
     * @brief Construct a new Content Layout object. By default, there is a main tile 
     *        for each ContentLayout. Render all tiles use default tile stage.
     * 
     * @param renderEngineHandle render engine handle.
     * @param subTileCount sub-tile count. When subTileCount is 0, ContentLayout is a
     *                     single content layout. When subTileCount is larger than 0,
     *                     ContentLayout is a multiple content layout with the same 
     *                     number contents as subTileCount.
     */
    ContentLayout(dwRenderEngineHandle_t renderEngineHandle, uint32_t subTileCount)
        : m_renderEngineHanle(renderEngineHandle)
        , m_tile{}
        , m_tileState{}
        , m_subTileCount(std::min(subTileCount, MAX_SUB_TILES_COUNT))
        , m_subTiles(subTileCount, 0)
        , m_subTileStates(subTileCount, dwRenderEngineTileState{})
    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_initTileState(&m_tileState));

        for (size_t idx = 0; idx < m_subTileCount; ++idx)
        {
            FRWK_CHECK_DW_ERROR(dwRenderEngine_initTileState(&m_subTileStates[idx]));
        }
    }

    /**
     * @brief Construct a new Content Layout object.
     *        See description of ContentLayout(dwRenderEngineHandle_t, uint32_t)
     * 
     * @param renderEngineHandle render engine handle.
     */
    ContentLayout(dwRenderEngineHandle_t renderEngineHandle)
        : ContentLayout(renderEngineHandle, 0)
    {
    }

    /**
     * @brief Add tiles as needed.
     */
    void initialize()
    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_addTile(&m_tile, &m_tileState, m_renderEngineHanle));
        if (m_subTileCount > 0)
        {
            FRWK_CHECK_DW_ERROR(dwRenderEngine_addTilesByCount(m_subTiles.data(), m_subTileCount, MAX_SUB_TILES_PER_ROW, m_subTileStates.data(), m_renderEngineHanle));
        }
    }

    /**
     * @brief Remove all added tiles.
     */
    void release()
    {
        for (auto& subTile : m_subTiles)
        {
            FRWK_CHECK_DW_ERROR(dwRenderEngine_removeTile(subTile, m_renderEngineHanle));
        }
        FRWK_CHECK_DW_ERROR(dwRenderEngine_removeTile(m_tile, m_renderEngineHanle));
    }

    /**
     * @brief Set the main tile as current tile.
     */
    void setCurrentTile()
    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_setTile(m_tile, m_renderEngineHanle));
    }

    /**
     * @brief Render background for current tile. User should call this function at the
     *        beginning of each frame rendering.
     */
    void resetCurrentTileBackground()
    {
        FRWK_CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngineHanle));
    }

    /**
     * @brief Set a sub-tile as current tile.
     * 
     * @param tileIdx The tile index of the tile that user want to set it as current tile.
     */
    void setCurrentSubTile(size_t tileIdx)
    {
        if (tileIdx < m_subTileCount)
        {
            FRWK_CHECK_DW_ERROR(dwRenderEngine_setTile(m_subTiles[tileIdx], m_renderEngineHanle));
        }
    }

private:
    static constexpr uint32_t MAX_SUB_TILES_COUNT   = 9;
    static constexpr uint32_t MAX_SUB_TILES_PER_ROW = 3;

    using SubTileVector      = dw::core::VectorFixed<uint32_t, MAX_SUB_TILES_COUNT>;
    using SubTileStateVector = dw::core::VectorFixed<dwRenderEngineTileState, MAX_SUB_TILES_COUNT>;

    dwRenderEngineHandle_t m_renderEngineHanle;
    uint32_t m_tile;
    dwRenderEngineTileState m_tileState;
    uint32_t m_subTileCount;
    SubTileVector m_subTiles;
    SubTileStateVector m_subTileStates;
};

} // namespace minipipeline
#endif // SMP_CONTENT_LAYOUT_HPP_
