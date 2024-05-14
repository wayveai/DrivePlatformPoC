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
// SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "VisualizationNode.hpp"
#include "VisualizationNodeImpl.hpp"

#include <dwcgf/parameter/ParameterProvider.hpp>

namespace minipipeline
{

std::unique_ptr<VisualizationNode>
VisualizationNode::create(dw::framework::ParameterProvider& provider)
{
    auto constructorArguments       = dw::framework::createConstructorArguments<VisualizationNode>();
    VisualizationNodeParams& params = std::get<0>(constructorArguments);
    dw::framework::populateParameters<VisualizationNode>(constructorArguments, provider);

    return dw::framework::makeUniqueFromTuple<VisualizationNode>(std::move(constructorArguments));
}

VisualizationNode::VisualizationNode(const VisualizationNodeParams& params, const dwContextHandle_t ctx)
    : ExceptionSafeProcessNode(std::make_unique<VisualizationNodeImpl>(params, ctx))
{
}

} // namespace minipipeline

#include <dwcgf/node/NodeFactory.hpp>

DW_REGISTER_NODE(minipipeline::VisualizationNode)
