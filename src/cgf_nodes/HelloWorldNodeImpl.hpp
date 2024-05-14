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
// SPDX-FileCopyrightText: Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef HELLOWORLD_NODE_IMPL_HPP_
#define HELLOWORLD_NODE_IMPL_HPP_

#include "HelloWorldNode.hpp"
#include <dwcgf/node/SimpleNodeT.hpp>

namespace dw
{
namespace framework
{

class HelloWorldNodeImpl : public SimpleNodeT<HelloWorldNode>
{
public:
    static constexpr char LOG_TAG[] = "HelloWorldNode";
    using Base                      = SimpleNodeT<HelloWorldNode>;

    // Initialization and destruction
    HelloWorldNodeImpl(const HelloWorldNodeParams& params, const dwContextHandle_t ctx);
    ~HelloWorldNodeImpl() override;

    dwStatus reset() final;

private:
    // Passes functions
    dwStatus process();

    // Internal states of node
    int m_port0Value{0};
    int m_port1Value{10000};
    size_t m_epochCount{0};
    HelloWorldNodeParams m_params{};
    dwContextHandle_t m_ctx{DW_NULL_HANDLE};
};

} // namespace framework
} // namespace dw

#endif // HELLOWORLD_NODE_IMPL_HPP_
