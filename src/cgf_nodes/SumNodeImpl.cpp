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
// SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "SumNodeImpl.hpp"
#include <dwshared/dwfoundation/dw/core/logger/Logger.hpp>

namespace dw
{
namespace framework
{

constexpr char SumNodeImpl::LOG_TAG[];

SumNodeImpl::SumNodeImpl(const SumNodeParams& params, const dwContextHandle_t ctx)
    : m_params(params)
    , m_ctx(ctx)
{
    // Create input/output ports
    NODE_INIT_INPUT_PORT("VALUE_0"_sv);
    NODE_INIT_INPUT_PORT("VALUE_1"_sv);

    // Init passes
    NODE_REGISTER_PASS("PROCESS"_sv, [this]() {
        return process();
    });

    DW_LOGD << "SumNodeImpl: created" << Logger::State::endl;
}

///////////////////////////////////////////////////////////////////////////////////////
SumNodeImpl::~SumNodeImpl()
{
    DW_LOGD << "SumNodeImpl: destructed" << Logger::State::endl;
}

///////////////////////////////////////////////////////////////////////////////////////
dwStatus SumNodeImpl::process()
{
    auto& inPort0 = NODE_GET_INPUT_PORT("VALUE_0"_sv);
    auto& inPort1 = NODE_GET_INPUT_PORT("VALUE_1"_sv);
    if (inPort0.isBufferAvailable() && inPort1.isBufferAvailable())
    {
        auto inputValue0 = *inPort0.getBuffer();
        auto inputValue1 = *inPort1.getBuffer();
        DW_LOGD << "[Epoch " << m_epochCount << "]"
                << " Received " << inputValue0 << " from input VALUE_0"
                << ", received " << inputValue1 << " from input VALUE_1."
                << " Add together: " << (inputValue0 + inputValue1) << "!" << Logger::State::endl;
    }

    ++m_epochCount;
    return DW_SUCCESS;
}

} // namespace framework
} // namespace dw
