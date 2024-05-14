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

#include "HelloWorldNodeImpl.hpp"
#include <dwshared/dwfoundation/dw/core/logger/Logger.hpp>

namespace dw
{
namespace framework
{

constexpr char HelloWorldNodeImpl::LOG_TAG[];

HelloWorldNodeImpl::HelloWorldNodeImpl(const HelloWorldNodeParams& params, const dwContextHandle_t ctx)
    : m_params(params)
    , m_ctx(ctx)
{
    // Create input/output ports
    NODE_INIT_OUTPUT_PORT("VALUE_0"_sv);
    NODE_INIT_OUTPUT_PORT("VALUE_1"_sv);

    // Init passes
    NODE_REGISTER_PASS("PROCESS"_sv, [this]() {
        return process();
    });

    DW_LOGD << "HelloWorldNodeImpl: created" << Logger::State::endl;
}

///////////////////////////////////////////////////////////////////////////////////////
HelloWorldNodeImpl::~HelloWorldNodeImpl()
{
    DW_LOGD << "HelloWorldNodeImpl: destructed" << Logger::State::endl;
}

dwStatus HelloWorldNodeImpl::reset()
{
    m_port0Value = 0;
    m_port1Value = 10000;
    return Base::reset();
}

///////////////////////////////////////////////////////////////////////////////////////
dwStatus HelloWorldNodeImpl::process()
{
    auto& outPort0 = NODE_GET_OUTPUT_PORT("VALUE_0"_sv);
    auto& outPort1 = NODE_GET_OUTPUT_PORT("VALUE_1"_sv);
    if (outPort0.isBufferAvailable() && outPort1.isBufferAvailable())
    {
        *outPort0.getBuffer() = m_port0Value++;
        DW_LOGD << "[Epoch " << m_epochCount << "] Sent value0 = " << *outPort0.getBuffer() << Logger::State::endl;
        outPort0.send();

        *outPort1.getBuffer() = m_port1Value--;
        DW_LOGD << "[Epoch " << m_epochCount << "] Sent value1 = " << *outPort1.getBuffer() << Logger::State::endl;
        outPort1.send();
    }
    DW_LOGD << "[Epoch " << m_epochCount++ << "] Greetings from HelloWorldNodeImpl: Hello " << m_params.name.c_str() << "!" << Logger::State::endl;
    return DW_SUCCESS;
}

} // namespace framework
} // namespace dw
