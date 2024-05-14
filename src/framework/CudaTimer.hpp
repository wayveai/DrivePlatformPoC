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
// SPDX-FileCopyrightText: Copyright (c) 2016-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_COMMON_CUDATIMER_HPP_
#define SAMPLES_COMMON_CUDATIMER_HPP_

#include <driver_types.h>
#include <cuda_runtime.h>

namespace dw_samples
{
namespace common
{

class CudaTimer
{
public:
    CudaTimer()
        : m_isTimeValid(false)
        , m_stream(static_cast<cudaStream_t>(0))
    {
        cudaEventCreateWithFlags(&m_start, cudaEventBlockingSync);
        cudaEventCreateWithFlags(&m_stop, cudaEventBlockingSync);
    }
    ~CudaTimer()
    {
        cudaEventDestroy(m_stop);
        cudaEventDestroy(m_start);
    }

    void setStream(cudaStream_t stream)
    {
        m_stream = stream;
    }

    void start()
    {
        cudaEventRecord(m_start, m_stream);
        m_isTimeValid = false;
    }
    void stop()
    {
        cudaEventRecord(m_stop, m_stream);
        m_isTimeValid = true;
    }

    bool isTimeValid() const
    {
        return m_isTimeValid;
    }

    //Result in us
    float32_t getTime()
    {
        float32_t res;
        cudaEventSynchronize(m_stop);
        cudaEventElapsedTime(&res, m_start, m_stop);
        return 1e3f * res;
    }

private:
    bool m_isTimeValid;
    cudaStream_t m_stream;
    cudaEvent_t m_start;
    cudaEvent_t m_stop;
};

} // namespace testing
} // namespace dw

#endif // TESTS_COMMON_CUDATIMER_HPP_
