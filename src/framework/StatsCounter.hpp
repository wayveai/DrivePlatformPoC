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

#ifndef SAMPLES_COMMON_STATSCOUNTER_HPP_
#define SAMPLES_COMMON_STATSCOUNTER_HPP_

#include <dw/core/base/Types.h>

#include <limits>
#include <sstream>
#include <algorithm>
#include <vector>
#include <cmath>

namespace dw_samples
{
namespace common
{

// Classes
class StatsCounter
{
public:
    StatsCounter() = default;

    explicit StatsCounter(size_t maxSampleCount)
        : m_maxSampleCount(maxSampleCount)
    {
    }

    void addSample(float32_t sample)
    {
        if (m_samples.size() < m_maxSampleCount)
        {
            m_samples.push_back(sample);
        }

        m_count++;
        m_sum += sample;
        m_sumSq += sample * sample;
        if (sample > m_max)
        {
            m_max = sample;
        }
        if (sample < m_min)
        {
            m_min = sample;
        }
    }
    void addSample(uint32_t sample)
    {
        addSample(static_cast<float32_t>(sample));
    }
    void addSample(int32_t sample)
    {
        addSample(static_cast<float32_t>(sample));
    }

    template <typename T>
    void addSampleArray(const T* array, uint32_t size)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            addSample(static_cast<float32_t>(array[i]));
        }
    }

    uint32_t getSampleCount() const
    {
        return m_count;
    }

    float32_t getMin() const
    {
        return m_min;
    }

    float32_t getMax() const
    {
        return m_max;
    }

    float32_t getSum() const
    {
        return m_sum;
    }

    float32_t getMean() const
    {
        return m_count > 0 ? m_sum / static_cast<float32_t>(m_count) : 0.0f;
    }

    float32_t getVariance() const
    {
        if (m_count == 0)
        {
            return 0.0f;
        }

        float32_t mean = getMean();
        float32_t var  = m_sumSq / static_cast<float32_t>(m_count) - mean * mean;

        return std::abs(var) <= 1e-7f ? 0.0f : var;
    }

    float32_t getStdDev() const
    {
        return std::sqrt(getVariance());
    }

    /*
    * Note: To simplify code and make the computation faster, this returns the true median only for odd sample counts.
    *       For even counts the true median would be (samples[n/2] + samples[n/2+1])/2, but sample[n/2] is returned instead.
    */
    float32_t getMedian()
    {
        if (m_samples.empty() || m_samples.size() >= m_maxSampleCount)
        {
            return std::numeric_limits<float32_t>::quiet_NaN();
        }

        auto n = m_samples.begin() + static_cast<ptrdiff_t>(m_samples.size() / 2);
        std::nth_element(m_samples.begin(), n, m_samples.end());
        return *n;
    }

    /*
     * Note: This method overrides constness to reorder the samples vector. It is considered const because all stats remain the same.
     */
    float32_t getMedian() const
    {
        return const_cast<StatsCounter*>(this)->getMedian();
    }

    template <class TStream>
    void writeToStream(TStream& stream) const
    {
        stream << "Median=" << getMedian() << ", mean=" << getMean() << ", var=" << getVariance() << ", std dev=" << getStdDev()
               << ", sample count=" << getSampleCount() << ", min=" << getMin() << ", max=" << getMax();
    }

protected:
    //Full sample array stored to calculate median
    size_t m_maxSampleCount = 5000;
    std::vector<float32_t> m_samples;

    //These are used to calculate mean and variance
    uint32_t m_count  = 0;
    float32_t m_sum   = 0.0f;
    float32_t m_sumSq = 0.0f;

    float32_t m_max = std::numeric_limits<float32_t>::lowest();
    float32_t m_min = std::numeric_limits<float32_t>::max();
};

inline std::ostream& operator<<(std::ostream& stream, const StatsCounter& counter)
{
    counter.writeToStream(stream);
    return stream;
}

} // namespace testing
} // namespace dw

#endif // TESTS_COMMON_STATSCOUNTER_HPP_
