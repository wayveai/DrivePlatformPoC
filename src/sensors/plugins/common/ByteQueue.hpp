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
// SPDX-FileCopyrightText: Copyright (c) 2019-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_PLUGINS_BYTEQUEUE_HPP
#define SAMPLES_PLUGINS_BYTEQUEUE_HPP

#include <vector>
#include <cstdint>

namespace dw
{
namespace plugin
{
namespace common
{

/* ByteQueue - creates a byte queue w/ a fixed message size
 *
 * This data structure takes in a raw byte streams, and allows the user
 * to pop off a full message worth of raw bytes using the dequeue operation.
 *
 * WARNING : This data structure is provided ONLY as a reference to demonstrate how
 * to support "multiple buffers in-flight" when developing a sensor *plugin.
 * It uses STL containers, which are susceptible to dynamic memory allocations.
 * Dynamic memory allocations generally fall outside of *most automotive code standards,
 * and we recommend customers to only use this container as a sample.
 */
class ByteQueue
{
public:
    explicit ByteQueue(size_t sizeOfMessage)
    {
        setSize(sizeOfMessage);
    }

    ~ByteQueue() = default;

    void setSize(size_t sizeOfMessage)
    {
        m_sizeOfMessage = sizeOfMessage;
    }

    void enqueue(const uint8_t*, size_t);

    bool peek(const uint8_t**);

    bool dequeue();

    void clear();

private:
    std::vector<uint8_t> m_deque;
    size_t m_sizeOfMessage;
};

inline void ByteQueue::enqueue(const uint8_t* data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        m_deque.push_back(*data);
        ++data;
    }
}

inline bool ByteQueue::peek(const uint8_t** address)
{
    if (m_deque.size() < m_sizeOfMessage)
    {
        return false;
    }
    else
        *address = &m_deque.front();

    return true;
}

inline bool ByteQueue::dequeue()
{
    if (m_deque.size() < m_sizeOfMessage)
        return false;
    else
        m_deque.erase(m_deque.begin(), m_deque.begin() + m_sizeOfMessage);

    return true;
}

inline void ByteQueue::clear()
{
    m_deque.clear();
}

} // namespace common
} // namespace plugin
} // namespace dw

#endif // SAMPLES_PLUGINS_BYTEQUEUE_HPP
