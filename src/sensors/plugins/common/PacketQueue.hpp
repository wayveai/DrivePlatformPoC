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

#ifndef SAMPLES_PLUGINS_PACKETQUEUE_HPP
#define SAMPLES_PLUGINS_PACKETQUEUE_HPP

#include <vector>
#include <queue>
#include <array>

namespace dw
{
namespace plugin
{
namespace common
{

/* PacketQueue - creates a raw data queue which can have a maximum queue size
 *
 * This data structure allows the user to store raw byte streams of arbitrary length
 * and user can pop the stored data using dequeue operation
 *
 * WARNING : This data structure is provided ONLY as a reference to demonstrate how
 * to support "multiple buffers in-flight" when developing a sensor *plugin.
 * It uses STL containers, which are susceptible to dynamic memory allocations.
 * Dynamic memory allocations generally fall outside of *most automotive code standards,
 * and we recommend customers to only use this container as a sample.
 */

class PacketQueue
{
public:
    explicit PacketQueue(size_t maxQueueSize)
    {
        m_maxQueueSize = maxQueueSize;
    }

    ~PacketQueue() = default;

    inline bool enqueue(const uint8_t*, size_t);

    inline bool peek(const uint8_t**);

    inline bool peek(const uint8_t**, size_t*);

    inline bool dequeue();

private:
    std::queue<std::vector<uint8_t>> m_queue;
    size_t m_maxQueueSize;
};

bool PacketQueue::enqueue(const uint8_t* data, size_t length)
{
    if (m_queue.size() < m_maxQueueSize)
    {
        m_queue.push(std::vector<uint8_t>(data, data + length));
        return true;
    }

    return false;
}

bool PacketQueue::peek(const uint8_t** address, size_t* length)
{
    bool ret = peek(address);
    *length  = m_queue.front().size();
    return ret;
}

bool PacketQueue::peek(const uint8_t** address)
{
    if (m_queue.empty())
        return false;

    *address = &(m_queue.front().front());
    return true;
}

bool PacketQueue::dequeue()
{
    if (m_queue.empty())
        return false;

    m_queue.pop();
    return true;
}

} // namespace common
} // namespace plugin
} // namespace dw

#endif // SAMPLES_PLUGINS_PACKETQUEUE_HPP
