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
// SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_PLUGINS_BUFFERPOOL_HPP
#define SAMPLES_PLUGINS_BUFFERPOOL_HPP

#include <vector>
#include <deque>
#include <condition_variable>
#include <unordered_set>

namespace dw
{
namespace plugins
{
namespace common
{

/* BufferPool<T> - creates a buffer pool of objects of type T
 *
 * To simplify the implementation - BufferPool does have one assumption
 * of type T - that T has a constructor without parameters, i.e. new T()
 * will create an empty object.
 *
 * Usage:
 *
 *   bool BufferPool<T>::get(T *&, timeout)
 *     Get an object from the pool and return a pointer reference to that
 *     object. Function returns true or false - means whether it succeeds
 *     getting an object from the pool within timeout interval.
 *
 *   BufferPool<T>::put(T *)
 *     Put an object back to the pool. Function returns true or false -
 *     means whether it succeeds putting an object back to the pool within
 *     timeout interval.
 *
 *   BufferPool<T> does not check for whether the object returned is from
 *   the queue or not. DO NOT return an object allocated somewhere else.
 *
 *   WARNING : This data structure is provided ONLY as a reference to demonstrate how
 *   to support "multiple buffers in-flight" when developing a sensor *plugin.
 *   It uses STL containers, which are susceptible to dynamic memory allocations.
 *   Dynamic memory allocations generally fall outside of *most automotive code standards,
 *   and we recommend customers to only use this container as a sample.
 *
 */
template <typename T>
class BufferPool
{
public:
    explicit BufferPool(size_t init_pool_size)
        : m_takenSlotSet(init_pool_size)
    {
        m_buffers.resize(init_pool_size);

        for (auto& f : m_buffers)
        {
            f = new T();
            m_buffer_queue.push_back(f);
        }
    }

    ~BufferPool()
    {
        for (auto& f : m_buffers)
            delete f;
    }

    typename std::vector<T*>::iterator begin()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return beginUnsafe();
    }

    typename std::vector<T*>::iterator end()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return endUnsafe();
    }

    bool empty()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return emptyUnsafe();
    }

    bool full()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return fullUnsafe();
    }

    bool get(T*& f, int timeout_us = 0)
    {
        bool ret;

        std::unique_lock<std::mutex> lock(m_mutex);

        if (timeout_us)
        {
            std::chrono::microseconds timeout(timeout_us);
            ret = m_cond_non_empty.wait_for(lock, timeout,
                                            [this]() { return !emptyUnsafe(); });
        }
        else
        {
            m_cond_non_empty.wait(lock,
                                  [this]() { return !emptyUnsafe(); });
            ret = true;
        }

        if (ret)
        {
            f = m_buffer_queue.front();
            m_buffer_queue.pop_front();
            m_takenSlotSet.insert(f);
            m_cond_non_full.notify_all();
        }
        return ret;
    }

    bool put(T* f, int timeout_us = 0)
    {
        bool ret;

        if (f == nullptr)
            return false;

        std::unique_lock<std::mutex> lock(m_mutex);

        if (timeout_us)
        {
            std::chrono::microseconds timeout(timeout_us);
            ret = m_cond_non_full.wait_for(lock, timeout,
                                           [this]() { return !fullUnsafe(); });
        }
        else
        {
            m_cond_non_full.wait(lock,
                                 [this]() { return !fullUnsafe(); });
            ret = true;
        }

        if (ret)
        {
            auto it = m_takenSlotSet.find(f);
            if (it != m_takenSlotSet.end())
            {
                m_buffer_queue.push_back(f);
                m_takenSlotSet.erase(it);
                m_cond_non_empty.notify_all();
            }
            else
            {
                ret = false;
            }
        }
        return ret;
    }

private:
    typename std::vector<T*> beginUnsafe() { return m_buffers->begin(); }
    typename std::vector<T*> endUnsafe() { return m_buffers->end(); }

    bool emptyUnsafe()
    {
        return m_buffer_queue.empty();
    }

    bool fullUnsafe()
    {
        return m_buffer_queue.size() >= m_buffers.size();
    }

protected:
    std::vector<T*> m_buffers;
    std::deque<T*> m_buffer_queue;
    std::mutex m_mutex;
    std::condition_variable m_cond_non_empty;
    std::condition_variable m_cond_non_full;
    std::unordered_set<T*> m_takenSlotSet;
};

} // namespace common
} // namespace plugins
} // namespace dw

#endif /* SAMPLES_PLUGINS_BUFFERPOOL_HPP */
