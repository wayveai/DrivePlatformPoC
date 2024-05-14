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

#include <dw/sensors/plugins/canbus/CANPlugin.h>
#include <dw/sensors/canbus/CAN.h>
#include <BufferPool.hpp>
#include <ByteQueue.hpp>
#include <iostream>
#include <unordered_map>

namespace dw
{
namespace plugins
{
namespace canbus
{

const size_t SAMPLE_BUFFER_POOL_SIZE = 5;

const uint32_t PACKET_OFFSET   = sizeof(uint32_t) + sizeof(dwTime_t);
const uint32_t RAW_PACKET_SIZE = sizeof(dwCANMessage) + PACKET_OFFSET;

typedef struct
{
    uint8_t rawData[RAW_PACKET_SIZE];
} rawPacket;

class SampleCANSensor
{
public:
    SampleCANSensor(dwContextHandle_t ctx, dwSensorHandle_t canSensor, size_t slotSize)
        : m_ctx(ctx)
        , m_sal(nullptr)
        , m_canSensor(canSensor)
        , m_virtualSensorFlag(true)
        , m_buffer(sizeof(dwCANMessage))
        , m_slotSize(slotSize)
    {
        resetSlot();
    }

    ~SampleCANSensor() = default;

    dwStatus createSensor(dwSALHandle_t sal, char const* params)
    {
        m_sal               = sal;
        m_virtualSensorFlag = false;

        // create CAN bus interface
        dwSensorParams parameters{};
        parameters.parameters = params;
        parameters.protocol   = "can.socket";

        if (dwSAL_createSensor(&m_canSensor, parameters, m_sal) != DW_SUCCESS)
        {
            std::cerr << "createSensor: Cannot create sensor "
                      << parameters.protocol << " with " << parameters.parameters << std::endl;

            return DW_FAILURE;
        }
        return DW_SUCCESS;
    }

    dwStatus startSensor()
    {
        if (!isVirtualSensor())
        {
            return dwSensor_start(m_canSensor);
        }

        return DW_SUCCESS;
    }

    dwStatus releaseSensor()
    {
        if (!isVirtualSensor())
        {
            return dwSAL_releaseSensor(m_canSensor);
        }

        return DW_SUCCESS;
    }

    dwStatus stopSensor()
    {
        if (!isVirtualSensor())
        {
            return dwSensor_stop(m_canSensor);
        }

        return DW_SUCCESS;
    }

    dwStatus resetSensor()
    {
        m_buffer.clear();
        resetSlot();

        if (!isVirtualSensor())
        {
            return dwSensor_reset(m_canSensor);
        }

        return DW_SUCCESS;
    }

    dwStatus readRawData(uint8_t const** data, size_t* size, dwTime_t timeout_us)
    {
        rawPacket* result = nullptr;
        bool ok           = m_slot->get(result);
        if (!ok)
        {
            std::cerr << "readRawData: Read raw data, slot not empty\n";
            return DW_BUFFER_FULL;
        }

        dwCANMessage* message =
            reinterpret_cast<dwCANMessage*>(&(result->rawData[PACKET_OFFSET]));

        while (dwSensorCAN_readMessage(message, timeout_us, (m_canSensor)) != DW_SUCCESS)
            ;

        // Copy meta-data
        uint32_t rawDataSize = sizeof(dwCANMessage);
        memcpy(&result->rawData[0], &rawDataSize, sizeof(uint32_t));
        memcpy(&result->rawData[sizeof(uint32_t)], &(message->timestamp_us), sizeof(dwTime_t));

        *data = &(result->rawData[0]);
        *size = RAW_PACKET_SIZE;
        return DW_SUCCESS;
    }

    dwStatus returnRawData(uint8_t const* data)
    {
        if (data == nullptr)
        {
            return DW_INVALID_HANDLE;
        }

        bool ok = m_slot->put(const_cast<rawPacket*>(m_map[const_cast<uint8_t*>(data)]));
        if (!ok)
        {
            std::cerr << "returnRawData: CANPlugin return raw data, invalid data pointer"
                      << std::endl;
            return DW_INVALID_ARGUMENT;
        }

        data = nullptr;

        return DW_SUCCESS;
    }

    dwStatus pushData(uint8_t const* data, size_t const size, size_t* lenPushed)
    {
        if (data == nullptr || lenPushed == nullptr)
        {
            return DW_INVALID_HANDLE;
        }

        m_buffer.enqueue(data, size);
        *lenPushed = size;
        return DW_SUCCESS;
    }

    dwStatus parseData(dwCANMessage* frame)
    {
        const dwCANMessage* msg;

        if (!m_buffer.peek(reinterpret_cast<const uint8_t**>(&msg)))
        {
            return DW_FAILURE;
        }

        *frame = *msg;

        m_buffer.dequeue();

        return DW_SUCCESS;
    }

    dwStatus setFilter(const uint32_t* ids, const uint32_t* masks, uint16_t numCanIDs)
    {
        return dwSensorCAN_setMessageFilter(ids, masks, numCanIDs, m_canSensor);
    }

    dwStatus setUseHwTimestamps(bool flag)
    {
        return dwSensorCAN_setUseHwTimestamps(flag, m_canSensor);
    }

    dwStatus clearFilter()
    {
        const uint32_t ids[]   = {0};
        const uint32_t masks[] = {0};
        return setFilter(ids, masks, 1);
    }

    dwStatus send(const dwCANMessage* msg, dwTime_t timeout_us)
    {
        return dwSensorCAN_sendMessage(msg, timeout_us, m_canSensor);
    }

    static std::vector<std::unique_ptr<dw::plugins::canbus::SampleCANSensor>> g_sensorContext;

private:
    void resetSlot()
    {
        m_slot = std::make_unique<dw::plugins::common::BufferPool<rawPacket>>(m_slotSize);
        m_map.clear();

        std::vector<rawPacket*> vectorOfRawPacketPtr;
        std::vector<uint8_t*> vectorOfRawDataPtr;
        for (uint8_t i = 0; i < m_slotSize; ++i)
        {
            rawPacket* rawPacketPtr = nullptr;
            bool ok                 = m_slot->get(rawPacketPtr);
            if (!ok)
            {
                std::cerr << "resetSlot(), BufferPool instanciated with non empty slots" << std::endl;
            }

            uint8_t* rawDataPtr = &(rawPacketPtr->rawData[0]);
            vectorOfRawPacketPtr.push_back(rawPacketPtr);
            vectorOfRawDataPtr.push_back(rawDataPtr);
        }

        for (uint8_t i = 0; i < m_slotSize; ++i)
        {
            rawPacket* rawPacketPtr = vectorOfRawPacketPtr[i];
            uint8_t* rawDataPtr     = vectorOfRawDataPtr[i];
            m_map[rawDataPtr]       = rawPacketPtr;
            bool ok                 = m_slot->put(rawPacketPtr);
            if (!ok)
            {
                std::cerr << "resetSlot(), BufferPool invalid put" << std::endl;
            }
        }
    }

    inline bool isVirtualSensor()
    {
        return m_virtualSensorFlag;
    }

    dwContextHandle_t m_ctx      = nullptr;
    dwSALHandle_t m_sal          = nullptr;
    dwSensorHandle_t m_canSensor = nullptr;
    bool m_virtualSensorFlag;

    dw::plugin::common::ByteQueue m_buffer;
    std::unique_ptr<dw::plugins::common::BufferPool<rawPacket>> m_slot;
    std::unordered_map<uint8_t*, rawPacket*> m_map;
    size_t m_slotSize;
};
} // namespace canbus
} // namespace plugins
} // namespace dw

std::vector<std::unique_ptr<dw::plugins::canbus::SampleCANSensor>> dw::plugins::canbus::SampleCANSensor::g_sensorContext;

//#######################################################################################
static bool checkValid(dw::plugins::canbus::SampleCANSensor* sensor)
{
    for (auto& i : dw::plugins::canbus::SampleCANSensor::g_sensorContext)
    {
        if (i.get() == sensor)
            return true;
    }
    return false;
}

// exported functions
extern "C" {

//#######################################################################################
dwStatus _dwSensorPlugin_createHandle(dwSensorPluginSensorHandle_t* sensor, dwSensorPluginProperties* properties,
                                      char const*, dwContextHandle_t ctx)
{
    if (!sensor)
        return DW_INVALID_ARGUMENT;

    size_t slotSize = dw::plugins::canbus::SAMPLE_BUFFER_POOL_SIZE; // Size of memory pool to read raw data from the sensor
    std::unique_ptr<dw::plugins::canbus::SampleCANSensor> sensorContext(new dw::plugins::canbus::SampleCANSensor(ctx, DW_NULL_HANDLE, slotSize));

    dw::plugins::canbus::SampleCANSensor::g_sensorContext.push_back(std::move(sensorContext));
    *sensor = dw::plugins::canbus::SampleCANSensor::g_sensorContext.back().get();

    // Populate sensor properties
    properties->packetSize = sizeof(dw::plugins::canbus::rawPacket);

    return DW_SUCCESS;
}

//#######################################################################################
dwStatus _dwSensorPlugin_createSensor(char const* params, dwSALHandle_t sal, dwSensorPluginSensorHandle_t sensor)
{

    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->createSensor(sal, params);
}

//#######################################################################################
dwStatus _dwSensorPlugin_start(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->startSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    for (auto iter =
             dw::plugins::canbus::SampleCANSensor::g_sensorContext.begin();
         iter != dw::plugins::canbus::SampleCANSensor::g_sensorContext.end();
         ++iter)
    {
        if ((*iter).get() == sensor)
        {
            sensorContext->stopSensor();
            sensorContext->releaseSensor();
            dw::plugins::canbus::SampleCANSensor::g_sensorContext.erase(iter);
            return DW_SUCCESS;
        }
    }
    return DW_FAILURE;
}

//#######################################################################################
dwStatus _dwSensorPlugin_stop(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->stopSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_reset(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->resetSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_readRawData(uint8_t const** data, size_t* size, dwTime_t* /*timestamp*/,
                                     dwTime_t timeout_us, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->readRawData(data, size, timeout_us);
}

//#######################################################################################
dwStatus _dwSensorPlugin_returnRawData(uint8_t const* data, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->returnRawData(data);
}

//########################################################################################
dwStatus _dwSensorPlugin_pushData(size_t* lenPushed, uint8_t const* data,
                                  size_t const size, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->pushData(data, size, lenPushed);
}

//########################################################################################
dwStatus _dwSensorCANPlugin_parseDataBuffer(dwCANMessage* frame,
                                            dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->parseData(frame);
}

//#######################################################################################
dwStatus _dwSensorPlugin_setFilter(const uint32_t* ids, const uint32_t* masks, uint16_t numCanIDs, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->setFilter(ids, masks, numCanIDs);
}

//#######################################################################################
dwStatus _dwSensorPlugin_clearFilter(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->clearFilter();
}

//#######################################################################################
dwStatus _dwSensorPlugin_setUseHwTimestamps(bool use, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->setUseHwTimestamps(use);
}

dwStatus _dwSensorPlugin_send(const dwCANMessage* msg, dwTime_t timeout_us, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::canbus::SampleCANSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->send(msg, timeout_us);
}

//#######################################################################################
dwStatus dwSensorCANPlugin_getFunctionTable(dwSensorCANPluginFunctionTable* functions)
{
    if (functions == nullptr)
    {
        return DW_INVALID_ARGUMENT;
    }

    functions->common               = {};
    functions->common.createHandle  = _dwSensorPlugin_createHandle;
    functions->common.createSensor  = _dwSensorPlugin_createSensor;
    functions->common.release       = _dwSensorPlugin_release;
    functions->common.start         = _dwSensorPlugin_start;
    functions->common.stop          = _dwSensorPlugin_stop;
    functions->common.reset         = _dwSensorPlugin_reset;
    functions->common.readRawData   = _dwSensorPlugin_readRawData;
    functions->common.returnRawData = _dwSensorPlugin_returnRawData;
    functions->common.pushData      = _dwSensorPlugin_pushData;
    functions->parseDataBuffer      = _dwSensorCANPlugin_parseDataBuffer;
    functions->clearFilter          = _dwSensorPlugin_clearFilter;
    functions->setFilter            = _dwSensorPlugin_setFilter;
    functions->setUseHwTimestamps   = _dwSensorPlugin_setUseHwTimestamps;
    functions->send                 = _dwSensorPlugin_send;

    return DW_SUCCESS;
}

} // extern "C"
