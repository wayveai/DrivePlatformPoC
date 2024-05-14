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
// SPDX-FileCopyrightText: Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <dw/sensors/plugins/imu/IMUPlugin.h>
#include <dw/sensors/canbus/CAN.h>
#include <BufferPool.hpp>
#include <ByteQueue.hpp>
#include <iostream>
#include <unordered_map>

namespace dw
{
namespace plugins
{
namespace imu
{

// Structure defining sample CAN acceleration
typedef struct
{
    int16_t accelLat;
    int16_t accelLong;
    int16_t accelVert;
} SampleCANReportAccel;

// Structure defining sample CAN gyro
typedef struct
{
    int16_t gyroRoll;
    int16_t gyroYaw;
} SampleCANReportGyro;

const uint32_t SAMPLE_CAN_REPORT_ACCEL = 0x06B;
const uint32_t SAMPLE_CAN_REPORT_GYRO  = 0x06C;

const size_t SAMPLE_BUFFER_POOL_SIZE = 5;

const uint32_t PACKET_OFFSET   = sizeof(uint32_t) + sizeof(dwTime_t);
const uint32_t RAW_PACKET_SIZE = sizeof(dwCANMessage) + PACKET_OFFSET;

typedef struct
{
    uint8_t rawData[RAW_PACKET_SIZE];
} rawPacket;

class SampleIMUSensor
{
public:
    SampleIMUSensor(dwContextHandle_t ctx, dwSensorHandle_t canSensor, size_t slotSize)
        : m_ctx(ctx)
        , m_sal(nullptr)
        , m_canSensor(canSensor)
        , m_virtualSensorFlag(true)
        , m_buffer(sizeof(dwCANMessage))
        , m_slotSize(slotSize)
    {
        resetSlot();
    }

    ~SampleIMUSensor() = default;

    dwStatus createSensor(dwSALHandle_t sal, char const* params)
    {
        m_sal               = sal;
        m_virtualSensorFlag = false;

        std::string paramsString = params;
        std::string searchString = "can-proto=";
        size_t pos               = paramsString.find(searchString);

        if (pos == std::string::npos)
        {
            std::cerr << "createSensor: Protocol not specified\n";
            return DW_FAILURE;
        }

        std::string protocolString = paramsString.substr(pos + searchString.length());
        pos                        = protocolString.find_first_of(",");
        protocolString             = protocolString.substr(0, pos);

        // create CAN bus interface
        dwSensorParams parameters{};
        parameters.parameters = params;
        parameters.protocol   = protocolString.c_str();

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
            return dwSensor_start(m_canSensor);

        return DW_SUCCESS;
    }

    dwStatus releaseSensor()
    {
        if (!isVirtualSensor())
            return dwSAL_releaseSensor(m_canSensor);

        return DW_SUCCESS;
    }

    dwStatus stopSensor()
    {
        if (!isVirtualSensor())
            return dwSensor_stop(m_canSensor);

        return DW_SUCCESS;
    }

    dwStatus resetSensor()
    {
        m_buffer.clear();
        resetSlot();

        if (!isVirtualSensor())
            return dwSensor_reset(m_canSensor);

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

        while (dwSensorCAN_readMessage(message, timeout_us, (m_canSensor)) == DW_SUCCESS)
        {
            // Filter invalid messages
            if (message->id == SAMPLE_CAN_REPORT_ACCEL || message->id == SAMPLE_CAN_REPORT_GYRO)
            {
                break;
            }
        }

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
            std::cerr << "returnRawData: IMUPlugin return raw data, invalid data pointer" << std::endl;
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

    dwStatus parseData(dwIMUFrame* frame, size_t* consumed)
    {
        const dwCANMessage* reference;

        if (!m_buffer.peek(reinterpret_cast<const uint8_t**>(&reference)))
        {
            return DW_NOT_AVAILABLE;
        }

        if (consumed)
            *consumed = sizeof(dwCANMessage);

        *frame               = {};
        frame->hostTimestamp = (*reference).timestamp_us;
        setSignalValid(frame->validityInfo.hostTimestamp);

        switch ((*reference).id)
        {
        case SAMPLE_CAN_REPORT_ACCEL:
        {
            auto ptr = reinterpret_cast<const SampleCANReportAccel*>((*reference).data);
            // All fields have range: -327.68 to 327.67, LSB: 0.01 m/s^2, thus
            // multiply by 0.01f to convert to m/s^2
            frame->acceleration[0] = static_cast<float32_t>(ptr->accelLong) * 0.01f;
            frame->acceleration[1] = static_cast<float32_t>(ptr->accelLat) * 0.01f;
            frame->acceleration[2] = static_cast<float32_t>(ptr->accelVert) * 0.01f;
            setSignalValid(frame->validityInfo.acceleration[0]);
            setSignalValid(frame->validityInfo.acceleration[1]);
            setSignalValid(frame->validityInfo.acceleration[2]);
            break;
        }
        case SAMPLE_CAN_REPORT_GYRO:
        {
            auto ptr = reinterpret_cast<const SampleCANReportGyro*>((*reference).data);
            // All fields have range: -6.5536 to 6.5534, LSB: 0.0002 rad/s, thus
            // multiply by 0.0002 to convert to rad/s
            frame->turnrate[0] = static_cast<float32_t>(ptr->gyroRoll) * 0.0002f;
            frame->turnrate[1] = 0; // XXX: need to extend DataSpeed protocol for this
            frame->turnrate[2] = static_cast<float32_t>(ptr->gyroYaw) * 0.0002f;
            setSignalValid(frame->validityInfo.turnrate[0]);
            clearSignalValid(frame->validityInfo.turnrate[1]); // XXX: need to extend DataSpeed protocol for this
            setSignalValid(frame->validityInfo.turnrate[2]);
            break;
        }
        default:
            m_buffer.dequeue();
            return DW_FAILURE;
        }
        m_buffer.dequeue();
        return DW_SUCCESS;
    }

    static std::vector<std::unique_ptr<dw::plugins::imu::SampleIMUSensor>> g_sensorContext;

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

    inline void setSignalValid(dwSignalValidity& validity)
    {
        dwSignal_encodeSignalValidity(&validity,
                                      DW_SIGNAL_STATUS_LAST_VALID,
                                      DW_SIGNAL_TIMEOUT_NONE,
                                      DW_SIGNAL_E2E_NO_ERROR);
    }

    inline void clearSignalValid(dwSignalValidity& validity)
    {
        dwSignal_encodeSignalValidity(&validity,
                                      DW_SIGNAL_STATUS_INIT,
                                      DW_SIGNAL_TIMEOUT_NO_INFORMATION,
                                      DW_SIGNAL_E2E_NO_INFORMATION);
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
} // namespace imu
} // namespace plugins
} // namespace dw

std::vector<std::unique_ptr<dw::plugins::imu::SampleIMUSensor>> dw::plugins::imu::SampleIMUSensor::g_sensorContext;

//#######################################################################################
static bool checkValid(dw::plugins::imu::SampleIMUSensor* sensor)
{
    for (auto& i : dw::plugins::imu::SampleIMUSensor::g_sensorContext)
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

    size_t slotSize = dw::plugins::imu::SAMPLE_BUFFER_POOL_SIZE; // Size of memory pool to read raw data from the sensor
    std::unique_ptr<dw::plugins::imu::SampleIMUSensor> sensorContext(new dw::plugins::imu::SampleIMUSensor(ctx, DW_NULL_HANDLE, slotSize));

    dw::plugins::imu::SampleIMUSensor::g_sensorContext.push_back(std::move(sensorContext));
    *sensor = dw::plugins::imu::SampleIMUSensor::g_sensorContext.back().get();

    // Populate sensor properties
    properties->packetSize = sizeof(dw::plugins::imu::rawPacket);

    return DW_SUCCESS;
}

//#######################################################################################
dwStatus _dwSensorPlugin_createSensor(char const* params, dwSALHandle_t sal, dwSensorPluginSensorHandle_t sensor)
{

    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->createSensor(sal, params);
}

//#######################################################################################
dwStatus _dwSensorPlugin_start(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->startSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    for (auto iter =
             dw::plugins::imu::SampleIMUSensor::g_sensorContext.begin();
         iter != dw::plugins::imu::SampleIMUSensor::g_sensorContext.end();
         ++iter)
    {
        if ((*iter).get() == sensor)
        {
            sensorContext->stopSensor();
            sensorContext->releaseSensor();
            dw::plugins::imu::SampleIMUSensor::g_sensorContext.erase(iter);
            return DW_SUCCESS;
        }
    }
    return DW_FAILURE;
}

//#######################################################################################
dwStatus _dwSensorPlugin_stop(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->stopSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_reset(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
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
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->readRawData(data, size, timeout_us);
}

//#######################################################################################
dwStatus _dwSensorPlugin_returnRawData(uint8_t const* data, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->returnRawData(data);
}

//#######################################################################################
dwStatus _dwSensorPlugin_pushData(size_t* lenPushed, uint8_t const* data, size_t const size, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->pushData(data, size, lenPushed);
}

//#######################################################################################
dwStatus _dwSensorIMUPlugin_parseDataBuffer(dwIMUFrame* frame, size_t* consumed, dwSensorPlugin_timeDomain*, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->parseData(frame, consumed);
}

//#######################################################################################
dwStatus dwSensorIMUPlugin_getFunctionTable(dwSensorIMUPluginFunctionTable* functions)
{
    if (functions == nullptr)
        return DW_INVALID_ARGUMENT;

    functions->common.createHandle  = _dwSensorPlugin_createHandle;
    functions->common.createSensor  = _dwSensorPlugin_createSensor;
    functions->common.release       = _dwSensorPlugin_release;
    functions->common.start         = _dwSensorPlugin_start;
    functions->common.stop          = _dwSensorPlugin_stop;
    functions->common.reset         = _dwSensorPlugin_reset;
    functions->common.readRawData   = _dwSensorPlugin_readRawData;
    functions->common.returnRawData = _dwSensorPlugin_returnRawData;
    functions->common.pushData      = _dwSensorPlugin_pushData;
    functions->parseDataBufferNew   = _dwSensorIMUPlugin_parseDataBuffer;

    return DW_SUCCESS;
}

} // extern "C"
