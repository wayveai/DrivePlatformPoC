/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2021-2022 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
///////////////////////////////////////////////////////////////////////////////////////
#ifndef NVLIDAR_HPP
#define NVLIDAR_HPP

//STD
#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <unordered_map>
#include <arpa/inet.h>

//DW - General
#include <dw/core/base/Types.h>
#include <dw/core/context/Context.h>
#include <dw/core/base/Status.h>

//DW - Sensor Module Specific
#include <dw/sensors/Sensors.h>
#include <dw/sensors/plugins/lidar/LidarPlugin.h>

//DW - Sample Container
#include <BufferPool.hpp>

//Project specific
#include "NVLidar_Properties.hpp"

namespace dw
{
namespace plugin
{
namespace lidar
{

class NVLidar
{
public:
    //Data members
    static std::vector<std::unique_ptr<NVLidar>> g_sensorContext;

    //Member functions
    //Constructors
    explicit NVLidar(dwContextHandle_t ctx);
    ~NVLidar() = default;

    //Common Sensor Plugin Functions
    dwStatus createSensor(dwSALHandle_t sal, char const* params);
    dwStatus startSensor();
    dwStatus stopSensor();
    dwStatus resetSensor();
    dwStatus releaseSensor();

    dwStatus readRawData(uint8_t const** data,
                         size_t* size,
                         dwTime_t* timestamp,
                         dwTime_t timeout_us);

    dwStatus returnRawData(uint8_t const* data);
    dwStatus pushData(size_t* lenPushed, uint8_t const* data, size_t const size);

    //Lidar Specific Plugin Functions
    dwStatus parseDataBuffer(dwLidarDecodedPacket* output, const uint64_t hostTimestamp);
    dwStatus getConstants(_dwSensorLidarDecoder_constants* constants);

    //Member Functions
    /**
         * Get type of lidar sensor.
         *
         * @return false if virtual snesor used for replay.
         *         true if live sensor stream used.
         */
    bool isVirtualSensor();

    /**
         * Get copy of file descriptor used for socket connection.
         *
         * @return uint32_t File descriptor value.
         */
    int32_t& getFileDescriptor();

    /**
         * Closes file descriptor used for socket connection.
         *
         * @return dwStatus DW_SUCCESS Successfully closed file descriptor.
         *                  DW_FAILURE Failed to close file descriptor.
         */
    dwStatus closeFileDescriptor();

    /**
         * Get parameters provided via command line.
         *
         * @param[out] string Reference to string to be populated with parameter value requested.
         * @param[in]  string Name of paramters to be searched for.
         * @param[in]  char*  Pointer to char array holding paramters values provided via command line.
         *
         * @return dwStatus DW_SUCCESS Successfully retrieved parameter value.
         *                  DW_FAILURE Failed to retrieve parameter value.
         */
    dwStatus getParameter(std::string& val, const std::string& param, const char* params);

private:
    //Framework
    dwContextHandle_t m_ctx;
    dwSALHandle_t m_sal = nullptr;

    //Socket connection
    bool m_isVirtual = true;
    int32_t m_fd     = -1;
    std::string m_protocol;
    std::string m_ip;
    uint16_t m_port = 0;
    struct sockaddr_in m_socketAddress;

    //Raw data assembly
    dwTime_t m_currScanTimestamp;
    uint32_t m_lastPacketSize;
    bool m_scanComplete = false;
    dw::plugins::common::BufferPool<rawPacket> m_dataBuffer{SLOT_SIZE};

    //Decoding
    std::unordered_map<uint8_t*, rawPacket*> m_map;
    std::queue<rawPacket> m_packetQueue;
    LidarPacket m_lidarOutput;

    //Decoder Constants
    _dwSensorLidarDecoder_constants m_constants = {};

    //Other
    bool m_init     = false;
    int m_frequency = 0;

    /**
         * Check if running packet collection is enough to assemble a full scan
         *
         * @return bool true  Received data worth a full scan.
         *              false Not enough data received.
         */
    bool isScanComplete();

    /**
         * Setup and connect socket to provided address.
         *
         * @return dwStatus DW_SUCCESS Successfully setup socket connection.
         *                  DW_FAILURE Failed to setup socket connection.
         */
    dwStatus establishConnection();
};

} // namespace lidar
} // namespace plugin
} // namespace dw
#endif
