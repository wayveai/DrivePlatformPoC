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
// Copyright (c) 2021-2023 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
///////////////////////////////////////////////////////////////////////////////////////
#include "NVLidar.hpp"

//STD
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <sys/socket.h>
#include <cstring>

namespace dw
{
namespace plugin
{
namespace lidar
{

NVLidar::NVLidar(dwContextHandle_t ctx)
    : m_ctx(ctx)
{
}

//################################################################################
//###################### Common Sensor Plugin Functions ##########################
//################################################################################
dwStatus NVLidar::createSensor(dwSALHandle_t sal, char const* params)
{
    //Set flag that real sensor is being used
    m_isVirtual = false;

    //Pass SAL handle
    m_sal = sal;

    //Get connections details
    if (getParameter(m_protocol, "protocol", params) != DW_SUCCESS)
    {
        std::cout << "NVLidar::createSensor: No protocol specified in parameter list!" << std::endl;
        return DW_FAILURE;
    }

    if (getParameter(m_ip, "ip", params) != DW_SUCCESS)
    {
        std::cout << "NVLidar::createSensor: No ip specified in parameter list!" << std::endl;
        return DW_FAILURE;
    }

    std::string portNumber;
    if (getParameter(portNumber, "port", params) != DW_SUCCESS)
    {
        std::cout << "NVLidar::createSensor: No port specified in parameter list!" << std::endl;
        return DW_FAILURE;
    }
    else
    {
        m_port = static_cast<uint16_t>(atoi(portNumber.c_str()));
    }

    //Get frequency of lidar scans
    std::string frequency;
    if (getParameter(frequency, "scan-frequency", params) != DW_SUCCESS)
    {
        std::cout << "NVLidar::createSensor: No scan-frequency specified in parameter list!" << std::endl;
        return DW_FAILURE;
    }
    else
    {
        m_frequency = static_cast<uint16_t>(atoi(frequency.c_str()));
    }

    for (uint8_t i = 0; i < SLOT_SIZE; ++i)
    {
        rawPacket* p_rawPacket = nullptr;
        m_dataBuffer.get(p_rawPacket);
        uint8_t* p_rawData = &(p_rawPacket->rawData[0]);
        m_map[p_rawData]   = p_rawPacket;
        m_dataBuffer.put(p_rawPacket);
    }

    return DW_SUCCESS;
}

dwStatus NVLidar::startSensor()
{
    if (!isVirtualSensor())
    {
        return establishConnection();
    }
    return DW_SUCCESS;
}

dwStatus NVLidar::stopSensor()
{
    if (!isVirtualSensor())
    {
        return closeFileDescriptor();
    }
    return DW_SUCCESS;
}

dwStatus NVLidar::resetSensor()
{
    dwStatus status = stopSensor();
    if (status != DW_SUCCESS)
    {
        return status;
    }

    // Clear buffer pool
    for (int i = 0; i < SLOT_SIZE; ++i)
    {
        // Get pointer to element in buffer
        rawPacket* p_rawPacket = nullptr;
        m_dataBuffer.get(p_rawPacket, 1000);

        // Clear memory
        std::memset(p_rawPacket, 0, sizeof(rawPacket));

        //Return pointer to element
        m_dataBuffer.put(p_rawPacket);
    }

    // Empty packet buffer
    while (!m_packetQueue.empty())
    {
        m_packetQueue.pop();
    }

    return startSensor();
}

dwStatus NVLidar::releaseSensor()
{
    if (!isVirtualSensor())
    {
        return closeFileDescriptor();
    }

    return DW_SUCCESS;
}

dwStatus NVLidar::readRawData(uint8_t const** data,
                              size_t* size,
                              dwTime_t* timestamp,
                              dwTime_t timeout_us)
{
    //Get pointer to object in buffer pool to be used for raw data storage
    rawPacket* p_rawPacket = nullptr;
    bool result            = m_dataBuffer.get(p_rawPacket, timeout_us);
    if (!result)
    {
        std::cerr << "NVLidar::readRawData: Failed to get slot from buffer pool" << std::endl;
        return DW_BUFFER_FULL;
    }

    //Get next packet
    uint32_t bytesReceived = recvfrom(m_fd, &(p_rawPacket->rawData[PAYLOAD_OFFSET]), MAX_UDP_PAYLOAD_SIZE, 0, nullptr, nullptr);

    //Get host timestamp for received packet
    dwTime_t packetTimestamp;
    dwContext_getCurrentTime(&packetTimestamp, m_ctx);

    //Copy meta data to buffer and assign to pointers passed to function call
    //NOTE: Raw data is always prefixed with size of received data (uint32_t) and timestamp (dwTime_t)
    std::memcpy(&(p_rawPacket->rawData[0]), &bytesReceived, sizeof(uint32_t));
    std::memcpy(&(p_rawPacket->rawData[sizeof(uint32_t)]), &packetTimestamp, sizeof(dwTime_t));
    *timestamp = packetTimestamp;
    *size      = bytesReceived;
    *data      = &(p_rawPacket->rawData[0]);

    return DW_SUCCESS;
}

dwStatus NVLidar::returnRawData(uint8_t const* data)
{
    if (data == nullptr)
    {
        return DW_INVALID_HANDLE;
    }

    //Return object to buffer pool
    bool result = m_dataBuffer.put(const_cast<rawPacket*>(m_map[const_cast<uint8_t*>(data)]));
    if (!result)
    {
        std::cerr << "NVLidar::returnRawData: Failed to return object to buffer pool!" << std::endl;
        return DW_INVALID_ARGUMENT;
    }

    data = nullptr;

    return DW_SUCCESS;
}

dwStatus NVLidar::pushData(size_t* lenPushed,
                           uint8_t const* data,
                           size_t const size)
{
    /*
     * NOTE: The parameter "size" equals the amount of raw data recevied in bytes plus the
     *       offset of 12 Bytes introduced by prepending the timestamp and amount of
     *       received data in readRawData => size = payload offset + bytesReceived
     *
     *       ===============================================================================
     *       | uint32_t | dwTime_t |                    rawData                            |
     *       ===============================================================================
     *          4 Bytes   8 Bytes                       bytesReceived
    */

    //Copy raw data collected in readRawData to rawPacket object
    rawPacket packet;

    //Copy data into packet struct
    std::memcpy(&(packet.rawData[PAYLOAD_OFFSET]), data, size);

    //Add scan to queue of received scans
    m_packetQueue.push(packet);

    //Update size of data "pushed" to queue
    *lenPushed = size;

    //Update size for last packet received
    m_lastPacketSize = size;

    return DW_SUCCESS;
}

//################################################################################
//###################### Lidar Specific Plugin Functions #########################
//################################################################################
dwStatus NVLidar::parseDataBuffer(dwLidarDecodedPacket* output, const uint64_t hostTimestamp)
{
    //Check status of packet queue and proceed accordingly
    if (!isScanComplete())
    {
        return DW_NOT_READY;
    }

    //Buffer for rawData assembly
    uint8_t assemblyBuffer[MAX_BUFFER_SIZE];

    //Assemble scan
    for (uint32_t i = 0; i < PACKETS_PER_SLICE; ++i)
    {
        //Get pointer to next packet in queue
        rawPacket* p_rawPacket = &(m_packetQueue.front());

        //Copy data from packet into assembly buffer
        std::memcpy(&(assemblyBuffer[i * MAX_UDP_PAYLOAD_SIZE]), &(p_rawPacket->rawData[PAYLOAD_OFFSET]), MAX_UDP_PAYLOAD_SIZE);

        //Release pointer to packet
        p_rawPacket = nullptr;

        //Remove packet from queue
        m_packetQueue.pop();
    }

    //Copy received raw scan into "readable" struct format
    std::memcpy(&m_lidarOutput, &assemblyBuffer, MAX_PAYLOAD_SIZE);

    //Fill output
    output->hostTimestamp   = hostTimestamp;
    output->sensorTimestamp = m_lidarOutput.sensor_timestamp;
    output->maxPoints       = MAX_POINTS_PER_SPIN;
    output->nPoints         = m_lidarOutput.n_points;
    output->scanComplete    = m_lidarOutput.scan_complete;

    dwLidarPointXYZI* xyzi = const_cast<dwLidarPointXYZI*>(output->pointsXYZI);
    dwLidarPointRTHI* rthi = const_cast<dwLidarPointRTHI*>(output->pointsRTHI);

    for (size_t i = 0; i < output->nPoints; ++i)
    {
        xyzi[i].x         = m_lidarOutput.xyzi[i].x;
        xyzi[i].y         = m_lidarOutput.xyzi[i].y;
        xyzi[i].z         = m_lidarOutput.xyzi[i].z;
        xyzi[i].intensity = m_lidarOutput.xyzi[i].intensity;

        rthi[i].radius    = m_lidarOutput.rthi[i].theta;
        rthi[i].theta     = m_lidarOutput.rthi[i].phi;
        rthi[i].phi       = m_lidarOutput.rthi[i].radius;
        rthi[i].intensity = m_lidarOutput.rthi[i].intensity;
    }

    //Clear assembly buffer
    std::memset(assemblyBuffer, 0, sizeof(assemblyBuffer));

    return DW_SUCCESS;
}

dwStatus NVLidar::getConstants(_dwSensorLidarDecoder_constants* constants)
{
    if (!m_init)
    {
        m_constants.maxPayloadSize = MAX_UDP_PAYLOAD_SIZE;

        if (isVirtualSensor())
        {
            m_frequency = MAX_SPINS_PER_SECOND;
        }

        //Populate lidar properties
        dwLidarProperties* properties = &(m_constants.properties);
        properties->pointsPerSpin     = MAX_POINTS_PER_SPIN;
        properties->pointsPerSecond   = MAX_POINTS_PER_SPIN * m_frequency;
        properties->packetsPerSpin    = PACKETS_PER_SPIN;
        properties->spinFrequency     = m_frequency;
        properties->packetsPerSecond  = m_frequency * PACKETS_PER_SPIN;
        properties->pointsPerPacket   = MAX_POINTS_PER_SLICE;
        properties->pointStride       = POINT_STRIDE;
        properties->availableReturns  = DW_LIDAR_RETURN_TYPE_ANY;
        strcpy(properties->deviceString, "CUSTOM_EX");

        //Assign populatatet constants object to function parameter
        *constants = m_constants;

        //Avoid repeated query for constants
        m_init = true;

        return DW_SUCCESS;
    }

    *constants = m_constants;

    return DW_SUCCESS;
}

//################################################################################
//############################## Helper Functions ################################
//################################################################################
bool NVLidar::isVirtualSensor()
{
    return m_isVirtual;
}

dwStatus NVLidar::closeFileDescriptor()
{
    int8_t result = close(m_fd);
    if (result == 0)
    {
        return DW_SUCCESS;
    }
    else
    {
        std::cerr << "NVLidar::closeFileDescriptor: Failed to close file descriptor!" << std::endl;
        return DW_FAILURE;
    }
}

int32_t& NVLidar::getFileDescriptor()
{
    return m_fd;
}

dwStatus NVLidar::getParameter(std::string& val, const std::string& param, const char* params)
{
    std::string paramsString(params);
    std::string searchString(param + "=");
    size_t pos = paramsString.find(searchString);
    if (pos == std::string::npos)
    {
        return DW_FAILURE;
    }

    val = paramsString.substr(pos + searchString.length());
    pos = val.find_first_of(',');
    val = val.substr(0, pos);

    return DW_SUCCESS;
}

bool NVLidar::isScanComplete()
{
    //NOTE: Size of last packet of raw data of a scan is used as separator between scans.
    if (m_lastPacketSize == LAST_PACKET_SIZE && ((m_packetQueue.size() % PACKETS_PER_SLICE) == 0))
    {
        //If most recent packet received is last packet of scan and queue holds the expected number of packets.
        return true;
    }
    else if (m_lastPacketSize == LAST_PACKET_SIZE && ((m_packetQueue.size() % PACKETS_PER_SLICE) != 0))
    {
        //If most recent packet received is last packet of scan and queue holds not the expected number of packets.
        //Empty queue to start with new scan
        while (!m_packetQueue.empty())
        {
            m_packetQueue.pop();
        }
        return false;
    }
    else if (m_packetQueue.size() >= PACKETS_PER_SLICE)
    {
        //If amount of buffered packets exceeds the number of packets needed to assemble a full scan
        while (!m_packetQueue.empty())
        {
            m_packetQueue.pop();
        }
        return false;
    }
    else
    {
        //If packets are still needed for complete scan and most recent packet received is not the last packet of a scan.
        return false;
    }
}

dwStatus NVLidar::establishConnection()
{
    //Depending on communication protocol (UDP vs. TCP) setup socket accordingly
    if (m_protocol == "udp")
    {
        m_fd = socket(AF_INET, SOCK_DGRAM, 0);
    }
    else if (m_protocol == "tcp")
    {
        m_fd = socket(AF_INET, SOCK_STREAM, 0);
    }
    else
    {
        std::cerr << "NVLidar::establishConnection: " << m_protocol << " is not a supported communication protocol! Please provide valid protocol type" << std::endl;
    }

    if (m_fd < 0)
    {
        std::cerr << "NVLidar::establishConnection: Failed to create socket!" << std::endl;
        return DW_FAILURE;
    }

    uint32_t reuse = 1;
    if (setsockopt(m_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0)
    {
        std::cerr << "NVLidar::establishConnection: Failed to set SO_REUSEPORT socket option" << std::endl;
        return DW_FAILURE;
    }

    if (setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        std::cerr << "NVLidar::establishConnection: Failed to set SO_REUSEADDR socket option" << std::endl;
        return DW_FAILURE;
    }

    int flags = 1;
    if (setsockopt(m_fd, SOL_SOCKET, SO_TIMESTAMP, &flags, sizeof(flags)) < 0)
    {
        std::cerr << "NVLidar::establishConnection: Failed to enable kernel timestamping socket options" << std::endl;
        return DW_FAILURE;
    }

    //Socket address
    m_socketAddress.sin_family      = AF_INET;
    m_socketAddress.sin_port        = htons(m_port);
    m_socketAddress.sin_addr.s_addr = inet_addr(m_ip.c_str());

    int8_t result = 0;

    //Establish connection
    if (m_protocol == "udp")
    {
        result = bind(m_fd, reinterpret_cast<struct sockaddr*>(&m_socketAddress), sizeof(m_socketAddress));
    }
    else
    {
        result = connect(m_fd, reinterpret_cast<struct sockaddr*>(&m_socketAddress), sizeof(m_socketAddress));
    }

    if (result < 0)
    {
        if (m_protocol == "udp")
        {
            std::cerr << "NVLidar::establishConnection: Failed to bind socket" << std::endl;
            return DW_FAILURE;
        }
        else
        {
            std::cerr << "NVLidar::establishConnection: Failed to connect to socket" << std::endl;
            return DW_FAILURE;
        }
    }

    return DW_SUCCESS;
}

} // namespace lidar
} // namespace plugin
} // namespace dw
