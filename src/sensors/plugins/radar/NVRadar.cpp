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
// SPDX-FileCopyrightText: Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "NVRadar.hpp"

//STD
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <sys/socket.h>
#include <dw/core/base/Types.h>
#include <cstring>

namespace dw
{
namespace plugin
{
namespace radar
{

NVRadar::NVRadar(dwContextHandle_t ctx)
    : m_ctx(ctx)
{
}

//################################################################################
//###################### Common Sensor Plugin Functions ##########################
//################################################################################
dwStatus NVRadar::createSensor(dwSALHandle_t sal, char const* params)
{
    //Set flag
    m_isVirtual = false;

    //Pass SAL handle
    m_sal = sal;

    //Get connections details
    getParameter(m_protocol, "protocol", params);
    getParameter(m_ip, "ip", params);
    std::string portNumber;
    getParameter(portNumber, "port", params);
    m_port = static_cast<uint16_t>(atoi(portNumber.c_str()));

    return DW_SUCCESS;
}

dwStatus NVRadar::startSensor()
{
    if (!isVirtualSensor())
    {
        return establishConnection();
    }
    return DW_SUCCESS;
}

dwStatus NVRadar::stopSensor()
{
    if (!isVirtualSensor())
    {
        return closeFileDescriptor();
    }
    return DW_SUCCESS;
}

dwStatus NVRadar::resetSensor()
{
    dwStatus ret = stopSensor();
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    // Clear raw data buffer
    for (int i = 0; i < MAX_SCANS_PER_SECOND; ++i)
    {
        //Get pointer to element in buffer
        rawPacket* p_rawPacket = nullptr;
        m_buffer.get(p_rawPacket, 1000);

        //Clear memory
        std::memset(p_rawPacket, 0, sizeof(rawPacket));

        //Return element to
        m_buffer.put(p_rawPacket);
    }

    // Empty packet buffer
    while (!m_packetQueue.empty())
    {
        m_packetQueue.pop();
    }

    return startSensor();
}

dwStatus NVRadar::releaseSensor()
{
    if (!isVirtualSensor())
    {
        return closeFileDescriptor();
    }

    return DW_SUCCESS;
}

dwStatus NVRadar::readRawData(uint8_t const** data,
                              size_t* size,
                              dwTime_t* timestamp,
                              dwTime_t timeout_us)
{
    //Add new entry to buffer and get pointer to it
    rawPacket* p_rawPacket = nullptr;
    bool result            = m_buffer.get(p_rawPacket, timeout_us);
    if (!result)
    {
        std::cerr << "NVRadar::readRawData: Failed to get slot from buffer pool!" << std::endl;
        return DW_BUFFER_FULL;
    }

    //Get next packet
    uint32_t bytesReceived = recvfrom(m_fd, &(p_rawPacket->rawData[PAYLOAD_OFFSET]), MAX_PACKET_SIZE, 0, nullptr, nullptr);

    //Get host timestamp for next packet
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

dwStatus NVRadar::returnRawData(uint8_t const* data)
{
    if (data == nullptr)
    {
        return DW_INVALID_HANDLE;
    }

    //Return object to buffer pool
    bool result = m_buffer.put(reinterpret_cast<rawPacket*>(const_cast<uint8_t*>(data)));
    if (!result)
    {
        std::cerr << "NVRadar::returnRawData: Failed to return object to buffer pool!" << std::endl;
        return DW_INVALID_ARGUMENT;
    }

    data = nullptr;

    return DW_SUCCESS;
}

dwStatus NVRadar::pushData(size_t* lenPushed,
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

    //Add rawPacket to queue of received packets if within queue limit
    if (m_packetQueue.size() < MAX_SCANS_PER_SECOND)
    {
        //Copy raw data collected in readRawData to rawPacket object
        rawPacket packet;

        //Copy data into packet struct
        std::memcpy(packet.rawData, data, size);

        //Add packet to queue
        m_packetQueue.push(packet);

        //Update size of data "pushed" to queue
        *lenPushed = size;

        /*
         * Keep track of size of most recent packet pushed.
         * The offset has to be substracted as size includes
         * the payload offset.
         */
        m_lastPacketSize = size - PAYLOAD_OFFSET;

        return DW_SUCCESS;
    }

    //If queue size limit exceeded discard packets by clearing it
    std::cerr << "NVRadar::pushData: Packet queue buffer full! Dropping packets..." << std::endl;
    while (!m_packetQueue.empty())
    {
        m_packetQueue.pop();
    }

    return DW_BUFFER_FULL;
}

//################################################################################
//###################### Radar Specific Plugin Functions #########################
//################################################################################
dwStatus NVRadar::parseDataBuffer(dwRadarScan* output, const dwRadarScanType scanType)
{
    //In this sample application we will only decode short range detection
    //and will hardcode in the implementation below
    (void)scanType;

    //Check status of packet queue and proceed accordingly
    if (!isScanComplete())
    {
        return DW_NOT_READY;
    }

    //Timestamp of first packet of running scan is its overall timestamp
    rawPacket* p_rawPacket = &(m_packetQueue.front());
    std::memcpy(&m_currScanTimestamp, (p_rawPacket->rawData) + sizeof(uint32_t), sizeof(dwTime_t));

    //Assemble scan
    for (uint32_t i = 0; i < PACKETS_PER_SCAN; ++i)
    {
        //Get pointer to next packet in queue
        rawPacket* p_rawPacket = &(m_packetQueue.front());

        //Copy data from packet into assembly buffer
        std::memcpy(&(m_assemblyBuffer[i * MAX_PACKET_SIZE]), &(p_rawPacket->rawData[PAYLOAD_OFFSET]), MAX_PACKET_SIZE);

        //Release pointer to packet
        p_rawPacket = nullptr;

        //Remove packet from queue
        m_packetQueue.pop();
    }

    //Copy assembled raw scan into "readable" struct format
    RadarOutput scan;
    std::memcpy(&scan, &m_assemblyBuffer, MAX_SCAN_SIZE);

    //Fill output
    output->hostTimestamp       = m_currScanTimestamp;
    output->sensorTimestamp     = scan.sensor_timestamp;
    output->scanIndex           = m_scanCount;
    output->scanType.returnType = DW_RADAR_RETURN_TYPE_DETECTION;
    output->scanType.range      = DW_RADAR_RANGE_SHORT;
    output->numReturns          = scan.num_returns;
    output->dopplerAmbiguity    = scan.doppler_ambiguity;

    //Fill in detections
    RadarDetection* detectionList = scan.det;
    for (size_t i = 0; i < output->numReturns; ++i)
    {
        //Grab reference to detection object that will be populated
        dwRadarDetection& point = m_mappedDetections[i];
        point                   = {};

        //Map values from RadarDetection to dwRadarDetection
        point.radius            = detectionList[i].radius;
        point.radialVelocity    = detectionList[i].radial_velocity;
        point.azimuth           = detectionList[i].azimuth_angle;
        point.rcs               = detectionList[i].rcs;
        point.elevationAngle    = detectionList[i].elevation_angle;
        point.elevationValidity = true;
        point.x                 = point.radius * std::cos(point.azimuth);
        point.y                 = point.radius * std::sin(point.azimuth);
        point.Vx                = point.radialVelocity * std::cos(point.azimuth);
        point.Vy                = point.radialVelocity * std::sin(point.azimuth);
    }

    //Point data to array with populated DW-based detection objects
    output->data = reinterpret_cast<void*>(m_mappedDetections);

    //Track scan count
    ++m_scanCount;

    return DW_SUCCESS;
}

dwStatus NVRadar::getConstants(_dwSensorRadarDecoder_constants* constants)
{
    //Set radar constants
    constants->maxPayloadSize    = MAX_PACKET_SIZE * PACKETS_PER_SCAN;
    constants->maxPacketsPerScan = PACKETS_PER_SCAN;

    //Set respective scan properties
    constants->properties.numScanTypes          = 1;
    constants->properties.packetsPerScan        = PACKETS_PER_SCAN;
    constants->properties.scansPerSecond        = MAX_SCANS_PER_SECOND;
    constants->properties.inputPacketsPerSecond = MAX_SCANS_PER_SECOND * PACKETS_PER_SCAN;

    //Set supported scan types
    constants->properties.supportedScanTypes[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_SHORT] = true;

    //Set detection limit per scan type
    constants->properties.maxReturnsPerScan[DW_RADAR_RETURN_TYPE_DETECTION][DW_RADAR_RANGE_SHORT] = MAX_DETECTIONS_PER_SCAN;

    //Populate return type specific values
    for (size_t i = 0; i < DW_RADAR_RETURN_TYPE_COUNT; ++i)
    {
        for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; ++j)
        {
            dwRadarReturnType returnType        = static_cast<dwRadarReturnType>(i);
            dwRadarRange range                  = static_cast<dwRadarRange>(j);
            constants->maxPointsPerPacket[i][j] = getMaxPointsPerPacket(returnType, range);
            constants->packetsPerScan[i][j]     = getPacketsPerScan(returnType, range);
        }
    }

    return DW_SUCCESS;
}

dwStatus NVRadar::validatePacket(const char* buffer, size_t length, dwRadarScanType* scanType)
{
    //If your sensor provides any additional information, one can use the parameter buffer to
    //extract any validation information or scan type information from the packet being processed.
    //In this example we only validate the packet length.
    (void)buffer;
    (void)scanType;

    //Check if packet has the expected length
    if (length == MAX_PACKET_SIZE || length == LAST_PACKET_SIZE)
    {
        return DW_SUCCESS;
    }

    return DW_FAILURE;
}

dwStatus NVRadar::setVehicleState(const dwRadarVehicleState* state)
{
    //To be implemented depending on radar specifications - No implementation provided for sample
    (void)state;
    return DW_NOT_IMPLEMENTED;
}

//################################################################################
//############################## Helper Functions ################################
//################################################################################
bool NVRadar::isVirtualSensor()
{
    return m_isVirtual;
}

dwStatus NVRadar::closeFileDescriptor()
{
    int8_t result = close(m_fd);
    if (result == 0)
    {
        return DW_SUCCESS;
    }
    else
    {
        std::cerr << "NVRadar::closeFileDescriptor: Failed to close file descriptor!" << std::endl;
        return DW_FAILURE;
    }
}

const int32_t& NVRadar::getFileDescriptor()
{
    return m_fd;
}

const dwStatus NVRadar::getParameter(std::string& val, const std::string& param, const char* params)
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

bool NVRadar::isScanComplete()
{
    //NOTE: Size of last packet of raw data of a scan is used as separator between scans.
    if (m_lastPacketSize == LAST_PACKET_SIZE && ((m_packetQueue.size() % PACKETS_PER_SCAN) == 0))
    {
        //If most recent packet received is last packet of scan and queue holds the expected number of packets.
        return true;
    }
    else if (m_lastPacketSize == LAST_PACKET_SIZE && ((m_packetQueue.size() % PACKETS_PER_SCAN) != 0))
    {
        //If most recent packet received is last packet of scan and queue holds not the expected number of packets.
        //Empty queue to start with new scan
        while (!m_packetQueue.empty())
        {
            m_packetQueue.pop();
        }
        return false;
    }
    else if (m_packetQueue.size() > PACKETS_PER_SCAN)
    {
        //If queue holds more packets than needed for a full scan, empty queue to start with new scan
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

dwStatus NVRadar::establishConnection()
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
        std::cerr << "NVRadar::establishConnection: " << m_protocol << " is not a supported communication protocol! Please provide valid protocol type" << std::endl;
        return DW_FAILURE;
    }

    if (m_fd < 0)
    {
        std::cerr << "NVRadar::establishConnection: Failed to create socket!" << std::endl;
        return DW_FAILURE;
    }

    uint32_t reuse = 1;
    if (setsockopt(m_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0)
    {
        std::cerr << "NVRadar::establishConnection: Failed to set SO_REUSEPORT socket option" << std::endl;
        return DW_FAILURE;
    }

    if (setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        std::cerr << "NVRadar::establishConnection: Failed to set SO_REUSEADDR socket option" << std::endl;
        return DW_FAILURE;
    }

    int flags = 1;
    if (setsockopt(m_fd, SOL_SOCKET, SO_TIMESTAMP, &flags, sizeof(flags)) < 0)
    {
        std::cerr << "NVRadar::establishConnection: Failed to enable kernel timestamping socket options" << std::endl;
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
            std::cerr << "NVRadar::establishConnection: Failed to bind socket" << std::endl;
            return DW_FAILURE;
        }
        else
        {
            std::cerr << "NVRadar::establishConnection: Failed to connect to socket" << std::endl;
            return DW_FAILURE;
        }
    }

    return DW_SUCCESS;
}

size_t NVRadar::getMaxPointsPerPacket(dwRadarReturnType returnType, dwRadarRange range)
{
    size_t points = 0;
    switch (returnType)
    {
    case DW_RADAR_RETURN_TYPE_DETECTION:
    {
        switch (range)
        {
        case DW_RADAR_RANGE_SHORT:
        {
            points = MAX_POINTS_PER_PACKET;
            break;
        }
        case DW_RADAR_RANGE_MEDIUM:
        case DW_RADAR_RANGE_LONG:
        case DW_RADAR_RANGE_UNKNOWN:
            break;
        default:
            throw std::runtime_error("NVRadar::getMaxPointsPerPacket: Trying to get properties of incorrect range type");
        }
    }
    case DW_RADAR_RETURN_TYPE_TRACK:
    case DW_RADAR_RETURN_TYPE_STATUS:
    case DW_RADAR_RETURN_TYPE_COUNT:
        break;
    default:
        throw std::runtime_error("NVRadar::getMaxPointsPerPacket: Trying to get properties of incorrect return type");
    }
    return points;
}

size_t NVRadar::getPacketsPerScan(dwRadarReturnType returnType, dwRadarRange range)
{
    size_t packets = 0;
    switch (returnType)
    {
    case DW_RADAR_RETURN_TYPE_DETECTION:
    {
        switch (range)
        {
        case DW_RADAR_RANGE_SHORT:
        {
            packets = PACKETS_PER_SCAN;
            break;
        }
        case DW_RADAR_RANGE_MEDIUM:
        case DW_RADAR_RANGE_LONG:
        case DW_RADAR_RANGE_UNKNOWN:
            break;
        default:
            throw std::runtime_error("NVRadar::getPacketsPerScan: Trying to get properties of incorrect range type");
        }
    }
    case DW_RADAR_RETURN_TYPE_TRACK:
    case DW_RADAR_RETURN_TYPE_STATUS:
    case DW_RADAR_RETURN_TYPE_COUNT:
        break;
    default:
        throw std::runtime_error("NVRadar::getPacketsPerScan: Trying to get properties of incorrect return type");
    }
    return packets;
}

} // namespace radar
} // namespace plugin
} // namespace dw
