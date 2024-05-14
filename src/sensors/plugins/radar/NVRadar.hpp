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

#ifndef NVRADAR_HPP
#define NVRADAR_HPP

//STD
#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <arpa/inet.h>

//DW - General
#include <dw/core/base/Types.h>
#include <dw/core/context/Context.h>
#include <dw/core/base/Status.h>

//DW - Sensor Module Specific
#include <dw/sensors/Sensors.h>
#include <dw/sensors/plugins/radar/RadarPlugin.h>

//DW - Sample Container
#include <BufferPool.hpp>

//Project specific
#include "NVRadar_Properties.hpp"

namespace dw
{
namespace plugin
{
namespace radar
{

class NVRadar
{
public:
    //Data members
    static std::vector<std::unique_ptr<NVRadar>> g_sensorContext;

    //Member functions
    //Constructors
    explicit NVRadar(dwContextHandle_t ctx);
    ~NVRadar() = default;

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

    //Radar Specific Plugin Functions
    dwStatus parseDataBuffer(dwRadarScan* output, const dwRadarScanType scanType);
    dwStatus getConstants(_dwSensorRadarDecoder_constants* constants);
    dwStatus validatePacket(const char* buffer, size_t length, dwRadarScanType* scanType);
    dwStatus setVehicleState(const dwRadarVehicleState* state);

    //Member Functions
    /**
         * Get type of radar sensor.
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
    const int32_t& getFileDescriptor();

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
    const dwStatus getParameter(std::string& val, const std::string& param, const char* params);

private:
    //Data members
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
    dw::plugins::common::BufferPool<rawPacket> m_buffer{MAX_SCANS_PER_SECOND};
    std::queue<rawPacket> m_packetQueue;
    uint32_t m_lastPacketSize;
    dwTime_t m_currScanTimestamp;
    unsigned int m_scanCount = 0;
    uint8_t m_assemblyBuffer[MAX_BUFFER_SIZE];
    dwRadarDetection m_mappedDetections[MAX_DETECTIONS_PER_SCAN];

    //Radar constants
    _dwSensorRadarDecoder_constants m_constants = {};

    //Member Functions
    /**
         * Check if running scan assembly received a full scan worth of data.
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

    /**
         * Get the maximum number of points that are sent per packet depending on the return type and range.
         *
         * @param[in]  dwRadarReturnType Type of data radar sent.
         * @param[in]  dwRadarRange      Range mode used.
         *
         * @return size_t Maximum number of points sent for respective type/range combination.
         */
    size_t getMaxPointsPerPacket(dwRadarReturnType returnType, dwRadarRange range);

    /**
         * Get the number of packets that are sent per scan depending on the return type and range.
         *
         * @param[in]  dwRadarReturnType Type of data radar sent.
         * @param[in]  dwRadarRange      Range mode used.
         *
         * @return size_t Number of packets sent for respective type/range combination.
         */
    size_t getPacketsPerScan(dwRadarReturnType returnType, dwRadarRange range);
};

} // namespace radar
} // namespace plugin
} // namespace dw
#endif
