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
// SPDX-FileCopyrightText: Copyright (c) 2015-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "SimpleRecordingPlayer.hpp"

#include <sstream>
#include <algorithm>

namespace dw_samples
{
namespace common
{

void SimpleRecordingPlayer::restart()
{
    m_isPendingCANMsgValid = false;
    if (m_canSensor)
        CHECK_DW_ERROR(dwSensor_reset(m_canSensor));

    m_isPendingIMUMsgValid = false;
    if (m_imuSensor)
        CHECK_DW_ERROR(dwSensor_reset(m_imuSensor));

    m_isPendingGPSMsgValid = false;
    if (m_gpsSensor)
        CHECK_DW_ERROR(dwSensor_reset(m_gpsSensor));

    m_isPendingLidarPacketValid = false;
    if (m_lidarSensor)
        CHECK_DW_ERROR(dwSensor_reset(m_lidarSensor));

    for (auto& data : m_radars)
    {
        dwSensor_reset(data.radar);
        data.pendingRadarScan = nullptr;
    }

    for (auto& data : m_cameras)
    {
        data.camera->resetCamera();
        data.pendingImage = nullptr;
    }
}

void SimpleRecordingPlayer::stepForward()
{
    dwStatus result;

    // Load from all sensors

    if (!m_isPendingCANMsgValid && m_canSensor != DW_NULL_HANDLE)
    {
        // Load CAN
        result = dwSensorCAN_readMessage(&m_pendingCANMsg, 100000, m_canSensor);
        if (result == DW_SUCCESS)
        {
            m_isPendingCANMsgValid = true;
            if (m_pendingCANMsg.timestamp_us == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: CAN msg has no timestamp. Playback sync will not work.");
        }
        else if (result == DW_END_OF_STREAM)
        {
            std::cout << "CAN reached end of stream." << std::endl;
            m_handler->handleEndOfStream();
            return;
        }
        else
        {
            std::stringstream ss;
            ss << "Terminating. Cannot read CAN frame: " << dwGetStatusName(result);
            throw std::runtime_error(ss.str());
        }
    }

    if (!m_isPendingIMUMsgValid && m_imuSensor != DW_NULL_HANDLE)
    {
        // Load IMU
        result = dwSensorIMU_readFrame(&m_pendingIMUMsg, 100000, m_imuSensor);
        if (result == DW_SUCCESS)
        {
            m_isPendingIMUMsgValid = true;
            if (m_pendingIMUMsg.hostTimestamp == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: IMU msg has no timestamp. Playback sync will not work.");
        }
        else if (result == DW_END_OF_STREAM)
        {
            std::cout << "IMU reached end of stream." << std::endl;
            m_handler->handleEndOfStream();
            return;
        }
        else
        {
            std::stringstream ss;
            ss << "Terminating. Cannot read IMU frame: " << dwGetStatusName(result);
            throw std::runtime_error(ss.str());
        }
    }

    if (!m_isPendingGPSMsgValid && m_gpsSensor != DW_NULL_HANDLE)
    {
        // Load GPS
        result = dwSensorGPS_readFrame(&m_pendingGPSMsg, 100000, m_gpsSensor);
        if (result == DW_SUCCESS)
        {
            m_isPendingGPSMsgValid = true;
            if (m_pendingGPSMsg.timestamp_us == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: GPS msg has no timestamp. Playback sync will not work.");
        }
        else if (result == DW_END_OF_STREAM)
        {
            std::cout << "GPS reached end of stream." << std::endl;
            m_handler->handleEndOfStream();
            return;
        }
        else
        {
            std::stringstream ss;
            ss << "Terminating. Cannot read GPS frame: " << dwGetStatusName(result);
            throw std::runtime_error(ss.str());
        }
    }

    if (!m_isPendingLidarPacketValid && m_lidarSensor != DW_NULL_HANDLE)
    {
        // Load LIDAR
        result = dwSensorLidar_readPacket(&m_pendingLidarPacket, 1000, m_lidarSensor);
        if (result == DW_SUCCESS)
        {
            m_isPendingLidarPacketValid = true;
            if (m_pendingLidarPacket->hostTimestamp == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: Lidar packet has no timestamp. Playback sync will not work.");
        }
        else if (result == DW_END_OF_STREAM)
        {
            std::cout << "Lidar reached end of stream." << std::endl;
            m_handler->handleEndOfStream();
            return;
        }
        else
        {
            std::stringstream ss;
            ss << "Terminating. Cannot read Lidar packet: " << dwGetStatusName(result);
            throw std::runtime_error(ss.str());
        }
    }

    for (auto& data : m_radars)
    {
        if (!data.pendingRadarScan)
        {
            result = dwSensorRadar_readScan(&data.pendingRadarScan, 100000, data.radar);
            if (result == DW_SUCCESS)
            {
                if (data.pendingRadarScan->hostTimestamp == 0 && !isSingleSensorPlayback())
                    throw std::runtime_error("SimpleRecordingPlayer: Radar scan has no timestamp. Playback sync will not work.");
            }
            else if (result == DW_END_OF_STREAM)
            {
                std::cout << "Radar reached end of stream." << std::endl;
                m_handler->handleEndOfStream();
            }
            else
            {
                std::stringstream ss;
                ss << "Terminating. Cannot read radar scan: " << dwGetStatusName(result);
                throw std::runtime_error(ss.str());
            }
        }
    }

    for (auto& data : m_cameras)
    {
        if (!data.pendingImage)
        {
            // Load image
            data.pendingImage = data.camera->readFrame();
            if (!data.pendingImage)
            {
                m_handler->handleEndOfStream();
                return;
            }

            dwTime_t timeStamp;
            dwImage_getTimestamp(&timeStamp, data.pendingImage);
            if (timeStamp == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: Camera image has no timestamp. Playback sync will not work.");
        }
    }

    // Find smallest timestamp
    dwTime_t earliestTimestamp = std::numeric_limits<dwTime_t>::max();

    if (m_isPendingCANMsgValid && m_pendingCANMsg.timestamp_us < earliestTimestamp)
        earliestTimestamp = m_pendingCANMsg.timestamp_us;

    if (m_isPendingIMUMsgValid && m_pendingIMUMsg.hostTimestamp < earliestTimestamp)
        earliestTimestamp = m_pendingIMUMsg.hostTimestamp;

    if (m_isPendingGPSMsgValid && m_pendingGPSMsg.timestamp_us < earliestTimestamp)
        earliestTimestamp = m_pendingGPSMsg.timestamp_us;

    if (m_isPendingLidarPacketValid && m_pendingLidarPacket->hostTimestamp < earliestTimestamp)
        earliestTimestamp = m_pendingLidarPacket->hostTimestamp;

    for (auto& radar : m_radars)
    {
        if (radar.pendingRadarScan && radar.pendingRadarScan->hostTimestamp < earliestTimestamp)
        {
            earliestTimestamp = radar.pendingRadarScan->hostTimestamp;
        }
    }

    for (auto& camera : m_cameras)
    {
        dwTime_t timeStamp;
        dwImage_getTimestamp(&timeStamp, camera.pendingImage);
        if (camera.pendingImage && timeStamp < earliestTimestamp)
            earliestTimestamp = timeStamp;
    }

    // Send an event for the first sensor that matches the earliest timestamp
    if (m_isPendingCANMsgValid && m_pendingCANMsg.timestamp_us == earliestTimestamp)
    {
        // Process CAN
        m_lastCANMsg = m_pendingCANMsg;
        m_handler->handleCAN(m_lastCANMsg);
        m_isPendingCANMsgValid = false;
    }
    else if (m_isPendingIMUMsgValid && m_pendingIMUMsg.hostTimestamp == earliestTimestamp)
    {
        // Process IMU
        m_lastIMUMsg = m_pendingIMUMsg;
        m_handler->handleIMU(m_lastIMUMsg);
        m_isPendingIMUMsgValid = false;
    }
    else if (m_isPendingGPSMsgValid && m_pendingGPSMsg.timestamp_us == earliestTimestamp)
    {
        // Process GPS
        m_lastGPSMsg = m_pendingGPSMsg;
        m_handler->handleGPS(m_lastGPSMsg);
        m_isPendingGPSMsgValid = false;
    }
    else if (m_isPendingLidarPacketValid && m_pendingLidarPacket->hostTimestamp == earliestTimestamp)
    {
        // Process Lidar Packet
        m_lastLidarPacket = m_pendingLidarPacket;
        m_handler->handleLidar(m_lastLidarPacket);
        m_isPendingLidarPacketValid = false;
        dwSensorLidar_returnPacket(m_lastLidarPacket, m_lidarSensor);
    }
    else
    {
        for (auto& radar : m_radars)
        {
            if (radar.pendingRadarScan && radar.pendingRadarScan->hostTimestamp == earliestTimestamp)
            {
                size_t idx            = &radar - &m_radars[0];
                m_lastRadarScans[idx] = radar.pendingRadarScan;
                m_handler->handleRadar(m_lastRadarScans[idx]);
                CHECK_DW_ERROR_MSG(dwSensorRadar_returnScan(m_lastRadarScans[idx], radar.radar), "SimpleRecrdingPlayer:: return radar scan error!");
                radar.pendingRadarScan = nullptr;
                break;
            }
        }

        for (auto& camera : m_cameras)
        {
            dwTime_t timeStamp;
            dwImage_getTimestamp(&timeStamp, camera.pendingImage);

            if (camera.pendingImage && timeStamp == earliestTimestamp)
            {
                // Process image
                size_t idx        = &camera - &m_cameras[0];
                m_lastImages[idx] = camera.pendingImage;
                m_handler->handleCamera(idx, camera.pendingImage);
                camera.pendingImage = nullptr;
                break;
            }
        }
    }
}
}
}
