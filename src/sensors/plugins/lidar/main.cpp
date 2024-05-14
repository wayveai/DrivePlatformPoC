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
#include <dw/sensors/plugins/lidar/LidarPlugin.h>

//STD
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

//Project specific
#include "NVLidar.hpp"

std::vector<std::unique_ptr<dw::plugin::lidar::NVLidar>> dw::plugin::lidar::NVLidar::g_sensorContext;

//################################################################################
//############################### Helper Functions ###############################
//################################################################################
static dwStatus IsValidSensor(dw::plugin::lidar::NVLidar* sensor)
{
    for (auto& i : dw::plugin::lidar::NVLidar::g_sensorContext)
    {
        if (i.get() == sensor)
        {
            return DW_SUCCESS;
        }
    }

    return DW_INVALID_HANDLE;
}

//################################################################################
//###################### Common Sensor Plugin Functions ##########################
//################################################################################

// exported functions
extern "C" {

dwStatus _dwSensorPlugin_createHandle(dwSensorPluginSensorHandle_t* sensor,
                                      dwSensorPluginProperties*,
                                      char const*,
                                      dwContextHandle_t ctx)
{
    if (!sensor)
    {
        return DW_INVALID_ARGUMENT;
    }

    auto sensorContext = std::unique_ptr<dw::plugin::lidar::NVLidar>(new dw::plugin::lidar::NVLidar(ctx));
    dw::plugin::lidar::NVLidar::g_sensorContext.push_back(move(sensorContext));
    *sensor = static_cast<dwSensorPluginSensorHandle_t>(dw::plugin::lidar::NVLidar::g_sensorContext.back().get());

    return DW_SUCCESS;
}
//################################################################################

dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    //Check if sensor in sensor list
    auto iter = std::find_if(dw::plugin::lidar::NVLidar::g_sensorContext.begin(),
                             dw::plugin::lidar::NVLidar::g_sensorContext.end(),
                             [&sensor](std::unique_ptr<dw::plugin::lidar::NVLidar>& lidarSensor) {
                                 return (lidarSensor.get() == sensor);
                             });

    //If sensor in list remove it
    if (iter != dw::plugin::lidar::NVLidar::g_sensorContext.end())
    {
        // Stop decoding process
        ret = sensor->stopSensor();
        if (ret != DW_SUCCESS)
        {
            return ret;
        }

        // Release resources claimed
        ret = sensor->releaseSensor();
        if (ret != DW_SUCCESS)
        {
            return ret;
        }

        // Remove sensor instance from context vector
        dw::plugin::lidar::NVLidar::g_sensorContext.erase(iter);
        return DW_SUCCESS;
    }

    //If sensor was not found in sensor list
    return DW_FAILURE;
}
//################################################################################

dwStatus _dwSensorPlugin_createSensor(char const* params,
                                      dwSALHandle_t sal,
                                      dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->createSensor(sal, params);
}
//################################################################################

dwStatus _dwSensorPlugin_start(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    if (!sensor->isVirtualSensor())
    {
        return sensor->startSensor();
    }

    return DW_SUCCESS;
}
//################################################################################

dwStatus _dwSensorPlugin_stop(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->stopSensor();
}
//################################################################################

dwStatus _dwSensorPlugin_reset(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->resetSensor();
}
//################################################################################

dwStatus _dwSensorPlugin_readRawData(uint8_t const** data,
                                     size_t* size,
                                     dwTime_t* timestamp,
                                     dwTime_t timeout_us,
                                     dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->readRawData(data, size, timestamp, timeout_us);
}
//################################################################################

dwStatus _dwSensorPlugin_returnRawData(uint8_t const* data,
                                       dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->returnRawData(data);
}
//################################################################################

dwStatus _dwSensorPlugin_pushData(size_t* lenPushed,
                                  uint8_t const* data,
                                  size_t const size,
                                  dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->pushData(lenPushed, data, size);
}

//################################################################################
//###################### Lidar Specific Plugin Functions #########################
//################################################################################
dwStatus _dwSensorLidarPlugin_parseDataBuffer(dwLidarDecodedPacket* output,
                                              const dwTime_t hostTimestamp,
                                              dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->parseDataBuffer(output, hostTimestamp);
}
//################################################################################

dwStatus _dwSensorLidarPlugin_getConstants(_dwSensorLidarDecoder_constants* constants,
                                           dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::lidar::NVLidar* sensor = static_cast<dw::plugin::lidar::NVLidar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->getConstants(constants);
}
//################################################################################

//################################################################################
//################# Sensor Class <-> Plugin Function Mapping #####################
//################################################################################
dwStatus dwSensorLidarPlugin_getFunctionTable(dwSensorLidarPluginFunctionTable* functions)
{
    if (functions == nullptr)
    {
        return DW_INVALID_ARGUMENT;
    }

    //Map common functions
    functions->common = {
        _dwSensorPlugin_createHandle,
        _dwSensorPlugin_createSensor,
        _dwSensorPlugin_release,
        _dwSensorPlugin_start,
        _dwSensorPlugin_stop,
        _dwSensorPlugin_reset,
        _dwSensorPlugin_readRawData,
        _dwSensorPlugin_returnRawData,
        _dwSensorPlugin_pushData};

    //Map lidar specific functions
    functions->parseDataBuffer     = _dwSensorLidarPlugin_parseDataBuffer;
    functions->getDecoderConstants = _dwSensorLidarPlugin_getConstants;

    return DW_SUCCESS;
}

} // extern "C"
