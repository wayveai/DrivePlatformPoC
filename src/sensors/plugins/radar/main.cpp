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
// SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

//STD
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

//DW
#include <dw/sensors/plugins/radar/RadarPlugin.h>

//Project specific
#include "NVRadar.hpp"

std::vector<std::unique_ptr<dw::plugin::radar::NVRadar>> dw::plugin::radar::NVRadar::g_sensorContext;

//################################################################################
//############################### Helper Functions ###############################
//################################################################################
static dwStatus IsValidSensor(dw::plugin::radar::NVRadar* sensor)
{
    for (auto& i : dw::plugin::radar::NVRadar::g_sensorContext)
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

    auto sensorContext = std::unique_ptr<dw::plugin::radar::NVRadar>(new dw::plugin::radar::NVRadar(ctx));
    dw::plugin::radar::NVRadar::g_sensorContext.push_back(move(sensorContext));
    *sensor = static_cast<dwSensorPluginSensorHandle_t>(dw::plugin::radar::NVRadar::g_sensorContext.back().get());

    return DW_SUCCESS;
}
//################################################################################

dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    //Check if sensor in sensor list
    auto iter = std::find_if(dw::plugin::radar::NVRadar::g_sensorContext.begin(),
                             dw::plugin::radar::NVRadar::g_sensorContext.end(),
                             [&sensor](std::unique_ptr<dw::plugin::radar::NVRadar>& radarSensor) {
                                 return (radarSensor.get() == sensor);
                             });

    //If sensor in list remove it
    if (iter != dw::plugin::radar::NVRadar::g_sensorContext.end())
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
        dw::plugin::radar::NVRadar::g_sensorContext.erase(iter);
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
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
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
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
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
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
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
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
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
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
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
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
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
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->pushData(lenPushed, data, size);
}

//################################################################################
//###################### Radar Specific Plugin Functions #########################
//################################################################################
dwStatus _dwSensorRadarPlugin_parseDataBuffer(dwRadarScan* output,
                                              const dwRadarScanType scanType,
                                              dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->parseDataBuffer(output, scanType);
}
//################################################################################

dwStatus _dwSensorRadarPlugin_getConstants(_dwSensorRadarDecoder_constants* constants,
                                           dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->getConstants(constants);
}
//################################################################################

dwStatus _dwSensorRadarPlugin_validatePacket(const char* rawData,
                                             size_t size,
                                             dwRadarScanType* messageType,
                                             dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->validatePacket(rawData, size, messageType);
}
//################################################################################

dwStatus _dwSensorRadarPlugin_setVehicleState(const dwRadarVehicleState* state,
                                              dwSensorPluginSensorHandle_t handle)
{
    dw::plugin::radar::NVRadar* sensor = static_cast<dw::plugin::radar::NVRadar*>(handle);
    dwStatus ret                       = IsValidSensor(sensor);
    if (ret != DW_SUCCESS)
    {
        return ret;
    }

    return sensor->setVehicleState(state);
}

//################################################################################
//################# Sensor Class <-> Plugin Function Mapping #####################
//################################################################################
dwStatus dwSensorRadarPlugin_getFunctionTable(dwSensorRadarPluginFunctionTable* functions)
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

    //Map radar specific functions
    functions->parseDataBuffer     = _dwSensorRadarPlugin_parseDataBuffer;
    functions->getDecoderConstants = _dwSensorRadarPlugin_getConstants;
    functions->validatePacket      = _dwSensorRadarPlugin_validatePacket;
    functions->setVehicleState     = _dwSensorRadarPlugin_setVehicleState;

    return DW_SUCCESS;
}

} // extern "C"
