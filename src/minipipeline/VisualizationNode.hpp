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
// SPDX-FileCopyrightText: Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SMP_NODES_VISUALIZATIONNODE_HPP_
#define SMP_NODES_VISUALIZATIONNODE_HPP_

#include <dw/calibration/cameramodel/CameraModel.h>
#include <dw/control/vehicleio/VehicleIO.h>
#include <dw/image/Image.h>
#include <dw/pointcloudprocessing/pointcloud/PointCloud.h>
#include <dw/sensors/radar/Radar.h>
#include <dwcgf/channel/Channel.hpp>
#include <dwcgf/node/Node.hpp>
#include <dwcgf/parameter/ParameterDescriptor.hpp>
#include <dwcgf/parameter/SemanticParameterTypes.hpp>
#include <dwcgf/pass/Pass.hpp>
#include <dwcgf/pass/PassDescriptor.hpp>
#include <dwcgf/port/Port.hpp>
#include <dwcgf/port/PortDescriptor.hpp>
#include <dwframework/dwnodes/common/channelpackets/Codec.hpp>
#include <dwframework/dwnodes/common/channelpackets/IMU.hpp>
#include <dwframework/dwnodes/common/channelpackets/Image.hpp>
#include <dwframework/dwnodes/common/channelpackets/PointCloud.hpp>
#include <dwframework/dwnodes/common/channelpackets/Radar.hpp>
#include <dwframework/dwnodes/common/channelpackets/SensorCommonTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/VehicleIOValStructures.hpp>
#include <dwframework/dwnodes/common/DetectorTypes.hpp>
#include <dwframework/dwnodes/common/DwRoadCastCommonTypes.hpp>
#include <dwframework/dwnodes/common/DwavEnums.hpp>
#include <dwframework/dwnodes/common/EnumDescriptions.hpp>
#include <dwframework/dwnodes/common/GlobalEgomotionCommonTypes.hpp>
#include <dwframework/dwnodes/common/PointCloudProcessingCommonTypes.hpp>
#include <dwframework/dwnodes/common/SelfCalibrationTypes.hpp>
#include <dwframework/dwnodes/common/SensorCommonImpl.hpp>
#include <dwframework/dwnodes/common/SensorCommonTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/Camera.hpp>
#include <dwframework/dwnodes/common/channelpackets/Codec.hpp>
#include <dwframework/dwnodes/common/channelpackets/DopplerMotionEstimator.hpp>
#include <dwframework/dwnodes/common/channelpackets/DwRoadCastCommonTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/DwframeworkTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/EgomotionState.hpp>
#include <dwframework/dwnodes/common/channelpackets/FeatureList.hpp>
#include <dwframework/dwnodes/common/channelpackets/GPS.hpp>
#include <dwframework/dwnodes/common/channelpackets/GlobalEgomotionCommonTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/IMU.hpp>
#include <dwframework/dwnodes/common/channelpackets/Image.hpp>
#include <dwframework/dwnodes/common/channelpackets/LidarPointCloud.hpp>
#include <dwframework/dwnodes/common/channelpackets/PointCloud.hpp>
#include <dwframework/dwnodes/common/channelpackets/PointCloudProcessingCommonTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/Pyramid.hpp>
#include <dwframework/dwnodes/common/channelpackets/Radar.hpp>
#include <dwframework/dwnodes/common/channelpackets/Rig.hpp>
#include <dwframework/dwnodes/common/channelpackets/SelfCalibrationTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/SensorCommonTypes.hpp>
#include <dwframework/dwnodes/common/channelpackets/Sensors.hpp>
#include <dwframework/dwnodes/common/channelpackets/Tensor.hpp>
#include <dwframework/dwnodes/common/channelpackets/VehicleIOLegacyStructures.hpp>
#include <dwframework/dwnodes/common/channelpackets/VehicleIOValStructures.hpp>
#ifdef PERCEPTION_ENABLED
#include "YoloNet.hpp"
#endif

namespace minipipeline
{

static constexpr size_t MAX_CAMERA_NAME_LENGTH = 64;
static constexpr size_t MAX_CAMERA_COUNT       = 9;
static constexpr size_t MAX_RADAR_COUNT        = 8;
static constexpr size_t MAX_LIDAR_COUNT        = 8;

struct VisualizationNodeParams
{
    dw::core::FixedString<MAX_CAMERA_NAME_LENGTH> cameraNames[MAX_CAMERA_COUNT];
    uint32_t imageWidth[MAX_CAMERA_COUNT]{};
    uint32_t imageHeight[MAX_CAMERA_COUNT]{};
    bool camEnable[MAX_CAMERA_COUNT]{};
    size_t masterCameraIndex{};
    cudaStream_t stream{};
    bool offscreen{};
    bool fullscreen{};
    uint32_t winSizeW{};
    uint32_t winSizeH{};

    dwTransformation3f extrinsics[MAX_CAMERA_COUNT]{};
};

class VisualizationNode : public dw::framework::ExceptionSafeProcessNode
{
public:
    static constexpr auto describeInputPorts()
    {
        using namespace dw;
        using namespace dw::framework;
        return describePortCollection(
            DW_DESCRIBE_PORT_ARRAY(dwImageHandle_t, MAX_CAMERA_COUNT, "IMAGE"_sv, PortBinding::REQUIRED),
            DW_DESCRIBE_PORT(dwRadarScan, "RADAR_PROCESSED_DATA"_sv),
            DW_DESCRIBE_PORT(dwIMUFrame, "IMU_FRAME"_sv, PortBinding::OPTIONAL),
            DW_DESCRIBE_PORT_ARRAY(dwPointCloud, MAX_LIDAR_COUNT, "LIDAR_POINT_CLOUD"_sv),
            DW_DESCRIBE_PORT_ARRAY(dwLidarPacketsArray, MAX_LIDAR_COUNT, "LIDAR_PACKET_ARRAYS"_sv),
            DW_DESCRIBE_PORT(dwVehicleIOSafetyState, "VIO_SAFETY_STATE"_sv, PortBinding::OPTIONAL),
            DW_DESCRIBE_PORT(dwVehicleIONonSafetyState, "VIO_NON_SAFETY_STATE"_sv, PortBinding::OPTIONAL));
#ifdef PERCEPTION_ENABLED
        DW_DESCRIBE_PORT(YoloScoreRectArray, "BOX_ARR"_sv)
        ,
            DW_DESCRIBE_PORT(uint32_t, "BOX_NUM"_sv),
#endif
    };

    static constexpr auto describeOutputPorts()
    {
        using namespace dw;
        using namespace dw::framework;
        return dw::framework::describePortCollection(
            DW_DESCRIBE_PORT(dwVehicleIOSafetyCommand, "VIO_SAFETY_CMD"_sv, PortBinding::OPTIONAL));
    };

    static constexpr auto describePasses()
    {
        using namespace dw;
        using namespace dw::framework;
        return describePassCollection(
            describePass("SETUP"_sv, DW_PROCESSOR_TYPE_CPU),
            describePass("ACQUIRE_FRAME"_sv, DW_PROCESSOR_TYPE_CPU),
            describePass("RENDER_FRAME"_sv, DW_PROCESSOR_TYPE_CPU),
            describePass("RENDER_INFO_BAR"_sv, DW_PROCESSOR_TYPE_CPU),
            describePass("RENDER_DEBUG"_sv, DW_PROCESSOR_TYPE_CPU),
            describePass("TEARDOWN"_sv, DW_PROCESSOR_TYPE_CPU));
    }

    static std::unique_ptr<VisualizationNode> create(dw::framework::ParameterProvider& provider);

    static constexpr auto describeParameters()
    {
        using namespace dw;
        using namespace dw::framework;
        return describeConstructorArguments<VisualizationNodeParams, dwContextHandle_t>(
            describeConstructorArgument(
                DW_DESCRIBE_UNNAMED_ARRAY_PARAMETER_WITH_SEMANTIC(
                    bool,
                    semantic_parameter_types::CameraEnabled,
                    MAX_CAMERA_COUNT,
                    &VisualizationNodeParams::camEnable),
                DW_DESCRIBE_UNNAMED_ARRAY_PARAMETER_WITH_SEMANTIC(
                    dw::core::FixedString<MAX_CAMERA_NAME_LENGTH>,
                    semantic_parameter_types::CameraName,
                    MAX_CAMERA_COUNT,
                    &VisualizationNodeParams::cameraNames),
                DW_DESCRIBE_UNNAMED_ARRAY_PARAMETER_WITH_SEMANTIC(
                    uint32_t,
                    semantic_parameter_types::ImageWidth,
                    MAX_CAMERA_COUNT,
                    &VisualizationNodeParams::imageWidth),
                DW_DESCRIBE_UNNAMED_ARRAY_PARAMETER_WITH_SEMANTIC(
                    uint32_t,
                    semantic_parameter_types::ImageHeight,
                    MAX_CAMERA_COUNT,
                    &VisualizationNodeParams::imageHeight),
                DW_DESCRIBE_PARAMETER(
                    size_t,
                    "masterCameraIndex"_sv,
                    &VisualizationNodeParams::masterCameraIndex),
                DW_DESCRIBE_PARAMETER(
                    bool,
                    "fullscreen"_sv,
                    &VisualizationNodeParams::fullscreen),
                DW_DESCRIBE_PARAMETER(
                    bool,
                    "offscreen"_sv,
                    &VisualizationNodeParams::offscreen),
                DW_DESCRIBE_PARAMETER(
                    uint32_t,
                    "winSizeW"_sv,
                    &VisualizationNodeParams::winSizeW),
                DW_DESCRIBE_PARAMETER(
                    uint32_t,
                    "winSizeH"_sv,
                    &VisualizationNodeParams::winSizeH),
                DW_DESCRIBE_INDEX_PARAMETER(
                    cudaStream_t,
                    "streamIndex"_sv,
                    &VisualizationNodeParams::stream)),
            describeConstructorArgument(
                DW_DESCRIBE_UNNAMED_PARAMETER(
                    dwContextHandle_t)));
    }

    VisualizationNode(const VisualizationNodeParams& params, const dwContextHandle_t ctx);
};

} // namespace minipipeline

#endif
