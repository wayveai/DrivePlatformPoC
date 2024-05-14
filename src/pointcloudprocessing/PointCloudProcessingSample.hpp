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
// SPDX-FileCopyrightText: Copyright (c) 2017-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// Driveworks sample includes
#include <dw/core/base/Version.h>
#include <framework/DriveWorksSample.hpp>
// HAL
#include <dw/sensors/sensormanager/SensorManager.h>

// Egomotion
#include <dw/egomotion/base/Egomotion.h>

// Vehicle IO
#include <dw/control/vehicleio/VehicleIO.h>

// Point Cloud Processor
#include <dw/pointcloudprocessing/pointcloud/PointCloud.h>
#include <dw/pointcloudprocessing/icp/PointCloudICP.h>
#include <dw/pointcloudprocessing/accumulator/PointCloudAccumulator.h>
#include <dw/pointcloudprocessing/stitcher/PointCloudStitcher.h>
#include <dw/pointcloudprocessing/planeextractor/PointCloudPlaneExtractor.h>
#include <dw/pointcloudprocessing/rangeimagecreator/PointCloudRangeImageCreator.h>

// Renderer
#include <dwvisualization/core/RenderEngine.h>

#include <deque>

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Point Cloud Processor Sample
// The sample demonstrates how to use point cloud processor APIs
//
//------------------------------------------------------------------------------
class PointCloudProcessingSample : public dw_samples::common::DriveWorksSample
{
private:
    // ------------------------------------------------
    // Global Constants
    // ------------------------------------------------
    // Number of stitched point clouds to visualize
    static const uint32_t MAX_SPINS_TO_VISUALIZE = 5;
    // Number of points to be used for plane extraction
    const size_t NUM_POINTS_FOR_GROUND_DETECTION = 20000;
    // Max number of point clouds
    static const uint32_t MAX_POINT_CLOUDS   = 3;
    static const uint32_t DEPTH_IMAGE_HEIGHT = 96;
    // ------------------------------------------------
    // Member variables
    // ------------------------------------------------
    dwContextHandle_t m_context             = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz    = DW_NULL_HANDLE;
    dwSALHandle_t m_sal                     = DW_NULL_HANDLE;
    dwRigHandle_t m_rigConfig               = DW_NULL_HANDLE;
    dwSensorManagerHandle_t m_sensorManager = DW_NULL_HANDLE;
    dwEgomotionHandle_t m_egomotion         = DW_NULL_HANDLE;
    dwVehicleIOHandle_t m_vehicleIO         = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine   = DW_NULL_HANDLE;
    dwImageStreamerHandle_t m_image2GL      = DW_NULL_HANDLE;
    dwImageHandle_t m_imageHandleRGBA       = DW_NULL_HANDLE;

    dwVehicleIOSafetyState m_prevVehicleSafeState;
    dwVehicleIOSafetyState m_currVehicleSafeState;
    dwVehicleIONonSafetyState m_prevVehicleNonSafeState;
    dwVehicleIONonSafetyState m_currVehicleNonSafeState;
    dwVehicleIOActuationFeedback m_prevVehicleActuationFeedbackState;
    dwVehicleIOActuationFeedback m_currVehicleActuationFeedbackState;
    const dwVehicle* m_vehicle;

    // Point Cloud Processor Handles
    dwPointCloudICPHandle_t m_icp                                   = DW_NULL_HANDLE;
    dwPointCloudStitcherHandle_t m_coordinateConverter              = DW_NULL_HANDLE;
    dwPointCloudStitcherHandle_t m_stitcher                         = DW_NULL_HANDLE;
    dwPointCloudRangeImageCreatorHandle_t m_rangeImageCreator       = DW_NULL_HANDLE;
    dwPointCloudPlaneExtractorHandle_t m_planeExtractor             = DW_NULL_HANDLE;
    dwPointCloudAccumulatorHandle_t m_accumulator[MAX_POINT_CLOUDS] = {DW_NULL_HANDLE};
    dwPointCloudExtractedPlane m_groundPlane;

    // Input file for defining sensors
    std::string m_rigFile;
    // Maximum number of ICP iterations to do
    uint32_t m_maxIters;
    // Maximum number of Lidar spins to process
    uint32_t m_numFrames;
    // Counter for number of lidar spins to process
    uint32_t m_spinNum = 0;

    // Rendering variables
    typedef struct WindowTile
    {
        uint32_t tileId;
        uint32_t renderBufferId;
    } WindowTile;

    WindowTile m_lidarTiles[MAX_POINT_CLOUDS];
    WindowTile m_stitchedTile;
    WindowTile m_imageTile;
    WindowTile m_trajectoryTile; // point cloud trajectory

    // This is the ICP Transform that starts at I, All transforms are accumulated to here.
    dwTransformation3f m_currentRigToWorld = DW_IDENTITY_TRANSFORMATION3F;
    // ICP prior pose used for initialization
    dwTransformation3f m_icpInitialPose = DW_IDENTITY_TRANSFORMATION3F;
    // ICP detla pose after the optimization
    dwTransformation3f m_icpRefinedPose = DW_IDENTITY_TRANSFORMATION3F;

    uint32_t m_lidarCount       = 0;
    dwTime_t m_registrationTime = DW_TIMEOUT_INFINITE; // reference timestamp for motion correction during stitching

    dwLidarProperties m_lidarProps[MAX_POINT_CLOUDS];
    dwTransformation3f m_sensorToRigs[MAX_POINT_CLOUDS];
    uint32_t m_lidarOverflowCount[MAX_POINT_CLOUDS];
    bool m_lidarAccumulated[MAX_POINT_CLOUDS];
    std::vector<dwTransformation3f> m_rigToWorldHistory;

    // Point Cloud Buffers
    dwPointCloud m_accumulatedPoints[MAX_POINT_CLOUDS]; // accumulated point clouds for each individual sensor
    dwImageHandle_t m_stitchedDepthImage;               // range image
    dwImageHandle_t m_stitchedDepthImageHost;           // range image on the host
    dwPointCloud m_stitchedPoints;                      // current registered point cloud (unorganized)
    dwPointCloud m_stitchedPointsHost;                  // cpu version of registered point cloud
    dwPointCloud m_stitchedDepthMap3D;                  // organized stitched point cloud
    dwPointCloud m_stitchedDepthMap3DPrev;              // previous organized stitched point cloud
    dwPointCloud m_transformedPoints;                   // point cloud transformed to other coordinates
    dwPointCloud m_groundInliers;                       // inliers of the extracted ground plane
    dwPointCloud m_groundOutliers;                      // outliers of the extracted ground plane

    // CUDA/GL
    cudaStream_t m_stream;
    dwImageCUDA* m_imageCUDA = nullptr;
    dwImageCPU* m_imageRGBA  = nullptr;

    // true if GPU version of point cloud processor is used
    bool m_renderMasterView = true;

    // ------------------------------------------------
    // Member functions
    // ------------------------------------------------
    void initVehicle();

    void initSensors();

    void initBuffers();

    void initAccumulation();

    void initStitching();

    void initRangeImages();

    void initSegmentation();

    void initICP();

    void initTransformation();

    void initMasterViewRendering(dwRenderEngineTileState tileState);

    void initTrajectoryViewRendering(dwRenderEngineTileState tileParam);

    void initRendering();

    bool getSpin();

    bool runLoop();

    void sampleForGroundDetection(const dwPointCloud* pointCloud);

    void checkDeviceType(const dwLidarProperties& prop);

    void copyToRenderBuffer(uint32_t renderBufferId, uint32_t offset, const dwPointCloud& pointCloud);

    void renderMasterView();

    void renderTrajectoryView();

    void renderPlaneNormalVector();

    void renderPointCloud(uint32_t renderBufferId,
                          uint32_t tileId,
                          uint32_t offset,
                          dwRenderEngineColorRGBA color,
                          const dwPointCloud& pointCloud);

    void renderCoordinate(const dwTransformation3f& rigToWorld);

    void renderTexts(const char* msg, const dwVector2f& location);

    void renderRangeImage(dwImageHandle_t image);

    void makeRGBAImage(dwImageCPU* imageRGBA,
                       const dwImageCPU* rangeImage);

public:
    ///------------------------------------------------------------------------------
    ///  initialize sample
    ///------------------------------------------------------------------------------
    PointCloudProcessingSample(const ProgramArguments& args);

    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initDriveWorks(dwContextHandle_t& context) const;

    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers
    /// -----------------------------
    bool onInitialize() override;

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override;

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///------------------------------------------------------------------------------
    void onProcess() override;

    ///------------------------------------------------------------------------------
    /// Render loop
    ///------------------------------------------------------------------------------
    void onRender() override;

    void onResizeWindow(int width, int height) override;

    void onKeyDown(int32_t key, int32_t scancode, int32_t mods) override;
};
