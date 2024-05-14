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
// SPDX-FileCopyrightText: Copyright (c) 2017-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <framework/WindowGLFW.hpp>
#include <framework/MathUtils.hpp>
#include <framework/Mat4.hpp>
#include <framework/ChecksExt.hpp>
#include <dw/sensors/Sensors.h>
#include "PointCloudProcessingSample.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::checkDeviceType(const dwLidarProperties& prop)
{
    if (std::string("VELO_HDL64E") != prop.deviceString &&
        std::string("VELO_HDL32E") != prop.deviceString)
    {
        throw std::runtime_error("Lidar recording was captured by " + std::string(prop.deviceString) +
                                 ", this sample only supports Velodyne HDL32E or HDL64E.\n");
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initVehicle()
{
    // Rig
    {
        CHECK_DW_ERROR_MSG(dwRig_initializeFromFile(&m_rigConfig, m_context, m_rigFile.c_str()),
                           "Could not initialize Rig from File");
        CHECK_DW_ERROR_MSG(dwRig_getVehicle(&m_vehicle, m_rigConfig), "Could not get Vehicle from Rig");
    }

    // Vehicle IO
    CHECK_DW_ERROR(dwVehicleIO_initialize(&m_vehicleIO, DW_VEHICLEIO_DATASPEED, m_vehicle, m_context));

    // Egomotion
    {
        dwEgomotionParameters egomotionParams{};
        CHECK_DW_ERROR_MSG(dwEgomotion_initParamsFromRig(&egomotionParams, m_rigConfig, "imu", "can"),
                           "Could not get egomotion parameters from rig");

        // Initialize Egomomotion parameters
        egomotionParams.motionModel     = DW_EGOMOTION_IMU_ODOMETRY;
        egomotionParams.automaticUpdate = true; // update automatically at least every 5ms
        egomotionParams.historySize     = 1000; // dw-ego default 1000
        CHECK_DW_ERROR(dwEgomotion_initialize(&m_egomotion, &egomotionParams, m_context));
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initSensors()
{
    // Sensor Manager
    CHECK_DW_ERROR(dwSensorManager_initializeFromRig(&m_sensorManager, m_rigConfig, DW_SENSORMANGER_MAX_NUM_SENSORS, m_sal));

    uint32_t imuCount = 0;
    uint32_t canCount = 0;
    CHECK_DW_ERROR(dwSensorManager_getNumSensors(&m_lidarCount, DW_SENSOR_LIDAR, m_sensorManager));
    CHECK_DW_ERROR(dwSensorManager_getNumSensors(&imuCount, DW_SENSOR_IMU, m_sensorManager));
    CHECK_DW_ERROR(dwSensorManager_getNumSensors(&canCount, DW_SENSOR_CAN, m_sensorManager));

    bool imuFound   = imuCount > 0 ? true : false;
    bool canFound   = canCount > 0 ? true : false;
    bool lidarFound = m_lidarCount > 0 ? true : false;

    if (!imuFound || !canFound || !lidarFound)
    {
        logError("IMU, CAN, and Lidar are required for this sample");
        stop();
    }

    if (m_lidarCount > MAX_POINT_CLOUDS)
    {
        logError("Sample only supports up to 3 lidars");
        stop();
    }

    CHECK_DW_ERROR(dwSensorManager_start(m_sensorManager));

    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        uint32_t lidarSensorIndex;
        dwSensorHandle_t lidarHandle;
        CHECK_DW_ERROR(dwSensorManager_getSensorIndex(&lidarSensorIndex, DW_SENSOR_LIDAR, i, m_sensorManager));
        CHECK_DW_ERROR(dwSensorManager_getSensorHandle(&lidarHandle, lidarSensorIndex, m_sensorManager));
        CHECK_DW_ERROR(dwSensorLidar_getProperties(&m_lidarProps[i], lidarHandle));
        checkDeviceType(m_lidarProps[i]);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initBuffers()
{
    dwMemoryType memoryType = DW_MEMORY_TYPE_CUDA;

    // initialize the memory stograge before buffer creation
    uint32_t capacity = 0;
    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        // point cloud accumulator
        m_accumulatedPoints[i].capacity = m_lidarProps[i].pointsPerSpin;
        m_accumulatedPoints[i].type     = memoryType;
        m_accumulatedPoints[i].format   = DW_POINTCLOUD_FORMAT_XYZI;

        capacity += m_lidarProps[i].pointsPerSpin;

        CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_accumulatedPoints[i]));
    }

    // stitching from multiple accumulated cloud of points
    m_stitchedPoints.capacity = capacity;
    m_stitchedPoints.type     = memoryType;
    m_stitchedPoints.format   = DW_POINTCLOUD_FORMAT_XYZI;

    // host memory
    m_stitchedPointsHost.capacity = capacity;
    m_stitchedPointsHost.type     = DW_MEMORY_TYPE_CPU;
    m_stitchedPointsHost.format   = DW_POINTCLOUD_FORMAT_XYZI;

    CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_stitchedPoints));

    // This is needed to host the data transferred back from CUDA
    // The subsample of the point clouds for the plane extraction
    // will access to this memory
    CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_stitchedPointsHost));

    // transformation of the stitched point cloudd
    m_transformedPoints.capacity = m_stitchedPoints.capacity;
    m_transformedPoints.type     = memoryType;
    m_transformedPoints.format   = DW_POINTCLOUD_FORMAT_XYZI;

    CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_transformedPoints));

    // ground plane extraction
    m_groundInliers.capacity  = static_cast<uint32_t>(NUM_POINTS_FOR_GROUND_DETECTION);
    m_groundOutliers.capacity = static_cast<uint32_t>(NUM_POINTS_FOR_GROUND_DETECTION);
    m_groundInliers.type      = DW_MEMORY_TYPE_CPU;
    m_groundOutliers.type     = DW_MEMORY_TYPE_CPU;
    m_groundInliers.format    = DW_POINTCLOUD_FORMAT_XYZI;
    m_groundOutliers.format   = DW_POINTCLOUD_FORMAT_XYZI;

    CHECK_DW_ERROR_MSG(dwPointCloud_createBuffer(&m_groundInliers), "Could not create buffer for CPU buffer");
    CHECK_DW_ERROR_MSG(dwPointCloud_createBuffer(&m_groundOutliers), "Could not create buffer for CPU buffer");
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initAccumulation()
{
    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        dwPointCloudAccumulatorParams params{};
        CHECK_DW_ERROR(dwPointCloudAccumulator_getDefaultParams(&params));
        params.organized                = false;
        params.enableMotionCompensation = true;
        params.egomotion                = m_egomotion;
        params.memoryType               = DW_MEMORY_TYPE_CUDA;
        uint32_t lidarSensorIndex;
        CHECK_DW_ERROR(dwSensorManager_getSensorIndex(&lidarSensorIndex, DW_SENSOR_LIDAR, i, m_sensorManager));
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&params.sensorTransformation,
                                                          lidarSensorIndex, m_rigConfig));
        CHECK_DW_ERROR(dwPointCloudAccumulator_initialize(&m_accumulator[i], &params, &m_lidarProps[i], m_context));
        CHECK_DW_ERROR(dwPointCloudAccumulator_bindOutput(&m_accumulatedPoints[i], m_accumulator[i]));
        CHECK_DW_ERROR(dwPointCloudAccumulator_setCUDAStream(m_stream, m_accumulator[i]));
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initStitching()
{
    CHECK_DW_ERROR(dwPointCloudStitcher_initialize(&m_stitcher, m_context));

    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        uint32_t lidarSensorIndex;
        CHECK_DW_ERROR(dwSensorManager_getSensorIndex(&lidarSensorIndex, DW_SENSOR_LIDAR, i, m_sensorManager));

        dwTransformation3f sensorToRig{};
        CHECK_DW_ERROR(dwRig_getSensorToRigTransformation(&sensorToRig, lidarSensorIndex, m_rigConfig));

        m_sensorToRigs[i] = sensorToRig;

        CHECK_DW_ERROR(dwPointCloudStitcher_bindInput(static_cast<dwBindSlot>(i + 1),
                                                      &m_accumulatedPoints[i],
                                                      &m_sensorToRigs[i],
                                                      m_stitcher));
    }

    CHECK_DW_ERROR(dwPointCloudStitcher_bindOutput(&m_stitchedPoints, m_stitcher));

    CHECK_DW_ERROR(dwPointCloudStitcher_setCUDAStream(m_stream, m_stitcher));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initRangeImages()
{
    dwPointCloudRangeImageCreatorParams params{};
    CHECK_DW_ERROR(dwPCRangeImageCreator_getDefaultParams(&params));

    uint32_t imageHeight = DEPTH_IMAGE_HEIGHT;
    uint32_t imageWidth  = m_stitchedPoints.capacity / DEPTH_IMAGE_HEIGHT;
    // check if we do not exceed a maximum depth map size
    uint32_t maxDepthMapSize;
    CHECK_DW_ERROR(dwPointCloudICP_getMaximumDepthMapSize(&maxDepthMapSize));
    if (maxDepthMapSize < imageHeight * imageWidth)
    {
        imageWidth = maxDepthMapSize / imageHeight;
    }

    params.memoryType                                     = DW_MEMORY_TYPE_CUDA;
    params.maxInputPoints                                 = m_stitchedPoints.capacity;
    params.height                                         = imageHeight;
    params.width                                          = imageWidth;
    params.clippingParams.minElevationRadians             = -DEG2RAD(15.f);
    params.clippingParams.maxElevationRadians             = DEG2RAD(15.f);
    params.clippingParams.orientedBoundingBox.center      = {0.f, 0.f, 0.f};
    params.clippingParams.orientedBoundingBox.rotation    = DW_IDENTITY_MATRIX3F;
    params.clippingParams.orientedBoundingBox.halfAxisXYZ = {m_vehicle->length / 2.f,
                                                             m_vehicle->width / 2.f,
                                                             m_vehicle->height / 2.f};

    CHECK_DW_ERROR(dwPCRangeImageCreator_initialize(&m_rangeImageCreator, &params, m_context));
    CHECK_DW_ERROR(dwPCRangeImageCreator_bindInput(&m_stitchedPoints, m_rangeImageCreator));

    // create depth images
    dwImageProperties props{};
    CHECK_DW_ERROR(dwPCRangeImageCreator_getImageProperties(&props, m_rangeImageCreator));

    CHECK_DW_ERROR(dwImage_create(&m_stitchedDepthImage, props, m_context));

    props.type = DW_IMAGE_CPU;
    CHECK_DW_ERROR(dwImage_create(&m_stitchedDepthImageHost, props, m_context));

    // output point clouds
    m_stitchedDepthMap3D.capacity = imageWidth * imageHeight;
    m_stitchedDepthMap3D.type     = m_stitchedPoints.type;
    m_stitchedDepthMap3D.format   = DW_POINTCLOUD_FORMAT_XYZI;

    m_stitchedDepthMap3DPrev.capacity = imageWidth * imageHeight;
    m_stitchedDepthMap3DPrev.type     = m_stitchedPoints.type;
    m_stitchedDepthMap3DPrev.format   = DW_POINTCLOUD_FORMAT_XYZI;

    CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_stitchedDepthMap3D));
    CHECK_DW_ERROR(dwPointCloud_createBuffer(&m_stitchedDepthMap3DPrev));

    // these point clouds are organized
    m_stitchedDepthMap3DPrev.organized = true;
    m_stitchedDepthMap3D.organized     = true;

    CHECK_DW_ERROR(dwPCRangeImageCreator_bindOutput(m_stitchedDepthImage, m_rangeImageCreator));
    CHECK_DW_ERROR(dwPCRangeImageCreator_bindPointCloudOutput(&m_stitchedDepthMap3D, m_rangeImageCreator));

    CHECK_DW_ERROR(dwPCRangeImageCreator_setCUDAStream(m_stream, m_rangeImageCreator));

    // map the memory to cuda image for rendering
    props.format       = DW_IMAGE_FORMAT_RGBA_UINT8;
    props.memoryLayout = DW_IMAGE_MEMORY_TYPE_DEFAULT;

    CHECK_DW_ERROR(dwImage_create(&m_imageHandleRGBA, props, m_context));
    CHECK_DW_ERROR(dwImage_getCPU(&m_imageRGBA, m_imageHandleRGBA));
    // CPU to GL streamer
    CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_image2GL, &props, DW_IMAGE_GL, m_context));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initSegmentation()
{
    dwPointCloudPlaneExtractorParams params{};
    CHECK_DW_ERROR(dwPCPlaneExtractor_getDefaultParameters(&params));
    params.cudaPipelineEnabled = false; // use CPU pipleine only for now

    params.maxInputPointCount              = m_stitchedPoints.capacity;
    params.minInlierFraction               = 0.5f;
    params.boxFilterParams.maxPointCount   = NUM_POINTS_FOR_GROUND_DETECTION;
    params.boxFilterParams.box.halfAxisXYZ = dwVector3f{8.f, 4.f, 1.f};

    CHECK_DW_ERROR(dwPCPlaneExtractor_initialize(&m_planeExtractor, &params, m_context));
    CHECK_DW_ERROR(dwPCPlaneExtractor_bindInput(&m_stitchedPointsHost, m_planeExtractor));
    CHECK_DW_ERROR(dwPCPlaneExtractor_bindOutput(&m_groundInliers, &m_groundOutliers, &m_groundPlane, m_planeExtractor));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initICP()
{
    dwImageProperties depthImageProps{};
    CHECK_DW_ERROR(dwPCRangeImageCreator_getImageProperties(&depthImageProps,
                                                            m_rangeImageCreator));

    dwPointCloudICPParams params{};
    CHECK_DW_ERROR(dwPointCloudICP_getDefaultParams(&params));
    params.maxIterations  = static_cast<uint16_t>(m_maxIters);
    params.depthmapSize.x = depthImageProps.width;
    params.depthmapSize.y = depthImageProps.height;
    params.maxPoints      = depthImageProps.width * depthImageProps.height;
    params.icpType        = dwPointCloudICPType::DW_POINT_CLOUD_ICP_TYPE_DEPTH_MAP;
    CHECK_DW_ERROR(dwPointCloudICP_initialize(&m_icp, &params, m_context));
    CHECK_DW_ERROR(dwPointCloudICP_bindInput(&m_stitchedDepthMap3D, &m_stitchedDepthMap3DPrev, &m_icpInitialPose, m_icp));
    CHECK_DW_ERROR(dwPointCloudICP_bindOutput(&m_icpRefinedPose, m_icp));
    CHECK_DW_ERROR(dwPointCloudICP_setCUDAStream(m_stream, m_icp));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initTransformation()
{
    CHECK_DW_ERROR(dwPointCloudStitcher_initialize(&m_coordinateConverter, m_context));

    CHECK_DW_ERROR(dwPointCloudStitcher_bindOutput(&m_transformedPoints, m_coordinateConverter));
    CHECK_DW_ERROR(dwPointCloudStitcher_setCUDAStream(m_stream, m_coordinateConverter))
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initMasterViewRendering(dwRenderEngineTileState tileParam)
{
    // tile size
    float32_t leftTileWidth   = 1.f / 5.f;
    float32_t leftTileHeight  = 1.f / 3.f;
    float32_t rightTileWidth  = 4.f / 5.f;
    float32_t rightTileHeight = 4.f / 5.f;

    // 1st tile
    tileParam.layout.viewport     = {0.f, 0.f, leftTileWidth, leftTileHeight};
    tileParam.layout.positionType = DW_RENDER_ENGINE_TILE_POSITION_TYPE_TOP_LEFT;
    CHECK_DW_ERROR(dwRenderEngine_addTile(&m_lidarTiles[0].tileId, &tileParam, m_renderEngine));

    // 2nd tile
    tileParam.layout.viewport = {0.f, leftTileHeight, leftTileWidth, leftTileHeight};
    CHECK_DW_ERROR(dwRenderEngine_addTile(&m_lidarTiles[1].tileId, &tileParam, m_renderEngine));

    // 3rd tile
    tileParam.layout.viewport = {0.f, 2.f * leftTileHeight, leftTileWidth, leftTileHeight};
    CHECK_DW_ERROR(dwRenderEngine_addTile(&m_lidarTiles[2].tileId, &tileParam, m_renderEngine));

    // 4th tile (stitched virtual Lidar)
    tileParam.layout.viewport = {leftTileWidth, 0.f, rightTileWidth, rightTileHeight};
    CHECK_DW_ERROR(dwRenderEngine_addTile(&m_stitchedTile.tileId, &tileParam, m_renderEngine));

    // 5th tile (range image tile)
    tileParam.layout.viewport = {leftTileWidth, rightTileHeight, rightTileWidth, 1.f / 5.f};
    CHECK_DW_ERROR(dwRenderEngine_addTile(&m_imageTile.tileId, &tileParam, m_renderEngine));

    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_lidarTiles[i].renderBufferId,
                                                   DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                                   sizeof(dwVector4f),
                                                   0,
                                                   m_lidarProps[i].pointsPerSpin,
                                                   m_renderEngine));
    }

    CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_stitchedTile.renderBufferId,
                                               DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                               sizeof(dwVector4f),
                                               0,
                                               m_stitchedPoints.capacity,
                                               m_renderEngine));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initTrajectoryViewRendering(dwRenderEngineTileState tileParam)
{
    dwRenderEngineTileState trajectoryTile = tileParam;
    trajectoryTile.layout.viewport         = {0.f, 0.f, 1.f, 1.f};
    trajectoryTile.layout.positionType     = DW_RENDER_ENGINE_TILE_POSITION_TYPE_TOP_LEFT;

    CHECK_DW_ERROR(dwRenderEngine_addTile(&m_trajectoryTile.tileId, &trajectoryTile, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_createBuffer(&m_trajectoryTile.renderBufferId,
                                               DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                                               sizeof(dwVector4f),
                                               0,
                                               m_transformedPoints.capacity * MAX_SPINS_TO_VISUALIZE,
                                               m_renderEngine));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initRendering()
{
    dwRenderEngineParams params{};
    CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params,
                                                    static_cast<uint32_t>(getWindowWidth()),
                                                    static_cast<uint32_t>(getWindowHeight())));
    CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

    CHECK_DW_ERROR(dwRenderEngine_initTileState(&params.defaultTile));

    dwRenderEngineTileState tileParam = params.defaultTile;
    tileParam.layout.sizeLayout       = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_RELATIVE;
    tileParam.layout.positionLayout   = DW_RENDER_ENGINE_TILE_LAYOUT_TYPE_RELATIVE;

    initMasterViewRendering(tileParam);
    initTrajectoryViewRendering(tileParam);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudProcessingSample::PointCloudProcessingSample(const ProgramArguments& args)
    : DriveWorksSample(args)
    , m_stream(cudaStreamDefault)
{
    m_rigFile   = getArgument("rigFile");
    m_numFrames = static_cast<uint32_t>(atoi(getArgument("numFrames").c_str()));
    m_maxIters  = static_cast<uint32_t>(atoi(getArgument("maxIters").c_str()));

    if (m_maxIters > 50)
        std::cerr << "`--maxIters` too large, set to " << (m_maxIters = 50) << std::endl;

    if (m_numFrames == 0)
        m_numFrames = static_cast<uint32_t>(-1);

    m_spinNum = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::initDriveWorks(dwContextHandle_t& context) const
{
    // initialize logger to print verbose message on console in color
    CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)));
    CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));

    // initialize SDK context, using data folder
    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = getEGLDisplay();
#endif

    CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
bool PointCloudProcessingSample::onInitialize()
{
    // -----------------------------------------
    // Initialize DriveWorks context and SAL
    // -----------------------------------------
    {
        initDriveWorks(m_context);
        CHECK_DW_ERROR_MSG(dwSAL_initialize(&m_sal, m_context), "Cannot initialize SAL");
        CHECK_CUDA_ERROR(cudaStreamCreate(&m_stream));
        CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));
    }

    initVehicle();

    initSensors();

    initBuffers();

    initAccumulation();

    initStitching();

    initRangeImages();

    initSegmentation();

    initICP();

    initTransformation();

    initRendering();

    // This is the first target. All points from next spins are put in the coordinates of this frame.
    {
        for (uint32_t i = 0; i < 10; ++i)
            getSpin();

        log("Skipped first 10 spins!\n");

        if (!getSpin())
        {
            log("No more data available to work with. Stop\n");
            return false;
        }
        std::swap(m_stitchedDepthMap3D, m_stitchedDepthMap3DPrev);
    }

    return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::onRelease()
{
    // -----------------------------------------
    // Release rig config
    // -----------------------------------------
    if (m_rigConfig)
        dwRig_release(m_rigConfig);

    // -----------------------------------------
    // Stop sensors
    // -----------------------------------------
    if (m_sensorManager)
    {
        dwSensorManager_stop(m_sensorManager);
        dwSensorManager_release(m_sensorManager);
    }

    // -----------------------------------------
    // Image
    // -----------------------------------------
    if (m_imageHandleRGBA)
        dwImage_destroy(m_imageHandleRGBA);

    if (m_image2GL)
        dwImageStreamerGL_release(m_image2GL);

    // -----------------------------------------
    // Release renderer
    // -----------------------------------------
    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        dwRenderEngine_destroyBuffer(m_lidarTiles[i].renderBufferId, m_renderEngine);
    }
    dwRenderEngine_destroyBuffer(m_stitchedTile.renderBufferId, m_renderEngine);
    dwRenderEngine_destroyBuffer(m_trajectoryTile.renderBufferId, m_renderEngine);

    // -----------------------------------------
    // Release egomotion
    // -----------------------------------------
    if (m_egomotion)
        dwEgomotion_release(m_egomotion);

    // -----------------------------------------
    // Release vehicle io
    // -----------------------------------------
    if (m_vehicleIO)
        dwVehicleIO_release(m_vehicleIO);
    // -----------------------------------------
    // Release point cloud processor memory
    // -----------------------------------------
    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        dwPointCloud_destroyBuffer(&m_accumulatedPoints[i]);
    }

    dwPointCloud_destroyBuffer(&m_stitchedDepthMap3D);
    dwPointCloud_destroyBuffer(&m_stitchedDepthMap3DPrev);
    dwPointCloud_destroyBuffer(&m_stitchedPoints);
    dwImage_destroy(m_stitchedDepthImage);
    dwImage_destroy(m_stitchedDepthImageHost);

    dwPointCloud_destroyBuffer(&m_stitchedPointsHost);

    dwPointCloud_destroyBuffer(&m_groundInliers);
    dwPointCloud_destroyBuffer(&m_groundOutliers);
    dwPointCloud_destroyBuffer(&m_transformedPoints);

    // -----------------------------------------
    // Release point cloud processor modules
    // -----------------------------------------
    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        if (m_accumulator[i])
            dwPointCloudAccumulator_release(m_accumulator[i]);
    }

    if (m_stitcher)
        dwPointCloudStitcher_release(m_stitcher);

    if (m_rangeImageCreator)
        dwPCRangeImageCreator_release(m_rangeImageCreator);

    if (m_coordinateConverter)
        dwPointCloudStitcher_release(m_coordinateConverter);

    if (m_planeExtractor)
        dwPCPlaneExtractor_release(m_planeExtractor);

    if (m_icp)
        dwPointCloudICP_release(m_icp);

    // -----------------------------------------
    // Release DriveWorks and SAL
    // -----------------------------------------
    dwSAL_release(m_sal);
    dwRenderEngine_release(m_renderEngine);
    dwVisualizationRelease(m_viz);
    dwRelease(m_context);
    dwLogger_release();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
bool PointCloudProcessingSample::getSpin()
{
    const dwLidarDecodedPacket* pckt;
    dwIMUFrame imuFrame;

    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        m_lidarOverflowCount[i] = 0;
        m_lidarAccumulated[i]   = false;
    }

    uint32_t numLidarsAccumulated = 0;
    while (numLidarsAccumulated < m_lidarCount)
    {
        const dwSensorEvent* acquiredEvent = nullptr;
        dwStatus status                    = dwSensorManager_acquireNextEvent(&acquiredEvent, 0, m_sensorManager);

        if (status != DW_SUCCESS)
        {
            if (status == DW_END_OF_STREAM)
            {
                std::cout << "End of stream reached" << std::endl;
                return false;
            }
            else
            {
                std::cerr << "Unable to acquire the next sensor manager event: "
                          << dwGetStatusName(status) << std::endl;
                stop();
            }
            return false;
        }

        switch (acquiredEvent->type)
        {
        case DW_SENSOR_CAN:
        {
            std::swap(m_prevVehicleSafeState, m_currVehicleSafeState);
            std::swap(m_prevVehicleNonSafeState, m_currVehicleNonSafeState);
            std::swap(m_prevVehicleActuationFeedbackState, m_currVehicleActuationFeedbackState);

            CHECK_DW_ERROR(dwVehicleIO_consumeCANFrame(&acquiredEvent->canFrame, 0, m_vehicleIO));

            CHECK_DW_ERROR(dwVehicleIO_getVehicleSafetyState(&m_currVehicleSafeState, m_vehicleIO));
            CHECK_DW_ERROR(dwVehicleIO_getVehicleNonSafetyState(&m_currVehicleNonSafeState, m_vehicleIO));
            CHECK_DW_ERROR(dwVehicleIO_getVehicleActuationFeedback(&m_currVehicleActuationFeedbackState, m_vehicleIO));

            if (m_prevVehicleNonSafeState.frontSteeringTimestamp < m_currVehicleNonSafeState.frontSteeringTimestamp)
            {
                CHECK_DW_ERROR(dwEgomotion_addVehicleIOState(&m_currVehicleSafeState, &m_currVehicleNonSafeState, &m_currVehicleActuationFeedbackState, m_egomotion));
            }
            break;
        }

        case DW_SENSOR_LIDAR:
            pckt = acquiredEvent->lidFrame;

            {
                const uint32_t& lidarIndex = acquiredEvent->sensorTypeIndex;
                if (m_lidarAccumulated[lidarIndex])
                {
                    m_lidarOverflowCount[lidarIndex]++;
                    if (m_lidarOverflowCount[lidarIndex] > m_lidarProps[lidarIndex].packetsPerSpin)
                        logWarn("Lidar %d finished two spins before all other lidars finished one spin\n", lidarIndex);
                }
                else
                {
                    CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_addLidarPacket(pckt, m_accumulator[lidarIndex]),
                                       "Internal error when adding lidar packet to accumulator");

                    bool ready = false;
                    CHECK_DW_ERROR(dwPointCloudAccumulator_isReady(&ready, m_accumulator[lidarIndex]));

                    if (ready)
                    {
                        if (lidarIndex == 0)
                            m_registrationTime = pckt->hostTimestamp;

                        m_lidarAccumulated[lidarIndex] = true;
                        CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_process(m_accumulator[lidarIndex]),
                                           "Could not process Accumulation");
                        numLidarsAccumulated++;
                    }
                }
            }
            break;

        case DW_SENSOR_IMU:
            imuFrame = acquiredEvent->imuFrame;
            CHECK_DW_ERROR_MSG(dwEgomotion_addIMUMeasurement(&imuFrame, m_egomotion),
                               "Could not add IMU measurement");
            break;

        case DW_SENSOR_GPS:
        case DW_SENSOR_RADAR:
        case DW_SENSOR_TIME:
        case DW_SENSOR_DATA:
        case DW_SENSOR_COUNT:
        case DW_SENSOR_CAMERA:
        default:
            // Ignore unknown sensor events
            break;
        } // end switch

        CHECK_DW_ERROR_MSG(dwSensorManager_releaseAcquiredEvent(acquiredEvent, m_sensorManager),
                           "Could not release acquired event");
    }

    CHECK_DW_ERROR_MSG(dwPointCloudStitcher_enableMotionCompensation(m_registrationTime, m_egomotion, m_stitcher),
                       "Cound not set motion compensation");

    CHECK_DW_ERROR_MSG(dwPointCloudStitcher_process(m_stitcher),
                       "Could not register accumulated Points");

    CHECK_DW_ERROR_MSG(dwPCRangeImageCreator_process(m_rangeImageCreator),
                       "Could not create range image")

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::copyToRenderBuffer(uint32_t renderBufferId, uint32_t offset,
                                                    const dwPointCloud& pointCloud)
{
    uint32_t sizeInBytes     = pointCloud.capacity * sizeof(dwVector4f);
    dwVector4f* dataToRender = nullptr;

    dwRenderEngine_mapBuffer(renderBufferId,
                             reinterpret_cast<void**>(&dataToRender),
                             offset,
                             sizeInBytes,
                             DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                             m_renderEngine);
    // copy the processing data to the internal gl buffer
    CHECK_CUDA_ERROR(cudaMemcpy(dataToRender, pointCloud.points,
                                sizeInBytes, cudaMemcpyDeviceToHost));

    dwRenderEngine_unmapBuffer(renderBufferId,
                               DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_3D,
                               m_renderEngine);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
bool PointCloudProcessingSample::runLoop()
{
    // accumulate & organize spin
    if (!getSpin())
        return false;

    // get ICP initialization
    m_icpInitialPose = DW_IDENTITY_TRANSFORMATION3F;
    CHECK_DW_ERROR(dwEgomotion_computeRelativeTransformation(&m_icpInitialPose, nullptr,
                                                             m_stitchedDepthMap3D.timestamp,
                                                             m_stitchedDepthMap3DPrev.timestamp, m_egomotion));
    // run ICP and get transformation aligning src on target
    if (dwPointCloudICP_process(m_icp) != DW_SUCCESS)
    {
        m_currentRigToWorld *= m_icpInitialPose;
    }
    else
    {
        dwTransformation3f deltaPose = DW_IDENTITY_TRANSFORMATION3F;
        Mat4_AxBinv(deltaPose.array, m_icpInitialPose.array, m_icpRefinedPose.array);
        float32_t deltaR = getRotationMagnitude(deltaPose);
        float32_t deltaT = getTranslationMagnitude(deltaPose);

        // use egomotion if delta rotation angle is larger than 2 deg and delta translation is larger than 5cm
        m_currentRigToWorld *= ((RAD2DEG(deltaR) > 2.0f) || deltaT > 0.05f) ? m_icpInitialPose : m_icpRefinedPose;
    }

    Mat4_RenormR(m_currentRigToWorld.array);
    std::swap(m_stitchedDepthMap3D, m_stitchedDepthMap3DPrev);

    CHECK_DW_ERROR(dwPointCloudStitcher_bindInput(DW_BIND_SLOT_1, &m_stitchedPoints, &m_currentRigToWorld, m_coordinateConverter));
    CHECK_DW_ERROR_MSG(dwPointCloudStitcher_process(m_coordinateConverter), "Could not transform from lidar to world coordinate");

    CHECK_CUDA_ERROR(cudaMemcpyAsync(m_stitchedPointsHost.points, m_stitchedPoints.points,
                                     sizeof(dwVector4f) * m_stitchedPoints.capacity,
                                     cudaMemcpyDeviceToHost, m_stream));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(m_stream));

    m_stitchedPointsHost.size = m_stitchedPoints.size;

    dwPCPlaneExtractor_process(m_planeExtractor);
    return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::onProcess()
{
    if (m_spinNum++ >= m_numFrames || !runLoop())
    {
        stop(); // clean return when screen is not controlled interactively.
        return;
    }

    std::cout << "----------Spin: " << m_spinNum << "----------\n";
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::renderMasterView()
{
    getMouseView().setCenter(0.f, 0.f, 0.f);
    for (uint32_t i = 0; i < m_lidarCount; i++)
    {
        renderPointCloud(m_lidarTiles[i].renderBufferId,
                         m_lidarTiles[i].tileId,
                         0,
                         DW_RENDER_ENGINE_COLOR_ORANGE,
                         m_accumulatedPoints[i]);

        dwRenderEngine_renderBuffer(m_lidarTiles[i].renderBufferId,
                                    m_accumulatedPoints[i].size,
                                    m_renderEngine);
    }

    // stitched point cloud
    renderPointCloud(m_stitchedTile.renderBufferId,
                     m_stitchedTile.tileId,
                     0,
                     DW_RENDER_ENGINE_COLOR_GREEN,
                     m_stitchedPoints);

    std::string msg = "fused point clouds, icp iteration: " + std::to_string(m_maxIters);
    renderTexts(msg.c_str(), {0.2f, 0.8f});

    dwRenderEngine_renderBuffer(m_stitchedTile.renderBufferId,
                                m_stitchedPoints.size,
                                m_renderEngine);

    dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);

    // estimated ground plane
    dwRenderEngine_renderPlanarGrid3D({1.5f, 0.0f, 2.5f, 1.5f}, 1.0f, 1.0f,
                                      reinterpret_cast<dwMatrix4f*>(&m_groundPlane.transformation),
                                      m_renderEngine);

    // plane normal vector
    renderPlaneNormalVector();

    // range image
    dwImageCPU* input     = nullptr;
    dwImageCUDA* srcImage = nullptr;
    CHECK_DW_ERROR(dwImage_getCUDA(&srcImage, m_stitchedDepthImage));
    CHECK_DW_ERROR(dwImage_getCPU(&input, m_stitchedDepthImageHost));

    CHECK_CUDA_ERROR(cudaMemcpy(input->data[0], srcImage->dptr[0],
                                srcImage->prop.width * srcImage->prop.height * sizeof(float32_t),
                                cudaMemcpyDeviceToHost));

    makeRGBAImage(m_imageRGBA, input);
    dwRenderEngine_setTile(m_imageTile.tileId, m_renderEngine);
    renderRangeImage(m_imageHandleRGBA);
    renderTexts("range image from fused point clouds", {0.2f, 1.0f});
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::renderCoordinate(const dwTransformation3f& rigToWorld)
{
    dwRenderEngine_setLineWidth(2.0f, m_renderEngine);
    dwVector3f arrow[2];
    for (uint32_t i = 0; i < 3; i++)
    {
        float32_t localAxis[3] = {
            i == 0 ? 1.f : 0.f,
            i == 1 ? 1.f : 0.f,
            i == 2 ? 1.f : 0.f};

        // origin
        arrow[0].x = rigToWorld.array[12];
        arrow[0].y = rigToWorld.array[13];
        arrow[0].z = rigToWorld.array[14];

        // axis
        Mat4_Axp(reinterpret_cast<float32_t*>(&arrow[1]), rigToWorld.array, localAxis);
        dwRenderEngine_setColor({localAxis[0], localAxis[1], localAxis[2], 1.0f}, m_renderEngine);
        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ARROWS_3D,
                              arrow,
                              sizeof(dwVector3f) * 2,
                              0,
                              1,
                              m_renderEngine);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::renderTexts(const char* msg, const dwVector2f& location)
{
    // store previous tile
    uint32_t previousTile = 0;
    CHECK_DW_ERROR(dwRenderEngine_getTile(&previousTile, m_renderEngine));

    // select default tile
    CHECK_DW_ERROR(dwRenderEngine_setTile(0, m_renderEngine));

    // get default tile state
    dwRenderEngineTileState previousDefaultState{};
    CHECK_DW_ERROR(dwRenderEngine_getState(&previousDefaultState, m_renderEngine));

    // set text render settings
    CHECK_DW_ERROR(dwRenderEngine_setModelView(&DW_IDENTITY_MATRIX4F, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D({1.0f, 1.0f}, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setColor({0.0f, 0.6f, 1.0f, 1.0f}, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_20, m_renderEngine));

    // render
    CHECK_DW_ERROR(dwRenderEngine_renderText2D(msg, location, m_renderEngine));

    // restore previous settings
    CHECK_DW_ERROR(dwRenderEngine_setState(&previousDefaultState, m_renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setTile(previousTile, m_renderEngine));
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::renderTrajectoryView()
{
    uint32_t index          = m_spinNum % MAX_SPINS_TO_VISUALIZE;
    uint32_t offset         = index * sizeof(dwVector4f) * m_transformedPoints.capacity;
    uint32_t tileId         = m_trajectoryTile.tileId;
    uint32_t renderBufferId = m_trajectoryTile.renderBufferId;

    m_rigToWorldHistory.push_back(m_currentRigToWorld);
    if (m_rigToWorldHistory.size() > 3 * MAX_SPINS_TO_VISUALIZE)
        m_rigToWorldHistory.erase(m_rigToWorldHistory.begin());

    // model view and projection
    getMouseView().setCenter(m_rigToWorldHistory.back().array[12],
                             m_rigToWorldHistory.back().array[13],
                             m_rigToWorldHistory.back().array[14]);

    renderPointCloud(renderBufferId, tileId, offset, DW_RENDER_ENGINE_COLOR_LIGHTGREY, m_transformedPoints);

    // render the buffer
    dwRenderEngine_renderBuffer(renderBufferId, m_transformedPoints.size * MAX_SPINS_TO_VISUALIZE, m_renderEngine);

    // render the coordinate
    renderCoordinate(m_rigToWorldHistory.back());

    // render the 3D trajectory
    dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, m_renderEngine);
    dwRenderEngine_setPointSize(2.f, m_renderEngine);
    dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_3D,
                          m_rigToWorldHistory.data(),
                          sizeof(dwTransformation3f),
                          offsetof(dwTransformation3f, array) + 3 * 4 * sizeof(float32_t),
                          static_cast<uint32_t>(m_rigToWorldHistory.size()),
                          m_renderEngine);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::renderPlaneNormalVector()
{
    dwVector3f arrow[2];
    float32_t localAxis[3] = {0.f, 0.f, 1.f};

    // origin
    arrow[0].x = m_groundPlane.transformation.array[12];
    arrow[0].y = m_groundPlane.transformation.array[13];
    arrow[0].z = m_groundPlane.transformation.array[14];

    // axis
    Mat4_Axp(reinterpret_cast<float32_t*>(&arrow[1]), m_groundPlane.transformation.array, localAxis);
    dwRenderEngine_setLineWidth(3.f, m_renderEngine);
    dwRenderEngine_setColor({localAxis[0], localAxis[1], localAxis[2], 1.0f}, m_renderEngine);
    dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_ARROWS_3D,
                          arrow,
                          sizeof(dwVector3f) * 2,
                          0,
                          1,
                          m_renderEngine);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::onRender()
{
    // clear buffer
    dwRenderEngine_reset(m_renderEngine);

    const char* msg = m_renderMasterView
                          ? R"(Press right arrow key -> to view trajectory)"
                          : R"(Press left arrow key <- to view the point clouds and range image)";

    (m_renderMasterView) ? renderMasterView() : renderTrajectoryView();

    // fps and text
    renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    renderTexts(msg, {0.5f, 1.0f});
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::renderRangeImage(dwImageHandle_t image)
{
    if (!m_image2GL)
        return;
    dwImageHandle_t imageGLHandle = DW_NULL_HANDLE;

    const dwTime_t TIME_OUT = 30000;
    CHECK_DW_ERROR(dwImageStreamerGL_producerSend(image, m_image2GL));
    CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&imageGLHandle, TIME_OUT, m_image2GL));

    if (imageGLHandle)
    {
        dwImageGL* imageGL = nullptr;
        CHECK_DW_ERROR(dwImage_getGL(&imageGL, imageGLHandle));

        dwVector2f range{};
        range.x = imageGL->prop.width;
        range.y = imageGL->prop.height;
        dwRenderEngine_setCoordinateRange2D(range, m_renderEngine);
        dwRenderEngine_renderImage2D(imageGL, {0.0f, 0.0f, range.x, range.y}, m_renderEngine);

        CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&imageGLHandle, m_image2GL));
        CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, TIME_OUT, m_image2GL));
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::renderPointCloud(uint32_t renderBufferId,
                                                  uint32_t tileId,
                                                  uint32_t offset,
                                                  dwRenderEngineColorRGBA color,
                                                  const dwPointCloud& pointCloud)
{
    // tile
    dwRenderEngine_setTile(tileId, m_renderEngine);

    // model view and projection
    dwMatrix4f modelView;
    Mat4_AxB(modelView.array, getMouseView().getModelView()->array, DW_IDENTITY_TRANSFORMATION3F.array);
    dwRenderEngine_setModelView(&modelView, m_renderEngine);
    dwRenderEngine_setProjection(getMouseView().getProjection(), m_renderEngine);

    // color and size
    dwRenderEngine_setBackgroundColor({0.1f, 0.1f, 0.1f, 1.0f}, m_renderEngine);
    dwRenderEngine_setColor(color, m_renderEngine);
    dwRenderEngine_setPointSize(1.f, m_renderEngine);

    // transfer to gl buffer
    copyToRenderBuffer(renderBufferId, offset, pointCloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::makeRGBAImage(dwImageCPU* imageRGBA, const dwImageCPU* rangeImage)
{
    if (!imageRGBA)
        return;

    float32_t* data = reinterpret_cast<float32_t*>(rangeImage->data[0]);

    auto minmaxValue   = std::minmax_element(data, data + rangeImage->prop.height * rangeImage->prop.width);
    float32_t minValue = data[minmaxValue.first - data];
    float32_t maxValue = data[minmaxValue.second - data];

    for (uint32_t i = 0; i < rangeImage->prop.width * rangeImage->prop.height; i++)
    {
        uint8_t scaledValue     = 0;
        float32_t originalValue = data[i];
        float32_t deltaValue    = maxValue - minValue;
        // use (max value - current value) instead of (current value - min value) to invert the display
        if (std::abs(deltaValue) > 1e-6f)
            scaledValue = static_cast<uint8_t>(255.f * (maxValue - originalValue) / deltaValue);

        imageRGBA->data[0][4 * i]     = scaledValue;
        imageRGBA->data[0][4 * i + 1] = scaledValue;
        imageRGBA->data[0][4 * i + 2] = scaledValue;
        imageRGBA->data[0][4 * i + 3] = 255;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::onResizeWindow(int width, int height)
{
    dwRenderEngine_reset(m_renderEngine);
    dwRectf rect;
    rect.width  = width;
    rect.height = height;
    rect.x      = 0;
    rect.y      = 0;
    dwRenderEngine_setBounds(rect, m_renderEngine);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudProcessingSample::onKeyDown(int32_t key, int32_t scancode, int32_t mods)
{
    (void)scancode;
    (void)mods;

    if (key == GLFW_KEY_RIGHT)
    {
        m_renderMasterView = false;
    }

    if (key == GLFW_KEY_LEFT)
    {
        m_renderMasterView = true;
    }
}
