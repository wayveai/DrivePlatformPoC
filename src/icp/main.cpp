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

// Driveworks sample includes
#include <dw/core/base/Version.h>
#include <framework/DriveWorksSample.hpp>
#include <framework/Mat4.hpp>
#include <framework/MathUtils.hpp>
#include <framework/WindowGLFW.hpp>

// Renderer
#include <dwvisualization/core/RenderEngine.h>
#include <dwvisualization/core/Renderer.h>

// Point cloud processing
#include <dw/pointcloudprocessing/icp/PointCloudICP.h>
#include <dw/pointcloudprocessing/accumulator/PointCloudAccumulator.h>

#include <array>
#include <algorithm>

#include "PlyWriters.hpp"

using namespace dw_samples::common;

using std::cout;
using std::endl;

//------------------------------------------------------------------------------
// Lidar ICP samples
// The sample demonstrates how to use ICP module to align Lidar point clouds
//
//------------------------------------------------------------------------------
class LidarICP : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Types
    // ------------------------------------------------
    // A point used in DW Lidar module.
    typedef dwLidarPointXYZI dwPoint;

    // ------------------------------------------------
    // settings
    // ------------------------------------------------

    // Input file required for module to run
    std::string lidarFile;
    // Location of output PLY file
    std::string ICPPlyFile;
    // How many spins to skip at the beginning
    uint32_t initSpin;
    // How many spins to skip for each ICP step
    uint32_t numSkip;
    // Maximum number of ICP iterations to do
    uint32_t maxIters;
    // Maximum number of Lidar spins to process
    uint32_t numFrames;
    // Counter for number of lidar spins to process
    uint32_t spinNum = 0;

    // ------------------------------------------------
    // Global Constants:
    // ------------------------------------------------
    // Number of points to dump to PLY file, points are random-sampled.
    const size_t VizPtsSize = 8000;
    // The resolution of a depthmap arrangement in Pts/Degree, see sample function's comments.
    const size_t DepthMapHorzRes = 2;
    // Number of Top and Bottom beams of lidar to ignore in sampling of points.
    const size_t DepthMapBeamStart = 10, DepthMapBeamEnd = 45;
    // Maximal number of points rendered
    const size_t MaxNumPointsToRender = 500000;

    // render grid size
    const float32_t WORLD_GRID_SIZE_IN_METERS = 200.0f;
    const float32_t WORLD_GRID_RES_IN_METERS  = 5.0f;

    dwVector2ui dmSize                = {0, 0}; // size of depthmap if we are using it.
    dwLidarProperties lidarProperties = {};

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwContextHandle_t context                   = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t viz          = DW_NULL_HANDLE;
    dwRenderEngineHandle_t renderEngine         = DW_NULL_HANDLE;
    dwSALHandle_t sal                           = DW_NULL_HANDLE;
    dwRendererHandle_t renderer                 = DW_NULL_HANDLE;
    dwPointCloudICPHandle_t icpHandle           = DW_NULL_HANDLE;
    dwSensorHandle_t lidarSensor                = DW_NULL_HANDLE;
    dwRenderBufferHandle_t groundPlaneRB        = DW_NULL_HANDLE;
    dwRenderBufferHandle_t pointCloudRB         = DW_NULL_HANDLE;
    dwRenderBufferHandle_t refPointCloudRB      = DW_NULL_HANDLE;
    dwPointCloudAccumulatorHandle_t accumulator = DW_NULL_HANDLE;

    // This is the ICP Transform that starts at I, All transforms are accumulated to here.
    dwTransformation3f icpRigToWorld = DW_IDENTITY_TRANSFORMATION3F;

    // ICP prior pose, which is set to last optimized pose (assuming nearly constant velocity)
    dwTransformation3f icpPriorPose = DW_IDENTITY_TRANSFORMATION3F;

    dwVector3f centerViewDiff{0, 0, 5};

    // Full spin point clouds, source and target
    dwPointCloud fullSpinPointClouds[2];

    // Partial point clouds pointing to user specified range
    dwPointCloud rangePointClouds[2];

    // Index of a source point cloud, 0 or 1
    uint32_t sourcePointCloudIdx = 0;

    // Calculated points accumulated over all the run
    std::vector<dwPoint> accumulatedPointCloud;

    // Reference points accumulated over all the run
    std::vector<dwPoint> accumulatedPointCloudRef;

    bool renderRefPC = true;

public:
    // initialize sample
    LidarICP(const ProgramArguments& args)
        : DriveWorksSample(args)
    {
        lidarFile = getArgument("lidarFile");

        initSpin  = uint32_t(atoi(getArgument("init").c_str()));
        numSkip   = uint32_t(atoi(getArgument("skip").c_str()));
        numFrames = uint32_t(atoi(getArgument("numFrames").c_str()));
        maxIters  = uint32_t(atoi(getArgument("maxIters").c_str()));

        // This is necessary because first frames do not contain enough points
        if (initSpin < 10)
            std::cerr << "`--init` too small, set to " << (initSpin = 10) << endl;

        if (numSkip > 5)
            std::cerr << "`--skip` too large, set to " << (numSkip = 5) << endl;

        if (maxIters > 50)
            std::cerr << "`--maxIters` too large, set to " << (maxIters = 50) << endl;
        if (numFrames == 0)
            numFrames = (uint32_t)-1;

        ICPPlyFile = getArgument("plyloc");

        if (!ICPPlyFile.empty())
        {
            ICPPlyFile += std::to_string(initSpin) + "-" +
                          std::to_string(numSkip + 1) + "-" +
                          std::to_string(maxIters) + "-icp.ply";
        }

        spinNum = 0;
    }

    /// -----------------------------
    /// Initialize Logger and DriveWorks context
    /// -----------------------------
    void initializeDriveWorks(dwContextHandle_t& context) const
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

    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            initializeDriveWorks(context);

            CHECK_DW_ERROR_MSG(dwSAL_initialize(&sal, context),
                               "Cannot initialize SAL");
        }

        // ----------------------------------------------------
        // Create lidar sensor and check if its the right kind.
        // ----------------------------------------------------
        {
            dwSensorParams sensorParams{};
            std::string sensorParamStr = "file=" + lidarFile;
            sensorParams.parameters    = sensorParamStr.c_str();
            sensorParams.protocol      = "lidar.virtual";

            CHECK_DW_ERROR_MSG(dwSAL_createSensor(&lidarSensor, sensorParams, sal), "Cannot create lidar sensor");

            dwSensorLidar_getProperties(&lidarProperties, lidarSensor);

            // Currently sample only works with Velodyne like Lidars
            if (std::string("VELO_HDL64E") != lidarProperties.deviceString)
            {
                logError("Sample running with Lidar device named %s"
                         "\n This sample designed for velodyne 64 data only.\n",
                         lidarProperties.deviceString);
                return false;
            }

            CHECK_DW_ERROR_MSG(dwSensor_start(lidarSensor), "Cannot start lidar");
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&viz, context));

            // init render engine with default params
            dwRenderEngineParams params{};
            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            CHECK_DW_ERROR(dwRenderEngine_initialize(&renderEngine, &params, viz));

            CHECK_DW_ERROR_MSG(dwRenderer_initialize(&renderer, viz),
                               "Cannot initialize Renderer, maybe no GL context available?");
            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, renderer);

            constructGrid();
            preprarePointCloudRenderBuffers();
        }

        // -------------------------------------
        // Initialize Point Cloud Accumulator
        // -------------------------------------
        {
            dwPointCloudAccumulatorParams params{};
            CHECK_DW_ERROR(dwPointCloudAccumulator_getDefaultParams(&params));

            params.filterWindowSize = 8;
            params.organized        = true;

            CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_initialize(&accumulator, &params, &lidarProperties, context), "Point Cloud Accumulator Init failed");
            CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_getSweepSize(&dmSize, accumulator), "Cannot retrieve sweep size from point cloud accumulator");

            dmSize.y = DepthMapBeamEnd - DepthMapBeamStart;

            // Initialize point cloud structures
            for (uint32_t idx = 0; idx < 2; ++idx)
            {
                fullSpinPointClouds[idx].type     = DW_MEMORY_TYPE_CPU;
                fullSpinPointClouds[idx].format   = DW_POINTCLOUD_FORMAT_XYZI;
                fullSpinPointClouds[idx].capacity = lidarProperties.pointsPerSpin;

                CHECK_DW_ERROR_MSG(dwPointCloud_createBuffer(&fullSpinPointClouds[idx]), "Failed to create buffer for full spin point cloud");

                rangePointClouds[idx].type      = DW_MEMORY_TYPE_CPU;
                rangePointClouds[idx].format    = DW_POINTCLOUD_FORMAT_XYZI;
                rangePointClouds[idx].capacity  = dmSize.x * dmSize.y;
                rangePointClouds[idx].size      = dmSize.x * dmSize.y;
                rangePointClouds[idx].points    = static_cast<dwVector4f*>(fullSpinPointClouds[idx].points) + DepthMapBeamStart * dmSize.x;
                rangePointClouds[idx].organized = true;
            }
        }

        // --------------------------------------------------
        // Skip initial spins as per the command line args,
        // and set the next immediate frame as target
        // --------------------------------------------------
        log("Skipping %d spins..\n", initSpin);
        {
            for (uint i = 0; i < initSpin; ++i)
            {
                getSpin(fullSpinPointClouds[1]);
            }

            log(" Done!\n");

            getSpin(fullSpinPointClouds[1]);
        }

        // -----------------------------
        // Initialize ICP module
        // -----------------------------
        {
            //We are using point cloud version for smaple,currently.
            dwPointCloudICPParams params{};
            CHECK_DW_ERROR(dwPointCloudICP_getDefaultParams(&params));
            params.maxPoints     = dmSize.x * dmSize.y;
            params.icpType       = DW_POINT_CLOUD_ICP_TYPE_DEPTH_MAP;
            params.depthmapSize  = {dmSize.x, dmSize.y};
            params.maxIterations = maxIters;

            CHECK_DW_ERROR_MSG(dwPointCloudICP_initialize(&icpHandle, &params, context), "ICP Init failed");
        }

        accumulatePointClouds(fullSpinPointClouds[1], fullSpinPointClouds[1], DW_IDENTITY_TRANSFORMATION3F);
        return true;
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        // -----------------------------------------
        // Randomly sample accumulated points into a PLY file
        // -----------------------------------------
        if (!ICPPlyFile.empty())
        {
            PlyWriter ply(ICPPlyFile.c_str());
            for (const auto& t : accumulatedPointCloud)
            {
                ply.PushOnePoint(t, t.intensity);
            }
        }

        // -----------------------------------------
        // Stop sensor
        // -----------------------------------------
        dwSensor_stop(lidarSensor);
        dwSAL_releaseSensor(lidarSensor);

        // -----------------------------------------
        // Release icp
        // -----------------------------------------
        dwPointCloudICP_release(icpHandle);

        dwPointCloud_destroyBuffer(&fullSpinPointClouds[0]);
        dwPointCloud_destroyBuffer(&fullSpinPointClouds[1]);

        // -----------------------------------------
        // Release lidar accumulator
        // -----------------------------------------
        dwPointCloudAccumulator_release(accumulator);

        // -----------------------------------------
        // Release renderer and streamer
        // -----------------------------------------
        {
            dwRenderBuffer_release(pointCloudRB);
            dwRenderBuffer_release(refPointCloudRB);
            dwRenderBuffer_release(groundPlaneRB);

            if (renderEngine != DW_NULL_HANDLE)
            {
                CHECK_DW_ERROR(dwRenderEngine_release(renderEngine));
            }
            dwRenderer_release(renderer);
        }

        // -----------------------------------------
        // Release DriveWorks and SAL
        // -----------------------------------------
        {
            dwSAL_release(sal);
            dwVisualizationRelease(viz);
            dwRelease(context);
            dwLogger_release();
        }
    }

    //------------------------------------------------------------------------------
    void onKeyDown(int32_t key, int32_t, int32_t) override
    {
        if (key == GLFW_KEY_F2)
        {
            renderRefPC = !renderRefPC;
        }
    }

    //------------------------------------------------------------------------------
    dwVector3f interpolateColor(float32_t value)
    {
        static std::vector<dwVector3f> HeatMap = {
            {0, 0, 1.0f},
            {0, 1.0f, 0},
            {1.0f, 1.0f, 0},
            {1.0f, 0, 0},
            {1.0f, 1.0f, 1.0f}};

        if (value <= 0)
            return HeatMap.front();
        if (value >= 1)
            return HeatMap.back();

        float32_t relative = value;
        int32_t numInts    = static_cast<int32_t>(HeatMap.size() - 1);
        int32_t index      = static_cast<int32_t>(relative * numInts); // multiply and round up
        relative -= static_cast<float32_t>(index) / numInts;
        relative *= numInts;
        return {(HeatMap[index].x * (1.f - relative) + HeatMap[index + 1].x * relative),
                (HeatMap[index].y * (1.f - relative) + HeatMap[index + 1].y * relative),
                (HeatMap[index].z * (1.f - relative) + HeatMap[index + 1].z * relative)};
    }

    //------------------------------------------------------------------------------
    void accumulatePointClouds(const dwPointCloud& srcPointCloud,
                               const dwPointCloud& tgtPointCloud,
                               const dwTransformation3f& prevRigToWorld)
    {
        const float32_t vizRatio = static_cast<float32_t>(srcPointCloud.size) / VizPtsSize;

        for (uint32_t pointIdx = 0; pointIdx < srcPointCloud.size; ++pointIdx)
        {
            if (rand() / RAND_MAX >= vizRatio)
                continue;

            const dwPoint& s = static_cast<const dwPoint*>(srcPointCloud.points)[pointIdx];

            dwPoint t{};
            Mat4_Axp(reinterpret_cast<float32_t*>(&t), icpRigToWorld.array, reinterpret_cast<const float32_t*>(&s));

            // We are coloring the points by height for clarity of the 3D view, so scaling height from 0m->4m
            t.intensity = (s.z + 2.5f) / 4.f;
            accumulatedPointCloud.push_back(t);

            // Add reference point to accumulated point cloud
            const dwPoint& sref = static_cast<const dwPoint*>(tgtPointCloud.points)[pointIdx];
            Mat4_Axp(reinterpret_cast<float32_t*>(&t), prevRigToWorld.array, reinterpret_cast<const float32_t*>(&sref));

            t.intensity = sref.intensity;
            accumulatedPointCloudRef.push_back(t);
        }

        // update render buffer to keep latest points
        {
            float32_t* map    = nullptr;
            uint32_t maxVerts = 0;
            uint32_t stride   = 0;

            size_t size     = std::min(accumulatedPointCloud.size(), MaxNumPointsToRender);
            size_t startIdx = accumulatedPointCloud.size() < MaxNumPointsToRender ? 0 : accumulatedPointCloud.size() - MaxNumPointsToRender;

            dwRenderBuffer_map(&map, &maxVerts, &stride, pointCloudRB);

            float32_t* buffer = map;
            for (size_t idx = startIdx; idx < size + startIdx; idx++)
            {
                const dwPoint& t = accumulatedPointCloud[idx];
                buffer[0]        = t.x;
                buffer[1]        = t.y;
                buffer[2]        = t.z;

                dwVector3f color = interpolateColor(t.intensity);
                buffer[3]        = color.x;
                buffer[4]        = color.y;
                buffer[5]        = color.z;

                buffer += stride;
            }

            dwRenderBuffer_unmap(maxVerts, pointCloudRB);

            // Reference point cloud buffer
            dwRenderBuffer_map(&map, &maxVerts, &stride, refPointCloudRB);

            buffer = map;
            for (size_t idx = startIdx; idx < size + startIdx; idx++)
            {
                const dwPoint& t = accumulatedPointCloudRef[idx];
                buffer[0]        = t.x;
                buffer[1]        = t.y;
                buffer[2]        = t.z;

                buffer += stride;
            }

            dwRenderBuffer_unmap(maxVerts, refPointCloudRB);
        }
    }

    //------------------------------------------------------------------------------
    // Fetch a spin from lidar.
    bool getSpin(dwPointCloud& pointCloud)
    {
        for (uint32_t i = 0; i < lidarProperties.packetsPerSpin; ++i)
        {
            const dwLidarDecodedPacket* pckt = nullptr;
            if (dwSensorLidar_readPacket(&pckt, 50000, lidarSensor) != DW_SUCCESS)
            {
                return false;
            }

            CHECK_DW_ERROR(dwPointCloudAccumulator_addLidarPacket(pckt, accumulator));
            dwSensorLidar_returnPacket(pckt, lidarSensor);
        }

        // Should be ready
        bool isReady = false;
        CHECK_DW_ERROR(dwPointCloudAccumulator_isReady(&isReady, accumulator));

        if (!isReady)
        {
            throw std::runtime_error("Accumulator collected all spin packets but still reports it is not ready");
        }

        CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_bindOutput(&pointCloud, accumulator), "Cannot bind output buffer to Point Cloud Accumulator");
        CHECK_DW_ERROR_MSG(dwPointCloudAccumulator_process(accumulator), "Cannot process accumulated data");

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///
    /// Fetches spin, runs ICP and return the time of last spin received
    /// The transform from ICP is fetched and accumulated into g_icpTx that is used to transform this spin.
    /// We swap sampled source and target as source becomes target for next pair.
    ///------------------------------------------------------------------------------

    void onProcess() override
    {
        if (isPaused())
            return;

        if (spinNum++ >= numFrames)
        {
            stop(); // clean return when screen is not controlled interactively.
            return;
        }

        // skip some spins, to get larger temporal distance between consecutive spins
        for (uint32_t k = 0; k < numSkip; ++k)
        {
            if (!getSpin(fullSpinPointClouds[sourcePointCloudIdx]))
            {
                stop();
                return;
            }
        }

        // get next source points
        if (!getSpin(fullSpinPointClouds[sourcePointCloudIdx]))
        {
            stop();
            return;
        }

        uint32_t targetPointCloudIdx = (sourcePointCloudIdx + 1) % 2;

        // run ICP and get transformation aligning src on target
        dwTransformation3f icpCurrentRigToPrevRig{};
        dwStatus status = DW_SUCCESS;
        clock_t start   = clock();
        {
            const dwPointCloud* srcPointCloud = &rangePointClouds[sourcePointCloudIdx];
            const dwPointCloud* tgtPointCloud = &rangePointClouds[targetPointCloudIdx];

            CHECK_DW_ERROR_MSG(dwPointCloudICP_bindInput(srcPointCloud, tgtPointCloud, &icpPriorPose, icpHandle), "Failed to bind ICP input");
            CHECK_DW_ERROR_MSG(dwPointCloudICP_bindOutput(&icpCurrentRigToPrevRig, icpHandle), "Failed to bind ICP output");

            status = dwPointCloudICP_process(icpHandle);
        }
        float64_t icpTime = 1000 * (float64_t(clock()) - start) / CLOCKS_PER_SEC;

        if (status != DW_SUCCESS)
        {
            cout << "[WARNING] ICP failed with error " << dwGetStatusName(status) << endl;
            return;
        }

        icpPriorPose = icpCurrentRigToPrevRig;

        dwTransformation3f prevRigToWorld = icpRigToWorld;

        // accumulate transformation (and fix Rotation matrix)
        icpRigToWorld *= icpCurrentRigToPrevRig;
        Mat4_RenormR(icpRigToWorld.array);

        // Get some stats about the ICP perforlmance
        dwPointCloudICPResultStats icpResultStats{};
        CHECK_DW_ERROR_MSG(dwPointCloudICP_getLastResultStats(&icpResultStats, icpHandle), "Couldn't get ICP result stats.");

        cout << "ICP Time: " << icpTime << "ms" << endl
             << "Number of Iterations: " << icpResultStats.actualNumIterations << endl
             << "Number of point correspondences: " << icpResultStats.numCorrespondences << endl
             << "RMS cost: " << icpResultStats.rmsCost << endl
             << "Inlier fraction: " << icpResultStats.inlierFraction << endl
             << "ICP Spin Transform: " << icpCurrentRigToPrevRig << endl
             << "Full Transform: " << icpRigToWorld << endl;

        accumulatePointClouds(fullSpinPointClouds[sourcePointCloudIdx], fullSpinPointClouds[targetPointCloudIdx], prevRigToWorld);

        // Swap source and target point clouds
        sourcePointCloudIdx = (sourcePointCloudIdx + 1) % 2;

        cout << "----------Spin: " << spinNum << "----------\n";
    }

    ///------------------------------------------------------------------------------
    /// Initialize World Grid
    ///------------------------------------------------------------------------------
    void constructGrid()
    {
        // World grid
        int32_t gridResolution =
            static_cast<int32_t>(WORLD_GRID_SIZE_IN_METERS / WORLD_GRID_RES_IN_METERS);

        // Rendering data
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        dwRenderBuffer_initialize(&groundPlaneRB, layout, DW_RENDER_PRIM_LINELIST,
                                  2 * (gridResolution + 1), viz);

        // update the data
        float32_t* map;
        uint32_t maxVerts, stride;

        dwRenderBuffer_map(&map, &maxVerts, &stride, groundPlaneRB);

        int32_t nVertices = 0;
        float32_t x, y;

        // Horizontal lines
        x = -0.5f * WORLD_GRID_SIZE_IN_METERS;
        for (int32_t i = 0; i <= gridResolution; ++i)
        {
            y = -0.5f * WORLD_GRID_SIZE_IN_METERS;

            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;
            nVertices++;

            y                           = 0.5f * WORLD_GRID_SIZE_IN_METERS;
            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;

            nVertices++;
            x = x + WORLD_GRID_RES_IN_METERS;
        }

        // Vertical lines
        y = -0.5f * WORLD_GRID_SIZE_IN_METERS;
        for (int32_t i = 0; i <= gridResolution; ++i)
        {
            x = -0.5f * WORLD_GRID_SIZE_IN_METERS;

            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;
            nVertices++;

            x                           = 0.5f * WORLD_GRID_SIZE_IN_METERS;
            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;

            nVertices++;
            y = y + WORLD_GRID_RES_IN_METERS;
        }

        dwRenderBuffer_unmap(maxVerts, groundPlaneRB);
    }

    ///------------------------------------------------------------------------------
    /// Init point cloud rendering
    ///------------------------------------------------------------------------------
    void preprarePointCloudRenderBuffers()
    {
        // RenderBuffer
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
        layout.colFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        // Initialize Full Spin Point cloud
        dwRenderBuffer_initialize(&pointCloudRB, layout, DW_RENDER_PRIM_POINTLIST, MaxNumPointsToRender, viz);

        // Reference point cloud
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        dwRenderBuffer_initialize(&refPointCloudRB, layout, DW_RENDER_PRIM_POINTLIST, MaxNumPointsToRender, viz);
    }

    ///------------------------------------------------------------------------------
    /// Render Lidar Point Cloud
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        glDepthFunc(GL_LESS);

        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // compute modelview by moving camera to the center of the current estimated position
        {
            float32_t center[3] = {icpRigToWorld.array[0 + 3 * 4], icpRigToWorld.array[1 + 3 * 4], icpRigToWorld.array[2 + 3 * 4]};

            if (!isPaused())
            {
                getMouseView().setCenter(center[0] + centerViewDiff.x, center[1] + centerViewDiff.y, center[2] + centerViewDiff.z);
            }
            else
            {
                // store current difference to center, to keep it when not paused
                centerViewDiff.x = getMouseView().getCenter()[0] - center[0];
                centerViewDiff.y = getMouseView().getCenter()[1] - center[1];
                centerViewDiff.z = getMouseView().getCenter()[2] - center[2];
            }
        }

        // 3D rendering
        dwRenderer_setModelView(getMouseView().getModelView(), renderer);
        dwRenderer_setProjection(getMouseView().getProjection(), renderer);

        dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREY, renderer);
        dwRenderer_renderBuffer(groundPlaneRB, renderer);

        if (renderRefPC)
        {
            dwRenderer_setColor(DW_RENDERER_COLOR_YELLOW, renderer);
            dwRenderer_renderBuffer(refPointCloudRB, renderer);
        }

        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
        dwRenderer_renderBuffer(pointCloudRB, renderer);

        renderutils::renderFPS(renderEngine, getCurrentFPS());
    }
};

//------------------------------------------------------------------------------
int32_t main(int32_t argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    const std::string defLidarFile = (dw_samples::SamplesDataPath::get() + "/samples/sensors/lidar/lidar_top_hdl64e.bin");
    typedef ProgramArguments::Option_t opt;

    ProgramArguments args(argc, argv,
                          {opt("lidarFile", defLidarFile.c_str(), "Path to lidar file, needs to be DW captured Velodyne HDL-64E file."),
                           opt("plyloc", "", "If specified use this directory to write ICP-fused ASCII-PLY file."),
                           opt("init", "20", "These many initial spins are skipped before first pair is fed to ICP."),
                           opt("skip", "0", "Number of frames to skip to perform ICP between a pair."),
                           opt("numFrames", "0", "These many pairs are used to perform ICP before stopping. 0 - all frames"),
                           opt("maxIters", "12", "Number of ICP iterataions to run.")});

    // -------------------
    // initialize and start a window application
    LidarICP app(args);

    app.initializeWindow("Lidar ICP Sample", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
