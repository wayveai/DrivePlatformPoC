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
// SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// Framework
#include <framework/DriveWorksSample.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Checks.hpp>

// Context, SAL
#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>

// IMAGE
#include <dw/interop/streamer/ImageStreamer.h>

// CAMERA
#include <dw/sensors/camera/Camera.h>

// RIG
#include <dw/rig/Rig.h>

// Codecs
#include <dw/sensors/camera/CodecHeaderVideo.h>
#include <dw/sensors/Codec.h>
#include <dw/sensors/codecs/sensorserializer/SensorSerializer.h>

using namespace dw_samples::common;

#define MAX_PORTS_COUNT 4 //the number of device blocks, each with multiple cameras
#define MAX_CAMS_PER_PORT 4
#define MAX_CAMS MAX_PORTS_COUNT* MAX_CAMS_PER_PORT
#define FIRST_CAMERA_IDX 0
#define MAX_PIPELINE_COUNT 4 // ICP/ISP0/ISP1/ISP2
#define ICP_PIPELINE_IDX 0
#define ISP_PIPELINE_IDX 1
#define ISP1_PIPELINE_IDX 2
#define ISP2_PIPELINE_IDX 3
#define MAX_CAMERA_FIFO_SIZE 30

static inline std::string getNvSciErrorStr(NvSciError sciError)
{
    std::stringstream ss{};
    ss << static_cast<uint32_t>(sciError);
    std::string s;
    ss >> s;
    return s;
}

#define CHECK_NVSCI_ERROR(x)                                                                                                                                                                                    \
    {                                                                                                                                                                                                           \
        NvSciError RESULT = x;                                                                                                                                                                                  \
        if (RESULT != NvSciError_Success)                                                                                                                                                                       \
        {                                                                                                                                                                                                       \
            throw std::runtime_error(std::string("NvSci Error ") + getNvSciErrorStr(RESULT) + std::string(" executing NvSci function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
        }                                                                                                                                                                                                       \
    };

/**
 * @brief : Helper Container for NvSci Synchronization Object for Camera Module.
 *          - Abstraction layer that hides details of synchronization primitives to initialize and de-initialize a Synchronization Object.
 *          - A Synchronization Object is a container holding the reconciled
 *            NvSciSyncAttrList defining constraints of the Fence and the handle of the actual Primitive.
 *          - NvSciSync clients must supply the properties and constraints of an NvSciSync object to NvSciSync before allocating the object.
 *            This is expressed with attributes. An attribute is a key - value pair.
 *          - Refer to documentation of NvSciSync for additional details on synchronization.
 */
struct SampleCameraNvSciSyncResources
{
    // A reference to Synchronization Object to be allocated by applications
    NvSciSyncObj syncObj;

    // A synchronization primitive snapshot that must be reached to release the waiters.
    NvSciSyncFence fence;

    SampleCameraNvSciSyncResources()
        : fence(NvSciSyncFenceInitializer) {}
};

class CameraCustomSimpleApp : public DriveWorksSample
{
private:
    // Driveworks Context and SAL
    dwContextHandle_t m_context           = DW_NULL_HANDLE;
    dwVisualizationContextHandle_t m_viz  = DW_NULL_HANDLE;
    dwRendererHandle_t m_renderer         = DW_NULL_HANDLE;
    dwRenderEngineHandle_t m_renderEngine = DW_NULL_HANDLE;
    dwRenderEngineParams params{};
    dwRenderEngineColorRGBA m_colorPerPort[MAX_PORTS_COUNT];

    dwSALHandle_t m_sal             = DW_NULL_HANDLE;
    dwSensorHandle_t m_cameraMaster = DW_NULL_HANDLE;
    uint32_t m_totalCameras; //actual number of cameras used
    bool m_isProcessed1WriteEnabled;
    bool m_bUseNvSciPath; // Enable NvSci Path with External NvSciSync and NvSciBuf Objects

    dwRigHandle_t m_rigConfig{};

    // Per camera.
    // All active objects are at the front of array in order of the port.
    // So if there number of cameras per port is [4,2,4,1] then the
    // first 11 elements of these arrays will contain their information.
    // The total number of cameras is stored in m_totalCameras
    dwSensorHandle_t m_camera[MAX_CAMS];
    dwImageStreamerHandle_t m_streamerToGL[MAX_CAMS] = {DW_NULL_HANDLE};
    uint32_t m_tileVideo[MAX_CAMS];
    dwRectf m_renderRanges[MAX_CAMS] = {{0.0f, 0.0f, 0.0f, 0.0f}};

    bool m_recordCamera                                                                     = false;
    bool m_recordRaw                                                                        = false;
    dwSensorSerializerHandle_t m_serializer[dwCameraISPType::DW_CAMERA_MAX_ISP_COUNT]       = {DW_NULL_HANDLE};
    dwSensorSerializerNewHandle_t m_serializerNew[dwCameraISPType::DW_CAMERA_MAX_ISP_COUNT] = {DW_NULL_HANDLE};
    dwCodecHeaderHandle_t m_codecHeader                                                     = DW_NULL_HANDLE;

    bool m_useRaw        = false;
    bool m_useProcessed  = false;
    bool m_useProcessed1 = false;
    bool m_useProcessed2 = false;
    uint32_t m_fifoSize  = 0U;
    bool m_bUseNewCodecStack;

    enum RecordFileFormat
    {
        RECORD_FILE_FORMAT_H264,
        RECORD_FILE_FORMAT_H265,
        RECORD_FILE_FORMAT_MP4,
        RECORD_FILE_FORMAT_LRAW,
        RECORD_FILE_FORMAT_RAW,
        RECORD_FILE_FORMAT_POD,
        RECORD_FILE_FORMAT_UNSUPPORTED,
    };

    RecordFileFormat m_recordFormat = RECORD_FILE_FORMAT_UNSUPPORTED;

    std::unique_ptr<ScreenshotHelper> m_screenshot;

    // Frame grab variables
    dwRect m_roi;
    std::string m_pathName;
    dwImageStreamerHandle_t m_streamerToCPUGrab[MAX_CAMS] = {DW_NULL_HANDLE};
    int m_screenshotCount                                 = 0;
    bool m_frameGrab                                      = false;

    // press P, then group 0-3 then link 0-3 and that camera will be stopped
    bool m_pauseSensor = false;
    // press O, then group 0-3 then link 0-3 and that camera will be started
    bool m_startSensor = false;
    // press I, then group 0-3 then link 0-3 and that camera will be reset
    bool m_resetSensor = false;

    bool m_isCameraSelected   = false;
    uint32_t m_cameraSelected = 0;

    // List of imageHandle activated with setImagePool
    // std::vector<dwImageHandle_t> m_imagePoolList;
    dwImageHandle_t m_imagesForImagePool[MAX_CAMS][MAX_PIPELINE_COUNT][MAX_CAMERA_FIFO_SIZE] = {nullptr};

    /**
     * This module represents the library's instance created for that external applications and acts as a container for other NvSciSync resources.
     * All other NvSciSync resources are associated with an NvSciSyncModule on creation.
     */
    NvSciSyncModule m_nvsciSyncModule{};

    // Synchronization Containers for ICP Pipeline.
    SampleCameraNvSciSyncResources m_icpSyncResourcesObj[MAX_CAMS];

    // Synchronization Containers for ISP 0/1/2 Pipeline.
    SampleCameraNvSciSyncResources m_ispSyncResourcesObj[MAX_CAMS];

    /**
     * NvSciSync can be used to wait on a fence from the CPU.
     * Allocating those resources is controlled by the external application.
     */
    NvSciSyncCpuWaitContext m_cpuWaitContext{};

public:
    CameraCustomSimpleApp(const ProgramArguments& args);

    void initializeDriveWorks(dwContextHandle_t& context) const;

    // Sample framework
    void onProcess() override final {}
    void onRelease() override final
    {
        for (uint32_t cameraIdx = 0; cameraIdx < m_totalCameras; cameraIdx++)
        {
            for (uint32_t pipelineIdx = 0; pipelineIdx < MAX_PIPELINE_COUNT; pipelineIdx++)
            {
                for (uint32_t imageIdx = 0; imageIdx < m_fifoSize; imageIdx++)
                {
                    if (m_imagesForImagePool[cameraIdx][pipelineIdx][imageIdx] != nullptr)
                    {
                        dwImage_destroy(m_imagesForImagePool[cameraIdx][pipelineIdx][imageIdx]);
                    }
                }
            }
        }

        CHECK_DW_ERROR(dwSensor_stop(m_camera[FIRST_CAMERA_IDX]));
        for (uint32_t idx = 0; idx < dwCameraISPType::DW_CAMERA_MAX_ISP_COUNT; idx++)
        {
            if (m_serializer[idx] != nullptr)
            {
                dwSensorSerializer_stop(m_serializer[idx]);
                dwSensorSerializer_release(m_serializer[idx]);
            }

            if (m_serializerNew[idx] != nullptr)
            {
                CHECK_DW_ERROR(dwSensorSerializerNew_stop(m_serializerNew[idx]));
                CHECK_DW_ERROR(dwSensorSerializerNew_release(m_serializerNew[idx]));
                CHECK_DW_ERROR(dwCodecHeader_destroy(m_codecHeader));
            }
        }

        for (uint32_t i = 0; i < m_totalCameras; ++i)
        {
            if (m_streamerToCPUGrab[i])
            {
                dwImageStreamer_release(m_streamerToCPUGrab[i]);
            }
            if (m_streamerToGL[i])
            {
                dwImageStreamerGL_release(m_streamerToGL[i]);
            }
            if (m_camera[i])
            {
                dwSAL_releaseSensor(m_camera[i]);
            }
        }

        m_screenshot.reset();
        if (m_rigConfig)
            dwRig_release(m_rigConfig);
        if (m_cameraMaster)
            dwSAL_releaseSensor(m_cameraMaster);
        if (m_sal)
            dwSAL_release(m_sal);
        if (m_renderEngine)
            dwRenderEngine_release(m_renderEngine);
        if (m_renderer)
            dwRenderer_release(m_renderer);
        if (m_viz)
            dwVisualizationRelease(m_viz);
        if (m_context)
            dwRelease(m_context);
    }

    void onKeyDown(int key, int scancode, int mods) override
    {
        (void)scancode;
        (void)mods;

        if (key == GLFW_KEY_S)
        {
            m_screenshot->triggerScreenshot();
        }

        if (key == GLFW_KEY_F)
        {
            m_frameGrab = true;
        }

        if (key == GLFW_KEY_P)
        {
            m_startSensor = false;
            m_resetSensor = false;
            if (m_pauseSensor)
            {
                m_isCameraSelected = false;
            }
            m_pauseSensor = !m_pauseSensor;
            std::cout << "Pause sensor toggled " << m_pauseSensor << std::endl;
        }

        if (key == GLFW_KEY_O)
        {
            m_resetSensor = false;
            m_pauseSensor = false;

            if (m_startSensor)
            {
                m_isCameraSelected = false;
            }
            m_startSensor = !m_startSensor;
            std::cout << "Start sensor toggled " << m_startSensor << std::endl;
        }

        if (key == GLFW_KEY_I)
        {
            m_startSensor = false;
            m_pauseSensor = false;

            if (m_resetSensor)
            {
                m_isCameraSelected = false;
            }
            m_resetSensor = !m_resetSensor;
            std::cout << "Reset sensor toggled " << m_resetSensor << std::endl;
        }

        if ((key >= GLFW_KEY_0 && key <= GLFW_KEY_9) || (key >= GLFW_KEY_A && key <= GLFW_KEY_F))
            if (m_pauseSensor || m_startSensor || m_resetSensor)
            {
                if (!m_isCameraSelected)
                {
                    // between KEY_9 and KEY_A there are 2 symbols : and ;, so we skip them
                    m_cameraSelected   = key - ((key < GLFW_KEY_A) ? GLFW_KEY_0 : GLFW_KEY_0 + 2);
                    m_isCameraSelected = true;
                }
            }
    }

    ///------------------------------------------------------------------------------
    /// Image stream frame grabber.
    ///     - read from camera
    ///     - get an image with a useful format
    ///     - save image to file
    ///------------------------------------------------------------------------------
    void frameGrab(dwImageHandle_t frameCUDA, uint32_t index)
    {
        dwTime_t timeout = 500000;

        // stream that image to the CPU domain
        CHECK_DW_ERROR(dwImageStreamer_producerSend(frameCUDA, m_streamerToCPUGrab[index]));

        // receive the streamed image as a handle
        dwImageHandle_t frameCPU;
        CHECK_DW_ERROR(dwImageStreamer_consumerReceive(&frameCPU, timeout, m_streamerToCPUGrab[index]));

        // get an image from the frame
        dwImageCPU* imgCPU;
        CHECK_DW_ERROR(dwImage_getCPU(&imgCPU, frameCPU));

        // write the image to a file
        char fname[128];
        dwTime_t timestamp;
        dwImage_getTimestamp(&timestamp, frameCPU);
        if (timestamp == 0)
        {
            timestamp = m_screenshotCount++;
        }
        sprintf(fname, "GMSL_IDX%s_framegrab_%s.png", std::to_string(index).c_str(), std::to_string(timestamp).c_str());
        uint32_t error = lodepng_encode32_file(fname, imgCPU->data[0], imgCPU->prop.width, imgCPU->prop.height);
        std::cout << "Frame Grab saved to " << fname << " " << error << "\n";

        // reset frame grab flag
        // returned the consumed image
        CHECK_DW_ERROR(dwImageStreamer_consumerReturn(&frameCPU, m_streamerToCPUGrab[index]));

        // notify the producer that the work is done
        CHECK_DW_ERROR(dwImageStreamer_producerReturn(nullptr, timeout, m_streamerToCPUGrab[index]));
    }

    /**
     * @brief Sets and creates a pool of image to be used as output by the camera layer.
     *         - Append Camera Module NvSci Attributes to imageProperties.
     *         - This can be extended to append Other Engine Attributes.
     *         - Creates Images based on input parameters to be used to Camera Module.
     *         - If this is called, the default pool is not allocated.
     * 
     * @param cameraIdx : Active Camera Index
     * @param imagePoolIndex : Index for associated Pipeline for which ImagePool is being created 
     * @param outputType : Ouptut type of the image.
     */
    void setImagePool(uint32_t cameraIdx, uint32_t imagePoolIndex, dwCameraOutputType outputType)
    {
        dwImagePool pool{};

        std::cout << "Setting Image Pool for CameraOutputType: " << outputType;

        for (uint32_t i = 0; i < m_fifoSize; i++)
        {
            dwImageProperties imgProps{};

            // Gets information about the image properties for a given 'dwCameraImageOutputType'.
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imgProps, outputType, m_camera[cameraIdx]));

            // Append the NvSci allocation attribute of Camera Module.
            CHECK_DW_ERROR(dwSensorCamera_appendAllocationAttributes(&imgProps, outputType, m_camera[cameraIdx]));

            CHECK_DW_ERROR(dwImage_create(&m_imagesForImagePool[cameraIdx][imagePoolIndex][i], imgProps, m_context));
        }

        // pass the frames to the camera.
        pool.imageCount = m_fifoSize;
        pool.images     = &m_imagesForImagePool[cameraIdx][imagePoolIndex][0];

        CHECK_DW_ERROR(dwSensorCamera_setImagePool(pool, m_camera[cameraIdx]));
    }

    /**
     * @brief Set the Sync Obj object to be used with external applications
     * 
     * @param syncResourcesObj : Helper container to support Syncronization between modules.
     * @param cameraIdx : Active Camera Index
     * @param outputType : Ouptut type of the image.
     *                      - ICP and ISP Piplines maintain separate Sync Objects
     *                      - ISP0/1/2 All share a single syncronization Object.        
     */
    void setSyncObj(SampleCameraNvSciSyncResources& syncResourcesObj, uint32_t const cameraIdx, dwCameraOutputType const outputType)
    {
        // Input attribute keys specify desired synchronization object constraints and can be set/retrieved from the unreconciled NvSciSyncAttrList.
        std::vector<NvSciSyncAttrList> unreconciledList;

        // Unreconciled NvSciSyncAttrList consisting of the key/value pairs which caused the reconciliation failure.
        // This field is populated only if the reconciliation failed.
        NvSciSyncAttrList conflictList;

        /**
         * Each application wanting to use a sync object indicates its needs in the form of various attributes 
         * before the sync object is created. All applications attributes and has NvSciSync that reconcile them.
         * Successful reconciliation creates a new attribute list satisfying all applications constraints. 
         */
        NvSciSyncAttrList reconciledList;

        // NvSciSyncAttrList consisting a set of NvSciSyncAttrKey attributes defining synchronization object constraints to be used to wait on specific fence.
        NvSciSyncAttrList attrSyncList{};

        // NvSciSyncAttrList consisting a set of NvSciSyncAttrKey attributes defining synchronization object constraints to signal to signal a specific fence
        // Indicating completion of engine processing.
        NvSciSyncAttrList cameraAttrSyncList{};

        bool cpuWaiter              = true;
        NvSciSyncAccessPerm cpuPerm = NvSciSyncAccessPerm_WaitOnly;

        NvSciSyncAttrKeyValuePair syncAttrs[] = {
            {NvSciSyncAttrKey_NeedCpuAccess,
             (void*)&cpuWaiter,
             sizeof(cpuWaiter)},
            {NvSciSyncAttrKey_RequiredPerm,
             (void*)&cpuPerm,
             sizeof(cpuPerm)},
        };

        // Add Sync List - Waiter
        CHECK_NVSCI_ERROR(NvSciSyncAttrListCreate(m_nvsciSyncModule, &attrSyncList));

        // Set Waiter Sync Attributes for the List
        CHECK_NVSCI_ERROR(NvSciSyncAttrListSetAttrs(attrSyncList, syncAttrs, sizeof(syncAttrs) / sizeof(NvSciSyncAttrKeyValuePair)));

        // Add SyncList to the UnreconciledList Vector
        unreconciledList.push_back(attrSyncList);

        // Add Camera Sync Attributes - Signaler
        CHECK_NVSCI_ERROR(NvSciSyncAttrListCreate(m_nvsciSyncModule, &cameraAttrSyncList));

        // Fill the sync attributes for the camera pipeline to signal EOF fences from sensorCamera Module. Camera Signals when the process is complete.
        CHECK_DW_ERROR(dwSensorCamera_fillSyncAttributesNew(cameraAttrSyncList, dwSyncType::DW_SYNC_TYPE_SIGNALER, outputType, m_camera[cameraIdx]));

        // Add SyncList to the UnreconciledList Vector
        unreconciledList.push_back(cameraAttrSyncList);

        /**
         * @note :
         * This can be extented to add Sync Attributes (NvSciSyncAttrList) from other engines and applications by appending to unreconciledList
         */

        // Reconciles the input unreconciled NvSciSyncAttrLists into a new reconciled NvSciSyncAttrList to be used to create Sync Object.
        CHECK_NVSCI_ERROR(NvSciSyncAttrListReconcile(&unreconciledList[0], unreconciledList.size(),
                                                     &reconciledList, &conflictList));

        // Allocates and initializes a NvSciSyncObj that meets all the constraints specified in the given reconciled NvSciSyncAttrList.
        CHECK_NVSCI_ERROR(NvSciSyncObjAlloc(reconciledList, &syncResourcesObj.syncObj));

        // Set the synchronization object to which the camera pipeline will signal EOF fences.
        CHECK_DW_ERROR(dwSensorCamera_setSyncObjectNew(syncResourcesObj.syncObj, dwSyncType::DW_SYNC_TYPE_SIGNALER, outputType, m_camera[cameraIdx]));

        for (uint32_t idx = 0; idx < unreconciledList.size(); idx++)
        {
            if (unreconciledList[idx] != nullptr)
            {
                NvSciSyncAttrListFree(unreconciledList[idx]);
                unreconciledList[idx] = nullptr;
            }
        }

        if (conflictList != nullptr)
        {
            NvSciSyncAttrListFree(conflictList);
            conflictList = nullptr;
        }

        if (reconciledList != nullptr)
        {
            NvSciSyncAttrListFree(reconciledList);
            reconciledList = nullptr;
        }
    }

    /**
     * @brief Helper method to set and allocate and exchange buffers and synchronization objects across Camera Sensor and external modules/applications.
     */
    void setNvSciAttributesHelper()
    {
        std::cout << "Main: Setting Image Pool." << std::endl;

        // NvSciBuf
        for (uint32_t idx = 0; idx < m_totalCameras; ++idx)
        {
            if (m_useRaw)
            {
                // Create images to set the ImagePool for ICP Pipeline.
                setImagePool(idx, ICP_PIPELINE_IDX, dwCameraOutputType::DW_CAMERA_OUTPUT_NATIVE_RAW);
            }

            if (m_useProcessed)
            {
                // Create images to set the ImagePool for ISP0 Pipeline.
                setImagePool(idx, ISP_PIPELINE_IDX, dwCameraOutputType::DW_CAMERA_OUTPUT_NATIVE_PROCESSED);
            }

            if (m_useProcessed1)
            {
                // Create images to set the ImagePool for ISP1 Pipeline.
                setImagePool(idx, ISP1_PIPELINE_IDX, dwCameraOutputType::DW_CAMERA_OUTPUT_NATIVE_PROCESSED1);
            }

            if (m_useProcessed2)
            {
                // Create images to set the ImagePool for ISP2 Pipeline.
                setImagePool(idx, ISP2_PIPELINE_IDX, dwCameraOutputType::DW_CAMERA_OUTPUT_NATIVE_PROCESSED2);
            }
        }

        // Set a CPU Wait Context to wait for wait/signals from CPU modules.
        CHECK_NVSCI_ERROR(NvSciSyncCpuWaitContextAlloc(m_nvsciSyncModule, &m_cpuWaitContext));

        for (uint32_t idx = 0; idx < m_totalCameras; ++idx)
        {
            if (m_useRaw)
            {
                // Set synchronization primitives for externally allocated syncronization objects for ICP Pipeline.
                setSyncObj(m_icpSyncResourcesObj[idx], idx, dwCameraOutputType::DW_CAMERA_OUTPUT_NATIVE_RAW);
            }

            // ISP0/1/2 share single SyncObj
            if (m_useProcessed || m_useProcessed1 || m_useProcessed2)
            {
                // Set synchronization primitives for externally allocated syncronization objects for ISP0/1/2 Pipeline.
                setSyncObj(m_ispSyncResourcesObj[idx], idx, dwCameraOutputType::DW_CAMERA_OUTPUT_NATIVE_PROCESSED);
            }
        }
    }

    dwStatus initializeSerializer(dwSensorParams const& paramsClient, std::string fileName, uint32_t pipeLineIdx = 0)
    {
        if (m_bUseNewCodecStack)
        {
            return initializeSerializerNew(paramsClient, fileName, pipeLineIdx);
        }

        dwSerializerParams serializerParams;
        std::string newParams(paramsClient.parameters);

        // sample_camera recorder does not support async-record
        std::string asyncRecordStr(",async-record=1");
        size_t pos = newParams.find(asyncRecordStr);

        if (pos != std::string::npos)
        {
            newParams.erase(newParams.find(asyncRecordStr), asyncRecordStr.size());
        }

        newParams += std::string(",type=disk,file=") + fileName;

        if (pipeLineIdx == dwCameraISPType::DW_CAMERA_ISP1)
        {
            newParams += ",isp-type=1";
        }

        serializerParams.parameters = newParams.c_str();
        serializerParams.onData     = nullptr;

        CHECK_DW_ERROR(dwSensorSerializer_initialize(&(m_serializer[pipeLineIdx]), &serializerParams, m_camera[FIRST_CAMERA_IDX]));
        CHECK_DW_ERROR(dwSensorSerializer_start(m_serializer[pipeLineIdx]));

        return DW_SUCCESS;
    }

    dwStatus initializeSerializerNew(dwSensorParams const& paramsClient, std::string fileName, uint32_t pipeLineIdx = 0)
    {
        dwSerializerParams serializerParams;
        std::string newParams(paramsClient.parameters);

        // new codec stack only supports async-record
        newParams += std::string(",async-record=1,type=disk,file=") + fileName;

        if (pipeLineIdx == dwCameraISPType::DW_CAMERA_ISP1)
        {
            newParams += ",isp-type=1";
        }

        if (m_recordFormat == RECORD_FILE_FORMAT_LRAW || m_recordFormat == RECORD_FILE_FORMAT_RAW)
        {
            throw std::runtime_error("lraw/raw video format encoding is not supported on new codec stack. Please switch to pod \n");
        }

        if (m_recordFormat == RECORD_FILE_FORMAT_H264 || m_recordFormat == RECORD_FILE_FORMAT_H265)
        {
            throw std::runtime_error("h264/h265 video format encoding is not supported on new codec stack. Plese switch to mp4 \n");
        }

        setFileFormat(newParams);

        serializerParams.parameters = newParams.c_str();
        serializerParams.onData     = nullptr;

        dwCameraProperties cameraProperties{};
        CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&cameraProperties, m_camera[FIRST_CAMERA_IDX]));

        dwImageProperties imageProperties;
        if (pipeLineIdx == dwCameraISPType::DW_CAMERA_ISP0)
        {
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_camera[FIRST_CAMERA_IDX]));
        }
        else if (pipeLineIdx == dwCameraISPType::DW_CAMERA_ISP1)
        {
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED1, m_camera[FIRST_CAMERA_IDX]));
        }
        else if (pipeLineIdx == dwCameraISPType::DW_CAMERA_ISP2)
        {
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED2, m_camera[FIRST_CAMERA_IDX]));
        }

        // Create encoder
        dwEncoderConfig encoderConfig{};
        encoderConfig.rateControl.quality        = 20;
        encoderConfig.rateControl.gopSize        = 5;
        encoderConfig.rateControl.averageBitRate = 200000000;
        encoderConfig.rateControl.maxBitRate     = 200000000 * 4;
        encoderConfig.rateControl.mode           = DW_ENCODER_RATE_CONTROL_MODE_CONSTQP;

        dwCodecConfigVideo configVideo{};
        configVideo.width       = imageProperties.width;
        configVideo.height      = imageProperties.height;
        configVideo.frameRate   = cameraProperties.framerate;
        configVideo.bitDepth    = cameraProperties.imageBitDepth;
        configVideo.format      = DW_CODEC_VIDEO_FORMAT_YUV420;
        configVideo.rateControl = encoderConfig.rateControl;

        dwCodecMimeTypeConst_t codecType = DW_CODEC_MIME_TYPE_VIDEO_H264;
        if (m_recordFormat == RECORD_FILE_FORMAT_H265)
        {
            codecType = DW_CODEC_MIME_TYPE_VIDEO_H265;
        }

        CHECK_DW_ERROR(dwCodecHeader_createNew(&m_codecHeader, codecType.mime, &configVideo, nullptr, m_context));

        CHECK_DW_ERROR(dwSensorSerializerNew_initialize(&(m_serializerNew[pipeLineIdx]), m_codecHeader, &serializerParams, m_context));

        CHECK_DW_ERROR(dwSensorSerializerNew_start(m_serializerNew[pipeLineIdx]));

        return DW_SUCCESS;
    }

    void getRecordFileFormat(std::string format)
    {
        if (format.compare("h264") == 0)
        {
            m_recordFormat = RECORD_FILE_FORMAT_H264;
        }
        else if (format.compare("h265") == 0)
        {
            m_recordFormat = RECORD_FILE_FORMAT_H265;
        }
        else if (format.compare("raw") == 0)
        {
            m_recordFormat = RECORD_FILE_FORMAT_RAW;
        }
        else if (format.compare("lraw") == 0)
        {
            m_recordFormat = RECORD_FILE_FORMAT_LRAW;
        }
        else if (format.compare("mp4") == 0)
        {
            m_recordFormat = RECORD_FILE_FORMAT_MP4;
        }
        else if (format.compare("pod") == 0)
        {
            m_recordFormat = RECORD_FILE_FORMAT_POD;
        }
        else
        {
            m_recordFormat = RECORD_FILE_FORMAT_UNSUPPORTED;
        }
    }

    void setFileFormat(std::string params)
    {
        if (m_recordFormat == RECORD_FILE_FORMAT_H264)
        {
            params += ",format=h264";
        }
        else if (m_recordFormat == RECORD_FILE_FORMAT_H265)
        {
            params += ",format=h265";
        }
        else if (m_recordFormat == RECORD_FILE_FORMAT_RAW)
        {
            params += ",format=raw";
        }
        else if (m_recordFormat == RECORD_FILE_FORMAT_POD)
        {
            params += ",format=pod";
        }
        else if (m_recordFormat == RECORD_FILE_FORMAT_LRAW)
        {
            params += ",format=lraw";
        }
        else if (m_recordFormat == RECORD_FILE_FORMAT_MP4)
        {
            params += ",format=mp4";
        }
        else
        {
            std::cout << "setFileFormat: Incorrect format of sensor params \n";
        }
    }

    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------

        // initialize logger to print verbose message on console in color
        dwLogger_initialize(getConsoleLoggerCallback(true));
        dwLogger_setLogLevel(DW_LOG_VERBOSE);

        initializeDriveWorks(m_context);
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));

        // Get Sync Module
        CHECK_DW_ERROR(dwContext_getNvSciSyncModule(&m_nvsciSyncModule, m_context));

        // if rig is selected
        m_totalCameras = 0;

        // the complete configuration is automatically loaded by dwRig module
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rigConfig, m_context,
                                                getArgument("rig").c_str()));

        uint32_t cnt = 0;
        CHECK_DW_ERROR(dwRig_getSensorCountOfType(&cnt, DW_SENSOR_CAMERA, m_rigConfig));

        m_isProcessed1WriteEnabled = (std::stoi(getArgument("enable-processed1-write")) > 0);
        m_bUseNvSciPath            = (std::stoi(getArgument("set-external-nvsci-objs")) > 0);

        //------------------------------------------------------------------------------
        // initializes cameras
        // -----------------------------------------
        dwSensorParams paramsClient[MAX_CAMS] = {};
        for (uint32_t i = 0; i < cnt; i++)
        {
            uint32_t cameraSensorIdx = 0;
            CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&cameraSensorIdx, DW_SENSOR_CAMERA, i, m_rigConfig));

            // get rig parsed protocol from dwRig
            const char* protocol = nullptr;
            CHECK_DW_ERROR(dwRig_getSensorProtocol(&protocol, cameraSensorIdx, m_rigConfig));
            const char* params = nullptr;
            CHECK_DW_ERROR(dwRig_getSensorParameterUpdatedPath(&params, cameraSensorIdx, m_rigConfig));
            paramsClient[i].protocol   = protocol;
            paramsClient[i].parameters = params;

            std::cout << "onInitialize: creating camera.gmsl with params: " << params << std::endl;
            CHECK_DW_ERROR(dwSAL_createSensor(&m_camera[m_totalCameras], paramsClient[i], m_sal));

            m_totalCameras++;

            m_useRaw        = std::string::npos != std::string(params).find("raw");
            m_useProcessed  = std::string::npos != std::string(params).find("processed");
            m_useProcessed1 = std::string::npos != std::string(params).find("processed1");
            m_useProcessed2 = std::string::npos != std::string(params).find("processed2");

            m_bUseNewCodecStack = false;
            std::string newCodecStackStr("newcodecstack=1");
            size_t pos = std::string(params).find(newCodecStackStr);

            if (pos != std::string::npos)
            {
                m_bUseNewCodecStack = true;
            }

            if (!m_useProcessed && !m_useRaw) //if neither output format specified, assume processed output
                m_useProcessed = true;
            if (!m_useProcessed)
                std::cout << "Processed not selected as master output format parameter. No images will be previewed." << std::endl;

            if (m_bUseNvSciPath)
            {
                auto pos = std::string(params).find("fifo-size=");
                if (pos != std::string::npos)
                {
                    static constexpr uint32_t COMMA_INCLUDED_STR_MAX_SIZE = 5;
                    auto fifoSizeStr                                      = std::string(params).substr(pos + std::string("fifo-size=").size(), COMMA_INCLUDED_STR_MAX_SIZE);
                    auto fifoSizeStrDataPos                               = fifoSizeStr.find(",");
                    if ((fifoSizeStrDataPos != std::string::npos) && (fifoSizeStr.size() != 0))
                    {
                        m_fifoSize = std::stoi(fifoSizeStr.substr(0, fifoSizeStrDataPos));
                    }
                }

                if (m_fifoSize == 0)
                {
                    // Minimum Recommended FifoSize
                    m_fifoSize = 4;
                }
            }
        }

        CHECK_DW_ERROR(dwSAL_start(m_sal));

        if (m_bUseNvSciPath)
        {
            setNvSciAttributesHelper();
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            CHECK_DW_ERROR(dwVisualizationInitialize(&m_viz, m_context));
            m_colorPerPort[0] = {1, 0, 0, 1};
            m_colorPerPort[1] = {0, 1, 0, 1};
            m_colorPerPort[2] = {0, 0, 1, 1};
            m_colorPerPort[3] = {0, 0, 0, 1};

            log("onInitialize: Total cameras %d\n", m_totalCameras);

            CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&params, getWindowWidth(), getWindowHeight()));
            params.defaultTile.lineWidth = 2.0f;
            params.defaultTile.font      = DW_RENDER_ENGINE_FONT_VERDANA_24;
            params.maxBufferCount        = 1;

            float32_t windowSize[2] = {static_cast<float32_t>(getWindowWidth()), static_cast<float32_t>(getWindowHeight())};
            params.bounds           = {0, 0};

            uint32_t tilesPerRow = 1;
            params.bounds.width  = windowSize[0];
            params.bounds.height = windowSize[1];
            switch (m_totalCameras)
            {
            case 1:
                tilesPerRow = 1;
                break;
            case 2:
                params.bounds.height = (windowSize[1] / 2);
                params.bounds.y      = (windowSize[1] / 2);
                tilesPerRow          = 2;
                break;
            case 3:
                tilesPerRow = 2;
                break;
            case 4:
                tilesPerRow = 2;
                break;
            default:
                tilesPerRow = 4;
                break;
            }

            CHECK_DW_ERROR(dwRenderEngine_initialize(&m_renderEngine, &params, m_viz));

            dwRenderEngineTileState paramList[MAX_CAMS];
            for (uint32_t i = 0; i < m_totalCameras; ++i)
            {
                dwRenderEngine_initTileState(&paramList[i]);
                paramList[i].modelViewMatrix = DW_IDENTITY_MATRIX4F;
                paramList[i].font            = DW_RENDER_ENGINE_FONT_VERDANA_24;
            }

            dwRenderEngine_addTilesByCount(m_tileVideo, m_totalCameras, tilesPerRow, paramList, m_renderEngine);
        }

        //------------------------------------------------------------------------------
        // initializes streamers
        // -----------------------------------------
        if (m_useProcessed)
        {
            dwImageProperties imageProperties{};
            for (uint32_t i = 0; i < m_totalCameras; ++i)
            {
                std::cout << "onInitialize: getting image props " << i << std::endl;
                CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imageProperties, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera[i]));

                std::cout << "onInitialize: initilizing stream: " << i << std::endl;
                CHECK_DW_ERROR(dwImageStreamerGL_initialize(&m_streamerToGL[i], &imageProperties, DW_IMAGE_GL, m_context));

                CHECK_DW_ERROR(dwImageStreamer_initialize(&m_streamerToCPUGrab[i], &imageProperties, DW_IMAGE_CPU, m_context));
            }
        }

        //------------------------------------------------------------------------------
        // initializes recorder serializer
        // -----------------------------------------
        {
            const std::string& videoFile = getArgument("write-file");
            m_recordCamera               = !videoFile.empty();

            if (m_recordCamera)
            {
                // Determine format from file extension
                std::string format;
                size_t found = videoFile.find_last_of(".");

                if (found != std::string::npos)
                {
                    format = videoFile.substr(found + 1);
                }

                if (format.compare("h264") != 0 && format.compare("h265") != 0 && format.compare("mp4") != 0 && format.compare("raw") != 0 && format.compare("lraw") != 0)
                {
                    logError("Unsupported output format. Must be one of h264, h265, mp4 or raw/lraw\n");
                    return false;
                }

                if (format.compare("raw") == 0 || format.compare("lraw") == 0)
                    m_recordRaw = true;

                getRecordFileFormat(format);
                if (m_recordFormat == RECORD_FILE_FORMAT_UNSUPPORTED)
                {
                    logError("Initializing serializer for output formt not supported\n");
                    return false;
                }

                // Initialize Base Serializer (raw/ISP0)
                dwStatus status = initializeSerializer(paramsClient[FIRST_CAMERA_IDX], std::string(getArgument("write-file")), 0);
                if (status != DW_SUCCESS)
                {
                    logError("Initializing serializer for raw/processed pipleline failed\n");
                    return false;
                }

                if (m_useProcessed1 && m_isProcessed1WriteEnabled)
                {
                    if (format.compare("h264") == 0 || format.compare("h265") == 0 || format.compare("mp4") == 0)
                    {
                        std::string fileName = videoFile.substr(0, found) + std::string("-processed1.") + format;
                        status               = initializeSerializer(paramsClient[FIRST_CAMERA_IDX], fileName, dwCameraISPType::DW_CAMERA_ISP1);
                        if (status != DW_SUCCESS)
                        {
                            logError("Initializing serializer for processed1 pipleline failed\n");
                            return false;
                        }
                    }
                }
            }
        }

        m_screenshot.reset(new ScreenshotHelper(m_context, m_sal, getWindowWidth(), getWindowHeight(), "CameraGMSLCustom"));

        std::cout << "Main: Starting master." << std::endl;
        for (uint32_t i = 0; i < m_totalCameras; ++i)
        {
            CHECK_DW_ERROR(dwSensor_start(m_camera[i]));
        }

        return true;
    }

    /**
     * @brief Wait on specified NvSciSyncFence on allocated CPU Context.
     * 
     * @param fence : Defines the opaque NvSciSyncFence, synchronization primitive snapshot that must be reached to release the waiters.
     * @param outputType : Ouptut type of the image.
     * @param frame : A pointer to a handle to a frame read from the camera.
     */
    void waitOnFence(NvSciSyncFence& fence, dwCameraOutputType outputType, dwCameraFrameHandle_t& frame)
    {
        // Get EOF fence of the current camera frame
        CHECK_DW_ERROR(dwSensorCamera_getEOFFence(&fence, outputType, frame));

        // Performs a synchronous wait on the EOF fence captured from camera  object until the has been signaled or the timeout expires.
        // Allocated NvSciSyncCpuWaitContext is used to wait on current frame fence.
        CHECK_NVSCI_ERROR(NvSciSyncFenceWait(&fence, m_cpuWaitContext, -1));
    }

    void onRenderHelper(dwCameraFrameHandle_t frame, uint8_t cameraIndex, NvSciSyncFence& fence)
    {
        dwImageHandle_t img           = DW_NULL_HANDLE;
        dwCameraOutputType outputType = DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8;

        CHECK_DW_ERROR(dwRenderEngine_setTile(m_tileVideo[cameraIndex], m_renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));

        CHECK_DW_ERROR(dwSensorCamera_getImageAsync(&img, outputType, frame));

        // Synchronize for wait on specified fence for signal from CPU on externally allocated CPU Sync Context.
        if (m_bUseNvSciPath)
        {
            waitOnFence(fence, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frame);
        }

        // stream that image to the GL domain
        CHECK_DW_ERROR(dwImageStreamerGL_producerSend(img, m_streamerToGL[cameraIndex]));

        // receive the streamed image as a handle
        dwImageHandle_t frameGL;
        CHECK_DW_ERROR(dwImageStreamerGL_consumerReceive(&frameGL, 33000, m_streamerToGL[cameraIndex]));

        dwImageGL* imageGL;
        CHECK_DW_ERROR(dwImage_getGL(&imageGL, frameGL));

        // render received texture
        {
            dwVector2f range{};
            range.x = imageGL->prop.width;
            range.y = imageGL->prop.height;
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderImage2D(imageGL, {0, 0, range.x, range.y}, m_renderEngine));
            dwRenderEngine_setColor(m_colorPerPort[0], m_renderEngine);

            dwTime_t timestamp;
            dwImage_getTimestamp(&timestamp, img);
            std::string tileString = std::string("Camera:") + std::to_string(cameraIndex) + std::string(" Time:") + std::to_string(timestamp);

            CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_16, m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_renderText2D(tileString.c_str(), {25, 80}, m_renderEngine));

            m_screenshot->processScreenshotTrig();
        }
        // returned the consumed image
        CHECK_DW_ERROR(dwImageStreamerGL_consumerReturn(&frameGL, m_streamerToGL[cameraIndex]));

        // notify the producer that the work is done
        CHECK_DW_ERROR(dwImageStreamerGL_producerReturn(nullptr, 32000, m_streamerToGL[cameraIndex]));

        if (m_frameGrab && img)
        {
            frameGrab(img, cameraIndex);
        }
    }

    void checkToPauseStartResetSensor()
    {
        if (m_isCameraSelected)
        {
            if (m_cameraSelected > m_totalCameras)
            {
                std::cout << "Index above camera count, no action" << std::endl;
                return;
            }

            if (m_pauseSensor)
            {
                std::cout << "Pausing sensor " << m_cameraSelected << std::endl;
                dwStatus status = dwSensor_stop(m_camera[m_cameraSelected]);
                if (status != DW_SUCCESS)
                {
                    throw std::runtime_error("onRender: Cannot pause camera " + m_cameraSelected);
                }
                m_pauseSensor = false;
            }

            if (m_startSensor)
            {
                std::cout << "Starting sensor " << m_cameraSelected << std::endl;
                dwStatus status = dwSensor_start(m_camera[m_cameraSelected]);
                if (status != DW_SUCCESS)
                {
                    throw std::runtime_error("onRender: Cannot start camera " + m_cameraSelected);
                }
                m_startSensor = false;
            }

            if (m_resetSensor)
            {
                std::cout << "Reset sensor " << m_cameraSelected << std::endl;
                dwStatus status = dwSensor_reset(m_camera[m_cameraSelected]);
                if (status != DW_SUCCESS)
                {
                    throw std::runtime_error("onRender: Cannot reset camera " + m_cameraSelected);
                }
                m_resetSensor = false;
            }
        }
    }

    void onRender() override
    {

        checkToPauseStartResetSensor();
        dwCameraFrameHandle_t frame[MAX_CAMS];

        // Default synchronization primitive where syncronization objects would be used to wait/signal upon.
        // Every NvSciSyncFence must be initialized to default after its storage is allocated before the first time it is passed to any NvSciSync API.
        NvSciSyncFence fence[MAX_CAMS] = {NvSciSyncFenceInitializer};

        for (uint32_t i = 0; i < m_totalCameras; ++i)
        {
            dwStatus status = DW_NOT_READY;

            uint32_t countFailure = 0;
            while ((status == DW_NOT_READY) || (status == DW_END_OF_STREAM))
            {
                status = dwSensorCamera_readFrame(&frame[i], 333333, m_camera[i]);
                countFailure++;
                if (countFailure == 1000000)
                {
                    std::cout << "Camera doesn't seem responsive, exit loop and stopping the sample" << std::endl;
                    break;
                }

                // Reset the sensor to support loopback
                if (status == DW_END_OF_STREAM)
                {
                    dwSensor_reset(m_camera[i]);
                    std::cout << "Video reached end of stream" << std::endl;
                }
            }

            if (status == DW_TIME_OUT)
            {
                throw std::runtime_error("onRender: Timeout waiting for camera " + i);
            }

            CHECK_DW_ERROR(status);
        }
        for (uint32_t i = 0; i < m_totalCameras; ++i)
        {

            if (m_useRaw && m_bUseNvSciPath)
            {
                // Wait of fence captured from Camera Module on extrnally allocated CPU Context in the frame for ICP Pipeline.
                waitOnFence(fence[i], DW_CAMERA_OUTPUT_NATIVE_RAW, frame[i]);
            }

            if (m_recordCamera && i == 0)
            {
                if (m_bUseNewCodecStack)
                {
                    CHECK_DW_ERROR(dwSensorSerializerNew_serializeCameraFrameAsync(frame[i], m_serializerNew[dwCameraISPType::DW_CAMERA_ISP0]));
                }
                else
                {
                    CHECK_DW_ERROR(dwSensorSerializer_serializeCameraFrameAsync(frame[i], m_serializer[dwCameraISPType::DW_CAMERA_ISP0]));
                }
            }

            if (m_useProcessed)
            {
                onRenderHelper(frame[i], i, fence[i]);
            }
#if (VIBRANTE_PDK_DECIMAL >= 6000200) || defined(LINUX_AND_EMU)
            dwImageHandle_t img = DW_NULL_HANDLE;
            if (m_useProcessed1)
            {
                if (m_bUseNvSciPath)
                {
                    waitOnFence(fence[i], DW_CAMERA_OUTPUT_NATIVE_PROCESSED1, frame[i]);
                }

                // Only native processed ISP1/2 outputs(not cuda) are available for now
                CHECK_DW_ERROR(dwSensorCamera_getImageAsync(&img, DW_CAMERA_OUTPUT_NATIVE_PROCESSED1, frame[i]));

                if (m_recordCamera && m_isProcessed1WriteEnabled)
                {
                    if (m_bUseNewCodecStack)
                    {
                        CHECK_DW_ERROR(dwSensorSerializerNew_serializeCameraFrameAsync(frame[i], m_serializerNew[dwCameraISPType::DW_CAMERA_ISP1]));
                    }
                    else
                    {
                        CHECK_DW_ERROR(dwSensorSerializer_serializeCameraFrameAsync(frame[i], m_serializer[dwCameraISPType::DW_CAMERA_ISP1]));
                    }
                }
            }
            if (m_useProcessed2)
            {
                if (m_bUseNvSciPath)
                {
                    waitOnFence(fence[i], DW_CAMERA_OUTPUT_NATIVE_PROCESSED2, frame[i]);
                }

                CHECK_DW_ERROR(dwSensorCamera_getImageAsync(&img, DW_CAMERA_OUTPUT_NATIVE_PROCESSED2, frame[i]));
            }
#endif
        }

        m_frameGrab = false;

        for (uint32_t i = 0; i < m_totalCameras; ++i)
        {
            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame[i]));
        }

        if (!m_useProcessed)
        {
            // Display a message on screen that no image is expected.
            CHECK_DW_ERROR(dwRenderEngine_setTile(m_tileVideo[0], m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_resetTile(m_renderEngine));
            CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D({640, 480}, m_renderEngine));
            dwRenderEngine_setColor({1, 1, 1, 0}, m_renderEngine);
            CHECK_DW_ERROR(dwRenderEngine_renderText2D("No preview available when only raw capture.", {25, 25}, m_renderEngine));
        }
        renderutils::renderFPS(m_renderEngine, getCurrentFPS());
    }
};

//#######################################################################################
CameraCustomSimpleApp::CameraCustomSimpleApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
}

void CameraCustomSimpleApp::initializeDriveWorks(dwContextHandle_t& context) const
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

//#######################################################################################
int main(int argc, const char** argv)
{
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("rig", (dw_samples::SamplesDataPath::get() + "/samples/sensors/camera/camera/rig.json").c_str(), "This is the main source of input configuration for the cameras, can setup any possible configuration"),
                              ProgramArguments::Option_t("write-file", "", "If this string is not empty, then the serializer will record in this location\n"),
                              ProgramArguments::Option_t("enable-processed1-write", "0", "If this set to 1, and write-file is not empty, serializer will record ISP Pipeline1 in the location\n"),
                              ProgramArguments::Option_t("set-external-nvsci-objs", "0", "If this set to 1, Use NvSci for communication and synchronization \n"),
                          },
                          "Sample illustrating how to use camera interfaces");

    // Window/GL based application
    CameraCustomSimpleApp app(args);
    app.initializeWindow("Camera Sample", 1280, 800, args.enabled("offscreen"));
    return app.run();
}