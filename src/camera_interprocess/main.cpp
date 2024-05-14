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

#include <framework/DriveWorksSample.hpp>
#include <framework/Checks.hpp>

#include <memory>
#include <thread>
#include <string>
#include <unistd.h>

// Context, SAL
#include <dw/core/context/Context.h>
#include <dw/sensors/Sensors.h>

// IMAGE
#include <dw/interop/streamer/ImageStreamer.h>

#include <dwcgf/channel/ChannelFactory.hpp>
#include <dwcgf/channel/ChannelConnector.hpp>
#include <dwcgf/port/Port.hpp>
#include "ChannelPacketTypes.hpp"
#include <dwframework/dwnodes/common/factories/DWChannelFactory.hpp>

// Renderer
#include <dwvisualization/core/Renderer.h>

#include <image_common/utils.hpp>
#include <dw/sensors/camera/Camera.h>
#include <dw/rig/Rig.h>
#include <dwvisualization/image/FrameCapture.h>

#define EGL_STREAMER

#define MAX_CAMS 4
#define MAX_CAMERA_POOL_SIZE 128
#define DEFAULT_TIMEOUT 33333

// clang-format off
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// clang-format on

/**
 * Class that holds functions anda variables common to all stereo samples
 */
using namespace dw_samples::common;

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

static void setCpuWaiterAttributes(NvSciSyncAttrList attrList)
{
    const bool cpuAccess           = true;
    const NvSciSyncAccessPerm perm = NvSciSyncAccessPerm_WaitOnly;
    dw::core::Array<NvSciSyncAttrKeyValuePair, 2U> pairArray = {{{NvSciSyncAttrKey_NeedCpuAccess, &cpuAccess, sizeof(cpuAccess)},
                                                                 {NvSciSyncAttrKey_RequiredPerm, &perm, sizeof(perm)}}};
    CHECK_NVSCI_ERROR(NvSciSyncAttrListSetAttrs(attrList, &pairArray[0], pairArray.size()));
}

enum class SyncMode
{
    NONE,
    P2C,
    C2P,
    BOTH
};

class DwCGFCameraInterprocessApp : public DriveWorksSample
{
    using PacketTSpec = dwImageProperties;
    using PacketT     = dwImageHandle_t;

public:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t context = DW_NULL_HANDLE;
    dwSALHandle_t m_sal       = DW_NULL_HANDLE;

    bool m_hasProducer      = true;
    uint32_t m_numConsumers = 1;

    SyncMode m_syncMode = SyncMode::NONE;
    // image with RGBA format that we are going to copy convert onto
    std::thread m_workers[MAX_CAMS];
    std::thread m_senderWorkers[MAX_CAMS];

    dwSensorHandle_t m_camera[MAX_CAMS];
    uint32_t m_totalCameras = 0; //actual number of cameras used,

    dwImageProperties m_imageProperties[MAX_CAMS];
    dwCameraOutputType m_outputType = DW_CAMERA_OUTPUT_NATIVE_PROCESSED;

    int m_frames  = 300;
    int m_counter = 1;

    uint32_t m_height = 2168u; //encode image height
    uint32_t m_width  = 3848u; // encode image width

    bool m_isProducer = false;
    bool m_waitISP    = false;

    dwFrameCaptureHandle_t m_frameCapture = DW_NULL_HANDLE;
    dwFrameCaptureParams frameParams{};

    // dwChannel C2C Param
    std::string m_prodReaches{};
    std::string m_prodStreamNames{};
    std::vector<std::string> m_consReaches{};
    std::vector<std::string> m_consStreamNames{};

    // Channel factory
    std::unique_ptr<dw::framework::ChannelFactory> m_channelFactory{};
    std::unique_ptr<dw::framework::ChannelConnector> m_channelConnector{};

    // Channels
    std::shared_ptr<dw::framework::ChannelObject> m_channelOutputImages{};
    std::shared_ptr<dw::framework::ChannelObject> m_channelInputImage{};

    // Ports
    std::unique_ptr<dw::framework::PortOutput<PacketT>> m_outputImagePorts{};
    std::unique_ptr<dw::framework::PortInput<PacketT>> m_inputImagePort{};

    NvSciSyncObj m_producerSignalerSyncObj[MAX_CAMS];
    NvSciSyncCpuWaitContext m_cpuWaitContext{};

    std::map<dwImageHandle_t, dwCameraFrameHandle_t> m_img2Frame{};
    std::map<dwImageHandle_t, dwImageHandle_t*> m_img2channelPool{};

public:
    DwCGFCameraInterprocessApp(const ProgramArguments& args);

    // Sample framework
    void onInitFrameCapture();
    void startCameraThread(int i);
    void processReturns(uint32_t i);
    void processSends(uint32_t i);
    void initCameras();
    bool onInitialize() override final;
    void onRender() override final;
    void onProcess() override final {}
    void onRelease() override final;
    SyncMode parseSyncMode(std::string syncMode);
    bool p2cEnabled();
    bool c2pEnabled();

    dw::framework::OnSetSyncAttrs getConsumerSyncAttrs(size_t camIdx, bool isSignaler);
    dw::framework::OnSetSyncAttrs getProducerSyncAttrs(size_t camIdx, bool isSignaler);

    void waitNvMediaImage(dwImageHandle_t img);

    void setImagePool(int i);

    void setSyncObj(int i);
};

void DwCGFCameraInterprocessApp::onInitFrameCapture()
{
    std::string params = "type=disk";
    params += ",framerate=30";
    params += ",file=" + getArgument("capture");
    params += ",bitrate=" + getArgument("bitrate");

    frameParams.params.parameters          = params.c_str();
    frameParams.width                      = m_width;
    frameParams.height                     = m_height;
    frameParams.mode                       = DW_FRAMECAPTURE_MODE_SERIALIZE;
    frameParams.setupForDirectCameraOutput = true;
    CHECK_DW_ERROR(dwFrameCapture_initialize(&m_frameCapture, &frameParams, m_sal, context));
}

//#######################################################################################
DwCGFCameraInterprocessApp::DwCGFCameraInterprocessApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
}

SyncMode DwCGFCameraInterprocessApp::parseSyncMode(std::string syncMode)
{
    SyncMode out;
    if (syncMode == "p2c")
    {
        out = SyncMode::P2C;
    }
    else if (syncMode == "c2p")
    {
        out = SyncMode::C2P;
    }
    else if (syncMode == "both")
    {
        out = SyncMode::BOTH;
    }
    else if (syncMode == "none")
    {
        out = SyncMode::NONE;
    }
    else
    {
        throw std::runtime_error("Sync mode not recognized");
    }

    return out;
}

bool DwCGFCameraInterprocessApp::p2cEnabled()
{
    return m_syncMode == SyncMode::BOTH || m_syncMode == SyncMode::P2C;
}

bool DwCGFCameraInterprocessApp::c2pEnabled()
{
    return m_syncMode == SyncMode::BOTH || m_syncMode == SyncMode::C2P;
}

dw::framework::OnSetSyncAttrs DwCGFCameraInterprocessApp::getConsumerSyncAttrs(size_t camIdx, bool isSignaler)
{
    camIdx = 0;
    if (p2cEnabled() && !isSignaler)
    {
        return [](NvSciSyncAttrList attrList) {
            setCpuWaiterAttributes(attrList);
        };
    }
    return {};
}

dw::framework::OnSetSyncAttrs DwCGFCameraInterprocessApp::getProducerSyncAttrs(size_t camIdx, bool isSignaler)
{
    if (p2cEnabled() && isSignaler)
    {
        return [this, camIdx](NvSciSyncAttrList attrList) {
            CHECK_DW_ERROR(dwSensorCamera_fillSyncAttributesNew(attrList, dwSyncType::DW_SYNC_TYPE_SIGNALER, m_outputType, m_camera[camIdx]));
        };
    }
    return {};
}

// Set the image pool for the camera as the dwChannel's image pool for the ith camera.
void DwCGFCameraInterprocessApp::setImagePool(int i)
{
    dwImageHandle_t imagesPool[MAX_CAMERA_POOL_SIZE];
    dwImagePool pool{};
    size_t imgCnt = 0;
    // Take all available output buffers from the channel
    while (true)
    {
        // Wait for the frame to become available.
        // This is important as though the channel is connected, a background thread
        // may still be in the process of making the frames available to the client.
        dwStatus status = m_outputImagePorts->wait(100'000);
        if (status == DW_NOT_AVAILABLE)
        {
            break;
        }
        else if (status != DW_SUCCESS)
        {
            throw std::runtime_error("Channel Producer timeout when acquiring packet.");
        }

        // get the output image handle.
        // The returned pointer to image handle serves as a token to interact with the channel
        // for that frame.
        dwImageHandle_t* outputFreePacket = m_outputImagePorts->getFreeElement();
        // add the frame to the list of frames to be passed to the camera.
        imagesPool[imgCnt++] = *outputFreePacket;
        // save the pointer to image handle so it can be used to send the frame later.
        m_img2channelPool[*outputFreePacket] = outputFreePacket;
    }
    // pass the frames retrieved from channel to the camera.
    pool.imageCount = imgCnt;
    pool.images     = &imagesPool[0];
    CHECK_DW_ERROR(dwSensorCamera_setImagePool(pool, m_camera[i]));

    return;
}

void DwCGFCameraInterprocessApp::setSyncObj(int i)
{
    // retrieve the sync obj from the producer channel
    auto span = dw::core::make_span<NvSciSyncObj>(&m_producerSignalerSyncObj[i], 1);
    m_outputImagePorts->getSyncSignaler().getSyncObjs(span);
    // set the sync obj for the camera.
    CHECK_DW_ERROR(dwSensorCamera_setSyncObjectNew(span[0], dwSyncType::DW_SYNC_TYPE_SIGNALER, m_outputType, m_camera[i]));

    return;
}

void DwCGFCameraInterprocessApp::startCameraThread(int i)
{
    setImagePool(i);
    setSyncObj(i);

    CHECK_DW_ERROR(dwSensor_start(m_camera[i]));
    m_workers[i] = std::thread([this, i]() {
        processReturns(i);
    });
    m_senderWorkers[i] = std::thread([this, i]() {
        processSends(i);
    });
}

void DwCGFCameraInterprocessApp::waitNvMediaImage(dwImageHandle_t img)
{
#if VIBRANTE_PDK_DECIMAL < 6000400
    dwImageNvMedia* nvmedia;
    NvMediaTaskStatus taskStatus;
    CHECK_DW_ERROR(dwImage_getNvMedia(&nvmedia, img));
    NvMediaImageGetStatus(nvmedia->img, NVMEDIA_IMAGE_TIMEOUT_INFINITE, &taskStatus);
#else
    static_cast<void>(img);
#endif
}

void DwCGFCameraInterprocessApp::processSends(uint32_t i)
{
    uint64_t delta = 0;
    dwTime_t t1, t2;
    int counter         = 1;
    dwImageHandle_t img = DW_NULL_HANDLE;
    dwCameraFrameHandle_t frame;

    std::thread::id id = std::this_thread::get_id();
    std::cout << "Camera[" << i << "] Starting camera sender thread:" << id << std::endl;

    while (counter < m_frames)
    {
        std::cout << "Camera[" << i << "] Producer is sending frame:" << counter << std::endl;
        CHECK_DW_ERROR(dwSensorCamera_readFrame(&frame, 1000000, m_camera[i]));
        CHECK_DW_ERROR(dwSensorCamera_getImage(&img, m_outputType, frame));
        m_img2Frame[img] = frame;

        NvSciSyncFence signalFence = NvSciSyncFenceInitializer;
        CHECK_DW_ERROR(dwSensorCamera_getEOFFence(&signalFence, m_outputType, frame));

        dwImageMetaData meta;
        CHECK_DW_ERROR(dwImage_getMetaData(&meta, img));
        CHECK_DW_ERROR(dwContext_getCurrentTime(&t1, context));

        if (p2cEnabled())
        {
            // set signal fences
            auto span = dw::core::make_span<NvSciSyncFence>(&signalFence, 1);
            m_outputImagePorts->setSignalFences(m_img2channelPool[img], span);
        }

        if (m_waitISP)
        {
            std::cout << "Camera[" << i << "] Waiting for isp processing ..." << std::endl;
            waitNvMediaImage(img);
        }

        CHECK_DW_ERROR(m_outputImagePorts->send(m_img2channelPool[img]));

        CHECK_DW_ERROR(dwContext_getCurrentTime(&t2, context));
        delta += (t2 - t1);
        counter++;
    }
}

void DwCGFCameraInterprocessApp::processReturns(uint32_t i)
{
    uint64_t delta = 0;
    int counter    = 1;

    dwTime_t t1, t2;

    std::thread::id id = std::this_thread::get_id();
    std::cout << "Camera[" << i << "] Starting camera thread:" << id << std::endl;

    while (counter < m_frames)
    {
        CHECK_DW_ERROR(dwContext_getCurrentTime(&t1, context));

        // dwChannel return  Packets
        if (m_outputImagePorts->wait(10'000'000) == DW_SUCCESS)
        {
            PacketT* outputFreePacket = m_outputImagePorts->getFreeElement();

            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_img2Frame[*outputFreePacket]));
        }

        CHECK_DW_ERROR(dwContext_getCurrentTime(&t2, context));
        delta += (t2 - t1);

        counter++;
    }
    std::cout << "Camera[" << i << "] producer average returning time is " << delta / counter << std::endl;
    DwCGFCameraInterprocessApp::stop();
}

void DwCGFCameraInterprocessApp::initCameras()
{
    // if rig is selected
    dwRigHandle_t rigConfig{};
    CHECK_DW_ERROR(dwRig_initializeFromFile(&rigConfig, context,
                                            getArgument("rig").c_str()));

    uint32_t cnt = 0;
    CHECK_DW_ERROR(dwRig_getSensorCountOfType(&cnt, DW_SENSOR_CAMERA, rigConfig));

    for (uint32_t i = 0; i < cnt; i++)
    {
        uint32_t cameraSensorIdx = 0;
        CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&cameraSensorIdx, DW_SENSOR_CAMERA, i, rigConfig));

        const char* protocol = nullptr;
        CHECK_DW_ERROR(dwRig_getSensorProtocol(&protocol, cameraSensorIdx, rigConfig));
        const char* params = nullptr;
        CHECK_DW_ERROR(dwRig_getSensorParameterUpdatedPath(&params, cameraSensorIdx, rigConfig));
        dwSensorParams paramsClient{};
        paramsClient.protocol   = protocol;
        paramsClient.parameters = params;

        std::cout << "onInitialize: creating camera.gmsl with params: " << params << std::endl;
        CHECK_DW_ERROR(dwSAL_createSensor(&m_camera[m_totalCameras], paramsClient, m_sal));
        m_totalCameras++;
    }

    dwRig_release(rigConfig);

    // Start SAL so that the full camera group can be safely instantiated.
    // This must be called prior to using dwSensorCamera_appendAllocationAttributes()
    // as the dirver for cameras other than master may not yet be initialized.
    CHECK_DW_ERROR(dwSAL_start(m_sal));

    for (uint32_t i = 0; i < m_totalCameras; ++i)
    {
        // Query the image properties from the camera
        CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&m_imageProperties[i], m_outputType, m_camera[i]));
        // Append the underlying driver's allocation attributes to the image properties.
        CHECK_DW_ERROR(dwSensorCamera_appendAllocationAttributes(&m_imageProperties[i], m_outputType, m_camera[i]));

        std::string channelParam = "role=producer,type=NVSCI,id=40002,timeout=100000,fifo-size=4";
        channelParam += (std::string(",streamName=") + m_prodStreamNames);
        channelParam += (std::string(",reach=") + m_prodReaches);
        m_channelOutputImages = m_channelFactory->makeChannel(channelParam.c_str());
        // Pass the image properties to the channel, so that it can allocate images with the right properties and reconcile
        // requirements with connected consumers.
        // Also pass the sync attributes needed.
        // The second parameter is for signal sync attributes.
        // If p2c is enabled, set lambda to get sync attributes from the camera using dwSensorCamera_fillSyncAttributes
        // If p2c is disabled, set lambda to null.
        // The third parameter is for wait sync attributes.
        // Since c2p is not supported, this is set to null in any case.
        m_outputImagePorts = std::make_unique<dw::framework::PortOutput<PacketT>>(m_imageProperties[i], getProducerSyncAttrs(i, p2cEnabled()), getProducerSyncAttrs(i, c2pEnabled()));
        m_outputImagePorts->bindChannel(m_channelOutputImages.get());
        m_channelConnector->addChannel(m_channelOutputImages);
    }
    // Channel start connection thread
    m_channelConnector->start();
    // Break when: 1. Each producer connect at least one consumer. 2.All consumers are connected. Are both satisfied.
    while (!m_channelConnector->waitUntilConnected(0))
    {
        usleep(100);
    }

    for (uint32_t i = 0; i < m_totalCameras; ++i)
    {
        startCameraThread(i);
    }
}

std::vector<std::string> splitString(std::string str, char splitChar)
{
    std::vector<std::string> result{};
    std::istringstream ss(str);
    for (std::string line; std::getline(ss, line, splitChar);)
    {
        result.push_back(line);
    }
    return result;
}

//#######################################################################################
bool DwCGFCameraInterprocessApp::onInitialize()
{
    // -----------------------------------------
    // Initialize DriveWorks context and SAL
    // -----------------------------------------
    // initialize logger to print verbose message on console in color
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // initialize SDK context, using data folder
    dwContextParameters sdkParams = {};

    CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
    dwSAL_initialize(&m_sal, context);

    m_frames = atoi(getArgument("frames").c_str());

    if (getArgument("role").compare("producer") == 0)
    {
        m_isProducer = true;
    }

    m_prodReaches     = getArgument("prod-reaches");
    m_prodStreamNames = getArgument("prod-stream-names");
    m_consReaches     = splitString(getArgument("cons-reaches"), ':');
    m_consStreamNames = splitString(getArgument("cons-stream-names"), ':');
    m_syncMode        = parseSyncMode(getArgument("sync-mode"));
    m_waitISP         = m_syncMode == SyncMode::NONE;

    if (m_consReaches.size() != m_consStreamNames.size())
    {
        throw std::runtime_error("Size of cons-reaches does not match cons-stream-names. They must be equal");
    }
    m_hasProducer  = !m_prodReaches.empty();
    m_numConsumers = m_consReaches.size();

    // Create Factory and Connector
    m_channelFactory   = std::make_unique<dw::framework::DWChannelFactory>(context);
    m_channelConnector = std::make_unique<dw::framework::ChannelConnector>();

    if (m_syncMode != SyncMode::NONE)
    {
        CHECK_NVSCI_ERROR(NvSciSyncCpuWaitContextAlloc(m_channelFactory->getNvSciSyncModule(), &m_cpuWaitContext));
    }

    if (m_isProducer)
    {
        initCameras();
    }

    // Consumer proceed with channel connection
    if (!m_isProducer)
    {
        try
        {
            std::string hStr = getArgument("encode-height");
            std::string wStr = getArgument("encode-width");
            int32_t const h  = hStr.empty() ? 0 : std::stoi(hStr.c_str());
            int32_t const w  = wStr.empty() ? 0 : std::stoi(wStr.c_str());
            if (h > 0)
            {
                m_height = static_cast<uint32_t>(h);
            }
            if (w > 0)
            {
                m_width = static_cast<uint32_t>(w);
            }
        }
        catch (...)
        {
            throw std::runtime_error("invalid encode width or height");
        }

        onInitFrameCapture();

        dwImageProperties props{};
        props.memoryLayout = DW_IMAGE_MEMORY_TYPE_BLOCK;
        //should be received image size
        props.height = m_height;
        props.width  = m_width;
        props.format = DW_IMAGE_FORMAT_YUV420_UINT8_SEMIPLANAR;
        props.type   = DW_IMAGE_NVMEDIA;
        props.meta.flags |= DW_IMAGE_FLAGS_NVSCI_SURF_ATTR;
        CHECK_DW_ERROR(dwFrameCapture_appendAllocationAttributes(&props, m_frameCapture));

        std::string channelParam = "role=consumer,type=NVSCI,id=40002,timeout=100000,fifo-size=4";
        channelParam += (std::string(",streamName=") + m_consStreamNames[0]);
        channelParam += (std::string(",reach=") + m_consReaches[0]);
        m_channelInputImage = m_channelFactory->makeChannel(channelParam.c_str());

        // Pass lambdas to set consumer sync attributes to port
        // The first argument is the lambda to set signaler sync attributes.
        // If c2p is not currently supported, the returned lambda is null in any case corresponding to no sync.
        // The second argument is the lambda to set waiter sync attributes.
        // If p2c is enabled, the consumer sets attributes for cpu wait.
        // If p2c is disabled, the consumer sets lambda as null, which corresponds to no sync.
        m_inputImagePort = std::make_unique<dw::framework::PortInput<PacketT>>(props, getConsumerSyncAttrs(0, c2pEnabled()), getConsumerSyncAttrs(0, p2cEnabled()));
        m_inputImagePort->bindChannel(m_channelInputImage.get());
        m_channelConnector->addChannel(m_channelInputImage);

        // Channel start connection thread
        m_channelConnector->start();
        // Break when: 1. Each producer connect at least one consumer. 2.All consumers are connected. Are both satisfied.
        while (!m_channelConnector->waitUntilConnected(0))
        {
            usleep(100);
        }
    }
    return true;
}

//#######################################################################################
void DwCGFCameraInterprocessApp::onRender()
{
    // Consumer
    if (!m_isProducer)
    {
        dwTime_t tc;
        dwContext_getCurrentTime(&tc, context);

        if (m_counter >= m_frames)
        {
            stop();
            return;
        }
        {
            const auto& inputPort = m_inputImagePort;

            // wait for the next camera frame to arrive
            dwStatus ret = inputPort->wait(20'000'000);
            if (ret == DW_TIME_OUT)
            {
                std::cout << "Consumer[0] exit by timeout." << std::endl;
            }

            // receive the next camera frame
            std::shared_ptr<PacketT> inputPtr = inputPort->recv();
            if (inputPtr == nullptr)
            {
                throw std::runtime_error("Channel Consumer receive nullptr.");
            }

            NvSciSyncFence waitFence = NvSciSyncFenceInitializer;

            // if hardware producer to consumer sync enabled, wait on the fence
            // so that the operation to fill the received frame is complete
            if (p2cEnabled())
            {
                auto span = dw::core::make_span<NvSciSyncFence>(&waitFence, 1);
                inputPort->getWaitFences(inputPtr.get(), span);
                NvSciSyncFenceWait(&waitFence, m_cpuWaitContext, -1);
            }

            // queue encoding of the frame
            CHECK_DW_ERROR(dwFrameCapture_appendFrame(*inputPtr, m_frameCapture));

            //  Wait for the underlying frame to finish being encoded before returning
            waitNvMediaImage(*inputPtr);

            // release the frame back to producer
            inputPtr = nullptr;
        }
        m_counter++;
    }
}

//#######################################################################################
void DwCGFCameraInterprocessApp::onRelease()
{
    if (m_isProducer)
    {
        for (uint32_t i = 0; i < m_totalCameras; i++)
        {
            m_workers[i].join();
            m_senderWorkers[i].join();
            dwSensor_stop(m_camera[i]);
        }
    }
    else
    {
        dwFrameCapture_release(m_frameCapture);
    }

    m_channelFactory->stopServices();
    // -----------------------------------
    // Release SDK
    // -----------------------------------
    dwSAL_release(m_sal);
    dwRelease(context);
}

//#######################################################################################
int main(int argc, const char** argv)
{
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {
                              ProgramArguments::Option_t("rig", "", "rig path"),
                              ProgramArguments::Option_t("role", "consumer", "consumer or producer"),
                              ProgramArguments::Option_t("frames", "300", "frames to be processed then quit"),
                              ProgramArguments::Option_t("streamer", "0", "EGL/NvSci streamer name postfix indicators"),
                              ProgramArguments::Option_t("capture", "capture.h264", "file name to be captured, likes capture.h264"),
                              ProgramArguments::Option_t("bitrate", "8000000", "default bitrate to initialize dwFrameCapture module in consumer"),
                              ProgramArguments::Option_t("prod-stream-names", "", "colon-separated list of producer nvsciipc endpoints, for NVSCI mode only"),
                              ProgramArguments::Option_t("prod-reaches", "", "colon-separated list of producer reaches (process|chip), for NVSCI mode only"),
                              ProgramArguments::Option_t("cons-stream-names", "", "colon-separated list of consumer nvsciipc endpoints, for NVSCI mode only"),
                              ProgramArguments::Option_t("cons-reaches", "", "colon-separated list of consumer reaches (process|chip), for NVSCI mode only"),
                              ProgramArguments::Option_t("sync-mode", "p2c", "sync-mode between cons and prod"),
                              ProgramArguments::Option_t("encode-height", "2168", "encoded image height"),
                              ProgramArguments::Option_t("encode-width", "3848", "encoded image width"),
                          },
                          "Sample illustrating how to use an image streamer in a cross process scenario");

    // Window/GL based application
    bool offscreen = true;
    DwCGFCameraInterprocessApp app(args);
    app.initializeWindow("Image Streamer Cross Process Sample", 1280, 800, offscreen);
    return app.run();
}

// clang-format off
    #pragma GCC diagnostic pop
// clang-format on
