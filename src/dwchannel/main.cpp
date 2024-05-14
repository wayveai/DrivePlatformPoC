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

#include <stdlib.h>
#include <thread>
#include <iostream>
#include <string>
#include <numeric>
#include <unistd.h>
#include <ctime>
#include <future>
#include <sstream>
#include <signal.h>

#include <framework/ProgramArguments.hpp>

#include <dw/core/context/Context.h>
#include <dw/core/base/Version.h>

#include <dwcgf/channel/ChannelFactory.hpp>
#include <dwcgf/channel/ChannelConnector.hpp>
#include <dwcgf/port/Port.hpp>

#include "ChannelPacketTypes.hpp"
#include "CustomRawBuffer.hpp"

#include <dwframework/dwnodes/common/factories/DWChannelFactory.hpp>
#include <framework/Log.hpp>

using namespace dw::framework;

static std::atomic_bool g_isRunning{true};

static void handleTerminate(int sig)
{
    std::cout << "Terminating on exit signal " << sig << std::endl;
    g_isRunning = false;
}

static constexpr size_t MAX_DOWNSTREAM_CONSUMERS = 64U;

static constexpr char LOG_TAG[] = "DWChannelSample";

#define CHECK_DW_ERROR(x)                                                                                                                                                                                \
    {                                                                                                                                                                                                    \
        dwStatus RESULT = x;                                                                                                                                                                             \
        if (RESULT != DW_SUCCESS)                                                                                                                                                                        \
        {                                                                                                                                                                                                \
            throw std::runtime_error(std::string("DW Error ") + dwGetStatusName(RESULT) + std::string(" executing DW function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
        }                                                                                                                                                                                                \
    };

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

// Define simple template for handling the supported data types
// of this sample app, either IntWithTimestamp, dwImage, or CustomRawPacket.
// That way the main app class can be re-used for each of the data types.
template <typename PacketT>
struct TypeCallbacks;

struct IntWithTimestamp
{
    int count;
    dwTime_t timestamp;
};
DWFRAMEWORK_DECLARE_PACKET_TYPE_POD(IntWithTimestamp);

template <>
struct TypeCallbacks<IntWithTimestamp>
{
    static IntWithTimestamp getSpecimen()
    {
        return {};
    }

    static size_t getSize()
    {
        return sizeof(IntWithTimestamp);
    }

    static dwTime_t getTimestamp(IntWithTimestamp data)
    {
        return data.timestamp;
    }

    static void setTimestamp(IntWithTimestamp& data, dwTime_t time)
    {
        data.timestamp = time;
    }
};

template <>
struct TypeCallbacks<dwImageHandle_t>
{
    static constexpr size_t IMAGE_WIDTH  = 3860;
    static constexpr size_t IMAGE_HEIGHT = 2160;

    static dwImageProperties getSpecimen()
    {
        // This can be changed to change the parameters of the images to
        // be streamed.
        dwImageProperties prop{.type   = DW_IMAGE_CUDA,
                               .width  = IMAGE_WIDTH,
                               .height = IMAGE_HEIGHT,
                               .format = DW_IMAGE_FORMAT_YUV420_UINT8_PLANAR};
        prop.memoryLayout = DW_IMAGE_MEMORY_TYPE_PITCH;

        return prop;
    }

    static size_t getSize()
    {
        auto prop          = getSpecimen();
        size_t elementSize = 0;
        size_t planeCount  = 0;

        uint32_t planeChannelCount[DW_MAX_IMAGE_PLANES] = {0};
        dwVector2ui planeSize[DW_MAX_IMAGE_PLANES]      = {0};

        size_t imageSize = 0;

        CHECK_DW_ERROR(dwImage_getDataLayout(&elementSize, &planeCount, planeChannelCount, planeSize, &prop));

        for (size_t i = 0; i < planeCount; i++)
        {
            imageSize += elementSize * planeSize[i].x * planeChannelCount[i] * planeSize[i].y;
        }

        return imageSize;
    }

    static dwTime_t getTimestamp(dwImageHandle_t img)
    {
        dwTime_t packetTime;
        CHECK_DW_ERROR(dwImage_getTimestamp(&packetTime, img));
        return packetTime;
    }

    static void setTimestamp(dwImageHandle_t img, dwTime_t time)
    {
        CHECK_DW_ERROR(dwImage_setTimestamp(time, img));
    }
};

template <>
struct TypeCallbacks<CustomRawBuffer>
{
    static CustomRawBuffer getSpecimen()
    {
        // This can be changed to change the properties of the CustomRawBuffer being streamed.
        CustomRawBuffer specimen{};
        specimen.capacity   = 256;
        specimen.memoryType = MemoryType::CPU;
        return specimen;
    }

    static size_t getSize()
    {
        return sizeof(CustomRawBuffer);
    }

    static dwTime_t getTimestamp(CustomRawBuffer& data)
    {
        return data.timestamp;
    }

    static void setTimestamp(CustomRawBuffer& data, dwTime_t time)
    {
        data.timestamp = time;
    }
};

static void setCpuWaiterAttributes(NvSciSyncAttrList attrList)
{
    const bool cpuAccess           = true;
    const NvSciSyncAccessPerm perm = NvSciSyncAccessPerm_WaitOnly;
    dw::core::Array<NvSciSyncAttrKeyValuePair, 2U> pairArray = {{{NvSciSyncAttrKey_NeedCpuAccess, &cpuAccess, sizeof(cpuAccess)},
                                                                 {NvSciSyncAttrKey_RequiredPerm, &perm, sizeof(perm)}}};
    CHECK_NVSCI_ERROR(NvSciSyncAttrListSetAttrs(attrList, &pairArray[0], pairArray.size()));
}

static void setCpuSignalerAttributes(NvSciSyncAttrList attrList)
{
    const bool cpuAccess           = true;
    const NvSciSyncAccessPerm perm = NvSciSyncAccessPerm_SignalOnly;
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

static constexpr size_t SKIP_FRAME_NUMBER = 0;
static constexpr size_t MAX_CONSUMER      = 4;
static constexpr size_t WAIT_TIMEOUT_US   = 10'000'000;

struct BaseChannelUserParams
{
    dwContextHandle_t context;
    uint32_t maxFrameNumber;
    SyncMode syncMode;
    std::string channelParam;
};

class BaseChannelUser
{
public:
    BaseChannelUser(ChannelFactory& channelFactory, BaseChannelUserParams params)
        : m_channelFactory(channelFactory)
        , m_params(std::move(params))
    {
        std::cout << "Creating channel with parameters: " << m_params.channelParam << std::endl;
        m_channel = m_channelFactory.makeChannel(m_params.channelParam.c_str());

        if (m_params.syncMode != SyncMode::NONE)
        {
            CHECK_NVSCI_ERROR(NvSciSyncCpuWaitContextAlloc(m_channelFactory.getNvSciSyncModule(), &m_cpuWaitContext));
        }
    }

    ~BaseChannelUser()
    {
        join();
    }

    void join()
    {
        if (m_thread.joinable())
        {
            m_thread.join();
        }
    }

    void runAsync()
    {
        start();
        m_thread = std::thread([this]() { run(); });
    }

    uint32_t getUID()
    {
        return m_channel->getParams().getUID();
    }

protected:
    virtual void run()
    {
    }

    virtual void start()
    {
    }

    dwTime_t getClockRealtime()
    {
        struct timespec time;
        clock_gettime(CLOCK_REALTIME, &time);
        dwTime_t result = time.tv_sec * 1000000000 + time.tv_nsec;
        return result;
    }

    dwTime_t getClockMonotonic()
    {
        struct timespec time;
        clock_gettime(CLOCK_MONOTONIC, &time);
        dwTime_t result = time.tv_sec * 1000000000 + time.tv_nsec;
        return result;
    }

    bool p2cEnabled()
    {
        return m_params.syncMode == SyncMode::BOTH || m_params.syncMode == SyncMode::P2C;
    }

    bool c2pEnabled()
    {
        return m_params.syncMode == SyncMode::BOTH || m_params.syncMode == SyncMode::C2P;
    }

    ChannelFactory& m_channelFactory;
    BaseChannelUserParams m_params{};
    dwTime_t m_elapsed{};
    std::shared_ptr<ChannelObject> m_channel{};
    NvSciSyncCpuWaitContext m_cpuWaitContext{};
    std::thread m_thread{};
    NvSciSyncObj m_signalerSyncObj{};
    NvSciSyncFence m_signalerSyncFence{};
    std::vector<NvSciSyncObj> m_waiterSyncObjs{};
    std::vector<NvSciSyncFence> m_waiterSyncFences{};
};

template <typename PacketT>
class ProducerChannelUser : public BaseChannelUser
{
public:
    using PacketTSpec = typename parameter_traits<PacketT>::SpecimenT;

    ProducerChannelUser(ChannelFactory& channelFactory, BaseChannelUserParams params)
        : BaseChannelUser(channelFactory, params)
    {
        PacketTSpec ref = TypeCallbacks<PacketT>::getSpecimen();
        m_outputPort    = std::make_unique<dw::framework::PortOutput<PacketT>>(ref, getCpuSignalerAttrs(), getCpuWaiterAttrs());
        CHECK_DW_ERROR(m_outputPort->bindChannel(this->m_channel.get()));
    }

    void start() override
    {
        if (this->p2cEnabled())
        {
            auto span = dw::core::make_span<NvSciSyncObj>(&this->m_signalerSyncObj, 1);
            m_outputPort->getSyncSignaler().getSyncObjs(span);
        }

        if (this->c2pEnabled())
        {
            this->m_waiterSyncObjs.resize(MAX_DOWNSTREAM_CONSUMERS, nullptr);
            this->m_waiterSyncFences.resize(MAX_DOWNSTREAM_CONSUMERS, NvSciSyncFenceInitializer);
            auto span = dw::core::make_span<NvSciSyncObj>(&this->m_waiterSyncObjs[0], MAX_DOWNSTREAM_CONSUMERS);
            m_outputPort->getSyncWaiter().getSyncObjs(span);
        }
    }

    void disconnectEndpoint(const char* nvsciipcEndpoint)
    {
        this->m_channel->disconnectEndpoint(nvsciipcEndpoint);
    }

    void connectEndpoint(const char* nvsciipcEndpoint)
    {
        this->m_channel->connectEndpoint(nvsciipcEndpoint);
    }

    void connect()
    {
        this->m_channel->connect(0);
    }

    void printStats()
    {
        std::cout << "Producer Send BandWidth: " << (((TypeCallbacks<PacketT>::getSize() * this->m_params.maxFrameNumber) / (1024 * 1024)) / (this->m_elapsed / 1000000000.0f))
                  << "MB/s" << std::endl;
    }

    void run() override
    {
        // get threadStart time for perf measurement
        dwTime_t threadStartTime = this->getClockMonotonic();

        std::future<void> future{};

        size_t currFrame = 0;
        while (g_isRunning && (currFrame < this->m_params.maxFrameNumber))
        {
            // get slot from channel memory pool
            dwStatus status = m_outputPort->wait(2000000);
            if (status != DW_SUCCESS)
            {
                throw std::runtime_error(std::string("Producer timeout when acquiring packet. ") + dwGetStatusName(status));
            }
            PacketT* outputFreePacket = m_outputPort->getFreeElement();

            if (outputFreePacket == nullptr)
            {
                throw std::runtime_error("Producer doesn't have enough empty slot.");
            }

            if (this->c2pEnabled())
            {
                // get wait fences
                auto span = dw::core::make_span<NvSciSyncFence>(&this->m_waiterSyncFences[0], this->m_waiterSyncFences.size());
                m_outputPort->getWaitFences(outputFreePacket, span);
                if (span.size() != this->m_waiterSyncFences.size())
                {
                    throw std::runtime_error("Producer wait fence span size mismatch");
                }
            }

            if (this->p2cEnabled())
            {
                // set signal fences
                NvSciSyncFence fence = NvSciSyncFenceInitializer;
                CHECK_NVSCI_ERROR(NvSciSyncObjGenerateFence(this->m_signalerSyncObj, &fence));
                auto span = dw::core::make_span<NvSciSyncFence>(&fence, 1);
                m_outputPort->setSignalFences(outputFreePacket, span);
            }

            auto work = [this, outputFreePacket]() {
                if (this->c2pEnabled())
                {
                    for (uint32_t i = 0U; i < this->m_waiterSyncFences.size(); i++)
                    {
                        CHECK_NVSCI_ERROR(NvSciSyncFenceWait(&this->m_waiterSyncFences[i], this->m_cpuWaitContext, -1));
                    }
                }

                usleep(10000); // sleep to emulate busy writing

                // get send time for perf measurement
                dwTime_t time = this->getClockRealtime();
                TypeCallbacks<PacketT>::setTimestamp(*outputFreePacket, time);

                if (this->p2cEnabled())
                {
                    // signal the sync object that frame is done
                    CHECK_NVSCI_ERROR(NvSciSyncObjSignal(this->m_signalerSyncObj));
                }
            };

            if (this->p2cEnabled())
            {
                if (future.valid())
                {
                    future.wait();
                }
                future = std::async(std::launch::async, work);
            }
            else
            {
                work(); // work must be completed before sending
            }

            // send data from memory slot
            CHECK_DW_ERROR(m_outputPort->send(outputFreePacket));
            currFrame++;
        }

        dwTime_t threadEndTime = this->getClockMonotonic();
        this->m_elapsed        = threadEndTime - threadStartTime;

        std::cout << "Producer uid: " << this->getUID() << " done" << std::endl;
    }

private:
    dw::framework::OnSetSyncAttrs getCpuWaiterAttrs()
    {
        if (m_params.syncMode == SyncMode::BOTH || m_params.syncMode == SyncMode::C2P)
        {
            return [](NvSciSyncAttrList attrList) {
                setCpuWaiterAttributes(attrList);
            };
        }
        return {};
    }

    dw::framework::OnSetSyncAttrs getCpuSignalerAttrs()
    {
        if (m_params.syncMode == SyncMode::BOTH || m_params.syncMode == SyncMode::P2C)
        {
            return [this](NvSciSyncAttrList attrList) {
                setCpuSignalerAttributes(attrList);
            };
        }
        return {};
    }

    std::unique_ptr<dw::framework::PortOutput<PacketT>> m_outputPort{};
};

template <typename PacketT>
class ConsumerChannelUser : public BaseChannelUser
{
public:
    using PacketTSpec = typename parameter_traits<PacketT>::SpecimenT;

    ConsumerChannelUser(ChannelFactory& channelFactory, BaseChannelUserParams params)
        : BaseChannelUser(channelFactory, params)
    {
        PacketTSpec ref = TypeCallbacks<PacketT>::getSpecimen();
        m_inputPort     = std::make_unique<dw::framework::PortInput<PacketT>>(ref, getCpuWaiterAttrs(), getCpuSignalerAttrs());
        CHECK_DW_ERROR(m_inputPort->bindChannel(this->m_channel.get()));
    }

    void start() override
    {
        if (this->p2cEnabled())
        {
            this->m_waiterSyncObjs.push_back(nullptr);
            auto span = dw::core::make_span<NvSciSyncObj>(&this->m_waiterSyncObjs.back(), 1);
            m_inputPort->getSyncWaiter().getSyncObjs(span);
        }

        if (this->c2pEnabled())
        {
            auto span = dw::core::make_span<NvSciSyncObj>(&this->m_signalerSyncObj, 1);
            m_inputPort->getSyncSignaler().getSyncObjs(span);
        }
    }

    void printStats()
    {
        dwTime_t accumulateLatency = 0;
        if (m_sendLatencyArr.size() != this->m_params.maxFrameNumber)
        {
            std::cout << m_sendLatencyArr.size() << std::endl;
            std::cout << "Consumer Idx[" << this->getUID() << "] Packets Number doesn't match with number of frames, use mailbox mode ? " << std::endl;
        }

        if (m_sendLatencyArr.size() <= SKIP_FRAME_NUMBER)
        {
            return;
        }

        accumulateLatency += std::accumulate(m_sendLatencyArr.begin() + SKIP_FRAME_NUMBER, m_sendLatencyArr.end(), 0);
        std::cout << "Consumer Idx[" << this->getUID() << "] Latency: " << (accumulateLatency / (this->m_params.maxFrameNumber - SKIP_FRAME_NUMBER)) << "us" << std::endl;
        std::cout << "Consumer Idx[" << this->getUID() << "] Recv BandWidth: " << (((TypeCallbacks<PacketT>::getSize() * m_sendLatencyArr.size()) / (1024 * 1024)) / (this->m_elapsed / 1000000000.0f)) << "MB/s" << std::endl;
    }

    void run() override
    {
        // make sure the port is bound to a channel
        if (!m_inputPort->isBound())
        {
            std::cout << "Consumer[" << this->getUID() << "] doesn't bind port." << std::endl;
            return;
        }

        // get threadStart time for perf measurement
        dwTime_t threadStartTime = this->getClockMonotonic();

        std::future<void> future{};

        size_t currFrame = 0;
        while (g_isRunning && (currFrame < this->m_params.maxFrameNumber))
        {
            // recv slot
            dwStatus ret = m_inputPort->wait(WAIT_TIMEOUT_US);
            if (ret == DW_TIME_OUT)
            {
                std::cout << "Consumer[" << this->getUID() << "] exit by timeout." << std::endl;
                break;
            }

            std::shared_ptr<PacketT> inputPtr = m_inputPort->recv();

            if (inputPtr == nullptr)
            {
                std::cout << "Consumer[" << this->getUID() << "] recv failure, exiting" << std::endl;
                break;
            }

            NvSciSyncFence waitFence = NvSciSyncFenceInitializer;
            if (this->p2cEnabled())
            {
                auto span = dw::core::make_span<NvSciSyncFence>(&waitFence, 1);
                m_inputPort->getWaitFences(inputPtr.get(), span);
            }

            if (this->c2pEnabled())
            {
                // set signal fence
                NvSciSyncFence fence = NvSciSyncFenceInitializer;
                CHECK_NVSCI_ERROR(NvSciSyncObjGenerateFence(this->m_signalerSyncObj, &fence));
                auto span = dw::core::make_span<NvSciSyncFence>(&fence, 1);
                m_inputPort->setSignalFences(inputPtr.get(), span);
            }

            auto work = [ this, inputPacket = inputPtr.get(), waitFence ]()
            {
                if (this->p2cEnabled())
                {
                    CHECK_NVSCI_ERROR(NvSciSyncFenceWait(&waitFence, this->m_cpuWaitContext, -1));
                }

                // collect receive latency
                dwTime_t packetTime = TypeCallbacks<PacketT>::getTimestamp(*inputPacket);
                dwTime_t current    = this->getClockRealtime();
                m_sendLatencyArr.push_back((current - packetTime) / 1000);

                if (this->c2pEnabled())
                {
                    // signal the sync object that frame is done
                    CHECK_NVSCI_ERROR(NvSciSyncObjSignal(this->m_signalerSyncObj));
                }
            };

            if (this->c2pEnabled())
            {
                if (future.valid())
                {
                    future.wait();
                }
                future = std::async(std::launch::async, work);
            }
            else
            {
                work(); // must wait for work to complete before we return packet.
            }

            // release slot
            inputPtr = nullptr;
            currFrame++;
        }

        dwTime_t threadEndTime = this->getClockMonotonic();
        this->m_elapsed        = threadEndTime - threadStartTime;

        std::cout << "Consumer uid: " << this->getUID() << " done" << std::endl;
    }

private:
    dw::framework::OnSetSyncAttrs getCpuWaiterAttrs()
    {
        if (m_params.syncMode == SyncMode::BOTH || m_params.syncMode == SyncMode::P2C)
        {
            return [](NvSciSyncAttrList attrList) {
                setCpuWaiterAttributes(attrList);
            };
        }
        return {};
    }

    dw::framework::OnSetSyncAttrs getCpuSignalerAttrs()
    {
        if (m_params.syncMode == SyncMode::BOTH || m_params.syncMode == SyncMode::C2P)
        {
            return [](NvSciSyncAttrList attrList) {
                setCpuSignalerAttributes(attrList);
            };
        }
        return {};
    }

    std::vector<dwTime_t> m_sendLatencyArr{};
    std::unique_ptr<dw::framework::PortInput<PacketT>> m_inputPort{};
};

template <typename PacketT>
class DWChannelSample
{
    dwContextHandle_t m_context = DW_NULL_HANDLE;
    dwImageProperties m_props{};

    bool m_hasProducer = true;

    uint32_t m_numConsumers = 1;
    uint32_t m_downStreams  = 1;

    uint32_t m_maxFrameNumber = 128;
    SyncMode m_syncMode       = SyncMode::NONE;

    std::string m_portId{};
    std::string m_ipAddr{};
    std::string m_type{};
    std::string m_channelMode{};
    std::string m_prodReaches{};
    std::string m_prodStreamNames{};
    std::vector<std::string> m_consReaches{};
    std::vector<std::string> m_consStreamNames{};
    std::vector<std::string> m_lateAttaches{};
    std::string m_lateAttachLocations{};
    dwTime_t m_prod_timeout{};
    dwTime_t m_cons_timeout{};

    bool m_isMailBox   = false;
    bool m_isReuse     = false;
    int32_t m_fifoSize = -1;
    uint32_t m_numLocalConsumers{};
    bool m_isInteractive{};
    bool m_greedyReattach{};

    std::thread m_connectionThread{};
    std::thread m_userInputThread{};
    std::atomic_bool m_started{};
    std::mutex m_mutex{};
    std::condition_variable m_cv{};
    bool m_connected{};

    // Channel factory
    std::unique_ptr<ChannelFactory> m_channelFactory{};

    // Producer
    std::unique_ptr<ProducerChannelUser<PacketT>> m_producer{};

    // Consumers
    dw::core::VectorFixed<std::unique_ptr<ConsumerChannelUser<PacketT>>, MAX_CONSUMER> m_consumers{};

    ProgramArguments m_args{};

    const std::string& getArgument(const char* name) const
    {
        return m_args.get(name);
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

    SyncMode parseSyncMode(std::string syncMode)
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

    void parseAndCheckArguments()
    {
        // parse and check channel type
        m_type = getArgument("type");
        if (m_type == "NVSCI")
        {
            m_prodReaches         = getArgument("prod-reaches");
            m_prodStreamNames     = getArgument("prod-stream-names");
            m_consReaches         = splitString(getArgument("cons-reaches"), ':');
            m_consStreamNames     = splitString(getArgument("cons-stream-names"), ':');
            m_lateAttaches        = splitString(getArgument("late-attach"), ':');
            m_numLocalConsumers   = std::atoi(getArgument("num-local-consumers").c_str());
            m_greedyReattach      = std::atoi(getArgument("greedy-reattach").c_str());
            m_lateAttachLocations = getArgument("late-attach-locations");
            if (m_consReaches.size() != m_consStreamNames.size())
            {
                throw std::runtime_error("Size of cons-reaches does not match cons-stream-names. They must be equal");
            }
            m_hasProducer  = !m_prodReaches.empty() || m_numLocalConsumers > 0 || m_lateAttaches.size() > 0;
            m_numConsumers = m_consReaches.size() + m_numLocalConsumers;
        }
        else
        {
            uint32_t prod = std::atoi(getArgument("prod").c_str());
            if (prod > 1)
            {
                throw std::runtime_error("Number of Producer must be 0 or 1.");
            }

            // parse and check producer with downstream.
            m_hasProducer = static_cast<bool>(prod);
            m_downStreams = std::atoi(getArgument("downstreams").c_str());
            if ((m_hasProducer && m_downStreams == 0) || (!m_hasProducer && m_downStreams >= 1))
            {
                throw std::runtime_error("downstreams argument must with prod.");
            }

            // parse and check number of consumer.
            m_numConsumers = std::atoi(getArgument("cons").c_str());
            if (m_hasProducer && (m_numConsumers > m_downStreams))
            {
                throw std::runtime_error("Number of consumers cannot exclude downStreams.");
            }
            if (m_numConsumers > MAX_CONSUMER)
            {
                throw std::runtime_error("Number of consumers cannot exclude to maximum number of consumer this sample.");
            }

            if (m_type == "SHMEM_LOCAL")
            {
                if (m_numConsumers != m_downStreams)
                {
                    throw std::runtime_error("Number of downStreams (downstream) and Number of Consumers (cons) must equal when use SHMEM, as consumers and producers are in a same process.");
                }
            }
            else if (m_type != "SOCKET")
            {
                throw std::runtime_error("Invalid ChannelType.");
            }

            // convert from ms to us.
            m_prod_timeout = std::stoul(getArgument("prod-timeout")) * 1000;
            m_cons_timeout = std::stoul(getArgument("cons-timeout")) * 1000;
        }

        // parse and check channel mode
        m_channelMode = getArgument("mode");
        if (m_channelMode == "mailbox")
        {
            m_isMailBox = true;
        }
        else if (m_channelMode == "reuse")
        {
            m_isMailBox = true;
            m_isReuse   = true;
        }
        else if (std::atoi(m_channelMode.c_str()) > 0)
        {
            m_fifoSize = std::atoi(m_channelMode.c_str());
        }
        else
        {
            throw std::runtime_error("Invalid ChannelMode");
        }

        m_ipAddr = getArgument("ip");
        m_portId = getArgument("port");

        m_maxFrameNumber = atoi(getArgument("frames").c_str());

        m_syncMode = parseSyncMode(getArgument("sync-mode"));

        m_isInteractive = std::atoi(getArgument("interactive").c_str());

        return;
    }

    void initChannels()
    {
        // Create Factory and Connector
        m_channelFactory = std::make_unique<dw::framework::DWChannelFactory>(m_context);

        BaseChannelUserParams params{};
        params.context        = m_context;
        params.maxFrameNumber = m_maxFrameNumber;
        params.syncMode       = m_syncMode;

        uint32_t uid = 1U;

        if (m_hasProducer)
        {
            std::stringstream channelParam{};
            channelParam << "role=producer"
                         << ",type=" << m_type
                         << ",id=" << m_portId
                         << ",fifo-size=10";
            if (m_type == "NVSCI")
            {
                if (!m_prodStreamNames.empty())
                {
                    channelParam << ",streamName=" << m_prodStreamNames;
                    channelParam << ",reach=" << m_prodReaches;
                }
                channelParam << ",num-clients=" << m_lateAttaches.size() + m_numLocalConsumers;
                if (!m_lateAttachLocations.empty())
                {
                    channelParam << ",late-locs=" << m_lateAttachLocations << std::endl;
                }
            }
            else if (m_type == "SOCKET")
            {
                channelParam << ",ip=" << m_ipAddr;
                channelParam << ",num-clients=" << getArgument("downstreams");
                channelParam << ",producer-fifo=1";
                channelParam << ",connect-timeout=" << m_prod_timeout;
            }
            params.channelParam = channelParam.str();
            m_producer          = std::make_unique<ProducerChannelUser<PacketT>>(*m_channelFactory, params);
        }

        for (size_t consId = 0; consId < m_numConsumers; consId++)
        {
            std::stringstream channelParam;
            channelParam << "role=consumer"
                         << ",type=" + m_type
                         << ",ip=" + m_ipAddr
                         << ",id=" + m_portId
                         << ",uid=" << uid;
            if (m_isMailBox)
            {
                channelParam << ",mode=mailbox";
                if (m_isReuse)
                {
                    channelParam << ",reuse=true";
                }
            }
            else
            {
                channelParam << ",fifo-size=" << m_channelMode;
            }
            if (m_type == "NVSCI")
            {
                if (consId < m_consStreamNames.size())
                {
                    channelParam << ",streamName=" << m_consStreamNames[consId];
                    channelParam << ",reach=" << m_consReaches[consId];
                }
            }
            else if (m_type == "SOCKET")
            {
                channelParam << ",connect-timeout=" << m_cons_timeout;
            }
            params.channelParam = channelParam.str();
            m_consumers.push_back(nullptr);
            m_consumers.back() = std::make_unique<ConsumerChannelUser<PacketT>>(*m_channelFactory, params);
        }
    }

public:
    DWChannelSample(const ProgramArguments& args)
        : m_args(args)
    {
    }

    bool initialize()
    {
        CHECK_DW_ERROR(dwInitialize(&m_context, DW_VERSION, nullptr));
        parseAndCheckArguments();
        initChannels();
        return true;
    }

    void runUserInput()
    {
        if (!m_isInteractive)
        {
            m_started = true;
            m_cv.notify_all();
            return;
        }
        std::cout << "This sample app accepts the following user inputs:" << std::endl;
        std::cout << "'c' sends command to producer to late-connect or re-connect" << std::endl;
        std::cout << "'c <nvsciipc endpoint name> sends command to producer to add the given nvsciipc endpoint for connection" << std::endl;
        std::cout << "'d <nvsciipc endpoint name> sends command to producer to disconnect given nvsciipc endpoint" << std::endl;

        std::cout << "Press enter to start" << std::endl;
        std::string cmd;
        std::getline(std::cin, cmd);
        m_started = true;
        m_cv.notify_all();
        if (!m_hasProducer)
        {
            std::cout << "This instance has no producer so no commands are accepted" << std::endl;
            return;
        }
        while (g_isRunning)
        {
            std::cout << "Please input next command:" << std::endl;
            std::getline(std::cin, cmd);
            if (cmd == "")
            {
                continue;
            }
            else if (cmd == "c")
            {
                std::cout << "Sending connect command to producer" << std::endl;
                m_producer->connect();
            }
            else if (cmd.find("c ") == 0)
            {
                std::string target = cmd.substr(2);
                std::cout << "Sending connect command to producer for nvsciipc endpoint: " << target << std::endl;
                m_producer->connectEndpoint(target.c_str());
            }
            else if (cmd.find("d ") == 0)
            {
                std::string target = cmd.substr(2);
                std::cout << "Sending disconnect command to producer for nvsciipc endpoint: " << target << std::endl;
                m_producer->disconnectEndpoint(target.c_str());
            }
            else
            {
                std::cout << "Command unrecognized" << std::endl;
            }
        }
    }

    void runConnection()
    {
        std::cout << "Starting channel connection" << std::endl;
        m_channelFactory->startServices();
        while (g_isRunning)
        {
            dw::core::Optional<ChannelEvent> channelEvent = m_channelFactory->popEvent(10'000);
            if (!channelEvent.has_value())
            {
                continue;
            }
            switch (channelEvent.value().type)
            {
            case ChannelEventType::ERROR:
            {
                std::cerr << "An error was reported by the channel with uid:" << channelEvent.value().uid << std::endl;
                dw::core::Optional<ChannelError> error = m_channelFactory->popError();
                uint32_t uid                           = channelEvent.value().uid;
                dwStatus status                        = static_cast<dwStatus>(error.value().errorSignal.errorIDs[1]);
                std::cerr << "The error status is " << dwGetStatusName(status) << std::endl;
                if (status != DW_SUCCESS && status != DW_END_OF_STREAM)
                {
                    g_isRunning = false;
                }
                else if (!error.value().nvsciipcEndpoint.empty())
                {
                    std::cerr << "The nvsciipc endpoint is " << error.value().nvsciipcEndpoint.c_str() << std::endl;
                    std::cerr << "Taking corrective action to disconnect the endpoint" << std::endl;
                    if (uid == m_producer->getUID())
                    {
                        m_producer->disconnectEndpoint(error.value().nvsciipcEndpoint.c_str());
                        if (m_greedyReattach)
                        {
                            std::cout << "Greedy reattachment enabled, attempting to automatically reconnect" << std::endl;
                            std::cout << "Warning: once late/reconnection is started, other endpoints will not be able to"
                                      << " be connected until this endpoint has finished being late/re-connected." << std::endl;
                            m_producer->connectEndpoint(error.value().nvsciipcEndpoint.c_str());
                            m_producer->connect();
                        }
                    }
                }
                if (!m_connected)
                {
                    std::cerr << "Error happened before initial connection succeeded, shutting down" << std::endl;
                    g_isRunning = false;
                }
            }
            break;
            case ChannelEventType::CONNECTED:
                if (m_hasProducer && m_connected && (m_producer->getUID() == channelEvent.value().uid))
                {
                    std::cout << "Producer late connection / re-connection succeeded" << std::endl;
                }
                break;
            case ChannelEventType::DISCONNECTED:
            case ChannelEventType::READY:
                break;
            case ChannelEventType::GROUP_CONNECTED:
            {
                std::cout << "All channels connected!" << std::endl;
                m_connected = true;
                m_cv.notify_all();

                if (m_lateAttaches.size() > 0U)
                {
                    std::cout << "Initializing late connections" << std::endl;
                    for (auto& str : m_lateAttaches)
                    {
                        m_producer->connectEndpoint(str.c_str());
                    }
                    m_producer->connect();
                }
                break;
            }
            default:
                throw std::runtime_error("Unknown channel event type!");
                break;
            }
        }
        m_channelFactory->stopServices();
    }

    void process()
    {
        m_userInputThread = std::thread(&DWChannelSample::runUserInput, this);
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this]() { return m_started || !g_isRunning; });
            if (m_started && g_isRunning)
            {
                m_connectionThread = std::thread(&DWChannelSample::runConnection, this);
            }
        }

        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [this]() { return m_connected || !g_isRunning; });

        struct sigaction sa
        {
        };
        sa.sa_handler = handleTerminate;
        sigaction(SIGTERM, &sa, nullptr);
        sigaction(SIGINT, &sa, nullptr);

        if (m_hasProducer)
        {
            m_producer->runAsync();
        }

        for (size_t consId = 0; consId < m_numConsumers; consId++)
        {
            m_consumers[consId]->runAsync();
        }

        if (m_hasProducer)
        {
            m_producer->join();
        }

        for (size_t consId = 0; consId < m_numConsumers; consId++)
        {
            m_consumers[consId]->join();
        }

        g_isRunning = false;
        m_connectionThread.join();
        m_userInputThread.join();

        if (m_isReuse)
        {
            return;
        }

        if (m_hasProducer)
        {
            m_producer->printStats();
        }

        for (size_t consId = 0; consId < m_numConsumers; ++consId)
        {
            m_consumers[consId]->printStats();
        }
    }

    void release()
    {
        if (m_hasProducer)
        {
            m_producer.reset();
        }

        for (size_t consId = 0; consId < m_numConsumers; consId++)
        {
            m_consumers[consId].reset();
        }

        m_channelFactory.reset();

        dwRelease(m_context);
    }
};

int main(int argc, const char** argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
                          {ProgramArguments::Option_t("prod", "1", "Have a producer in this process (0/1)."),
                           ProgramArguments::Option_t("downstreams", "1", "Number of the producer's downstreams [1,4]."),
                           ProgramArguments::Option_t("cons", "1", "Number of consumers in this process [0,4]"),
                           ProgramArguments::Option_t("ip", "127.0.0.1", "Source IP Address"),
                           ProgramArguments::Option_t("prod-stream-names", "", "colon-separated list of producer nvsciipc endpoints, for NVSCI mode only"),
                           ProgramArguments::Option_t("prod-reaches", "", "colon-separated list of producer reaches (process|chip), for NVSCI mode only"),
                           ProgramArguments::Option_t("prod-timeout", "1000", "set the time-out value for producer, for Socket mode only"),
                           ProgramArguments::Option_t("cons-stream-names", "", "colon-separated list of consumer nvsciipc endpoints, for NVSCI mode only"),
                           ProgramArguments::Option_t("cons-reaches", "", "colon-separated list of consumer reaches (process|chip), for NVSCI mode only"),
                           ProgramArguments::Option_t("cons-timeout", "100", "set the time-out value for consumer, for Socket mode only"),
                           ProgramArguments::Option_t("sync-mode", "none", "Type of synchronization (none|p2c|c2p|both), for NVSCI mode only"),
                           ProgramArguments::Option_t("dataType", "dwImage", "the type of data to be transfered (int|dwImage|custom)"),
                           ProgramArguments::Option_t("port", "40002", "PortId"),
                           ProgramArguments::Option_t("mode", "10", "ChannelMode (mailbox/reuse/[N])"),
                           ProgramArguments::Option_t("type", "SOCKET", "ChannelType (SOCKET/SHMEM_LOCAL/NVSCI)"),
                           ProgramArguments::Option_t("frames", "128", "number of frames to run the sample"),
                           ProgramArguments::Option_t("interactive", "0", "run in interactive mode"),
                           ProgramArguments::Option_t("num-local-consumers", "0", "number of local NVSCI consumers, for NVSCI mode only"),
                           ProgramArguments::Option_t("late-attach-locations", "", "colon separated list of SOC-ID.VM-ID tuples, for example '0.0:1.0', for NVSCI mode only"),
                           ProgramArguments::Option_t("late-attach", "", "colon-separated list of producer nvsciipc endpoints, for NVSCI mode only"),
                           ProgramArguments::Option_t("greedy-reattach", "0", "greedily reattach any disconnected consumer nvsciipc endpoints, for NVSCI mode only"),
                           ProgramArguments::Option_t("loglevel", "DW_LOG_WARN", "The log level")},
                          "DW Channel Sample.");

    std::string loglevel        = args.get("loglevel");
    dwLoggerVerbosity verbosity = DW_LOG_WARN;
    if (loglevel == "DW_LOG_VERBOSE")
    {
        verbosity = DW_LOG_VERBOSE;
    }
    else if (loglevel == "DW_LOG_DEBUG")
    {
        verbosity = DW_LOG_DEBUG;
    }
    else if (loglevel == "DW_LOG_WARN")
    {
        verbosity = DW_LOG_WARN;
    }
    else if (loglevel == "DW_LOG_ERROR")
    {
        verbosity = DW_LOG_ERROR;
    }

    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(verbosity);

    auto runApp = [](auto& app) {
        app.initialize();
        app.process();
        app.release();
    };

    auto dataType = args.get("dataType");
    if (dataType == "dwImage")
    {
        DWChannelSample<dwImageHandle_t> app(args);
        runApp(app);
    }
    else if (dataType == "int")
    {
        DWChannelSample<IntWithTimestamp> app(args);
        runApp(app);
    }
    else if (dataType == "custom")
    {
        DWChannelSample<CustomRawBuffer> app(args);
        runApp(app);
    }
    else
    {
        throw std::runtime_error("dataType arg is not one of (dwImage|int|custom)");
    }

    return 0;
}
