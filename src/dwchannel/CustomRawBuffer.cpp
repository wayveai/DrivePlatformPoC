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

#include "CustomRawBuffer.hpp"

#include <dwcgf/channel/IChannelPacket.hpp>
#include <dwcgf/channel/ChannelFactory.hpp>

#include <cuda.h>

#define CHECK_CUDA_ERROR(x)                          \
    do                                               \
    {                                                \
        cudaError_t res = (x);                       \
        if (res != cudaSuccess)                      \
        {                                            \
            throw std::runtime_error("Cuda error!"); \
        }                                            \
    } while (0);

#define CHECK_NVSCI_ERROR(x)                          \
    do                                                \
    {                                                 \
        NvSciError res = (x);                         \
        if (res != NvSciError_Success)                \
        {                                             \
            throw std::runtime_error("NvSci error!"); \
        }                                             \
    } while (0);

using IChannelPacket = dw::framework::IChannelPacket;
using GenericData    = dw::framework::GenericData;

class CustomRawBufferPacket : public IChannelPacket
{
public:
    CustomRawBufferPacket(GenericData specimen)
    {
        // GenericData is a generic pointer with some type information
        // to check if casting is safe. The getData<T>() checks the type information
        // before casting its pointer to T*, similar to dynamic_cast.
        CustomRawBuffer* customRawBuffer = specimen.getData<CustomRawBuffer>();

        // First check if the right type of data has been passed.
        if (customRawBuffer == nullptr)
        {
            throw std::runtime_error("Wrong specimen data type passed");
        }

        m_data = *customRawBuffer;

        if (m_data.memoryType == MemoryType::CPU)
        {
            m_buffer = malloc(m_data.capacity);
        }
        else
        {
            CHECK_CUDA_ERROR(cudaMalloc(&m_buffer, m_data.capacity));
        }
    }

    ~CustomRawBufferPacket()
    {
        if (m_data.memoryType == MemoryType::CPU)
        {
            free(m_buffer);
        }
        else
        {
            cudaFree(m_buffer);
        }
    }

    // implement method to get pointer to the payload of this packet
    GenericData getGenericData() override
    {
        return GenericData(&m_data);
    }

protected:
    CustomRawBuffer m_data{};
    void* m_buffer{};
};

class CustomRawBufferSocketPacket : public CustomRawBufferPacket, public IChannelPacket::SocketCallbacks
{
public:
    CustomRawBufferSocketPacket(GenericData data)
        : CustomRawBufferPacket(data)
        , m_serializedBufferSize{sizeof(m_data) + m_data.capacity}
        , m_serializedBuffer{new uint8_t[m_serializedBufferSize]}
    {
    }

    // Get the pointer to the serialized data buffer
    uint8_t* getBuffer() override
    {
        return m_serializedBuffer.get();
    }

    // Get the MAXIMUM size of the serialized data buffer
    size_t getBufferSize() override
    {
        return m_serializedBufferSize;
    }

    // Serialize the packet to internal buffer
    // return the actual serialized buffer size
    size_t serialize() override
    {
        memcpy(&m_serializedBuffer[0], &m_data, sizeof(m_data));

        if (m_data.memoryType == MemoryType::CPU)
        {
            memcpy(&m_serializedBuffer[sizeof(m_data)], m_buffer, m_data.size);
        }
        else
        {
            CHECK_CUDA_ERROR(cudaMemcpy(&m_serializedBuffer[sizeof(m_data)], m_buffer, m_data.size, cudaMemcpyDeviceToHost));
        }
        return sizeof(m_data) + m_data.size;
    }

    // Deserialize the packet from internal buffer
    void deserialize(size_t) override
    {
        memcpy(&m_data, &m_serializedBuffer[0], sizeof(m_data));
        m_data.buffer = m_buffer;
        if (m_data.memoryType == MemoryType::CPU)
        {
            memcpy(m_data.buffer, &m_serializedBuffer[sizeof(m_data)], m_data.size);
        }
        else
        {
            CHECK_CUDA_ERROR(cudaMemcpy(m_data.buffer, &m_serializedBuffer[sizeof(m_data)], m_data.size, cudaMemcpyHostToDevice));
        }
    }

private:
    size_t m_serializedBufferSize{};
    std::unique_ptr<uint8_t[]> m_serializedBuffer{};
};

class CustomRawBufferNvSciPacket : public IChannelPacket, public IChannelPacket::NvSciCallbacks
{
public:
    CustomRawBufferNvSciPacket(GenericData specimen)
    {
        // GenericData is a generic pointer with some type information
        // to check if casting is safe. The getData<T>() checks the type information
        // before casting its pointer to T*, similar to dynamic_cast.
        CustomRawBuffer* customRawBuffer = specimen.getData<CustomRawBuffer>();

        // First check if the right type of data has been passed.
        if (customRawBuffer == nullptr)
        {
            throw std::runtime_error("Wrong specimen data type passed");
        }

        m_data = *customRawBuffer;
    }

    ~CustomRawBufferNvSciPacket()
    {
        if (m_cudaHandle != nullptr)
        {
            cudaDestroyExternalMemory(m_cudaHandle);
        }
    }

    uint32_t getNumBuffers() const override
    {
        // return 2 as one NvSciBufObj will be used for the CustomRawBuffer struct itself
        // The second NvSciBufObj will be used for CustomRawBuffer::buffer
        return 2U;
    }

    void fillNvSciBufAttributes(uint32_t bufferIndex, NvSciBufAttrList& attrList) const override
    {
        if (bufferIndex == 0U)
        {
            // set the attributes for storing CustomRawBuffer struct itself, which is CPU memory.
            return setNvSciBufAttributes(attrList, sizeof(m_data), MemoryType::CPU);
        }
        else if (bufferIndex == 1U)
        {
            // set the attributes for the allocated buffer.
            return setNvSciBufAttributes(attrList, m_data.capacity, m_data.memoryType);
        }
        else
        {
            throw std::runtime_error("bufferIndex is out of bounds");
        }
    }

    void initializeFromNvSciBufObjs(dw::core::span<NvSciBufObj> bufs) override
    {
        if (bufs.size() != 2U)
        {
            throw std::runtime_error("Wrong number of bufs passed");
        }

        m_dataSharedBufObj = bufs[0];
        m_bufferPtrBufObj  = bufs[1];

        CHECK_NVSCI_ERROR(NvSciBufObjGetCpuPtr(m_dataSharedBufObj, reinterpret_cast<void**>(&m_dataShared)));

        if (m_data.memoryType == MemoryType::CPU)
        {
            CHECK_NVSCI_ERROR(NvSciBufObjGetCpuPtr(m_bufferPtrBufObj, &m_bufferPtrOrig));
        }
        else
        {
            cudaExternalMemoryHandleDesc cudaMemHandleDesc = {};
            cudaMemHandleDesc.type                         = cudaExternalMemoryHandleTypeNvSciBuf;
            cudaMemHandleDesc.handle.nvSciBufObject        = m_bufferPtrBufObj;
            cudaMemHandleDesc.size                         = m_data.capacity;
            cudaMemHandleDesc.flags                        = 0;
            CHECK_CUDA_ERROR(cudaImportExternalMemory(&m_cudaHandle, &cudaMemHandleDesc));

            cudaExternalMemoryBufferDesc cudaBufferDesc = {};
            memset(&cudaBufferDesc, 0, sizeof(cudaBufferDesc));
            cudaBufferDesc.size   = m_data.capacity;
            cudaBufferDesc.offset = 0;
            CHECK_CUDA_ERROR(cudaExternalMemoryGetMappedBuffer(&m_bufferPtrOrig, m_cudaHandle, &cudaBufferDesc));
        }
        m_data.buffer = m_bufferPtrOrig;
    }

    // implement method to get pointer to the payload of this packet
    GenericData getGenericData() override
    {
        return GenericData(&m_data);
    }

    void pack() override
    {
        *m_dataShared = m_data;
    }

    void unpack() override
    {
        m_data = *m_dataShared;

        // patch the pointer in case the struct was sent by another process,
        // the pointer may be different.
        m_data.buffer = m_bufferPtrOrig;
    }

private:
    void setNvSciBufAttributes(NvSciBufAttrList attrList, size_t size, MemoryType memoryType) const
    {
        const NvSciBufType bufferType               = NvSciBufType_RawBuffer;
        const NvSciBufAttrValAccessPerm permissions = NvSciBufAccessPerm_ReadWrite;
        const uint64_t rawSize                      = size;

        dw::core::Array<NvSciBufAttrKeyValuePair, 3> rawBufferAttributes =
            {{{NvSciBufGeneralAttrKey_Types, &bufferType, sizeof(bufferType)},
              {NvSciBufGeneralAttrKey_RequiredPerm, &permissions, sizeof(permissions)},
              {NvSciBufRawBufferAttrKey_Size, &rawSize, sizeof(rawSize)}}};

        CHECK_NVSCI_ERROR(NvSciBufAttrListSetAttrs(attrList,
                                                   rawBufferAttributes.data(),
                                                   rawBufferAttributes.size()));

        if (memoryType == MemoryType::CPU)
        {
            enableCpu(attrList);
        }
        else
        {
            enableCuda(attrList);
        }
    }

    void enableCuda(NvSciBufAttrList attrList) const
    {
        NvSciRmGpuId gpuIds[] = {{0}};
        int32_t cudaDevice;
        CHECK_CUDA_ERROR(cudaGetDevice(&cudaDevice));
        cudaDeviceProp deviceProps{};
        CHECK_CUDA_ERROR(cudaGetDeviceProperties(&deviceProps, cudaDevice));
        static_assert(sizeof(deviceProps.uuid) == sizeof(gpuIds[0]), "BufferCUDA: cuda uuid size does not match size of NvSciRmGpuId");
        memcpy(static_cast<void*>(&gpuIds[0]), static_cast<void*>(&deviceProps.uuid), sizeof(gpuIds[0]));

        dw::core::Array<NvSciBufAttrKeyValuePair, 1> rawBufferAttributes =
            {{{NvSciBufGeneralAttrKey_GpuId, &gpuIds, sizeof(gpuIds)}}};

        CHECK_NVSCI_ERROR(NvSciBufAttrListSetAttrs(attrList,
                                                   rawBufferAttributes.data(),
                                                   rawBufferAttributes.size()));
    }

    void enableCpu(NvSciBufAttrList attrList) const
    {
        const bool cpuAccessFlag = true;

        dw::core::Array<NvSciBufAttrKeyValuePair, 1> rawBufferAttributes =
            {{{NvSciBufGeneralAttrKey_NeedCpuAccess, &cpuAccessFlag, sizeof(cpuAccessFlag)}}};

        CHECK_NVSCI_ERROR(NvSciBufAttrListSetAttrs(attrList,
                                                   rawBufferAttributes.data(),
                                                   rawBufferAttributes.size()));
    }

    NvSciBufObj m_dataSharedBufObj{};
    NvSciBufObj m_bufferPtrBufObj{};
    CustomRawBuffer m_data{};
    CustomRawBuffer* m_dataShared{};
    void* m_bufferPtrOrig{};
    cudaExternalMemory_t m_cudaHandle{};
};

namespace
{
struct Proxy
{

    using ChannelPacketConstructor          = dw::framework::ChannelPacketConstructor;
    using ChannelPacketConstructorSignature = dw::framework::ChannelPacketConstructorSignature;
    using ChannelFactory                    = dw::framework::ChannelFactory;

    Proxy()
    {
        m_sigShem   = {CustomRawBufferTypeID, dw::framework::ChannelType::SHMEM_LOCAL};
        m_sigSocket = {CustomRawBufferTypeID, dw::framework::ChannelType::SOCKET};
        m_sigNvSci  = {CustomRawBufferTypeID, dw::framework::ChannelType::NVSCI};

        ChannelFactory::registerPacketConstructor(m_sigShem, ChannelPacketConstructor([](GenericData ref, dwContextHandle_t context) -> std::unique_ptr<IChannelPacket> {
                                                      static_cast<void>(context);
                                                      return std::make_unique<CustomRawBufferPacket>(ref);
                                                  }));
        ChannelFactory::registerPacketConstructor(m_sigSocket, ChannelPacketConstructor([](GenericData ref, dwContextHandle_t context) -> std::unique_ptr<IChannelPacket> {
                                                      static_cast<void>(context);
                                                      return std::make_unique<CustomRawBufferSocketPacket>(ref);
                                                  }));
        ChannelFactory::registerPacketConstructor(m_sigNvSci, ChannelPacketConstructor([](GenericData ref, dwContextHandle_t context) -> std::unique_ptr<IChannelPacket> {
                                                      static_cast<void>(context);
                                                      return std::make_unique<CustomRawBufferNvSciPacket>(ref);
                                                  }));
    }
    ~Proxy()
    {
        ChannelFactory::unregisterPacketConstructor(m_sigShem);
        ChannelFactory::unregisterPacketConstructor(m_sigSocket);
        ChannelFactory::unregisterPacketConstructor(m_sigNvSci);
    }

private:
    ChannelPacketConstructorSignature m_sigShem;
    ChannelPacketConstructorSignature m_sigSocket;
    ChannelPacketConstructorSignature m_sigNvSci;
};
static Proxy g_registerPacketConstructors;
} // namespace