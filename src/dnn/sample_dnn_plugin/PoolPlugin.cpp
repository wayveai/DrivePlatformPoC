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

#include <dw/dnn/plugin/DNNPlugin.h>

#include <cudnn.h>
#include <assert.h>
#include <memory>
#include <map>
#include <algorithm>
#include <string>
#include <ctime>

// Class for implementing Pooling layer in MNIST
class PoolPlugin
{
public:
#define CHECK_CUDA_ERROR(x)                                                                                                                                                                                                        \
    {                                                                                                                                                                                                                              \
        x;                                                                                                                                                                                                                         \
        auto result = cudaGetLastError();                                                                                                                                                                                          \
        if (result != cudaSuccess)                                                                                                                                                                                                 \
        {                                                                                                                                                                                                                          \
            char buf[80];                                                                                                                                                                                                          \
            getDateString(buf, 80);                                                                                                                                                                                                \
            throw std::runtime_error(std::string(buf) + std::string("CUDA Error ") + cudaGetErrorString(result) + std::string(" executing CUDA function:\n " #x) + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
        }                                                                                                                                                                                                                          \
    };

    struct PoolingParams
    {
        int32_t numChannels;
        int32_t inputHeight;
        int32_t inputWidth;
        int32_t outputHeight;
        int32_t outputWidth;
        int32_t kernelWidth;
        int32_t kernelHeight;
        int32_t strideX;
        int32_t strideY;
        int32_t paddingX;
        int32_t paddingY;
        cudnnPoolingMode_t poolingMode;
    };

    PoolPlugin()                  = default;
    PoolPlugin(const PoolPlugin&) = default;

    void deserializeFromFieldCollections(const char8_t* name,
                                         const dwDNNPluginFieldCollection& fieldCollection)
    {
        static_cast<void>(name);
        static_cast<void>(fieldCollection);
        m_poolingParams.strideX      = 2;
        m_poolingParams.strideY      = 2;
        m_poolingParams.kernelWidth  = 2;
        m_poolingParams.kernelHeight = 2;
        m_poolingParams.paddingX     = 0;
        m_poolingParams.paddingY     = 0;
        m_poolingParams.poolingMode  = CUDNN_POOLING_MAX;
    }

    // create the plugin at runtime from a byte stream
    void deserializeFromBuffer(const char8_t* name, const void* data, size_t length)
    {
        static_cast<void>(name);

        const char8_t* dataBegin = reinterpret_cast<const char8_t*>(data);
        const char8_t* dataItr   = dataBegin;
        m_poolingParams          = read<PoolingParams>(dataBegin, length, dataItr);
        m_inputBlobSize          = read<dwBlobSize>(dataBegin, length, dataItr);
        m_outputBlobSize         = read<dwBlobSize>(dataBegin, length, dataItr);
        m_precision              = read<dwPrecision>(dataBegin, length, dataItr);
    }

    void deserializeFromWeights(const dwDNNPluginWeights* weights, int32_t numWeights)
    {
        static_cast<void>(weights);
        static_cast<void>(numWeights);
        m_poolingParams.strideX      = 2;
        m_poolingParams.strideY      = 2;
        m_poolingParams.kernelWidth  = 2;
        m_poolingParams.kernelHeight = 2;
        m_poolingParams.paddingX     = 0;
        m_poolingParams.paddingY     = 0;
        m_poolingParams.poolingMode  = CUDNN_POOLING_MAX;
    }

    int32_t getNbOutputs() const
    {
        return 1;
    }

    dwBlobSize getOutputDimensions(int32_t index, const dwBlobSize* inputs, int32_t numInputDims)
    {
        if (numInputDims != 1 || inputs == nullptr || index > 0)
        {
            throw std::runtime_error("PoolPlugin::getOutputDimensions - expects only 1 input.");
        }

        const dwBlobSize& inputDimensions = inputs[0];
        dwBlobSize outputDimensions{};

        outputDimensions.width = (inputDimensions.width + m_poolingParams.paddingX * 2 -
                                  m_poolingParams.kernelWidth) /
                                     m_poolingParams.strideX +
                                 1;

        outputDimensions.height = (inputDimensions.height + m_poolingParams.paddingY * 2 -
                                   m_poolingParams.kernelHeight) /
                                      m_poolingParams.strideY +
                                  1;
        outputDimensions.channels  = inputDimensions.channels;
        outputDimensions.batchsize = inputDimensions.batchsize;
        return outputDimensions;
    }

    bool supportsFormatCombination(int32_t index, const dwDNNPluginTensorDesc* inOut,
                                   int32_t numInputs, int32_t numOutputs) const
    {
        if (numInputs != 1 || numOutputs != 1 || index >= (numInputs + numOutputs))
        {
            throw std::runtime_error("PoolPlugin::supportsFormatCombination - Unexpected number of inputs/outputs.");
        }

        bool condition = inOut[index].layout == DW_DNN_PLUGIN_LAYOUT_LINEAR;
        condition &= inOut[index].precision == DW_PRECISION_FP16 ||
                     inOut[index].precision == DW_PRECISION_FP32;
        condition &= inOut[index].precision == inOut[index].precision; // Check that all precisions are the same.
        return condition;
    }

    void configurePlugin(const dwDNNPluginTensorDesc* inputDescs, int32_t numInputs,
                         const dwDNNPluginTensorDesc* outputDescs, int32_t numOutputs)
    {
        if (numInputs != 1 || numOutputs != 1)
        {
            throw std::runtime_error("PoolPlugin::configurePlugin - Unexpected number of inputs/outputs.");
        }
        m_precision                  = inputDescs[0].precision;
        m_inputBlobSize              = inputDescs[0].dims;
        m_outputBlobSize             = outputDescs[0].dims;
        m_poolingParams.numChannels  = m_inputBlobSize.channels;
        m_poolingParams.inputHeight  = m_inputBlobSize.height;
        m_poolingParams.inputWidth   = m_inputBlobSize.width;
        m_poolingParams.outputHeight = m_outputBlobSize.height;
        m_poolingParams.outputWidth  = m_outputBlobSize.width;
    }

    int32_t setup()
    {
        CHECK_CUDA_ERROR(cudnnCreate(&m_cudnn));                         // initialize cudnn and cublas
        CHECK_CUDA_ERROR(cudnnCreateTensorDescriptor(&m_srcDescriptor)); // create cudnn tensor descriptors we need for bias addition
        CHECK_CUDA_ERROR(cudnnCreateTensorDescriptor(&m_dstDescriptor));
        CHECK_CUDA_ERROR(cudnnCreatePoolingDescriptor(&m_poolingDescriptor));
        CHECK_CUDA_ERROR(cudnnSetPooling2dDescriptor(m_poolingDescriptor,
                                                     m_poolingParams.poolingMode, CUDNN_NOT_PROPAGATE_NAN,
                                                     m_poolingParams.kernelHeight,
                                                     m_poolingParams.kernelWidth,
                                                     m_poolingParams.paddingY, m_poolingParams.paddingX,
                                                     m_poolingParams.strideY, m_poolingParams.strideX));

        return 0;
    }

    void terminate()
    {
        CHECK_CUDA_ERROR(cudnnDestroyTensorDescriptor(m_srcDescriptor));
        CHECK_CUDA_ERROR(cudnnDestroyTensorDescriptor(m_dstDescriptor));
        CHECK_CUDA_ERROR(cudnnDestroyPoolingDescriptor(m_poolingDescriptor));
        CHECK_CUDA_ERROR(cudnnDestroy(m_cudnn));
    }

    size_t getWorkspaceSize(int) const
    {
        return 0;
    }

    int32_t enqueue(int32_t batchSize, const void* const* inputs, void** outputs, void*, cudaStream_t stream)
    {
        const float32_t kONE  = 1.0f;
        const float32_t kZERO = 0.0f;
        cudnnSetStream(m_cudnn, stream);
        // Update batch size
        m_inputBlobSize.batchsize  = batchSize;
        m_outputBlobSize.batchsize = batchSize;

        // Prepare tensors
        cudnnSetTensor4dDescriptor(m_srcDescriptor, CUDNN_TENSOR_NCHW, m_typeMap[m_precision],
                                   m_inputBlobSize.batchsize, m_poolingParams.numChannels,
                                   m_poolingParams.inputHeight, m_poolingParams.inputWidth);
        cudnnSetTensor4dDescriptor(m_dstDescriptor, CUDNN_TENSOR_NCHW, m_typeMap[m_precision],
                                   m_outputBlobSize.batchsize, m_poolingParams.numChannels,
                                   m_poolingParams.outputHeight, m_poolingParams.outputWidth);

        CHECK_CUDA_ERROR(cudnnPoolingForward(m_cudnn, m_poolingDescriptor, &kONE, m_srcDescriptor, inputs[0],
                                             &kZERO, m_dstDescriptor, outputs[0]));
        return 0;
    }

    size_t getSerializationSize()
    {
        size_t serializationSize = 0U;
        serializationSize += sizeof(m_poolingParams);
        serializationSize += sizeof(m_inputBlobSize);
        serializationSize += sizeof(m_outputBlobSize);
        serializationSize += sizeof(m_precision);
        return serializationSize;
    }

    void serialize(void* buffer)
    {
        char8_t* d = static_cast<char8_t*>(buffer);

        write(d, m_poolingParams);
        write(d, m_inputBlobSize);
        write(d, m_outputBlobSize);
        write(d, m_precision);
    }

    dwPrecision getOutputPrecision(int32_t index, const dwPrecision* inputPrecisions, int32_t numInputs) const
    {
        if (index >= numInputs || inputPrecisions == nullptr)
        {
            throw std::runtime_error("PoolPlugin::getOutputPrecision - Invalid argument.");
        }
        return inputPrecisions[index];
    }

    static const char8_t* getPluginType()
    {
        return "MaxPool";
    }

    static const char8_t* getPluginVersion()
    {
        return "2";
    }

    void setPluginNamespace(const char8_t* libNamespace)
    {
        m_namespace = libNamespace;
    }

    const char8_t* getPluginNamespace() const
    {
        return m_namespace.c_str();
    }

    dwDNNPluginFieldCollection getFieldCollection()
    {
        return m_fieldCollection;
    }

private:
    size_t type2size(dwPrecision precision) { return precision == DW_PRECISION_FP32 ? sizeof(float32_t) : sizeof(__half); }

    template <typename T>
    void write(char8_t*& buffer, const T& val)
    {
        *reinterpret_cast<T*>(buffer) = val;
        buffer += sizeof(T);
    }

    template <typename T>
    T read(const char8_t* data, size_t totalLength, const char8_t*& itr)
    {
        if (static_cast<size_t>(itr + sizeof(T) - data) > totalLength)
        {
            throw std::runtime_error("PoolPlugin - Failed to deserialize.");
        }

        T val = *reinterpret_cast<const T*>(itr);
        itr += sizeof(T);
        return val;
    }

    inline void getDateString(char* buf, size_t length)
    {
        time_t now          = ::time(nullptr);
        struct tm* calendar = localtime(&now);
        strftime(buf, length, "[%Y-%m-%d %X] ", calendar);
    }

    dwBlobSize m_inputBlobSize{};
    dwBlobSize m_outputBlobSize{};

    PoolingParams m_poolingParams{};

    dwPrecision m_precision{DW_PRECISION_FP32};
    std::map<dwPrecision, cudnnDataType_t> m_typeMap = {{DW_PRECISION_FP32, CUDNN_DATA_FLOAT},
                                                        {DW_PRECISION_FP16, CUDNN_DATA_HALF},
                                                        {DW_PRECISION_INT8, CUDNN_DATA_INT8}};

    cudnnHandle_t m_cudnn;
    cudnnTensorDescriptor_t m_srcDescriptor, m_dstDescriptor;
    cudnnPoolingDescriptor_t m_poolingDescriptor;

    std::string m_namespace;
    dwDNNPluginFieldCollection m_fieldCollection{};
};

dwStatus _dwDNNPlugin_create(_dwDNNPluginHandle_t* handle)
{
    std::unique_ptr<PoolPlugin> plugin(new PoolPlugin());
    *handle = reinterpret_cast<_dwDNNPluginHandle_t>(plugin.release());
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_setup(_dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->setup();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_terminate(_dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->terminate();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_deserializeFromBuffer(const char8_t* name, const void* buffer, size_t len,
                                            _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->deserializeFromBuffer(name, buffer, len);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_deserializeFromFieldCollection(const char8_t* name, const dwDNNPluginFieldCollection* fieldCollection,
                                                     _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->deserializeFromFieldCollections(name, *fieldCollection);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_deserializeFromWeights(const dwDNNPluginWeights* weights, int32_t numWeights,
                                             _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->deserializeFromWeights(weights, numWeights);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_destroy(_dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    delete plugin;
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getNumOutputs(int32_t* numOutputs, _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    *numOutputs = plugin->getNbOutputs();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getOutputDimensions(dwBlobSize* outputDimensions,
                                          int32_t outputIndex, const dwBlobSize* inputDimensions,
                                          int32_t numInputs, _dwDNNPluginHandle_t handle)
{
    auto plugin       = reinterpret_cast<PoolPlugin*>(handle);
    *outputDimensions = plugin->getOutputDimensions(outputIndex, inputDimensions, numInputs);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getWorkspaceSize(size_t* workspaceSize, int32_t maxBatchSize,
                                       _dwDNNPluginHandle_t handle)
{
    auto plugin    = reinterpret_cast<PoolPlugin*>(handle);
    *workspaceSize = plugin->getWorkspaceSize(maxBatchSize);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_supportsFormatCombination(bool* res, int32_t index, const dwDNNPluginTensorDesc* inOut,
                                                int32_t numInputs, int32_t numOutputs,
                                                _dwConstDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<const PoolPlugin*>(handle);
    *res        = plugin->supportsFormatCombination(index, inOut, numInputs, numOutputs);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_configurePlugin(const dwDNNPluginTensorDesc* inputDescs, int32_t numInputs,
                                      const dwDNNPluginTensorDesc* outputDescs, int32_t numOutputs,
                                      _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->configurePlugin(inputDescs, numInputs, outputDescs, numOutputs);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_enqueue(int32_t batchSize, const void* const* inputs, void** outputs,
                              void* workspace, cudaStream_t stream, _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->enqueue(batchSize, inputs, outputs, workspace, stream);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getSerializationSize(size_t* serializationSize, _dwDNNPluginHandle_t handle)
{
    auto plugin        = reinterpret_cast<PoolPlugin*>(handle);
    *serializationSize = plugin->getSerializationSize();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_serialize(void* buffer, _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->serialize(buffer);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getPluginType(const char8_t** pluginType, _dwConstDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<const PoolPlugin*>(handle);
    *pluginType = plugin->getPluginType();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getPluginVersion(const char8_t** pluginVersion, _dwConstDNNPluginHandle_t handle)
{
    auto plugin    = reinterpret_cast<const PoolPlugin*>(handle);
    *pluginVersion = plugin->getPluginVersion();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_setPluginNamespace(const char8_t* pluginNamespace, _dwDNNPluginHandle_t handle)
{
    auto plugin = reinterpret_cast<PoolPlugin*>(handle);
    plugin->setPluginNamespace(pluginNamespace);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getPluginNamespace(const char8_t** pluginNamespace, _dwConstDNNPluginHandle_t handle)
{
    auto plugin      = reinterpret_cast<const PoolPlugin*>(handle);
    *pluginNamespace = plugin->getPluginNamespace();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_isOutputBroadcastAcrossBatch(bool* isOutputBroadcastAcrossBatch,
                                                   int32_t, const bool*,
                                                   int32_t, _dwConstDNNPluginHandle_t)
{
    *isOutputBroadcastAcrossBatch = false;
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_canBroadcastInputAcrossBatch(bool* canBroadcastInputAcrossBatch, int32_t,
                                                   _dwConstDNNPluginHandle_t)
{
    *canBroadcastInputAcrossBatch = false;
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getOutputPrecision(dwPrecision* outputPrecision, int32_t outputIndex,
                                         const dwPrecision* inputPrecisions, int32_t numInputs,
                                         _dwConstDNNPluginHandle_t handle)
{
    auto plugin      = reinterpret_cast<const PoolPlugin*>(handle);
    *outputPrecision = plugin->getOutputPrecision(outputIndex, inputPrecisions, numInputs);
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_getPluginFieldCollection(dwDNNPluginFieldCollection* fieldCollection,
                                               _dwDNNPluginHandle_t handle)
{
    auto plugin      = reinterpret_cast<PoolPlugin*>(handle);
    *fieldCollection = plugin->getFieldCollection();
    return DW_SUCCESS;
}

dwStatus _dwDNNPlugin_clone(_dwDNNPluginHandle_t* out, _dwDNNPluginHandle_t handle)
{
    auto input = reinterpret_cast<PoolPlugin*>(handle);
    std::unique_ptr<PoolPlugin> plugin(new PoolPlugin(*input));
    *out = reinterpret_cast<_dwDNNPluginHandle_t>(plugin.release());

    return DW_SUCCESS;
}
