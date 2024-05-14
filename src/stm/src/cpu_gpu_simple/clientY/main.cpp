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
// SPDX-FileCopyrightText: Copyright (c) 2018-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <stm.h>
#include <memory>
#include <iostream>
#include <unistd.h>

#ifdef NDEBUG
#undef NDEBUG
#include <assert.h>
#define NDEBUG
#else
#include <assert.h>
#endif

#include "gpu.hpp"

cudaStream_t m_stream;
cudaEvent_t start, stop;
int* canaryHost;
int* canaryDevice;

void submit(void* params, cudaStream_t stream)
{
    (void)params;
    (void)stream;

    testKernel(stream);
    fprintf(stderr, "In submit\n");
}

void test2(void* params)
{
    (void)params;

    fprintf(stderr, "In test 2\n");
}

void test1(void* params)
{
    (void)params;

    fprintf(stderr, "In test 1\n");
}

int main(int argc, const char** argv)
{
    (void)argc;
    (void)argv;

    assert(cudaStreamCreateWithFlags(&m_stream, cudaStreamNonBlocking) == cudaSuccess);

    stmClientInit("clientGpuY"); // Needs to be called before registration

    stmRegisterCudaSubmitter(submit, "submit", NULL);
    stmRegisterCpuRunnable(test1, "test1", NULL);

    // Register all resources in the workload
    stmRegisterCudaResource("CUDA_STREAMY", m_stream);

    stmEnterScheduler();

    stmClientExit(); // Removes all STM data structures. Can't use STM calls anymore after this.
}
