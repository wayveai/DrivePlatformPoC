/* Copyright (c) 2017-2022 NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#if VIBRANTE_PDK_DECIMAL >= 6000400
#include "VpuTestContext.hpp"
#include <iostream>

constexpr int SUCCESS{0};

void VpuTestContext::SetUp()
{
    std::cout << "Setting up vpu_simple test" << std::endl;
    if (cupvaStreamCreate(&stream, PVA_PVA0, PVA_VPU0) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to create cuPVA stream.");
    }

    int bufSize = MAT_W * MAT_H * sizeof(int);
    dst_ref     = (int*)malloc(bufSize);

    if (cupvaMemAlloc((void**)&src1_d, bufSize, PVA_READ_WRITE, PVA_ALLOC_DRAM) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to allocate cuPVA memory.");
    }
    if (cupvaMemAlloc((void**)&src2_d, bufSize, PVA_READ_WRITE, PVA_ALLOC_DRAM) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to allocate cuPVA memory.");
    }
    if (cupvaMemAlloc((void**)&dst_d, bufSize, PVA_READ_WRITE, PVA_ALLOC_DRAM) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to allocate cuPVA memory.");
    }

    if (cupvaMemGetHostPointer((void**)&src1_h, (void*)src1_d) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to get cuPVA host pointer.");
    }
    if (cupvaMemGetHostPointer((void**)&src2_h, (void*)src2_d) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to get cuPVA host pointer.");
    }
    if (cupvaMemGetHostPointer((void**)&dst_cmp, (void*)dst_d) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to get cuPVA host pointer.");
    }

    for (int i = 0; i < MAT_W * MAT_H; i++)
    {
        src1_h[i]  = rand();
        src2_h[i]  = rand();
        dst_cmp[i] = 0;
        dst_ref[i] = 0;
    }
}

void VpuTestContext::CreateCmdProg(uint64_t waitDuration)
{
    if (CreateMatAddExec(&m_exec) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to create MatAddExec.");
    }
    if (CreateMatAddProg(waitDuration, &m_prog, &m_exec, src1_d, src2_d, dst_d, MAT_H, MAT_W, bh, bw) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Failed to create MatAddProg.");
    }
}

void VpuTestContext::ExpectUnfinished()
{
    // Initially, both should be the same (all zeros)
    int err = memcmp(dst_ref, dst_cmp, MAT_W * MAT_H * sizeof(int));

    if (err != SUCCESS)
    {
        throw std::runtime_error("Mismatch in cuPVA and cpu arrays in ExpectUnfinished()");
    }
    RunMatAddCPU();
    std::cout << "ExpectUnfinished Passed" << std::endl;
}

void VpuTestContext::RunTest()
{
    std::cout << "RunningTest" << std::endl;

    cupvaCmd_t const* cmds[1] = {&m_prog};
    if (cupvaStreamSubmit(stream, cmds, NULL, 1, PVA_IN_ORDER, -1, -1) != PVA_ERROR_NONE)
    {
        throw std::runtime_error("Error in submitting cuPVA commands.");
    }
}

void VpuTestContext::ExpectFinished()
{
    int err = memcmp(dst_ref, dst_cmp, MAT_W * MAT_H * sizeof(int));

    if (err != SUCCESS)
    {
        throw std::runtime_error("Mismatch in cuPVA and cpu arrays in ExpectFinished()");
    }

    // Randomize numbers for next frame and clear results to check synchronization
    for (int i = 0; i < MAT_W * MAT_H; i++)
    {
        src1_h[i]  = rand();
        src2_h[i]  = rand();
        dst_cmp[i] = 0;
        dst_ref[i] = 0;
    }
    std::cout << "ExpectFinished Passed" << std::endl;
}

void VpuTestContext::RunMatAddCPU()
{
    for (int y = 0; y < MAT_H; y++)
    {
        for (int x = 0; x < MAT_W; x++)
        {
            int addr      = y * MAT_W + x;
            dst_ref[addr] = src1_h[addr] + src2_h[addr];
        }
    }
}
#endif