/*
 * Copyright (c) 2020-2022 NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <stm.h>
#include <stdlib.h>

#if VIBRANTE_PDK_DECIMAL >= 6000400
#include <VpuTestContext.hpp>

void confirmUnfinished(void* params)
{
    auto test = (VpuTestContext*)params;
    test->ExpectUnfinished();
}

void confirmFinished(void* params)
{
    auto test = (VpuTestContext*)params;
    test->ExpectFinished();
}

void vpuSubmit(void* params, cupvaStream_t vpu)
{
    (void)vpu; //unused
    auto test = (VpuTestContext*)params;
    test->RunTest();
}
#endif

// ------------------------------------- //
// testbench                             //
// ------------------------------------- //
int main(int argc, const char** argv)
{
#if VIBRANTE_PDK_DECIMAL >= 6000400
    (void)argc;
    (void)argv;
    VpuTestContext test;
    test.SetUp();
    test.CreateCmdProg(0);
    stmClientInit("client0"); // Needs to be called before registration
    stmRegisterCpuRunnable(confirmUnfinished, "confirmUnfinished", &test);
    stmRegisterCpuRunnable(confirmFinished, "confirmFinished", &test);

    stmRegisterVpuSubmitter(vpuSubmit, "vpuSubmit0", &test);
    stmRegisterVpuSubmitter(vpuSubmit, "vpuSubmit1", &test);

    // Register all resources in the workload
    stmRegisterVpuResource("PVA_STREAM0", test.getStream());

    stmEnterScheduler();

    stmClientExit(); // Removes all STM data structures. Can't use STM calls anymore after this.
#endif
}
