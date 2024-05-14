/* Copyright (c) 2017-2022 NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef STM_TESTS_LEGACY_VPU_VPUTESTCONTEXT_H
#define STM_TESTS_LEGACY_VPU_VPUTESTCONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif
#include <mat_add_task.h>
#ifdef __cplusplus
}
#endif
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <cupva_host_wrapper.h>

constexpr int MAT_W{1024};
constexpr int MAT_H{256};
constexpr int bh{32};
constexpr int bw{32};

class VpuTestContext final
{

public:
    ~VpuTestContext()
    {
        (void)cupvaMemFree(src1_d);
        (void)cupvaMemFree(src2_d);
        (void)cupvaMemFree(dst_d);
        free(dst_ref);
    }

    void SetUp();
    void CreateCmdProg(uint64_t waitDuration);
    void RunTest();
    void ExpectUnfinished();
    void ExpectFinished();

    cupvaStream_t getStream() { return stream; }
    int32_t* getSrc1_d() { return this->src1_d; }
    int32_t* getSrc2_d() { return this->src2_d; }
    int32_t* getDst_d() { return this->dst_d; }

private:
    void RunMatAddCPU();
    int* dst_ref;

    int32_t* src1_d;
    int32_t* src2_d;
    int32_t* dst_d;

    int32_t* src1_h;
    int32_t* src2_h;
    int32_t* dst_cmp;

    cupvaStream_t stream;
    cupvaExecutable_t m_exec;
    cupvaCmd_t m_prog;
};
#endif