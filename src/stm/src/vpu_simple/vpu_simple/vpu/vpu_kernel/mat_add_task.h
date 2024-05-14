/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef MAT_ADD_TASK_H
#define MAT_ADD_TASK_H

#include <cupva_host.h>
#include <stdint.h>
#include <stdio.h>

#define CUPVA_CHECK_ERROR_RETURN(__v)                    \
    if ((__v) != CUPVA_ERROR_NONE)                       \
    {                                                    \
        printf("CUPVA C-API return error: %d\n", (__v)); \
        return ((__v) != CUPVA_ERROR_NONE);              \
    }

int32_t CreateMatAddExec(cupvaExecutable_t *exec);
int32_t CreateMatAddProg(double waitDurationTicks, cupvaCmd_t *cmdProg, cupvaExecutable_t const *exec, int32_t *src1_d,
                         int32_t *src2_d, int32_t *dst_d, uint32_t ih, uint32_t iw, uint32_t bh, uint32_t bw);

#endif // MAT_ADD_TASK_H
