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
// SPDX-FileCopyrightText: Copyright (c) 2015-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_COMMON_CONSOLECOLOR_HPP_
#define SAMPLES_COMMON_CONSOLECOLOR_HPP_

#include <iosfwd>
#include <stdio.h>
#include <dw/core/logger/Logger.h>

enum EConsoleColor
{
    COLOR_DEFAULT,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_YELLOW
};

void printColored(FILE* fd, EConsoleColor color, const char* msg);
dwLogCallback getConsoleLoggerCallback(bool useColors, bool disableBuffering = false);
dwLoggerCallback getConsoleLoggerExtendedCallback(bool useColors, bool disableBuffering = false);
dwLoggerCallback baseExtendedCallback();
void prependLoggerMessageInfo(std::ostream& oss, dwLoggerMessage const& msg);

void streamIsoTimestamp(std::ostream& os);
void streamTag(std::ostream& os, char const* tag);
void streamThreadId(std::ostream& os, char const* tid);
void streamSourceCodeLocation(std::ostream& os, char const* fileName, int32_t lineNum);
void streamVerbosity(std::ostream& os, dwLoggerVerbosity const level);

void logError(const char* format, ...);
void logWarn(const char* format, ...);
void log(const char* format, ...);

#endif // SAMPLES_COMMON_CONSOLECOLOR_HPP_
