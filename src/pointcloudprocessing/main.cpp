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
// SPDX-FileCopyrightText: Copyright (c) 2017-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "PointCloudProcessingSample.hpp"

//------------------------------------------------------------------------------
int32_t main(int32_t argc, const char** argv)
{
    typedef ProgramArguments::Option_t opt;

    ProgramArguments args(argc, argv,
                          {opt("rigFile", (dw_samples::SamplesDataPath::get() + "/samples/sensors/lidar/rig.json").c_str()),
                           opt("numFrames", "0", "The number of frames to process. By default it processes all frames"),
                           opt("maxIters", "12", "Number of ICP iterations to run."),
                           opt("displayWindowHeight", "900", "display window height "),
                           opt("displayWindowWidth", "1500", "display window width")});

    PointCloudProcessingSample app(args);
    int32_t width  = std::atoi(args.get("displayWindowWidth").c_str());
    int32_t height = std::atoi(args.get("displayWindowHeight").c_str());
    app.initializeWindow("Point Cloud Processor Sample", width, height, args.enabled("offscreen"));
    return app.run();
}
