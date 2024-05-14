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
// SPDX-FileCopyrightText: Copyright (c) 2015-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_FRAMEWORK_SAMPLESDATAPATH_HPP_
#define SAMPLES_FRAMEWORK_SAMPLESDATAPATH_HPP_

#include <string>
#include <vector>

namespace dw_samples
{
namespace common
{

// SamplesDataPath retrieval to be used from any sample to accommodate for different installation locations
class SamplesDataPath
{
public:
    // List of well-known search paths, giving preference to env SDK_TESTS_DATA_PATH (if set), and including
    // all subfolders from the current executable up to the filesystem root
    static std::vector<std::string> getSearchPaths();

    // Searches for 'fileOrDirectory' in list of search paths, returning base path for the first hit.
    // Returns emtpy string if search was not successful
    static std::string getBasePathFor(std::string const& fileOrDirectory,
                                      std::vector<std::string> const& searchPaths = getSearchPaths());

    // Searches for 'fileOrDirectory' in list of search paths, returning path for the first hit.
    // Returns emtpy string if search was not successful
    static std::string getPathFor(std::string const& fileOrDirectory,
                                  std::vector<std::string> const& searchPaths = getSearchPaths());

    // Convenience functions returning the data base path of the DATA_ROOT file from the list of search paths.
    // Returns emtpy string if search was not successful
    static std::string getBasePath(std::vector<std::string> const& searchPaths = getSearchPaths());

    // Convenience function aliasing getBasePath()
    static std::string get();

    // Convenience function aliasing getBasePath() with static string storage (not thread-safe)
    static char const* get_cstr();
};

// Runtime information about current executable
class Executable
{
public:
    // Returns name of executable
    static std::string getName();

    // Returns the current executable's path or throws
    static std::string getPath();

    // Returns the current executable's parent folder path or throws
    static std::string getFolderPath();
};

// Filesystem property helpers
class Filesystem
{
public:
    // Returns true if fileName exists and is a file
    static bool fileExists(char const* fileName);

    // Returns true if directoryName exists and is a directory
    static bool directoryExists(char const* directoryName);
};

} // namespace common

// Make symbol available in dw_samples namespace also for convenience
using SamplesDataPath = common::SamplesDataPath;

} // namespace dw_samples

#endif // SAMPLES_FRAMEWORK_SAMPLESDATAPATH_HPP_
