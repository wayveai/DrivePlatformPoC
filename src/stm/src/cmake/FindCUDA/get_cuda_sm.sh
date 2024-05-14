#!/bin/bash
#
# /////////////////////////////////////////////////////////////////////////////////////////
#
# Notice
# ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
# NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
# THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
# MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
#
# NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
# information or for any infringement of patents or other rights of third parties that may
# result from its use. No license is granted by implication or otherwise under any patent
# or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
# expressly authorized by NVIDIA. Details are subject to change without notice.
# This code supersedes and replaces all information previously supplied.
# NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
# components in life support devices or systems without express written approval of
# NVIDIA CORPORATION & AFFILIATES.
#
# SPDX-FileCopyrightText: Copyright (c) 2018-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
# /////////////////////////////////////////////////////////////////////////////////////////

# Prints the compute capability of all CUDA device installed
# on the system usable with the nvcc `--gpu-code=` flag

timestamp=$(date +%s.%N)
gcc_binary=${CMAKE_CXX_COMPILER:-$(which c++)}
cuda_root=${CUDA_DIR:-/usr/local/cuda}
CUDA_INCLUDE_DIRS=${CUDA_INCLUDE_DIRS:-${cuda_root}/include}
CUDA_CUDART_LIBRARY=${CUDA_CUDART_LIBRARY:-${cuda_root}/lib64/libcudart.so}
generated_binary="/tmp/cuda-compute-version-helper-$$-$timestamp"

# create a 'here document' that is code we compile and use to probe the card
source_code="$(cat << EOF
#include <cuda_runtime_api.h>
#include <sstream>
#include <iostream>

int main() {
  auto device_count = int{};
  auto status = cudaGetDeviceCount(&device_count);
  if (status != cudaSuccess) {
    std::cerr << "cudaGetDeviceCount() failed: " << cudaGetErrorString(status)
              << std::endl;
    return -1;
  }
  if (!device_count) {
    std::cerr << "No cuda devices found" << std::endl;
    return -1;
  }

  std::stringstream flag;
  flag << "--gpu-code=";

  for (auto i = 0; i < device_count; ++i) {
    if (i != 0)
      flag << ",";

    auto prop = cudaDeviceProp{};
    status = cudaGetDeviceProperties(&prop, i);
    if (status != cudaSuccess) {
      std::cerr
          << "cudaGetDeviceProperties() for device ${device_index} failed: "
          << cudaGetErrorString(status) << std::endl;
      return -1;
    }

    flag << "sm_" << prop.major * 10 + prop.minor;
  }

  std::cout << flag.str() << std::endl;
  return 0;
}
EOF
)"
echo "$source_code" | $gcc_binary -std=c++11 -x c++ -I"$CUDA_INCLUDE_DIRS" -o "$generated_binary" - -x none "$CUDA_CUDART_LIBRARY"

# probe the card and cleanup
$generated_binary
ret_code=$?
rm $generated_binary
exit $ret_code
