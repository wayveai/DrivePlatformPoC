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

#ifndef SAMPLE_ICP_PLY_WRITER_HPP_
#define SAMPLE_ICP_PLY_WRITER_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

struct Color
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

inline std::ostream& operator<<(std::ostream& out, Color c)
{
    out << static_cast<int32_t>(c.R) << '\t'
        << static_cast<int32_t>(c.G) << '\t'
        << static_cast<int32_t>(c.B) << '\t';
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const dwLidarPointXYZI& point)
{
    out << point.x << '\t'
        << point.y << '\t'
        << point.z << '\t';
    return out;
}

class PlyWriter
{
public:
    PlyWriter(const char* plyFileName)
        : NumPoints(0), WriteNumberOfVerticesHere(0), PlyFile(plyFileName, std::ios::binary)
    {

        std::cout << "Ply writer init'd at: " << plyFileName << "\n";

        const std::string plyHeader1 =
                              "ply\n\
        format ascii 1.0\n\
        element vertex ",
                          plyHeader2 =
                              "0000000000\n\
        property float x\n\
        property float y\n\
        property float z\n\
        property uchar red\n\
        property uchar green\n\
        property uchar blue\n \
        end_header\n";

        PlyFile << std::string(plyHeader1);
        WriteNumberOfVerticesHere = PlyFile.tellp();
        PlyFile << plyHeader2;
        PlyFile.flush();
    }

    Color Interp(float32_t value)
    {
        static std::vector<Color> HeatMap = {
            {0, 0, 255},
            {0, 255, 0},
            {255, 255, 0},
            {255, 0, 0},
            {255, 255, 255}};

        if (value <= 0)
            return HeatMap.front();
        if (value >= 1)
            return HeatMap.back();

        float32_t relative = value;
        int32_t numInts    = static_cast<int32_t>(HeatMap.size() - 1);
        int32_t index      = static_cast<int32_t>(relative * numInts); // multiply and round up
        relative -= static_cast<float32_t>(index) / numInts;
        relative *= numInts;
        return {static_cast<uint8_t>(HeatMap[index].R * (1.f - relative) + HeatMap[index + 1].R * relative),
                static_cast<uint8_t>(HeatMap[index].G * (1.f - relative) + HeatMap[index + 1].G * relative),
                static_cast<uint8_t>(HeatMap[index].B * (1.f - relative) + HeatMap[index + 1].B * relative)};
    }

    inline void PushPoints(const dwLidarPointXYZI* begin, const dwLidarPointXYZI* end, const Color& color)
    {
        for (auto i = begin; i != end; i++, NumPoints++)
            PlyFile << *i << ", " << color << '\n';
    }

    template <typename Pt>
    inline void PushOnePoint(const Pt& i, const Color& color)
    {
        NumPoints++;
        PlyFile << i << color << "\n";
    }

    template <typename Pt>
    inline void PushOnePoint(const Pt& i, float32_t value /* [0..1]*/)
    {
        NumPoints++;
        PlyFile << i << Interp(value) << "\n";
    }

    inline ~PlyWriter()
    {
        PlyFile.flush();

        PlyFile.seekp(WriteNumberOfVerticesHere);
        PlyFile << std::setw(10) << std::setfill(' ') << NumPoints;
        PlyFile.close();

        std::cout << "\nPlywriting complete, wrote " << NumPoints << " points\n";
    }

private:
    size_t NumPoints;
    std::streamoff WriteNumberOfVerticesHere;
    std::ofstream PlyFile;
};

#endif // SAMPLE_ICP_PLY_WRITER_HPP_
