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

#ifndef SAMPLES_EGOMOTION_TRAJECTORYLOGGER_HPP_
#define SAMPLES_EGOMOTION_TRAJECTORYLOGGER_HPP_

#include <dw/core/base/Types.h>
#include <dw/sensors/gps/GPS.h>

#include <framework/Log.hpp>

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <iomanip>
#include <cmath>

class TrajectoryLogger
{
public:
    enum class Color
    {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        CYAN,
        MAGENTA,
        WHITE
    };

    enum class Display
    {
        SHOW,
        HIDE_MARKERS,
        HIDE
    };

    struct Trajectory
    {
        std::vector<dwVector3d> position;
        Color color;
    };

    ///------------------------------------------------------------------------------
    /// Add trajectory with name and color
    ///------------------------------------------------------------------------------
    void addTrajectory(const std::string& trajectoryName, Color color, Display display = Display::SHOW)
    {
        std::string bgr;
        m_trajectories.insert({trajectoryName, Trajectory{.position = {}, .color = color}});
        m_visibility.insert({trajectoryName, display});
    }

    ///------------------------------------------------------------------------------
    /// Add position in WGS84
    ///------------------------------------------------------------------------------
    void addWGS84(const std::string& trajectoryName, const dwGeoPointWGS84& position)
    {
        m_trajectories.at(trajectoryName).position.push_back({position.lon, position.lat, position.height});
    }

    ///------------------------------------------------------------------------------
    /// Add position in WGS84 from dwGPSFrame
    ///------------------------------------------------------------------------------
    void addWGS84(const std::string& trajectoryName, const dwGPSFrame& frame)
    {
        m_trajectories.at(trajectoryName).position.push_back({frame.longitude, frame.latitude, frame.altitude});
    }

    ///------------------------------------------------------------------------------
    /// Get trajectory size
    ///------------------------------------------------------------------------------
    std::size_t size(const std::string& trajectoryName)
    {
        return m_trajectories.at(trajectoryName).position.size();
    }

    ///------------------------------------------------------------------------------
    /// Write TrajectoryLogger contents to KML file
    ///------------------------------------------------------------------------------
    void writeKML(const std::string filename)
    {
        std::ofstream file(filename, std::ios_base::out | std::ios_base::trunc);

        if (!file.is_open())
        {
            logError("Could not open file with name '%s'", filename.c_str());
        }

        file << std::fixed << std::setprecision(8);

        // write header
        file << R"(<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>TrajectoryLog</name>)" << std::endl;

        // write styles, transparency hardcoded to 50%
        for (auto& kv : m_trajectories)
        {
            std::string abgr;
            switch (kv.second.color)
            {
            case Color::RED:
                abgr = "7F0000FF";
                break;
            case Color::GREEN:
                abgr = "7F00FF00";
                break;
            case Color::BLUE:
                abgr = "7FFF0000";
                break;
            case Color::YELLOW:
                abgr = "7F00FFFF";
                break;
            case Color::CYAN:
                abgr = "7FFFFF00";
                break;
            case Color::MAGENTA:
                abgr = "7FFF00FF";
                break;
            case Color::WHITE:
            default:
                abgr = "7FFFFFFF";
                break;
            }

            file << R"(<Style id=")" << kv.first << R"(">)";
            file << "<LineStyle><color>" << abgr << "</color><width>3</width></LineStyle></Style>";
            file << std::endl;
        }

        // write trajectories
        for (auto& kv : m_trajectories)
        {
            file << "<Placemark><name>" << kv.first << "</name><styleUrl>#" << kv.first << "</styleUrl>";
            file << "<visibility>" << (m_visibility[kv.first] != Display::HIDE) << "</visibility><MultiGeometry>";

            for (std::size_t i = 0; i < kv.second.position.size(); i++)
            {
                file << "<LineString><tessellate>1</tessellate><altitudeMode>clampToGround</altitudeMode><coordinates>" << std::endl;

                for (std::size_t points = 0; i < kv.second.position.size() && points < MAX_TRAJECTORY_SIZE; i++, points++)
                {
                    const dwVector3d& point = kv.second.position[i];
                    file << point.x << "," << point.y << "," << point.z << std::endl;
                }

                file << "</coordinates></LineString>" << std::endl;
            }

            file << "</MultiGeometry></Placemark>" << std::endl;
        }

        // write markers
        for (auto& kv : m_trajectories)
        {
            if (!kv.second.position.empty())
            {
                const dwVector3d& start = kv.second.position.front();
                const dwVector3d& end   = kv.second.position.back();

                file << "<Placemark><name>" << kv.first << " (begin)</name><visibility>" << (m_visibility[kv.first] == Display::SHOW) << "</visibility><Point><coordinates>";
                file << start.x << "," << start.y << "," << start.z;
                file << "</coordinates></Point></Placemark>" << std::endl;

                file << "<Placemark><name>" << kv.first << " (end)</name><visibility>" << (m_visibility[kv.first] == Display::SHOW) << "</visibility><Point><coordinates>";
                file << end.x << "," << end.y << "," << end.z;
                file << "</coordinates></Point></Placemark>" << std::endl;
            }
        }

        // write footer
        file << "</Document></kml>";

        file.flush();
        file.close();
    }

protected:
    static constexpr std::size_t MAX_TRAJECTORY_SIZE = 64000;

    std::map<std::string, Trajectory> m_trajectories;
    std::map<std::string, Display> m_visibility;
};

#endif //SAMPLES_EGOMOTION_TRAJECTORYLOGGER_HPP_
