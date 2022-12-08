// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 2022.10 MapIV yudai.yamazaki

#ifndef CONVERT_HPP_
#define CONVERT_HPP_

#include "septentrio_gnss_driver/converter/geo_pos_conv.hpp"

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

enum class MGRSPrecision {
  _10_KIRO_METER = 1,
  _1_KIRO_METER = 2,
  _100_METER = 3,
  _10_METER = 4,
  _1_METER = 5,
  _100_MIllI_METER = 6,
  _10_MIllI_METER = 7,
  _1_MIllI_METER = 8,
  _100MICRO_METER = 9,
};

enum class CoordinateSystem {
  UTM = 0,
  MGRS = 1,
  PLANE = 2,
  LOCAL_CARTESIAN = 3,
};

struct GNSSStat
{
  GNSSStat()
  : coordinate_system(CoordinateSystem::MGRS),
    northup(true),
    zone(0),
    x(0),
    y(0),
    z(0),
    latitude(0),
    longitude(0),
    altitude(0)
  {
  }

  CoordinateSystem coordinate_system;
  bool northup;
  int zone;
  double x;
  double y;
  double z;
  double latitude;
  double longitude;
  double altitude;
};

struct Vector2d
{
  double x;
  double y;
};

double LLA2OrthometricHeight(const GNSSStat & lla);

GNSSStat LLA2LocalCartesian(const GNSSStat & lla, const GNSSStat & lla_origin);

GNSSStat LLA2UTM(const GNSSStat & lla);

GNSSStat UTM2MGRS(const GNSSStat & utm, const MGRSPrecision & precision);

GNSSStat LLA2MGRS(const GNSSStat & lla, const MGRSPrecision & precision);

GNSSStat LLA2PLANE(const GNSSStat & lla, const int & plane_zone);

double getDotNorm(Vector2d a, Vector2d b);

double getCrossNorm(Vector2d a, Vector2d b);

double getMeridianConvergence(const GNSSStat & lla, const GNSSStat & converted, const std::string & coordinate, const int & plane_zone);

void QuatMsg2RPY(const geometry_msgs::msg::Quaternion & quat_msg, double & roll, double & pitch, double & yaw);

void RPY2QuatMsg(const double & roll, const double & pitch, const double & yaw, geometry_msgs::msg::Quaternion & quat_msg);

#endif  // GNSS_POSER__CONVERT_HPP_
