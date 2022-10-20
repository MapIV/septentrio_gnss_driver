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

#include "septentrio_gnss_driver/converter/convert.hpp"

double LLA2OrthometricHeight(const GNSSStat & lla)
{
  double OrthometricHeight{0.0};

  GeographicLib::Geoid egm2008("egm2008-1");
  OrthometricHeight = egm2008.ConvertHeight(
    lla.latitude, 
    lla.longitude, 
    lla.altitude, 
    GeographicLib::Geoid::ELLIPSOIDTOGEOID);

  return OrthometricHeight;
}

GNSSStat LLA2LocalCartesian(const GNSSStat & lla, const GNSSStat & lla_origin)
{
  GNSSStat local_cartesian;
  local_cartesian.coordinate_system = CoordinateSystem::LOCAL_CARTESIAN;

  GeographicLib::LocalCartesian localCartesian_origin(
    lla_origin.latitude,
    lla_origin.longitude,
    lla_origin.altitude);

  localCartesian_origin.Forward(
    lla.latitude,
    lla.longitude,
    lla.altitude,
    local_cartesian.x,
    local_cartesian.y,
    local_cartesian.z);

  local_cartesian.latitude = lla.latitude;
  local_cartesian.longitude = lla.longitude;
  local_cartesian.altitude = lla.altitude;

  return local_cartesian;
}

GNSSStat LLA2UTM(const GNSSStat & lla)
{
  GNSSStat utm;
  utm.coordinate_system = CoordinateSystem::UTM;

  GeographicLib::UTMUPS::Forward(lla.latitude, lla.longitude, utm.zone, utm.northup, utm.x, utm.y);

  utm.z = LLA2OrthometricHeight(lla);

  utm.latitude = lla.latitude;
  utm.longitude = lla.longitude;
  utm.altitude = lla.altitude;

  return utm;
}

GNSSStat UTM2MGRS(const GNSSStat & utm, const MGRSPrecision & precision)
{
  constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

  GNSSStat mgrs = utm;
  mgrs.coordinate_system = CoordinateSystem::MGRS;

  std::string mgrs_code;
  GeographicLib::MGRS::Forward(
    utm.zone,
    utm.northup, 
    utm.x,
    utm.y, 
    utm.latitude, 
    static_cast<int>(precision), 
    mgrs_code);
  
  mgrs.zone = std::stod(mgrs_code.substr(0, GZD_ID_size));
  
  mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, 
    static_cast<int>(precision))) * std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) -static_cast<int>(precision));  // set unit as [m]
  
  mgrs.y = std::stod(mgrs_code.substr(GZD_ID_size + static_cast<int>(precision), 
    static_cast<int>(precision))) *std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) - static_cast<int>(precision));  // set unit as [m]

  mgrs.z = utm.z; // TODO(ryo.watanabe)

  return mgrs;
}

GNSSStat LLA2MGRS(const GNSSStat & lla, const MGRSPrecision & precision)
{
  const auto utm = LLA2UTM(lla);
  const auto mgrs = UTM2MGRS(utm, precision);
  return mgrs;
}

GNSSStat LLA2PLANE(const GNSSStat & lla, const int & plane_zone)
{
  GNSSStat plane;
  plane.coordinate_system = CoordinateSystem::PLANE;

  geo_pos_conv geo;
  geo.set_plane(plane_zone);
  geo.llh_to_xyz(lla.latitude, lla.longitude, lla.altitude);
  plane.x = geo.y();
  plane.y = geo.x();
  plane.z = LLA2OrthometricHeight(lla);

  return plane;
}