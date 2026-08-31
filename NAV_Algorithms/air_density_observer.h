/***********************************************************************//**
 * @file		air_density_observer.h
 * @brief		air-density measurement (interface)
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#ifndef AIR_DENSITY_OBSERVER_H_
#define AIR_DENSITY_OBSERVER_H_

#include "pt2.h"
#include "QuadraticLeastSquareFit.h"
#include "trigger.h"

typedef float evaluation_type;
typedef float  acquisition_type;
#define altitude_scale_factor 1.0f
#define pressure_scale_factor 1.0f

#define ALTITUDE_TRIGGER_HYSTERESIS	50.0f
#define MAX_ALLOWED_SLOPE_VARIANCE	3e-9
#define MAX_ALLOWED_OFFSET_VARIANCE	200
#define MINIMUM_ALTITUDE_RANGE		250.0f
#define MAXIMUM_ALTITUDE_RANGE		500.0f
#define USE_AIR_DENSITY_LETHARGY	1
#define AIR_DENSITY_LETHARGY 		0.7f
#define AIR_DENSITY_DECIMATION		20

//! Maintains offset and slope of the air density measurement
class density_data
{
public:
  density_data( void)
    : density_offset( 1.224096628212817f),
      density_slope( -0.000115412739613f),
      variance( ZERO),
      valid( false)
  {}
  float density_offset;
  float density_slope;
  float variance;
  bool valid;
};

//! Measures air density and reference pressure
class air_density_observer_t
{
public:
  air_density_observer_t (void)
  : min_altitude(10000.0f),
    max_altitude(0.0f),
    altitude_trigger( ALTITUDE_TRIGGER_HYSTERESIS),
    decimation_counter( 20),
    sample_counter( 0),
    pressure_decimation_filter( 1.0f / AIR_DENSITY_DECIMATION * 0.25f)
  {
  }

  density_data feed_metering( float pressure, float MSL_altitude);

  void initialize( float altitude)
  {
    altitude_trigger.initialize(altitude);
    min_altitude = max_altitude = altitude;
    density_QFF_calculator.reset();
  }
private:
    Quadratic_Least_Square_Fit <acquisition_type, evaluation_type> density_QFF_calculator;
    float min_altitude;
    float max_altitude;
    trigger altitude_trigger;
    unsigned decimation_counter;
    unsigned sample_counter;
    pt2 <float, float> pressure_decimation_filter;
};

#endif /* AIR_DENSITY_OBSERVER_H_ */
