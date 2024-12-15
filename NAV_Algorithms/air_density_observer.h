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

#include "Linear_Least_Square_Fit.h"
#include "trigger.h"

#define DENSITY_MEASURMENT_COLLECTS_INTEGER 1
typedef double evaluation_type;
typedef uint64_t measurement_type;

#define MAX_ALLOWED_SLOPE_VARIANCE 1e-9
#define MAX_ALLOWED_OFFSET_VARIANCE 0.05
#define MINIMUM_ALTITUDE_RANGE	250.0f
#define ALTITUDE_TRIGGER_HYSTERESIS 50.0f

//! this class maintains offset and slope of the air density measurement
class air_data_result
{
public:
  air_data_result( void)
    : density_correction(1.0f),
      QFF(101325.0f),
      valid( false)
  {}
  float density_correction;
  float QFF;
  bool valid;
};

//! this class measures air density and QFF
class air_density_observer
{
public:
  air_density_observer (void)
  : min_altitude(10000.0f),
    max_altitude(0.0f),
    altitude_trigger( ALTITUDE_TRIGGER_HYSTERESIS)
  {
  }
  air_data_result feed_metering( float pressure, float MSL_altitude);

  void initialize( float altitude)
  {
    altitude_trigger.initialize(altitude);
    min_altitude = max_altitude = altitude;
    density_QFF_calculator.reset();
  }
private:

  //    linear_least_square_fit<int64_t,evaluation_float_type> density_QFF_calculator;
    linear_least_square_fit< measurement_type, evaluation_type> density_QFF_calculator;
    float min_altitude;
    float max_altitude;
    trigger altitude_trigger;
};

#endif /* AIR_DENSITY_OBSERVER_H_ */
