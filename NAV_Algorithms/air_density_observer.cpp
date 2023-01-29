/***********************************************************************//**
 * @file		air_density_observer.cpp
 * @brief		air-density measurement using a linear least square fit altitude over pressure
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

#include "embedded_math.h"
#include <air_density_observer.h>

air_data_result air_density_observer::feed_metering( float pressure, float MSL_altitude)
{
  air_data_result air_data;

#if DENSITY_MEASURMENT_COLLECTS_INTEGER
  density_QFF_calculator.add_value( MSL_altitude * 100.0f, pressure);
#else
  // use values normalized around 1.0
  density_QFF_calculator.add_value( MSL_altitude * 1e-3, pressure * 1e-5);
#endif

  if( MSL_altitude > max_altitude)
    max_altitude = MSL_altitude;

  if( MSL_altitude < min_altitude)
    min_altitude = MSL_altitude;

  if( false == altitude_trigger.process(MSL_altitude))
    return air_data;

  if( ((max_altitude - min_altitude) < MINIMUM_ALTITUDE_RANGE) // ... forget this measurement
    || (density_QFF_calculator.get_count() < 3000))
    {
      max_altitude = min_altitude = MSL_altitude;
      density_QFF_calculator.reset();
      return air_data;
    }

  linear_least_square_result<evaluation_type> result;
  density_QFF_calculator.evaluate(result);

//  Due to numeric effects, when using float the
//  variance has been observed to be negative in some cases !
//  Therefore using float this test can not be used.
 assert( result.variance_slope > 0);
 assert( result.variance_offset > 0);

 if( result.variance_slope < MAX_ALLOWED_VARIANCE)
    {
      air_data.QFF = result.y_offset;
      float density = 100.0f * result.slope * -0.10194f; // div by -9.81f;

#if DENSITY_MEASURMENT_COLLECTS_INTEGER
      float pressure = density_QFF_calculator.get_mean_y();
#else
      float pressure = 1e5 * density_QFF_calculator.get_mean_y();
#endif

      float std_density = 1.0496346613e-5f * pressure + 0.1671546011f;
      air_data.density_correction = density / std_density;
      air_data.valid = true;
    }

  max_altitude = min_altitude = MSL_altitude;
  density_QFF_calculator.reset();

  return air_data;
}
