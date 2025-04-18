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

air_data_result air_density_observer_t::feed_metering( float pressure, float GNSS_altitude)
{
  air_data_result air_data;

  pressure_decimation_filter.respond( pressure);
  altitude_decimation_filter.respond( GNSS_altitude);
  --decimation_counter;
  if( decimation_counter > 0)
    return air_data;
  decimation_counter = AIR_DENSITY_DECIMATION;

  density_QFF_calculator.add_value( GNSS_altitude * 100.0f, pressure);

  // update elevation range
  if( GNSS_altitude > max_altitude)
    max_altitude = GNSS_altitude;

  if( GNSS_altitude < min_altitude)
    min_altitude = GNSS_altitude;

  // if range too low: continue
  if( (max_altitude - min_altitude) < MINIMUM_ALTITUDE_RANGE)
    return air_data;

  // elevation range triggering
  if( (max_altitude - min_altitude < MAXIMUM_ALTITUDE_RANGE) &&
      false == altitude_trigger.process(GNSS_altitude))
    return air_data;

  // if data points too rare: continue
  if (density_QFF_calculator.get_count() < 100)
    return air_data;

  // process last acquisition phase data
  linear_least_square_result<evaluation_type> result;
  density_QFF_calculator.evaluate(result);

//  Due to numeric effects, the variance has been observed
//  to be negative in some cases.
//  If this is the case: Throw away this result.
  if( ( result.variance_slope < 0) || ( result.variance_offset < 0))
      {
      density_QFF_calculator.reset();
      air_data.valid=false;
      return air_data;
      }

 if( (result.variance_slope < MAX_ALLOWED_SLOPE_VARIANCE) &&
     (result.variance_offset < MAX_ALLOWED_OFFSET_VARIANCE) )
    {
      air_data.QFF = (float)(result.y_offset);
      float density = 100.0f * (float)(result.slope) * -0.101936f; // div by -9.81f;

      float reference_altitude = density_QFF_calculator.get_mean_x() * 0.01f;
      float std_density =
	  reference_altitude * reference_altitude *   0.000000003547494f
	  -0.000115412739613f * reference_altitude +1.224096628212817f;
      air_data.density_correction = density / std_density;
      air_data.density_variance = result.variance_slope;
      air_data.valid = true;
    }

  max_altitude = min_altitude = GNSS_altitude;
  density_QFF_calculator.reset();

  return air_data;
}
