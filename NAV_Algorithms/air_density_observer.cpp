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
#include "NAV_tuning_parameters.h"
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

  // initial range setup
  if( min_altitude == ZERO)
    min_altitude = max_altitude = GNSS_altitude;

  // update elevation range
  if( GNSS_altitude > max_altitude)
    max_altitude = GNSS_altitude;

  if( GNSS_altitude < min_altitude)
    min_altitude = GNSS_altitude;

  // if range too low: continue
  if( (max_altitude - min_altitude) < MINIMUM_ALTITUDE_RANGE)
    return air_data;

  // elevation range triggering
  if( not altitude_trigger.process(GNSS_altitude)
      and (max_altitude - min_altitude < MAXIMUM_ALTITUDE_RANGE))
    return air_data;

  // if data points too rare: continue
  if (density_QFF_calculator.get_count() < 100)
    return air_data;

  // process last acquisition phase data
  linear_least_square_result<evaluation_type> result;
  bool result_valid = density_QFF_calculator.evaluate( result);

  air_data.QFF = (float)(result.y_offset);
  float density = 100.0f * (float)(result.slope) * - RECIP_GRAVITY;

  float reference_altitude = density_QFF_calculator.get_mean_x() * 0.01f;
  float std_density =
      reference_altitude * reference_altitude *   0.000000003547494f
      -0.000115412739613f * reference_altitude +1.224096628212817f;
  air_data.density_correction = density / std_density;

  air_data.valid = true;

  density_over_altitude_fit.add_value( reference_altitude, density);
  if( result.variance_slope < MAX_ALLOWED_SLOPE_VARIANCE * 0.333f)
    // double weight if precision high
    density_over_altitude_fit.add_value( reference_altitude, density);

  if( density_over_altitude_fit.get_count() > 6)
    density_over_altitude_fit.forget_older_data();

  if( density_over_altitude_fit.get_count() > 2)
    {
      linear_least_square_result< float> result;
      density_over_altitude_fit.evaluate( result);
      air_data.density_offset = result.y_offset;
      air_data.density_slope = result.slope;
    }

  max_altitude = min_altitude = GNSS_altitude;
  density_QFF_calculator.reset();

  return air_data;
}
