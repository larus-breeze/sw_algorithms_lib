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
#include "NAV_tuning_parameters.h"

#include "stdio.h" // todo patch

density_data air_density_observer_t::feed_metering( float pressure, float GNSS_altitude)
{
  density_data air_data;

  pressure_decimation_filter.respond( pressure);

  --decimation_counter;
  if( decimation_counter > 0)
    return air_data;
  decimation_counter = AIR_DENSITY_DECIMATION;

  density_QFF_calculator.add_observation_point( GNSS_altitude * altitude_scale_factor, pressure_decimation_filter.get_output() * pressure_scale_factor);

#if 1
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
#else

  ++sample_counter;
  if(( sample_counter % 1000) != 0)
    return air_data;

#endif
  // process last acquisition phase data
  density_QFF_calculator.calculate();

  air_data.density_offset = density_QFF_calculator.get_coefficient( 1) / -9.81;
  air_data.density_slope = density_QFF_calculator.get_coefficient( 2) * -2.0f / 9.81f;
  air_data.variance = density_QFF_calculator.get_coefficient( -1);

  printf("\nrel error: %e\n", SQRT( air_data.variance) / air_data.density_slope);

//  if( true or air_data.variance < 1e10) // todo patch
  if( air_data.density_offset < 1.3f and air_data.density_offset > 1.0f)
    air_data.valid = true;
  else
    air_data.valid = false;

  max_altitude = min_altitude = GNSS_altitude;
  density_QFF_calculator.reset(); // todo patch

  return air_data;
}
