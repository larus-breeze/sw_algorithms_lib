/***********************************************************************//**
 * @file		flight_observer.cpp
 * @brief		maintains important derived data for gliders
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

#include "system_configuration.h"
#include "flight_observer.h"
#include "embedded_math.h"

#define ONE_DIV_BY_GRAVITY_TIMES_2 0.0509684f
#define RECIP_GRAVITY 0.1094f

//! calculate instant windspeed and variometer data, update @ 100 Hz
void flight_observer_t::update (
    const float3vector &gnss_velocity,
    const float3vector &gnss_acceleration,
    const float3vector &ahrs_acceleration,
    const float3vector &heading_vector,
    float GNSS_negative_altitude,
    float pressure_altitude,
    float TAS,
    float IAS,
    circle_state_t circle_state,
    const float3vector &wind_average
  )
{
  vario_uncompensated_pressure = KalmanVario_pressure.update ( pressure_altitude, ahrs_acceleration.e[DOWN]);
  speed_compensation_TAS = kinetic_energy_differentiator.respond( IAS * IAS * ONE_DIV_BY_GRAVITY_TIMES_2);
  vario_averager_pressure.respond( speed_compensation_TAS  - vario_uncompensated_pressure); 	// -> positive on positive energy gain

  if( isnan( gnss_acceleration.e[NORTH])) // no GNSS fix by now
    {
      // workaround for no GNSS fix: maintain GNSS vario with pressure data
      vario_uncompensated_GNSS = - KalmanVario_GNSS.update ( pressure_altitude, ZERO, ahrs_acceleration.e[DOWN]); // todo MURX: what can we do without vertical speed information ?
      // this time use pressure vario
      vario_averager_GNSS.respond( speed_compensation_TAS  - vario_uncompensated_pressure);
    }
  else
    {
      float3vector air_velocity = heading_vector * TAS;
      windspeed_instant_observer.update( gnss_velocity - air_velocity, heading_vector, circle_state);

      // non TEC compensated vario in NED-system, reports negative if *climbing* !
      vario_uncompensated_GNSS = - KalmanVario_GNSS.update ( GNSS_negative_altitude, gnss_velocity.e[DOWN], ahrs_acceleration.e[DOWN]);

#if VARIO_USE_SQUARED_VELOCITY // use squared absolute velocity for speed-compensation
      specific_energy = specific_energy * 0.9f + 0.1f *  // primitive PT1 smoothing for upsampling
	  (
	  SQR( gnss_velocity.e[NORTH] - wind_average.e[NORTH]) +
	  SQR( gnss_velocity.e[EAST]  - wind_average.e[EAST])  +
	  SQR( KalmanVario_GNSS.get_x(KalmanVario_PVA_t::VARIO)
	       ));

      float speed_compensation = specific_energy_differentiator.respond(specific_energy) * ONE_DIV_BY_GRAVITY_TIMES_2;
      vertical_speed_compensation_AHRS = speed_compensation;
      vario_averager_GNSS.respond( vario_uncompensated_GNSS + speed_compensation);
#else

      vertical_speed_compensation_AHRS = KalmanVario_GNSS.get_x(KalmanVario_PVA_t::VARIO) * KalmanVario_GNSS.get_x(KalmanVario_PVA_t::ACCELERATION_OBSERVED)
				    * RECIP_GRAVITY;

      horizontal_speed_compensation_GNSS =
    		  (
    		  ((gnss_velocity.e[NORTH] - wind_average.e[NORTH]) * gnss_acceleration.e[NORTH]) +
    		  ((gnss_velocity.e[EAST]  - wind_average.e[EAST])  * gnss_acceleration.e[EAST])
    		  ) * RECIP_GRAVITY;

#if VARIO_USE_GNSS_IAS_FUSION // use highpass+lowpass fusion-filter to combine GNSS and TAS speed compensation signals
      float fusioned = speed_compensation_fusioner.respond( horizontal_speed_compensation_GNSS + vertical_speed_compensation_AHRS, speed_compensation_TAS);
      vertical_speed_compensation_AHRS = fusioned; // todo patch
      vario_averager_GNSS.respond( fusioned + vario_uncompensated_GNSS);
#else
      vario_averager_GNSS.respond( horizontal_speed_compensation_GNSS + vertical_speed_compensation_AHRS + vario_uncompensated_GNSS);
#endif  // fusion filter
#endif  // use squared absolute velocity
    }
}

void flight_observer_t::reset(float pressure_negative_altitude, float GNSS_negative_altitude)
{
  KalmanVario_GNSS.reset( GNSS_negative_altitude, -9.81f);
  KalmanVario_pressure.reset( pressure_negative_altitude, -9.81f);
}
