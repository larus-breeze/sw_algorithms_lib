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
void flight_observer_t::update_every_10ms (
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
      windspeed_decimator_100Hz_10Hz.respond( gnss_velocity - air_velocity);

      // The Kalman-filter-based uncompensated vario in NED-system reports negative if *climbing* !
      vario_uncompensated_GNSS = - KalmanVario_GNSS.update ( GNSS_negative_altitude, gnss_velocity.e[DOWN], ahrs_acceleration.e[DOWN]);

#if 1 // experimental: (INS-acceleration vector* GNSS-velocity) speed compensation
      air_velocity = gnss_velocity - wind_average;
      air_velocity.e[DOWN] = KalmanVario_GNSS.get_x( KalmanVario_PVA_t::VARIO);
      float3vector acceleration = ahrs_acceleration;
      acceleration.e[DOWN] = KalmanVario_GNSS.get_x( KalmanVario_PVA_t::ACCELERATION_OBSERVED);
      //      speed_compensation_GNSS = air_velocity * acceleration * RECIP_GRAVITY;
      float speed_compensation_INS_GNSS = air_velocity * acceleration * RECIP_GRAVITY;
#endif
#if 1 // horizontal kalman filter for velocity and acceleration
      Kalman_v_a_observer_N.update(gnss_velocity.e[NORTH] - wind_average.e[NORTH], ahrs_acceleration.e[NORTH]);
      Kalman_v_a_observer_E.update(gnss_velocity.e[EAST]  - wind_average.e[EAST],  ahrs_acceleration.e[EAST]);

      float speed_compensation_kalman = (
		Kalman_v_a_observer_N.get_x(Kalman_V_A_observer_t::VELOCITY) * Kalman_v_a_observer_N.get_x(Kalman_V_A_observer_t::ACCELERATION) +
		Kalman_v_a_observer_E.get_x(Kalman_V_A_observer_t::VELOCITY) * Kalman_v_a_observer_E.get_x(Kalman_V_A_observer_t::ACCELERATION) +
		KalmanVario_GNSS.get_x( KalmanVario_PVA_t::VARIO)            * KalmanVario_GNSS.get_x( KalmanVario_PVA_t::ACCELERATION_OBSERVED)
	  ) * RECIP_GRAVITY;

      specific_energy =
	  (
	      SQR( gnss_velocity.e[NORTH] - wind_average.e[NORTH]) +
	      SQR( gnss_velocity.e[EAST]  - wind_average.e[EAST])  +
	      SQR( gnss_velocity.e[DOWN]) * VERTICAL_ENERGY_TUNING_FACTOR
	   )  * ONE_DIV_BY_GRAVITY_TIMES_2;

      // blending of three mechanisms for speed-compensation
      speed_compensation_GNSS = GNSS_INS_speedcomp_fusioner.respond( 0.5 * (speed_compensation_kalman + speed_compensation_INS_GNSS), specific_energy_differentiator.respond(specific_energy));
#endif
#if 0 // speed-compensation using differentiation of kinetic energy in air-system
      specific_energy = specific_energy * 0.9f + 0.1f *  // primitive PT1 smoothing for upsampling
	  (
	      SQR( gnss_velocity.e[NORTH] - wind_average.e[NORTH]) +
	      SQR( gnss_velocity.e[EAST]  - wind_average.e[EAST])  +
	      SQR( gnss_velocity.e[DOWN]) * VERTICAL_ENERGY_TUNING_FACTOR
	   );

      speed_compensation_GNSS = specific_energy_differentiator.respond(specific_energy) * ONE_DIV_BY_GRAVITY_TIMES_2;
#endif
      vario_averager_GNSS.respond( vario_uncompensated_GNSS + speed_compensation_GNSS);
    }
}

void flight_observer_t::reset(float pressure_negative_altitude, float GNSS_negative_altitude)
{
  KalmanVario_GNSS.reset( GNSS_negative_altitude, -9.81f);
  KalmanVario_pressure.reset( pressure_negative_altitude, -9.81f);
}
