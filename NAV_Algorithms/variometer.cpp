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

#include <variometer.h>
#include "system_configuration.h"
#include "embedded_math.h"

#define ONE_DIV_BY_GRAVITY_TIMES_2 0.0509684f
#define RECIP_GRAVITY 0.1094f

//! calculate instant windspeed and variometer data, update @ 100 Hz
void variometer_t::update_every_10ms (
    const float3vector &gnss_velocity,
    const float3vector &gnss_acceleration,
    const float3vector &ahrs_acceleration,
    const float3vector &heading_vector,
    float GNSS_negative_altitude,
    float pressure_altitude,
    float TAS,
    float IAS,
    circle_state_t circle_state,
    const float3vector &wind_average,
    bool GNSS_fix_avaliable
  )
{
  vario_uncompensated_pressure = KalmanVario_pressure.update (
      pressure_altitude, ahrs_acceleration.e[DOWN]);
  speed_compensation_IAS = kinetic_energy_differentiator.respond (
      IAS * IAS * ONE_DIV_BY_GRAVITY_TIMES_2);
  vario_averager_pressure.respond (
      speed_compensation_IAS - vario_uncompensated_pressure); // -> positive on positive energy gain

  if (!GNSS_fix_avaliable)
    {
      // workaround for no GNSS fix: maintain GNSS vario with pressure data
      vario_uncompensated_GNSS = vario_uncompensated_pressure;
      speed_compensation_GNSS = speed_compensation_IAS;
      vario_averager_GNSS.respond (
	  speed_compensation_IAS - vario_uncompensated_pressure);
    }
  else
    {
      // calculation of all the prerequisites:

      // run the 100 Hz -> 10 Hz wind speed decimation filter
      float3vector ahrs_instant_air_velocity = heading_vector * TAS;
      windspeed_decimator_100Hz_10Hz.respond ( gnss_velocity - ahrs_instant_air_velocity);

      // The Kalman-Vario
      // The Kalman-filter-based un-compensated variometer in NED-system reports negative if *climbing* !
      vario_uncompensated_GNSS = -KalmanVario_GNSS.update ( GNSS_negative_altitude, gnss_velocity.e[DOWN], ahrs_acceleration.e[DOWN]);

      // 3d acceleration from the AHRS and from the vertical Kalman filter
      float3vector acceleration = ahrs_acceleration;
      // the vertical component comes from the Kalman Vario, effective value without gravitation
      acceleration.e[DOWN] = KalmanVario_GNSS.get_x ( KalmanVario_PVA_t::ACCELERATION_OBSERVED);

      // horizontal kalman filter for velocity and acceleration in the air- (not ground) system
      Kalman_v_a_observer_N.update ( gnss_velocity.e[NORTH] - wind_average.e[NORTH], ahrs_acceleration.e[NORTH]);
      Kalman_v_a_observer_E.update ( gnss_velocity.e[EAST]  - wind_average.e[EAST],  ahrs_acceleration.e[EAST]);

      // compute our kinetic energy in the air-system
      specific_energy = (
            SQR( gnss_velocity.e[NORTH] - wind_average.e[NORTH])
	  + SQR( gnss_velocity.e[EAST]  - wind_average.e[EAST])
	  + SQR( gnss_velocity.e[DOWN]))
	  * ONE_DIV_BY_GRAVITY_TIMES_2;

      // speed-compensation type 1 = scalar product( air_velocity , acceleration) / g;
      speed_compensation_INS_GNSS_1 = ahrs_instant_air_velocity * acceleration * RECIP_GRAVITY;

      // speed compensation type 2 = air velocity * acceleration , both Kalman-filtered
      speed_compensation_kalman_2 =
	   (Kalman_v_a_observer_N.get_x ( Kalman_V_A_Aoff_observer_t::VELOCITY) * Kalman_v_a_observer_N.get_x ( Kalman_V_A_Aoff_observer_t::ACCELERATION)
	  + Kalman_v_a_observer_E.get_x ( Kalman_V_A_Aoff_observer_t::VELOCITY) * Kalman_v_a_observer_E.get_x ( Kalman_V_A_Aoff_observer_t::ACCELERATION)
	  + KalmanVario_GNSS.get_x (KalmanVario_PVA_t::VARIO) * KalmanVario_GNSS.get_x ( KalmanVario_PVA_t::ACCELERATION_OBSERVED))
	  * RECIP_GRAVITY;

      // speed compensation type 3 comes from the derivative of the specific energy
      speed_compensation_energy_3 = specific_energy_differentiator.respond ( specific_energy);

      // speed-compensation type 4 is the product of acceleration and velocity, both calculated along the heading axis
      float3vector kalman_air_velocity;
      kalman_air_velocity.e[NORTH] = Kalman_v_a_observer_N.get_x ( Kalman_V_A_Aoff_observer_t::VELOCITY);
      kalman_air_velocity.e[EAST]  = Kalman_v_a_observer_E.get_x ( Kalman_V_A_Aoff_observer_t::VELOCITY);
      kalman_air_velocity.e[DOWN]  = KalmanVario_GNSS.get_x (  	   KalmanVario_PVA_t::VARIO);
      float air_velocity_projected = kalman_air_velocity * heading_vector;

      acceleration.e[NORTH] = Kalman_v_a_observer_N.get_x ( Kalman_V_A_Aoff_observer_t::ACCELERATION);
      acceleration.e[EAST]  = Kalman_v_a_observer_E.get_x ( Kalman_V_A_Aoff_observer_t::ACCELERATION);
      acceleration.e[DOWN]  = KalmanVario_GNSS.get_x (	    KalmanVario_PVA_t::ACCELERATION_OBSERVED);
      float acceleration_projected = acceleration * heading_vector;

      speed_compensation_projected_4 = air_velocity_projected * acceleration_projected * RECIP_GRAVITY;

      // blending of three mechanisms for speed-compensation
      speed_compensation_GNSS = GNSS_INS_speedcomp_fusioner.respond( 0.3333333f * (speed_compensation_INS_GNSS_1 + speed_compensation_kalman_2 + speed_compensation_projected_4), speed_compensation_energy_3);
      vario_averager_GNSS.respond ( vario_uncompensated_GNSS + speed_compensation_GNSS);
    }
}

void variometer_t::reset(float pressure_negative_altitude, float GNSS_negative_altitude)
{
  KalmanVario_GNSS.reset( GNSS_negative_altitude, -9.81f);
  KalmanVario_pressure.reset( pressure_negative_altitude, -9.81f);
}
