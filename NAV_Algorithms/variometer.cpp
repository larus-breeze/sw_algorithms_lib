/***********************************************************************//**
 * @file		variometer.cpp
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
#define GNSS_FIX_USAGE_DELAY 3000 // 30s
#define GNSS_FIX_EVALUATION_DELAY 200 // 2s

//! calculate variometer data, update @ 100 Hz
void variometer_t::update_at_100Hz (
    const float3vector &gnss_velocity,
    const float3vector &ahrs_acceleration,
    const float3vector &heading_vector,
    float GNSS_negative_altitude,
    float pressure_altitude,
    float IAS,
    const float3vector &speed_compensator_wind,
    bool GNSS_fix_available
  )
{
  if( GNSS_fix_available)
      {
	if( GNSS_availability_counter < GNSS_FIX_USAGE_DELAY)
	  ++GNSS_availability_counter;
      }
    else
	GNSS_availability_counter = 0;

#if USE_OLD_FASHIONED_PRESSURE_VARIO
  vario_uncompensated_pressure = pressure_vario_differentiator.respond( pressure_altitude);
#else
  vario_uncompensated_pressure = KalmanVario_pressure.update (
      pressure_altitude, ahrs_acceleration[DOWN]);
#endif
  speed_compensation_IAS = kinetic_energy_differentiator.respond (
      IAS * IAS * ONE_DIV_BY_GRAVITY_TIMES_2);
  vario_averager_pressure.respond (
      speed_compensation_IAS - vario_uncompensated_pressure); // -> positive on positive energy gain

  // prepare GNSS vario data if we have a stable sat fix
  if ( GNSS_availability_counter > GNSS_FIX_EVALUATION_DELAY)
    {
      // The Kalman-filter-based un-compensated variometer in NED-system reports negative if *climbing* !
      KalmanVario_GNSS.update ( GNSS_negative_altitude, gnss_velocity[DOWN], ahrs_acceleration[DOWN]);

      // 3d acceleration from the AHRS and from the vertical Kalman filter
      float3vector acceleration = ahrs_acceleration;
      // the vertical component comes from the Kalman Vario, effective value without gravitation
      acceleration[DOWN] = KalmanVario_GNSS.get_x ( KalmanVario_PVA_t::ACCELERATION_OBSERVED);

      // horizontal kalman filter for velocity and acceleration in the air- (not ground) system
      Kalman_v_a_observer_N.update ( gnss_velocity[NORTH] - speed_compensator_wind[NORTH], ahrs_acceleration[NORTH]);
      Kalman_v_a_observer_E.update ( gnss_velocity[EAST]  - speed_compensator_wind[EAST],  ahrs_acceleration[EAST]);

      // compute our kinetic energy in the air-system
      specific_energy = (
            SQR( gnss_velocity[NORTH] - speed_compensator_wind[NORTH])
	  + SQR( gnss_velocity[EAST]  - speed_compensator_wind[EAST])
	  + SQR( gnss_velocity[DOWN]))
	  * ONE_DIV_BY_GRAVITY_TIMES_2;

      // speed-compensation type 1 = scalar product( air_velocity , acceleration) / g;
      speed_compensation_INS_GNSS_1 = ( gnss_velocity - speed_compensator_wind) * acceleration * RECIP_GRAVITY;

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
      kalman_air_velocity[NORTH] = Kalman_v_a_observer_N.get_x ( Kalman_V_A_Aoff_observer_t::VELOCITY);
      kalman_air_velocity[EAST]  = Kalman_v_a_observer_E.get_x ( Kalman_V_A_Aoff_observer_t::VELOCITY);
      kalman_air_velocity[DOWN]  = KalmanVario_GNSS.get_x (  	   KalmanVario_PVA_t::VARIO);
      float air_velocity_projected = kalman_air_velocity * heading_vector;

      acceleration[NORTH] = Kalman_v_a_observer_N.get_x ( Kalman_V_A_Aoff_observer_t::ACCELERATION);
      acceleration[EAST]  = Kalman_v_a_observer_E.get_x ( Kalman_V_A_Aoff_observer_t::ACCELERATION);
      acceleration[DOWN]  = KalmanVario_GNSS.get_x (	    KalmanVario_PVA_t::ACCELERATION_OBSERVED);
      float acceleration_projected = acceleration * heading_vector;

      speed_compensation_projected_4 = air_velocity_projected * acceleration_projected * RECIP_GRAVITY;

      // blending of three mechanisms for speed-compensation
      GNSS_INS_speedcomp_fusioner.respond( 0.3333333f * (speed_compensation_INS_GNSS_1 + speed_compensation_kalman_2 + speed_compensation_projected_4), speed_compensation_energy_3);
//      GNSS_INS_speedcomp_fusioner.respond( 0.5 * (speed_compensation_INS_GNSS_1 + speed_compensation_kalman_2), speed_compensation_energy_3);
      vario_averager_GNSS.respond ( vario_uncompensated_GNSS + speed_compensation_GNSS);
    }

  // soft switch between pressure-based emergency vario and generic GNSS vario
  if ( GNSS_availability_counter < GNSS_FIX_USAGE_DELAY)
    {
      // workaround for no GNSS fix: maintain GNSS vario with pressure data
      vario_uncompensated_GNSS = vario_uncompensated_pressure;
      speed_compensation_GNSS = speed_compensation_IAS;
      vario_averager_GNSS.respond ( speed_compensation_IAS - vario_uncompensated_pressure);
    }
  else
    {
      // use GNSS vario data for output
      vario_uncompensated_GNSS = -KalmanVario_GNSS.get_x( KalmanVario_PVA_t::VARIO);
//      speed_compensation_GNSS = GNSS_INS_speedcomp_fusioner.get_value();
//      speed_compensation_GNSS = 0.5 * (speed_compensation_INS_GNSS_1 + speed_compensation_kalman_2);
      speed_compensation_GNSS = 0.3333333f * (speed_compensation_INS_GNSS_1 + speed_compensation_kalman_2 + speed_compensation_projected_4);
      vario_averager_GNSS.respond ( vario_uncompensated_GNSS + speed_compensation_GNSS);
    }
}

void variometer_t::reset(float pressure_negative_altitude, float GNSS_negative_altitude)
{
  KalmanVario_GNSS.reset( GNSS_negative_altitude, -9.81f);
  KalmanVario_pressure.reset( pressure_negative_altitude, -9.81f);
}
