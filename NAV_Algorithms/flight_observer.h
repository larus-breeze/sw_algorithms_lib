/***********************************************************************//**
 * @file		flight_observer.h
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

#ifndef FLIGHT_OBSERVER_H_
#define FLIGHT_OBSERVER_H_

#include "system_configuration.h"
#include "GNSS.h"
#include <differentiator.h>
#include "KalmanVario.h"
#include "KalmanVario_PVA.h"
#include "Kalman_V_A_Aoff_observer.h"
#include "embedded_math.h"
#include "windobserver.h"
#include "NAV_tuning_parameters.h"
#include "HP_LP_fusion.h"

#if USE_HARDWARE_EEPROM	== 0
#include "EEPROM_emulation.h"
#endif

#include "pt2.h"
#include "HP_LP_fusion.h"
#include "delay_line.h"

//! this class is responsible for all glider flight data
class flight_observer_t
{
public:
  flight_observer_t( void)
  :
  vario_averager_pressure( FAST_SAMPLING_TIME / configuration( VARIO_TC)),
  vario_averager_GNSS( FAST_SAMPLING_TIME / configuration( VARIO_TC)),
  windspeed_decimator_100Hz_10Hz( FAST_SAMPLING_TIME),
  kinetic_energy_differentiator( 1.0f, FAST_SAMPLING_TIME),
  speed_compensation_IAS( ZERO),
  vario_uncompensated_GNSS( ZERO),
  vario_uncompensated_pressure( ZERO),
  KalmanVario_GNSS( 0.0f, 0.0f, 0.0f, - GRAVITY),
  KalmanVario_pressure( 0.0f, 0.0f, 0.0f, - GRAVITY),
  specific_energy_differentiator( 1.0f, FAST_SAMPLING_TIME),
  GNSS_INS_speedcomp_fusioner(SPEED_COMPENSATION_FUSIONER_FEEDBACK),
  specific_energy(0.0f),
  vertical_energy_tuning_factor(configuration( VETF)),
  speed_compensation_GNSS( 0.0F)
  {
  };
	void update_every_10ms
	(
	    const float3vector &gnss_velocity,
	    const float3vector &gnss_acceleration,
	    const float3vector &ahrs_acceleration,
	    const float3vector &heading_vector,
	    float GNSS_altitude,
	    float pressure_altitude,
	    float TAS,
	    float IAS,
	    circle_state_t circle_state,
	    const float3vector &wind_average,
	    bool GNSS_fix_avaliable
	);

	void reset(float pressure_altitude, float GNSS_altitude);

	float get_pressure_altitude( void) const;

	float get_speed_compensation_IAS( void ) const
	{
		return speed_compensation_IAS;
	}

	float get_speed_compensation_GNSS( void ) const
	{
		return speed_compensation_GNSS;
	}

	float get_vario_uncompensated_GNSS( void ) const
	{
		return vario_uncompensated_GNSS;
	}

	float get_vario_pressure( void ) const
	{
		return (float)( vario_averager_pressure.get_output());
	}

	float get_vario_GNSS( void ) const
	{
		return vario_averager_GNSS.get_output();
	}

	float3vector get_instant_wind( void ) const
	{
		return windspeed_decimator_100Hz_10Hz.get_output();
	}

	float get_filtered_GNSS_altitude( void) const
	{
	  // the Kalman filter operates on *negative* altitude
		return - KalmanVario_GNSS.get_x( KalmanVario_PVA_t::ALTITUDE);
	}

	float get_effective_vertical_acceleration( void) const
	{
		return KalmanVario_GNSS.get_x( KalmanVario_PVA_t::ACCELERATION_OBSERVED);
	}

private:
	pt2<float3vector,float> windspeed_decimator_100Hz_10Hz;

	// filter systems for variometer
	pt2<float,float> vario_averager_pressure;
	pt2<float,float> vario_averager_GNSS;
	differentiator<float,float>kinetic_energy_differentiator;
	KalmanVario_PVA_t KalmanVario_GNSS;
	KalmanVario_t KalmanVario_pressure;
	differentiator<float,float>specific_energy_differentiator;
	Kalman_V_A_Aoff_observer_t Kalman_v_a_observer_N;
	Kalman_V_A_Aoff_observer_t Kalman_v_a_observer_E;
	HP_LP_fusion <float, float> GNSS_INS_speedcomp_fusioner;

	// variometer-related signals
	float vario_uncompensated_pressure;
	float speed_compensation_IAS;
	float speed_compensation_GNSS;
	float vario_uncompensated_GNSS;
	float specific_energy;
	float vertical_energy_tuning_factor;
};

#endif /* FLIGHT_OBSERVER_H_ */
