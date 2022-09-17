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
#include "embedded_math.h"
#include "windobserver.h"
#include "NAV_tuning_parameters.h"

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
  vario_averager_pressure( configuration( VARIO_TC)),
  vario_averager_GNSS( configuration( VARIO_TC)),
  windspeed_instant_observer( configuration( WIND_TC)),
  kinetic_energy_differentiator( 1.0f, 1.0f / 100.0f),
  speed_compensation_TAS( ZERO),
  vario_uncompensated_GNSS( ZERO),
  vario_uncompensated_pressure( ZERO),
#if VARIO_USE_GNSS_IAS_FUSION
  speed_compensation_fusioner( 0.998f),
#endif
  KalmanVario_GNSS( 0.0f, 0.0f, 0.0f, -9.81f),
  KalmanVario_pressure( 0.0f, 0.0f, 0.0f, -9.81f),
  specific_energy_differentiator( 1.0f, 1.0f / 100.0f),
  specific_energy(0.0f)
  {
  };
	void update
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
	    const float3vector &wind_average
	);

	void reset(float pressure_altitude, float GNSS_altitude);

	float get_pressure_altitude( void) const;

	float get_speed_compensation_TAS( void ) const
	{
		return speed_compensation_TAS;
	}

	float get_speed_compensation_INS( void ) const
	{
		return horizontal_speed_compensation_GNSS + vertical_speed_compensation_AHRS;
//		return horizontal_speed_compensation_GNSS; // todo patch
	}

	float get_vertical_speed_compensation( void ) const
	{
		return vertical_speed_compensation_AHRS;
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

	const float3vector & get_instant_wind( void ) const
	{
		return windspeed_instant_observer.get_output();
	}

	float get_effective_vertical_acceleration( void) const
	{
		return KalmanVario_GNSS.get_x( KalmanVario_PVA_t::ACCELERATION_OBSERVED);
	}

private:
	pt2<float,float> vario_averager_pressure;
	pt2<float,float> vario_averager_GNSS;
	wind_observer_t windspeed_instant_observer;

	differentiator<float,float>kinetic_energy_differentiator;

	float speed_compensation_TAS;
	float horizontal_speed_compensation_GNSS;
	float vertical_speed_compensation_AHRS;
	float vario_uncompensated_GNSS;
	float vario_uncompensated_pressure;

#if VARIO_USE_GNSS_IAS_FUSION
	HP_LP_fusion <float, float> speed_compensation_fusioner;
#endif
	KalmanVario_PVA_t KalmanVario_GNSS;
	KalmanVario_t KalmanVario_pressure;
	differentiator<float,float>specific_energy_differentiator;
	float specific_energy;
};

#endif /* FLIGHT_OBSERVER_H_ */
