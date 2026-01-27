/***********************************************************************//**
 * @file		variometer.h
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

#ifndef VARIOMETER_H_
#define VARIOMETER_H_

#include "system_configuration.h"
#include "GNSS.h"
#include "AHRS.h"
#include <differentiator.h>
#include "KalmanVario.h"
#include "KalmanVario_PVA.h"
#include "Kalman_V_A_Aoff_observer.h"
#include "embedded_math.h"
#include "NAV_tuning_parameters.h"
#include "HP_LP_fusion.h"

#if USE_HARDWARE_EEPROM	== 0
#include "EEPROM_emulation.h"
#endif

#include "pt2.h"
#include "HP_LP_fusion.h"
#include "delay_line.h"

//! this class is responsible for all glider flight data
class variometer_t
{
public:
  variometer_t( void)
  :
    vario_averager_pressure( FAST_SAMPLING_TIME / configuration( VARIO_P_TC)),
    vario_averager_GNSS( FAST_SAMPLING_TIME / configuration( VARIO_TC)),
    kinetic_energy_differentiator( 1.0f, FAST_SAMPLING_TIME),
    KalmanVario_GNSS( 0.0f, 0.0f, 0.0f, - GRAVITY),
    specific_energy_differentiator( 1.0f, FAST_SAMPLING_TIME),
#if USE_OLD_FASHIONED_PRESSURE_VARIO
    pressure_vario_differentiator( 1.0f, FAST_SAMPLING_TIME),
#else
    KalmanVario_pressure( 0.0f, 0.0f, 0.0f, - GRAVITY),
#endif
    Kalman_v_a_observer_N(),
    Kalman_v_a_observer_E(),
    GNSS_INS_speedcomp_fusioner(SPEED_COMPENSATION_FUSIONER_FEEDBACK),
    GNSS_availability_counter(0),
    vario_uncompensated_pressure( ZERO),
    speed_compensation_IAS( ZERO),
    speed_compensation_GNSS( 0.0f),
    vario_uncompensated_GNSS( ZERO),
    specific_energy(0.0f),
    speed_compensation_INS_GNSS_1(0.0f),
    speed_compensation_kalman_2(0.0f),
    speed_compensation_energy_3(0.0f),
    speed_compensation_projected_4(0.0f)
  {
  };

  void tune( void)
  {
    vario_averager_pressure.tune( FAST_SAMPLING_TIME / configuration( VARIO_P_TC));
    vario_averager_GNSS.tune( FAST_SAMPLING_TIME / configuration( VARIO_TC));
  }

    void update_at_100Hz
    (
	const float3vector &gnss_velocity,
	const float3vector &ahrs_acceleration,
	const float3vector &heading_vector,
	float GNSS_altitude,
	float pressure_altitude,
	float IAS,
	const float3vector &wind_average,
	bool GNSS_fix_avaliable
    );

    void reset(float pressure_altitude, float GNSS_altitude);

    float get_pressure_altitude( void) const;

  float
  get_speed_compensation (unsigned index) const
  {
    switch (index)
      {
      case 0:
	return speed_compensation_INS_GNSS_1;
	break;
      case 1:
	return speed_compensation_kalman_2;
	break;

      case 2:
	return speed_compensation_energy_3;
	break;
      default:
	return speed_compensation_projected_4;
	break;
      }
  }

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
	// filter systems for variometer
	pt2<float,float> vario_averager_pressure;
	pt2<float,float> vario_averager_GNSS;
	differentiator<float,float>kinetic_energy_differentiator;
	KalmanVario_PVA_t KalmanVario_GNSS;

	differentiator<float,float>specific_energy_differentiator;
#if USE_OLD_FASHIONED_PRESSURE_VARIO
	differentiator<float,float>pressure_vario_differentiator;
#else
	KalmanVario_t KalmanVario_pressure;
#endif
	Kalman_V_A_Aoff_observer_t Kalman_v_a_observer_N;
	Kalman_V_A_Aoff_observer_t Kalman_v_a_observer_E;
	HP_LP_fusion <float, float> GNSS_INS_speedcomp_fusioner;
	unsigned GNSS_availability_counter;

	// variometer-related signals
	float vario_uncompensated_pressure;
	float speed_compensation_IAS;
	float speed_compensation_GNSS;
	float vario_uncompensated_GNSS;
	float specific_energy;

	float speed_compensation_INS_GNSS_1;
	float speed_compensation_kalman_2;
	float speed_compensation_energy_3;
	float speed_compensation_projected_4;
};

#endif /* VARIOMETER_H_ */
