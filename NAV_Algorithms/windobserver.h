/***********************************************************************//**
 * @file		windobserver.h
 * @brief		smart wind observer changing it's behavior dependent on flight pattern
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

#ifndef WINDOBSERVER_H_
#define WINDOBSERVER_H_

#include "float3vector.h"
#include "AHRS.h"

//! Specialized decimating filter for 3d wind data
class wind_observer_t
{
public:
  wind_observer_t (float beta_design)
  : decimating_counter(DECIMATION)
  {
//  beta_max = exp( - T_sample / Tau );
    beta_max = 0.982f; // todo implement correct parameter setup
    stage_1_N = ZERO;
    stage_1_E = ZERO;
  }
  const float3vector & get_output( void) const
  {
    return present_output;
  }

  void update (const float3vector &current_value, const float3vector & heading_vector, circle_state_t state)
  {
    stage_1_N = stage_1_N * beta_max + current_value.e[NORTH] * ( ONE - beta_max);
    stage_1_E = stage_1_E * beta_max + current_value.e[EAST]  * ( ONE - beta_max);
    stage_1_D = stage_1_D * beta_max + current_value.e[DOWN]  * ( ONE - beta_max);

    --decimating_counter;
    if( decimating_counter == 0)
      {
	decimating_counter = DECIMATION;

	float alpha_N, beta_N, alpha_E, beta_E;

	if( state == STRAIGHT_FLIGHT)
	  {
	    beta_N = beta_E = beta_max;
	    alpha_N = alpha_E = ONE - beta_max;
	  }
	else
	  {
	    alpha_N = (ONE - beta_max) * SQR( heading_vector.e[NORTH]);
	    alpha_E = (ONE - beta_max) * SQR( heading_vector.e[EAST]);
	    beta_N = ONE - alpha_N;
	    beta_E = ONE - alpha_E;
	  }
	present_output.e[NORTH] = present_output.e[NORTH] * alpha_N + stage_1_N * beta_N;
	present_output.e[EAST]  = present_output.e[EAST]  * alpha_E + stage_1_E * beta_E;
	present_output.e[DOWN]  = present_output.e[DOWN]  * (ONE - beta_max) + stage_1_D * beta_max;
      }

    // catch denormalized data
    if( !isnormal( present_output.e[EAST]) || !isnormal( present_output.e[NORTH]) || !isnormal( present_output.e[DOWN]))
      present_output.e[NORTH] = present_output.e[EAST] = present_output.e[DOWN] = ZERO;
  }

 private:
  enum { DECIMATION = 55}; // two stages -> decimation 1/100s -> 30 s; 55 = sqrt(3000)
  float stage_1_N; //!< North component filter feedback
  float stage_1_E; //!< East component filter feedback
  float stage_1_D; //!< Down component filter feedback
  uint32_t decimating_counter;
  float beta_max; //!< nominator coefficient as configured

  float3vector present_output; // maintained to save computing time
};

#endif /* WINDOBSERVERT_H_ */
