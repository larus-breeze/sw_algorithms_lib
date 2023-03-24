/***********************************************************************//**
 * @file		KalmanVario.cpp
 * @brief		Kalman filter for variometer (interface)
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

#include <Kalman_V_A_Aoff_observer.h>

ROM float Kalman_V_A_Aoff_observer_t::Gain[N][L]= //!< Kalman Gain for 100Hz sampling rate
    {
	   0.045525746461218,   0.005179336297606,
	   0.102816402740272,   0.906791781469036,
	  -0.097637066442666,   0.001591461803718
    };

void Kalman_V_A_Aoff_observer_t::update( const float velocity, const float acceleration)
{
  // predict x[] by propagating it through the system model
  float x_est_0 = x[0] + Ta * x[1];
  float x_est_1 = x[1];
  float x_est_2 = x[2];

  float innovation_v = velocity     - x_est_0;
  float innovation_a = acceleration - x_est_1 - x_est_2;

  // x[] correction
  x[0] = x_est_0 + Gain[0][0] * innovation_v + Gain[0][1] * innovation_a;
  x[1] = x_est_1 + Gain[1][0] * innovation_v + Gain[1][1] * innovation_a;
  x[2] = x_est_2 + Gain[2][0] * innovation_v + Gain[2][1] * innovation_a;
}
