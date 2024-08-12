/***********************************************************************//**
 * @file		KalmanVario_PVA.cpp
 * @brief		Kalman filter for variometer based on GNSS/INS data
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

#include <KalmanVario_PVA.h>

ROM float KalmanVario_PVA_t::Gain[N][L]= //!< Kalman Gain for 100Hz sampling rate
    {
	   0.014626994999089,   0.008222803848244,  -0.000024520221770,
	   0.018501308658548,   0.034426098147065,   0.005245193067348,
	   0.014936286742175,   0.066922555154085,   0.907003973713478,
	  -0.014960806963946,  -0.064591358235264,   0.001394841262837

    };

float KalmanVario_PVA_t::update( const float altitude, const float velocity, const float acceleration)
{
  // predict x[] by propagating it through the system model
  float x_est_0 = x[0] + Ta * x[1] + Ta_s_2 * x[2];
  float x_est_1 = x[1] + Ta * x[2];
  float x_est_2 = x[2];
  float x_est_3 = x[3];

  float innovation_x = altitude     - x_est_0;
  float innovation_v = velocity     - x_est_1;
  float innovation_a = acceleration - x_est_2 - x_est_3;

  // x[] correction
  x[0] = x_est_0 + Gain[0][0] * innovation_x + Gain[0][1] * innovation_v + Gain[0][2] * innovation_a;
  x[1] = x_est_1 + Gain[1][0] * innovation_x + Gain[1][1] * innovation_v + Gain[1][2] * innovation_a;
  x[2] = x_est_2 + Gain[2][0] * innovation_x + Gain[2][1] * innovation_v + Gain[2][2] * innovation_a;
  x[3] = x_est_3 + Gain[3][0] * innovation_x + Gain[3][1] * innovation_v + Gain[3][2] * innovation_a;

  return x[1]; // return velocity
}
