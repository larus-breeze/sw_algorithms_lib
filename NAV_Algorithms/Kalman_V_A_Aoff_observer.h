/***********************************************************************//**
 * @file		KalmanVario.h
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

#ifndef APPLICATION_VA_AOFF_OBSERVER_H_
#define APPLICATION_VA_AOFF_OBSERVER_H_

#include "embedded_memory.h"
#include "embedded_math.h"
#include <stdint.h>
#include "system_configuration.h"

/**
 * @brief Kalman-filter-based sensor fusion observer for horizontal movement
 *
 * Blend vertical acceleration inclusive gravity with altitude data
 * to provide UN-compensated variometer plus vertical net acceleration w/o gravity
 */
class Kalman_V_A_Aoff_observer_t
{
private:

  // constants
  enum
  {
    N = 3,  //!< size of state vector x = { altitude, vario, vertical-acceleration, acceleration-offset }
    L = 2  //!< number of measurement channels = { altitude, vertical_acceleration_measurement }
  };
  static constexpr float Ta = 0.01f; 			//!< sampling rate
  static ROM float Gain[N][L];				//!< Pre-computed Kalman Gain

  // variables
  float x[N];	//!< state vector: altitude, vario, acceleration, acceleration offset

public:
  typedef enum// state vector components
  {
    VELOCITY, ACCELERATION, ACCELERATION_OFFSET
  }  state;

  Kalman_V_A_Aoff_observer_t ( float v=ZERO, float a=ZERO)
    : x{ v, a, 0.0f}
  {}

  void update( const float velocity, const float acceleration);

  inline float get_x( state index) const
  {
      return x[index];
  };
};

#endif /* APPLICATION_VA_OBSERVER_H_ */
