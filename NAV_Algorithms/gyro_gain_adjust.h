/***********************************************************************//**
 * @file		gyro_gain_adjust.h
 * @brief		gyro gain fine-tuning
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 8.10.2025 Dr. Klaus Schaefer. All rights reserved.
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

#ifndef NAV_ALGORITHMS_GYRO_GAIN_ADJUST_H_
#define NAV_ALGORITHMS_GYRO_GAIN_ADJUST_H_

#include "Linear_Least_Square_Fit.h"

class gyro_gain_adjust
{
public:
  gyro_gain_adjust ()
    : samples_right(0),
      samples_left(0),
      fine_tuned_gain( 1.0f),
      fine_tuned_gain_valid(false)
  {

  }
  void learn( double value, double correction)
  {
    if( value > 0.0f)
      ++samples_right;
    else
      ++samples_left;
    llsf.add_value( value, correction);
  }
  bool calculate( void)
  {
    if( samples_right < MIN_SAMPLES || samples_left < MIN_SAMPLES)
      return false;

    linear_least_square_result <double> result;
    llsf.evaluate( result);

    samples_right = samples_left = 0;
    llsf.reset();

    fine_tuned_gain = result.slope + 1.0f;
//    fine_tuned_gain = 1.0f;
    fine_tuned_gain_valid = true;
    return true;
  }
  bool update_gain( double &gain) const
  {
    if( fine_tuned_gain_valid)
      gain = fine_tuned_gain;
//    gain = (9 * gain + fine_tuned_gain) * 0.1f;
    return fine_tuned_gain_valid;
  }
private:
  enum{ MIN_SAMPLES = 6000};
  unsigned samples_right;
  unsigned samples_left;
  linear_least_square_fit <double> llsf;
  double fine_tuned_gain;
  bool fine_tuned_gain_valid;
};

#endif /* NAV_ALGORITHMS_GYRO_GAIN_ADJUST_H_ */
