/***********************************************************************//**
 * @file		compass_calibrator_3D.h
 * @brief		induction sensor calibration and magnetic disturbance compensation
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 20.8.2024 Dr. Klaus Schaefer. All rights reserved.
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

#ifndef NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_
#define NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_

#include "embedded_memory.h"
#include "embedded_math.h"
#include "float3vector.h"
#include "quaternion.h"

//! 3 dimensional magnetic calibration and error compensation mechanism
class compass_calibrator_3D
{
public:
  enum { AXES=3, PARAMETERS=11, OBSERVATIONS=22};

  struct compass_calibrator_3D_data_t
  {
    float target_vector[AXES][OBSERVATIONS];
    float observation_matrix[AXES][OBSERVATIONS][PARAMETERS];

    float temporary_solution_matrix[PARAMETERS][PARAMETERS];
    float transposed_matrix[PARAMETERS][OBSERVATIONS];
    float matrix_to_be_inverted_data[PARAMETERS][PARAMETERS];
    float solution_mapping_data[PARAMETERS][OBSERVATIONS];
  };

  compass_calibrator_3D( compass_calibrator_3D_data_t * _data)
    : data (_data),
      calibration_successful(false)
  {
    start_learning();
  }

  void start_learning( void)
  {
    for( unsigned i=0; i<OBSERVATIONS; ++i)
      heading_sector_error[i]=1e20f;
  }

  bool learn (const float3vector &observed_induction,const float3vector &expected_induction, const quaternion<float> &q, bool turning_right, float error_margin);

  float3vector calibrate( const float3vector &induction, const quaternion<float> &q);

  bool available( void) const
  {
    return calibration_successful;
  }

  bool calculate( compass_calibrator_3D_data_t * data);

private:
  compass_calibrator_3D_data_t *data;
  float c[AXES][PARAMETERS];
  float heading_sector_error[OBSERVATIONS];
  bool calibration_successful;
};

#endif /* NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_ */
