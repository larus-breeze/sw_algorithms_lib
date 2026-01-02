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

#include "matrix_functions.h"

#if 0
typedef double computation_float_type;
#define ARM_MATRIX_INSTANCE arm_matrix_instance_f64
#define ARM_MAT_TRANS arm_mat_trans_f64
#define ARM_MAT_MULT arm_mat_mult_f64
#define ARM_MAT_INVERSE arm_mat_inverse_f64
#else
typedef float computation_float_type;
#define ARM_MATRIX_INSTANCE arm_matrix_instance_f32
#define ARM_MAT_TRANS arm_mat_trans_f32
#define ARM_MAT_MULT arm_mat_mult_f32
#define ARM_MAT_INVERSE arm_mat_inverse_f32
#endif

class magnetic_calculation_data_t;

//! 3 dimensional magnetic sensor transfer matrix
class compass_calibrator_3D_t
{
public:
  enum { AXES=3, PARAMETERS=4, OBSERVATIONS=22, INVALID=-1};

  compass_calibrator_3D_t( magnetic_calculation_data_t &_d)
    : buffer_used_for_calibration(INVALID),
      measurement_counter(0),
      d( _d),
      calibration_status( CALIBRATION_INVALID)
{
    start_learning();
  }

  void start_learning( void)
  {
    populated_sectors = 0;
    last_sector_collected = INVALID;
    measurement_counter=0;
    for( unsigned i=0; i < OBSERVATIONS; ++i)
      heading_sector_error[i]=1e20f;
  }

  bool learn (
      const float3vector &observed_induction,
      const float3vector &expected_induction,
      const quaternion<float> &q,
      bool turning_right,
      float error_margin);
float3vector calibrate( const float3vector &induction);
  bool calculate( void);
  bool available( void) const
  {
    return buffer_used_for_calibration != INVALID;
  }

  const computation_float_type * get_current_parameters( void) const
  {
    if( buffer_used_for_calibration == INVALID)
      return 0;

    return &(c[buffer_used_for_calibration][0][0]);
  }

  void set_current_parameters( const float * source, bool using_orientation_defaults = false)
  {
    if( using_orientation_defaults )
      calibration_status = USING_ORIENTATION_DEFAULTS;
    else
      calibration_status = INITIAL;

    int next_buffer;
    if( buffer_used_for_calibration != 0)
      next_buffer = 0;
    else
      next_buffer = 1;

    computation_float_type * destination = &(c[next_buffer][0][0]);
    for( unsigned i=0; i < AXES * PARAMETERS; ++i)
      *destination++ = *source++;

    buffer_used_for_calibration = next_buffer;
  }

private:
  enum calibration_status_t { CALIBRATION_INVALID, USING_ORIENTATION_DEFAULTS, INITIAL, REGULAR};
  int buffer_used_for_calibration;
  unsigned populated_sectors;
  int last_sector_collected;
  int measurement_counter;
  calibration_status_t calibration_status;

  // sensor transfer matrix
  computation_float_type c[2][AXES][PARAMETERS]; // double buffering for multi-thrading support

  // observation data
  computation_float_type observation_matrix[AXES][OBSERVATIONS][PARAMETERS];
  computation_float_type target_vector[compass_calibrator_3D_t::AXES][compass_calibrator_3D_t::OBSERVATIONS];
  computation_float_type heading_sector_error[OBSERVATIONS];

  magnetic_calculation_data_t & d;
};

class magnetic_calculation_data_t
{
  friend class compass_calibrator_3D_t;
  // temporary computation data
  computation_float_type temporary_solution_matrix[compass_calibrator_3D_t::PARAMETERS][compass_calibrator_3D_t::PARAMETERS];
  computation_float_type transposed_matrix[compass_calibrator_3D_t::PARAMETERS][compass_calibrator_3D_t::OBSERVATIONS];
  computation_float_type matrix_to_be_inverted_data[compass_calibrator_3D_t::PARAMETERS][compass_calibrator_3D_t::PARAMETERS];
  computation_float_type solution_mapping_data[compass_calibrator_3D_t::PARAMETERS][compass_calibrator_3D_t::OBSERVATIONS];
};


extern compass_calibrator_3D_t compass_calibrator_3D;
extern compass_calibrator_3D_t external_compass_calibrator_3D;
void trigger_compass_calibrator_3D_calculation( bool use_external_magnetometer);
extern magnetic_calculation_data_t temporary_mag_calculation_data;

#endif /* NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_ */
