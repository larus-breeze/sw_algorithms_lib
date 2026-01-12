/***********************************************************************//**
 * @file		compass_calibrator_3D.cpp
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

#include "system_configuration.h"
#include "embedded_math.h"
#include <matrix_functions.h>
#include "compass_calibrator_3D.h"
#include "NAV_tuning_parameters.h"

#if PRINT_3D_MAG_PARAMETERS
#include "stdio.h"
#endif

ROM float RECIP_SECTOR_SIZE = compass_calibrator_3D_t::OBSERVATIONS / M_PI_F / TWO / TWO;

bool compass_calibrator_3D_t::learn (
    const float3vector &observed_induction,
    const float3vector &expected_induction,
    const quaternion<float> &q,
    bool turning_right,
    float error_margin)
{
  int sector_index;
  float present_heading = q.get_heading();
  if( present_heading < ZERO)
	present_heading += M_PI_F * TWO;

  if( calibration_status == USING_ORIENTATION_DEFAULTS)
    // we need to be fast and therefore use only one circling direction
    sector_index = (unsigned)(present_heading * RECIP_SECTOR_SIZE * TWO);
  else
    sector_index = (turning_right ? OBSERVATIONS / TWO : 0) + (unsigned)(present_heading * RECIP_SECTOR_SIZE);

  // if we have just left the last sector to be collected: report ready for computation
  if( ( last_sector_collected != -1) && ( sector_index != last_sector_collected) && (++measurement_counter > MINIMUM_MAG_CALIB_SAMPLES))
    return true;

  if( heading_sector_error[sector_index] > 1e19) // this sector has not been written before
    ++populated_sectors;

  if ( heading_sector_error[sector_index] < error_margin)
    return false; // we have collected more precise data for this observation earlier

  heading_sector_error[sector_index] = error_margin;

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      target_vector[axis][sector_index] = expected_induction[axis];

      observation_matrix[axis][sector_index][0] = 1.0f;
      observation_matrix[axis][sector_index][1] = observed_induction[0];
      observation_matrix[axis][sector_index][2] = observed_induction[1];
      observation_matrix[axis][sector_index][3] = observed_induction[2];
    }

  if( (last_sector_collected == INVALID) && populated_sectors >= OBSERVATIONS)
    // now we are collecting data within the last sector to be collected
    last_sector_collected = sector_index;

  return false;
}

bool compass_calibrator_3D_t::calculate( void)
{
//  unsigned size = sizeof( magnetic_calculation_data_t);
//  unsigned obj_size = sizeof( *this);

  ARM_MATRIX_INSTANCE solution;
  solution.numCols=PARAMETERS;
  solution.numRows=PARAMETERS;
  solution.pData = (computation_float_type *)d.temporary_solution_matrix;

  ARM_MATRIX_INSTANCE  observations;
  observations.numCols=PARAMETERS;
  observations.numRows=OBSERVATIONS;

  ARM_MATRIX_INSTANCE observations_transposed;
  observations_transposed.numCols=OBSERVATIONS;
  observations_transposed.numRows=PARAMETERS;
  observations_transposed.pData = (computation_float_type *)d.transposed_matrix;

  ARM_MATRIX_INSTANCE matrix_to_be_inverted;
  matrix_to_be_inverted.numCols=PARAMETERS;
  matrix_to_be_inverted.numRows=PARAMETERS;
  matrix_to_be_inverted.pData = (computation_float_type *)d.matrix_to_be_inverted_data;

  ARM_MATRIX_INSTANCE solution_mapping;
  solution_mapping.numCols=OBSERVATIONS;
  solution_mapping.numRows=PARAMETERS;
  solution_mapping.pData = (computation_float_type *)d.solution_mapping_data;

  int next_buffer;
  if( buffer_used_for_calibration == 0)
    next_buffer = 1;
  else
    next_buffer = 0;

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      observations.pData = &(observation_matrix[axis][0][0]);

      // calculation, once per axis (FRONT, RIGHT, DOWN):
      // target vector:        T = ideal induction values for all observations
      // single measurement:   < 1 induction_x induction_y induction_z >
      // measurement matrix:   M = single measurement * OBSERVATIONS
      // solution matrix:      S = inverse( M_transposed * M) * M_transposed
      // axis parameter set:   P = S * T

      arm_status result = ARM_MAT_TRANS( &observations, &observations_transposed);
      if( result != ARM_MATH_SUCCESS)
	{
	  start_learning(); // discard data
	  return false;
	}

      result = ARM_MAT_MULT( &observations_transposed, &observations, &matrix_to_be_inverted);
      if( result != ARM_MATH_SUCCESS)
	{
	  start_learning(); // discard data
	  return false;
	}

      result = ARM_MAT_INVERSE( &matrix_to_be_inverted, &solution);
      if( result != ARM_MATH_SUCCESS)
	{
	  start_learning(); // discard data
	  return false;
	}

      result = ARM_MAT_MULT( &solution, &observations_transposed, &solution_mapping);
      if( result != ARM_MATH_SUCCESS)
	{
	  start_learning(); // discard data
	  return false;
	}

      ARM_MATRIX_INSTANCE target_vector_inst;
      target_vector_inst.numCols=1;
      target_vector_inst.numRows=OBSERVATIONS;
      target_vector_inst.pData=&(target_vector[axis][0]);

      ARM_MATRIX_INSTANCE axis_parameter_set;
      axis_parameter_set.numCols=1;
      axis_parameter_set.numRows=PARAMETERS;
      axis_parameter_set.pData=&(c[next_buffer][axis][0]);
      result = ARM_MAT_MULT( &solution_mapping, &target_vector_inst, &axis_parameter_set);
      if( result != ARM_MATH_SUCCESS)
	{
	  start_learning(); // discard data
	  return false;
	}

      if( ( calibration_status == INITIAL) || ( calibration_status == REGULAR))
	{
	  unsigned other_buffer = next_buffer == 0 ? 1 : 0;
	  for( unsigned k=0; k < PARAMETERS; ++k)
	    c[next_buffer][axis][k] = (ONE - MAG_CALIBRATION_LETHARGY) * c[next_buffer][axis][k] + MAG_CALIBRATION_LETHARGY * c[other_buffer][axis][k];
	}
    }

  switch( calibration_status)
  {
	case CALIBRATION_INVALID:
	case USING_ORIENTATION_DEFAULTS:
	  calibration_status = INITIAL;
	  break;
	case INITIAL:
	  calibration_status = REGULAR;
	  break;
	case REGULAR:
	default:
	  break;
  };

#if PRINT_3D_MAG_PARAMETERS

      float variance_sum = 0.0f;
      for( unsigned k=0; k < AXES; ++k)
	{
	  for( unsigned i=0; i<PARAMETERS; ++i)
	    {
	      printf("%e\t", (double)(c[next_buffer][k][i]));
	      variance_sum += SQR(c[0][k][i]-c[1][k][i]);
	    }
//	  printf("\n");
	}
      printf(" Std dev. = %e\n", SQRT( variance_sum / AXES / PARAMETERS));

      float3matrix m;
      m.e[0][0]=c[next_buffer][0][1];
      m.e[0][1]=c[next_buffer][0][2];
      m.e[0][2]=c[next_buffer][0][3];

      m.e[1][0]=c[next_buffer][1][1];
      m.e[1][1]=c[next_buffer][1][2];
      m.e[1][2]=c[next_buffer][1][3];

      m.e[2][0]=c[next_buffer][2][1];
      m.e[2][1]=c[next_buffer][2][2];
      m.e[2][2]=c[next_buffer][2][3];

      quaternion<float> q;
      q.from_rotation_matrix(m);
      eulerangle<float > e;
      e = q;

      printf("Rotation rpy = %f %f %f\n\n", e.roll * 180/M_PI, e.pitch * 180/M_PI, e.yaw * 180/M_PI);

#endif

  buffer_used_for_calibration = next_buffer; // switch now in a thread-save manner

  start_learning(); // ... again

  return true;
}

float3vector compass_calibrator_3D_t::calibrate( const float3vector &induction)
  {
    if( calibration_status == CALIBRATION_INVALID) // we do not have a valid calibration
      return induction;

    unsigned b=buffer_used_for_calibration; // just to save expensive characters ...

    float3vector retv;
    for( int i = 0; i < AXES; ++i)
      {
	retv[i] =
	    c[b][i][0] +
	    c[b][i][1] * induction[0] +
	    c[b][i][2] * induction[1] +
	    c[b][i][3] * induction[2];
      }

    return retv;
  }
