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

#define PRINT_PARAMETERS 1
#if PRINT_PARAMETERS
#include "stdio.h"
#endif

#include "soft_iron_compensator.h"
#include "embedded_math.h"

#include <matrix_functions.h>

ROM float RECIP_SECTOR_SIZE = soft_iron_compensator_t::OBSERVATIONS / M_PI_F / TWO / TWO;

bool soft_iron_compensator_t::learn ( const float3vector &induction_error, const quaternion<float> &q, bool turning_right, float error_margin)
{
  float present_heading = q.get_heading();
  if( present_heading <0.0f)
    present_heading += M_PI_F * TWO;

  int sector_index = (turning_right ? OBSERVATIONS / TWO : 0) + (unsigned)(present_heading * RECIP_SECTOR_SIZE);

  // if we have just left the last sector to be collected: report ready for computation
  if( ( last_sector_collected != -1) && ( sector_index != last_sector_collected) && (++measurement_counter > 10000))
    return true;

  if( heading_sector_error[sector_index] > 1e19) // this sector has not been written before
    ++populated_sectors;

  if ( heading_sector_error[sector_index] < error_margin)
    return false; // we have collected more precise data for this observation earlier

  heading_sector_error[sector_index] = error_margin;

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      target_vector[axis][sector_index] = induction_error[axis];

      observation_matrix[axis][sector_index][0] = 1.0f;
      observation_matrix[axis][sector_index][1] = q[0] * q[1];
      observation_matrix[axis][sector_index][2] = q[0] * q[2];
      observation_matrix[axis][sector_index][3] = q[0] * q[3];
      observation_matrix[axis][sector_index][4] = q[1] * q[1];
      observation_matrix[axis][sector_index][5] = q[1] * q[2];
      observation_matrix[axis][sector_index][6] = q[1] * q[3];
      observation_matrix[axis][sector_index][7] = q[2] * q[2];
      observation_matrix[axis][sector_index][8] = q[2] * q[3];
      observation_matrix[axis][sector_index][9] = q[3] * q[3];
    }

  if( (last_sector_collected == -1) && populated_sectors >= OBSERVATIONS)
    last_sector_collected = sector_index;

  return false;
}

bool soft_iron_compensator_t::calculate( void)
{
//  if( buffer_used_for_calibration != INVALID)
//    return false;

  computation_float_type temporary_solution_matrix[PARAMETERS][PARAMETERS];
  ARM_MATRIX_INSTANCE solution;
  solution.numCols=PARAMETERS;
  solution.numRows=PARAMETERS;
  solution.pData = (computation_float_type *)temporary_solution_matrix;

  ARM_MATRIX_INSTANCE  observations;
  observations.numCols=PARAMETERS;
  observations.numRows=OBSERVATIONS;

  computation_float_type transposed_matrix[PARAMETERS][OBSERVATIONS];
  ARM_MATRIX_INSTANCE observations_transposed;
  observations_transposed.numCols=OBSERVATIONS;
  observations_transposed.numRows=PARAMETERS;
  observations_transposed.pData = (computation_float_type *)transposed_matrix;

  computation_float_type matrix_to_be_inverted_data[PARAMETERS][PARAMETERS];
  ARM_MATRIX_INSTANCE matrix_to_be_inverted;
  matrix_to_be_inverted.numCols=PARAMETERS;
  matrix_to_be_inverted.numRows=PARAMETERS;
  matrix_to_be_inverted.pData = (computation_float_type *)matrix_to_be_inverted_data;

  computation_float_type solution_mapping_data[PARAMETERS][OBSERVATIONS];
  ARM_MATRIX_INSTANCE solution_mapping;
  solution_mapping.numCols=OBSERVATIONS;
  solution_mapping.numRows=PARAMETERS;
  solution_mapping.pData = (computation_float_type *)solution_mapping_data;

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
      // single measurement:   < 1 induction q0q1 q0q2 q0q3 q1q1 q1q2 q1q3 q2q2 q2q3 q3q3 > (single row)
      // measurement matrix:   M = single measurement * # OBSERVATIONS
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
#if 0	    // use average between new and old parameter set
      if( buffer_used_for_calibration != INVALID) // if we already had a valid parameter set
	{
	  int other_buffer = next_buffer == 1 ? 0 : 1;
	  for( unsigned i=0; i<PARAMETERS; ++i)
	    c[next_buffer][axis][i] = c[next_buffer][axis][i] * 0.25 + c[other_buffer][axis][i] * 0.75;
	}
#endif
    }

  buffer_used_for_calibration = next_buffer; // switch now in a thread-save manner

#if PRINT_PARAMETERS

  for( unsigned k=0; k < AXES; ++k)
    {
      for( unsigned i=0; i<PARAMETERS; ++i)
	printf("%e\t", (double)(c[next_buffer][k][i]));
      printf("\n");
    }
  printf("\n");
#endif

  start_learning(); // ... again

  return true;
}

float3vector soft_iron_compensator_t::compensate( const float3vector &induction, const quaternion<float> &q)
  {
    if( buffer_used_for_calibration == INVALID) // we do not have a valid calibration
      return float3vector();

    unsigned b=buffer_used_for_calibration; // just to save expensive characters ...

    float3vector retv;
    for( int i = 0; i < 3; ++i)
      {
	retv[i] =
	    c[b][i][0] +
	    c[b][i][1] * q[0] * q[1] + c[b][i][2] * q[0] * q[2] + c[b][i][3] * q[0] * q[3] +
	    c[b][i][4] * q[1] * q[1] + c[b][i][5] * q[1] * q[2] + c[b][i][6] * q[1] * q[3] +
	    c[b][i][7] * q[2] * q[2] + c[b][i][8] * q[2] * q[3] + c[b][i][9] * q[3] * q[3] ;
      }
    return retv;
  }
