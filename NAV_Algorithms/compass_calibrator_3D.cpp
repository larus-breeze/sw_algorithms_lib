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

#include "compass_calibrator_3D.h"
#include "embedded_math.h"

#define __PROGRAM_START 0
#include "arm_math.h"

ROM float RECIP_SECTOR_SIZE = compass_calibrator_3D::OBSERVATIONS / M_PI_F / TWO / TWO;

bool compass_calibrator_3D::learn (const float3vector &observed_induction,const float3vector &expected_induction, const quaternion<float> &q, bool turning_right, float error_margin)
{
  float present_heading = q.get_heading();
  if( present_heading <0.0f)
    present_heading += M_PI_F * TWO;

  unsigned sector_index = (turning_right ? OBSERVATIONS / TWO : 0) + (unsigned)(present_heading * RECIP_SECTOR_SIZE);

  if ( heading_sector_error[sector_index] < error_margin)
    return false; // we have collected more precise data for this observation earlier

  heading_sector_error[sector_index] = error_margin;

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      data->target_vector[axis][sector_index] = expected_induction[axis];

      data->observation_matrix[axis][sector_index][0] = 1.0f;
      data->observation_matrix[axis][sector_index][1] = observed_induction[axis];
      data->observation_matrix[axis][sector_index][2] = q[0] * q[1];
      data->observation_matrix[axis][sector_index][3] = q[0] * q[2];
      data->observation_matrix[axis][sector_index][4] = q[0] * q[3];
      data->observation_matrix[axis][sector_index][5] = q[1] * q[1];
      data->observation_matrix[axis][sector_index][6] = q[1] * q[2];
      data->observation_matrix[axis][sector_index][7] = q[1] * q[3];
      data->observation_matrix[axis][sector_index][8] = q[2] * q[2];
      data->observation_matrix[axis][sector_index][9] = q[2] * q[3];
      data->observation_matrix[axis][sector_index][10]= q[3] * q[3];
    }

  for( unsigned i = 0; i < OBSERVATIONS; ++i)
    if( heading_sector_error[i] > 1e19)
      return false; // some observations are still missing

  return true; // complete
}

bool compass_calibrator_3D::calculate( compass_calibrator_3D_data_t * data)
{
//  if( calibration_successful)
//    return false;

  arm_matrix_instance_f32 solution;
  solution.numCols=PARAMETERS;
  solution.numRows=PARAMETERS;
  solution.pData = (float *)(data->temporary_solution_matrix);

  arm_matrix_instance_f32 observations;
  observations.numCols=PARAMETERS;
  observations.numRows=OBSERVATIONS;

  arm_matrix_instance_f32 observations_transposed;
  observations_transposed.numCols=OBSERVATIONS;
  observations_transposed.numRows=PARAMETERS;
  observations_transposed.pData = (float *)(data->transposed_matrix);

  arm_matrix_instance_f32 matrix_to_be_inverted;
  matrix_to_be_inverted.numCols=PARAMETERS;
  matrix_to_be_inverted.numRows=PARAMETERS;
  matrix_to_be_inverted.pData = (float *)(data->matrix_to_be_inverted_data);

  arm_matrix_instance_f32 solution_mapping;
  solution_mapping.numCols=OBSERVATIONS;
  solution_mapping.numRows=PARAMETERS;
  solution_mapping.pData = (float *)(data->solution_mapping_data);

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      observations.pData = &(data->observation_matrix[axis][0][0]);

      // calculation, once per axis (FRONT, RIGHT, DOWN):
      // target vector:        T = ideal induction values for all observations
      // single measurement:   < 1 induction q0q1 q0q2 q0q3 q1q1 q1q2 q1q3 q2q2 q2q3 q3q3 > (single row)
      // measurement matrix:   M = single measurement * # OBSERVATIONS
      // solution matrix:      S = inverse( M_transposed * M) * M_transposed
      // axis parameter set:   P = S * T

      arm_status result = arm_mat_trans_f32( &observations, &observations_transposed);
      assert( result == ARM_MATH_SUCCESS);

      result = arm_mat_mult_f32( &observations_transposed, &observations, &matrix_to_be_inverted);
      assert( result == ARM_MATH_SUCCESS);

      result = arm_mat_inverse_f32( &matrix_to_be_inverted, &solution);
      assert( result == ARM_MATH_SUCCESS);

      result = arm_mat_mult_f32( &solution, &observations_transposed, &solution_mapping);
      assert( result == ARM_MATH_SUCCESS);

      arm_matrix_instance_f32 target_vector_inst;
      target_vector_inst.numCols=1;
      target_vector_inst.numRows=OBSERVATIONS;
      target_vector_inst.pData=&(data->target_vector[axis][0]);

      arm_matrix_instance_f32 axis_parameter_set;
      axis_parameter_set.numCols=1;
      axis_parameter_set.numRows=PARAMETERS;
      axis_parameter_set.pData=&(c[axis][0]);
      result = arm_mat_mult_f32( &solution_mapping, &target_vector_inst, &axis_parameter_set);
      assert( result == ARM_MATH_SUCCESS);
    }

  calibration_successful = true;

#if PRINT_PARAMETERS

  for( unsigned k=0; k<3; ++k)
    {
      for( unsigned i=0; i<PARAMETERS; ++i)
	printf("%e\t", (double)(c[k][i]));
      printf("\n");
    }
  printf("\n");
#endif

  start_learning(); // ... again

  return true;
}

float3vector compass_calibrator_3D::calibrate( const float3vector &induction, const quaternion<float> &q)
  {
    if( ! calibration_successful)
      return float3vector();

    float3vector retv;
    for( int i = 0; i < 3; ++i)
      {
	retv[i] =
	    c[i][0] + c[i][1] * induction[i] +
	    c[i][2] * q[0] * q[1] + c[i][3] * q[0] * q[2] + c[i][4] * q[0] * q[3] +
	    c[i][5] * q[1] * q[1] + c[i][6] * q[1] * q[2] + c[i][7] * q[1] * q[3] +
	    c[i][8] * q[2] * q[2] + c[i][9] * q[2] * q[3] + c[i][10]* q[3] * q[3] ;
      }
    return retv;
  }
