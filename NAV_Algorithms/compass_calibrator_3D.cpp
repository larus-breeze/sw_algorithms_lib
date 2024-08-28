#include "compass_calibrator_3D.h"
#include "embedded_math.h"

#define __PROGRAM_START 0
#include "arm_math.h"

bool compass_calibrator_3D::learn (const float3vector &observed_induction,const float3vector &expected_induction, const quaternion<float> &q)
{
  if( next_populated_observation >= DIM)
    return false;

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      target_vector[axis] = expected_induction[axis];

      observation_matrix[axis][next_populated_observation][0] = 1.0f;
      observation_matrix[axis][next_populated_observation][1] = observed_induction[axis];
      observation_matrix[axis][next_populated_observation][2] = q[0] * q[1];
      observation_matrix[axis][next_populated_observation][3] = q[0] * q[2];
      observation_matrix[axis][next_populated_observation][4] = q[0] * q[3];
      observation_matrix[axis][next_populated_observation][5] = q[1] * q[1];
      observation_matrix[axis][next_populated_observation][6] = q[1] * q[2];
      observation_matrix[axis][next_populated_observation][7] = q[1] * q[3];
      observation_matrix[axis][next_populated_observation][8] = q[2] * q[2];
      observation_matrix[axis][next_populated_observation][9] = q[2] * q[3];
      observation_matrix[axis][next_populated_observation][10]= q[3] * q[3];
    }

  ++next_populated_observation;
  return next_populated_observation < DIM;
}

bool compass_calibrator_3D::calculate( float *temporary_solution_matrix)
{
  if( next_populated_observation >= DIM)
    return 0.0f;

  arm_matrix_instance_f32 source;
  source.numCols=DIM;
  source.numRows=DIM;

  arm_matrix_instance_f32 destination;
  destination.numCols=DIM;
  destination.numRows=DIM;
  destination.pData=temporary_solution_matrix;

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      source.pData = &(observation_matrix[axis][0][0]);
      arm_status result = arm_mat_inverse_f32( &source, &destination);
      if( result != ARM_MATH_SUCCESS)
	return false;

      arm_matrix_instance_f32 target_vector_inst;
      target_vector_inst.numCols=1;
      target_vector_inst.numRows=DIM;
      target_vector_inst.pData=target_vector;

      arm_matrix_instance_f32 solution_inst;
      solution_inst.numCols=1;
      solution_inst.numRows=DIM;
      solution_inst.pData=&(c[axis][0]);

      result = arm_mat_mult_f32( &destination, &target_vector_inst, &solution_inst);
    }
  calibration_successful = true;
  return true;
}

float3vector compass_calibrator_3D::calibrate( const float3vector &induction, const quaternion<float> &q)
  {
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
