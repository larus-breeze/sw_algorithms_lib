#include "stdio.h"

#include "compass_calibrator_3D.h"
#include "embedded_math.h"

#define __PROGRAM_START 0
#include "arm_math.h"

bool compass_calibrator_3D::learn (const float3vector &observed_induction,const float3vector &expected_induction, const quaternion<float> &q)
{
  float present_heading = q.get_heading();
  if( present_heading <0.0f)
    present_heading += M_PI_F * TWO;

  unsigned sector_index = present_heading / TWO / M_PI_F * DIM;

  if ( (covered_heading_sectors & (1 << sector_index)) != 0) // sector data already collected
    return false;

  covered_heading_sectors |= (1 << sector_index); // mark sector done

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      target_vector[axis][next_populated_observation] = expected_induction[axis];

      observation_matrix[axis][next_populated_observation][0] = 1.0f;
      observation_matrix[axis][next_populated_observation][1] = observed_induction[0];
      observation_matrix[axis][next_populated_observation][2] = observed_induction[1];
      observation_matrix[axis][next_populated_observation][3] = observed_induction[2];
    }

  ++next_populated_observation;
  return next_populated_observation >= DIM;
}

bool compass_calibrator_3D::calculate( float temporary_solution_matrix[DIM][DIM])
{
//  if( calibration_successful)
//    return false;

  arm_matrix_instance_f32 destination;
  destination.numCols=DIM;
  destination.numRows=DIM;
  destination.pData = (float *)temporary_solution_matrix;

  arm_matrix_instance_f32 source;
  source.numCols=DIM;
  source.numRows=DIM;

  for( unsigned axis = 0; axis < AXES; ++axis)
    {
      source.pData = &(observation_matrix[axis][0][0]);
      arm_status result = arm_mat_inverse_f32( &source, &destination);
      if( result != ARM_MATH_SUCCESS)
	return false;

      arm_matrix_instance_f32 target_vector_inst;
      target_vector_inst.numCols=1;
      target_vector_inst.numRows=DIM;
      target_vector_inst.pData=&(target_vector[axis][0]);

      arm_matrix_instance_f32 solution_inst;
      solution_inst.numCols=1;
      solution_inst.numRows=DIM;
      if( ! calibration_successful)
	{
	  solution_inst.pData=&(c[axis][0]);
	  result = arm_mat_mult_f32( &destination, &target_vector_inst, &solution_inst);
	}
      else
	{
	      float solution[DIM];
	      solution_inst.pData=solution;
	      result = arm_mat_mult_f32( &destination, &target_vector_inst, &solution_inst);
	      for( unsigned i=0; i< DIM; ++i)
		c[axis][i] = c[axis][i] * 0.99 + solution[i] * 0.01f;
	}
    }

  calibration_successful = true;

  for( unsigned k=0; k<3; ++k)
    {
      for( unsigned i=0; i<DIM; ++i)
	printf("%e\t", (double)(c[k][i]));
      printf("\n");
    }
  printf("\n");

  next_populated_observation = 0; // start new data collection
  covered_heading_sectors = 0;

  return true;
}

float3vector compass_calibrator_3D::calibrate( const float3vector &induction, const quaternion<float> &q)
  {
    if( ! calibration_successful)
      return induction;

    float3vector retv;
    for( int i = 0; i < 3; ++i)
      {
	retv[i] =
	    c[i][0] +
	    c[i][1] * induction[0] +
	    c[i][2] * induction[1] +
	    c[i][3] * induction[2];
      }
    return retv;
  }
