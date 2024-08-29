/*
 * compass_calibrator_3D.h
 *
 *  Created on: Aug 20, 2024
 *      Author: schaefer
 */

#ifndef NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_
#define NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_

#include "embedded_memory.h"
#include "embedded_math.h"
#include "float3vector.h"
#include "quaternion.h"

class compass_calibrator_3D
{
public:
  enum { AXES=3, DIM=11};

  compass_calibrator_3D( void)
  : next_populated_observation(0),
    covered_heading_sectors(0),
    calibration_successful(false)
  {
  }

  void start_learning( void)
  {
    next_populated_observation = 0;
    covered_heading_sectors=0;
  }

  bool learn (const float3vector &observed_induction,const float3vector &expected_induction, const quaternion<float> &q);
  float3vector calibrate( const float3vector &induction, const quaternion<float> &q);
  bool calculate( float temporary_solution_matrix[DIM][DIM]);

private:
  float c[AXES][DIM];
  float target_vector[AXES][DIM];
  uint8_t next_populated_observation;
  float observation_matrix[AXES][DIM][DIM];
  uint16_t covered_heading_sectors;
  bool calibration_successful;
};

#endif /* NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_ */
