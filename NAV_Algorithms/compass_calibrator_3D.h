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
  enum { AXES=3, PARAMETERS=11, OBSERVATIONS=22};

  compass_calibrator_3D( void)
    : calibration_successful(false)
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
  bool calculate( void);

private:
  float c[AXES][PARAMETERS];
  float target_vector[AXES][OBSERVATIONS];
  float observation_matrix[AXES][OBSERVATIONS][PARAMETERS];
  float heading_sector_error[OBSERVATIONS];
  bool calibration_successful;
};

#endif /* NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_ */
