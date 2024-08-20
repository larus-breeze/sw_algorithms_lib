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
  float3vector calibrate( const float3vector &induction, const quaternion<float> &q)
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
private:
  static ROM float c[3][11];
};

#endif /* NAV_ALGORITHMS_COMPASS_CALIBRATOR_3D_H_ */
