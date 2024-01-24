#ifndef NAV_ALGOROTHMS_COMPASS_GROUND_CALIB
#define NAV_ALGOROTHMS_COMPASS_GROUND_CALIB

#include "system_configuration.h"
#include "pt2.h"
#include "float3vector.h"
#include "compass_calibration.h"

#define CUTOFF_DIV_BY_SAMPLING_FREQ 0.01f

class compass_ground_calibration_t
{
public:
  compass_ground_calibration_t( void)
  : averager( CUTOFF_DIV_BY_SAMPLING_FREQ)
  {
    for( unsigned axis = X; axis <= Z; ++axis)
      {
	max[axis]=min[axis]=0.0f; // assuming offset < 50% of range
      }
  }
  void feed( const float3vector & mag_raw)
  {
    averager.respond( mag_raw);
    float3vector mean_value = averager.get_output();
    for( unsigned axis = X; axis <= Z; ++axis)
      {
	if( mean_value[axis] > max[axis])
	     max[axis] = mean_value[axis];
	if( mean_value[axis] < min[axis])
	     min[axis] = mean_value[axis];
      }
     }
  void get_calibration_result( single_axis_calibration_t * calibration_3d)
    {
      for( unsigned axis = X; axis <= Z; ++axis)
	{
	  float range = max[axis]-min[axis];
	  calibration_3d[axis].scale = 2.0 / range;
	  calibration_3d[axis].offset = (max[axis] + min[axis]) / 2.0f;
	  calibration_3d[axis].variance = 0.1f; // a dummy, we don't know it here!
	}
}
private:
  float max[3];
  float min[3];
  pt2 <float3vector, float> averager;
  enum{ X, Y, Z};
};

#endif


