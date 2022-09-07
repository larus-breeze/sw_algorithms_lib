#include "embedded_math.h"
#include <air_density_observer.h>

air_data_result air_density_observer::feed_metering( float pressure, float MSL_altitude)
{
  air_data_result air_data;
  density_QFF_calculator.add_value( MSL_altitude * 100.0f, pressure);

  if( MSL_altitude > max_altitude)
    max_altitude = MSL_altitude;

  if( MSL_altitude < min_altitude)
    min_altitude = MSL_altitude;
#if 0
  if( density_QFF_calculator.get_count() <= 3000) // measure 5 minutes
    return air_data;
#else
  if( false == altitude_trigger.process(MSL_altitude))
    return air_data;

#endif

  if( (max_altitude - min_altitude) < MINIMUM_ALTITUDE_RANGE) // ... forget this measurement
    {
      max_altitude = min_altitude = MSL_altitude;
      density_QFF_calculator.reset();
      return air_data;
    }

  linear_least_square_result<evaluation_float_type> result;
  density_QFF_calculator.evaluate(result);

  assert( result.variance_slope > 0);
  assert( result.variance_offset > 0);

  if( result.variance_slope < MAX_ALLOWED_VARIANCE)
    {
      air_data.QFF = result.y_offset;
      float density = result.slope * 100.0f * -0.10194f; // div by -9.81f;
      float pressure = density_QFF_calculator.get_mean_y();
      float std_density = 1.0496346613e-5f * pressure + 0.1671546011f;
      air_data.density_correction = density / std_density;
      air_data.valid = true;
    }

  max_altitude = min_altitude = MSL_altitude;
  density_QFF_calculator.reset();

  return air_data;
}
