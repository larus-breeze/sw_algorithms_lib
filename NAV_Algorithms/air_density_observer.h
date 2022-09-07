#ifndef AIR_DENSITY_OBSERVER_H_
#define AIR_DENSITY_OBSERVER_H_

#include "Linear_Least_Square_Fit.h"
#include "trigger.h"

typedef double evaluation_float_type;
#define MAX_ALLOWED_VARIANCE	1e-9
#define MINIMUM_ALTITUDE_RANGE	300.0f
#define ALTITUDE_TRIGGER_HYSTERESIS 50.0f

class air_data_result
{
public:
  air_data_result( void)
    : valid( false)
  {}
  float density_correction;
  float QFF;
  bool valid;
};

class air_density_observer
{
public:
  air_density_observer (void)
  : altitude_trigger( ALTITUDE_TRIGGER_HYSTERESIS)
  {
  }
  air_data_result feed_metering( float pressure, float MSL_altitude);

  void initialize( float altitude)
  {
    altitude_trigger.initialize(altitude);
    min_altitude = max_altitude = altitude;
    density_QFF_calculator.reset();
  }
private:

    linear_least_square_fit<int64_t,evaluation_float_type> density_QFF_calculator;
    float min_altitude;
    float max_altitude;
    trigger altitude_trigger;
};

#endif /* AIR_DENSITY_OBSERVER_H_ */
