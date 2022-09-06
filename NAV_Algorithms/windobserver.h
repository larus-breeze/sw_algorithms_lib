/** ***********************************************************************
 * @file		windobserver.h
 * @brief		smart windspeed measurement
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef WINDOBSERVER_H_
#define WINDOBSERVER_H_

#include "float3vector.h"
#include "AHRS.h"

class wind_observer_t
{
public:
  wind_observer_t (float beta_design)
  : decimating_counter(DECIMATION)
  {
//  beta_max = exp( - T_sample / Tau );
    beta_max = 0.982f; // todo implement correct parameter setup
    stage_1_N = ZERO;
    stage_1_E = ZERO;
  }
  const float3vector & get_output( void) const
  {
    return present_output;
  }

  void update (const float3vector &current_value, const float3vector & heading_vector, circle_state_t state)
  {
    stage_1_N = stage_1_N * beta_max + current_value.e[NORTH] * ( ONE - beta_max);
    stage_1_E = stage_1_E * beta_max + current_value.e[EAST]  * ( ONE - beta_max);

    --decimating_counter;
    if( decimating_counter == 0)
      {
	decimating_counter = DECIMATION;

	float alpha_N, beta_N, alpha_E, beta_E;

//	if( true) // todo smart filtering shortcut !
	if( state == STRAIGHT_FLIGHT) // todo patch
	  {
	    beta_N = beta_E = beta_max;
	    alpha_N = alpha_E = ONE - beta_max;
	  }
	else
	  {
	    alpha_N = (ONE - beta_max) * SQR( heading_vector.e[NORTH]);
	    alpha_E = (ONE - beta_max) * SQR( heading_vector.e[EAST]);
	    beta_N = ONE - alpha_N;
	    beta_E = ONE - alpha_E;
	  }
#if 0 // todo remove test
	probe[0] = alpha_N;
	probe[1] = alpha_E;
#endif
	present_output.e[NORTH] = present_output.e[NORTH] * alpha_N + stage_1_N * beta_N;
	present_output.e[EAST]  = present_output.e[EAST]  * alpha_E + stage_1_E * beta_E;
      }

    if( !isnormal( present_output.e[EAST]) || !isnormal( present_output.e[NORTH]))
      present_output.e[NORTH] = present_output.e[EAST] = ZERO;
  }

 private:
  enum { DECIMATION = 55}; // two stages -> decimation 1/100s -> 30 s; 55 = sqrt(3000)
  float stage_1_N;
  float stage_1_E;
  uint32_t decimating_counter;
  float beta_max; // nominator coefficient configured

  float3vector present_output; // maintained to save computing time
};

#endif /* WINDOBSERVERT_H_ */
