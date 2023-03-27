/*
 * induction_observer.h
 *
 *  Created on: Feb 6, 2023
 *      Author: schaefer
 */

#ifndef NAV_ALGORITHMS_INDUCTION_OBSERVER_H_
#define NAV_ALGORITHMS_INDUCTION_OBSERVER_H_
#include "mean_and_variance_finder.h"
#include "float3vector.h"

template <class sample_type> class induction_observer_t
{
public:
  induction_observer_t( float _scale_factor)
  : scale_factor( _scale_factor)
  {}
  void feed (float3vector induction, bool right_turn)
  {
    if (right_turn)
	for( unsigned i=0; i<3; ++i)
	  induction_observer_right[i].feed (induction.e[i] * scale_factor);
    else
	for( unsigned i=0; i<3; ++i)
	  induction_observer_left[i].feed (induction.e[i] * scale_factor);
  }
  void
  reset (void)
  {
    for( unsigned i=0; i<3; ++i)
      {
	induction_observer_right[i].reset ();
	induction_observer_left[i].reset ();
      }
}
  bool data_valid( void) const
  {
    return (
	(induction_observer_right[0].get_samples() > MINIMUM_SAMPLES) &&
	(induction_observer_left[0].get_samples() > MINIMUM_SAMPLES) );
  }
  float3vector get_estimated_induction( void) const
  {
    float3vector retv;
    for( unsigned i=0; i<3; ++i)
      retv.e[i] = (induction_observer_right[i].get_mean() + induction_observer_left[i].get_mean()) * 0.5f / scale_factor;
    return retv;
  }
  float get_variance( void ) const
  {
    float sum = 0.0f;
    for( unsigned i=0; i<3; ++i)
      {
	sum += induction_observer_right[i].get_variance();
	sum += induction_observer_left[i].get_variance();
      }
    return sum * 0.1666666666f / scale_factor / scale_factor;
  }
private:
  enum{ MINIMUM_SAMPLES = 10000};
  float scale_factor;
  mean_and_variance_finder_t <sample_type> induction_observer_right[3];
  mean_and_variance_finder_t <sample_type> induction_observer_left[3];
};

#endif /* NAV_ALGORITHMS_INDUCTION_OBSERVER_H_ */
