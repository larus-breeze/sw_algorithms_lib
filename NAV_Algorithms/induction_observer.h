/*
 * induction_observer.h
 *
 *  Created on: Feb 6, 2023
 *      Author: schaefer
 */

#ifndef NAV_ALGORITHMS_INDUCTION_OBSERVER_H_
#define NAV_ALGORITHMS_INDUCTION_OBSERVER_H_
#include "mean_and_variance_finder.h"

class induction_observer_t
{
public:
  void
  feed (float induction_north, float induction_east, bool right_turn)
  {
    if (right_turn)
      {
	north_induction_observer_right.feed (induction_north);
	east_induction_observer_right.feed (induction_east);
      }
    else
      {
	north_induction_observer_left.feed (induction_north);
	east_induction_observer_left.feed (induction_east);
      }
  }
  void
  reset (void)
  {
    north_induction_observer_right.reset ();
    east_induction_observer_right.reset ();
    north_induction_observer_left.reset ();
    east_induction_observer_left.reset ();
  }
  bool data_valid( void) const
  {
    return (
	(north_induction_observer_right.get_samples() > MINIMUM_SAMPLES) &&
	(north_induction_observer_left.get_samples() > MINIMUM_SAMPLES) );
  }
  float get_north_induction( void) const
  {
    return (north_induction_observer_right.get_mean() + north_induction_observer_left.get_mean()) * 0.5f;
  }
  float get_east_induction( void) const
  {
    return (east_induction_observer_right.get_mean() + east_induction_observer_left.get_mean()) * 0.5f;
  }
  float get_variance( void ) const
  {
    float sum =
	north_induction_observer_right.get_variance() +
	east_induction_observer_right.get_variance() +
	north_induction_observer_left.get_variance() +
	east_induction_observer_left.get_variance();
    return sum * 0.25f;
  }
private:
  enum{ MINIMUM_SAMPLES = 10000};
  mean_and_variance_finder_t north_induction_observer_right;
  mean_and_variance_finder_t east_induction_observer_right;
  mean_and_variance_finder_t north_induction_observer_left;
  mean_and_variance_finder_t east_induction_observer_left;
};

#endif /* NAV_ALGORITHMS_INDUCTION_OBSERVER_H_ */
