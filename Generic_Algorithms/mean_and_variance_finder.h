#include "embedded_math.h"

#ifndef GENERIC_ALGORITHMS_MEAN_AND_VARIANCE_FINDER_H_
#define GENERIC_ALGORITHMS_MEAN_AND_VARIANCE_FINDER_H_

class mean_and_variance_finder_t
{
public:
  mean_and_variance_finder_t( void)
  : sum(0.0f),
    square_sum(0.0f),
    samples(0)
  {}
  void reset( void)
  {
    sum = 0.0f;
    square_sum = 0.0f;
    samples = 0;
  }
  void feed( float value)
  {
    sum += value;
    square_sum += SQR( value);
    ++samples;
  }
  unsigned get_samples( void ) const
  {
    return samples;
  }
  float get_mean( void) const
  {
    return sum / (float)samples;
  }
  float get_variance( void) const
  {
    return square_sum / (float)samples - SQR( get_mean());
  }

private:
  float sum;
  float square_sum;
  unsigned samples;
};

#endif /* GENERIC_ALGORITHMS_MEAN_AND_VARIANCE_FINDER_H_ */
