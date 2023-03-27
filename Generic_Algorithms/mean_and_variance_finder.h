#include "embedded_math.h"

#ifndef GENERIC_ALGORITHMS_MEAN_AND_VARIANCE_FINDER_H_
#define GENERIC_ALGORITHMS_MEAN_AND_VARIANCE_FINDER_H_

template <class sample_data>class mean_and_variance_finder_t
{
public:
  mean_and_variance_finder_t( void)
  : sum(0),
    square_sum(0),
    samples(0)
  {}
  void reset( void)
  {
    sum = 0;
    square_sum = 0;
    samples = 0;
  }
  void feed( sample_data value)
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
    return sum / samples;
  }
  float get_variance( void) const
  {
    sample_data mean = sum / samples;
    return square_sum / samples - mean * mean;
  }

private:
  sample_data sum;
  sample_data square_sum;
  unsigned samples;
};

#endif /* GENERIC_ALGORITHMS_MEAN_AND_VARIANCE_FINDER_H_ */
