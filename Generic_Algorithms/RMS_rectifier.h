/*
 * RMS_rectifier.h
 *
 *  Created on: Jul 14, 2025
 *      Author: schaefer
 */

#ifndef GENERIC_ALGORITHMS_RMS_RECTIFIER_H_
#define GENERIC_ALGORITHMS_RMS_RECTIFIER_H_

#include <embedded_math.h>

template < class type> class RMS_rectifier
{
public:
  RMS_rectifier( type _feed_forward)
    :feed_forward( _feed_forward),
     accu(0)
  {}

  void feed( type input)
  {
    accu *= ONE - feed_forward;
    accu += SQR( input) * feed_forward;
  }

  type get_output( void) const
  {
    return SQRT( accu);
  }

private:
  type feed_forward;
  type accu;
};

#endif /* GENERIC_ALGORITHMS_RMS_RECTIFIER_H_ */
