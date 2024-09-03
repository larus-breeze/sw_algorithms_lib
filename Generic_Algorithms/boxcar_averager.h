/*
 * boxcar_averager.h
 *
 *  Created on: May 17, 2023
 *      Author: schaefer
 */

#ifndef GENERIC_ALGORITHMS_BOXCAR_AVERAGER_H_
#define GENERIC_ALGORITHMS_BOXCAR_AVERAGER_H_

//! FIR averager using a rectangle window
template <class data_t, unsigned length> class boxcar_averager
{
public:
  boxcar_averager( void)
    :storage{0},
     ptr(storage)
    {};
  data_t respond( const data_t &right)
  {
    *ptr=right;

    ++ptr;
    if( ptr >= storage + length)
      ptr=storage;

    data_t average = {0};
    for(unsigned i=0; i<length; ++i)
      average += storage[i];

    return average / length;
  }
private:
  data_t storage[length];
  data_t * ptr;
};



#endif /* GENERIC_ALGORITHMS_BOXCAR_AVERAGER_H_ */
