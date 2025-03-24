#ifndef GENERIC_ALGORITHMS_SLOPE_LIMITER_H_
#define GENERIC_ALGORITHMS_SLOPE_LIMITER_H_

template <class data_t> class slope_limiter
{
public:
  slope_limiter( void)
    :last_output{0}
    {};
  data_t respond( const data_t &input, const data_t &max_step)
  {
    data_t retv;
    if( input > last_output + max_step)
      retv = last_output + max_step;
    else
      if( input < last_output - max_step)
        retv = last_output - max_step;
      else
      retv = input;

    last_output = retv;

    return retv;
  }
private:
  data_t last_output;
};

#endif /* GENERIC_ALGORITHMS_SLOPE_LIMITER_H_ */
