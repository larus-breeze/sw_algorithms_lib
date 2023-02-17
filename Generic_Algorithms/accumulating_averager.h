#ifndef GENERIC_ALGORITHMS_ACCUMULATING_AVERAGER_H_
#define GENERIC_ALGORITHMS_ACCUMULATING_AVERAGER_H_

template <typename type> class accumulating_averager
{
public:
  accumulating_averager(void)
  : sum(0), samples(0)
  {}
  void update( type input)
  {
    sum += input;
    ++samples;
  }
  type get_average( void) const
  {
    float tmp = 1.0f / samples;
    return sum * tmp;
  }
  void reset( type set_this_value, unsigned set_this_count)
  {
    sum = set_this_value * set_this_count;
    samples = set_this_count;
  }
private:
  type sum;
  unsigned samples;
};

#endif /* GENERIC_ALGORITHMS_ACCUMULATING_AVERAGER_H_ */
