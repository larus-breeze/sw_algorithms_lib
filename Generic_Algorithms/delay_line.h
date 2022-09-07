#ifndef DELAY_LINE_H_
#define DELAY_LINE_H_

template <class data_t, unsigned length> class delay_line
{
public:
  delay_line( void)
    :storage{0},
     ptr(storage)
    {};
  data_t respond( const data_t &right)
  {
    data_t retv=*ptr;
    *ptr=right;

    ++ptr;

    if( ptr >= storage + length)
      ptr=storage;

    return retv;
  }
private:
  data_t storage[length];
  data_t * ptr;
};

#endif /* DELAY_LINE_H_ */
