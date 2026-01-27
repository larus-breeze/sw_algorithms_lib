#ifndef REMINDER_FLAG_H_
#define REMINDER_FLAG_H_

class reminder_flag
{
public:
  reminder_flag( void)
  : status( false)
  {}
  void set( void)
  {
    status = true;
  }
  bool test_and_reset( void)
  {
    bool copy = status;
    status = false;
    return copy;
  }

private:
  bool status;
};

#endif /* REMINDER_FLAG_H_ */
