#ifndef HP_LP_FUSION_H_
#define HP_LP_FUSION_H_

#include "embedded_math.h"

template<typename type> class HP_LP_fusion
{
public:
  HP_LP_fusion( type feedback_tap) // feedback_tap shall be positive !
    : a1( -feedback_tap),
	  old_output(0),
	  old_HP_input(0)
      {}

  type respond( type HP_input, type LP_input)
  {
    type new_output =
	  (ONE + a1) * LP_input
	  - a1 * HP_input + a1 * old_HP_input
	  - a1 * old_output;
    old_HP_input = HP_input;
    old_output = new_output;
    return new_output;
  }

  operator type ( void) const
    {
      return old_output;
    }

private:
  type a1; // a1, value usually negative
  type old_output;
  type old_HP_input;
};



#endif /* HP_LP_FUSION_H_ */
