/*
 * trigger.h
 *
 *  Created on: Aug 23, 2022
 *      Author: schaefer
 */

#ifndef TRIGGER_H_
#define TRIGGER_H_

class trigger
{
public:
  trigger( float _hysteresis)
  : hysteresis( _hysteresis)
  {}
  bool initialize( float value, bool _going_up=true)
  {
    minimax = value;
  }
  bool process( float value)
  {
    if( going_up)
      {
	if( value > minimax)
	  {
	    minimax=value;
	  }
	else
	  {
	    if( value < minimax - hysteresis)
	      {
		minimax=value;
		going_up=false;
		return true;
	      }
	  }
	return false;
      }
    else
      {
	if( value < minimax)
	  {
	    minimax=value;
	  }
	else
	  {
	    if( value > minimax + hysteresis)
	      {
		minimax=value;
		going_up=true;
		return true;
	      }
	  }
	return false;
      }
  }
private:
  float hysteresis;
  float minimax;
  bool going_up;
};



#endif /* TRIGGER_H_ */
