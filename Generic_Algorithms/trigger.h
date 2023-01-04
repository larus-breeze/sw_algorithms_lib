/***********************************************************************//**
 * @file		trigger.h
 * @brief		class for hysteresis-based triggering on input data
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#ifndef TRIGGER_H_
#define TRIGGER_H_

//! class for hysteresis-based triggering on input data
class trigger
{
public:
  trigger( float _hysteresis)
  : hysteresis( _hysteresis)
  {}
  bool initialize( float value, bool _going_up=true)
  {
    minimax = value;
    return true;
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
