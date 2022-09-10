/***********************************************************************//**
 * @file		HP_LP_fusion.h
 * @brief		measurement data fusion filter
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

#ifndef HP_LP_FUSION_H_
#define HP_LP_FUSION_H_

#include "embedded_math.h"

template<typename type, typename basetype> class HP_LP_fusion
{
public:
  HP_LP_fusion( basetype feedback_tap) // feedback_tap shall be positive !
    : a1( -feedback_tap),
	  old_output(0),
	  old_HP_input(0)
      {}

  type respond( type HP_input, type LP_input)
  {
    type new_output =
	    LP_input * (ONE + a1)
	  - HP_input * a1
	  + old_HP_input * a1
	  - old_output * a1 ;
    old_HP_input = HP_input;
    old_output = new_output;
    return new_output;
  }

  operator type ( void) const
    {
      return old_output;
    }

private:
  basetype a1; // a1, value usually negative
  type old_output;
  type old_HP_input;
};



#endif /* HP_LP_FUSION_H_ */
