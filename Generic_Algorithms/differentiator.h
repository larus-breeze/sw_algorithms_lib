/***********************************************************************//**
 * @file		differentiator.h
 * @brief		differentiate data (template)
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

#ifndef DIFFERENTIATOR_H_
#define DIFFERENTIATOR_H_

#include "vector.h"

//! multi-dimensional integrator
template <class basetype, class datatype>
 class differentiator
   {
public:
//! constructor taking sampling-time and initial value
   differentiator( basetype Tdiff, basetype Tsampling, const datatype& init_value = 0)
      : time_constant( Tdiff / Tsampling),
	old_value( init_value),
	differentiation(),
	output()
      {};

   //! update differentiator taking next input value
      datatype respond( const datatype & right)
         {
            output = (right-old_value) * time_constant;
            old_value = right;
            return output;
         };

//! returns current output
   const datatype & get_value( void) const
      {
        return output;
      };

//! cast to vector<size> returns current output
    datatype operator () ( void) const
      {
      return get_value();
      };

private:
//! sampling time
   const basetype time_constant;
//! maintains old output
   datatype old_value;
   datatype differentiation;
   datatype output;
   };

#endif /* DIFFERENTIATOR_H_ */
