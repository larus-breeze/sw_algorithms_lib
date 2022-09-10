/***********************************************************************//**
 * @file		integrator.h
 * @brief		integrate data (template)
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

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "vector.h"

//! multi-dimensional integrator
template <class basetype, class datatype>
 class integrator
   {
public:
  //! constructor taking Ti, Ts and initial values
     integrator( basetype Tintegrator, basetype Tsampling, const datatype& init_value = 0)
        : time_constant( Tsampling / Tintegrator), value( init_value)
        {};

     //! constructor taking sampling-time only
      integrator( basetype Tsampling)
	 : time_constant( Tsampling ), value( 0)
	 {};

//! update integrator taking next input value
   const datatype & response( datatype & right)
      {
        value += right * time_constant;
        return value;
      };

//! returns current output
   const datatype & get_value( void) const
      {
        return value;
      };

//! cast to vector<size> returns current output
    datatype operator () ( void) const
      {
      return get_value();
      };
         
//! set current output value
   void set_value( const datatype & _value)
      {
        this->value = _value;
      };

private:
//! sampling time
   basetype time_constant;
//! maintains current integrator output
   datatype value;
   };

#endif
