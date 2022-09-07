/***********************************************************************//**
 * @file		integrator.hpp
 * @brief		vector numeric integrator
 * @author		Dr. Klaus Schaefer
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
