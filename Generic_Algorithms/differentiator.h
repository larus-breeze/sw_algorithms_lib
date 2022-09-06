/*
 * Differentiator.h
 *
 *  Created on: 09.12.2014
 *      Author: user
 */

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
      : time_constant( Tdiff / Tsampling), old_value( init_value)
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
