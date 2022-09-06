/*
 * pt2.h
 *
 *  Created on: Mar 7, 2013
 *      Author: schaefer
 */

#ifndef PT2_H_
#define PT2_H_

#include <ringbuffer.h>
#include "embedded_math.h"

// butterworth filter prototype parameters at Fcutoff/Fsampling = 0.25
// B coefficients -> nominator
// A coefficients -> DE-nominator, A0 = 1
#define B0 0.292893218813452
#define B1 0.585786437626905
#define B2 0.292893218813452
#define A1 ZERO
#define A2 0.171572875253810
#define DESIGN_FREQUENCY 0.25

//! Second order IIR filter
template <class datatype, class basetype> class pt2
{
public:
	pt2( basetype fcutoff) //! constructor taking Fc/Fs
	: input( datatype()),
	  output( datatype())
	{
		basetype delta = SIN( M_PI * (DESIGN_FREQUENCY - fcutoff)) / SIN( M_PI * (fcutoff + DESIGN_FREQUENCY));
		basetype a0x = A2 * SQR(delta) - A1 + ONE;
		basetype a1x = -2.0 * delta * A2 + (SQR(delta) + ONE) * A1 - 2.0 * delta;
		basetype a2x = A2 - delta * A1 + SQR(delta);

		basetype b0x = B2 * SQR( delta) - B1 * delta + B0;
		basetype b1x = - 2.0 * delta * B2 + (SQR(delta) + 1) * B1 - 2.0 * delta * B0;
		basetype b2x = B2 - delta * B1 + SQR( delta) * B0;

		// normalize denominator a0 = ONE
		a1 = a1x / a0x;
		a2 = a2x / a0x;
		b0 = b0x / a0x;
		b1 = b1x / a0x;
		b2 = b2x / a0x;

		// fine-tune DC-gain = 1.0
		delta = (b0 + b1 + b2) / (ONE + a1 + a2);
		b0 /= delta;
		b1 /= delta;
		b2 /= delta;
	}
	void settle( const datatype &input)
	{
		basetype tuning = ONE  / ( ONE + a1 + a2);
		old.setAllValues( input * tuning);
	}
	datatype respond( const datatype &input)
	{
		this->input=input;;
		datatype x = input - old.getValueAt(-1) * a1 - old.getValueAt(-2) * a2;
		output = x * b0 + old.getValueAt(-1) * b1 + old.getValueAt(-2) * b2;
		old.pushValue( x);
		return output;
	}
	datatype get_output( void) const
	{
	  return output;
	}
	datatype get_last_input( void) const
	{
	  return input;
	}
private:
	RingBuffer<datatype,2> old;
	basetype b0, b1, b2, a1, a2;    //!< z-transformed transfer-function (b=nominator)
	datatype input;
	datatype output;
};

#endif /* PT2_H_ */
