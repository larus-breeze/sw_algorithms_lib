/***********************************************************************//**
 * @file		ascii_support.cpp
 * @brief		Simple and fast ASCII converters (implementation)
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

#include "embedded_memory.h"
#include "embedded_math.h"
#include "ascii_support.h"
#include <stdlib.h>

float string2float(char *input)
{
	float result = 0.0;
	bool negative;

	while( *input && ((*input == ' ') || (*input == '\t')))
	  ++input;

	if( *input == '-')
	  {
	    negative = true;
	    ++input;
	  }
	else
	  negative = false;

	while( *input && (*input != '.'))
	{
		if( (*input < '0') || (*input > '9'))
			return result;
		result = result * 10.0f + (float)(*input - '0');
		++input;
	}
	++input;
	float factor = 0.1f;
	while( (*input >= '0') && (*input <= '9'))
	{
		result = result + factor * (float)(*input - '0');
		factor *= 0.1f;
		++input;
	}
	if( *input == 'e')
	  {
	    int exponent=atoi( input+1);
	    while( exponent > 0)
	      {
		result *= 10.0f;
		--exponent;
	      }
	    while( exponent < 0)
	      {
		result /= 10.0f;
		++exponent;
	      }
	  }
	return negative ? -result : result;
}

static ROM char ASCIItable[]="zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz";

char* itoa( char* result, int value, int base)
{
	// check that the base if valid
	if (base < 2 || base > 36) { *result = '\0'; return result; }

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = ASCIItable[35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

static ROM char hexdigits[] = "0123456789abcdef";

char * utox( char* result, uint32_t value, uint8_t nibbles)
{
	for( int i=0; i < nibbles; ++i)
	{
		*result = hexdigits[(value >> 4*(nibbles-1)) & 0x0f];
		value <<= 4;
		++result;
	}
	*result=0;
	return result;
}

char * lutox( char* result, uint64_t value)
{
  utox(  result, (uint32_t)(value >> 32), 8);
  utox( result+8, value & 0xffffffff, 8);
  result[16]=0;
  return result+16;
}

char * my_itoa( char * target, int value)
 {
 	if( value < 0)
 	{
 		*target++ = '-';
 		return my_itoa( target, -value);
 	}
 	if( value < 10)
 	{
 		*target++ = (char)(value + '0');
 		*target=0;
 		return target;
 	}
 	else
 	{
 		target = my_itoa( target, value / 10);
 		return my_itoa( target, value % 10);
 	}
 }

char * my_ftoa( char * target, float value)
 {
 	if( value < 0.0f)
 	{
 		*target++='-';
 		value = -value;
 	}
 	else if( value == 0.0f)
 	{
 		*target++='0';
 		*target++='.';
 		*target++='0';
 		*target++=0;
 		return target;
 	}

 	int exponent=0;

 	while( value >= 10.0f)
 	{
 		value /= 10.0f;
 		++exponent;
 	}

 	while( value < 1.0f)
 	{
 		value *= 10.0f;
 		--exponent;
 	}

 	uint8_t digit = (uint8_t)value;
 	value -= (float)digit;
 	*target++ = (char)(digit + '0');
 	*target++ = '.';

 	value *= 1000000.0f;
 	unsigned fractional_number = round( value);
 	for( unsigned digit=0; digit<6; ++digit)
 	  {
 	    target[5-digit] = (char)(fractional_number % 10 + '0');
 	    fractional_number /= 10;
 	  }
 	target+=6;
 	*target++='e';
 	return( my_itoa( target, (int)exponent));
 }

void portable_ftoa ( char* res, float value, unsigned  no_of_decimals, unsigned res_len )
{
//	ASSERT( no_of_decimals <= res_len-2);

	unsigned i=no_of_decimals;
	while( i-- > 0)
		value *=10.0f;

	int number;
	char sign;

	if( value < 0.0f)
	{
		sign = '-';
		number = (int)( -value + 0.5f);
	}
	else
		{
		sign = ' ';
		number = (int)( value + 0.5f);
		}

	char * target = res + res_len;
	*target-- = 0;

	for( i=no_of_decimals; i; --i)
	{
		*target-- = (char)(number % 10 + '0');
		number /= 10;
	}

	*target-- = '.';
	if( number == 0)
	{
		*target -- = '0';
	}
	else while(( number > 0) && ( target > res+1))
	{
		*target-- = (char)(number % 10 + '0');
		number /= 10;
	}

	*target-- = sign;

	while( target >= res)
		*target-- = ' ';

}

//! signed integer to ASCII returning the string end
char * format_integer( char *s, int32_t value)
{
  if( value < 0)
    {
      *s++='-';
      return format_integer( s, -value);
    }
  if( value < 10)
      *s++ = value + '0';
    else
    {
      s = format_integer( s, value / 10);
      *s++ = value % 10 + '0';
    }
  *s=0;
  return s;
}

