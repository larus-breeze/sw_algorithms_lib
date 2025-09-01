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

static ROM char ASCIItable[]="zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz";

char* itoa( int value, char* result, int base)
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
 		*target=0;
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

 	value += 0.0000005f; // rounding, not truncating ...

 	uint8_t digit = (uint8_t)value;
 	value -= (float)digit;
 	*target++ = (char)(digit + '0');
 	*target++ = '.';

 	for( unsigned i=0; i<6; ++i)
 	  {
 	    value *= 10.0f;
	    digit = (uint8_t)value;
	    value -= (float)digit;
	    *target++ = (char)(digit + '0');
 	  }

 	*target++='e';
 	return( my_itoa( target, (int)exponent));
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

#define isdigit(c) (c >= '0' && c <= '9')
float my_atof(const char *s)
{
  /*
   * Copyright (C) 2014, Galois, Inc.
   * This sotware is distributed under a standard, three-clause BSD license.
   * Please see the file LICENSE, distributed with this software, for specific
   * terms and conditions.
   * Taken from here: https://github.com/GaloisInc/minlibc/blob/master/atof.c
   */

  // This function stolen from either Rolf Neugebauer or Andrew Tolmach.
  // Probably Rolf.
  float a = 0.0f;
  int e = 0;
  int c;
  while ((c = *s++) != '\0' && isdigit(c)) {
    a = a*10.0f + (float)(c - '0');
  }
  if (c == '.') {
    while ((c = *s++) != '\0' && isdigit(c)) {
      a = a*10.0f + (float)(c - '0');
      e = e-1;
    }
  }
  if (c == 'e' || c == 'E') {
    int sign = 1;
    int i = 0;
    c = *s++;
    if (c == '+')
      c = *s++;
    else if (c == '-') {
      c = *s++;
      sign = -1;
    }
    while (isdigit(c)) {
      i = i*10 + (c - '0');
      c = *s++;
    }
    e += i*sign;
  }
  while (e > 0) {
    a *= 10.0f;
    e--;
  }
  while (e < 0) {
    a *= 0.1f;
    e++;
  }
  return a;
}

