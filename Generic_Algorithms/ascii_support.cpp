/** ***********************************************************************
 * @file		ascii_support.c
 * @brief		Simple and fast ASCII converters
 * @author		Dr. Klaus Schaefer
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

char* itoa(int value, char* result, int base)
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

char * utox(uint32_t value, char* result, uint8_t nibbles)
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

char * lutox(uint64_t value, char* result)
{
  utox( (uint32_t)(value >> 32), result, 8);
  utox( value & 0xffffffff, result+8, 8);
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
 	unsigned fractional_number = ROUND( value);
 	for( unsigned digit=0; digit<6; ++digit)
 	  {
 	    target[5-digit] = (char)(fractional_number % 10 + '0');
 	    fractional_number /= 10;
 	  }
 	target+=6;
 	*target++='e';
 	return( my_itoa( target, (int)exponent));
 }

void portable_ftoa ( float value, char* res, unsigned  no_of_decimals, unsigned res_len )
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

