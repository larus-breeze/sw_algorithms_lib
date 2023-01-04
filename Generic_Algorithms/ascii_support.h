/***********************************************************************//**
 * @file		ascii_support.h
 * @brief		Simple and fast ASCII converters
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

#ifndef ASCII_SUPPORT_H_
#define ASCII_SUPPORT_H_

#include <stdint.h>
#ifdef _WIN32
# include <stdlib.h>
#endif

char * my_itoa( char * target, int value);
char * my_ftoa( char * target, float value);

#ifdef __cplusplus

char * utox(uint32_t value, char* result, uint8_t nibbles = 8);
char * lutox(uint64_t value, char* result);

extern "C"
 {
#endif /* __cplusplus */

float string2float(char *input);
char* ftoa(float Value, char* Buffer);
#ifndef _WIN32
char* itoa(int value, char* result, int base=10);
#endif

inline char * format_2_digits( char * target, uint32_t data)
{
  data %= 100;
  *target++ = (char)(data / 10 + '0');
  *target++ = (char)(data % 10 + '0');
  *target = 0; // just be sure string is terminated
  return target;
}

inline char *append_string( char *target, const char *source)
{
  while( *source)
      *target++ = *source++;
  *target = 0; // just to be sure :-)
  return target;
}

#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif /* ASCII_SUPPORT_H_ */
