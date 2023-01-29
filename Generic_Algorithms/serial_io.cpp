/***********************************************************************//**
 * @file		serial_io.cpp
 * @brief		serial input output class
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

#include "ascii_support.h"
#include "serial_io.h"

void serial_output::puti( int value, int base)
{
	char buffer[20];
	itoa(  value, buffer, base);
	puts( buffer);
}
void serial_output::putx( int32_t value, uint8_t digits)
{
	char buffer[20];
	utox( buffer, value, digits);
	puts( buffer);
}
void serial_output::putf( float value)
{
	char buffer[20];
	my_ftoa( buffer, value);
	puts( buffer);
}
void serial_output::puts( const char * data)
{
	while (*data != 0)
		put( *data++);
}
void serial_output::newline( void)
{
	puts((char *)"\r\n");
}
void serial_output::blank( void)
{
	put( ' ');
}
int serial_input::geti( void)
{
  int value;
  bool negative = false;
  char c;

  while( true)
    {
      c=get_blocking();
      if( c == '-')
	  negative = true;
      else if(( c >= '0') && (c <= '9'))
	break; // numbers starting
    }

  value = c - '0'; // ASCII -> bin

  while((( c = get_blocking()) >= '0') && ( c <= '9'))
    {
      value = value * 10 + ( c - '0'); // add next digit

    }
  return negative ? -value : value;
}
