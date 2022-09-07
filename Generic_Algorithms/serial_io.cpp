/*
 * serial_io.cpp
 *
 *  Created on: Jul 3, 2013
 *      Author: schaefer
 */

#include "ascii_support.h"
#include "serial_io.h"

void serial_output::puti( int value, int base)
{
	char buffer[20];
	itoa( value, buffer, base);
	puts( buffer);
}
void serial_output::putx( int32_t value, uint8_t digits)
{
	char buffer[20];
	utox( value, buffer, digits);
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
