/***********************************************************************//**
 * @file		delay_line.h
 * @brief		delay line for testing purposes
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

#ifndef DELAY_LINE_H_
#define DELAY_LINE_H_

//! delay line for testing purposes (template)
template <class data_t, unsigned length> class delay_line
{
public:
  delay_line( void)
    :storage{0},
     ptr(storage)
    {};
  data_t respond( const data_t &right)
  {
    data_t retv=*ptr;
    *ptr=right;

    ++ptr;

    if( ptr >= storage + length)
      ptr=storage;

    return retv;
  }
private:
  data_t storage[length];
  data_t * ptr;
};

#endif /* DELAY_LINE_H_ */
