/***********************************************************************//**
 * @file		aiborne_detector.h
 * @brief		reports recent landing once
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * 			air density formula developed by Philipp Puetz
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

#ifndef NAV_ALGORITHMS_AIRBORNE_DETECTOR_H_
#define NAV_ALGORITHMS_AIRBORNE_DETECTOR_H_

//! observation of the aircraft state (ground / flying)
class airborne_detector_t
{
public:
  airborne_detector_t( void)
  : just_landed( false),
    airborne_counter( 0)
  {}
  void report_to_be_airborne( bool yes)
  {
    if( yes)
      {
	if( airborne_counter < LEVEL)
	  ++ airborne_counter;
      }
    else
      {
      if( airborne_counter > 0)
	{
	  --airborne_counter;
	  if( airborne_counter == 0)
	      just_landed = true;
	}
      }
  }
    bool detect_just_landed( void)
    {
      if( just_landed)
	{
	  just_landed = false;
	  airborne_counter=0;
	  return true;
	}
      return false;
    }
private:
  enum { LEVEL = 200}; // 20s @ 10Hz
  bool just_landed;
  unsigned airborne_counter;
};

#endif /* NAV_ALGORITHMS_AIRBORNE_DETECTOR_H_ */
