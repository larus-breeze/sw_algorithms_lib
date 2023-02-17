/***********************************************************************//**
 * @file 		soaring_flight_averager.h
 * @brief 		specialized averager for circling and straight flight
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

#ifndef SMART_AVERAGER_H_
#define SMART_AVERAGER_H_

#include "pt2.h"

#define ONE_DIV_2PI 0.159155f
#define PI_TIMES_2 6.2832f

//! template for an average filter for circling and straight flight
template<class value_t>
  class soaring_flight_averager
  {
  public:
    soaring_flight_averager (float cutoff_div_fsample) :
	active_state (STRAIGHT_FLIGHT), averager (cutoff_div_fsample), present_output (
	    0), used_sectors (0)
    {
    }

    const value_t&
    get_value (void) const
    {
      return present_output;
    }

    void
    update (const value_t &current_value, float heading,
	    circle_state_t new_state)
    {
      if (heading < ZERO)
	heading += PI_TIMES_2;

      switch (active_state)
	{
	case TRANSITION: // impossible, just to keep the compiler calm
	default:
	  break;
	case STRAIGHT_FLIGHT:
	  if (new_state == CIRCLING)
	    {
	      active_state = CIRCLING;

	      used_sectors = 0;
	      // use a little trick to provide a smooth transition
	      fill_recordings_with_old_average ();
	      record_input (current_value, heading);
	    }
	  else
	    present_output = averager.respond (current_value);
	  break;
	case CIRCLING:
	  if (new_state == STRAIGHT_FLIGHT)
	    {
	      active_state = STRAIGHT_FLIGHT;
	      averager.settle (present_output); // smooth transition
	    }
	  else
	    record_input (current_value, heading);

	  // prepare new output here @ input sample rate for performance reasons

	  value_t average =
	    { 0 };
	  unsigned used_sectors_count = 0;

	  for (unsigned i = 0; i < N_SECTORS; ++i)
	    {
	      if (used_sectors & (1 << i)) // if this sector is in use
		{
		  average = average + sector_averages[i];
		  ++used_sectors_count;
		}
	    }

	  present_output = average * (1.0f / (float) used_sectors_count); // as division may not be implemented

	  break;
	}
    }

    void
    record_input (value_t current_value, float heading)
    {
      unsigned index = (unsigned) (heading * ONE_DIV_2PI * N_SECTORS);
//      ASSERT(index < N_SECTORS);

      sector_averages[index] = sector_averages[index] * 0.75f
	  + current_value * 0.25f;

      used_sectors |= (1 << index); // set the used bit
    }

    void
    reset (void)
    {
      present_output =
	{ 0 };
      fill_recordings_with_old_average ();
    }

    bool
    circle_completed (void) const
    {
      return used_sectors == (1 << N_SECTORS) - 1;
    }

    void
    fill_recordings_with_old_average (void)
    {
      used_sectors = (1 << N_SECTORS) - 1; // all sectors
      for (unsigned i = 0; i < N_SECTORS; ++i)
	{
	  sector_averages[i] = present_output;
	}
    }

private:

    enum
    {
      N_SECTORS = 16
    };

    circle_state_t active_state;
    pt2<value_t, float> averager; // IIR-averager for straight flight
    value_t present_output; // maintained to save computing time
    value_t sector_averages[N_SECTORS]; // boxcar averager for circling flight
    uint32_t used_sectors; // bitfield
  };

#endif /* SMART_AVERAGER_H_ */
