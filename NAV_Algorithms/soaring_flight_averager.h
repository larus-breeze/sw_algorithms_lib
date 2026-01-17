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

#include <AHRS.h>
#define SQR(x) ((x)*(x))
#include "pt2.h"
#include "embedded_math.h"

#define ONE_DIV_2PI 0.159155f
#define PI_TIMES_2 6.2832f

//! template for an average filter for circling and straight flight
template<class value_t, bool CLAMP_OUTPUT_FIRST_CIRCLE = false, bool SOFT_TAKEOFF = true>
  class soaring_flight_averager
  {
  public:
    soaring_flight_averager (float normalized_stop_frequency) :
	active_state (STRAIGHT_FLIGHT),
	averager (normalized_stop_frequency),
	present_output(0),
	old_sector(0)
    {
      for (unsigned index = 0; index < N_SECTORS; ++index)
	  {
	    sector_averages[index] = {0};
	    sector_sample_count[index] = 0;
	  }
    };

    void tune( float normalized_stop_frequency)
    {
	averager.tune( normalized_stop_frequency);
    }

    const value_t & get_output (void) const
    {
      return present_output;
    }

    void
    update (const value_t &current_value, float heading,
	    flight_state_t new_state)
    {
      if (heading < ZERO)
	heading += PI_TIMES_2;

      switch (active_state)
	{
	case STRAIGHT_FLIGHT:
	  if (new_state == CIRCLING)
	    {
	      active_state = CIRCLING;

	      // use a little trick to provide a smooth transition
	      if( SOFT_TAKEOFF)
		fill_recordings_with_value (averager.get_output());
	      else
		reset( current_value);

	      record_input (current_value, heading);
	      present_output = get_boxcar_average();
	    }
	  else
	    {
	      present_output = averager.respond (current_value);
	    }
	  break;
	case CIRCLING:
	  if (new_state == STRAIGHT_FLIGHT)
	    {
	      active_state = STRAIGHT_FLIGHT;
	      reset( present_output); // smooth transition
	    }

	  record_input (current_value, heading);

	  if( CLAMP_OUTPUT_FIRST_CIRCLE)
	    {
	      if( circle_completed())
		present_output = get_boxcar_average();
	    }
	  else
	    present_output = get_boxcar_average();

	  break;
	case TRANSITION: // impossible, just to keep the compiler calm
	default:
	    assert( 0);
	  break;
	}
    }

    void record_input (value_t current_value, volatile float heading)
    {
      unsigned index = find_sector_index( heading);

      if( old_sector != index) // on sector change
	{
	  old_sector = index;
	  if( sector_sample_count[index] > 1) // if sector has been used in the circle before
	    {
	      // reset sector
	      sector_sample_count[index] = 0;
	      sector_averages[index] = {0};
	    }
	}

      sector_averages[index] += current_value;
      ++ sector_sample_count[index];
    }

    void reset( value_t value)
    {
      averager.settle(value);
      for (unsigned i = 0; i < N_SECTORS; ++i)
	{
	  sector_averages[i] = {0};
	  sector_sample_count[i] = 0;
	}
      present_output = value;
    }

    bool circle_completed (void) const
    {
      for (unsigned index = 0; index < N_SECTORS; ++index)
	if( sector_sample_count[index] == 0)
	  return false;
      return true;
    }

    void relax( void)
    {
      averager.settle({0});
    }

  private:

    value_t get_boxcar_average( void)
    {
      value_t average = { 0 };

      unsigned number_of_used_sectors = 0;
      for (unsigned index = 0; index < N_SECTORS; ++index)
	if( sector_sample_count[index] > 0)
	  {
	    average = average + sector_averages[index] * ( ONE / sector_sample_count[index]);
	    ++number_of_used_sectors;
	  }
      if( number_of_used_sectors == 0)
	return {0};
      else
	return average * (1.0f / (float) number_of_used_sectors); // as division may not be implemented
    }

    unsigned find_sector_index( float heading)
    {
      unsigned retv = (unsigned) (heading * ONE_DIV_2PI * N_SECTORS);
      if( retv >= N_SECTORS) // prevent rounding errors
	retv = N_SECTORS -1;
      return retv;
    }

    void fill_recordings_with_value ( value_t value)
    {
      for (unsigned i = 0; i < N_SECTORS; ++i)
	{
	  sector_averages[i] = value;
	  sector_sample_count[i] = 1;
	}
    }

    enum { N_SECTORS = 16};

    flight_state_t active_state;
    pt2<value_t, float> averager; // IIR-averager for straight flight
    value_t present_output; // maintained to save computing time
    value_t sector_averages[N_SECTORS]; // boxcar averager for circling flight
    unsigned sector_sample_count[N_SECTORS]; // boxcar averager for circling flight
    unsigned old_sector;
  };




#endif /* SMART_AVERAGER_H_ */
