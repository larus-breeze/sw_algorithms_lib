/***********************************************************************//**
 * @file		wind_observer.h
 * @brief		wind measurement system (interface)
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

#ifndef NAV_ALGORITHMS_WIND_OBSERVER_H_
#define NAV_ALGORITHMS_WIND_OBSERVER_H_

#include "float3vector.h"
#include "pt2.h"
#include "soaring_flight_averager.h"
#include "accumulating_averager.h"
#include "EEPROM_emulation.h"
#include "persistent_data.h"

class wind_oberserver_t
{
public:
  wind_oberserver_t( float sampling_frequency)
    :instant_wind_averager( configuration( WIND_TC)  < 0.25f
     ? configuration( WIND_TC)
     : (FAST_SAMPLING_TIME / configuration( WIND_TC) ) ),
    wind_average_observer( configuration( MEAN_WIND_TC) < 0.25f
     ? configuration( MEAN_WIND_TC)
     : (SLOW_SAMPLING_TIME / configuration( MEAN_WIND_TC) ) ),
    relative_wind_observer( configuration( MEAN_WIND_TC) < 0.25f
     ? configuration( MEAN_WIND_TC) * 10.0f
     : (SLOW_SAMPLING_TIME / configuration( MEAN_WIND_TC) ) ),
    corrected_wind_averager( configuration( MEAN_WIND_TC)  < 0.25f
     ? configuration( MEAN_WIND_TC) * 10.0f
     : (SLOW_SAMPLING_TIME / configuration( MEAN_WIND_TC) ) ),
    circling_wind_averager(),
    circling_state( STRAIGHT_FLIGHT),
    old_circling_state( STRAIGHT_FLIGHT),
    wind_correction_nav()
  {}

  float3vector report_instant_value( void) const
  {
    if( circling_state != STRAIGHT_FLIGHT)
      return wind_average_observer.get_value(); // report last circle mean
    else
      return instant_wind_averager.get_output(); // report short-term average
  }

  float3vector report_average_value( void) const
  {
    if( circling_state == CIRCLING)
      return circling_wind_averager.get_average();
    else
      return wind_average_observer.get_value();
  }

  float report_crosswind( void) const
  {
    return wind_correction_nav.e[RIGHT];
  }

  float report_headwind( void) const
  {
    return wind_correction_nav.e[FRONT];
  }

  float3vector report_corrected_wind( void) const
  {
    return corrected_wind_averager.get_output();
  }

  float3vector report_speed_compensator_wind( void) const
  {
    return wind_average_observer.get_value();
  }
#if 0
  float3vector report_average_wind( void) const
  {
  }
#endif
  void process_at_100_Hz( const float3vector &instant_wind)
  {
    instant_wind_averager.respond( instant_wind);
  }

  void process_at_10_Hz( const AHRS_type & ahrs)
  {
    circling_state = ahrs.get_circling_state ();
    wind_average_observer.update(
	instant_wind_averager.get_output(), // do this here because of the update rate 10Hz
    	ahrs.get_euler ().y,
	circling_state);

    float3vector relative_wind_NAV  = instant_wind_averager.get_output() - wind_average_observer.get_value();
    float3vector relative_wind_BODY =  ahrs.get_body2nav().reverse_map(relative_wind_NAV);

    if( circling_state == STRAIGHT_FLIGHT && old_circling_state == TRANSITION)
      relative_wind_observer.reset({0});
    else
      relative_wind_observer.update( relative_wind_BODY, ahrs.get_euler ().y, ahrs.get_circling_state ());

    if(( circling_state == CIRCLING))
      {
        if( old_circling_state == TRANSITION) // when starting to circle
  	{
  	  circling_wind_averager.reset( wind_average_observer.get_value(), 100);
  	  relative_wind_observer.reset({0});
  	}
      }

    wind_correction_nav = ahrs.get_body2nav() * relative_wind_observer.get_value();
    wind_correction_nav.e[DOWN]=0.0f;

    corrected_wind_averager.respond( instant_wind_averager.get_output() - wind_correction_nav);
    circling_wind_averager.update( instant_wind_averager.get_output() - wind_correction_nav);

    old_circling_state = circling_state;
  }

private:
  pt2<float3vector,float> instant_wind_averager;
  soaring_flight_averager< float3vector, true> wind_average_observer; // configure wind average clamping on first circle
  soaring_flight_averager< float3vector, false, false> relative_wind_observer;
  pt2<float3vector,float> corrected_wind_averager;
  accumulating_averager < float3vector> circling_wind_averager;
  circle_state_t circling_state;
  circle_state_t old_circling_state;
  float3vector wind_correction_nav;
};

#endif /* NAV_ALGORITHMS_WIND_OBSERVER_H_ */
