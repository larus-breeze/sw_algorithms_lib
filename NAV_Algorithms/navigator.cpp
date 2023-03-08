/***********************************************************************//**
 * @file		navigator.cpp
 * @brief		combine sensor observations to provide flight-relevant data
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

#include <navigator.h>

// to be called at 100 Hz
void navigator_t::update_every_10ms (
    const float3vector &acc,
    const float3vector &mag,
    const float3vector &gyro)
{
  wind_obsolete = true;

  ahrs.update( gyro, acc, mag,
	    GNSS_acceleration,
	    GNSS_heading,
	    GNSS_fix_type == (SAT_FIX | SAT_HEADING));

#if DEVELOPMENT_ADDITIONS
  ahrs_magnetic.update_compass(
	  gyro, acc, mag,
	  GNSS_acceleration);
#endif
  float3vector heading_vector;
  heading_vector[NORTH] = ahrs.get_north ();
  heading_vector[EAST]  = ahrs.get_east  ();
  heading_vector[DOWN]  = ahrs.get_down  (); // todo: do we need this one ?

  flight_observer.update_every_10ms (
      GNSS_velocity,
      GNSS_acceleration,
      ahrs.get_nav_acceleration (),
      heading_vector,
      GNSS_negative_altitude,
      atmosphere.get_negative_altitude(),
      TAS,
      IAS,
      ahrs.get_circling_state(),
      wind_average_observer.get_value(),
      (GNSS_fix_type != 0)
      );
}

void navigator_t::update_GNSS_data( const coordinates_t &coordinates)
{
  GNSS_fix_type = coordinates.sat_fix_type;

  if (coordinates.sat_fix_type == SAT_FIX_NONE) // presently no GNSS fix
    {
      GNSS_velocity = 	{ 0 };
      GNSS_acceleration = { 0 };
      GNSS_heading = 0.0f;
      GNSS_negative_altitude = 0.0f;
      GNSS_speed = 0.0f;
    }
  else
    {
      GNSS_velocity = coordinates.velocity;
      GNSS_acceleration = coordinates.acceleration;
      GNSS_heading = coordinates.relPosHeading;
      GNSS_negative_altitude = coordinates.position.e[DOWN];
      GNSS_speed = coordinates.speed_motion;
    }
}

// to be called at 10 Hz
void navigator_t::update_every_100ms (const coordinates_t &coordinates)
{
  atmosphere.feed_QFF_density_metering(
	air_pressure_resampler_100Hz_10Hz.get_output(),
	flight_observer.get_filtered_GNSS_altitude());

  atmosphere.update_density_correction(); // here because of the 10 Hz call frequency

  instant_wind_averager.respond( flight_observer.get_instant_wind());

  wind_average_observer.update( flight_observer.get_instant_wind(), // do this here because of the update rate 10Hz
				ahrs.get_euler ().y,
				ahrs.get_circling_state ());

  float3vector relative_wind_NAV  = flight_observer.get_instant_wind() - wind_average_observer.get_value();
  float3vector relative_wind_BODY =  ahrs.get_body2nav().reverse_map(relative_wind_NAV);

  if( ahrs.get_circling_state () == STRAIGHT_FLIGHT && old_circling_state == TRANSITION)
    relative_wind_observer.reset({0});
  else
    relative_wind_observer.update(relative_wind_BODY, ahrs.get_euler ().y, ahrs.get_circling_state ());

  if(( ahrs.get_circling_state () == CIRCLING))
    {
      if(old_circling_state == TRANSITION) // when starting to circle
	{
	  circling_wind_averager.reset(wind_average_observer.get_value(), 100);
	  relative_wind_observer.reset({0});
	}
    }

  float3vector wind_correction_nav = ahrs.get_body2nav() * relative_wind_observer.get_value();
  wind_correction_nav.e[DOWN]=0.0f;

  corrected_wind_averager.respond( flight_observer.get_instant_wind() - wind_correction_nav);
  circling_wind_averager.update( flight_observer.get_instant_wind() - wind_correction_nav);

  vario_integrator.update (flight_observer.get_vario_GNSS(), // here because of the update rate 10Hz
			   ahrs.get_euler ().y,
			   ahrs.get_circling_state ());

  old_circling_state = ahrs.get_circling_state ();
  wind_obsolete = false;
}

//! copy all navigator data into output_data structure
void navigator_t::report_data( output_data_t &d)
{
    d.TAS 			= TAS_averager.get_output();
    d.IAS 			= IAS_averager.get_output();

    d.euler			= ahrs.get_euler();
    d.q				= ahrs.get_attitude();

    d.vario			= flight_observer.get_vario_GNSS(); // todo pick one vario
    d.vario_pressure		= flight_observer.get_vario_pressure();
    d.integrator_vario		= vario_integrator.get_value();
    d.vario_uncompensated 	= flight_observer.get_vario_uncompensated_GNSS();

    if( ! wind_obsolete) // we have new wind information
      {
	// reported wind smoothening to avoid hard changes during transitions
	    last_wind = d.wind			= last_wind * 0.95f + report_instant_wind() * 0.05f;
	    last_wind_average = d.wind_average	= last_wind_average * 0.95f + report_average_wind() * 0.05f;

#if DEVELOPMENT_ADDITIONS
	    last_headwind  = d.headwind 	= get_relative_wind().e[FRONT];
	    last_crosswind = d.crosswind	= get_relative_wind().e[RIGHT];
#endif
      }
    else // wind has not recently been updated
      {
	d.wind 		= last_wind;
	d.wind_average	= last_wind_average;

#if DEVELOPMENT_ADDITIONS
	d.headwind 	= last_headwind;
	d.crosswind	= last_crosswind;
#endif
      }

    d.speed_compensation_TAS 	= flight_observer.get_speed_compensation_IAS();
    d.speed_compensation_GNSS 	= flight_observer.get_speed_compensation_GNSS();
    d.effective_vertical_acceleration
				= flight_observer.get_effective_vertical_acceleration();

    d.circle_mode 		= ahrs.get_circling_state();
    d.turn_rate			= ahrs.get_turn_rate();
    d.slip_angle		= ahrs.getSlipAngle();
    d.nick_angle		= ahrs.getNickAngle();
    d.G_load			= ahrs.get_G_load();
    d.pressure_altitude		= - atmosphere.get_negative_altitude();
    d.magnetic_disturbance	= ahrs.getMagneticDisturbance();

#if DEVELOPMENT_ADDITIONS

    d.nav_correction		= ahrs.get_nav_correction();
    d.gyro_correction		= ahrs.get_gyro_correction();
    d.nav_acceleration_gnss 	= ahrs.get_nav_acceleration();
    d.nav_induction_gnss 	= ahrs.get_nav_induction();

    d.euler_magnetic		= ahrs_magnetic.get_euler();
    d.q_magnetic		= ahrs_magnetic.get_attitude();
    d.nav_acceleration_mag 	= ahrs_magnetic.get_nav_acceleration();
    d.nav_induction_mag 	= ahrs_magnetic.get_nav_induction();

    d.HeadingDifferenceAhrsDgnss = ahrs.getHeadingDifferenceAhrsDgnss();
    d.QFF			= atmosphere.get_QFF();
    d.air_density		= atmosphere.get_density();
    d.satfix			= (float)(d.c.sat_fix_type);
    d.inst_wind_N		= flight_observer.get_instant_wind().e[NORTH];
    d.inst_wind_E		= flight_observer.get_instant_wind().e[EAST];
    d.inst_wind_corrected_N	= report_corrected_wind().e[NORTH];
    d.inst_wind_corrected_E	= report_corrected_wind().e[EAST];
#endif
}
