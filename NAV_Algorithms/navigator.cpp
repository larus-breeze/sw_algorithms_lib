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
void navigator_t::update_at_100Hz (
    const float3vector &acc,
    const float3vector &mag,
    const float3vector &gyro)
{
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
  heading_vector[DOWN]  = ahrs.get_down  ();

  wind_observer.process_at_100_Hz( GNSS_velocity - heading_vector * TAS);

  flight_observer.update_at_100Hz (
      GNSS_velocity,
      ahrs.get_nav_acceleration (),
      heading_vector,
      GNSS_negative_altitude,
      atmosphere.get_negative_altitude(),
      IAS,
      wind_observer.get_speed_compensator_wind(),
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
      GNSS_negative_altitude = coordinates.position[DOWN];
      GNSS_speed = coordinates.speed_motion;
    }
}

// to be called at 10 Hz
bool navigator_t::update_at_10Hz ()
{
  bool landing_detected=false;
  atmosphere.feed_QFF_density_metering(
	air_pressure_resampler_100Hz_10Hz.get_output(),
	flight_observer.get_filtered_GNSS_altitude());

  atmosphere.update_density_correction(); // here because of the 10 Hz call frequency

  wind_observer.process_at_10_Hz( ahrs);

  vario_integrator.update (flight_observer.get_vario_GNSS(), // here because of the update rate 10Hz
			   ahrs.get_euler ().y,
			   ahrs.get_circling_state ());

  airborne_detector.report_to_be_airborne( abs( flight_observer.get_speed_compensation_GNSS()) > AIRBORNE_TRIGGER_SPEED);
  if( airborne_detector.detect_just_landed())
    {
      ahrs.write_calibration_into_EEPROM();
      landing_detected = true;
    }
  return landing_detected;
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
    d.integrator_vario		= vario_integrator.get_output();
    d.vario_uncompensated 	= flight_observer.get_vario_uncompensated_GNSS();

    d.wind 			= wind_observer.get_instant_value();
    d.wind_average		= wind_observer.get_average_value();

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
    d.air_density		= atmosphere.get_density();
    d.nav_induction_gnss 	= ahrs.get_nav_induction();

#if DEVELOPMENT_ADDITIONS

    d.QFF			= atmosphere.get_QFF();
    d.nav_correction		= ahrs.get_nav_correction();
    d.gyro_correction		= ahrs.get_gyro_correction();
    d.nav_acceleration_gnss 	= ahrs.get_nav_acceleration();

    d.euler_magnetic		= ahrs_magnetic.get_euler();
    d.q_magnetic		= ahrs_magnetic.get_attitude();
    d.nav_acceleration_mag 	= ahrs_magnetic.get_nav_acceleration();
    d.nav_induction_mag 	= ahrs_magnetic.get_nav_induction();

    d.HeadingDifferenceAhrsDgnss = ahrs.getHeadingDifferenceAhrsDgnss();
    d.satfix			= (float)(d.c.sat_fix_type);
    d.inst_wind_N		= wind_observer.get_measurement()[NORTH];
    d.inst_wind_E		= wind_observer.get_measurement()[EAST];
    d.headwind 			= wind_observer.get_headwind();
    d.crosswind			= wind_observer.get_crosswind();
    d.inst_wind_corrected_N	= wind_observer.get_corrected_wind()[NORTH];
    d.inst_wind_corrected_E	= wind_observer.get_corrected_wind()[EAST];
    for( unsigned i=0; i<4; ++i)
      d.speed_compensation[i]  	= flight_observer.get_speed_compensation(i);
    d.cross_acc_correction 	= ahrs_magnetic.get_cross_acc_correction();
    d.vario_wind_N		= wind_observer.get_speed_compensator_wind()[NORTH];
    d.vario_wind_E		= wind_observer.get_speed_compensator_wind()[EAST];
#endif
}
