/***********************************************************************//**
 * @file		navigator.h
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

#ifndef NAVIGATORT_H_
#define NAVIGATORT_H_

#include <system_configuration.h>
#include <AHRS.h>
#include <soaring_flight_averager.h>
#include "GNSS.h"
#include "differentiator.h"
#include "atmosphere.h"
#include "flight_observer.h"
#include "data_structures.h"
#include "accumulating_averager.h"

//! organizes horizontal navigation, wind observation and variometer
class navigator_t
{
public:
  navigator_t (void)
	:ahrs (0.01f),
#if DEVELOPMENT_ADDITIONS
	 ahrs_magnetic (0.01f),
#endif
	 atmosphere (101325.0f),
	 flight_observer(),
	 air_pressure_resampler_100Hz_10Hz(0.04f), // f/fc = 80% * 0.5 * 0.1
	 pitot_pressure(0.0f),
	 TAS( 0.0f),
	 IAS( 0.0f),
	 GNSS_velocity(),
	 GNSS_speed(),
	 GNSS_acceleration(),
	 GNSS_heading(),
	 GNSS_negative_altitude( ZERO),
	 GNSS_fix_type( 0),
	 vario_integrator( configuration( VARIO_INT_TC) < 0.25f
	   ? configuration( VARIO_INT_TC) // normalized stop frequency given, old version
	   : (FAST_SAMPLING_TIME / configuration( VARIO_INT_TC) ) ), // time-constant given, new version
	 instant_wind_averager( configuration( WIND_TC)  < 0.25f
	   ? configuration( MEAN_WIND_TC) * 10.0f // WIND_TC designed for 100Hz but now used at 10 Hz
	   : (SLOW_SAMPLING_TIME / configuration( MEAN_WIND_TC) ) ),
	 wind_average_observer( configuration( MEAN_WIND_TC) < 0.25f
	   ? configuration( MEAN_WIND_TC)
	   : (FAST_SAMPLING_TIME / configuration( MEAN_WIND_TC) ) ),
	 relative_wind_observer( configuration( MEAN_WIND_TC) < 0.25f
	   ? configuration( MEAN_WIND_TC) * 10.0f
	   : (SLOW_SAMPLING_TIME / configuration( MEAN_WIND_TC) ) ),
	 corrected_wind_averager( configuration( MEAN_WIND_TC)  < 0.25f
	   ? configuration( MEAN_WIND_TC) * 10.0f
	   : (SLOW_SAMPLING_TIME / configuration( MEAN_WIND_TC) ) ),
	 circling_wind_averager(),
	 TAS_averager(1.0f / 1.0f / 100.0f),
	 IAS_averager(1.0f / 1.0f / 100.0f),
	 old_circling_state( STRAIGHT_FLIGHT),
	 wind_obsolete( true),
	 last_wind({0}),
	 last_wind_average({0}),
	 last_headwind(0.0f),
	 last_crosswind(0.0f)

  {};

  void set_density_data( float temperature, float humidity)
  {
    if( ! isnan( temperature) && ! isnan( humidity) )
      atmosphere.set_ambient_air_data( CLIP( temperature, -40.0f, 50.0f), CLIP( humidity, 0.0f, 1.0f));
    else
      atmosphere.disregard_ambient_air_data();
  }
  void initialize_QFF_density_metering( float MSL_altitude)
  {
    atmosphere.initialize( MSL_altitude);
  }

  void feed_QFF_density_metering( float pressure, float MSL_altitude)
  {
    atmosphere.feed_QFF_density_metering( pressure, MSL_altitude);
  }

  void disregard_density_data( void)
  {
    atmosphere.disregard_ambient_air_data();
  }

  void report_data( output_data_t &d);

  void set_from_add_mag ( const float3vector &acc, const float3vector &mag)
  {
    ahrs.attitude_setup(acc, mag);
  }
  void set_from_euler ( float r, float n, float y)
  {
    ahrs.set_from_euler(r, n, y);
  }
  /**
   * @brief update absolute pressure
   * called @ 100 Hz
   */
  void update_pressure( float pressure)
  {
    atmosphere.set_pressure(pressure);
    air_pressure_resampler_100Hz_10Hz.respond(pressure);
  }

  void reset_altitude( void)
  {
    flight_observer.reset( atmosphere.get_negative_altitude(), GNSS_negative_altitude);
  }
  /**
   * @brief update pitot pressure
   * called @ 100 Hz
   *
   * Calculate actual TAS and IAS from pitot pressure and density
   */
  void update_pitot( float pressure)
  {
    pitot_pressure = pressure < 0.0f ? 0.0f : pressure;
    if( pitot_pressure > 4500.0f)
      return; // ignore implausible spikes

    TAS = atmosphere.get_TAS_from_dynamic_pressure ( pitot_pressure);
    if( TAS > 5.0f)
      TAS_averager.respond(TAS);
    else
      TAS_averager.settle(0.0f); // avoid underflow on decay

    IAS = atmosphere.get_IAS_from_dynamic_pressure ( pitot_pressure);
    if( IAS > 5.0f)
      IAS_averager.respond(IAS);
    else
      IAS_averager.settle(0.0f); // avoid underflow on decay
  }

  /**
   * @brief update AHRS from IMU
   *
   * to be called @ 100 Hz, triggers all fast calculations,
   * especially AHRS attitude data and fast flight-observer stuff
   */
  void update_every_10ms( const float3vector &acc, const float3vector &mag, const float3vector &gyro);

  /**
     * @brief slow update flight observer data
     *
     * to be called @ 10 Hz
     * calculate wind data and vario average for "vario integrator"
     */
  void update_every_100ms( const coordinates_t &coordinates);

    /**
       * @brief update on new navigation data from GNSS
       *
       * to be called @ 50 .. 100 Hz (GNSS-dependent)
       * set new position fix etc.
       */
  void update_GNSS_data( const coordinates_t &coordinates);

  /**
   * @brief return aggregate flight observer
   */
//  const flight_observer_t &get_flight_observer( void) const
//    {
//    return flight_observer;
//    }

  void set_attitude( float roll, float nick, float yaw)
  {
    ahrs.set_from_euler(roll, nick, yaw);
#if DEVELOPMENT_ADDITIONS
    ahrs_magnetic.set_from_euler(roll, nick, yaw);
#endif
  }

  float get_IAS( void) const
  {
    return IAS;
  }

  const float3vector & get_relative_wind( void) const
  {
    return relative_wind_observer.get_value();
  }

  float3vector report_instant_wind( void) const
  {
    if( ahrs.get_circling_state() != STRAIGHT_FLIGHT)
      return wind_average_observer.get_value(); // report last circle mean
    else
      return instant_wind_averager.get_output(); // report short-term average
  }
  
  float3vector report_average_wind( void) const
  {
    if( ahrs.get_circling_state() == CIRCLING)
      return circling_wind_averager.get_average();
    else
      return wind_average_observer.get_value();
  }

  float3vector report_corrected_wind( void) const
  {
    return corrected_wind_averager.get_output();
  }

private:
  AHRS_type	ahrs;
#if DEVELOPMENT_ADDITIONS
  AHRS_type	ahrs_magnetic;
#endif
  atmosphere_t 		atmosphere;
  flight_observer_t 	flight_observer;

  pt2<float,float> air_pressure_resampler_100Hz_10Hz;
  float 	pitot_pressure;
  float 	TAS;
  float 	IAS;

  float3vector 	GNSS_velocity; //!< 3-dim velocity
  float		GNSS_speed;	//!< ground speed as reported from GNSS
  float3vector 	GNSS_acceleration;
  float 	GNSS_heading;
  float 	GNSS_negative_altitude;
  unsigned	GNSS_fix_type;

  soaring_flight_averager< float, false, false> vario_integrator;
  pt2<float3vector,float> instant_wind_averager;
  soaring_flight_averager< float3vector, true> wind_average_observer; // configure wind average clamping on first circle
  soaring_flight_averager< float3vector, false, false> relative_wind_observer;
  pt2<float3vector,float> corrected_wind_averager;
  accumulating_averager < float3vector> circling_wind_averager;
  pt2<float,float> TAS_averager;
  pt2<float,float> IAS_averager;
  circle_state_t old_circling_state;

  // workaround for different sampling rates 10Hz and 100Hz
  bool wind_obsolete; //!< wind has not been updated recently
  float3vector last_wind;
  float3vector last_wind_average;
  float last_headwind;
  float last_crosswind;
};

#endif /* NAVIGATORT_H_ */
