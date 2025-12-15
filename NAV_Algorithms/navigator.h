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
#include <variometer.h>
#include "GNSS.h"
#include "differentiator.h"
#include "atmosphere.h"
#include "data_structures.h"
#include "accumulating_averager.h"
#include "airborne_detector.h"
#include "wind_observer.h"

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
	 variometer(),
	 wind_observer(),
	 airborne_detector(),
	 air_pressure_resampler_100Hz_10Hz(0.025f), // = 2.5 Hz @ 100Hz
	 pitot_pressure(0.0f),
	 TAS( 0.0f),
	 IAS( 0.0f),
	 GNSS_velocity(),
	 GNSS_speed(),
	 GNSS_speed_accuracy(),
	 GNSS_acceleration(),
	 GNSS_heading(),
	 GNSS_negative_altitude( ZERO),
	 GNSS_fix_type( 0),
	 GNSS_type(GNSS_TYPE_NOT_DEFINED),
	 vario_integrator( configuration( VARIO_INT_TC) < 0.25f
	   ? configuration( VARIO_INT_TC) // normalized stop frequency given, old version
	   : (FAST_SAMPLING_TIME / configuration( VARIO_INT_TC) ) ), // time-constant given, new version
	 TAS_averager(1.0f / 1.0f / 100.0f),
	 IAS_averager(1.0f / 1.0f / 100.0f)
  {};

  void tune(void)
  {
    variometer.tune();
    vario_integrator.tune( configuration( VARIO_INT_TC) < 0.25f
	   ? configuration( VARIO_INT_TC) // normalized stop frequency given, old version
	   : (FAST_SAMPLING_TIME / configuration( VARIO_INT_TC) ) ); // time-constant given, new version
    wind_observer.tune();
    ahrs.tune();
#if DEVELOPMENT_ADDITIONS
    ahrs_magnetic.tune();
#endif
  }

  void update_magnetic_induction_data( float declination, float inclination)
  {
    ahrs.update_magnetic_induction_data( declination, inclination);
#if DEVELOPMENT_ADDITIONS
    ahrs_magnetic.update_magnetic_induction_data( declination, inclination);
#endif
  }
  void set_density_data( float temperature, float humidity)
  {
    if( ! isnan( temperature) && ! isnan( humidity) )
      atmosphere.set_ambient_air_data( CLIP( temperature, -40.0f, 50.0f), CLIP( humidity, 0.0f, 1.0f));
    else
      atmosphere.disregard_ambient_air_data();
  }

  void set_gnss_type(GNSS_configration_t type)
  {
    GNSS_type = type;
  }

  bool get_speed_accuracy_bad_status(void)
  {
    if (GNSS_type == GNSS_M9N)
    {
	if (GNSS_speed_accuracy > 0.35)
	  return true;
	return false;
    }
    else if ((GNSS_type == GNSS_F9P_F9H) || (GNSS_type == GNSS_F9P_F9P))
    {
	if (GNSS_speed_accuracy > 0.15)
	  return true;
	return false;
    }
    else
    {
	ASSERT(0);
	return true;
    }
  }

  bool get_magnetic_disturbance_bad_status(void)
  {
    if (ahrs.getMagneticDisturbance() > MAGNETIC_DISTURBANCE_LIMIT)
	return true;
    return false;
  }

  void initialize_QFF_density_metering( float MSL_altitude)
  {
    atmosphere.initialize( MSL_altitude);
  }

  void feed_QFF_density_metering( float pressure, float MSL_altitude)
  {
    atmosphere.air_density_metering( pressure, MSL_altitude);
  }

  void disregard_density_data( void)
  {
    atmosphere.disregard_ambient_air_data();
  }

  void report_data( output_data_t &d);

  void set_from_add_mag ( const float3vector &acc, const float3vector &mag)
  {
    ahrs.attitude_setup(acc, mag);
#if DEVELOPMENT_ADDITIONS
    ahrs_magnetic.attitude_setup(acc, mag);
#endif
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
    variometer.reset( atmosphere.get_negative_pressure_altitude(), GNSS_negative_altitude);
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
  void update_at_100Hz( const float3vector &acc, const float3vector &mag, const float3vector &gyro);

  /**
     * @brief slow update flight observer data
     *
     * to be called @ 10 Hz
     * calculate wind data and vario average for "vario integrator"
     * @return true if a landing has just been detected
     */
  bool update_at_10Hz();

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

  void set_attitude( float roll, float pitch, float yaw)
  {
    ahrs.set_from_euler(roll, pitch, yaw);
#if DEVELOPMENT_ADDITIONS
    ahrs_magnetic.set_from_euler(roll, pitch, yaw);
#endif
  }

private:
  AHRS_type	ahrs;
#if DEVELOPMENT_ADDITIONS
  AHRS_type	ahrs_magnetic;
#endif
  atmosphere_t 	atmosphere;
  variometer_t 	variometer;
  wind_oberserver_t wind_observer;
  airborne_detector_t	airborne_detector;

  pt2<float,float> air_pressure_resampler_100Hz_10Hz;
  float 	pitot_pressure;
  float 	TAS;
  float 	IAS;

  float3vector 	GNSS_velocity; //!< 3-dim velocity
  float		GNSS_speed;	//!< ground speed as reported from GNSS
  float		GNSS_speed_accuracy; //!< observing gnss quality for reporting
  float3vector 	GNSS_acceleration;
  float 	GNSS_heading;
  float 	GNSS_negative_altitude;
  unsigned	GNSS_fix_type;
  GNSS_configration_t GNSS_type;

  soaring_flight_averager< float, false, false> vario_integrator;
  pt2<float,float> TAS_averager;
  pt2<float,float> IAS_averager;
};

#endif /* NAVIGATORT_H_ */
