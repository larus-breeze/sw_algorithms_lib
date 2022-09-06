/** ***********************************************************************
 * @file		navigator.cpp
 * @brief		maintain speed attitude and position data
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef NAVIGATORT_H_
#define NAVIGATORT_H_

#include <system_configuration.h>
#include <AHRS.h>
#include "GNSS.h"
#include "differentiator.h"
#include "smart_averager.h"
#include "atmosphere.h"
#include "flight_observer.h"
#include "data_structures.h"

class navigator_t
{
public:
  navigator_t (void)
	:ahrs (0.01f),
#if PARALLEL_MAGNETIC_AHRS
	 ahrs_magnetic (0.01f),
#endif
	 atmosphere (101325.0f),
	 vario_integrator( configuration( VARIO_INT_TC)),
	 wind_average_observer( configuration( MEAN_WIND_TC)),
	 relative_wind_observer( configuration( MEAN_WIND_TC)),
	 corrected_wind_averager( 1.0f / 15.0f / 10.0f), 	// 15s @ 10Hz
	 GNSS_speed( ZERO),
	 GNSS_negative_altitude( ZERO),
	 TAS_averager(1.0f / 1.0f / 100.0f)
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
  void update_pressure_and_altitude( float pressure, float MSL_altitude)
  {
    atmosphere.set_pressure(pressure);
  }

  void reset_altitude( void)
  {
    flight_observer.reset( atmosphere.get_negative_altitude(), GNSS_negative_altitude);
  }
  /**
   * @brief update pitot pressure
   * called @ 100 Hz
   */
  void update_pitot( float pressure)
  {
    pitot_pressure=pressure;
    TAS = atmosphere.get_TAS_from_dynamic_pressure ( pitot_pressure);
    IAS = atmosphere.get_IAS_from_dynamic_pressure ( pitot_pressure);
    TAS_averager.respond(TAS);
  }
  /**
   * @brief update AHRS from IMU
   * called @ 100 Hz, triggers all fast calculations
   */
  void update_IMU( const float3vector &acc, const float3vector &mag, const float3vector &gyro);
  /**
   * @brief update navigation GNSS
   * called @ 10 Hz
   */

  void update_GNSS( const coordinates_t &coordinates /* , const float3vector & _GNSS_acceleration*/);

  /**
   * @brief return aggregate flight observer
   */
  const flight_observer_t &get_flight_observer( void) const
    {
    return flight_observer;
    }

  void set_attitude( float roll, float nick, float yaw)
  {
    ahrs.set_from_euler(roll, nick, yaw);
#if PARALLEL_MAGNETIC_AHRS
    ahrs_magnetic.set_from_euler(roll, nick, yaw);
#endif
  }

  float get_IAS( void) const
  {
    return IAS;
  }

  float get_declination( void) const
  {
    return ahrs.get_declination();
  }

private:
  AHRS_type 		ahrs;
  atmosphere_t 		atmosphere;
  flight_observer_t 	flight_observer;

#if PARALLEL_MAGNETIC_AHRS
  AHRS_type	ahrs_magnetic;
#endif

  float 	pitot_pressure;
  float 	TAS;
  float 	IAS;

  float3vector 	GNSS_velocity; //!< 3-dim velocity
  float		GNSS_speed;	//!< ground speed as reported from GNSS
  float3vector 	GNSS_acceleration;
  float 	GNSS_heading;
  float 	GNSS_negative_altitude;
  unsigned	GNSS_fix_type;

  smart_averager< float> 	vario_integrator;
  smart_averager< float3vector, true> wind_average_observer; // configure wind average clamping on first circle
  smart_averager< float3vector> relative_wind_observer;
  pt2<float3vector,float> corrected_wind_averager;
  pt2<float,float> TAS_averager;
};

#endif /* NAVIGATORT_H_ */
