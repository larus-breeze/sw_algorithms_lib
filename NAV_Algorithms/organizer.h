/***********************************************************************//**
 * @file		organizer.h
 * @brief		combine algorithms to be used by the flight sensor
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

#ifndef ORGANIZER_H_
#define ORGANIZER_H_

#include <variometer.h>
#include "data_structures.h"
#include "navigator.h"
#include "earth_induction_model.h"
#include "sensor_orientation_setup.h"
#include "system_state.h"

//! set of algorithms and data to be used by Larus flight sensor
class organizer_t
{
public:
  organizer_t( void)
    : pitot_offset(0.0f),
      pitot_span(0.0f),
      QNH_offset(0.0f),
      magnetic_induction_update_counter(0)
  {}

//! initialization of the AHRS taking configuration data
  void initialize_before_measurement( void)
  {
    pitot_offset= configuration (PITOT_OFFSET);
    pitot_span 	= configuration (PITOT_SPAN);
    QNH_offset	= configuration (QNH_OFFSET);

    quaternion<float> q;
    q.from_euler (configuration (SENS_TILT_ROLL),
		  configuration (SENS_TILT_PITCH),
		  configuration (SENS_TILT_YAW));
    q.get_rotation_matrix (sensor_mapping);

    navigator.tune();
  }

  void fine_tune_sensor_orientation( const vector_average_collection_t & values)
  {
    ::fine_tune_sensor_orientation( values, sensor_mapping);
  }

  void update_sensor_orientation_data( const vector_average_collection_t & values)
  {
    ::update_sensor_orientation_data( values);
  }

  //! initialize the earth magnetic field data taking the observed location
  void update_magnetic_induction_data( double latitude, double longitude)
  {
    induction_values induction_data;
    induction_data = earth_induction_model.get_induction_data_at( latitude, longitude);
    if( induction_data.valid)
      navigator.update_magnetic_induction_data( induction_data.declination, induction_data.inclination);

    navigator.set_earth_rotation(latitude);
  }

  //! attitude setup after getting the first set of acceleration an magnetic data
  void initialize_after_first_measurement( output_data_t & output_data)
  {
    navigator.update_pressure( output_data.m.static_pressure - QNH_offset);
    navigator.initialize_QFF_density_metering( -output_data.c.position[DOWN]);
    navigator.reset_altitude ();

    // setup initial attitude
    float3vector acc = sensor_mapping * output_data.m.acc;
    float3vector mag = sensor_mapping * output_data.m.mag;

    if (output_data.c.sat_fix_type & SAT_HEADING)
      {
	navigator.set_attitude ( 0.0f, 0.0f, output_data.c.relPosHeading); // todo use acc data some day ?
      }
    else
      navigator.set_from_add_mag( acc, mag); // initialize attitude from acceleration + compass
  }

  //! update the navigator taking the current pressure measurements
  void on_new_pressure_data( float static_pressure, float pitot_pressure)
  {
    navigator.update_pressure( static_pressure - QNH_offset);
    navigator.update_pitot ( ( pitot_pressure - pitot_offset) * pitot_span);
  }

  //! the "SLOW" update of the observed properties
  bool update_every_100ms( output_data_t & output_data)
  {
    bool landing_detected = navigator.update_at_10Hz ();

    navigator.feed_QFF_density_metering( output_data.m.static_pressure - QNH_offset, -output_data.c.position[DOWN]);

    if( ++magnetic_induction_update_counter > 36000) // every hour
      {
	update_magnetic_induction_data( output_data.c.latitude, output_data.c.longitude);
	magnetic_induction_update_counter=0;
      }

    return landing_detected;
  }

  // initialization of the AHRS attitude if the roll pitch heading angles are known
  void set_attitude ( float roll, float pitch, float present_heading)
  {
    navigator.set_attitude ( roll, pitch, present_heading);
  }

  void set_GNSS_type(GNSS_configration_t configuration)
  {
    navigator.set_gnss_type(configuration);
  }

  //! all that needs to be done when a new data set comes from the GNSS receiver
  void update_GNSS_data( const coordinates_t &coordinates)
  {
    navigator.update_GNSS_data(coordinates);
  }

  //! the "FAST" update of the observed properties
  void update_at_100_Hz( output_data_t & output_data)
  {
    // rotate sensor coordinates into airframe coordinates
    float3vector acc  = sensor_mapping * output_data.m.acc;
    float3vector gyro = sensor_mapping * output_data.m.gyro;
#if 0 // SIMULATE_EXTERNAL_MAGNETOMETER
    float3vector mag  = output_data.external_magnetometer_reading;
    float3vector x_mag;
    x_mag[FRONT] = + mag[RIGHT];
    x_mag[RIGHT] = - mag[FRONT];
    x_mag[DOWN]  = + mag[DOWN];
    navigator.update_at_100Hz (acc, mag, gyro, x_mag, true);
#else
//    float3vector mag  = sensor_mapping * output_data.m.mag;
    float3vector mag  = output_data.m.mag;
    navigator.update_at_100Hz (acc, mag, gyro, mag, false);
#endif

#if DEVELOPMENT_ADDITIONS
    output_data.body_acc  = acc;
    output_data.body_gyro = gyro;
#endif

  }

  void cleanup_after_landing( void)
  {
    navigator.cleanup_after_landing();
  }

//! write the output data for the SIL environment
  void report_data ( output_data_t &data)
  {
    navigator.report_data ( data);

    if ( navigator.get_speed_accuracy_bad_status() == true )
      update_system_state_set(GNSS_VELOCITY_ACCURACY_BAD);
    else
      update_system_state_clear(GNSS_VELOCITY_ACCURACY_BAD);

    if ( navigator.get_magnetic_disturbance_bad_status() == true )
	update_system_state_set(MAGNETIC_DISTURBANCE_BAD);
    else
	update_system_state_clear(MAGNETIC_DISTURBANCE_BAD);
  }

private:
  navigator_t navigator; 	//!< the container for all the algorithms
  float3matrix sensor_mapping; 	//!< sensor -> airframe rotation matrix
  float pitot_offset; 		//!< pitot pressure sensor offset
  float pitot_span;   		//!< pitot pressure sensor span factor
  float QNH_offset;   		//!< static pressure sensor offset
  unsigned magnetic_induction_update_counter; //!< timer for declination+inclination update
};

#endif /* ORGANIZER_H_ */
