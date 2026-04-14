/***********************************************************************//**
 * @file		organizer.h
 * @brief		combine algorithms to be used by the flight sensor
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2026 Dr. Klaus Schaefer. All rights reserved.
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
#include "communicator_command.h"
#include "abstract_EEPROM_storage.h"

typedef enum
{
  FRONT_UP,
  FRONT_DOWN,
  RIGHT_UP,
  RIGHT_DOWN,
  BOTTOM_UP,
  BOTTOM_DOWN,
  INVALID
} attitude_t;

typedef struct
{
  float3vector * source;
  float3vector * destination;
  float3vector sum;
  unsigned counter;
} vector_average_organizer_t;

//! set of algorithms and data to be used by Larus flight sensor
class organizer_t
{
public:
  organizer_t( void)
    : pitot_offset(0.0f),
      pitot_span(0.0f),
      accelerometer_offset(),
      accelerometer_gain(),
      QNH_offset(0.0f),
      magnetic_induction_update_counter(0),
      fine_tune_sensor_attitude_in_progress( false),
      acceleration_calibration_in_progress(false),
      attitude_in_progress( INVALID),
      acceleration_sums{0},
      acceleration_measurement_complete(0),
      acceleration_count_down(0)
  {
    for( unsigned i=0; i<3; ++i) // poor CPP language ...
      accelerometer_gain[i]=ONE;
  }

  void tune_pressure_gauges( void)
  {
    pitot_offset= configuration (PITOT_OFFSET);
    pitot_span 	= configuration (PITOT_SPAN);
    QNH_offset	= configuration (QNH_OFFSET);
  }

//! initialization of the AHRS taking configuration data
  void initialize_before_measurement( void)
  {
    tune_pressure_gauges();

    quaternion<float> q;
    q.from_euler (configuration (SENS_TILT_ROLL),
		  configuration (SENS_TILT_PITCH),
		  configuration (SENS_TILT_YAW));
    q.get_rotation_matrix (sensor_mapping);

    float acc_calibration[6];
    bool acc_parameters_available = permanent_data_file.retrieve_data( ACCELEROMETER_CALIBRATION, 6, acc_calibration);
    if( acc_parameters_available)
      {
	for( unsigned i=0; i<3; ++i)
	  {
	    accelerometer_offset[i] = acc_calibration[2*i];
	    accelerometer_gain[i] = acc_calibration[2*i + 1];
	  }
      }

    setup_compass_calibrator_3d();
    navigator.tune();
  }

  void tune_filters( void)
  {
    navigator.tune();
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
  void initialize_after_first_measurement( const D_GNSS_coordinates_t &c, const measurement_data_t &m)
  {
    navigator.update_pressure( m.static_pressure - QNH_offset);

    if( c.sat_fix_type > SAT_FIX_NONE)
      {
	update_magnetic_induction_data( c.latitude, c.longitude);
	navigator.initialize_QFF_density_metering( c.GNSS_MSL_altitude);
	navigator.reset_altitude ( c.GNSS_MSL_altitude);
      }
    else
      {
	navigator.initialize_QFF_density_metering( navigator.get_pressure_altitude() );
	navigator.reset_altitude ( navigator.get_pressure_altitude()); // we do not have a GNSS altitude
      }

    // setup initial attitude
    float3vector acc = make_body_acceleration( m.acc);
    float3vector mag = m.mag; // coordinate rotation sensor->body done within calibration !

    if ( c.sat_fix_type & SAT_HEADING)
      {
	navigator.set_attitude ( 0.0f, 0.0f, c.relPosHeading);
      }
    else
      navigator.set_from_acc_mag( acc, mag); // initialize attitude from acceleration + compass
  }

  //! update the navigator taking the current pressure measurements
  void on_new_pressure_data( float static_pressure, float pitot_pressure)
  {
    navigator.update_pressure( static_pressure - QNH_offset);
    navigator.update_pitot ( ( pitot_pressure - pitot_offset) * pitot_span);
  }

  //! the "SLOW" update of the observed properties
  bool update_at_10Hz( const D_GNSS_coordinates_t &c, const measurement_data_t &m)
  {
    bool landing_detected_here = navigator.update_at_10Hz ();

    if ( navigator.get_speed_accuracy_bad_status() == true )
      update_system_state_set(GNSS_VELOCITY_ACCURACY_BAD);
    else
      {
	update_system_state_clear(GNSS_VELOCITY_ACCURACY_BAD);

	// this stuff can only be done with good GNSS quality
	navigator.feed_QFF_density_metering( m.static_pressure - QNH_offset, c.GNSS_MSL_altitude);

	if( ++magnetic_induction_update_counter > MAGNETIC_UPDATE_TIME_TENTH_SECS) // every 15 minutes
	  {
	    update_magnetic_induction_data( c.latitude, c.longitude);
	    magnetic_induction_update_counter=0;
	  }
      }

    if ( navigator.get_magnetic_disturbance_bad_status() == true )
	update_system_state_set(MAGNETIC_DISTURBANCE_BAD);
    else
      {
	update_system_state_clear(MAGNETIC_DISTURBANCE_BAD);
      }

    if( landing_detected_here)
      cleanup_after_landing ();

    return landing_detected_here;
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
  void update_GNSS_data( const D_GNSS_coordinates_t &coordinates)
  {

    navigator.update_GNSS_data( coordinates);
  }

  //! the "FAST" update of the observed properties
  void update_at_100_Hz( const measurement_data_t &m, uint32_t system_state, const float3vector &x_mag)
  {
    float3vector acc = make_body_acceleration( m.acc);
    float3vector gyro = sensor_mapping * m.gyro;

    bool external_magnetometer_active = system_state & EXTERNAL_MAGNETOMETER_AVAILABLE;

    navigator.update_at_100Hz (
	acc, m.mag, gyro,
	x_mag, external_magnetometer_active);

    if( acceleration_calibration_in_progress)
      {
      bool finished = manage_acceleration_calibration( m.acc);
      if( finished)
	  acceleration_calibration_in_progress = false;
      }
  }

  void cleanup_after_landing( void)
  {
    navigator.cleanup_after_landing();
  }

//! write the output data for the SIL environment
  void report_data ( state_vector_t &data)
  {
    navigator.report_data ( data);
  }

  // return true if significant changes in configuration were made
  bool on_command( communicator_command_t command, D_GNSS_coordinates_t &coordinates, measurement_data_t &observations)
  {
	  switch (command)
	    {
	    case MEASURE_CALIB_LEFT:
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_left);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      break;

	    case MEASURE_CALIB_RIGHT:
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_right);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      break;

	    case MEASURE_CALIB_LEVEL:
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_level);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      break;

	    case SET_SENSOR_ROTATION:

	      // make sure that we have all three measurements
	      if (vector_average_collection.acc_observed_left.abs () < 0.001f)
		break;
	      if (vector_average_collection.acc_observed_right.abs () < 0.001f)
		break;
	      if (vector_average_collection.acc_observed_level.abs () < 0.001f)
		break;

	      update_sensor_orientation_data ( vector_average_collection);
	      initialize_before_measurement ();
	      initialize_after_first_measurement (coordinates, observations);
	      break;

	    case FINE_TUNE_CALIB: // names "straight flight" in Larus Display Menu
#if 0 // todo patch
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_level);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      fine_tune_sensor_attitude_in_progress = true;
#else
	      start_acceleration_calibration();
#endif
	      break;

	    case TIME_CONSTANT_CHANGED:
	    case GNSS_CONFIG_CHANGED:
		tune_filters();
		return true;
	      break;

	    case TUNE_PRESSURE_GAUGES:
		tune_pressure_gauges();
		return true;
	      break;

	    case NO_COMMAND:
	      break;
	    }
	  return false;
  }

  bool manage_attitude_setup_in_progress( D_GNSS_coordinates_t &coordinates, measurement_data_t &observations)
  {
    // vector averaging in case of ground or air calibration activity *********************************
    if (vector_average_organizer.counter != 0)
	{
	  vector_average_organizer.sum += *(vector_average_organizer.source);
	  --vector_average_organizer.counter;

	  // if measurement complete now
	  if (vector_average_organizer.counter == 0)
	    {
	      float inverse_count = 1.0f / VECTOR_AVERAGE_COUNT_SETUP;
	      *(vector_average_organizer.destination) =
		  vector_average_organizer.sum * inverse_count;

	      if ( fine_tune_sensor_attitude_in_progress)
		{
		  // in this case we do not wait for another command but re-calculate immediately
		  fine_tune_sensor_orientation ( vector_average_collection);
		  initialize_before_measurement ();
		  initialize_after_first_measurement (coordinates, observations);
		  fine_tune_sensor_attitude_in_progress = false;
		  return true;
		}
	    }
	}
    return false;
  }

  bool manage_acceleration_calibration( const float3vector &acceleration);

private:
  void update_sensor_orientation_data( const vector_average_collection_t & values);
  void fine_tune_sensor_orientation( const vector_average_collection_t & values);

  float3vector calibrate_acceleration( const float3vector &acc_readings)
  {
    // acceleration sensor calibration
    float3vector acc = acc_readings;
    acc -= accelerometer_offset;
    acc = acc.element_wise_multiply( accelerometer_gain);
    return acc;
  }

  float3vector make_body_acceleration( const float3vector &acc_readings)
  {
    float3vector acc = calibrate_acceleration( acc_readings);

    // rotate sensor coordinates into airframe coordinates
    acc  = sensor_mapping * acc;

    return acc;
  }

  void start_acceleration_calibration( void)
  {
    attitude_in_progress = INVALID;
    acceleration_measurement_complete = 0;
    acceleration_count_down = 0;
    acceleration_calibration_in_progress = true;
  }

  navigator_t navigator; 	//!< the container for all the algorithms
  float3matrix sensor_mapping; 	//!< sensor -> airframe rotation matrix
  float pitot_offset; 		//!< pitot pressure sensor offset
  float pitot_span;   		//!< pitot pressure sensor span factor
  float3vector accelerometer_offset;	//!< zero G reading of the accelerometers
  float3vector accelerometer_gain;	//!< gain factor to be applied to the accelerometers
  float QNH_offset;   		//!< static pressure sensor offset
  unsigned magnetic_induction_update_counter; //!< timer for declination+inclination update
  vector_average_organizer_t vector_average_organizer = { 0 };
  vector_average_collection_t vector_average_collection = { 0 };
  bool fine_tune_sensor_attitude_in_progress;

  bool acceleration_calibration_in_progress;
  attitude_t attitude_in_progress;
  float acceleration_sums[6];
  unsigned acceleration_measurement_complete; //!< bitfield
  unsigned acceleration_count_down;
};

#endif /* ORGANIZER_H_ */
