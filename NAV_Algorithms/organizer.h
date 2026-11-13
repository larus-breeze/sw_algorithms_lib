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
#include "flexible_file_format.h"

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

typedef enum
{
  WAITING,
  ACTIVE,
  DONE
} acceleration_calibrator_state;

typedef struct
{
  float3vector * source;
  float3vector * destination;
  float3vector sum;
  unsigned counter;
} vector_average_organizer_t;

class input_data_t
{
public:
  flexible_log_file_record_type type;
  void * data;
};

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
      vector_average_organizer({ 0 }),
      vector_average_collection({ 0 }),
      fine_tune_sensor_attitude_in_progress( false),
      acceleration_calibration_time_to_go( ACCELERATION_CALIBRATION_TIMEOUT),
      attitude_in_progress( INVALID),
      acceleration_sums{0},
      acceleration_measurement_complete(0),
      acceleration_count_down(0),
      external_mag_measurement({0}),
      counter_10Hz(0),
      state( VIRGIN)
  {
    for( unsigned i=0; i<3; ++i) // poor CPP language ...
      accelerometer_gain[i]=ONE;

    initialize_before_measurement ();
  }

  // poor man's RTTI input ;-)
  bool on_new_information( const input_data_t &input)
  {
    switch( input.type)
    {
      case IMU_DATA:
	return on_new_information( ( const imu_data_timed_t &)(input.data));
	break;
      case MAGNETOMETER_DATA:
	return on_new_information( ( const magnetometer_data_timed_t &)(input.data));
	break;
      case PRESSURE_DATA:
	return on_new_information( ( const pressure_data_timed_t &)(input.data));
	break;
      case SUPPLEMENTARY_DATA:
	return on_new_information( ( const supplementary_data_t &)(input.data));
	break;
      case D_GNSS_DATA:
	return on_new_information( ( const D_GNSS_coordinates_t &)(input.data));
	break;
      case SENSOR_STATUS:
	return on_new_information( ( const uint32_t &)(input.data));
	break;
      default:
	return false;
	break;
    }
  }

  //! write the output data for the SIL environment
    void report_data ( state_vector_t &data)
    {
      navigator.report_data ( data);
    }

// *************************************************************************************
private:

  bool on_new_information( const imu_data_timed_t &input)
  {
    update_at_100_Hz( input, external_mag_measurement);
    ++ counter_10Hz;
    if( counter_10Hz >= 9)
      {
	counter_10Hz = 0;
	update_status_flags();
	bool landing_detected_here = navigator.update_at_10Hz ();
	if( landing_detected_here)
	  cleanup_after_landing ();
      }
    return true;
  }

  bool on_new_information( const magnetometer_data_timed_t &input)
  {
    external_mag_measurement = input;
    return true;
  }

  bool on_new_information( const pressure_data_timed_t &input)
  {
    on_new_static_pressure( input.absolute_pressure);
    on_new_pitot_pressure( input.dynamic_pressure);
    return true;
  }

  bool on_new_information( const supplementary_data_t &input)
  {
    return true;
  }

  bool on_new_information( const D_GNSS_coordinates_t &input)
  {
    update_GNSS_data( input);
    if( ++magnetic_induction_update_counter > MAGNETIC_UPDATE_TIME_TENTH_SECS) // every 15 minutes
      {
	navigator.update_geo_data( input.latitude, input.longitude);
	magnetic_induction_update_counter=0;
      }
    return true;
  }

  bool on_new_information( const uint32_t &input)
  {
    return true;
  }

  void update_status_flags( void)
      {
        if ( navigator.get_speed_accuracy_bad_status() == true )
          update_system_state_set(GNSS_VELOCITY_ACCURACY_BAD);
        else
          {
    	update_system_state_clear(GNSS_VELOCITY_ACCURACY_BAD);

          }

        if ( navigator.get_magnetic_disturbance_bad_status() == true )
    	update_system_state_set(MAGNETIC_DISTURBANCE_BAD);
        else
          {
    	update_system_state_clear(MAGNETIC_DISTURBANCE_BAD);
          }
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
      set_acceleration_calibration( acc_calibration);

    setup_compass_calibrator_3d();
    navigator.tune();
  }

  void tune_filters( void)
  {
    navigator.tune();
  }

  //! attitude setup after getting the first set of acceleration an magnetic data
  void initialize_after_first_measurement( const D_GNSS_coordinates_t &c, const measurement_data_t &m);

  //! update the navigator taking the current pressure measurements
  void on_new_pressure_data( float static_pressure, float pitot_pressure)
  {
    navigator.update_pressure( static_pressure - QNH_offset);
    navigator.update_pitot ( ( pitot_pressure - pitot_offset) * pitot_span);
  }

  //! update the navigator taking the current pressure measurement
  void on_new_static_pressure( float static_pressure)
  {
    navigator.update_pressure( static_pressure - QNH_offset);
  }

  //! update the navigator taking the current pressure measurement
  void on_new_pitot_pressure( float pitot_pressure)
  {
    navigator.update_pitot ( ( pitot_pressure - pitot_offset) * pitot_span);
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
  void update_at_100_Hz( const imu_data_timed_t &m, const magnetometer_data_timed_t &x_mag);

  //! the "SLOW" update of the observed properties
  bool update_at_10Hz( const D_GNSS_coordinates_t &c);

  void cleanup_after_landing( void)
  {
    navigator.cleanup_after_landing();
  }

  // return true if significant changes in configuration were made
  bool on_command( communicator_command_t command, D_GNSS_coordinates_t &coordinates, measurement_data_t &observations);

  bool manage_attitude_setup_in_progress( D_GNSS_coordinates_t &coordinates, measurement_data_t &observations);

  acceleration_calibrator_state manage_acceleration_calibration( const float3vector &acceleration);

private:
  void update_sensor_orientation_data( const vector_average_collection_t & values);
  void fine_tune_sensor_orientation( const vector_average_collection_t & values);

  //! apply accelerometer calibration: per axis bias / g and sensitivity
  bool set_acceleration_calibration( const float calibration[6])
  {
    for( unsigned i=0; i<3; ++i)
      {
	float bias = calibration[2*i];
	float sensitivity = calibration[2*i + 1];
	if( ! ( ( bias > -MAX_ACCELERATION_BIAS) && ( bias < MAX_ACCELERATION_BIAS)
	    && ( sensitivity > MIN_ACCELERATION_SENSITIVITY) && ( sensitivity < MAX_ACCELERATION_SENSITIVITY) ) )
	  return false; // implausible (or NaN): keep present values
      }

    for( unsigned i=0; i<3; ++i)
      {
	accelerometer_offset[i] = calibration[2*i];
	accelerometer_gain[i] = calibration[2*i + 1];
      }
    return true;
  }

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

  navigator_t navigator; 	//!< the container for all the algorithms
  float3matrix sensor_mapping; 	//!< sensor -> airframe rotation matrix
  float pitot_offset; 		//!< pitot pressure sensor offset
  float pitot_span;   		//!< pitot pressure sensor span factor
  float3vector accelerometer_offset;	//!< zero G reading of the accelerometers
  float3vector accelerometer_gain;	//!< gain factor to be applied to the accelerometers
  float QNH_offset;   		//!< static pressure sensor offset

  unsigned magnetic_induction_update_counter; //!< timer for declination+inclination update
  vector_average_organizer_t vector_average_organizer;
  vector_average_collection_t vector_average_collection;
  bool fine_tune_sensor_attitude_in_progress;

  unsigned acceleration_calibration_time_to_go;
  attitude_t attitude_in_progress;
  float acceleration_sums[6];

  unsigned acceleration_measurement_complete; //!< bitfield
  unsigned acceleration_count_down;

  magnetometer_data_timed_t external_mag_measurement;
  unsigned counter_10Hz;
  enum
  {
    VIRGIN,
    INITIALIZED,
  } state;

  };
#endif /* ORGANIZER_H_ */
