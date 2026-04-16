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
      acceleration_calibration_time_to_go( ACCELERATION_CALIBRATION_TIMEOUT),
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
  void initialize_after_first_measurement( const D_GNSS_coordinates_t &c, const measurement_data_t &m);

  //! update the navigator taking the current pressure measurements
  void on_new_pressure_data( float static_pressure, float pitot_pressure)
  {
    navigator.update_pressure( static_pressure - QNH_offset);
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
  void update_at_100_Hz( const measurement_data_t &m, uint32_t system_state, const float3vector &x_mag);

  //! the "SLOW" update of the observed properties
  bool update_at_10Hz( const D_GNSS_coordinates_t &c, const measurement_data_t &m);

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
  bool on_command( communicator_command_t command, D_GNSS_coordinates_t &coordinates, measurement_data_t &observations);

  bool manage_attitude_setup_in_progress( D_GNSS_coordinates_t &coordinates, measurement_data_t &observations);

  acceleration_calibrator_state manage_acceleration_calibration( const float3vector &acceleration);

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

  unsigned acceleration_calibration_time_to_go;
  attitude_t attitude_in_progress;
  float acceleration_sums[6];
  unsigned acceleration_measurement_complete; //!< bitfield
  unsigned acceleration_count_down;
};

#endif /* ORGANIZER_H_ */
