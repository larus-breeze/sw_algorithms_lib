/***********************************************************************//**
 * @file		organizer.cpp
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

#include "organizer.h"
#include "NAV_tuning_parameters.h"

#define MINIMAL_ACC 0.4f

//! compute sensor orientation relative to airframe front/right/down system
//  and make changes permanent
void organizer_t::update_sensor_orientation_data( const vector_average_collection_t & values)
{
  float3vector calibrated_acc_observed_left =  calibrate_acceleration(values.acc_observed_left);
  float3vector calibrated_acc_observed_right = calibrate_acceleration(values.acc_observed_right);
  float3vector calibrated_acc_observed_level = calibrate_acceleration(values.acc_observed_level);

  // resolve sensor orientation using measurements
  float3vector front_down_sensor_helper = calibrated_acc_observed_right.vector_multiply( calibrated_acc_observed_left);
  float3vector u_right_sensor = front_down_sensor_helper.vector_multiply( calibrated_acc_observed_level);
  u_right_sensor.normalize();

  float3vector u_down_sensor = calibrated_acc_observed_level * -1.0f;
  u_down_sensor.normalize();

  float3vector u_front_sensor=u_right_sensor.vector_multiply(u_down_sensor);
  u_front_sensor.normalize();

  // calculate the new rotation matrix using our calibration data
  float3matrix new_sensor_mapping;
  new_sensor_mapping.e[0][0]=u_front_sensor[0];
  new_sensor_mapping.e[0][1]=u_front_sensor[1];
  new_sensor_mapping.e[0][2]=u_front_sensor[2];

  new_sensor_mapping.e[1][0]=u_right_sensor[0];
  new_sensor_mapping.e[1][1]=u_right_sensor[1];
  new_sensor_mapping.e[1][2]=u_right_sensor[2];

  new_sensor_mapping.e[2][0]=u_down_sensor[0];
  new_sensor_mapping.e[2][1]=u_down_sensor[1];
  new_sensor_mapping.e[2][2]=u_down_sensor[2];

  quaternion<float> q;
  q.from_rotation_matrix( new_sensor_mapping);
  eulerangle<float> euler = q;

  // make the change permanent
  write_EEPROM_value( SENS_TILT_ROLL,  euler.roll);
  write_EEPROM_value( SENS_TILT_PITCH, euler.pitch);
  write_EEPROM_value( SENS_TILT_YAW,   euler.yaw);
}

//! fine tuning of the sensor orientation angles relative to the airframes body
// taking the attitude of a straight-and-level flight period
void organizer_t::fine_tune_sensor_orientation( const vector_average_collection_t & values)
{
  float3vector calibrated_acc_observed_level = calibrate_acceleration( values.acc_observed_level);

  float3vector gravity_measurement_body = sensor_mapping * calibrated_acc_observed_level;

  // correct for "gravity pointing to minus "down" "
  gravity_measurement_body.negate();

  // evaluate observed "down" direction in the body frame
  float3vector unity_vector_down_body;
  unity_vector_down_body = gravity_measurement_body;
  unity_vector_down_body.normalize();

  // find two more unity vectors defining the corrected coordinate system
  float3vector unity_vector_front_body;
  unity_vector_front_body[FRONT] = unity_vector_down_body[DOWN];
  unity_vector_front_body[DOWN]  = unity_vector_down_body[FRONT];
  unity_vector_front_body.normalize();

  float3vector unity_vector_right_body;
  unity_vector_right_body = unity_vector_down_body.vector_multiply( unity_vector_front_body);
  unity_vector_right_body.normalize();

  // fine tune the front vector using the other ones
  unity_vector_front_body = unity_vector_right_body.vector_multiply( unity_vector_down_body);

  // calculate the rotation matrix using our calibration data
  float3matrix observed_correction_matrix;
  observed_correction_matrix.e[FRONT][0]=unity_vector_front_body[0];
  observed_correction_matrix.e[FRONT][1]=unity_vector_front_body[1];
  observed_correction_matrix.e[FRONT][2]=unity_vector_front_body[2];

  observed_correction_matrix.e[RIGHT][0]=unity_vector_right_body[0];
  observed_correction_matrix.e[RIGHT][1]=unity_vector_right_body[1];
  observed_correction_matrix.e[RIGHT][2]=unity_vector_right_body[2];

  observed_correction_matrix.e[DOWN][0]=unity_vector_down_body[0];
  observed_correction_matrix.e[DOWN][1]=unity_vector_down_body[1];
  observed_correction_matrix.e[DOWN][2]=unity_vector_down_body[2];

  quaternion<float> q_observed_correction;
  q_observed_correction.from_rotation_matrix(observed_correction_matrix);

  quaternion<float> q_present_setting;
  q_present_setting.from_euler (
	configuration (SENS_TILT_ROLL),
	configuration (SENS_TILT_PITCH),
	configuration (SENS_TILT_YAW));

  quaternion <float> q_sensor_orientation_corrected;
  q_sensor_orientation_corrected = q_observed_correction * q_present_setting;

  quaternion<float> q_new_setting;
  q_new_setting = q_observed_correction * q_present_setting;

  eulerangle<float> new_euler = q_new_setting;

  // make the change permanent
  write_EEPROM_value( SENS_TILT_ROLL,  new_euler.roll);
  write_EEPROM_value( SENS_TILT_PITCH, new_euler.pitch);
  write_EEPROM_value( SENS_TILT_YAW,   new_euler.yaw);
}

acceleration_calibrator_state organizer_t::manage_acceleration_calibration( const float3vector &acceleration)
{
  attitude_t attitude = INVALID;
  float acceleration_measurement;

  if( abs( acceleration[FRONT]) < MINIMAL_ACC)
    {
      if( abs( acceleration[RIGHT]) < MINIMAL_ACC)
	{
	  acceleration_measurement = acceleration[DOWN];

	  if( acceleration_measurement > ZERO)
	      attitude = BOTTOM_UP;
	    else
	      attitude = BOTTOM_DOWN;
	}
      else if( abs( acceleration[DOWN]) < MINIMAL_ACC)
	{
	  acceleration_measurement = acceleration[RIGHT];
	  if( acceleration_measurement > ZERO)
	      attitude = RIGHT_UP;
	    else
	      attitude = RIGHT_DOWN;
	}
    }
  else
    {
      if( abs( acceleration[RIGHT]) < MINIMAL_ACC)
	{
	  if( abs( acceleration[DOWN]) < MINIMAL_ACC)
	    {
	      acceleration_measurement = acceleration[FRONT];

	      if( acceleration_measurement > ZERO)
		  attitude = FRONT_UP;
		else
		  attitude = FRONT_DOWN;
	    }
	}
    }

  if( attitude == INVALID)
    return WAITING;

  // if this position is already done: forget ...
  if( acceleration_measurement_complete & (1 << attitude))
    return WAITING;

  if( attitude != attitude_in_progress)
    {
      attitude_in_progress = attitude;
      acceleration_sums[attitude] = ZERO;
      acceleration_count_down = ACCELERATION_CALIBRATION_COUNT + ACCELERATION_CALIBRATION_WAIT;
    }
  else
    {
      --acceleration_count_down;

      // need to wait until measurement stable
      if( acceleration_count_down > ACCELERATION_CALIBRATION_COUNT)
	return ACTIVE; // we are active: waiting

      acceleration_sums[attitude] += acceleration_measurement;
      if( acceleration_count_down == 0)
	  acceleration_measurement_complete |= (1 << attitude_in_progress); // this position is done
    }

  if( acceleration_measurement_complete == 0b111111) // all positions done ?
    {
      float calibration[6];

      calibration[0] = (acceleration_sums[FRONT_UP] + acceleration_sums[FRONT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT / GRAVITY;
      calibration[1] = (acceleration_sums[FRONT_UP] - acceleration_sums[FRONT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT / GRAVITY;

      calibration[2] = (acceleration_sums[RIGHT_UP] + acceleration_sums[RIGHT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT / GRAVITY;
      calibration[3] = (acceleration_sums[RIGHT_UP] - acceleration_sums[RIGHT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT / GRAVITY;

      calibration[4] = (acceleration_sums[BOTTOM_UP] + acceleration_sums[BOTTOM_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT / GRAVITY;
      calibration[5] = (acceleration_sums[BOTTOM_UP] - acceleration_sums[BOTTOM_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT / GRAVITY;

      permanent_data_file.store_data( ACCELEROMETER_CALIBRATION, 6, calibration);

      // start using these values
      for( unsigned i=0; i<3; ++i)
	{
	  accelerometer_offset[i] = calibration[2*i];
	  accelerometer_gain[i] =   calibration[2*i + 1];
	}

      acceleration_measurement_complete = 0;

      return DONE; // nothing else to do
    }
  else
    return ACTIVE; // work in progress
}

bool organizer_t::manage_attitude_setup_in_progress( D_GNSS_coordinates_t &coordinates, measurement_data_t &observations)
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

// return true if significant changes in configuration were made
bool organizer_t::on_command( communicator_command_t command, D_GNSS_coordinates_t &coordinates, measurement_data_t &observations)
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
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_level);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      fine_tune_sensor_attitude_in_progress = true;
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

//! the "FAST" update of the observed properties
  void organizer_t::update_at_100_Hz( const measurement_data_t &m, uint32_t system_state, const float3vector &x_mag)
  {
    float3vector acc = make_body_acceleration( m.acc);
    float3vector gyro = sensor_mapping * m.gyro;

    bool external_magnetometer_active = system_state & EXTERNAL_MAGNETOMETER_AVAILABLE;

    navigator.update_at_100Hz (
	acc, m.mag, gyro,
	x_mag, external_magnetometer_active);


    if( acceleration_calibration_time_to_go > 0)
      {
	acceleration_calibrator_state state = manage_acceleration_calibration( m.acc);
	switch( state)
	{
	  default:
	  case WAITING:
	      --acceleration_calibration_time_to_go;
	    break;
	  case ACTIVE:
	      acceleration_calibration_time_to_go = ACCELERATION_CALIBRATION_TIMEOUT;
	    break;
	  case DONE:
	      acceleration_calibration_time_to_go = 0;
	      signal_logger_event( ACCELERATION_CALIBRATION_DONE);
	    break;
	}
      }
  }

  //! the "SLOW" update of the observed properties
    bool organizer_t::update_at_10Hz( const D_GNSS_coordinates_t &c, const measurement_data_t &m)
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

    //! attitude setup after getting the first set of acceleration an magnetic data
    void organizer_t::initialize_after_first_measurement( const D_GNSS_coordinates_t &c, const measurement_data_t &m)
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


