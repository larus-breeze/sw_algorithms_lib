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

bool organizer_t::manage_acceleration_calibration( const float3vector &acceleration)
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
    return false;

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
	return false;

      acceleration_sums[attitude] += acceleration_measurement;
      if( acceleration_count_down == 0)
	{
	  acceleration_measurement_complete |= (1 << attitude_in_progress);
	}
    }

  if( acceleration_measurement_complete == 0b111111)
    {
      float calibration[6];

      calibration[0] = (acceleration_sums[FRONT_UP] + acceleration_sums[FRONT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT;
      calibration[1] = (acceleration_sums[FRONT_UP] - acceleration_sums[FRONT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT;

      calibration[2] = (acceleration_sums[RIGHT_UP] + acceleration_sums[RIGHT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT;
      calibration[3] = (acceleration_sums[RIGHT_UP] - acceleration_sums[RIGHT_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT;

      calibration[4] = (acceleration_sums[BOTTOM_UP] + acceleration_sums[BOTTOM_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT;
      calibration[5] = (acceleration_sums[BOTTOM_UP] - acceleration_sums[BOTTOM_DOWN]) * 0.5f / ACCELERATION_CALIBRATION_COUNT;

      permanent_data_file.store_data( ACCELEROMETER_CALIBRATION, 6, calibration);

      for( unsigned i=0; i<3; ++i)
	{
	  accelerometer_offset[i] = calibration[2*i];
	  accelerometer_gain[i] =   calibration[2*i + 1];
	}

      acceleration_measurement_complete = 0;

      return true;
    }
  else
    return false;
}
