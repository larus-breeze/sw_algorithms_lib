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

typedef struct
{
  float3vector acc_observed_left;
  float3vector acc_observed_right;
  float3vector acc_observed_level;
} vector_average_collection_t;

//! set of algorithms and data to be used by Larus flight sensor
class organizer_t
{
public:
  organizer_t( void)
    : pitot_offset(0.0f),
      pitot_span(0.0f),
      QNH_offset(0.0f),
      magnetic_induction_update_counter(0)
  {

  }

  void update_sensor_orientation_data( const vector_average_collection_t & values)
  {
    // resolve sensor orientation using measurements
    float3vector front_down_sensor_helper = values.acc_observed_right.vector_multiply( values.acc_observed_left);
    float3vector u_right_sensor = front_down_sensor_helper.vector_multiply( values.acc_observed_level);
    u_right_sensor.normalize();

    float3vector u_down_sensor = values.acc_observed_level * -1.0f;
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

  void fine_tune_sensor_orientation( const vector_average_collection_t & values)
  {
    float3vector gravity_measurement_body = sensor_mapping * values.acc_observed_level;

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

  void update_magnetic_induction_data( double latitude, double longitude)
  {
    induction_values induction_data;
    induction_data = earth_induction_model.get_induction_data_at( latitude, longitude);
    if( induction_data.valid)
      navigator.update_magnetic_induction_data( induction_data.declination, induction_data.inclination);
  }

  void initialize_after_first_measurement( output_data_t & output_data)
  {
    navigator.update_pressure( output_data.m.static_pressure - QNH_offset);
    navigator.initialize_QFF_density_metering( -output_data.c.position[DOWN]);
    navigator.reset_altitude ();

    // setup initial attitude
    acc = sensor_mapping * output_data.m.acc;
    mag = sensor_mapping * output_data.m.mag;

    if (output_data.c.sat_fix_type & SAT_HEADING)
      {
	navigator.set_attitude ( 0.0f, 0.0f, output_data.c.relPosHeading); // todo use acc data some day ?
      }
    else
      navigator.set_from_add_mag( acc, mag); // initialize attitude from acceleration + compass
  }

  void on_new_pressure_data( output_data_t & output_data)
  {
    navigator.update_pressure(output_data.m.static_pressure - QNH_offset);
    navigator.update_pitot ( (output_data.m.pitot_pressure - pitot_offset) * pitot_span);
  }

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

  void set_attitude ( float roll, float pitch, float present_heading)
  {
    navigator.set_attitude ( roll, pitch, present_heading);
  }

  void update_GNSS_data( const coordinates_t &coordinates)
  {
    navigator.update_GNSS_data(coordinates);
  }

  void update_every_10ms( output_data_t & output_data)
  {
    // rotate sensor coordinates into airframe coordinates
    acc  = sensor_mapping * output_data.m.acc;
    mag  = sensor_mapping * output_data.m.mag;
    gyro = sensor_mapping * output_data.m.gyro;

#if DEVELOPMENT_ADDITIONS
    output_data.body_acc  = acc;
    output_data.body_gyro = gyro;
#endif

    navigator.update_at_100Hz (acc, mag, gyro);
  }

  void report_data ( output_data_t &data)
  {
    navigator.report_data ( data);
  }

  void set_density_data( float temp, float humidity)
  {
    navigator.set_density_data( temp, humidity);
  }

  void disregard_density_data()
  {
    navigator.disregard_density_data();
  }

private:
  navigator_t navigator;
  float3vector acc; //!< acceleration in airframe system
  float3vector mag; //!< normalized magnetic induction in airframe system
  float3vector gyro; //!< rotation-rates in airframe system
  float3matrix sensor_mapping; //!< sensor -> airframe rotation matrix
  float pitot_offset; //!< pitot pressure sensor offset
  float pitot_span;   //!< pitot pressure sensor span factor
  float QNH_offset;   //!< static pressure sensor offset
  unsigned magnetic_induction_update_counter;
};

#endif /* ORGANIZER_H_ */
