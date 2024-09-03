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
#include "compass_calibrator_3D.h"

//! set of algorithms and data to be used by Larus flight sensor
class organizer_t
{
public:
  organizer_t( compass_calibrator_3D::compass_calibrator_3D_data_t *calibration_matrix_data)
   : navigator( calibration_matrix_data),
     pitot_offset(0.0f),
     pitot_span(0.0f),
     QNH_offset(0.0f),
     magnetic_induction_update_counter(0)
  {

  }

  void initialize_before_measurement( void)
  {
    pitot_offset= configuration (PITOT_OFFSET);
    pitot_span 	= configuration (PITOT_SPAN);
    QNH_offset	= configuration (QNH_OFFSET);

      {
        quaternion<float> q;
        q.from_euler (configuration (SENS_TILT_ROLL),
  		      configuration (SENS_TILT_PITCH),
  		      configuration (SENS_TILT_YAW));
        q.get_rotation_matrix (sensor_mapping);
      }

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

  void set_attitude ( float roll, float nick, float present_heading)
  {
    navigator.set_attitude ( roll, nick, present_heading);
  }
  void update_GNSS_data( const coordinates_t &coordinates)
  {
    navigator.update_GNSS_data(coordinates);
  }

  void update_every_10ms( output_data_t & output_data, float *tweaks)
  {
#if 0 // modify many parameters
    // rotate sensor coordinates into airframe coordinates
    float3vector acc_mod = output_data.m.acc;

    acc_mod[0] += tweaks[0];
    acc_mod[0] *= 1.0f + tweaks[1];
    acc_mod[1] += tweaks[2];
    acc_mod[1] *= 1.0f + tweaks[3];
    acc_mod[2] += tweaks[4];
    acc_mod[2] *= 1.0f + tweaks[5];

    float3vector mag_mod = output_data.m.mag;
        
    mag_mod[0] += tweaks[9] * output_data.m.mag[1] + tweaks[10] * output_data.m.mag[2];
    mag_mod[1] += tweaks[11] * output_data.m.mag[0] + tweaks[12] * output_data.m.mag[2];
    mag_mod[2] += tweaks[13] * output_data.m.mag[0] + tweaks[14] * output_data.m.mag[1];

    float3vector gyro_mod = output_data.m.gyro;

    gyro_mod[0] *= 1.0f + tweaks[6];
    gyro_mod[1] *= 1.0f + tweaks[7];
    gyro_mod[2] *= 1.0f + tweaks[8];

    acc  = sensor_mapping * acc_mod;
    mag  = sensor_mapping * mag_mod;
    gyro = sensor_mapping * gyro_mod;
#endif
#if 1
    float3vector acc_mod = output_data.m.acc;
    acc_mod[0] = (1.0f + tweaks[0]) * output_data.m.acc[0] + tweaks[1];
    acc_mod[1] = (1.0f + tweaks[2]) * output_data.m.acc[1] + tweaks[3];
    acc_mod[2] = (1.0f + tweaks[4]) * output_data.m.acc[2] + tweaks[5];

    acc  = sensor_mapping * acc_mod;
    mag  = sensor_mapping * output_data.m.mag;
    gyro = sensor_mapping * output_data.m.gyro;
#endif
#if 0 // magnetic sensor matrix optimization matrix
    float3vector mag_mod = output_data.m.mag;
    mag_mod[0] = 1.0f  * output_data.m.mag[0] + tweaks[1] * output_data.m.mag[1] + tweaks[2] * output_data.m.mag[2];
    mag_mod[1] = tweaks[3] * output_data.m.mag[0] + 1.0f * output_data.m.mag[1] + tweaks[5] * output_data.m.mag[2];
    mag_mod[2] = tweaks[6] * output_data.m.mag[0] + tweaks[7] * output_data.m.mag[1] + 1.0f  * output_data.m.mag[2];

    acc  = sensor_mapping * output_data.m.acc;
    mag  = sensor_mapping * mag_mod;
    gyro = sensor_mapping * output_data.m.gyro;
#endif
#if 0 // gyro matrix
    float3vector gyro_mod;

    gyro_mod[0] = (1.0f + tweaks[0]) * output_data.m.gyro[0] + tweaks[1] * output_data.m.gyro[1] + tweaks[2] * output_data.m.gyro[2];
    gyro_mod[1] = tweaks[3] * output_data.m.gyro[0] + (1.0f + tweaks[4]) * output_data.m.gyro[1] + tweaks[5] * output_data.m.gyro[2];
    gyro_mod[2] = tweaks[6] * output_data.m.gyro[0] + tweaks[7] * output_data.m.gyro[1] + (1.0f + tweaks[8]) * output_data.m.gyro[2];

    acc  = sensor_mapping * output_data.m.acc;
    mag  = sensor_mapping * output_data.m.mag;
    gyro = sensor_mapping * gyro_mod;
#endif
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
