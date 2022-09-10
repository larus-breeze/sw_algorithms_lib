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

#include "data_structures.h"
#include "navigator.h"
#include "flight_observer.h"

//! set of algorithms and data to be used by Larus flight sensor
class organizer_t
{
public:
  organizer_t( void)
  {

  }

  void initialize_before_measurement( void)
  {
    declination = navigator.get_declination();
    pitot_offset= configuration (PITOT_OFFSET);
    pitot_span 	= configuration (PITOT_SPAN);
    QNH_offset	= configuration (QNH_OFFSET);

      {
        quaternion<float> q;
        q.from_euler (configuration (SENS_TILT_ROLL),
  		      configuration (SENS_TILT_NICK),
  		      configuration (SENS_TILT_YAW));
        q.get_rotation_matrix (sensor_mapping);
      }

  }

  void initialize_after_first_measurement( output_data_t & output_data)
  {
    navigator.update_pressure_and_altitude( output_data.m.static_pressure - QNH_offset, -output_data.c.position.e[DOWN]);
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
    navigator.update_pressure_and_altitude(output_data.m.static_pressure - QNH_offset, -output_data.c.position[DOWN]);
    navigator.update_pitot ( (output_data.m.pitot_pressure - pitot_offset) * pitot_span);
  }

  void update_GNSS( output_data_t & output_data)
  {
    navigator.update_GNSS ( output_data.c);
    navigator.feed_QFF_density_metering( output_data.m.static_pressure - QNH_offset, -output_data.c.position[DOWN]);
  }

  void set_attitude ( float roll, float nick, float present_heading)
  {
	navigator.set_attitude ( roll, nick, present_heading);
  }

  void update_IMU( output_data_t & output_data)
  {
    // rotate sensor coordinates into airframe coordinates
    acc  = sensor_mapping * output_data.m.acc;
    mag  = sensor_mapping * output_data.m.mag;
    gyro = sensor_mapping * output_data.m.gyro;

    // copy data for our logger
    output_data.body_acc  = acc;
    output_data.body_gyro = gyro;

    navigator.update_IMU (acc, mag, gyro);
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

  float
  getDeclination () const
  {
    return declination;
  }

private:
  navigator_t navigator;
  float3vector acc; //!< acceleration in airframe system
  float3vector mag; //!< normalized magnetic induction in airframe system
  float3vector gyro; //!< rotation-rates in airframe system
  float3matrix sensor_mapping; //!< sensor -> airframe rotation matrix
  float declination;  //!< magnetic declination / rad
  float pitot_offset; //!< pitot pressure sensor offset
  float pitot_span;   //!< pitot pressure sensor span factor
  float QNH_offset;   //!< static pressure sensor offset
};

#endif /* ORGANIZER_H_ */
