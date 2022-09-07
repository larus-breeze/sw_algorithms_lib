/*
 * organizer.h
 *
 *  Created on: Sep 5, 2022
 *      Author: schaefer
 */

#ifndef ORGANIZER_H_
#define ORGANIZER_H_

#include "data_structures.h"
#include "navigator.h"
#include "flight_observer.h"

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
  float3vector acc, mag, gyro;
  float3matrix sensor_mapping;
  float declination;
  float pitot_offset;
  float pitot_span;
  float QNH_offset;
};

#endif /* ORGANIZER_H_ */
