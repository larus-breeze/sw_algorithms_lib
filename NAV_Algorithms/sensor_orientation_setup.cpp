/***********************************************************************//**
 * @file		sensor_orientation_setup.cpp
 * @brief		helper functions
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

#include "data_structures.h"
#include "navigator.h"
#include "sensor_orientation_setup.h"
#include "compass_calibrator_3D.h"
#include "abstract_EEPROM_storage.h"

  void setup_compass_calibrator_3d( void)
  {
    float compass_calibrator_3d_data [ 12];
    bool using_orientation_defaults = false;

    bool parameters_available = permanent_data_file.retrieve_data( MAG_SENSOR_XFER_MATRIX, 12, compass_calibrator_3d_data);

    if( not parameters_available) // use sensor orientation data as a first approximation for the sensor transfer matrix
	{
	  float roll, pitch, yaw;

	  permanent_data_file.retrieve_data( SENS_TILT_ROLL,  1, &roll);
	  permanent_data_file.retrieve_data( SENS_TILT_PITCH, 1, &pitch);
	  permanent_data_file.retrieve_data( SENS_TILT_YAW,   1, &yaw);

	  float3matrix rotation_matrix;
	  quaternion<float> q;
	  q.from_euler ( roll, pitch, yaw);
	  q.get_rotation_matrix ( rotation_matrix);

	  compass_calibrator_3d_data[0] = ZERO;
	  compass_calibrator_3d_data[1] = rotation_matrix.e[0][0];
	  compass_calibrator_3d_data[2] = rotation_matrix.e[0][1];
	  compass_calibrator_3d_data[3] = rotation_matrix.e[0][2];

	  compass_calibrator_3d_data[4] = ZERO;
	  compass_calibrator_3d_data[5] = rotation_matrix.e[1][0];
	  compass_calibrator_3d_data[6] = rotation_matrix.e[1][1];
	  compass_calibrator_3d_data[7] = rotation_matrix.e[1][2];

	  compass_calibrator_3d_data[8] = ZERO;
	  compass_calibrator_3d_data[9]  = rotation_matrix.e[2][0];
	  compass_calibrator_3d_data[10] = rotation_matrix.e[2][1];
	  compass_calibrator_3d_data[11] = rotation_matrix.e[2][2];

	  using_orientation_defaults = true;
	}
    compass_calibrator_3D.set_current_parameters( compass_calibrator_3d_data, using_orientation_defaults);

    parameters_available = permanent_data_file.retrieve_data( EXT_MAG_SENSOR_XFER_MATRIX, 12, compass_calibrator_3d_data);
    if( parameters_available)
	external_compass_calibrator_3D.set_current_parameters( compass_calibrator_3d_data);

  }




