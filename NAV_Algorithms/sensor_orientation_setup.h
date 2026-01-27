/***********************************************************************//**
 * @file		sensor_orientation_setup.h
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

#ifndef NAV_ALGORITHMS_SENSOR_ORIENTATION_SETUP_H_
#define NAV_ALGORITHMS_SENSOR_ORIENTATION_SETUP_H_

#include "float3vector.h"

typedef struct
{
  float3vector acc_observed_left;
  float3vector acc_observed_right;
  float3vector acc_observed_level;
} vector_average_collection_t;

void update_sensor_orientation_data( const vector_average_collection_t & values);
void fine_tune_sensor_orientation( const vector_average_collection_t & values, const float3matrix &sensor_mapping);
void setup_compass_calibrator_3d( void);

#endif /* NAV_ALGORITHMS_SENSOR_ORIENTATION_SETUP_H_ */
