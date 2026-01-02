/*
 * sensor_orientation_setup.h
 *
 *  Created on: Jan 2, 2026
 *      Author: schaefer
 */

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
