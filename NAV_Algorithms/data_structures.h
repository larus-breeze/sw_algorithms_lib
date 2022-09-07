/*
 * data_structures.h
 *
 *  Created on: Dec 29, 2020
 *      Author: schaefer
 */

#ifndef DATA_STRUCTURES_H_
#define DATA_STRUCTURES_H_

#include "system_configuration.h"
#include "float3vector.h"
#include "euler.h"
#include "quaternion.h"
#include "GNSS.h"

#pragma pack(push, 1)

typedef struct
{
  float3vector acc;   //XSENSE MTi1 IMU
  float3vector gyro;  //XSENSE MTi1 IMU
  float3vector mag;   //XSENSE MTi1 IMU
  float3vector lowcost_acc;
  float3vector lowcost_gyro;
  float3vector lowcost_mag;
  float pitot_pressure;
  float static_pressure;
  float absolute_pressure;  //this is the second ms5611 on the PCB.
  float static_sensor_temperature;  //log temperature to monitor temperature in enclosure
  float absolute_sensor_temperature;
  float supply_voltage;  //Measuring the supply voltage. Might be related to sensor noise.
#if WITH_DENSITY_DATA
  float outside_air_temperature; //!< OAT from external sensor if installed
  float outside_air_humidity; //!< 0.0 -> 1.0 NOT percent
#endif
} measurement_data_t;

typedef struct
{
  measurement_data_t m;
  coordinates_t c;
} input_data_t;

typedef struct
{
  measurement_data_t m;
  coordinates_t c;
  float IAS;
  float TAS;
  float vario_uncompensated;
  float vario;
  float vario_pressure;
  float speed_compensation_TAS;
  float speed_compensation_INS;
  float integrator_vario;
  float3vector wind;
  float3vector wind_average;
  uint32_t circle_mode;
  float3vector nav_acceleration_gnss;
  float3vector nav_induction_gnss;
  float3vector nav_correction;
  float3vector gyro_correction;
  quaternion<float> q;
  eulerangle<float> euler;
  float effective_vertical_acceleration;
  float turn_rate;
  float slip_angle;
  float nick_angle;
  float G_load;

#if PARALLEL_MAGNETIC_AHRS
  float3vector nav_acceleration_mag;
  float3vector nav_induction_mag;
  eulerangle<float> euler_magnetic;
  quaternion<float> q_magnetic;
#endif

  float3vector body_acc;
  float3vector body_gyro;
  float HeadingDifferenceAhrsDgnss;
  float QFF;
  float air_density;
} output_data_t;

#pragma pack(pop)

#endif /* DATA_STRUCTURES_H_ */
