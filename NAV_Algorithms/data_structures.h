/***********************************************************************//**
 * @file		data_stuctures.h
 * @brief		definitions for measured and derived data
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

#ifndef DATA_STRUCTURES_H_
#define DATA_STRUCTURES_H_

#include "system_configuration.h"
#include "AHRS.h"
#include "GNSS.h"

#pragma pack(push, 1)

//! contains all input data from the sensors
typedef struct
{
  float3vector acc;   //XSENSE MTi1 IMU
  float3vector gyro;  //XSENSE MTi1 IMU
  float3vector mag;   //XSENSE MTi1 IMU
  float pitot_pressure;
  float static_pressure;
  float static_sensor_temperature;  //log temperature to monitor temperature in enclosure
  float supply_voltage;  //Measuring the supply voltage. Might be related to sensor noise.
} measurement_data_t;

//! this structure contains all the observations from all sensors and the GNSS-receiver
typedef struct
{
  float3vector acc;
  float3vector gyro;
  float3vector mag;
  float temperature;
} extra_sensor_data_t;

//! this structure contains all the observations from sensors and GNSS
typedef struct
{
  measurement_data_t m;
#if WITH_DENSITY_DUMMY
  float dummy1;
  float dummy2;
#endif
  coordinates_t c;
#if WITH_EXTERNAL_IMU
  extra_sensor_data_t extra;
#endif
#if RUN_MICROPHONE
  float sound_intensity;
#endif
} observations_type;

//! combination of all input and output data in one structure
typedef struct
{
  measurement_data_t m;
  coordinates_t c;
#if WITH_EXTERNAL_IMU
  extra_sensor_data_t extra;
#endif
#if RUN_MICROPHONE
  float sound_intensity;
#endif
  float IAS;
  float TAS;
  float vario_uncompensated;
  float vario;
  float vario_pressure;
  float speed_compensation_TAS;
  float speed_compensation_GNSS;
  float integrator_vario;
  float3vector wind;
  float3vector wind_average;
  uint32_t flight_mode;
  quaternion<float> q;
  eulerangle<float> euler;
  float effective_vertical_acceleration;
  float turn_rate;
  float slip_angle;
  float pitch_angle;
  float G_load;
  float pressure_altitude;
  float air_density;
  float magnetic_disturbance;
  float3vector nav_acceleration_gnss;
  float3vector nav_induction_gnss;

#if DEVELOPMENT_ADDITIONS
  float3vector nav_correction;
  float3vector gyro_correction;

  float3vector nav_acceleration_mag;
  float3vector nav_induction_mag;
  eulerangle<float> euler_magnetic;
  quaternion<float> q_magnetic;

  float3vector body_acc;
  float3vector body_gyro;
  float HeadingDifferenceAhrsDgnss;
  float QFF;
  float satfix;
  float inst_wind_N;
  float inst_wind_E;
  float headwind;
  float crosswind;
  float inst_wind_corrected_N;
  float inst_wind_corrected_E;
  float speed_compensation[3];
  float cross_acc_correction;
  float vario_wind_N;
  float vario_wind_E;
  float3vector body_induction;
  float3vector body_induction_error;
  float gyro_correction_power;
#endif

} output_data_t;

#pragma pack(pop)

#endif /* DATA_STRUCTURES_H_ */
