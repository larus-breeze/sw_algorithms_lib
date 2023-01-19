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

//! contains all calibrated data from the sensors
typedef struct
{
  float3vector acc;   //XSENSE MTi1 IMU
  float3vector gyro;  //XSENSE MTi1 IMU
  float3vector mag;   //XSENSE MTi1 IMU
#if WITH_LOWCOST_SENSORS
  float3vector lowcost_acc;
  float3vector lowcost_gyro;
  float3vector lowcost_mag;
#endif
  float pitot_pressure;
  float static_pressure;
#if WITH_LOWCOST_SENSORS
  float absolute_pressure;  //this is the second ms5611 on the PCB.
#endif
  float static_sensor_temperature;  //log temperature to monitor temperature in enclosure
#if WITH_LOWCOST_SENSORS
  float absolute_sensor_temperature;
#endif
  float supply_voltage;  //Measuring the supply voltage. Might be related to sensor noise.
#if WITH_DENSITY_DATA
  float outside_air_temperature; //!< OAT from external sensor if installed
  float outside_air_humidity; //!< 0.0 -> 1.0 NOT percent
#endif
} measurement_data_t;

//! this structure contains all the observations from sensors and GNSS
typedef struct
{
  measurement_data_t m;
  coordinates_t c;
} observations_type;

//! combination of all input and output data in one structure
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
  float speed_compensation_GNSS;
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
  float satfix;
  float headwind;
  float crosswind;
  float inst_wind_N;
  float inst_wind_E;
} output_data_t;

#pragma pack(pop)

enum availability_bits
{
	GNSS_AVAILABLE 		= 1,
	D_GNSS_AVAILABLE 	= 2,

	MTI_SENSOR_AVAILABE 	= 0x10,
	FXOS_SENSOR_AVAILABLE 	= 0x20,
	L3GD20_SENSOR_AVAILABLE = 0x40,
	MS5611_STATIC_AVAILABLE = 0x80,
	MS5611_PITOT_AVAILABLE  = 0x100,
	PITOT_SENSOR_AVAILABLE 	= 0x200,
	AIR_SENSOR_AVAILABLE 	= 0x400,

	USB_OUTPUT_ACTIVE	= 0x1000,
	BLUEZ_OUTPUT_ACTIVE	= 0x2000,
	CAN_OUTPUT_ACTIVE	= 0x4000,
	USART_2_OUTPUT_ACTIVE	= 0x8000
};

extern uint32_t system_state; //!< bits collected from availability_bits

#endif /* DATA_STRUCTURES_H_ */
