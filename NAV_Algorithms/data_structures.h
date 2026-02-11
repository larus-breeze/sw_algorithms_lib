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

typedef enum
{
  LEGACY_LOG_FORMAT,
  STD_LOG_FORMAT,
  EXTENDED_LOG_FORMAT
} log_file_format_t;

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

//! this structure contains all the observations from sensors and GNSS
typedef struct
{
  measurement_data_t m;
  legacy_coordinates_t c;
} legacy_observations_type;

//! this structure contains all the observations from sensors and GNSS
typedef struct
{
  measurement_data_t m;
  D_GNSS_coordinates_t c;
  uint32_t sensor_status;
} observations_type;

//! this structure contains all the observations plus external magnetometer data
typedef struct
{
  measurement_data_t m;
  D_GNSS_coordinates_t c;
  float3vector external_magnetometer_reading;
  uint32_t sensor_status;
} extended_observations_type;

//! combination of all input and output data in one structure
typedef struct
{
  extended_observations_type obs; 	//!< original recordings
  float IAS;				//!< Indicated airspeed
  float TAS;				//!< True airspeed
  float ground_speed; 			//!< ground speed
  float ground_track; 			//!< ground track / rad
  float vario;				//!< variometer output
  float vario_average;			//!< average variometer output
  float3vector wind;			//!< instant wind
  float3vector wind_average;		//!< average wind
  uint32_t flight_mode;			//!< GROUND, STRAIGHT, TRANSITION, CIRCLING

  quaternion<float> q;			//!< Attitude quaternion
  eulerangle<float> euler;		//!< Attitude euler angles

  float effective_vertical_acceleration;//!< Vertical acceleration w/o gravity
  float turn_rate;			//!< Projected turn rate
  float slip_angle;			//!< Slip angle from body-acceleration
  float pitch_angle;			//!< Pitch-angle from bode acceleration
  float G_load;				//!< G-Load absolute 3d
  float pressure_altitude;		//!< Pressure altitude
  float air_density;			//!< Air density observed
  float magnetic_disturbance;		//!< Magnitude of magnetic error vector
  float3vector nav_acceleration_gnss;	//!< Acceleration = velocity derivative
  float3vector nav_induction;		//!< Induction in world frame observed

#if DEVELOPMENT_ADDITIONS

  float vario_pressure;			//!< Pneumatic variometer uncompensated
  float speed_compensation_TAS;		//!< Speed compensation form IAS
  float vario_uncompensated_GNSS;	//!< Uncompensated variometer GNSS+INS
  float speed_compensation_GNSS;	//!< Speed compensation GNSS+INS

  float3vector nav_correction;		//!< Attitude correction NAV signal rad/s
  float3vector gyro_correction;		//!< Attitude correction BODY rad/s
  float gyro_correction_power;		//!< Filtered magnitude of gyro correction signal
  float cross_acc_correction;		//!< Attitude correction from acceleration

  float3vector body_acc;		//!< Acceleration BODY frame
  float3vector body_gyro;		//!< Gyro signal BODY frame
  float QFF;				//!< Projected pressure at sea level
  float satfix;				//!< Sat fix type NO, FIX, D-GNSS

  float3vector instant_wind;		//!< Instant wind observation

  float3vector body_induction;		//!< Body induction calibrated
  float3vector body_induction_error;	//!< Induction error BODY
  float3vector expected_nav_induction;	//!< NAV induction from NOAA model

  // experimental additions below

  float vario_wind_N;			//!< Wind used for vario compensation N
  float vario_wind_E;			//!< Wind used for vario compensation E
  float headwind;			//!< Headwind component when circling
  float crosswind;			//!< Crosswind component when circling
  float inst_wind_corrected_N;		//!< Wind corrected when circling N
  float inst_wind_corrected_E;		//!< Wind corrected when circling E

  quaternion<float> q_magnetic;		//!< Attitude from magetic corrected AHRS
  eulerangle<float> euler_magnetic;	//!< Euler angles from mag AHRS
  float HeadingDifferenceAhrsDgnss;	//!< Difference D-GNSS vs MAG heading
  float3vector nav_acceleration_mag;	//!< NAV acceleration from mag AHRS
  float3vector nav_induction_mag;	//!< NAV induction from mag AHRS

  float speed_compensation[3];		//!< Speed compensator signals experimental

#endif
} output_data_t;

#pragma pack(pop)

#endif /* DATA_STRUCTURES_H_ */
