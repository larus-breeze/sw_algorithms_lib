/*
 * old_data_structures.h
 *
 *  Created on: Nov 5, 2022
 *      Author: schaefer
 */

#ifndef OLD_DATA_STRUCTURES_H_
#define OLD_DATA_STRUCTURES_H_

#define WITH_DENSITY_DATA 1

#include "system_configuration.h"
#include "quaternion.h"
#include "GNSS.h"
#include "data_structures.h"
#include "cmath"

#pragma pack(push, 1)

typedef struct
{
  float3vector acc;
  float3vector gyro;
  float3vector mag;
  float3vector lowcost_acc;
  float3vector lowcost_gyro;
  float3vector lowcost_mag;
  float pitot_pressure;
  float static_pressure;
  float absolute_pressure;  		//this is the second ms5611 on the PCB.
  float static_sensor_temperature;  	//log temperature to monitor temperature in enclosure
  float absolute_sensor_temperature;
  float supply_voltage;  		//Measuring the supply voltage. Might be related to sensor noise.
#if WITH_DENSITY_DATA
  float outside_air_temperature; //!< OAT from external sensor if installed
  float outside_air_humidity; //!< 0.0 -> 1.0 NOT percent
#endif
} old_measurement_data_t;

typedef struct
{
  float3vector position;  	//!< NED / meters
  float3vector velocity;  	//!< NED / m/s
  float3vector acceleration;  	//!< NED / m/s^2 (from velocity delta)
  float  heading_motion;	// degrees
  float  speed_motion;		// m/s
  float3vector relPosNED;	//
  float relPosHeading;
  float speed_acc;		// speed accuracy m/s
  double latitude;		//!< degrees
  double longitude;		//!< degrees

  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;

  uint8_t minute;
  uint8_t second;
  uint8_t SATS_number;	//!< number of tracked satellites
  uint8_t sat_fix_type;	//!< bit 0: SAT FIX, bit 1: SAT HEADING availale

#if INCLUDING_NANO
  int32_t nano;		// nanoseconds from time stamp
#endif

  int16_t geo_sep_dm; 	//!< (WGS ellipsoid height - elevation MSL) in 0.1m units
  uint16_t dummy;
} old_coordinates_t;

typedef struct
{
  old_measurement_data_t m;
  old_coordinates_t c;
} old_input_data_t;

#pragma pack(pop)

inline void new_format_from_old( measurement_data_t & out_m, coordinates_t & out_c, const old_input_data_t & in)
{
  out_m.acc = in.m.acc;
  out_m.mag = in.m.mag;
  out_m.gyro = in.m.gyro;

  out_m.static_pressure = in.m.static_pressure;
  out_m.static_sensor_temperature = in.m.static_sensor_temperature;
  out_m.pitot_pressure = in.m.pitot_pressure;
  out_m.supply_voltage = in.m.supply_voltage;

  out_c.position = in.c.position;
  out_c.velocity = in.c.velocity;
  out_c.acceleration = in.c.acceleration;  	//!< NED / m/s^2 (from velocity delta)
  out_c.heading_motion = in.c.heading_motion;	// degrees
  out_c.speed_motion = in.c.speed_motion;
  out_c.relPosNED = in.c.relPosNED;	//
  out_c.relPosHeading = in.c.relPosHeading;
  out_c.speed_acc = in.c.speed_acc;
  out_c.latitude = in.c.latitude;
  out_c.longitude = in.c.longitude;

  out_c.year = in.c.year;
  out_c.month = in.c.month;
  out_c.day = in.c.day;
  out_c.hour = in.c.hour;

  out_c.minute = in.c.minute;
  out_c.second = in.c.second;
  out_c.geo_sep_dm = in.c.geo_sep_dm;

  out_c.nano = 0xefffffff; // not used in old format

  // patches
  if( isnormal(out_c.relPosHeading))
    out_c.sat_fix_type = 3; // D-GNSS available
  else
    out_c.sat_fix_type = 1;

  out_c.SATS_number  = 55; // just a joke ...

#if GNSS_VERTICAL_SPEED_INVERTED // for simulation with some old data
  out_c.velocity[DOWN] *= -1.0f; // earlier we recorded the wrong sign
#endif
}

#endif /* OLD_DATA_STRUCTURES_H_ */
