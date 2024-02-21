/***********************************************************************//**
 * @file		CAN_output.cpp
 * @brief		CAN-bus output to air display system
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

#include "system_configuration.h"
#include "generic_CAN_driver.h"
#include "CAN_output.h"
#include "data_structures.h"
#include "system_state.h"

#define DEGREE_2_RAD 1.7453292e-2f

#ifdef CAN_FORMAT_2021

enum CAN_ID_SENSOR
{
  c_CAN_Id_EulerAngles	= 0x101,    //!< int16_t roll nick yaw / 1/1000 rad
  c_CAN_Id_Airspeed     = 0x102,    //!< int16_t TAS, IAS / km/h
  c_CAN_Id_Vario        = 0x103,    //!< int16_t vario, vario-integrator / mm/s
  c_CAN_Id_GPS_Date_Time= 0x104,    //!< uint8_t year-2000, month, day, hour, mins, secs
  c_CAN_Id_GPS_LatLon   = 0x105,    //!< int32_t lat, lon / 10^-7 degrees
  c_CAN_Id_GPS_Alt      = 0x106,    //!< int32_t MSL altitude / mm, int32_t geo separation in 1/10 m
  c_CAN_Id_GPS_Trk_Spd  = 0x107,    //!< int32_t ground vector / 1/1000 rad, uint16_t groundspeed / km/h
  c_CAN_Id_Wind         = 0x108,    //!< wind int16_t 1/1000 rad , uint16_t km/h, average wind int16_t, int16_t
  c_CAN_Id_Atmosphere   = 0x109,    //!< uint32_t pressure / Pa uint32_t density / g/m^3
  c_CAN_Id_GPS_Sats     = 0x10a,    //!< uin8_t No of Sats, uint8_t Fix-Type NO=0 2D=1 3D=2 RTK=3
  c_CAN_Id_Acceleration = 0x10b,    //!< int16_t G-force mm / s^2, int16_t vertical acc mm / m^2, vario uncomp mm / s, u_int8_t circle mode
  c_CAN_Id_TurnCoord    = 0x10c,    //!< slip angle int16_t 1/1000 rad, turn rate int16_t 1/1000 rad/s, nick angle 1/1000 rad
  c_CAN_Id_SystemState  = 0x10d,    //!< u32 system_state, u32 git_tag dec
  c_CID_KSB_Vdd         = 0x112,    //!< unit16_t as voltage * 10
};

void CAN_output ( const output_data_t &x, bool horizon_activated)
{
  CANpacket p;

  if( horizon_activated)
    {
      p.id=c_CAN_Id_EulerAngles;		// 0x101
      p.dlc=6;
      p.data_sh[0] = (int16_t)(round(x.euler.r * 1000.0f)); 	// unit = 1/1000 RAD
      p.data_sh[1] = (int16_t)(round(x.euler.n * 1000.0f));
      p.data_sh[2] = (int16_t)(round(x.euler.y * 1000.0f));
      CAN_send(p, 1);
    }
  else
    {
      p.id=c_CAN_Id_EulerAngles;		// 0x101
      p.dlc=6;
      p.data_sh[0] = ZERO;
      p.data_sh[1] = ZERO;
      p.data_sh[2] = (int16_t)(round(x.euler.y * 1000.0f));
      CAN_send(p, 1);
    }

  p.id=c_CAN_Id_Airspeed;		// 0x102
  p.dlc=4;
  p.data_sh[0] = (int16_t)(round(x.TAS * 3.6f)); 		// m/s -> km/h
  p.data_sh[1] = (int16_t)(round(x.IAS * 3.6f)); 		// m/s -> km/h
  CAN_send(p, 1);

  p.id=c_CAN_Id_Vario;			// 0x103
  p.dlc=4;
  p.data_sh[0] = (int16_t)(round(x.vario * 1000.0f)); 		// mm/s
  p.data_sh[1] = (int16_t)(round(x.integrator_vario * 1000.0f)); 	// mm/s
  CAN_send(p, 1);

  p.id=c_CAN_Id_GPS_Date_Time;		// 0x104
  p.dlc=6;
  p.data_b[0] = x.c.year;
  p.data_b[1] = x.c.month;
  p.data_b[2] = x.c.day;
  p.data_b[3] = x.c.hour;
  p.data_b[4] = x.c.minute;
  p.data_b[5] = x.c.second;
  CAN_send(p, 1);

  p.id=c_CAN_Id_GPS_LatLon;		// 0x105
  p.dlc=8;
  p.data_sw[0] = (int32_t)(x.c.latitude * (double)1e7);
  p.data_sw[1] = (int32_t)(x.c.longitude * (double)1e7);  //
  CAN_send(p, 1);

  p.id=c_CAN_Id_GPS_Alt;		// 0x106
  p.dlc=8;
  p.data_sw[0] = (int32_t)(x.c.position[DOWN] * -1e3f);// in mm
  p.data_sw[1] = x.c.geo_sep_dm; // geo separation in 1/10 m
  CAN_send(p, 1);

  p.id=c_CAN_Id_GPS_Trk_Spd;		// 0x107
  p.dlc=4;
  p.data_sh[0] = (int16_t)(round(x.c.heading_motion * 17.4533f)); // 1/1000 rad
  p.data_h[1] = (int16_t)(round(x.c.speed_motion * 3.6f));
  CAN_send(p, 1);

  p.id=c_CAN_Id_Wind;			// 0x108
  p.dlc =8;

  float wind_direction = ATAN2( - x.wind[EAST], - x.wind[NORTH]);
  if( wind_direction < 0.0f)
    wind_direction += 6.2832f;
  p.data_sh[0] = (int16_t)(round(wind_direction * 1000.0f)); // 1/1000 rad
  p.data_h[1] = (int16_t)(round(SQRT( SQR(x.wind[EAST])+ SQR(x.wind[NORTH])) * 3.6f));

  wind_direction = ATAN2( - x.wind_average[EAST], - x.wind_average[NORTH]);
  if( wind_direction < 0.0f)
    wind_direction += 6.2832f;
  p.data_sh[2] = (int16_t)(round(wind_direction * 1000.0f)); // 1/1000 rad
  p.data_h[3] = (int16_t)(round(SQRT( SQR(x.wind_average[EAST])+ SQR(x.wind_average[NORTH])) * 3.6f));

  CAN_send(p, 1);

  p.id=c_CAN_Id_Atmosphere;		// 0x109
  p.dlc=8;
  p.data_w[0] = (uint32_t)(x.m.static_pressure);
  p.data_w[1] = (uint32_t)(x.air_density * 1000.0f);
  CAN_send(p, 1);

  p.id=c_CAN_Id_GPS_Sats;		// 0x10a
  p.dlc=2;
  p.data_b[0] = x.c.SATS_number;
  p.data_b[1] = x.c.sat_fix_type;
  CAN_send(p, 1);

  p.id=c_CAN_Id_Acceleration;		// 0x10b
  p.dlc=7;
  p.data_sh[0] = (int16_t)(round(x.G_load * 1000.0f));	// G-Belastung mm/s^2 nach oben pos.
  p.data_sh[1] = (int16_t)(round(x.effective_vertical_acceleration * -1000.0f)); // mm/s^2
  p.data_sh[2] = (int16_t)(round(x.vario_uncompensated * -1000.0f)); // mm/s
  p.data_sb[6] = (int8_t)(x.circle_mode);
  CAN_send(p, 1);

  p.id=c_CID_KSB_Vdd;			// 0x112
  p.dlc=2;
  p.data_h[0] = (uint16_t)(round(x.m.supply_voltage * 10.0f)); 	// 1/10 V
  CAN_send(p, 1);

  p.id=c_CAN_Id_TurnCoord;				// 0x10c
  p.dlc=6;
  p.data_sh[0] = (int16_t)(round(x.slip_angle * 1000.0f));	// slip angle in radiant from body acceleration
  p.data_sh[1] = (int16_t)(round(x.turn_rate  * 1000.0f)); 	// turn rate rad/s
  p.data_sh[2] = (int16_t)(round(x.nick_angle * 1000.0f));	// nick angle in radiant from body acceleration
  if( CAN_send(p, 1)) // check CAN for timeout this time
    system_state |= CAN_OUTPUT_ACTIVE;
  else
    system_state &= ~CAN_OUTPUT_ACTIVE;

#ifndef GIT_TAG_DEC
#define GIT_TAG_DEC 0xffffffff
#endif
  
  p.id=c_CAN_Id_SystemState;				// 0x10d
  p.dlc=8;
  p.data_w[0] = system_state;
  p.data_w[1] = GIT_TAG_DEC;
  CAN_send(p, 1);
}

#else

enum CAN_ID_SENSOR
{
  // all values in SI-STD- (metric) units, angles in radians
  // format IEEE float32 little-endian
  CAN_Id_Roll_Nick	= 0x400,    //!< float roll-angle, float nick-angle (FRONT-RIGHT-DOWN-system)
  CAN_Id_Heading	= 0x401,    //!< float true heading, [OPTIONAL float magnetic declination]
  CAN_Id_Airspeed	= 0x402,    //!< float TAS, float IAS / m/s
  CAN_Id_Vario		= 0x403,    //!< float vario, float vario-average / m/s

  CAN_Id_GPS_Date_Time	= 0x404,    //!< uint8_t year-2000, month, day, hour, mins, secs
  CAN_Id_GPS_LatLon	= 0x405,    //!< float latitude, float longitude / degrees(!)
  CAN_Id_GPS_Alt	= 0x406,    //!< float MSL altitude,float geo separation
  CAN_Id_GPS_Trk_Spd	= 0x407,    //!< float ground track, float groundspeed / m/s
  CAN_Id_GPS_Sats	= 0x408,    //!< uin8_t No of Sats, (uint8_t)bool SAT FIX valid, (uint8_t)bool SAT HEADING available

  CAN_Id_Wind		= 0x409,    //!< float wind direction (where from), float wind speed
  CAN_Id_Wind_Average	= 0x40a,    //!< float average wind direction, float average wind speed m/s
  CAN_Id_Atmosphere	= 0x40b,    //!< float ambient pressure / Pa, float air density / kg/m^3
  CAN_Id_Acceleration	= 0x40c,    //!< float body-frame G-load m/s^2, body acceleration angle roll-axis
  CAN_Id_TurnRate	= 0x40d,    //!< float turn rate to the right, (uint8_t) (enum  { STRAIGHT_FLIGHT, TRANSITION, CIRCLING} )
  CAN_Id_SystemState	= 0x40e,    //!< u32 system_state, u32 git_tag_dec
  CAN_Id_Voltage	= 0x40f,    //!< float supply voltage
};

void CAN_output ( const output_data_t &x)
{
  CANpacket p;

#if  HORIZON_DATA_SECRET == 0
  p.id=CAN_Id_Roll_Nick;
  p.dlc=8;
  p.data_f[0] = x.euler.r;
  p.data_f[1] = x.euler.n;
  CAN_send(p, 1);
#endif
  p.id=CAN_Id_Heading;
  p.dlc=4;
  p.data_f[0] = x.euler.y;
  CAN_send(p, 1);

  p.id=CAN_Id_Airspeed;
  p.dlc=8;
  p.data_f[0] = x.TAS;
  p.data_f[1] = x.IAS;
  CAN_send(p, 1);

  p.id=CAN_Id_Vario;
  p.dlc=8;
  p.data_f[0] = x.vario;
  p.data_f[1] = x.integrator_vario;
  CAN_send(p, 1);

  p.id=CAN_Id_Wind;
  p.dlc=8;
  p.data_f[0] = ATAN2( - x.wind[EAST], - x.wind[NORTH]);
  p.data_f[1] = x.wind.abs();
  CAN_send(p, 1);

  p.id=CAN_Id_Wind_Average;
  p.dlc=8;
  p.data_f[0] = ATAN2( - x.wind_average[EAST], - x.wind_average[NORTH]);
  p.data_f[1] = x.wind.abs();
  CAN_send(p, 1);

  p.id=CAN_Id_Atmosphere;
  p.dlc=8;
  p.data_f[0] = x.m.static_pressure;
  p.data_f[1] = x.air_density;
  CAN_send(p, 1);

  p.id=CAN_Id_Acceleration;
  p.dlc=7;
  p.data_f[0] = x.G_load;
  p.data_f[1] = x.slip_angle;
  CAN_send(p, 1);

  p.id=CAN_Id_TurnRate;
  p.dlc=5;
  p.data_f[0] = x.turn_rate;
  p.data_b[4] = (uint8_t)(x.circle_mode);


  p.id=CAN_Id_GPS_Date_Time;
  p.dlc=6;
  p.data_b[0] = x.c.year;
  p.data_b[1] = x.c.month;
  p.data_b[2] = x.c.day;
  p.data_b[3] = x.c.hour;
  p.data_b[4] = x.c.minute;
  p.data_b[5] = x.c.second;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_LatLon;
  p.dlc=8;
  p.data_f[0] = (float)(x.c.latitude); // -> 4m of accuracy
  p.data_f[1] = (float)(x.c.longitude);
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Alt;		// 0x106
  p.dlc=8;
  p.data_f[0] = x.c.position[DOWN] * -1.0f;
  p.data_f[1] = x.c.geo_sep_dm * 0.1f; // dm -> m
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Trk_Spd;
  p.dlc=8;
  p.data_f[0] = x.c.heading_motion * DEGREE_2_RAD;
  p.data_f[1] = x.c.speed_motion;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Sats;
  p.dlc=2;
  p.data_b[0] = x.c.SATS_number;
  p.data_b[1] = x.c.sat_fix_type;
  CAN_send(p, 1);

  p.id=CAN_Id_Voltage;
  p.dlc=4;
  p.data_f[0] = x.m.supply_voltage;

  if( CAN_send(p, 1)) // check CAN for timeout this time
    system_state |= CAN_OUTPUT_ACTIVE;
  else
    system_state &= ~CAN_OUTPUT_ACTIVE;

#ifndef GIT_TAG_DEC
#define GIT_TAG_DEC 0xffffffff
#endif

  p.id=CAN_Id_SystemState;
  p.dlc=8;
  p.data_w[0] = system_state;
  p.data_w[1] = GIT_TAG_DEC;
  CAN_send(p, 1);
}


#endif
