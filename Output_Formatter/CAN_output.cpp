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
#include "CAN_gateway.h"

#if 1 // OLD_CAN_FORMAT

enum CAN_ID_SENSOR
{
  c_CAN_Id_EulerAngles	= 0x101,    //!< int16_t roll pitch yaw / 1/1000 rad
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
  c_CAN_Id_TurnCoord    = 0x10c,    //!< slip angle int16_t 1/1000 rad, turn rate int16_t 1/1000 rad/s, pitch angle 1/1000 rad
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
      p.data_sh[0] = (int16_t)(round(x.euler.roll * 1000.0f)); 	// unit = 1/1000 RAD
      p.data_sh[1] = (int16_t)(round(x.euler.pitch * 1000.0f));
      p.data_sh[2] = (int16_t)(round(x.euler.yaw * 1000.0f));
      CAN_send(p, 1);
    }
  else
    {
      p.id=c_CAN_Id_EulerAngles;		// 0x101
      p.dlc=6;
      p.data_sh[0] = ZERO;
      p.data_sh[1] = ZERO;
      p.data_sh[2] = (int16_t)(round(x.euler.yaw * 1000.0f));
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
  p.data_sh[0] = (int16_t)(round(x.G_load * 1000.0f));	// G-load mm/s^2
  p.data_sh[1] = (int16_t)(round(x.effective_vertical_acceleration * -1000.0f)); // mm/s^2
  p.data_sh[2] = (int16_t)(round(x.vario_uncompensated * -1000.0f)); // mm/s
  p.data_sb[6] = (int8_t)(x.flight_mode);
  CAN_send(p, 1);

  p.id=c_CID_KSB_Vdd;			// 0x112
  p.dlc=2;
  p.data_h[0] = (uint16_t)(round(x.m.supply_voltage * 10.0f)); 	// 1/10 V
  CAN_send(p, 1);

  p.id=c_CAN_Id_TurnCoord;				// 0x10c
  p.dlc=6;
  p.data_sh[0] = (int16_t)(round(x.slip_angle * 1000.0f));	// slip angle in radiant from body acceleration
  p.data_sh[1] = (int16_t)(round(x.turn_rate  * 1000.0f)); 	// turn rate rad/s
  p.data_sh[2] = (int16_t)(round(x.pitch_angle * 1000.0f));	// pitch angle in radiant from body acceleration
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
  CAN_Id_Roll_Pitch	= 0x120,    //!< float roll-angle, float pitch-angle (FRONT-RIGHT-DOWN-system)
  CAN_Id_Heading	= 0x121,    //!< float true heading, turn-rate
  CAN_Id_Airspeed	= 0x122,    //!< float TAS, float IAS / m/s
  CAN_Id_Vario		= 0x123,    //!< float vario, float vario-average / m/s
  CAN_Id_Wind		= 0x124,    //!< float wind direction (where from), float wind speed
  CAN_Id_Wind_Average	= 0x125,    //!< float average wind direction, float average wind speed m/s
  CAN_Id_Atmosphere	= 0x126,    //!< float ambient pressure / Pa, float air density / kg/m^3
  CAN_Id_Acceleration	= 0x127,    //!< float body-frame G-load m/s^2, vertical acceleration world frame
  CAN_Id_SlipPitch	= 0x128,    //!< float slip angle from body-acc, float pitch angle from body-acc
  CAN_Id_Voltage_Circle	= 0x129,    //!< float supply voltage, uint8_t circle-mode
  CAN_Id_SystemState    = 0x12a,    //!< u32 system_state, u32 git_tag dec
  CAN_Id_Sensor_Health	= 0x12b,    //!< float magnetic disturbance

  CAN_Id_GPS_Date_Time	= 0x140,    //!< uint8_t year-2000, month, day, hour, mins, secs
  CAN_Id_GPS_Lat	= 0x141,    //!< double latitude
  CAN_Id_GPS_Lon	= 0x142,    //!< double longitude
  CAN_Id_GPS_Alt	= 0x143,    //!< float MSL altitude, float geo separation
  CAN_Id_GPS_Trk_Spd	= 0x144,    //!< float ground track, float groundspeed / m/s
  CAN_Id_GPS_Sats	= 0x145,    //!< uin8_t No of Sats, (uint8_t)bool SAT FIX type

  CAN_Id_Heartbeat_Sens	= 0x520,
  CAN_Id_Heartbeat_GNSS	= 0x540,
  CAN_Id_Heartbeat_IMU	= 0x560

};

void CAN_output ( const output_data_t &x, bool horizon_activated)
{
  CANpacket p( 0x7ff, 8);
  if( horizon_activated)
    {
      p.id=CAN_Id_Roll_Pitch;
      p.dlc=8;
      p.data_f[0] = x.euler.roll;
      p.data_f[1] = x.euler.pitch;
      CAN_send(p, 1);
    }

  p.id=CAN_Id_Heading;
  p.data_f[0] = x.euler.yaw;
  p.data_f[1] = x.turn_rate;
  CAN_send(p, 1);

  p.id=CAN_Id_Airspeed;
  p.data_f[0] = x.TAS;
  p.data_f[1] = x.IAS;
  CAN_send(p, 1);

  p.id=CAN_Id_Vario;
  p.data_f[0] = x.vario;
  p.data_f[1] = x.integrator_vario;
  CAN_send(p, 1);

  p.id=CAN_Id_Wind;
  float direction = ATAN2( - x.wind[EAST], - x.wind[NORTH]);
  // map into 0 .. 2 * pi
  if( direction < 0)
    direction += 2.0f * M_PI_F;
  p.data_f[0] = direction;
  p.data_f[1] = x.wind.abs();
  CAN_send(p, 1);

  p.id=CAN_Id_Wind_Average;
  direction = ATAN2( - x.wind_average[EAST], - x.wind_average[NORTH]);
  // map into 0 .. 2 * pi
  if( direction < 0)
    direction += 2.0f * M_PI_F;
  p.data_f[0] = direction;
  p.data_f[1] = x.wind_average.abs();
  CAN_send(p, 1);

  p.id=CAN_Id_Atmosphere;
  p.data_f[0] = x.m.static_pressure;
  p.data_f[1] = x.air_density;
  CAN_send(p, 1);

  p.id=CAN_Id_Acceleration;
  p.data_f[0] = x.G_load;
  p.data_f[1] = x.effective_vertical_acceleration;
  CAN_send(p, 1);

  p.id=CAN_Id_SlipPitch;
  p.data_f[0] = x.slip_angle; // pendulum readings
  p.data_f[1] = x.pitch_angle;
  CAN_send(p, 1);

  p.id=CAN_Id_Voltage_Circle;
  p.dlc=5;
  p.data_f[0] = x.m.supply_voltage;
  p.data_b[4] = (uint8_t)(x.flight_mode);
  CAN_send(p, 1);

  p.id=CAN_Id_Sensor_Health;
  p.dlc=4;
  p.data_f[0] = x.magnetic_disturbance;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Date_Time;
  p.dlc=7;
  p.data_h[0] = x.c.year + 2000; // GNSS reports only 2000 + x
  p.data_b[2] = x.c.month;
  p.data_b[3] = x.c.day;
  p.data_b[4] = x.c.hour;
  p.data_b[5] = x.c.minute;
  p.data_b[6] = x.c.second;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Lat;
  p.dlc=8;
  // latitude handled in degrees internally
  p.data_d = x.c.latitude * M_PI / 180.0;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Lon;
  // longitude handled in degrees internally
  p.data_d = x.c.longitude * M_PI / 180.0;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Alt;
  p.dlc=8;
  p.data_f[0] = - x.c.position[DOWN];
  p.data_f[1] = x.c.geo_sep_dm * 0.1f; // dm -> m
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Trk_Spd;
  // heading handled in degrees
  p.data_f[0] = x.c.heading_motion * M_PI_F / 180.0f;
  p.data_f[1] = x.c.speed_motion;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Sats;
  p.dlc=2;
  p.data_b[0] = x.c.SATS_number;
  p.data_b[1] = x.c.sat_fix_type;

  if( CAN_send(p, 1)) // check CAN for timeout this time
    system_state |= CAN_OUTPUT_ACTIVE;
  else
    system_state &= ~CAN_OUTPUT_ACTIVE;

  p.id=CAN_Id_SystemState;
  p.dlc=8;
  p.data_w[0] = system_state;
  p.data_w[1] = GIT_TAG_DEC;
  CAN_send(p, 1);
}

#endif

enum
{
CAN_Id_Heartbeat_Sens	= 0x520,
CAN_Id_Heartbeat_GNSS	= 0x540,
CAN_Id_Heartbeat_IMU	= 0x560
};

extern uint32_t UNIQUE_ID[4]; // MPU silicon ID

void CAN_heartbeat( void)
{
  CANpacket p( CAN_Id_Heartbeat_Sens, 8);

  p.data_h[0] = 2;
  p.data_h[1] = 0;
  p.data_w[1] = UNIQUE_ID[0];
  CAN_send(p, 1);

  p.id=CAN_Id_Heartbeat_GNSS;
  p.data_h[0] = 3;
  p.data_h[1] = 0;
  p.data_w[1] = UNIQUE_ID[0];
  CAN_send(p, 1);
}

