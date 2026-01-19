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

#ifndef   ACQUIRE_GNSS_DATA_GUARD
#define   ACQUIRE_GNSS_DATA_GUARD()
#define   RELEASE_GNSS_DATA_GUARD()
#endif

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
  CAN_Id_Identify_Sensor = 0x521,
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
  p.data_f[0] = x.obs.m.static_pressure;
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
  p.data_f[0] = x.obs.m.supply_voltage;
  p.data_b[4] = (uint8_t)(x.flight_mode);
  CAN_send(p, 1);

  p.id=CAN_Id_Sensor_Health;
  p.dlc=8;
  p.data_f[0] = x.magnetic_disturbance;
  p.data_f[1] = x.obs.c.speed_acc;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Date_Time;
  p.dlc=7;

  ACQUIRE_GNSS_DATA_GUARD();

  p.data_h[0] = x.obs.c.year + 2000; // GNSS reports only 2000 + x
  p.data_b[2] = x.obs.c.month;
  p.data_b[3] = x.obs.c.day;
  p.data_b[4] = x.obs.c.hour;
  p.data_b[5] = x.obs.c.minute;
  p.data_b[6] = x.obs.c.second;

  {
    CANpacket q( CAN_Id_GPS_Lat, 8);
    // latitude handled in degrees internally
    q.data_d = x.obs.c.latitude * M_PI / 180.0;

    CANpacket r( CAN_Id_GPS_Lon, 8);
    // longitude handled in degrees internally
    r.data_d = x.obs.c.longitude * M_PI / 180.0;

    RELEASE_GNSS_DATA_GUARD();

    CAN_send(p, 1);
    CAN_send(q, 1);
    CAN_send(r, 1);
  }

  p.id=CAN_Id_GPS_Alt;
  p.dlc=8;
  p.data_f[0] = x.obs.c.GNSS_MSL_altitude;
  p.data_f[1] = x.obs.c.geo_sep_dm * 0.1f; // dm -> m
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Trk_Spd;
  // heading handled in degrees
  p.data_f[0] = x.obs.c.heading_motion * M_PI_F / 180.0f;
  p.data_f[1] = x.obs.c.speed_motion;
  CAN_send(p, 1);

  p.id=CAN_Id_GPS_Sats;
  p.dlc=2;
  p.data_b[0] = x.obs.c.SATS_number;
  p.data_b[1] = x.obs.c.sat_fix_type;
  CAN_send(p, 1);

  p.id=CAN_Id_SystemState;
  p.dlc=8;
  p.data_w[0] = system_state;
  p.data_w[1] = GIT_TAG_DEC;
  CAN_send(p, 1);
}

extern uint32_t UNIQUE_ID[4]; // MPU silicon ID

void CAN_heartbeat( void)
{
  CANpacket p( CAN_Id_Heartbeat_Sens, 8);

  p.data_h[0] = 2;
  p.data_h[1] = 0;
  p.data_w[1] = UNIQUE_ID[0];
  CAN_send(p, 1);

  p.id = CAN_Id_Identify_Sensor;
  p.data_w[0] = 1; // todo: replace me by version of H/W
  p.data_w[1] = __REV( GIT_TAG_DEC);
  CAN_send(p, 1);

  p.id=CAN_Id_Heartbeat_GNSS;
  p.data_h[0] = 3;
  p.data_h[1] = 0;
  p.data_w[1] = UNIQUE_ID[0];
  CAN_send(p, 1);
}

