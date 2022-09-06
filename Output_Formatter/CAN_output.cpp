/** ***********************************************************************
 * @file		CAN_output.cpp
 * @brief		format internal data and send to CAN
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "system_configuration.h"
#include "generic_CAN_driver.h"
#include "CAN_output.h"
#include "data_structures.h"

enum CAN_ID_SENSOR
{
  c_CAN_Id_EulerAngles	= 0x101,	//!< int16_t roll nick yaw / 1/1000 rad
  c_CAN_Id_Airspeed     = 0x102,	//!< uint16_t TAS, IAS / km/h
  c_CAN_Id_Vario	= 0x103,	//!< int16_t vario, vario-integrator / mm/s
  c_CAN_Id_GPS_Date_Time= 0x104,	//!< uint8_t year-2000, month, day, hour, mins, secs
  c_CAN_Id_GPS_LatLon	= 0x105,	//!< int32_t lat, lon / 10^-7 degrees
  c_CAN_Id_GPS_Alt	= 0x106,	//!< int64_t MSL altitude / mm
  c_CAN_Id_GPS_Trk_Spd	= 0x107,	//!< int16_t ground vector / 1/1000 rad, uint16_t groundspeed / km/h
  c_CAN_Id_Wind		= 0x108,	//!< int16_t 1/1000 rad , uint16_t km/h
  c_CAN_Id_Atmosphere	= 0x109,	//!< uint16_t pressure / Pa uint16_t density / g/m^3
  c_CAN_Id_GPS_Sats	= 0x10a,	//!< uin8_t No of Sats, Fix-Type NO=0 2D=1 3D=2 RTK=3
  c_CAN_Id_Acceleration = 0x10b,	//!< int16_t G-force mm / s^2
  c_CAN_Id_TurnCoord	= 0x10c,	//!< slip angle int16_t 1/1000 rad, turn rate int16_t 1/1000 rad/s
  c_CAN_Id_SystemState	= 0x10d		//!< slip angle int16_t 1/1000 rad, turn rate int16_t 1/1000 rad/s
};

void CAN_output ( const output_data_t &x)
{
  CANpacket p;

  p.id=c_CAN_Id_EulerAngles;		// 0x101
  p.dlc=6;
  p.data_sh[0] = (int16_t)(round(x.euler.r * 1000.0f)); 	// unit = 1/1000 RAD
  p.data_sh[1] = (int16_t)(round(x.euler.n * 1000.0f));
  p.data_sh[2] = (int16_t)(round(x.euler.y * 1000.0f));
  CAN_send(p, 1);

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
  p.data_l = (int64_t)(x.c.position.e[DOWN] * -1e3f);// in mm
  CAN_send(p, 1);

  p.id=c_CAN_Id_GPS_Trk_Spd;		// 0x107
  p.dlc=4;
  p.data_sh[0] = (int16_t)(round(x.c.heading_motion * 17.4533f)); // 1/1000 rad
  p.data_h[1] = (int16_t)(round(x.c.speed_motion * 3.6f));
  CAN_send(p, 1);

  p.id=c_CAN_Id_Wind;			// 0x108
  p.dlc =8;

  float wind_direction = ATAN2( - x.wind.e[EAST], - x.wind.e[NORTH]);
  if( wind_direction < 0.0f)
    wind_direction += 6.2832f;
  p.data_sh[0] = (int16_t)(round(wind_direction * 1000.0f)); // 1/1000 rad
  p.data_h[1] = (int16_t)(round(SQRT( SQR(x.wind.e[EAST])+ SQR(x.wind.e[NORTH])) * 3.6f));

  wind_direction = ATAN2( - x.wind_average.e[EAST], - x.wind_average.e[NORTH]);
  if( wind_direction < 0.0f)
    wind_direction += 6.2832f;
  p.data_sh[2] = (int16_t)(round(wind_direction * 1000.0f)); // 1/1000 rad
  p.data_h[3] = (int16_t)(round(SQRT( SQR(x.wind_average.e[EAST])+ SQR(x.wind_average.e[NORTH])) * 3.6f));

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

  p.id=c_CAN_Id_TurnCoord;				// 0x10c
  p.dlc=6;
  p.data_sh[0] = (int16_t)(round(x.slip_angle * 1000.0f));	// slipwinkel in rad aus Scheinlot
  p.data_sh[1] = (int16_t)(round(x.turn_rate  * 1000.0f)); 	// drehrate in rad/s
  p.data_sh[2] = (int16_t)(round(x.nick_angle * 1000.0f));	// nickwinkel in rad aus Scheinlot
#ifdef CAN_OUTPUT_ACTIVE
  if( CAN_send(p, 1)) // check CAN for timeout this time
      system_state |= CAN_OUTPUT_ACTIVE;

  p.id=c_CAN_Id_SystemState;				// 0x10d
  p.dlc=4;
  p.data_w[0] = system_state;
  CAN_send(p, 1);
#else
  CAN_send(p, 1);

  p.id=c_CAN_Id_SystemState;				// 0x10d
  p.dlc=4;
  p.data_w[0] = 0x00; // dummy
  CAN_send(p, 1);

  p.id=0x200; // dummy for audio heart beat
  p.dlc=1;
  CAN_send(p, 1);
#endif
}
