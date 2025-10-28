/***********************************************************************//**
 * @file		NMEA_format.cpp
 * @brief		ASCII converters for NMEA string output
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

#include "NMEA_format.h"
#include "ascii_support.h"
#include "embedded_math.h"

#define ANGLE_SCALE 1e-7f
#define MPS_TO_NMPH 1.944f // 90 * 60 NM / 10000km * 3600 s/h
#define RAD_TO_DEGREE_10 572.958f
#define RAD_TO_DEGREE 57.2958f
#define METER_TO_FEET 3.2808f
#define MPS_TO_KMPH 3.6f

ROM char HEX[]="0123456789ABCDEF";

// ********* Generic stuff ************************************************
inline char hex4( uint8_t data)
{
  return HEX[data];
}

//! add end delimiter, evaluate and add checksum and add CR+LF
char * NMEA_append_tail( char *p)
 {
 	uint8_t checksum = 0;
 	assert( p[0] =='$');
 	for( p=p+1; *p && *p !='*'; ++p)
 		checksum ^= *p;
 	p[0] = '*';
 	p[1] = hex4(checksum >> 4);
 	p[2] = hex4(checksum & 0x0f);
 	p[3] = '\r';
 	p[4] = '\n';
 	p[5] = 0;
 	return p + 5;
 }

//! format an float into ASCII with 1 to 4 digits after the decimal point
char * to_ascii_x_decimals( float number, int32_t decimals, char *s)
{
  if (decimals < 1 || decimals > 4)
    return s;

  int32_t whole = (int32_t) number;
  s = format_integer(s, whole);

  *s++ = '.';
  float remaining = number - (int32_t)number;
  if (remaining < 0.0)
    remaining = remaining * -1;
  float dec = 10;
  for(int32_t i = 0; i < decimals - 1; i++)
    {
      if( remaining < 1 / dec )
        *s++ = '0';
      dec = dec * 10;
    }

  float parts = number * dec - (float)whole * dec;
  if (parts < 0.0)
    parts = parts * -1;
  s = format_integer(s, (int32_t)parts);
  return s;
}

//! format an integer into ASCII with exactly two digits after the decimal point
//! @param number value * 100
char * to_ascii_2_decimals( int32_t number, char *s)
{
  if( number < 0)
    {
      *s++='-';
      number = -number;
    }
  if( number >= 1000)
    {
      s = format_integer( s, number / 1000);
      number %= 1000;
    }
  *s++=HEX[number / 100]; // format 1 digit plus exactly 2 decimals
  *s++='.';
  number %= 100;
  *s++=HEX[number / 10];
  *s++=HEX[number % 10];
  *s=0;
  return s;
}

//! format an integer into ASCII with one decimal
//! @param number value * 10
char * to_ascii_1_decimal( int32_t number, char *s)
{
  if( number < 0)
    {
      *s++='-';
      number = -number;
    }
  if( number >= 100)
    {
      s = format_integer( s, number / 100);
      number %= 100;
    }
  *s++=HEX[number / 10]; // format exactly 1 decimal
  *s++='.';
  *s++=HEX[number % 10];
  *s=0;
  return s;
}

//! append an angle in ASCII into a NMEA string
void angle_format ( double angle, char posc, char negc, char * &p, bool force_5_digits=false)
{
  bool pos = angle > 0.0f;
  if (!pos)
    angle = -angle;

  int degree = (int) angle;
  double minutes = (angle - (double)degree) * 60.0;

  // add 5th digit if requested
  if( force_5_digits)
    {
      *p++ = (char)(degree / 100 + '0');
      degree %= 100;
    }

  // otherwise 2 digits fixed
  *p++ = (char)(degree / 10 + '0');
  *p++ = (char)(degree % 10 + '0');

  int min = (int) minutes;
  *p++ = (char)(min / 10 + '0');
  *p++ = (char)(min % 10 + '0');

  *p++ = '.';

  minutes -= min;
  minutes *= 100000;
  min = (int) round(minutes);

  p[4] = (char)(min % 10 + '0');
  min /= 10;
  p[3] = (char)(min % 10 + '0');
  min /= 10;
  p[2] = (char)(min % 10 + '0');
  min /= 10;
  p[1] = (char)(min % 10 + '0');
  min /= 10;
  p[0] = (char)(min % 10 + '0');

  p += 5;

  *p++ = ',';
  *p++ = pos ? posc : negc;
}

void format_GNSS_timestamp(const coordinates_t &coordinates, char * &p)
{
  unsigned hundredth_seconds;
  if( coordinates.nano < 0)
      hundredth_seconds=0; // just ignore any (small) negative difference
  else
      hundredth_seconds=coordinates.nano / 10000000;

  *p++ = (coordinates.hour)   / 10 + '0';
  *p++ = (coordinates.hour)   % 10 + '0';
  *p++ = (coordinates.minute) / 10 + '0';
  *p++ = (coordinates.minute) % 10 + '0';
  *p++ = (coordinates.second) / 10 + '0';
  *p++ = (coordinates.second) % 10 + '0';
  *p++ = '.';
  *p++ = (char)(hundredth_seconds / 10 +'0');
  *p++ = (char)(hundredth_seconds % 10 +'0');
  *p++ = ',';
}

ROM char GPRMC[]="$GPRMC,";

//! NMEA-format time, position, groundspeed, track data
void format_RMC (const coordinates_t &coordinates, char * &p)
{
  char * line_start = p;
  p = append_string( p, GPRMC);
  format_GNSS_timestamp( coordinates, p);

  *p++ = coordinates.sat_fix_type != 0 ? 'A' : 'V';
  *p++ = ',';

  angle_format (coordinates.latitude, 'N', 'S', p, false);
  *p++ = ',';

  angle_format (coordinates.longitude, 'E', 'W', p, true);
  *p++ = ',';

  float value = coordinates.speed_motion * MPS_TO_NMPH;

  //Clipping to realistic values for a glider. Some ASCII functions crash if given to high values. TODO: fix
  value = CLIP<float>(value, 0, (100.0f * MPS_TO_NMPH));

  unsigned knots = (unsigned)(value * 10.0f + 0.5f);
  *p++ = (char)(knots / 1000 + '0');
  knots %= 1000;
  *p++ = (char)(knots / 100 + '0');
  knots %= 100;
  *p++ = (char)(knots / 10 + '0');
  *p++ = '.';
  *p++ = (char)(knots % 10 + '0');
  *p++ = ',';

  float true_track = coordinates.heading_motion;
  if( true_track < 0.0f)
    true_track += 360.0f;
  int angle_10 = (int) round(true_track * 10.0f);

  *p++ = (char)(angle_10 / 1000 + '0');
  angle_10 %= 1000;
  *p++ = (char)(angle_10 / 100 + '0');
  angle_10 %= 100;
  *p++ = (char)(angle_10 / 10 + '0');
  *p++ = '.';
  *p++ = (char)(angle_10 % 10 + '0');

  *p++ = ',';

  *p++ = (coordinates.day) / 10 + '0';
  *p++ = (coordinates.day) % 10 + '0';
  *p++ = (coordinates.month) / 10 + '0';
  *p++ = (coordinates.month) % 10 + '0';
  *p++ = ((coordinates.year)%100) / 10 + '0';
  *p++ = ((coordinates.year)%100) % 10 + '0';

  p=append_string( p, ",,,A");
  p = NMEA_append_tail ( line_start);
}

ROM char GPGGA[]="$GPGGA,";

//! NMEA-format position report, sat number and GEO separation
void format_GGA( const coordinates_t &coordinates, char * &p)
{
  char * line_start = p;
  p = append_string( p, GPGGA);
  format_GNSS_timestamp( coordinates, p);

  angle_format (coordinates.latitude, 'N', 'S', p, false);
  *p++ = ',';

  angle_format (coordinates.longitude, 'E', 'W', p, true);
  *p++ = ',';

  *p++ = coordinates.sat_fix_type  > 0 ? '1' : '0';
  *p++ = ',';

  *p++ = (coordinates.SATS_number) / 10 + '0';
  *p++ = (coordinates.SATS_number) % 10 + '0';
  *p++ = ',';

  *p++ = '1'; // fake HDOP
  *p++ = '.';
  *p++ = '0';
  *p++ = ',';

  int32_t altitude_msl_dm = (int32_t)(coordinates.position[DOWN] * -10.0f);
  p = to_ascii_1_decimal( altitude_msl_dm, p);
  *p++ = ',';
  *p++ = 'M';
  *p++ = ',';

  int32_t geo_sep_10 = coordinates.geo_sep_dm;
  p = to_ascii_1_decimal( geo_sep_10, p);
  *p++ = ',';
  *p++ = 'M';
  *p++ = ','; // no DGPS
  *p++ = ',';
  *p=0;

  p = NMEA_append_tail ( line_start);
}

// ********* Larus-specific protocols *************************************

ROM char PLARD[]="$PLARD,";

void format_PLARD ( float density, char type, char * &p)
{
  char * line_start = p;
  p = append_string( p, PLARD);
  p = to_ascii_2_decimals( round( density * 1e5f), p); // units = g / m^3, * 100 to get 2 decimals
  *p++ = ',';
  *p++ = type;
  *p=0;

  p = NMEA_append_tail ( line_start);
}

ROM char PLARB[]="$PLARB,";

void format_PLARB ( float voltage, char * &p)
{
  char * line_start = p;
  p = append_string( p, PLARB);
  p = to_ascii_2_decimals( round( voltage * 100.0f), p);
  p = NMEA_append_tail ( line_start);
}

ROM char PLARA[]="$PLARA,";

void format_PLARA ( float roll, float pitch, float yaw, char * &p)
{
  char * line_start = p;
  p = append_string( p, PLARA);

  p = to_ascii_1_decimal( round(roll * RAD_TO_DEGREE_10), p);

  p = append_string( p, ",");
  p = to_ascii_1_decimal( round(pitch * RAD_TO_DEGREE_10), p);

  if( yaw < 0.0f)
    yaw += 6.2832f;
  p = append_string( p, ",");
  p = to_ascii_1_decimal( round(yaw * RAD_TO_DEGREE_10), p);

  p = NMEA_append_tail ( line_start);
}

ROM char PLARW[]="$PLARW,";

//! format wind reporting NMEA sequence
void format_PLARW ( float wind_north, float wind_east, char windtype, char * &p)
{
  char * line_start = p;
  p = append_string( p, PLARW);

  //Clipping to realistic values for a glider. Some ASCII functions crash if given to high values. TODO: fix
  wind_north = CLIP<float>(wind_north, -50.0, 50.0);
  wind_east = CLIP<float>(wind_east, -50.0, 50.0);

  float direction;
  // report WHERE the wind the comes from, instead of our wind speed vector, so negative sign
  if( ( SQR(wind_east) + SQR( wind_north)) < SQR( NEGLECTABLE_WIND)) // avoid circling of neglectable wind
    direction = 0.0f;
  else
    direction = ATAN2( -wind_east, -wind_north);

  // map to 0..359 degrees
  int angle = (int) round( direction * RAD_TO_DEGREE);
  if( angle < 0)
      angle += 360;
  p=format_integer( p, angle);
  *p++ = ',';

  int speed = (int) round( MPS_TO_KMPH * SQRT( SQR( wind_north) + SQR( wind_east)));
  p=format_integer( p, speed);
  *p++ = ',';

  *p++ = windtype;

  p = append_string( p, ",A"); // always report "valid" for the moment
  p = NMEA_append_tail ( line_start);
}

ROM char PLARV[]="$PLARV,";

//! TEK vario, average vario, pressure altitude and speed (TAS)
void format_PLARV ( float variometer, float avg_variometer, float pressure_altitude, float TAS, char * &p)
{
  char * line_start = p;
  p = append_string( p, PLARV);

  //Clipping to realistic values for a glider. Some ASCII functions crash if given to high values. TODO: fix
  variometer = CLIP<float>(variometer, -50.0, 50.0);
  avg_variometer = CLIP<float>(avg_variometer, -50.0, 50.0);
  TAS = CLIP<float>(TAS, 0, 100);

  p=to_ascii_2_decimals( round( variometer * 100.0f), p);
  *p++ = ',';

  p=to_ascii_2_decimals( round( avg_variometer * 100.0f), p);
  *p++ = ',';

  p=format_integer( p, (int) round( pressure_altitude));
  *p++ = ',';

  p=format_integer( p, (int) round( TAS * MPS_TO_KMPH));

  p = NMEA_append_tail ( line_start);
}

ROM char PLARS[]="$PLARS,L,";
ROM char PLARS_MC[]="MC,";
ROM char PLARS_BAL[]="BAL,";
ROM char PLARS_BUGS[]="BUGS,";
ROM char PLARS_QNH[]="QNH,";
ROM char PLARS_CIR[]="CIR,";
//! format setting NMEA for MacCready, Ballast, Bugs, QNH
void format_PLARS ( float value, PLARS_TYPES option, char * &p)
{
  char * line_start = p;
  p = append_string( p, PLARS);
  enum PLARS_TYPES type = option;

  switch (type) {
    case MC:   //MC MacCready m/s (0.0 - 9.9)
      p = append_string( p, PLARS_MC);
      p=to_ascii_1_decimal(float32_t(value * 10.0), p);
      break;
    case BAL:  //BAL Ballast (fraction of water ballast 0.000 - 1.000)
      if (value < 0.0)
	{
	  value = 0.0;
	}
      if (value > 1.0)
	{
	  value = 1.0;
	}
      p = append_string(p, PLARS_BAL);
      p = to_ascii_x_decimals(value, 3, p);
      break;
    case BUGS:  //BUGS Bugs in % (0 - 50)
      if (value < 1.0)
        {
	  value = 1.0;
        }
      if (value > 1.5)
	{
	  value = 1.5;
	}
      value = (value - 1.0f) * 100.0f + 0.5f; // Scale CAN value 1.0 ... 1.5 to 0 ... 50
      p = append_string( p, PLARS_BUGS);
      p = format_integer(p, (int32_t)value);

        break;
    case QNH:  //QNH QNH in hPa
      p = append_string( p, PLARS_QNH);
      p=to_ascii_2_decimals(float32_t(value * 100.0), p);
        break;
    case CIR: //1 == Circling or 0 == Cruising
      p = append_string( p, PLARS_CIR);
      if (value < 0.5)  // CAN definition 0 == Vario
	{
	  p = append_string( p, "1"); // NMEA Circling CIR,1
	}
      else  // 1 == SpeedToFly
	{
	  p = append_string( p, "0"); // NMEA Cruise Mode CIR,0
	}
      break;
    default:
      //NOTE: this shall not happen
      break;
  }
  p = NMEA_append_tail ( line_start);
}

//! test a line for valid NMEA checksum
bool NMEA_checksum( const char *line)
 {
 	uint8_t checksum = 0;
 	if( line[0]!='$')
 		return false;
 	const char * p;
 	for( p=line+1; *p && *p !='*'; ++p)
 		checksum ^= *p;
 	return ( (p[0] == '*') && hex4( checksum >> 4) == p[1]) && ( hex4( checksum & 0x0f) == p[2]) && (p[3] == 0);
 }

//! this procedure formats all our NMEA sequences
void format_NMEA_string_fast( const output_data_t &output_data, string_buffer_t &NMEA_buf, bool horizon_available)
{
  char *next = NMEA_buf.string + NMEA_buf.length;

  // aircraft attitude
  if( horizon_available)
    format_PLARA(output_data.euler.roll, output_data.euler.pitch, output_data.euler.yaw, next);
  else
    format_PLARA( ZERO, ZERO, output_data.euler.yaw, next);

  // report instant and average total-energy-compensated variometer, pressure altitude, TAS
  format_PLARV ( output_data.vario,
		 output_data.integrator_vario,
		 output_data.pressure_altitude,
		 output_data.TAS,
		 next);

  // instant wind
  format_PLARW (output_data.wind[NORTH], output_data.wind[EAST], 'I', next);

//  assert(   next - NMEA_buf.string < string_buffer_t::BUFLEN);
  NMEA_buf.length = next - NMEA_buf.string;
}

//! this procedure formats all our NMEA sequences
void format_NMEA_string_slow( const output_data_t &output_data, string_buffer_t &NMEA_buf)
{
  char *next = NMEA_buf.string + NMEA_buf.length;

  // NMEA-format time, position, groundspeed, track data
  format_RMC ( output_data.c, next);

  // NMEA-format position report, sat number and GEO separation
  format_GGA ( output_data.c, next);

  // battery_voltage
  format_PLARB( output_data.m.supply_voltage, next);

  // air density
  format_PLARD( output_data.air_density, 'M', next);

  // average wind
  format_PLARW (output_data.wind_average[NORTH], output_data.wind_average[EAST], 'A', next);

//  assert(   next - NMEA_buf.string < string_buffer_t::BUFLEN);
  NMEA_buf.length = next - NMEA_buf.string;
}
