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
#define RAD_TO_DEGREE 57.2958f
#define METER_TO_FEET 3.2808f
#define MPS_TO_KMPH 3.6f

ROM char HEX[]="0123456789ABCDEF";

//! add end delimiter, evaluate and add checksum and add CR+LF
char * NMEA_append_tail( char *p)
 {
 	uint8_t checksum = 0;
 	assert( p[0] =='$');
 	for( p=p+1; *p && *p !='*'; ++p)
 		checksum ^= *p;
 	p[0] = '*';
 	p[1] = HEX[checksum >> 4];
 	p[2] = HEX[checksum & 0x0f];
 	p[3] = '\r';
 	p[4] = '\n';
 	p[5] = 0;
 	return p + 5;
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
void format_RMC (const coordinates_t &coordinates, float groundspeed, char * &p)
{
  char * line_start = p;
  append_string( p, GPRMC);
  format_GNSS_timestamp( coordinates, p);

  *p++ = coordinates.sat_fix_type != 0 ? 'A' : 'V';
  *p++ = ',';

  angle_format (coordinates.latitude, 'N', 'S', p, false);
  *p++ = ',';

  angle_format (coordinates.longitude, 'E', 'W', p, true);
  *p++ = ',';

  to_ascii_n_decimals( groundspeed * MPS_TO_NMPH, 1, p);
  *p++ = ',';

  float true_track = coordinates.heading_motion;
  if( true_track < 0.0f)
    true_track += 360.0f;

  to_ascii_n_decimals( true_track, 1, p);
  *p++ = ',';

  *p++ = (coordinates.day) / 10 + '0';
  *p++ = (coordinates.day) % 10 + '0';
  *p++ = (coordinates.month) / 10 + '0';
  *p++ = (coordinates.month) % 10 + '0';
  *p++ = ((coordinates.year)%100) / 10 + '0';
  *p++ = ((coordinates.year)%100) % 10 + '0';

  append_string( p, ",,,A");
  p = NMEA_append_tail ( line_start);
}

ROM char GPGGA[]="$GPGGA,";

//! NMEA-format position report, sat number and GEO separation
void format_GGA( const coordinates_t &coordinates, char * &p)
{
  char * line_start = p;
  append_string( p, GPGGA);
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

  to_ascii_n_decimals( coordinates.GNSS_MSL_altitude, 1, p);
  *p++ = ',';
  *p++ = 'M';
  *p++ = ',';

  to_ascii_n_decimals( coordinates.geo_sep_dm * 0.1f, 1, p);
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
  append_string( p, PLARD);
  to_ascii_n_decimals( density * 1e3f, 2, p); // units = g / m^3
  *p++ = ',';
  *p++ = type;
  *p=0;

  p = NMEA_append_tail ( line_start);
}

ROM char PLARB[]="$PLARB,";

void format_PLARB ( float voltage, char * &p)
{
  char * line_start = p;
  append_string( p, PLARB);
  to_ascii_n_decimals( voltage, 2, p);
  p = NMEA_append_tail ( line_start);
}

ROM char PLARA[]="$PLARA,";

void format_PLARA ( float roll, float pitch, float yaw, char * &p)
{
  char * line_start = p;
  append_string( p, PLARA);

  to_ascii_n_decimals( roll * RAD_TO_DEGREE, 1, p);

  append_string( p, ",");
  to_ascii_n_decimals( pitch * RAD_TO_DEGREE, 1, p);

  if( yaw < 0.0f)
    yaw += 6.2832f;
  append_string( p, ",");
  to_ascii_n_decimals( yaw * RAD_TO_DEGREE, 1, p);

  p = NMEA_append_tail ( line_start);
}

ROM char PLARW[]="$PLARW,";

//! format wind reporting NMEA sequence
void format_PLARW ( float wind_north, float wind_east, char windtype, char * &p)
{
  char * line_start = p;
  append_string( p, PLARW);

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
  format_integer( p, angle);
  *p++ = ',';

  int speed = (int) round( MPS_TO_KMPH * SQRT( SQR( wind_north) + SQR( wind_east)));
  format_integer( p, speed);
  *p++ = ',';

  *p++ = windtype;

  append_string( p, ",A"); // always report "valid" for the moment
  p = NMEA_append_tail ( line_start);
}

ROM char PLARV[]="$PLARV,";

//! TEK vario, average vario, pressure altitude and speed (TAS)
void format_PLARV ( float variometer, float avg_variometer, float pressure_altitude, float TAS, char * &p)
{
  char * line_start = p;
  append_string( p, PLARV);

  //Clipping to realistic values for a glider. Some ASCII functions crash if given to high values. TODO: fix
  variometer = CLIP<float>(variometer, -50.0, 50.0);
  avg_variometer = CLIP<float>(avg_variometer, -50.0, 50.0);
  TAS = CLIP<float>(TAS, 0, 100);

  to_ascii_n_decimals( variometer, 2, p);
  *p++ = ',';

  to_ascii_n_decimals( avg_variometer, 2, p);
  *p++ = ',';

  format_integer( p, (int) round( pressure_altitude));
  *p++ = ',';

  format_integer( p, (int) round( TAS * MPS_TO_KMPH));

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
  append_string( p, PLARS);
  enum PLARS_TYPES type = option;

  switch (type) {
    case MC:   //MC MacCready m/s (0.0 - 9.9)
      append_string( p, PLARS_MC);
      to_ascii_n_decimals( value, 1, p);
      break;
    case BAL:  //BAL Ballast (fraction of water ballast 0.000 - 1.000)
      CLIP( value, 0.0f, 1.0f);
      append_string( p, PLARS_BAL);
      to_ascii_n_decimals( value, 3, p);
      break;
    case BUGS:  //BUGS Bugs in % (0 - 50)
      CLIP( value, 1.0f, 1.5f);
      value = (value - 1.0f) * 100.0f + 0.5f; // Scale CAN
      append_string( p, PLARS_BUGS);
      to_ascii_n_decimals( value, 2, p);
        break;
    case QNH:  //QNH QNH in hPa
      append_string( p, PLARS_QNH);
      to_ascii_n_decimals( value, 2, p);
        break;
    case CIR: //1 == Circling or 0 == Cruising
      append_string( p, PLARS_CIR);
      if (value < 0.5)  // CAN definition 0 == Vario
	{
	  append_string( p, "1"); // NMEA Circling CIR,1
	}
      else  // 1 == SpeedToFly
	{
	  append_string( p, "0"); // NMEA Cruise Mode CIR,0
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
 	return ( (p[0] == '*') && HEX[ checksum >> 4] == p[1]) && ( HEX[ checksum & 0x0f] == p[2]) && (p[3] == 0);
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
		 output_data.vario_average,
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
  format_RMC ( output_data.obs.c, output_data.groundspeed, next);

  // NMEA-format position report, sat number and GEO separation
  format_GGA ( output_data.obs.c, next);

  // battery_voltage
  format_PLARB( output_data.obs.m.supply_voltage, next);

  // air density
  format_PLARD( output_data.air_density, 'M', next);

  // average wind
  format_PLARW (output_data.wind_average[NORTH], output_data.wind_average[EAST], 'A', next);

//  assert(   next - NMEA_buf.string < string_buffer_t::BUFLEN);
  NMEA_buf.length = next - NMEA_buf.string;
}
