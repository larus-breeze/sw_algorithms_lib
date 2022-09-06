/** ***********************************************************************
 * @file		NMEA_format.cpp
 * @brief		converters for NMEA string output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "NMEA_format.h"
#include "embedded_math.h"

#define ANGLE_SCALE 1e-7
#define MPS_TO_NMPH 1.944 // 90 * 60 NM / 10000km * 3600 s/h
#define RAD_TO_DEGREE_10 572.958
#define METER_TO_FEET 3.2808

ROM char HEX[]="0123456789ABCDEF";

inline float clip( float x, float min, float max )
{
  if( x < min)
    x = min;
  else if (x > max)
    x = max;
  return x;
}

char * format_integer( uint32_t value, char *s)
{
  if( value < 10)
    {
    *s++ = value + '0';
    return s;
    }
    else
    {
      s = format_integer( value / 10, s);
      s = format_integer( value % 10, s);
    }
  return s;
}

char * integer_to_ascii_2_decimals( int32_t number, char *s)
{
  if( number < 0)
    {
      *s++ = '-';
      number = -number;
    }
  s = format_integer( number / 100, s);
  *s++='.';
  s[1]=HEX[number % 10]; // format exact 2 decimals
  number /= 10;
  s[0]=HEX[number % 10];
  s[2]=0;
  return s+2;
}

inline char *append_string( char *target, const char *source)
{
  while( *source)
      *target++ = *source++;
  *target = 0; // just to be sure :-)
  return target;
}

char *
angle_format ( double angle, char * p, char posc, char negc)
{
  bool pos = angle > 0.0f;
  if (!pos)
    angle = -angle;

  int degree = (int) angle;

  *p++ = degree / 10 + '0';
  *p++ = degree % 10 + '0';

  double minutes = (angle - (double) degree) * 60.0;
  int min = (int) minutes;
  *p++ = min / 10 + '0';
  *p++ = min % 10 + '0';

  *p++ = '.';

  minutes -= min;
  minutes *= 100000;
  min = (int) (minutes + 0.5f);

  p[4] = min % 10 + '0';
  min /= 10;
  p[3] = min % 10 + '0';
  min /= 10;
  p[2] = min % 10 + '0';
  min /= 10;
  p[1] = min % 10 + '0';
  min /= 10;
  p[0] = min % 10 + '0';

  p += 5;

  *p++ = ',';
  *p++ = pos ? posc : negc;
  return p;
}

inline float
sqr (float a)
{
  return a * a;
}
ROM char GPRMC[]="$GPRMC,";

char *format_RMC (const coordinates_t &coordinates, char *p)
{
  p = append_string( p, GPRMC);

  *p++ = (coordinates.hour) / 10 + '0';
  *p++ = (coordinates.hour) % 10 + '0';
  *p++ = (coordinates.minute) / 10 + '0';
  *p++ = (coordinates.minute) % 10 + '0';
  *p++ = (coordinates.second) / 10 + '0';
  *p++ = (coordinates.second) % 10 + '0';
  *p++ = '.';
  *p++ = '0';
  *p++ = '0';
  *p++ = ',';
  *p++ = coordinates.sat_fix_type != 0 ? 'A' : 'V';
  *p++ = ',';

  p = angle_format (coordinates.latitude, p, 'N', 'S');
  *p++ = ',';

  p = angle_format (coordinates.longitude, p, 'E', 'W');
  *p++ = ',';

#if 0
  float value = VSQRTF
      (
	  sqr( gnss.coordinates.velocity.e[NORTH]) +
	  sqr( gnss.coordinates.velocity.e[EAST])
      ) * MPS_TO_NMPH;
#else
  float value = coordinates.speed_motion * MPS_TO_NMPH;
#endif

  unsigned knots = (unsigned)(value * 10.0f + 0.5f);
  *p++ = knots / 1000 + '0';
  knots %= 1000;
  *p++ = knots / 100 + '0';
  knots %= 100;
  *p++ = knots / 10 + '0';
  *p++ = '.';
  *p++ = knots % 10 + '0';
  *p++ = ',';

#if 0
  float true_track =
      ATAN2( gnss.coordinates.velocity.e[EAST], gnss.coordinates.velocity.e[NORTH]);
#else
  float true_track = coordinates.heading_motion;
#endif
  int angle_10 = true_track * 10.0 + 0.5;
  if( angle_10 < 0)
    angle_10 += 3600;

  *p++ = angle_10 / 1000 + '0';
  angle_10 %= 1000;
  *p++ = angle_10 / 100 + '0';
  angle_10 %= 100;
  *p++ = angle_10 / 10 + '0';
  *p++ = '.';
  *p++ = angle_10 % 10 + '0';

  *p++ = ',';

  *p++ = (coordinates.day) / 10 + '0';
  *p++ = (coordinates.day) % 10 + '0';
  *p++ = (coordinates.month) / 10 + '0';
  *p++ = (coordinates.month) % 10 + '0';
  *p++ = ((coordinates.year)%100) / 10 + '0';
  *p++ = ((coordinates.year)%100) % 10 + '0';

  *p++ = ',';
  *p++ = ',';
  *p++ = ',';
  *p++ = 'A';
  *p++ = 0;

  return p;
}

ROM char GPGGA[]="$GPGGA,";

char *format_GGA( const coordinates_t &coordinates, char *p)
{
  p = append_string( p, GPGGA);

  *p++ = (coordinates.hour)   / 10 + '0';
  *p++ = (coordinates.hour)   % 10 + '0';
  *p++ = (coordinates.minute) / 10 + '0';
  *p++ = (coordinates.minute) % 10 + '0';
  *p++ = (coordinates.second) / 10 + '0';
  *p++ = (coordinates.second) % 10 + '0';
  *p++ = '.';
  *p++ = '0';
  *p++ = '0';
  *p++ = ',';

  p = angle_format (coordinates.latitude, p, 'N', 'S');
  *p++ = ',';

  p = angle_format (coordinates.longitude, p, 'E', 'W');
  *p++ = ',';

  *p++ = coordinates.sat_fix_type  >= 0 ? '1' : '0';
  *p++ = ',';

  *p++ = (coordinates.SATS_number) / 10 + '0';
  *p++ = (coordinates.SATS_number) % 10 + '0';
  *p++ = ',';

  *p++ = '0'; // fake HDOP
  *p++ = '.';
  *p++ = '0';
  *p++ = ',';

  uint32_t altitude_msl_dm = coordinates.position.e[DOWN] * -10.0;
  *p++ = altitude_msl_dm / 10000 + '0';
  altitude_msl_dm %= 10000;
  *p++ = altitude_msl_dm / 1000 + '0';
  altitude_msl_dm %= 1000;
  *p++ = altitude_msl_dm / 100 + '0';
  altitude_msl_dm %= 100;
  *p++ = altitude_msl_dm / 10 + '0';
  *p++ = '.';
  *p++ = altitude_msl_dm % 10 + '0';
  *p++ = ',';
  *p++ = 'M';
  *p++ = ',';

  int32_t geo_sep = coordinates.geo_sep_dm;
  if( geo_sep < 0)
    {
      geo_sep = -geo_sep;
      *p++ = '-';
    }
  *p++ = geo_sep / 1000 + '0';
  geo_sep %= 1000;
  *p++ = geo_sep / 100 + '0';
  geo_sep %= 100;
  *p++ = geo_sep / 10 + '0';
  geo_sep %= 10;
  *p++ = '.';
  *p++ = geo_sep + '0';
  *p++ = ',';
  *p++ = 'm';
  *p++ = ','; // no DGPS
  *p++ = ',';
  *p++ = 0;

  return p;
}

ROM char GPMWV[]="$GPMWV,";

char *format_MWV ( float wind_north, float wind_east, char *p)
{
  p = append_string( p, GPMWV);

//  wind_north = 3.0; // this setting reports 18km/h from 53 degrees
//  wind_east = 4.0;

  while (*p)
    ++p;

  float direction =
      ATAN2( -wind_east, -wind_north);

  int angle_10 = direction * RAD_TO_DEGREE_10 + 0.5;
  if( angle_10 < 0)
    angle_10 += 3600;

  *p++ = angle_10 / 1000 + '0';
  angle_10 %= 1000;
  *p++ = angle_10 / 100 + '0';
  angle_10 %= 100;
  *p++ = angle_10 / 10 + '0';
  *p++ = '.';
  *p++ = angle_10 % 10 + '0';
  *p++ = ',';
  *p++ = 'T'; // true direction
  *p++ = ',';

  float value = SQRT(sqr( wind_north) + sqr( wind_east));

  unsigned wind = value * 10.0f;
  *p++ = wind / 1000 + '0';
  wind %= 1000;
  *p++ = wind / 100 + '0';
  wind %= 100;
  *p++ = wind / 10 + '0';
  *p++ = '.';
  *p++ = wind % 10 + '0';

  *p++ = ',';
  *p++ = 'M'; // m/s
  *p++ = ',';
  *p++ = 'A'; // valid
  *p++ = 0;

  return p;
}

#if USE_PTAS
ROM char PTAS1[]="$PTAS1,";

char *format_PTAS1 ( float vario, float avg_vario, float altitude, float TAS, char *p)
{
  vario=clip(vario, -10.0f, 10.0f);
  avg_vario=clip(avg_vario, -10.0f, 10.0f);

  uint16_t i_vario = vario * MPS_TO_NMPH * 10 + 200.5;
  uint16_t i_avg_vario = avg_vario * MPS_TO_NMPH * 10 + 200.5;
  uint16_t i_altitude = altitude * METER_TO_FEET + 2000.5;
  uint16_t i_TAS = TAS * MPS_TO_NMPH + 0.5;

  p = append_string( p, PTAS1);

  *p++ = i_vario / 100 + '0';
  i_vario %= 100;
  *p++ = i_vario / 10 + '0';
  *p++ = i_vario % 10 + '0';
  *p++ = ',';

  *p++ = i_avg_vario / 100 + '0';
  i_avg_vario %= 100;
  *p++ = i_avg_vario / 10 + '0';
  *p++ = i_avg_vario % 10 + '0';
  *p++ = ',';

  *p++ = i_altitude / 10000 + '0';
  i_altitude %= 10000;
  *p++ = i_altitude / 1000 + '0';
  i_altitude %= 1000;
  *p++ = i_altitude / 100 + '0';
  i_altitude %= 100;
  *p++ = i_altitude / 10 + '0';
  *p++ = i_altitude % 10 + '0';
  *p++ = ',';

  *p++ = i_TAS / 100 + '0';
  i_TAS %= 100;
  *p++ = i_TAS / 10 + '0';
  *p++ = i_TAS % 10 + '0';

  *p++ = 0;

  return p;
}
#endif // USE_PTAS

ROM char POV[]="$POV,S,";

char *format_POV( float TAS, float pabs, float pitot, float TEK_vario, float voltage, char *p)
{
  p = append_string( p, POV);
  p = integer_to_ascii_2_decimals( (int)(TAS * 360.0f), p); // m/s -> 1/100 km/h

  p = append_string( p, ",P,");
  p = integer_to_ascii_2_decimals( (int)pabs, p); // pressure already in Pa = 100 hPa

  if( pitot < 0.0f)
    pitot = 0.0f;
  p = append_string( p, ",Q,");
  p = integer_to_ascii_2_decimals( (int)(pitot * 100.0f), p); // dynamic pressure / Pa

  p = append_string( p, ",E,");
  p = integer_to_ascii_2_decimals( (int)(voltage * 100.0f), p);

  p = append_string( p, ",V,");
  p = integer_to_ascii_2_decimals( (int)(TEK_vario * 100.0f), p);

  *p++ = 0;

  return p;
}

char *append_POV( float humidity, float temperature, char *p)
{
  p = append_string( p, ",H,");
  p = integer_to_ascii_2_decimals( (int)(humidity * 100.0f), p);

  p = append_string( p, ",T,");
  p = integer_to_ascii_2_decimals( (int)(temperature * 100.0f), p);

  *p++ = 0;

  return p;
}

ROM char HCHDM[]="$HCHDM,";

char *append_HCHDM( float magnetic_heading, char *p) // report magnetic heading
{
  int heading = (int)(magnetic_heading * 573.0f); // -> 1/10 degree
  if( heading < 0)
    heading += 3600;

  p = append_string( p, HCHDM);

  *p++ = (char)(heading / 1000 + '0');
  heading %= 1000;
  *p++ = heading / 100 + '0';
  heading %= 100;
  *p++ = heading / 10 + '0';
  heading %= 10;
  *p++ = '.';
  *p++ = heading + '0';
  *p++ = ',';
  *p++ = 'M';

  *p++ = 0;
  return p;
}

inline char hex4( uint8_t data)
{
  return HEX[data];
}

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

char * NMEA_append_tail( char *p)
 {
 	uint8_t checksum = 0;
 	if( p[0]!='$')
 		return 0;
 	for( p=p+1; *p && *p !='*'; ++p)
 		checksum ^= *p;
 	p[0] = '*';
 	p[1] = hex4(checksum >> 4);
 	p[2] = hex4(checksum & 0x0f);
 	p[3] = '\r';
 	p[4] = '\n';
 	p[5] = 0;
 	return p+5;
 }

void format_NMEA_string( const output_data_t &output_data, NMEA_buffer_t &NMEA_buf, float declination)
{
  char *next;

  format_RMC ( output_data.c, NMEA_buf.string);
  next = NMEA_append_tail (NMEA_buf.string);

  format_GGA ( output_data.c, next);  //TODO: ensure that this reports the altitude in meter above medium sea level and height above wgs84: http://aprs.gids.nl/nmea/#gga
  next = NMEA_append_tail (next);

  format_MWV (output_data.wind_average.e[NORTH], output_data.wind_average.e[EAST], next);
  next = NMEA_append_tail (next);

#if USE_PTAS

  format_PTAS1 (output_data.vario,
		    output_data.integrator_vario,
		    output_data.c.position.e[DOWN] * -1.0,   //TODO: PTAS shall report pure barometric altitude, based on static_pressure. As there can be a QNH applied to in XCSOAR.
		    output_data.TAS,
		    next);
  next = NMEA_append_tail (next);
#endif
  format_POV( output_data.TAS, output_data.m.static_pressure,
			 output_data.m.pitot_pressure, output_data.m.supply_voltage, output_data.vario, next);

  if( output_data.m.outside_air_humidity > 0.0f) // report AIR data if available
	append_POV( output_data.m.outside_air_humidity*100.0f, output_data.m.outside_air_temperature, next);

  next = NMEA_append_tail (next);

  append_HCHDM( output_data.euler.y - declination, next); // report magnetic heading

  next = NMEA_append_tail (next);

  NMEA_buf.length = next - NMEA_buf.string;
}

