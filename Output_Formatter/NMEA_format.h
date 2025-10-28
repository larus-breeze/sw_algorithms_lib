/** ***********************************************************************
 * @file		NMEA_format.h
 * @brief		converters for NMEA string output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef APPLICATION_NMEA_FORMAT_H_
#define APPLICATION_NMEA_FORMAT_H_

#include "data_structures.h"
#include "embedded_math.h"

//! contains a string including it's length
class string_buffer_t
{
public:
  enum{ BUFLEN = 1024-sizeof(uint32_t)};
  char string[BUFLEN];
  uint32_t length;
};

enum PLARS_TYPES{
  MC = 0,
  BAL,
  BUGS,
  QNH,
  CIR
};

//! combine all data to be output to the NMEA port
void format_NMEA_string_fast( const output_data_t &output_data, string_buffer_t &NMEA_buf, bool horizon_available);
void format_NMEA_string_slow( const output_data_t &output_data, string_buffer_t &NMEA_buf);

char * to_ascii_x_decimals( float number, int32_t decimals, char *s);
char * to_ascii_2_decimals( int32_t number, char *s);
char * to_ascii_1_decimal( int32_t number, char *s);
inline char * to_ascii_2_decimals( float32_t number, char *s)
{
  return to_ascii_2_decimals( (int32_t)( number + 0.5f), s);
}
inline char * to_ascii_1_decimal( float32_t number, char *s)
{
  return to_ascii_1_decimal( (int32_t)( number + 0.5f), s);
}
void format_PLARV ( float variometer, float avg_variometer, float pressure_altitude, float TAS, char * &p);
void format_RMC (const coordinates_t &coordinates, char * &p);
void format_PLARS ( float value, PLARS_TYPES type, char * &p) ;
bool NMEA_checksum( const char *line);

#endif /* APPLICATION_NMEA_FORMAT_H_ */
