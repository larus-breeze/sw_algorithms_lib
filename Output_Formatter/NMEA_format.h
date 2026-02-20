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
void format_NMEA_string_fast( const state_vector_t &output_data, string_buffer_t &NMEA_buf, bool horizon_available);
void format_NMEA_string_slow( const measurement_data_t &m, const D_GNSS_coordinates_t &c, const state_vector_t &output_data, string_buffer_t &NMEA_buf);
void format_PLARS ( float value, PLARS_TYPES type, char * &p) ;
bool NMEA_checksum( const char *line);

#endif /* APPLICATION_NMEA_FORMAT_H_ */
