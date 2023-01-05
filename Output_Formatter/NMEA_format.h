/** ***********************************************************************
 * @file		NMEA_format.h
 * @brief		converters for NMEA string output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef APPLICATION_NMEA_FORMAT_H_
#define APPLICATION_NMEA_FORMAT_H_

#include "data_structures.h"

//! contains a string including it's length
class string_buffer_t
{
public:
  enum{ BUFLEN = 500};
  char string[BUFLEN];
  uint32_t length;
};

//! combine all data to be output to the NMEA port
void format_NMEA_string( const output_data_t &output_data, string_buffer_t &NMEA_buf, float declination);

#endif /* APPLICATION_NMEA_FORMAT_H_ */
