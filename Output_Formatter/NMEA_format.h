/** ***********************************************************************
 * @file		NMEA_format.h
 * @brief		converters for NMEA string output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef APPLICATION_NMEA_FORMAT_H_
#define APPLICATION_NMEA_FORMAT_H_

#include "data_structures.h"

class NMEA_buffer_t
{
public:
  char string[255];
  uint8_t length;
};

char *format_RMC (const coordinates_t &coordinates, char *p);
char *format_GGA( const coordinates_t &coordinates, char *p);
char *format_MWV ( float wind_north, float wind_east, char *p);
char *format_PTAS1 ( float vario, float avg_vario, float altitude, float TAS, char *p);
char *format_POV( float TAS, float pabs, float pitot, float TEK_vario, float voltage, char *p);
char *append_POV( float humidity, float temperature, char *p);
char *append_HCHDM( float magnetic_heading, char *p);

char * NMEA_append_tail( char *p);
bool NMEA_checksum( const char *line);

void format_NMEA_string( const output_data_t &output_data, NMEA_buffer_t &NMEA_buf, float declination);

#endif /* APPLICATION_NMEA_FORMAT_H_ */
