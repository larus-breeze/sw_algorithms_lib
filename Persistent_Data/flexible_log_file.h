#ifndef PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_
#define PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_

#include "stdint.h"
#include <iostream>
#include <fstream>
using namespace std;

  enum record_type
  {
    BASIC_SENSOR_DATA = 10,
    EXTENDED_SENSOR_DATA = 11,
    GNSS_DATA = 12,
    SENSOR_STATUS = 13,
    EEPROM_FILE = 20,
    EEPROM_FILE_RECORD = 21,
    FLASH_SHA256 = 22,
    FIRMWARE_STRING = 23,
    HARDWARE_ID = 24
  };
  enum
  {
    BASIC_SENSOR_DATA_LEN = 13,
    EXTENDED_SENSOR_DATA_LEN = 16,
    GNSS_DATA_LEN = 12,
    SENSOR_STATUS_LEN = 13,
    CRC_SEED = 0xfff1
  };

  class flexible_log_file_t
{
public:

  flexible_log_file_t ( uint32_t * buf, unsigned size_words)
  : buffer( buf),
    buffer_end( buf + size_words),
    write_pointer( buf)
  {
  }

  bool open( char * file_name);
  bool close( void)
  {
    outfile.write( (const char *)buffer, (write_pointer - buffer) * sizeof( uint32_t));
    outfile.close ();
    return true;
  }

  bool append_record( record_type type, uint32_t * data, unsigned data_size=0);

  static unsigned verify_record_get_size( uint32_t input);

private:
  bool write_block( uint32_t * begin, uint32_t size_words);
  uint32_t *buffer;
  uint32_t *buffer_end;
  uint32_t *write_pointer;
  ofstream outfile;
};

#endif /* PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_ */
