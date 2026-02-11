#ifndef PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_
#define PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_

#include "stdint.h"
#include <iostream>
#include <fstream>
#include "flexible_file_format.h" // record_type definition
using namespace std;

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

  bool append_record( flexible_log_file_record_type, uint32_t * data, unsigned data_size=0);

  static unsigned verify_record_get_size( uint32_t input);

private:
  enum {CRC_SEED = 0xfff1};

  bool write_block( uint32_t * begin, uint32_t size_words);
  uint32_t *buffer;
  uint32_t *buffer_end;
  uint32_t *write_pointer;
  ofstream outfile;
};

#endif /* PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_ */
