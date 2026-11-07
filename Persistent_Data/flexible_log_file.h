#ifndef PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_
#define PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_

#include "stdint.h"
#include "flexible_file_format.h" // record_type definition

extern bool write_block( uint32_t * begin, uint32_t size_words);

class flexible_log_file_t
{
  friend class flexible_log_file_implementation_t;

public:
  enum{ FLEXIBLE_LOG_FILE_FORMAT_VERSION = 2};

  flexible_log_file_t ( uint32_t * buf, unsigned size_words)
  : buffer( buf),
    buffer_end( buf + size_words),
    write_pointer( buf)
  {
  }

  virtual ~flexible_log_file_t ( void)
  {
  }

  bool append_record ( flexible_log_file_record_type type, uint32_t *data, uint32_t data_size_words, uint32_t _version = 1);
  virtual bool open( char * file_name) = 0;
  virtual bool close( void) = 0;

  static uint32_t verify_record_get_size( uint32_t * record, uint32_t _version = 1);

private:
  enum {CRC_SEED = 0xfff1};

  uint32_t *buffer;
  uint32_t *buffer_end;
  uint32_t *write_pointer;
};

#endif /* PERSISTENT_DATA_FLEXIBLE_LOG_FILE_H_ */
