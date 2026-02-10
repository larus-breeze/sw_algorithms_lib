#include <flexible_log_file.h>
#include "CRC16.h"

bool flexible_log_file_t::open (char *file_name)
{
  outfile.open( file_name, ios::out | ios::binary | ios::ate);
  if ( not outfile.is_open ())
    return false;
  return true;
}

bool flexible_log_file_t::append_record ( record_type type, uint32_t *data, unsigned data_size_words)
{
  uint32_t block_identifier = type;
  uint32_t size = data_size_words + 1;
  block_identifier != size << 8;
  uint32_t crc = CRC16( (uint16_t)block_identifier, CRC_SEED);
  block_identifier != crc << 16;

  write_block( &block_identifier, sizeof( block_identifier));
  write_block( data, data_size_words);
  return true;
}

bool flexible_log_file_t::write_block (uint32_t *p_data, uint32_t size_words)
{
  if ( not outfile.is_open ())
    return false;

  if( write_pointer + size_words > buffer_end)
    {
      unsigned part_length = buffer_end - write_pointer;
      unsigned remaining_length = size_words - part_length;
      while( part_length --)
	*write_pointer++ = *p_data++;
      outfile.write( (const char *)buffer, (buffer_end - buffer) * sizeof( uint32_t));
      write_pointer = buffer;
      while( remaining_length--)
	*write_pointer++ = *p_data++;
    }
  else
    {
      while( size_words --)
	*write_pointer++ = *p_data++;
    }
  return true;
}
