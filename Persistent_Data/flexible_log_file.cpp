#include "flexible_file_format.h"
#include <flexible_log_file.h>
#include "CRC16.h"

bool flexible_log_file_t::append_record ( flexible_log_file_record_type type, uint32_t *data, unsigned data_size_words)
{
  uint32_t block_identifier;
  uint32_t size;
  uint32_t long_size;
  uint32_t long_identifier = 0;
  if( type > 254 || data_size_words > 254) // in this case we use two more words for identifier and length
    {
      block_identifier = 0;
      long_identifier = type;
      long_size = data_size_words + 3; // including extended id and size
      type = EXTENDED_RECORD;
      size = EXTENDED_RECORD;
    }
  else
    {
      block_identifier = type;
      size = data_size_words + 1;
    }

  block_identifier |= (size << 8);
  uint32_t crc = CRC16( (uint16_t)block_identifier, CRC_SEED);
  block_identifier |= (crc << 16);

  write_block( &block_identifier, 1);
  if( long_identifier != 0)
    {
      write_block( (uint32_t *)&long_identifier, 1);
      write_block( &long_size, 1);
    }
  write_block( data, data_size_words);
  return true;
}

unsigned flexible_log_file_t::verify_record_get_size( uint32_t block_identifier)
{
  uint32_t type = block_identifier & 0xff;
  uint32_t size = (block_identifier & 0xff00) >> 8;
  uint16_t info = (uint16_t)((size << 8) | type);
  uint32_t crc_computed = CRC16( info, CRC_SEED);
  if( crc_computed != (block_identifier >> 16))
    return 0; // wrong CRC !
  else
    {
      if( type == 255 || size == 255) // extended record
	{
	  return 255;
	}
      else
	return size;
    }
}
