#include "flexible_file_format.h"
#include <flexible_log_file.h>
#include "CRC16.h"

bool flexible_log_file_t::append_record ( flexible_log_file_record_type type, uint32_t *data, uint32_t data_size_words)
{
  uint32_t block_identifier;
  uint32_t crc;
  if( type > 254 || data_size_words > 254) // in this case we use two more words for identifier and length
    {
      block_identifier = 0x0000ffff; // id=len=0xff

      uint32_t long_identifier = type;
      uint32_t long_size = data_size_words + 3; // including node, extended id and size

      crc = CRC16( (uint16_t)long_identifier, CRC_SEED);
      crc = CRC16( (uint16_t)(long_identifier >> 16), crc);
      crc = CRC16( (uint16_t)(long_size), crc);
      crc = CRC16( (uint16_t)(long_size >> 16), crc);

      block_identifier |= (crc << 16);

      write_block( &block_identifier, 1);
      write_block( (uint32_t *)&long_identifier, 1);
      write_block( (uint32_t *)&long_size, 1);
      write_block( data, data_size_words);
    }
  else
    {
      block_identifier = type;
      uint32_t size = data_size_words + 1;
      block_identifier |= (size << 8);
      crc = CRC16( (uint16_t)block_identifier, CRC_SEED);
      block_identifier |= (crc << 16);
      write_block( &block_identifier, 1);
      write_block( data, data_size_words);
    }

  return true;
}

uint32_t flexible_log_file_t::verify_record_get_size( uint32_t block_identifier)
{
  uint32_t type = block_identifier & 0xff;
  uint32_t size = (block_identifier & 0xff00) >> 8;
  uint16_t info = (uint16_t)((size << 8) | type);

  if( type == 255 and size == 255) // extended record
    return 255;

  uint32_t crc_computed = CRC16( info, CRC_SEED);
  if( crc_computed != (block_identifier >> 16))
    return 0; // wrong CRC !
  else
    return size - 1; // return data size w/o node
}

uint32_t flexible_log_file_t::verify_extended_record_get_size ( uint32_t record, uint32_t extended_id, uint32_t extended_size)
{
  uint16_t crc = CRC16( (uint16_t)extended_id, CRC_SEED);
  crc = CRC16( (uint16_t)(extended_id >> 16), crc);
  crc = CRC16( (uint16_t)(extended_size), crc);
  crc = CRC16( (uint16_t)(extended_size >> 16), crc);
  if( ( (record & 0xffff) != 0xffff) || (crc != (record >> 16)))
    return 0;
  return extended_size - 3;
}
