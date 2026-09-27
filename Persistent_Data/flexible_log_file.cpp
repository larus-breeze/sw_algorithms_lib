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

      crc = extended_header_crc( long_identifier, long_size, false);

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
      crc = CRC16_word( (uint16_t)block_identifier, CRC_SEED);
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

  if( size == 0) // size includes the identifier itself, 0 is impossible
    return 0;

  uint32_t crc_stored = block_identifier >> 16;
  if( crc_stored == CRC16_word( info, CRC_SEED))
    return size - 1; // return data size w/o node
  if( ACCEPT_LEGACY_CRC && ( crc_stored == CRC16( info, CRC_SEED))) // file written before the CRC fix
    return size - 1;
  return 0; // wrong CRC !
}

uint32_t flexible_log_file_t::verify_extended_record_get_size ( uint32_t record, uint32_t extended_id, uint32_t extended_size)
{
  if( (record & 0xffff) != 0xffff)
    return 0;
  uint32_t crc_stored = record >> 16;
  if( ( crc_stored != extended_header_crc( extended_id, extended_size, false))
      && not ( ACCEPT_LEGACY_CRC && ( crc_stored == extended_header_crc( extended_id, extended_size, true))))
    return 0;
  if( extended_size < 3) // size includes node, extended id and size itself
    return 0;
  return extended_size - 3;
}

uint16_t flexible_log_file_t::extended_header_crc( uint32_t extended_id, uint32_t extended_size, bool legacy)
{
  // legacy: CRC16() fed with 16 bit values, covering only their low bytes
  uint16_t (*crc16)( uint16_t, uint16_t) = legacy ? CRC16 : CRC16_word;
  uint16_t crc = crc16( (uint16_t)extended_id, CRC_SEED);
  crc = crc16( (uint16_t)(extended_id >> 16), crc);
  crc = crc16( (uint16_t)(extended_size), crc);
  return crc16( (uint16_t)(extended_size >> 16), crc);
}
