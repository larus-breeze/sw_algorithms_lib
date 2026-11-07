#include "flexible_file_format.h"
#include <flexible_log_file.h>
#include "CRC16.h"
#include "CRC32.h"

bool flexible_log_file_t::append_record ( flexible_log_file_record_type type, uint32_t *data, uint32_t data_size_words, uint32_t version)
{
  uint32_t block_identifier;
    block_identifier = type;
    uint32_t size = data_size_words + 1;
    block_identifier |= (size << 8);
    switch( version)
    {
      case 1:
	{
	  uint32_t crc;
	  crc = CRC16( (uint16_t)block_identifier, CRC_SEED);
	  block_identifier |= (crc << 16);
	  break;
	}
      case 2:
	{
	  CRC32 crc;
	  crc.update( block_identifier);
	  crc.update( data, data_size_words);
	  uint32_t folded_crc = crc.get_value_16bits();
	  block_identifier |= (folded_crc << 16);
	  break;
	}
      default:
	break;
    }
    write_block( &block_identifier, 1);
    write_block( data, data_size_words);

  return true;
}

uint32_t flexible_log_file_t::verify_record_get_size( uint32_t *record, uint32_t version)
{
  unsigned data_size_words;
  unsigned total_size_words;

  total_size_words = (record[0] >> 8) & 0xff;
  data_size_words = total_size_words - 1;

  switch( version)
  {
	case 1:
	  {
	    uint32_t crc_computed = CRC16( (uint16_t)(*record), CRC_SEED);
	    if( crc_computed != (*record >> 16))
	      return 0; // wrong CRC !
	    else
	      return data_size_words;
	    break;
	  }
	case 2:
	  {
	    CRC32 crc;
	    crc.update( record[0] & 0xffff); // crc zeroed out
	    crc.update( record + 1, data_size_words);
	    uint32_t folded_crc = crc.get_value_16bits();
	    if( (*record >> 16) != folded_crc)
	      return 0;
	    else
	      return data_size_words;
	    break;
	  }
	default:
	  break;
  }
  return 0;
}
